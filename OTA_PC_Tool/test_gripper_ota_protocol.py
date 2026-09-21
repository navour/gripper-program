import struct
import unittest
import zlib
from pathlib import Path

import gripper_ota_protocol as ota
import gripper_stlink_recovery as recovery


class FakeBootSerial:
    def __init__(self):
        self.rx = bytearray()
        self.state = 0
        self.session = 0
        self.expected_sequence = 0
        self.next_offset = 0
        self.image_size = 0
        self.image_crc32 = 0
        self.image = bytearray()
        self.manifest_role = None

    def write(self, raw):
        header = raw[: ota.HEADER_SIZE]
        sof, protocol, command, session, sequence, _flags, offset, length = struct.unpack(
            ota.HEADER_FORMAT, header
        )
        self.assert_frame(raw, sof, protocol, length)
        payload = raw[ota.HEADER_SIZE : ota.HEADER_SIZE + length]
        status = 0
        response_payload = b""

        if command == ota.CMD_HELLO:
            response_payload = (
                struct.pack(
                    "<IIIIIHHBBBBIII",
                    ota.PRODUCT_ID,
                    ota.LAYOUT_VERSION,
                    ota.APP_BASE,
                    ota.APP_SIZE,
                    ota.META_BASE,
                    0x0100,
                    ota.MAX_PAYLOAD,
                    ota.PROTOCOL_VERSION,
                    0,
                    self.state,
                    0,
                    0x07,
                    self.session,
                    self.next_offset,
                )
            )
        elif command == ota.CMD_BEGIN:
            self.manifest_role = payload[12]
            self.session = session
            self.expected_sequence = (sequence + 1) & 0xFFFF
            self.next_offset = 0
            self.image_size = struct.unpack_from("<I", payload, 32)[0]
            self.image_crc32 = struct.unpack_from("<I", payload, 36)[0]
            self.image = bytearray()
            self.state = 1
        elif command == ota.CMD_DATA:
            self.assertEqual(session, self.session)
            self.assertEqual(sequence, self.expected_sequence)
            self.assertEqual(offset, self.next_offset)
            self.image.extend(payload)
            self.next_offset += len(payload)
            self.expected_sequence = (self.expected_sequence + 1) & 0xFFFF
        elif command == ota.CMD_END:
            self.assertEqual(self.next_offset, self.image_size)
            self.assertEqual(zlib.crc32(self.image) & 0xFFFFFFFF, self.image_crc32)
            self.state = 2
            self.next_offset = 0
            offset = self.image_size
        elif command == ota.CMD_QUERY:
            response_payload = struct.pack(
                "<BBHIIIII",
                self.state,
                0,
                0x0100,
                self.session,
                self.next_offset,
                self.image_size,
                self.image_crc32,
                1,
            )
        elif command == ota.CMD_REBOOT:
            offset = self.image_size
        elif command == ota.CMD_ABORT:
            self.state = 3
            self.next_offset = 0
        else:
            status = 0x0014

        self.rx.extend(
            ota.encode_frame(
                command | ota.RESPONSE_MASK,
                session,
                sequence,
                offset=offset if command in (ota.CMD_END, ota.CMD_REBOOT) else self.next_offset,
                payload=response_payload,
                flags=status,
            )
        )
        return len(raw)

    def assert_frame(self, raw, sof, protocol, length):
        if sof != ota.SOF or protocol != ota.PROTOCOL_VERSION:
            raise AssertionError("bad request header")
        if len(raw) != ota.HEADER_SIZE + length + 2:
            raise AssertionError("bad request length")
        received = struct.unpack_from("<H", raw, len(raw) - 2)[0]
        if received != ota.crc16_modbus(raw[:-2]):
            raise AssertionError("bad request CRC")

    def assertEqual(self, left, right):
        if left != right:
            raise AssertionError(f"{left!r} != {right!r}")

    def read(self, length):
        data = bytes(self.rx[:length])
        del self.rx[:length]
        return data

    def flush(self):
        pass

    def reset_input_buffer(self):
        self.rx.clear()


class ProtocolTests(unittest.TestCase):
    def test_real_gripper2_bin_full_update(self):
        data = Path(__file__).with_name("gripper2_app.bin").read_bytes()
        image = ota.validate_application(data)
        fake = FakeBootSerial()
        _session, query = ota.SerialBootClient(fake).update(image, 2, (1, 15, 0))
        self.assertEqual(fake.manifest_role, 2)
        self.assertEqual(query.state, 2)
        self.assertEqual(query.image_size, len(data))
        self.assertEqual(query.image_crc32, zlib.crc32(data) & 0xFFFFFFFF)
        self.assertEqual(bytes(fake.image), data)

    def test_crc16_known_vector(self):
        self.assertEqual(ota.crc16_modbus(b"123456789"), 0x4B37)

    def test_manifest_and_full_update(self):
        data = bytearray([0xA5] * 517)
        struct.pack_into("<II", data, 0, 0x20007FE0, ota.APP_BASE + 0x41)
        data[32 : 32 + len(ota.APP_IDENTITY)] = ota.APP_IDENTITY
        image = ota.validate_application(bytes(data))
        self.assertEqual(image.crc32, zlib.crc32(data) & 0xFFFFFFFF)

        fake = FakeBootSerial()
        client = ota.SerialBootClient(fake)
        session, query = client.update(image, 1, (1, 15, 0))
        self.assertNotEqual(session, 0)
        self.assertEqual(query.state, 2)
        self.assertEqual(query.image_size, len(data))
        self.assertEqual(query.image_crc32, image.crc32)
        self.assertEqual(bytes(fake.image), bytes(data))

    def test_old_link_address_is_rejected(self):
        data = bytearray([0xFF] * 32)
        struct.pack_into("<II", data, 0, 0x20001000, 0x08000101)
        with self.assertRaises(ota.OtaError):
            ota.validate_application(bytes(data))

    def test_stlink_metadata_matches_bootloader_layout(self):
        data = bytearray([0xA5] * 517)
        struct.pack_into("<II", data, 0, 0x20007FE0, ota.APP_BASE + 0x41)
        data[32 : 32 + len(ota.APP_IDENTITY)] = ota.APP_IDENTITY
        image = ota.validate_application(bytes(data))
        page = recovery.build_valid_metadata(image, 1, (1, 15, 0))

        self.assertEqual(len(page), recovery.META_SIZE)
        self.assertEqual(struct.unpack_from("<I", page, 0)[0], recovery.META_MAGIC)
        self.assertEqual(page[6], recovery.META_STATE_VALID)
        self.assertEqual(page[7], 1)
        self.assertEqual(struct.unpack_from("<I", page, 24)[0], image.size)
        self.assertEqual(struct.unpack_from("<I", page, 28)[0], image.crc32)
        self.assertEqual(
            struct.unpack_from("<I", page, recovery.META_CRC_LENGTH)[0],
            zlib.crc32(page[: recovery.META_CRC_LENGTH]) & 0xFFFFFFFF,
        )
        self.assertEqual(
            struct.unpack_from("<I", page, 60)[0], recovery.META_COMMIT_MARKER
        )
        self.assertEqual(
            page[recovery.META_RECORD_SIZE :],
            bytes([0xFF]) * (recovery.META_SIZE - recovery.META_RECORD_SIZE),
        )

    def test_stlink_hex_never_reaches_calibration_page(self):
        data = bytearray([0xA5] * 517)
        struct.pack_into("<II", data, 0, 0x20007FE0, ota.APP_BASE + 0x41)
        data[32 : 32 + len(ota.APP_IDENTITY)] = ota.APP_IDENTITY
        image = ota.validate_application(bytes(data))
        metadata = recovery.build_valid_metadata(image, 1, (1, 15, 0))
        encoded = recovery.encode_ihex(
            ((ota.APP_BASE, image.data), (ota.META_BASE, metadata))
        ).decode("ascii")

        upper = 0
        maximum = 0
        for line in encoded.splitlines():
            raw = bytes.fromhex(line[1:])
            self.assertEqual(sum(raw) & 0xFF, 0)
            record_type = raw[3]
            if record_type == 4:
                upper = int.from_bytes(raw[4:6], "big") << 16
            elif record_type == 0:
                address = upper + int.from_bytes(raw[1:3], "big")
                maximum = max(maximum, address + raw[0])
        self.assertEqual(maximum, recovery.CALIB_BASE)


if __name__ == "__main__":
    unittest.main()
