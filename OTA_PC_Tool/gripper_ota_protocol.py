"""STM32G431 gripper Bootloader protocol and Modbus handoff helpers."""

from __future__ import annotations

from dataclasses import dataclass
import secrets
import struct
import time
import zlib
from typing import Callable, Optional, Tuple


SOF = b"GB"
PROTOCOL_VERSION = 1
HEADER_FORMAT = "<2sBBIHHIH"
HEADER_SIZE = struct.calcsize(HEADER_FORMAT)
MAX_PAYLOAD = 192

CMD_HELLO = 0x01
CMD_BEGIN = 0x02
CMD_DATA = 0x03
CMD_END = 0x04
CMD_QUERY = 0x05
CMD_ABORT = 0x06
CMD_REBOOT = 0x07
RESPONSE_MASK = 0x80

STATUS_NAMES = {
    0x0000: "OK",
    0x0001: "BAD_FRAME_CRC",
    0x0002: "BAD_PROTOCOL_VERSION",
    0x0003: "BAD_SESSION",
    0x0004: "BAD_SEQUENCE",
    0x0005: "BAD_OFFSET",
    0x0006: "BAD_LENGTH",
    0x0007: "WRONG_PRODUCT",
    0x0008: "WRONG_ROLE",
    0x0009: "IMAGE_TOO_LARGE",
    0x000A: "META_WRITE_FAILED",
    0x000B: "ERASE_FAILED",
    0x000C: "PROGRAM_FAILED",
    0x000D: "READBACK_FAILED",
    0x000E: "IMAGE_CRC_FAILED",
    0x000F: "IMAGE_HASH_FAILED",
    0x0010: "SIGNATURE_FAILED",
    0x0011: "INVALID_VECTOR",
    0x0012: "APP_NOT_VALID",
    0x0013: "BUSY",
    0x0014: "UNSUPPORTED",
    0x0015: "INTERNAL_ERROR",
}

STATE_NAMES = {0: "EMPTY", 1: "DOWNLOADING", 2: "VALID", 3: "INVALID"}

PRODUCT_ID = 0x0000D431
LAYOUT_VERSION = 1
APP_BASE = 0x08006000
APP_SIZE = 0x00019000
META_BASE = 0x0801F000
SRAM_BASE = 0x20000000
MAILBOX_BASE = 0x20007FE0
APP_IDENTITY_MAGIC = 0x50415247
APP_IDENTITY_TAIL = 0x21444947
APP_IDENTITY = struct.pack(
    "<IIIII",
    APP_IDENTITY_MAGIC,
    PRODUCT_ID,
    LAYOUT_VERSION,
    APP_BASE,
    APP_IDENTITY_TAIL,
)

MANIFEST_MAGIC = 0x474D4947
MANIFEST_FORMAT_VERSION = 1
MANIFEST_SIZE = 136
MANIFEST_FORMAT = "<IHHIBBHHHHHIIII"

REG_ENABLE = 0x0012
REG_OTA_KEY1 = 0x00F0
REG_OTA_KEY2 = 0x00F1
REG_OTA_CTRL = 0x00F2
OTA_KEY1 = 0x4F54
OTA_KEY2 = 0x4131


class OtaError(RuntimeError):
    """Base error for the updater."""


class ProtocolTimeout(OtaError):
    """No matching response was received before the deadline."""


class FrameError(OtaError):
    """A malformed Bootloader frame was received."""


class UpdateCancelled(OtaError):
    """The user cancelled an active update."""


class BootStatusError(OtaError):
    def __init__(self, frame: "BootFrame") -> None:
        self.frame = frame
        name = STATUS_NAMES.get(frame.status, f"0x{frame.status:04X}")
        super().__init__(
            f"Bootloader返回错误 {name}，next_offset=0x{frame.offset:08X}"
        )


@dataclass(frozen=True)
class BootFrame:
    command: int
    session_id: int
    sequence: int
    status: int
    offset: int
    payload: bytes


@dataclass(frozen=True)
class ImageInfo:
    data: bytes
    size: int
    crc32: int
    initial_msp: int
    reset_handler: int


@dataclass(frozen=True)
class HelloInfo:
    product_id: int
    layout_version: int
    app_base: int
    app_size: int
    meta_base: int
    boot_version: int
    max_payload: int
    protocol_version: int
    device_role: int
    state: int
    capabilities: int
    reported_session: int
    next_offset: int


@dataclass(frozen=True)
class QueryInfo:
    state: int
    device_role: int
    boot_version: int
    reported_session: int
    next_offset: int
    image_size: int
    image_crc32: int
    metadata_sequence: int


def crc16_modbus(data: bytes) -> int:
    crc = 0xFFFF
    for value in data:
        crc ^= value
        for _ in range(8):
            crc = ((crc >> 1) ^ 0xA001) if (crc & 1) else (crc >> 1)
    return crc & 0xFFFF


def encode_frame(
    command: int,
    session_id: int,
    sequence: int,
    offset: int = 0,
    payload: bytes = b"",
    flags: int = 0,
) -> bytes:
    if len(payload) > MAX_PAYLOAD:
        raise ValueError(f"payload不能超过{MAX_PAYLOAD}字节")
    header = struct.pack(
        HEADER_FORMAT,
        SOF,
        PROTOCOL_VERSION,
        command & 0xFF,
        session_id & 0xFFFFFFFF,
        sequence & 0xFFFF,
        flags & 0xFFFF,
        offset & 0xFFFFFFFF,
        len(payload),
    )
    raw = header + payload
    return raw + struct.pack("<H", crc16_modbus(raw))


def validate_application(data: bytes) -> ImageInfo:
    if len(data) < 8:
        raise OtaError("固件不足8字节，缺少MSP和Reset_Handler")
    if len(data) > APP_SIZE:
        raise OtaError(
            f"固件{len(data)}字节超过Application上限{APP_SIZE}字节"
        )

    initial_msp, reset_handler = struct.unpack_from("<II", data, 0)
    if not (SRAM_BASE <= initial_msp <= MAILBOX_BASE):
        raise OtaError(f"初始MSP 0x{initial_msp:08X}不在可用SRAM范围")
    if initial_msp & 0x7:
        raise OtaError(f"初始MSP 0x{initial_msp:08X}未按8字节对齐")
    reset_address = reset_handler & ~1
    if not (reset_handler & 1):
        raise OtaError(f"Reset_Handler 0x{reset_handler:08X}没有Thumb位")
    if not (APP_BASE <= reset_address < APP_BASE + len(data)):
        raise OtaError(
            f"Reset_Handler 0x{reset_handler:08X}不在当前Application镜像内；"
            "请检查Keil IROM1和VTOR是否为0x08006000"
        )
    if APP_IDENTITY not in data:
        raise OtaError(
            "固件缺少夹爪OTA身份签名；请使用已适配ST-Link/OTA双路径的"
            "Application重新Rebuild"
        )

    return ImageInfo(
        data=data,
        size=len(data),
        crc32=zlib.crc32(data) & 0xFFFFFFFF,
        initial_msp=initial_msp,
        reset_handler=reset_handler,
    )


def build_manifest(
    image: ImageInfo,
    role: int,
    version: Tuple[int, int, int],
    hardware_revision: int = 0,
    min_boot_version: int = 0x0100,
) -> bytes:
    if role not in (0, 1, 2):
        raise ValueError("role必须是0、1或2")
    major, minor, patch = version
    fixed = struct.pack(
        MANIFEST_FORMAT,
        MANIFEST_MAGIC,
        MANIFEST_FORMAT_VERSION,
        MANIFEST_SIZE,
        PRODUCT_ID,
        role,
        hardware_revision & 0xFF,
        0,  # flags: CRC-only prototype
        major & 0xFFFF,
        minor & 0xFFFF,
        patch & 0xFFFF,
        min_boot_version & 0xFFFF,
        LAYOUT_VERSION,
        APP_BASE,
        image.size,
        image.crc32,
    )
    manifest = fixed + bytes(32) + bytes(64)
    if len(manifest) != MANIFEST_SIZE:
        raise AssertionError("manifest wire size mismatch")
    return manifest


class SerialBootClient:
    def __init__(self, serial_port, log: Optional[Callable[[str], None]] = None):
        self.serial = serial_port
        self.log = log or (lambda _message: None)

    def _read_exact(self, length: int, deadline: float) -> bytes:
        result = bytearray()
        while len(result) < length:
            if time.monotonic() >= deadline:
                raise ProtocolTimeout(
                    f"接收超时：需要{length}字节，只收到{len(result)}字节"
                )
            chunk = self.serial.read(length - len(result))
            if chunk:
                result.extend(chunk)
        return bytes(result)

    def _find_sof(self, deadline: float) -> None:
        have_g = False
        while time.monotonic() < deadline:
            value = self.serial.read(1)
            if not value:
                continue
            if not have_g:
                have_g = value == SOF[:1]
            elif value == SOF[1:2]:
                return
            else:
                have_g = value == SOF[:1]
        raise ProtocolTimeout("等待Bootloader帧头GB超时")

    def read_frame(self, timeout: float) -> BootFrame:
        deadline = time.monotonic() + timeout
        self._find_sof(deadline)
        header_tail = self._read_exact(HEADER_SIZE - 2, deadline)
        header = SOF + header_tail
        (
            _sof,
            protocol,
            command,
            session_id,
            sequence,
            status,
            offset,
            payload_length,
        ) = struct.unpack(HEADER_FORMAT, header)

        if protocol != PROTOCOL_VERSION:
            raise FrameError(f"不支持的协议版本{protocol}")
        if payload_length > MAX_PAYLOAD:
            raise FrameError(f"响应payload长度{payload_length}越界")

        tail = self._read_exact(payload_length + 2, deadline)
        payload = tail[:payload_length]
        received_crc = struct.unpack_from("<H", tail, payload_length)[0]
        expected_crc = crc16_modbus(header + payload)
        if received_crc != expected_crc:
            raise FrameError(
                f"响应CRC16错误：收到0x{received_crc:04X}，计算0x{expected_crc:04X}"
            )
        return BootFrame(command, session_id, sequence, status, offset, payload)

    def transact(
        self,
        command: int,
        session_id: int,
        sequence: int,
        *,
        offset: int = 0,
        payload: bytes = b"",
        timeout: float = 1.0,
        retries: int = 3,
    ) -> BootFrame:
        request = encode_frame(command, session_id, sequence, offset, payload)
        last_error: Optional[Exception] = None
        for attempt in range(1, retries + 1):
            self.serial.write(request)
            self.serial.flush()
            deadline = time.monotonic() + timeout
            while time.monotonic() < deadline:
                try:
                    response = self.read_frame(max(0.02, deadline - time.monotonic()))
                except (ProtocolTimeout, FrameError) as exc:
                    last_error = exc
                    break

                # Ignore local echo or stale frames from an earlier request.
                if response.command != (command | RESPONSE_MASK):
                    continue
                if response.session_id != (session_id & 0xFFFFFFFF):
                    continue
                if response.sequence != (sequence & 0xFFFF):
                    continue
                if response.status != 0:
                    raise BootStatusError(response)
                return response

            self.log(f"命令0x{command:02X}第{attempt}次等待超时，准备重试")
        raise ProtocolTimeout(
            f"命令0x{command:02X}重试{retries}次仍无匹配响应"
        ) from last_error

    def hello(self, session_id: int, sequence: int = 0, timeout: float = 0.5) -> HelloInfo:
        frame = self.transact(
            CMD_HELLO,
            session_id,
            sequence,
            timeout=timeout,
            retries=1,
        )
        if len(frame.payload) != 40:
            raise FrameError(f"HELLO payload应为40字节，实际{len(frame.payload)}")
        p = frame.payload
        return HelloInfo(
            product_id=struct.unpack_from("<I", p, 0)[0],
            layout_version=struct.unpack_from("<I", p, 4)[0],
            app_base=struct.unpack_from("<I", p, 8)[0],
            app_size=struct.unpack_from("<I", p, 12)[0],
            meta_base=struct.unpack_from("<I", p, 16)[0],
            boot_version=struct.unpack_from("<H", p, 20)[0],
            max_payload=struct.unpack_from("<H", p, 22)[0],
            protocol_version=p[24],
            device_role=p[25],
            state=p[26],
            capabilities=struct.unpack_from("<I", p, 28)[0],
            reported_session=struct.unpack_from("<I", p, 32)[0],
            next_offset=struct.unpack_from("<I", p, 36)[0],
        )

    def query(self, session_id: int, sequence: int = 0) -> QueryInfo:
        frame = self.transact(CMD_QUERY, session_id, sequence)
        if len(frame.payload) != 24:
            raise FrameError(f"QUERY payload应为24字节，实际{len(frame.payload)}")
        p = frame.payload
        return QueryInfo(
            state=p[0],
            device_role=p[1],
            boot_version=struct.unpack_from("<H", p, 2)[0],
            reported_session=struct.unpack_from("<I", p, 4)[0],
            next_offset=struct.unpack_from("<I", p, 8)[0],
            image_size=struct.unpack_from("<I", p, 12)[0],
            image_crc32=struct.unpack_from("<I", p, 16)[0],
            metadata_sequence=struct.unpack_from("<I", p, 20)[0],
        )

    def update(
        self,
        image: ImageInfo,
        role: int,
        version: Tuple[int, int, int],
        *,
        hardware_revision: int = 0,
        progress: Optional[Callable[[int, int], None]] = None,
        cancelled: Optional[Callable[[], bool]] = None,
    ) -> Tuple[int, QueryInfo]:
        progress = progress or (lambda _done, _total: None)
        cancelled = cancelled or (lambda: False)
        session_id = secrets.randbits(32) or 1
        sequence = 0
        manifest = build_manifest(
            image, role, version, hardware_revision=hardware_revision
        )

        self.log(f"BEGIN：session=0x{session_id:08X}，擦除Application区")
        begin = self.transact(
            CMD_BEGIN,
            session_id,
            sequence,
            payload=manifest,
            timeout=8.0,
            retries=2,
        )
        if begin.offset != 0:
            raise OtaError(f"BEGIN返回的next_offset不是0：{begin.offset}")

        sequence = (sequence + 1) & 0xFFFF
        offset = 0
        progress(0, image.size)
        while offset < image.size:
            if cancelled():
                try:
                    self.transact(CMD_ABORT, session_id, sequence, retries=1)
                except OtaError:
                    pass
                raise UpdateCancelled("升级已由用户取消，Application保持无效状态")

            chunk = image.data[offset : offset + MAX_PAYLOAD]
            response = self.transact(
                CMD_DATA,
                session_id,
                sequence,
                offset=offset,
                payload=chunk,
                timeout=1.5,
                retries=4,
            )
            expected_next = offset + len(chunk)
            if response.offset != expected_next:
                raise OtaError(
                    f"DATA next_offset异常：期望{expected_next}，收到{response.offset}"
                )
            offset = response.offset
            sequence = (sequence + 1) & 0xFFFF
            progress(offset, image.size)

        self.log("END：Bootloader正在校验向量表和整包CRC32")
        try:
            end = self.transact(
                CMD_END,
                session_id,
                sequence,
                offset=image.size,
                timeout=5.0,
                retries=2,
            )
            if end.offset != image.size:
                raise OtaError(
                    f"END返回大小异常：期望{image.size}，收到{end.offset}"
                )
        except (ProtocolTimeout, BootStatusError) as exc:
            # END可能成功但ACK在总线上丢失；用Metadata状态做最终判断。
            if isinstance(exc, BootStatusError) and exc.frame.status != 0x0003:
                raise
            query_after_timeout = self.query(session_id, sequence)
            if not (
                query_after_timeout.state == 2
                and query_after_timeout.image_size == image.size
                and query_after_timeout.image_crc32 == image.crc32
            ):
                raise

        final_query = self.query(session_id, (sequence + 1) & 0xFFFF)
        if not (
            final_query.state == 2
            and final_query.image_size == image.size
            and final_query.image_crc32 == image.crc32
        ):
            raise OtaError(
                "END后Metadata不是VALID，或固件大小/CRC与本地文件不一致"
            )
        return session_id, final_query

    def reboot(self, session_id: int, sequence: int = 0) -> None:
        self.transact(CMD_REBOOT, session_id, sequence, timeout=1.5, retries=2)


def _read_serial_exact(serial_port, length: int, timeout: float) -> bytes:
    deadline = time.monotonic() + timeout
    data = bytearray()
    while len(data) < length and time.monotonic() < deadline:
        chunk = serial_port.read(length - len(data))
        if chunk:
            data.extend(chunk)
    if len(data) != length:
        raise ProtocolTimeout(f"Modbus响应超时：期望{length}字节，收到{len(data)}字节")
    return bytes(data)


def modbus_write_single(serial_port, slave: int, register: int, value: int) -> None:
    body = struct.pack(">BBHH", slave & 0xFF, 0x06, register & 0xFFFF, value & 0xFFFF)
    request = body + struct.pack("<H", crc16_modbus(body))
    serial_port.reset_input_buffer()
    serial_port.write(request)
    serial_port.flush()

    prefix = _read_serial_exact(serial_port, 2, 0.5)
    response_length = 5 if (prefix[1] & 0x80) else 8
    response = prefix + _read_serial_exact(serial_port, response_length - 2, 0.5)
    if crc16_modbus(response[:-2]) != struct.unpack_from("<H", response, len(response) - 2)[0]:
        raise FrameError("Application Modbus响应CRC错误")
    if response[0] != (slave & 0xFF):
        raise FrameError("Application Modbus响应从站地址不匹配")
    if response[1] & 0x80:
        raise OtaError(
            f"Application拒绝写寄存器0x{register:04X}，异常码0x{response[2]:02X}"
        )
    if response != request:
        raise FrameError(f"写寄存器0x{register:04X}的回显内容不匹配")


def enter_bootloader_from_application(serial_port, slave: int = 1) -> None:
    """Safely disable the motor, authorize OTA, then request a mailbox reset."""
    serial_port.reset_input_buffer()
    modbus_write_single(serial_port, slave, REG_ENABLE, 0)
    time.sleep(0.02)
    modbus_write_single(serial_port, slave, REG_OTA_KEY1, OTA_KEY1)
    modbus_write_single(serial_port, slave, REG_OTA_KEY2, OTA_KEY2)
    modbus_write_single(serial_port, slave, REG_OTA_CTRL, 1)


def wait_for_bootloader(
    client: SerialBootClient,
    timeout: float = 3.0,
    session_id: Optional[int] = None,
) -> Tuple[int, HelloInfo]:
    session_id = session_id or secrets.randbits(32) or 1
    deadline = time.monotonic() + timeout
    last_error: Optional[Exception] = None
    while time.monotonic() < deadline:
        try:
            client.serial.reset_input_buffer()
            return session_id, client.hello(session_id, timeout=0.25)
        except OtaError as exc:
            last_error = exc
            time.sleep(0.05)
    raise ProtocolTimeout("在等待时间内没有发现夹爪Bootloader") from last_error


def describe_hello(info: HelloInfo) -> str:
    return (
        f"product=0x{info.product_id:08X}, layout={info.layout_version}, "
        f"app=0x{info.app_base:08X}+0x{info.app_size:X}, "
        f"boot={info.boot_version >> 8}.{info.boot_version & 0xFF}, "
        f"role={info.device_role}, state={STATE_NAMES.get(info.state, info.state)}, "
        f"max_payload={info.max_payload}"
    )


def validate_hello(info: HelloInfo, requested_role: int) -> None:
    if info.product_id != PRODUCT_ID:
        raise OtaError(f"产品ID不匹配：0x{info.product_id:08X}")
    if info.layout_version != LAYOUT_VERSION:
        raise OtaError(f"Flash布局版本不匹配：{info.layout_version}")
    if info.app_base != APP_BASE or info.app_size != APP_SIZE:
        raise OtaError("Bootloader报告的Application分区与上位机不一致")
    if info.protocol_version != PROTOCOL_VERSION:
        raise OtaError(f"Bootloader协议版本不匹配：{info.protocol_version}")
    if info.max_payload < MAX_PAYLOAD:
        raise OtaError(f"Bootloader最大payload仅{info.max_payload}字节")
    if info.device_role not in (0, requested_role) and requested_role != 0:
        raise OtaError(
            f"目标Bootloader锁定role={info.device_role}，所选固件role={requested_role}"
        )
