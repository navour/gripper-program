"""Generate ST-Link recovery images for the STM32G431 gripper.

The recovery HEX installs valid OTA metadata together with the Application, so
the Bootloader applies the same vector and CRC checks after an offline repair as
it does after an RS485 update.  The calibration page is deliberately omitted.
"""

from __future__ import annotations

import argparse
from dataclasses import dataclass
import pathlib
import struct
from typing import Iterable, Sequence, Tuple
import zlib

from gripper_ota_protocol import (
    APP_BASE,
    APP_SIZE,
    LAYOUT_VERSION,
    MAILBOX_BASE,
    META_BASE,
    PRODUCT_ID,
    SRAM_BASE,
    ImageInfo,
    OtaError,
    validate_application,
)


BOOT_BASE = 0x08000000
BOOT_SIZE = 0x00006000
FACTORY_MARKER_BASE = 0x08005800
FACTORY_MARKER_MAGIC = 0x4B4C5453
FACTORY_MARKER_MAGIC_INV = (~FACTORY_MARKER_MAGIC) & 0xFFFFFFFF
META_SIZE = 0x00000800
CALIB_BASE = 0x0801F800

META_MAGIC = 0x4154454D
META_RECORD_VERSION = 1
META_STATE_VALID = 2
META_COMMIT_MARKER = 0xA55A3CC3
META_RECORD_SIZE = 64
META_CRC_LENGTH = 56

_META_PREFIX_FORMAT = "<IHBBIIIIIIHHHHI3I"


@dataclass(frozen=True)
class RecoveryArtifacts:
    app_recovery_hex: pathlib.Path
    full_recovery_hex: pathlib.Path
    metadata_bin: pathlib.Path
    info_txt: pathlib.Path


def build_valid_metadata(
    image: ImageInfo,
    role: int,
    version: Tuple[int, int, int],
    flags: int = 0,
) -> bytes:
    """Return one erased metadata page containing a committed VALID record."""

    if role not in (0, 1, 2):
        raise OtaError("Role必须是0、1或2")
    if len(version) != 3 or any(value < 0 or value > 0xFFFF for value in version):
        raise OtaError("版本号的Major/Minor/Patch必须位于0~65535")

    prefix = struct.pack(
        _META_PREFIX_FORMAT,
        META_MAGIC,
        META_RECORD_VERSION,
        META_STATE_VALID,
        role,
        1,  # first record in a freshly erased metadata page
        0,  # no OTA session: installed through ST-Link
        PRODUCT_ID,
        LAYOUT_VERSION,
        image.size,
        image.crc32,
        version[0],
        version[1],
        version[2],
        flags & 0xFFFF,
        image.size,
        0,
        0,
        0,
    )
    if len(prefix) != META_CRC_LENGTH:
        raise AssertionError("metadata CRC prefix must be 56 bytes")

    record_crc = zlib.crc32(prefix) & 0xFFFFFFFF
    record = prefix + struct.pack("<II", record_crc, META_COMMIT_MARKER)
    if len(record) != META_RECORD_SIZE:
        raise AssertionError("metadata record must be 64 bytes")
    return record + bytes([0xFF]) * (META_SIZE - len(record))


def validate_bootloader(data: bytes) -> None:
    if len(data) < 8:
        raise OtaError("Bootloader BIN不足8字节")
    if len(data) > BOOT_SIZE:
        raise OtaError(
            f"Bootloader为{len(data)}字节，超过分区上限{BOOT_SIZE}字节"
        )

    initial_msp, reset_handler = struct.unpack_from("<II", data, 0)
    reset_address = reset_handler & ~1
    if not (SRAM_BASE <= initial_msp <= MAILBOX_BASE) or (initial_msp & 0x7):
        raise OtaError(f"Bootloader初始MSP 0x{initial_msp:08X}无效")
    if not (reset_handler & 1) or not (
        BOOT_BASE <= reset_address < BOOT_BASE + len(data)
    ):
        raise OtaError(f"Bootloader Reset_Handler 0x{reset_handler:08X}无效")
    marker_offset = FACTORY_MARKER_BASE - BOOT_BASE
    if len(data) < marker_offset + 8:
        raise OtaError("Bootloader BIN缺少0x08005800处的ST-Link首次许可")
    marker = struct.unpack_from("<II", data, marker_offset)
    if marker != (FACTORY_MARKER_MAGIC, FACTORY_MARKER_MAGIC_INV):
        raise OtaError("Bootloader BIN中的ST-Link首次许可内容无效，请重新Rebuild")


def _ihex_record(address: int, record_type: int, data: bytes) -> str:
    if len(data) > 0xFF or address < 0 or address > 0xFFFF:
        raise ValueError("invalid Intel HEX record")
    body = bytes(
        [len(data), (address >> 8) & 0xFF, address & 0xFF, record_type]
    ) + data
    checksum = (-sum(body)) & 0xFF
    return ":" + (body + bytes([checksum])).hex().upper()


def encode_ihex(segments: Iterable[Tuple[int, bytes]]) -> bytes:
    """Encode non-overlapping absolute-address segments as Intel HEX."""

    ordered = sorted((base, bytes(data)) for base, data in segments if data)
    previous_end = -1
    for base, data in ordered:
        if base < 0 or base < previous_end:
            raise ValueError("Intel HEX segments overlap or have an invalid address")
        previous_end = base + len(data)

    lines = []
    active_upper = None
    for base, data in ordered:
        offset = 0
        while offset < len(data):
            absolute = base + offset
            upper = (absolute >> 16) & 0xFFFF
            lower = absolute & 0xFFFF
            if upper != active_upper:
                lines.append(_ihex_record(0, 0x04, struct.pack(">H", upper)))
                active_upper = upper
            count = min(16, len(data) - offset, 0x10000 - lower)
            lines.append(_ihex_record(lower, 0x00, data[offset : offset + count]))
            offset += count

    lines.append(_ihex_record(0, 0x01, b""))
    return ("\n".join(lines) + "\n").encode("ascii")


def generate_recovery_files(
    firmware_path: pathlib.Path,
    bootloader_path: pathlib.Path,
    output_dir: pathlib.Path,
    role: int,
    version: Tuple[int, int, int],
) -> RecoveryArtifacts:
    firmware_path = pathlib.Path(firmware_path)
    bootloader_path = pathlib.Path(bootloader_path)
    output_dir = pathlib.Path(output_dir)

    if not firmware_path.is_file():
        raise OtaError("请选择有效的Application BIN文件")
    if not bootloader_path.is_file():
        raise OtaError("找不到Bootloader BIN文件，请先Rebuild Bootloader")

    image = validate_application(firmware_path.read_bytes())
    bootloader = bootloader_path.read_bytes()
    validate_bootloader(bootloader)
    metadata = build_valid_metadata(image, role, version)

    if META_BASE + META_SIZE != CALIB_BASE:
        raise AssertionError("metadata/calibration layout mismatch")

    output_dir.mkdir(parents=True, exist_ok=True)
    stem = f"gripper{role}" if role in (1, 2) else "gripper_common"
    app_hex = output_dir / f"{stem}_stlink_app_recovery.hex"
    full_hex = output_dir / f"{stem}_stlink_full_recovery.hex"
    metadata_bin = output_dir / f"{stem}_metadata.bin"
    info_txt = output_dir / f"{stem}_recovery_info.txt"

    app_hex.write_bytes(
        encode_ihex(((APP_BASE, image.data), (META_BASE, metadata)))
    )
    full_hex.write_bytes(
        encode_ihex(
            (
                (BOOT_BASE, bootloader),
                (APP_BASE, image.data),
                (META_BASE, metadata),
            )
        )
    )
    metadata_bin.write_bytes(metadata)
    info_txt.write_text(
        "STM32G431夹爪ST-Link救援包\n"
        f"Role: {role}\n"
        f"Version: {version[0]}.{version[1]}.{version[2]}\n"
        f"Application: 0x{APP_BASE:08X}, {image.size} bytes, "
        f"CRC32=0x{image.crc32:08X}\n"
        f"Bootloader: 0x{BOOT_BASE:08X}, {len(bootloader)} bytes\n"
        f"Metadata: 0x{META_BASE:08X}, {META_SIZE} bytes\n"
        f"Calibration excluded: 0x{CALIB_BASE:08X}~0x0801FFFF\n\n"
        "普通Application救援使用*_stlink_app_recovery.hex；\n"
        "Bootloader也损坏时使用*_stlink_full_recovery.hex。\n"
        "STM32CubeProgrammer必须启用扇区擦除和Verify，禁止Full Chip Erase。\n",
        encoding="utf-8",
    )

    return RecoveryArtifacts(app_hex, full_hex, metadata_bin, info_txt)


def _parse_version(value: str) -> Tuple[int, int, int]:
    try:
        parts = tuple(int(item, 0) for item in value.split("."))
    except ValueError as exc:
        raise argparse.ArgumentTypeError("版本格式应为Major.Minor.Patch") from exc
    if len(parts) != 3 or any(item < 0 or item > 0xFFFF for item in parts):
        raise argparse.ArgumentTypeError("版本格式应为Major.Minor.Patch")
    return parts  # type: ignore[return-value]


def main(argv: Sequence[str] | None = None) -> int:
    tool_dir = pathlib.Path(__file__).resolve().parent
    default_bootloader = (
        tool_dir.parent
        / "Bootloader_G431"
        / "Bootloader_G431"
        / "MDK-ARM"
        / "Bootloader_G431"
        / "bootloader_g431.bin"
    )
    parser = argparse.ArgumentParser(description="生成夹爪ST-Link离线救援HEX")
    parser.add_argument("firmware", type=pathlib.Path, help="gripper Application BIN")
    parser.add_argument("--bootloader", type=pathlib.Path, default=default_bootloader)
    parser.add_argument("--output-dir", type=pathlib.Path, required=True)
    parser.add_argument("--role", type=int, choices=(0, 1, 2), required=True)
    parser.add_argument("--version", type=_parse_version, required=True)
    args = parser.parse_args(argv)

    artifacts = generate_recovery_files(
        args.firmware,
        args.bootloader,
        args.output_dir,
        args.role,
        args.version,
    )
    for path in artifacts.__dict__.values():
        print(path)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
