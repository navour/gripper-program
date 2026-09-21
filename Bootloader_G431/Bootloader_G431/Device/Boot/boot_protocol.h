#ifndef BOOT_PROTOCOL_H
#define BOOT_PROTOCOL_H

#include <stdint.h>

#define BOOT_PROTOCOL_MAX_PAYLOAD       (192U)
#define BOOT_PROTOCOL_HEADER_SIZE       (18U)
#define BOOT_PROTOCOL_CRC_SIZE          (2U)
#define BOOT_PROTOCOL_MAX_FRAME_SIZE    (BOOT_PROTOCOL_HEADER_SIZE + \
                                         BOOT_PROTOCOL_MAX_PAYLOAD + \
                                         BOOT_PROTOCOL_CRC_SIZE)

#define BOOT_COMMAND_HELLO              (0x01U)
#define BOOT_COMMAND_BEGIN              (0x02U)
#define BOOT_COMMAND_DATA               (0x03U)
#define BOOT_COMMAND_END                (0x04U)
#define BOOT_COMMAND_QUERY              (0x05U)
#define BOOT_COMMAND_ABORT              (0x06U)
#define BOOT_COMMAND_REBOOT             (0x07U)
#define BOOT_COMMAND_RESPONSE_MASK      (0x80U)

typedef enum
{
    BOOT_STATUS_OK = 0x0000U,
    BOOT_STATUS_BAD_FRAME_CRC = 0x0001U,
    BOOT_STATUS_BAD_PROTOCOL_VERSION = 0x0002U,
    BOOT_STATUS_BAD_SESSION = 0x0003U,
    BOOT_STATUS_BAD_SEQUENCE = 0x0004U,
    BOOT_STATUS_BAD_OFFSET = 0x0005U,
    BOOT_STATUS_BAD_LENGTH = 0x0006U,
    BOOT_STATUS_WRONG_PRODUCT = 0x0007U,
    BOOT_STATUS_WRONG_ROLE = 0x0008U,
    BOOT_STATUS_IMAGE_TOO_LARGE = 0x0009U,
    BOOT_STATUS_META_WRITE_FAILED = 0x000AU,
    BOOT_STATUS_ERASE_FAILED = 0x000BU,
    BOOT_STATUS_PROGRAM_FAILED = 0x000CU,
    BOOT_STATUS_READBACK_FAILED = 0x000DU,
    BOOT_STATUS_IMAGE_CRC_FAILED = 0x000EU,
    BOOT_STATUS_IMAGE_HASH_FAILED = 0x000FU,
    BOOT_STATUS_SIGNATURE_FAILED = 0x0010U,
    BOOT_STATUS_INVALID_VECTOR = 0x0011U,
    BOOT_STATUS_APP_NOT_VALID = 0x0012U,
    BOOT_STATUS_BUSY = 0x0013U,
    BOOT_STATUS_UNSUPPORTED = 0x0014U,
    BOOT_STATUS_INTERNAL_ERROR = 0x0015U
} BootStatus_t;

typedef struct
{
    uint8_t protocol_version;
    uint8_t command;
    uint32_t session_id;
    uint16_t sequence;
    uint16_t flags_or_status;
    uint32_t offset;
    uint16_t payload_length;
    uint8_t payload[BOOT_PROTOCOL_MAX_PAYLOAD];
} BootFrame_t;

typedef enum
{
    BOOT_RX_TIMEOUT = 0,
    BOOT_RX_FRAME,
    BOOT_RX_BAD_CRC,
    BOOT_RX_BAD_LENGTH,
    BOOT_RX_IO_ERROR
} BootReceiveResult_t;

BootReceiveResult_t BootProtocol_Receive(BootFrame_t *frame,
                                         uint32_t first_byte_timeout_ms);

uint8_t BootProtocol_SendResponse(const BootFrame_t *request,
                                  BootStatus_t status,
                                  uint32_t next_offset,
                                  const uint8_t *payload,
                                  uint16_t payload_length);

uint16_t BootProtocol_GetU16(const uint8_t *bytes);
uint32_t BootProtocol_GetU32(const uint8_t *bytes);
void BootProtocol_PutU16(uint8_t *bytes, uint16_t value);
void BootProtocol_PutU32(uint8_t *bytes, uint32_t value);

#endif /* BOOT_PROTOCOL_H */
