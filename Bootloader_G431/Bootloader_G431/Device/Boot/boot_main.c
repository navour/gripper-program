#include "boot_main.h"

#include <string.h>

#include "main.h"

#include "boot_flash.h"
#include "boot_image_verify.h"
#include "boot_jump.h"
#include "boot_metadata.h"
#include "boot_protocol.h"
#include "gripper_ota_layout.h"

#define BOOT_MANIFEST_MAGIC             (0x474D4947UL) /* bytes: GIMG */
#define BOOT_MANIFEST_FORMAT_VERSION    (1U)
#define BOOT_MANIFEST_WIRE_SIZE         (136U)
#define BOOT_MANIFEST_SUPPORTED_FLAGS   (0U)

#define BOOT_CAP_CRC32_ISO_HDLC         (1UL << 0)
#define BOOT_CAP_DUPLICATE_DATA_ACK     (1UL << 1)
#define BOOT_CAP_SESSION_QUERY          (1UL << 2)
#define BOOT_CAP_ROLE_LOCKED            (1UL << 3)

#define BOOT_HELLO_PAYLOAD_SIZE         (40U)
#define BOOT_QUERY_PAYLOAD_SIZE         (24U)
#define BOOT_POLL_INTERVAL_MS           (50UL)
#define BOOT_REBOOT_DELAY_MS            (20UL)

typedef struct
{
    uint32_t manifest_magic;
    uint16_t format_version;
    uint16_t header_size;
    uint32_t product_id;
    uint8_t role;
    uint8_t hardware_revision;
    uint16_t flags;
    uint16_t version_major;
    uint16_t version_minor;
    uint16_t version_patch;
    uint16_t min_boot_version;
    uint32_t layout_version;
    uint32_t image_base;
    uint32_t image_size;
    uint32_t image_crc32;
    uint8_t image_sha256[32];
    uint8_t signature[64];
} BootManifest_t;

typedef struct
{
    uint8_t active;
    uint16_t expected_sequence;
    uint32_t session_id;
    uint32_t next_offset;
    BootManifest_t manifest;
} BootDownloadContext_t;

static BootDownloadContext_t g_download;
static BootMetaRecord_t g_latest_metadata;
static uint8_t g_have_metadata;
static uint32_t g_mailbox_session;

static void BootMain_RefreshMetadata(void)
{
    g_have_metadata = BootMeta_GetLatest(&g_latest_metadata);
    if (g_have_metadata == 0U)
    {
        (void)memset(&g_latest_metadata, 0, sizeof(g_latest_metadata));
    }
}

static uint8_t BootMain_ReportedState(void)
{
    if (g_download.active != 0U)
    {
        return BOOT_META_STATE_DOWNLOADING;
    }

    return (g_have_metadata != 0U) ? g_latest_metadata.state
                                  : BOOT_META_STATE_EMPTY;
}

static uint32_t BootMain_ReportedSession(void)
{
    if (g_download.active != 0U)
    {
        return g_download.session_id;
    }
    if (g_have_metadata != 0U)
    {
        return g_latest_metadata.session_id;
    }
    return g_mailbox_session;
}

static uint8_t BootMain_FactoryApplicationIsValid(void)
{
    return ((BootFlash_FactoryMarkerIsArmed() != 0U) &&
            (BootImage_HasFactoryIdentity() != 0U) &&
            (BootImage_VectorIsValid(GRIPPER_APP_SIZE) != 0U))
               ? 1U
               : 0U;
}

static uint32_t BootMain_ReportedNextOffset(void)
{
    return (g_download.active != 0U) ? g_download.next_offset : 0UL;
}

static void BootMain_ParseManifest(const uint8_t *payload,
                                   BootManifest_t *manifest)
{
    manifest->manifest_magic = BootProtocol_GetU32(&payload[0]);
    manifest->format_version = BootProtocol_GetU16(&payload[4]);
    manifest->header_size = BootProtocol_GetU16(&payload[6]);
    manifest->product_id = BootProtocol_GetU32(&payload[8]);
    manifest->role = payload[12];
    manifest->hardware_revision = payload[13];
    manifest->flags = BootProtocol_GetU16(&payload[14]);
    manifest->version_major = BootProtocol_GetU16(&payload[16]);
    manifest->version_minor = BootProtocol_GetU16(&payload[18]);
    manifest->version_patch = BootProtocol_GetU16(&payload[20]);
    manifest->min_boot_version = BootProtocol_GetU16(&payload[22]);
    manifest->layout_version = BootProtocol_GetU32(&payload[24]);
    manifest->image_base = BootProtocol_GetU32(&payload[28]);
    manifest->image_size = BootProtocol_GetU32(&payload[32]);
    manifest->image_crc32 = BootProtocol_GetU32(&payload[36]);
    (void)memcpy(manifest->image_sha256, &payload[40], 32U);
    (void)memcpy(manifest->signature, &payload[72], 64U);
}

static BootStatus_t BootMain_ValidateManifest(const BootManifest_t *manifest)
{
    if ((manifest->manifest_magic != BOOT_MANIFEST_MAGIC) ||
        (manifest->format_version != BOOT_MANIFEST_FORMAT_VERSION) ||
        (manifest->header_size != BOOT_MANIFEST_WIRE_SIZE))
    {
        return BOOT_STATUS_BAD_LENGTH;
    }

    if (manifest->product_id != GRIPPER_PRODUCT_ID)
    {
        return BOOT_STATUS_WRONG_PRODUCT;
    }

    if (manifest->role > 2U)
    {
        return BOOT_STATUS_WRONG_ROLE;
    }

    if ((GRIPPER_DEVICE_ROLE != 0U) && (manifest->role != 0U) &&
        (manifest->role != GRIPPER_DEVICE_ROLE))
    {
        return BOOT_STATUS_WRONG_ROLE;
    }

    if ((manifest->flags & ~BOOT_MANIFEST_SUPPORTED_FLAGS) != 0U)
    {
        /* CRC-only prototype: SHA/signature flags are rejected, never ignored. */
        return BOOT_STATUS_UNSUPPORTED;
    }

    if ((manifest->layout_version != GRIPPER_LAYOUT_VERSION) ||
        (manifest->image_base != GRIPPER_APP_BASE))
    {
        return BOOT_STATUS_BAD_OFFSET;
    }

    if (manifest->image_size < 8UL)
    {
        return BOOT_STATUS_BAD_LENGTH;
    }

    if (manifest->image_size > GRIPPER_APP_SIZE)
    {
        return BOOT_STATUS_IMAGE_TOO_LARGE;
    }

    if (manifest->min_boot_version > GRIPPER_BOOT_VERSION_U16)
    {
        return BOOT_STATUS_UNSUPPORTED;
    }

    return BOOT_STATUS_OK;
}

static void BootMain_FillMetadata(BootMetaRecord_t *record,
                                  BootMetaState_t state,
                                  uint32_t session_id,
                                  const BootManifest_t *manifest,
                                  uint32_t next_offset)
{
    (void)memset(record, 0, sizeof(*record));
    record->state = (uint8_t)state;
    record->role = manifest->role;
    record->session_id = session_id;
    record->product_id = manifest->product_id;
    record->layout_version = manifest->layout_version;
    record->image_size = manifest->image_size;
    record->image_crc32 = manifest->image_crc32;
    record->version_major = manifest->version_major;
    record->version_minor = manifest->version_minor;
    record->version_patch = manifest->version_patch;
    record->flags = manifest->flags;
    record->next_offset = next_offset;
}

static void BootMain_MarkInvalid(void)
{
    BootMetaRecord_t invalid;

    if (g_download.active == 0U)
    {
        return;
    }

    BootMain_FillMetadata(&invalid,
                          BOOT_META_STATE_INVALID,
                          g_download.session_id,
                          &g_download.manifest,
                          g_download.next_offset);
    (void)BootMeta_Append(&invalid);
    g_download.active = 0U;
    BootMain_RefreshMetadata();
}

static BootStatus_t BootMain_FlashResultToStatus(BootFlashResult_t result)
{
    switch (result)
    {
        case BOOT_FLASH_OK:
            return BOOT_STATUS_OK;
        case BOOT_FLASH_READBACK_FAILED:
            return BOOT_STATUS_READBACK_FAILED;
        case BOOT_FLASH_PROGRAM_FAILED:
        case BOOT_FLASH_UNLOCK_FAILED:
            return BOOT_STATUS_PROGRAM_FAILED;
        case BOOT_FLASH_ERASE_FAILED:
            return BOOT_STATUS_ERASE_FAILED;
        case BOOT_FLASH_BAD_ARGUMENT:
        default:
            return BOOT_STATUS_INTERNAL_ERROR;
    }
}

static void BootMain_SendHello(const BootFrame_t *request)
{
    uint8_t payload[BOOT_HELLO_PAYLOAD_SIZE] = {0};
    uint32_t capabilities = BOOT_CAP_CRC32_ISO_HDLC |
                            BOOT_CAP_DUPLICATE_DATA_ACK |
                            BOOT_CAP_SESSION_QUERY;

    if (GRIPPER_DEVICE_ROLE != 0U)
    {
        capabilities |= BOOT_CAP_ROLE_LOCKED;
    }

    BootProtocol_PutU32(&payload[0], GRIPPER_PRODUCT_ID);
    BootProtocol_PutU32(&payload[4], GRIPPER_LAYOUT_VERSION);
    BootProtocol_PutU32(&payload[8], GRIPPER_APP_BASE);
    BootProtocol_PutU32(&payload[12], GRIPPER_APP_SIZE);
    BootProtocol_PutU32(&payload[16], GRIPPER_OTA_META_BASE);
    BootProtocol_PutU16(&payload[20], GRIPPER_BOOT_VERSION_U16);
    BootProtocol_PutU16(&payload[22], BOOT_PROTOCOL_MAX_PAYLOAD);
    payload[24] = GRIPPER_PROTOCOL_VERSION;
    payload[25] = GRIPPER_DEVICE_ROLE;
    payload[26] = BootMain_ReportedState();
    payload[27] = 0U;
    BootProtocol_PutU32(&payload[28], capabilities);
    BootProtocol_PutU32(&payload[32], BootMain_ReportedSession());
    BootProtocol_PutU32(&payload[36], BootMain_ReportedNextOffset());

    (void)BootProtocol_SendResponse(request,
                                    BOOT_STATUS_OK,
                                    BootMain_ReportedNextOffset(),
                                    payload,
                                    sizeof(payload));
}

static void BootMain_SendQuery(const BootFrame_t *request)
{
    uint8_t payload[BOOT_QUERY_PAYLOAD_SIZE] = {0};
    uint32_t image_size = 0UL;
    uint32_t image_crc32 = 0UL;

    if (g_download.active != 0U)
    {
        image_size = g_download.manifest.image_size;
        image_crc32 = g_download.manifest.image_crc32;
    }
    else if (g_have_metadata != 0U)
    {
        image_size = g_latest_metadata.image_size;
        image_crc32 = g_latest_metadata.image_crc32;
    }

    payload[0] = BootMain_ReportedState();
    payload[1] = GRIPPER_DEVICE_ROLE;
    BootProtocol_PutU16(&payload[2], GRIPPER_BOOT_VERSION_U16);
    BootProtocol_PutU32(&payload[4], BootMain_ReportedSession());
    BootProtocol_PutU32(&payload[8], BootMain_ReportedNextOffset());
    BootProtocol_PutU32(&payload[12], image_size);
    BootProtocol_PutU32(&payload[16], image_crc32);
    BootProtocol_PutU32(&payload[20],
                        (g_have_metadata != 0U)
                            ? g_latest_metadata.sequence
                            : 0UL);

    (void)BootProtocol_SendResponse(request,
                                    BOOT_STATUS_OK,
                                    BootMain_ReportedNextOffset(),
                                    payload,
                                    sizeof(payload));
}

static void BootMain_HandleBegin(const BootFrame_t *request)
{
    BootManifest_t manifest;
    BootMetaRecord_t downloading;
    BootStatus_t status;
    BootFlashResult_t flash_result;

    if (request->session_id == 0UL)
    {
        (void)BootProtocol_SendResponse(request,
                                        BOOT_STATUS_BAD_SESSION,
                                        BootMain_ReportedNextOffset(),
                                        NULL,
                                        0U);
        return;
    }

    if (request->payload_length != BOOT_MANIFEST_WIRE_SIZE)
    {
        (void)BootProtocol_SendResponse(request,
                                        BOOT_STATUS_BAD_LENGTH,
                                        BootMain_ReportedNextOffset(),
                                        NULL,
                                        0U);
        return;
    }

    BootMain_ParseManifest(request->payload, &manifest);
    status = BootMain_ValidateManifest(&manifest);
    if (status != BOOT_STATUS_OK)
    {
        (void)BootProtocol_SendResponse(request,
                                        status,
                                        BootMain_ReportedNextOffset(),
                                        NULL,
                                        0U);
        return;
    }

    /*
     * OTA takes ownership before it writes DOWNLOADING or erases any part of
     * the Application.  Consuming the ST-Link service authorization here
     * makes a torn OTA unable to fall back to identity/vector-only startup.
     */
    if (BootFlash_FactoryMarkerIsArmed() != 0U)
    {
        flash_result = BootFlash_ConsumeFactoryMarker();
        if (flash_result != BOOT_FLASH_OK)
        {
            (void)BootProtocol_SendResponse(
                request,
                BootMain_FlashResultToStatus(flash_result),
                0UL,
                NULL,
                0U);
            return;
        }
    }

    BootMain_FillMetadata(&downloading,
                          BOOT_META_STATE_DOWNLOADING,
                          request->session_id,
                          &manifest,
                          0UL);
    if (BootMeta_Append(&downloading) == 0U)
    {
        (void)BootProtocol_SendResponse(request,
                                        BOOT_STATUS_META_WRITE_FAILED,
                                        0UL,
                                        NULL,
                                        0U);
        return;
    }

    (void)memset(&g_download, 0, sizeof(g_download));
    g_download.active = 1U;
    g_download.session_id = request->session_id;
    g_download.expected_sequence = (uint16_t)(request->sequence + 1U);
    g_download.next_offset = 0UL;
    g_download.manifest = manifest;
    BootMain_RefreshMetadata();

    flash_result = BootFlash_EraseApplication();
    if (flash_result != BOOT_FLASH_OK)
    {
        status = BootMain_FlashResultToStatus(flash_result);
        BootMain_MarkInvalid();
        (void)BootProtocol_SendResponse(request, status, 0UL, NULL, 0U);
        return;
    }

    (void)BootProtocol_SendResponse(request,
                                    BOOT_STATUS_OK,
                                    0UL,
                                    NULL,
                                    0U);
}

static void BootMain_HandleData(const BootFrame_t *request)
{
    BootFlashResult_t flash_result;
    BootStatus_t status;

    if ((g_download.active == 0U) ||
        (request->session_id != g_download.session_id))
    {
        (void)BootProtocol_SendResponse(request,
                                        BOOT_STATUS_BAD_SESSION,
                                        BootMain_ReportedNextOffset(),
                                        NULL,
                                        0U);
        return;
    }

    if ((request->payload_length != 0U) &&
        (request->offset < g_download.next_offset) &&
        (((uint32_t)request->payload_length + request->offset) <=
         g_download.next_offset))
    {
        status = (BootFlash_ApplicationEquals(request->offset,
                                              request->payload,
                                              request->payload_length,
                                              g_download.manifest.image_size) != 0U)
                     ? BOOT_STATUS_OK
                     : BOOT_STATUS_READBACK_FAILED;
        (void)BootProtocol_SendResponse(request,
                                        status,
                                        g_download.next_offset,
                                        NULL,
                                        0U);
        return;
    }

    if (request->sequence != g_download.expected_sequence)
    {
        (void)BootProtocol_SendResponse(request,
                                        BOOT_STATUS_BAD_SEQUENCE,
                                        g_download.next_offset,
                                        NULL,
                                        0U);
        return;
    }

    if (request->offset != g_download.next_offset)
    {
        (void)BootProtocol_SendResponse(request,
                                        BOOT_STATUS_BAD_OFFSET,
                                        g_download.next_offset,
                                        NULL,
                                        0U);
        return;
    }

    if ((request->payload_length == 0U) ||
        ((uint32_t)request->payload_length >
         (g_download.manifest.image_size - g_download.next_offset)))
    {
        (void)BootProtocol_SendResponse(request,
                                        BOOT_STATUS_BAD_LENGTH,
                                        g_download.next_offset,
                                        NULL,
                                        0U);
        return;
    }

    if (((request->offset % GRIPPER_FLASH_PROGRAM_SIZE) != 0UL) ||
        ((((uint32_t)request->payload_length + request->offset) !=
          g_download.manifest.image_size) &&
         (((uint32_t)request->payload_length %
           GRIPPER_FLASH_PROGRAM_SIZE) != 0UL)))
    {
        (void)BootProtocol_SendResponse(request,
                                        BOOT_STATUS_BAD_LENGTH,
                                        g_download.next_offset,
                                        NULL,
                                        0U);
        return;
    }

    flash_result = BootFlash_WriteApplication(request->offset,
                                              request->payload,
                                              request->payload_length,
                                              g_download.manifest.image_size);
    status = BootMain_FlashResultToStatus(flash_result);
    if (status == BOOT_STATUS_OK)
    {
        g_download.next_offset += request->payload_length;
        ++g_download.expected_sequence;
    }

    (void)BootProtocol_SendResponse(request,
                                    status,
                                    g_download.next_offset,
                                    NULL,
                                    0U);
}

static void BootMain_HandleEnd(const BootFrame_t *request)
{
    BootMetaRecord_t valid;
    uint32_t calculated_crc;
    BootStatus_t status = BOOT_STATUS_OK;

    if ((g_download.active == 0U) ||
        (request->session_id != g_download.session_id))
    {
        status = BOOT_STATUS_BAD_SESSION;
    }
    else if (request->sequence != g_download.expected_sequence)
    {
        status = BOOT_STATUS_BAD_SEQUENCE;
    }
    else if ((request->payload_length != 0U) ||
             (request->offset != g_download.next_offset) ||
             (g_download.next_offset != g_download.manifest.image_size))
    {
        status = BOOT_STATUS_BAD_LENGTH;
    }
    else if (BootImage_VectorIsValid(g_download.manifest.image_size) == 0U)
    {
        status = BOOT_STATUS_INVALID_VECTOR;
    }
    else
    {
        calculated_crc = BootImage_CalculateCrc32(
            g_download.manifest.image_size);
        if (calculated_crc != g_download.manifest.image_crc32)
        {
            status = BOOT_STATUS_IMAGE_CRC_FAILED;
        }
    }

    if (status != BOOT_STATUS_OK)
    {
        if ((g_download.active != 0U) &&
            ((status == BOOT_STATUS_INVALID_VECTOR) ||
             (status == BOOT_STATUS_IMAGE_CRC_FAILED)))
        {
            BootMain_MarkInvalid();
        }
        (void)BootProtocol_SendResponse(request,
                                        status,
                                        BootMain_ReportedNextOffset(),
                                        NULL,
                                        0U);
        return;
    }

    BootMain_FillMetadata(&valid,
                          BOOT_META_STATE_VALID,
                          g_download.session_id,
                          &g_download.manifest,
                          g_download.next_offset);
    if (BootMeta_Append(&valid) == 0U)
    {
        (void)BootProtocol_SendResponse(request,
                                        BOOT_STATUS_META_WRITE_FAILED,
                                        g_download.next_offset,
                                        NULL,
                                        0U);
        return;
    }

    g_download.active = 0U;
    BootMain_RefreshMetadata();
    (void)BootProtocol_SendResponse(request,
                                    BOOT_STATUS_OK,
                                    valid.image_size,
                                    NULL,
                                    0U);
}

static void BootMain_HandleAbort(const BootFrame_t *request)
{
    if ((g_download.active == 0U) ||
        (request->session_id != g_download.session_id))
    {
        (void)BootProtocol_SendResponse(request,
                                        BOOT_STATUS_BAD_SESSION,
                                        BootMain_ReportedNextOffset(),
                                        NULL,
                                        0U);
        return;
    }

    BootMain_MarkInvalid();
    (void)BootProtocol_SendResponse(request,
                                    BOOT_STATUS_OK,
                                    0UL,
                                    NULL,
                                    0U);
}

static void BootMain_HandleReboot(const BootFrame_t *request)
{
    BootMain_RefreshMetadata();
    if ((g_have_metadata == 0U) ||
        (BootImage_MatchesMetadata(&g_latest_metadata) == 0U))
    {
        (void)BootProtocol_SendResponse(request,
                                        BOOT_STATUS_APP_NOT_VALID,
                                        BootMain_ReportedNextOffset(),
                                        NULL,
                                        0U);
        return;
    }

    (void)BootProtocol_SendResponse(request,
                                    BOOT_STATUS_OK,
                                    g_latest_metadata.image_size,
                                    NULL,
                                    0U);
    HAL_Delay(BOOT_REBOOT_DELAY_MS);
    NVIC_SystemReset();
}

static void BootMain_HandleFrame(const BootFrame_t *request)
{
    if (request->protocol_version != GRIPPER_PROTOCOL_VERSION)
    {
        (void)BootProtocol_SendResponse(request,
                                        BOOT_STATUS_BAD_PROTOCOL_VERSION,
                                        BootMain_ReportedNextOffset(),
                                        NULL,
                                        0U);
        return;
    }

    if ((request->command & BOOT_COMMAND_RESPONSE_MASK) != 0U)
    {
        return;
    }

    switch (request->command)
    {
        case BOOT_COMMAND_HELLO:
            BootMain_SendHello(request);
            break;

        case BOOT_COMMAND_BEGIN:
            BootMain_HandleBegin(request);
            break;

        case BOOT_COMMAND_DATA:
            BootMain_HandleData(request);
            break;

        case BOOT_COMMAND_END:
            BootMain_HandleEnd(request);
            break;

        case BOOT_COMMAND_QUERY:
            BootMain_SendQuery(request);
            break;

        case BOOT_COMMAND_ABORT:
            BootMain_HandleAbort(request);
            break;

        case BOOT_COMMAND_REBOOT:
            BootMain_HandleReboot(request);
            break;

        default:
            (void)BootProtocol_SendResponse(request,
                                            BOOT_STATUS_UNSUPPORTED,
                                            BootMain_ReportedNextOffset(),
                                            NULL,
                                            0U);
            break;
    }
}

static uint8_t BootMain_PollOnce(uint32_t timeout_ms)
{
    BootFrame_t request;
    BootReceiveResult_t receive_result;

    receive_result = BootProtocol_Receive(&request, timeout_ms);
    switch (receive_result)
    {
        case BOOT_RX_FRAME:
            BootMain_HandleFrame(&request);
            return 1U;

        case BOOT_RX_BAD_CRC:
            (void)BootProtocol_SendResponse(&request,
                                            BOOT_STATUS_BAD_FRAME_CRC,
                                            BootMain_ReportedNextOffset(),
                                            NULL,
                                            0U);
            break;

        case BOOT_RX_BAD_LENGTH:
            /* The declared tail was not consumed; do not drive RS485 against it. */
            break;

        case BOOT_RX_TIMEOUT:
        case BOOT_RX_IO_ERROR:
        default:
            break;
    }

    return 0U;
}

void Bootloader_Run(void)
{
    uint32_t boot_start_tick;
    uint32_t mailbox_reason = 0UL;
    uint8_t mailbox_requested;
    uint8_t application_valid;
    uint8_t factory_application_valid;
    BootMetaRecord_t invalid;

    HAL_GPIO_WritePin(RS485_DE_GPIO_Port, RS485_DE_Pin, GPIO_PIN_RESET);
    (void)memset(&g_download, 0, sizeof(g_download));
    g_mailbox_session = 0UL;

    mailbox_requested = BootMailbox_TakeRequest(&g_mailbox_session,
                                                &mailbox_reason);
    (void)mailbox_reason;
    BootMain_RefreshMetadata();

    application_valid = ((g_have_metadata != 0U) &&
                         (BootImage_MatchesMetadata(&g_latest_metadata) != 0U))
                            ? 1U
                            : 0U;

    /*
     * Bootloader reprogramming arms an explicit ST-Link service authorization.
     * While it is armed, a relocated Application with the expected identity
     * and vector table may start even if an older OTA Metadata record remains.
     * BEGIN consumes the authorization before writing Metadata or erasing the
     * Application, so an interrupted OTA can never use this compatibility path.
     */
    factory_application_valid = BootMain_FactoryApplicationIsValid();

    /* Persist a detected corruption once, so subsequent boots report INVALID. */
    if ((g_have_metadata != 0U) &&
        (g_latest_metadata.state == BOOT_META_STATE_VALID) &&
        (BootFlash_FactoryMarkerIsArmed() == 0U) &&
        (application_valid == 0U))
    {
        invalid = g_latest_metadata;
        invalid.state = BOOT_META_STATE_INVALID;
        invalid.next_offset = 0UL;
        (void)BootMeta_Append(&invalid);
        BootMain_RefreshMetadata();
    }

    if ((mailbox_requested == 0U) &&
        ((application_valid != 0U) ||
         (factory_application_valid != 0U)))
    {
        boot_start_tick = HAL_GetTick();
        while ((HAL_GetTick() - boot_start_tick) < GRIPPER_BOOT_HANDSHAKE_MS)
        {
            if (BootMain_PollOnce(BOOT_POLL_INTERVAL_MS) != 0U)
            {
                /* A CRC-valid Boot protocol frame captures the handshake window. */
                while (1)
                {
                    (void)BootMain_PollOnce(BOOT_POLL_INTERVAL_MS);
                }
            }
        }

        BootJump_ToApplication();
    }

    while (1)
    {
        (void)BootMain_PollOnce(BOOT_POLL_INTERVAL_MS);
    }
}
