#include "gripper_ota_app.h"

#include "gripper_ota_layout.h"
#include "stm32g4xx_hal.h"

#define GRIPPER_BOOT_MAILBOX_MAGIC       (0x544F4F42UL) /* bytes: BOOT */

typedef struct
{
    uint32_t magic;
    uint32_t session_id;
    uint32_t reason;
    uint32_t magic_inv;
    uint32_t crc32;
    uint32_t reserved[3];
} GripperBootMailbox_t;

typedef char GripperBootMailbox_SizeMustBe32Bytes[
    (sizeof(GripperBootMailbox_t) == GRIPPER_BOOT_MAILBOX_SIZE) ? 1 : -1];

/*
 * A relocated, OTA-aware Application carries this identity even when it was
 * written directly by Keil/ST-Link and therefore has no OTA metadata yet.
 * The Bootloader scans for the full five-word signature before allowing its
 * narrowly scoped first-deployment vector fallback.
 */
#if defined(__CC_ARM)
__attribute__((used))
#endif
const uint32_t g_gripper_app_identity[5] =
{
    GRIPPER_APP_IDENTITY_MAGIC,
    GRIPPER_PRODUCT_ID,
    GRIPPER_LAYOUT_VERSION,
    GRIPPER_APP_BASE,
    GRIPPER_APP_IDENTITY_TAIL
};

static uint32_t GripperOtaApp_Crc32(const uint8_t *data, uint32_t length)
{
    uint32_t crc = 0xFFFFFFFFUL;
    uint32_t index;
    uint8_t bit;

    for (index = 0UL; index < length; ++index)
    {
        crc ^= data[index];
        for (bit = 0U; bit < 8U; ++bit)
        {
            if ((crc & 1UL) != 0UL)
            {
                crc = (crc >> 1U) ^ 0xEDB88320UL;
            }
            else
            {
                crc >>= 1U;
            }
        }
    }

    return crc ^ 0xFFFFFFFFUL;
}

uint32_t GripperOtaApp_CreateSessionId(uint32_t entropy)
{
    uint32_t session_id;

    session_id = HAL_GetUIDw0() ^ HAL_GetUIDw1() ^ HAL_GetUIDw2() ^ entropy;
    session_id ^= (session_id << 13U);
    session_id ^= (session_id >> 17U);
    session_id ^= (session_id << 5U);

    return (session_id != 0UL) ? session_id : 1UL;
}

void GripperOtaApp_ResetToBootloader(uint32_t session_id, uint32_t reason)
{
    volatile GripperBootMailbox_t *mailbox =
        (volatile GripperBootMailbox_t *)GRIPPER_BOOT_MAILBOX_BASE;
    GripperBootMailbox_t request;

    request.magic = GRIPPER_BOOT_MAILBOX_MAGIC;
    request.session_id = (session_id != 0UL) ? session_id : 1UL;
    request.reason = reason;
    request.magic_inv = ~GRIPPER_BOOT_MAILBOX_MAGIC;
    request.crc32 = GripperOtaApp_Crc32((const uint8_t *)&request, 16UL);
    request.reserved[0] = 0UL;
    request.reserved[1] = 0UL;
    request.reserved[2] = 0UL;

    __disable_irq();

    /* Invalidate first, then publish a complete request with magic written last. */
    mailbox->magic = 0UL;
    __DSB();
    mailbox->session_id = request.session_id;
    mailbox->reason = request.reason;
    mailbox->magic_inv = request.magic_inv;
    mailbox->crc32 = request.crc32;
    mailbox->reserved[0] = request.reserved[0];
    mailbox->reserved[1] = request.reserved[1];
    mailbox->reserved[2] = request.reserved[2];
    __DSB();
    mailbox->magic = request.magic;
    __DSB();
    __ISB();

    NVIC_SystemReset();
    while (1)
    {
        /* NVIC_SystemReset() must not return. */
    }
}
