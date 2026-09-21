#include "boot_image_verify.h"

#include <stddef.h>

#include "boot_crc.h"
#include "gripper_ota_layout.h"

uint8_t BootImage_VectorIsValid(uint32_t image_size)
{
    uint32_t initial_msp = *(const uint32_t *)GRIPPER_APP_BASE;
    uint32_t reset_handler = *(const uint32_t *)(GRIPPER_APP_BASE + 4UL);
    uint32_t reset_address = reset_handler & ~1UL;
    uint32_t usable_sram_end = GRIPPER_BOOT_MAILBOX_BASE;

    if ((image_size < 8UL) || (image_size > GRIPPER_APP_SIZE))
    {
        return 0U;
    }

    if ((initial_msp < GRIPPER_SRAM_BASE) ||
        (initial_msp > usable_sram_end) ||
        ((initial_msp & 0x7UL) != 0UL))
    {
        return 0U;
    }

    if (((reset_handler & 1UL) == 0UL) ||
        (reset_address < GRIPPER_APP_BASE) ||
        (reset_address >= (GRIPPER_APP_BASE + image_size)))
    {
        return 0U;
    }

    return 1U;
}

uint8_t BootImage_HasFactoryIdentity(void)
{
    const uint32_t *cursor = (const uint32_t *)GRIPPER_APP_BASE;
    const uint32_t *end =
        (const uint32_t *)(GRIPPER_APP_END - (5UL * sizeof(uint32_t)));

    while (cursor <= end)
    {
        if ((cursor[0] == GRIPPER_APP_IDENTITY_MAGIC) &&
            (cursor[1] == GRIPPER_PRODUCT_ID) &&
            (cursor[2] == GRIPPER_LAYOUT_VERSION) &&
            (cursor[3] == GRIPPER_APP_BASE) &&
            (cursor[4] == GRIPPER_APP_IDENTITY_TAIL))
        {
            return 1U;
        }
        ++cursor;
    }

    return 0U;
}

uint32_t BootImage_CalculateCrc32(uint32_t image_size)
{
    if ((image_size == 0UL) || (image_size > GRIPPER_APP_SIZE))
    {
        return 0UL;
    }

    return BootCrc32_Calculate((const uint8_t *)GRIPPER_APP_BASE, image_size);
}

uint8_t BootImage_MatchesMetadata(const BootMetaRecord_t *record)
{
    if ((record == NULL) || (BootMeta_RecordIsValid(record) == 0U) ||
        (record->state != BOOT_META_STATE_VALID) ||
        (record->product_id != GRIPPER_PRODUCT_ID) ||
        (record->layout_version != GRIPPER_LAYOUT_VERSION) ||
        (record->image_size == 0UL) ||
        (record->image_size > GRIPPER_APP_SIZE))
    {
        return 0U;
    }

    if ((GRIPPER_DEVICE_ROLE != 0U) && (record->role != 0U) &&
        (record->role != GRIPPER_DEVICE_ROLE))
    {
        return 0U;
    }

    if (BootImage_VectorIsValid(record->image_size) == 0U)
    {
        return 0U;
    }

    return (BootImage_CalculateCrc32(record->image_size) ==
            record->image_crc32)
               ? 1U
               : 0U;
}
