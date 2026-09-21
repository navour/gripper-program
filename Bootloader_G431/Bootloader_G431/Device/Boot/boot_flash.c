#include "boot_flash.h"

#include <string.h>

#include "stm32g4xx_hal.h"
#include "gripper_ota_layout.h"

/*
 * Keil programs this doubleword in the final Bootloader page.  It authorizes
 * direct ST-Link Application installation until the next OTA BEGIN consumes
 * it without erasing the page.  Therefore a torn OTA can never fall back to
 * identity/vector-only startup, while an explicit Bootloader reflash re-arms
 * the maintenance path even when older Metadata remains in Flash.
 */
#if defined(__CC_ARM)
__attribute__((section("FactoryMarker"), used, aligned(8)))
#endif
const uint32_t g_boot_factory_marker[2] =
{
    GRIPPER_FACTORY_MARKER_MAGIC,
    GRIPPER_FACTORY_MARKER_MAGIC_INV
};

uint8_t BootFlash_FactoryMarkerIsArmed(void)
{
    const uint32_t *marker =
        (const uint32_t *)GRIPPER_FACTORY_MARKER_BASE;

    return ((marker[0] == GRIPPER_FACTORY_MARKER_MAGIC) &&
            (marker[1] == GRIPPER_FACTORY_MARKER_MAGIC_INV))
               ? 1U
               : 0U;
}

BootFlashResult_t BootFlash_ConsumeFactoryMarker(void)
{
    HAL_StatusTypeDef status;

    if (BootFlash_FactoryMarkerIsArmed() == 0U)
    {
        return BOOT_FLASH_BAD_ARGUMENT;
    }
    if (HAL_FLASH_Unlock() != HAL_OK)
    {
        return BOOT_FLASH_UNLOCK_FAILED;
    }

    __HAL_FLASH_CLEAR_FLAG(FLASH_FLAG_ALL_ERRORS);
    status = HAL_FLASH_Program(FLASH_TYPEPROGRAM_DOUBLEWORD,
                               GRIPPER_FACTORY_MARKER_BASE,
                               0ULL);
    (void)HAL_FLASH_Lock();

    if (status != HAL_OK)
    {
        return BOOT_FLASH_PROGRAM_FAILED;
    }
    return ((*(const uint64_t *)GRIPPER_FACTORY_MARKER_BASE) == 0ULL)
               ? BOOT_FLASH_OK
               : BOOT_FLASH_READBACK_FAILED;
}

static uint8_t BootFlash_RangeIsInside(uint32_t address,
                                       uint32_t length,
                                       uint32_t region_start,
                                       uint32_t region_size)
{
    uint32_t region_end = region_start + region_size;

    if ((length == 0UL) || (address < region_start) || (address >= region_end))
    {
        return 0U;
    }

    if ((address + length) < address)
    {
        return 0U;
    }

    return ((address + length) <= region_end) ? 1U : 0U;
}

static BootFlashResult_t BootFlash_ErasePages(uint32_t first_address,
                                              uint32_t region_size)
{
    FLASH_EraseInitTypeDef erase = {0};
    uint32_t page_error = 0xFFFFFFFFUL;
    HAL_StatusTypeDef status;

    if (((first_address % GRIPPER_FLASH_PAGE_SIZE) != 0UL) ||
        ((region_size % GRIPPER_FLASH_PAGE_SIZE) != 0UL) ||
        (region_size == 0UL))
    {
        return BOOT_FLASH_BAD_ARGUMENT;
    }

    erase.TypeErase = FLASH_TYPEERASE_PAGES;
    erase.Banks = FLASH_BANK_1;
    erase.Page = (first_address - GRIPPER_FLASH_BASE) / GRIPPER_FLASH_PAGE_SIZE;
    erase.NbPages = region_size / GRIPPER_FLASH_PAGE_SIZE;

    if (HAL_FLASH_Unlock() != HAL_OK)
    {
        return BOOT_FLASH_UNLOCK_FAILED;
    }

    __HAL_FLASH_CLEAR_FLAG(FLASH_FLAG_ALL_ERRORS);
    status = HAL_FLASHEx_Erase(&erase, &page_error);
    (void)HAL_FLASH_Lock();

    if ((status != HAL_OK) || (page_error != 0xFFFFFFFFUL))
    {
        return BOOT_FLASH_ERASE_FAILED;
    }

    return BOOT_FLASH_OK;
}

static BootFlashResult_t BootFlash_ProgramRange(uint32_t address,
                                                const uint8_t *data,
                                                uint32_t length,
                                                uint32_t region_start,
                                                uint32_t region_size,
                                                uint8_t allow_tail_padding)
{
    uint32_t source_index = 0UL;
    uint32_t write_length;
    uint64_t doubleword;
    uint8_t bytes[GRIPPER_FLASH_PROGRAM_SIZE];
    uint32_t copy_length;
    uint32_t index;
    HAL_StatusTypeDef status = HAL_OK;

    if ((data == NULL) || ((address % GRIPPER_FLASH_PROGRAM_SIZE) != 0UL))
    {
        return BOOT_FLASH_BAD_ARGUMENT;
    }

    write_length = (length + (GRIPPER_FLASH_PROGRAM_SIZE - 1UL)) &
                   ~(GRIPPER_FLASH_PROGRAM_SIZE - 1UL);
    if ((write_length != length) && (allow_tail_padding == 0U))
    {
        return BOOT_FLASH_BAD_ARGUMENT;
    }

    if (BootFlash_RangeIsInside(address,
                                write_length,
                                region_start,
                                region_size) == 0U)
    {
        return BOOT_FLASH_BAD_ARGUMENT;
    }

    if (HAL_FLASH_Unlock() != HAL_OK)
    {
        return BOOT_FLASH_UNLOCK_FAILED;
    }

    __HAL_FLASH_CLEAR_FLAG(FLASH_FLAG_ALL_ERRORS);

    while (source_index < length)
    {
        copy_length = length - source_index;
        if (copy_length > GRIPPER_FLASH_PROGRAM_SIZE)
        {
            copy_length = GRIPPER_FLASH_PROGRAM_SIZE;
        }

        for (index = 0UL; index < GRIPPER_FLASH_PROGRAM_SIZE; ++index)
        {
            bytes[index] = 0xFFU;
        }
        (void)memcpy(bytes, &data[source_index], copy_length);
        (void)memcpy(&doubleword, bytes, sizeof(doubleword));

        status = HAL_FLASH_Program(FLASH_TYPEPROGRAM_DOUBLEWORD,
                                   address + source_index,
                                   doubleword);
        if (status != HAL_OK)
        {
            break;
        }

        source_index += GRIPPER_FLASH_PROGRAM_SIZE;
    }

    (void)HAL_FLASH_Lock();

    if (status != HAL_OK)
    {
        return BOOT_FLASH_PROGRAM_FAILED;
    }

    if (memcmp((const void *)address, data, length) != 0)
    {
        return BOOT_FLASH_READBACK_FAILED;
    }

    return BOOT_FLASH_OK;
}

BootFlashResult_t BootFlash_EraseApplication(void)
{
    return BootFlash_ErasePages(GRIPPER_APP_BASE, GRIPPER_APP_SIZE);
}

BootFlashResult_t BootFlash_EraseMetadata(void)
{
    return BootFlash_ErasePages(GRIPPER_OTA_META_BASE, GRIPPER_OTA_META_SIZE);
}

BootFlashResult_t BootFlash_WriteApplication(uint32_t offset,
                                             const uint8_t *data,
                                             uint16_t length,
                                             uint32_t image_size)
{
    uint8_t is_final_chunk;

    if ((image_size == 0UL) || (image_size > GRIPPER_APP_SIZE) ||
        (offset >= image_size) || ((uint32_t)length > (image_size - offset)))
    {
        return BOOT_FLASH_BAD_ARGUMENT;
    }

    is_final_chunk = (((uint32_t)length + offset) == image_size) ? 1U : 0U;

    if (((offset % GRIPPER_FLASH_PROGRAM_SIZE) != 0UL) ||
        ((is_final_chunk == 0U) &&
         (((uint32_t)length % GRIPPER_FLASH_PROGRAM_SIZE) != 0UL)))
    {
        return BOOT_FLASH_BAD_ARGUMENT;
    }

    return BootFlash_ProgramRange(GRIPPER_APP_BASE + offset,
                                  data,
                                  length,
                                  GRIPPER_APP_BASE,
                                  GRIPPER_APP_SIZE,
                                  is_final_chunk);
}

uint8_t BootFlash_ApplicationEquals(uint32_t offset,
                                    const uint8_t *data,
                                    uint16_t length,
                                    uint32_t image_size)
{
    if ((data == NULL) || (image_size == 0UL) ||
        (image_size > GRIPPER_APP_SIZE) || (offset >= image_size) ||
        ((uint32_t)length > (image_size - offset)))
    {
        return 0U;
    }

    return (memcmp((const void *)(GRIPPER_APP_BASE + offset), data, length) == 0)
               ? 1U
               : 0U;
}

BootFlashResult_t BootFlash_ProgramMetadata(uint32_t address,
                                            const uint8_t *data,
                                            uint32_t length)
{
    return BootFlash_ProgramRange(address,
                                  data,
                                  length,
                                  GRIPPER_OTA_META_BASE,
                                  GRIPPER_OTA_META_SIZE,
                                  0U);
}
