#ifndef BOOT_FLASH_H
#define BOOT_FLASH_H

#include <stdint.h>

typedef enum
{
    BOOT_FLASH_OK = 0,
    BOOT_FLASH_BAD_ARGUMENT,
    BOOT_FLASH_UNLOCK_FAILED,
    BOOT_FLASH_ERASE_FAILED,
    BOOT_FLASH_PROGRAM_FAILED,
    BOOT_FLASH_READBACK_FAILED
} BootFlashResult_t;

BootFlashResult_t BootFlash_EraseApplication(void);
BootFlashResult_t BootFlash_EraseMetadata(void);
uint8_t BootFlash_FactoryMarkerIsArmed(void);
BootFlashResult_t BootFlash_ConsumeFactoryMarker(void);

BootFlashResult_t BootFlash_WriteApplication(uint32_t offset,
                                             const uint8_t *data,
                                             uint16_t length,
                                             uint32_t image_size);

uint8_t BootFlash_ApplicationEquals(uint32_t offset,
                                    const uint8_t *data,
                                    uint16_t length,
                                    uint32_t image_size);

BootFlashResult_t BootFlash_ProgramMetadata(uint32_t address,
                                            const uint8_t *data,
                                            uint32_t length);

#endif /* BOOT_FLASH_H */
