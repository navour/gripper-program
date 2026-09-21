#ifndef BOOT_IMAGE_VERIFY_H
#define BOOT_IMAGE_VERIFY_H

#include <stdint.h>

#include "boot_metadata.h"

uint8_t BootImage_VectorIsValid(uint32_t image_size);
uint8_t BootImage_HasFactoryIdentity(void);
uint32_t BootImage_CalculateCrc32(uint32_t image_size);
uint8_t BootImage_MatchesMetadata(const BootMetaRecord_t *record);

#endif /* BOOT_IMAGE_VERIFY_H */
