#ifndef GRIPPER_OTA_LAYOUT_H
#define GRIPPER_OTA_LAYOUT_H

#include <stdint.h>

/*
 * STM32G431CBT6 single-slot OTA layout.
 * Keep these values identical to Bootloader_G431/Device/Boot/gripper_ota_layout.h.
 */
#define GRIPPER_FLASH_BASE              (0x08000000UL)
#define GRIPPER_FLASH_END               (0x08020000UL)

#define GRIPPER_BOOT_BASE               (0x08000000UL)
#define GRIPPER_BOOT_SIZE               (0x00006000UL)
#define GRIPPER_BOOT_CODE_SIZE          (0x00005800UL)
#define GRIPPER_FACTORY_MARKER_BASE     (0x08005800UL)
#define GRIPPER_FACTORY_MARKER_SIZE     (0x00000800UL)

#define GRIPPER_APP_BASE                (0x08006000UL)
#define GRIPPER_APP_SIZE                (0x00019000UL)
#define GRIPPER_APP_END                 (GRIPPER_APP_BASE + GRIPPER_APP_SIZE)

#define GRIPPER_OTA_META_BASE           (0x0801F000UL)
#define GRIPPER_OTA_META_SIZE           (0x00000800UL)

#define GRIPPER_CALIB_BASE              (0x0801F800UL)
#define GRIPPER_CALIB_SIZE              (0x00000800UL)

#define GRIPPER_SRAM_BASE               (0x20000000UL)
#define GRIPPER_SRAM_SIZE               (0x00008000UL)
#define GRIPPER_BOOT_MAILBOX_BASE       (0x20007FE0UL)
#define GRIPPER_BOOT_MAILBOX_SIZE       (0x00000020UL)

#define GRIPPER_PRODUCT_ID              (0x0000D431UL)
#define GRIPPER_LAYOUT_VERSION          (1UL)

/* Compiled into OTA-aware Applications and required by factory fallback. */
#define GRIPPER_APP_IDENTITY_MAGIC      (0x50415247UL) /* bytes: GRAP */
#define GRIPPER_APP_IDENTITY_TAIL       (0x21444947UL) /* bytes: GID! */

#if ((GRIPPER_BOOT_BASE + GRIPPER_BOOT_SIZE) != GRIPPER_APP_BASE)
#error "Bootloader and Application layout is not contiguous"
#endif

#if ((GRIPPER_BOOT_BASE + GRIPPER_BOOT_CODE_SIZE) != GRIPPER_FACTORY_MARKER_BASE) || \
    ((GRIPPER_FACTORY_MARKER_BASE + GRIPPER_FACTORY_MARKER_SIZE) != GRIPPER_APP_BASE)
#error "Bootloader code/factory marker layout is invalid"
#endif

#if (GRIPPER_APP_END != GRIPPER_OTA_META_BASE)
#error "Application and Metadata layout is not contiguous"
#endif

#if ((GRIPPER_OTA_META_BASE + GRIPPER_OTA_META_SIZE) != GRIPPER_CALIB_BASE)
#error "Metadata and Calibration layout is not contiguous"
#endif

#if ((GRIPPER_CALIB_BASE + GRIPPER_CALIB_SIZE) != GRIPPER_FLASH_END)
#error "Flash layout does not end at the device Flash boundary"
#endif

#endif /* GRIPPER_OTA_LAYOUT_H */
