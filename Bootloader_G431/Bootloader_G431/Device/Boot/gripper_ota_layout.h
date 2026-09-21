#ifndef GRIPPER_OTA_LAYOUT_H
#define GRIPPER_OTA_LAYOUT_H

#include <stdint.h>

/* STM32G431CBT6: 128 KiB Flash and 32 KiB SRAM. */
#define GRIPPER_FLASH_BASE              (0x08000000UL)
#define GRIPPER_FLASH_END               (0x08020000UL)

#define GRIPPER_BOOT_BASE               (0x08000000UL)
#define GRIPPER_BOOT_SIZE               (0x00006000UL)
#define GRIPPER_BOOT_CODE_SIZE          (0x00005800UL)
#define GRIPPER_FACTORY_MARKER_BASE     (0x08005800UL)
#define GRIPPER_FACTORY_MARKER_SIZE     (0x00000800UL)
#define GRIPPER_FACTORY_MARKER_MAGIC    (0x4B4C5453UL) /* bytes: STLK */
#define GRIPPER_FACTORY_MARKER_MAGIC_INV (~GRIPPER_FACTORY_MARKER_MAGIC)

#define GRIPPER_APP_BASE                (0x08006000UL)
#define GRIPPER_APP_SIZE                (0x00019000UL)
#define GRIPPER_APP_END                 (GRIPPER_APP_BASE + GRIPPER_APP_SIZE)

#define GRIPPER_OTA_META_BASE           (0x0801F000UL)
#define GRIPPER_OTA_META_SIZE           (0x00000800UL)

#define GRIPPER_CALIB_BASE              (0x0801F800UL)
#define GRIPPER_CALIB_SIZE              (0x00000800UL)

#define GRIPPER_FLASH_PAGE_SIZE         (0x00000800UL)
#define GRIPPER_FLASH_PROGRAM_SIZE      (8UL)

#define GRIPPER_SRAM_BASE               (0x20000000UL)
#define GRIPPER_SRAM_SIZE               (0x00008000UL)
#define GRIPPER_BOOT_MAILBOX_BASE       (0x20007FE0UL)
#define GRIPPER_BOOT_MAILBOX_SIZE       (0x00000020UL)

#define GRIPPER_PRODUCT_ID              (0x0000D431UL)
#define GRIPPER_LAYOUT_VERSION          (1UL)

/* Compiled into OTA-aware Applications and required by factory fallback. */
#define GRIPPER_APP_IDENTITY_MAGIC      (0x50415247UL) /* bytes: GRAP */
#define GRIPPER_APP_IDENTITY_TAIL       (0x21444947UL) /* bytes: GID! */

/*
 * Development default: one common Bootloader accepts role 0/1/2 images.
 * For a role-locked production image, define GRIPPER_DEVICE_ROLE as 1 or 2
 * in the Keil target C preprocessor symbols (or replace this factory hook).
 */
#ifndef GRIPPER_DEVICE_ROLE
#define GRIPPER_DEVICE_ROLE             (0U)
#endif

#define GRIPPER_BOOT_VERSION_MAJOR      (1U)
#define GRIPPER_BOOT_VERSION_MINOR      (0U)
#define GRIPPER_BOOT_VERSION_U16        ((GRIPPER_BOOT_VERSION_MAJOR << 8) | \
                                         GRIPPER_BOOT_VERSION_MINOR)

#define GRIPPER_PROTOCOL_VERSION        (1U)
#define GRIPPER_BOOT_HANDSHAKE_MS       (500UL)

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

#if ((GRIPPER_BOOT_BASE % GRIPPER_FLASH_PAGE_SIZE) != 0UL) || \
    ((GRIPPER_APP_BASE % GRIPPER_FLASH_PAGE_SIZE) != 0UL) || \
    ((GRIPPER_OTA_META_BASE % GRIPPER_FLASH_PAGE_SIZE) != 0UL) || \
    ((GRIPPER_CALIB_BASE % GRIPPER_FLASH_PAGE_SIZE) != 0UL)
#error "All Flash partitions must be page aligned"
#endif

#if ((GRIPPER_DEVICE_ROLE != 0U) && (GRIPPER_DEVICE_ROLE != 1U) && \
     (GRIPPER_DEVICE_ROLE != 2U))
#error "GRIPPER_DEVICE_ROLE must be 0 (development), 1, or 2"
#endif

#endif /* GRIPPER_OTA_LAYOUT_H */
