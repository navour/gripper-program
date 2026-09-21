#ifndef BOOT_METADATA_H
#define BOOT_METADATA_H

#include <stdint.h>

typedef enum
{
    BOOT_META_STATE_EMPTY = 0,
    BOOT_META_STATE_DOWNLOADING = 1,
    BOOT_META_STATE_VALID = 2,
    BOOT_META_STATE_INVALID = 3
} BootMetaState_t;

typedef struct
{
    uint32_t magic;
    uint16_t record_version;
    uint8_t state;
    uint8_t role;
    uint32_t sequence;
    uint32_t session_id;
    uint32_t product_id;
    uint32_t layout_version;
    uint32_t image_size;
    uint32_t image_crc32;
    uint16_t version_major;
    uint16_t version_minor;
    uint16_t version_patch;
    uint16_t flags;
    uint32_t next_offset;
    uint32_t reserved[3];
    uint32_t record_crc32;
    uint32_t commit_marker;
} BootMetaRecord_t;

uint8_t BootMeta_GetLatest(BootMetaRecord_t *record);
uint8_t BootMeta_Append(BootMetaRecord_t *record);
uint8_t BootMeta_RecordIsValid(const BootMetaRecord_t *record);
uint8_t BootMeta_HasRecognizedRecord(void);

#endif /* BOOT_METADATA_H */
