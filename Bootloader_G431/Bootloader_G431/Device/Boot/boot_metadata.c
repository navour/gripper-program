#include "boot_metadata.h"

#include <stddef.h>
#include <string.h>

#include "boot_crc.h"
#include "boot_flash.h"
#include "gripper_ota_layout.h"

#define BOOT_META_MAGIC                 (0x4154454DUL) /* bytes: META */
#define BOOT_META_COMMIT_MARKER         (0xA55A3CC3UL)
#define BOOT_META_RECORD_VERSION        (1U)
#define BOOT_META_RECORD_SIZE           (64UL)
#define BOOT_META_RECORD_COUNT          (GRIPPER_OTA_META_SIZE / BOOT_META_RECORD_SIZE)
#define BOOT_META_CRC_LENGTH            (56UL)

typedef char BootMeta_RecordSizeMustBe64Bytes[
    (sizeof(BootMetaRecord_t) == BOOT_META_RECORD_SIZE) ? 1 : -1];
typedef char BootMeta_CrcOffsetMustBe56[
    (offsetof(BootMetaRecord_t, record_crc32) == BOOT_META_CRC_LENGTH) ? 1 : -1];

static uint32_t BootMeta_RecordAddress(uint32_t index)
{
    return GRIPPER_OTA_META_BASE + (index * BOOT_META_RECORD_SIZE);
}

static void BootMeta_Read(uint32_t index, BootMetaRecord_t *record)
{
    (void)memcpy(record,
                 (const void *)BootMeta_RecordAddress(index),
                 sizeof(*record));
}

static uint8_t BootMeta_SlotIsErased(uint32_t index)
{
    const uint32_t *words = (const uint32_t *)BootMeta_RecordAddress(index);
    uint32_t word_index;

    for (word_index = 0UL;
         word_index < (BOOT_META_RECORD_SIZE / sizeof(uint32_t));
         ++word_index)
    {
        if (words[word_index] != 0xFFFFFFFFUL)
        {
            return 0U;
        }
    }

    return 1U;
}

uint8_t BootMeta_RecordIsValid(const BootMetaRecord_t *record)
{
    uint32_t expected_crc;

    if ((record == NULL) || (record->magic != BOOT_META_MAGIC) ||
        (record->record_version != BOOT_META_RECORD_VERSION) ||
        (record->commit_marker != BOOT_META_COMMIT_MARKER) ||
        (record->state < BOOT_META_STATE_DOWNLOADING) ||
        (record->state > BOOT_META_STATE_INVALID))
    {
        return 0U;
    }

    expected_crc = BootCrc32_Calculate((const uint8_t *)record,
                                       BOOT_META_CRC_LENGTH);
    return (expected_crc == record->record_crc32) ? 1U : 0U;
}

uint8_t BootMeta_GetLatest(BootMetaRecord_t *record)
{
    BootMetaRecord_t candidate;
    uint32_t index;
    uint8_t found = 0U;

    if (record == NULL)
    {
        return 0U;
    }

    (void)memset(record, 0, sizeof(*record));

    for (index = 0UL; index < BOOT_META_RECORD_COUNT; ++index)
    {
        BootMeta_Read(index, &candidate);
        if (BootMeta_RecordIsValid(&candidate) != 0U)
        {
            if ((found == 0U) || (candidate.sequence > record->sequence))
            {
                *record = candidate;
                found = 1U;
            }
        }
    }

    return found;
}

uint8_t BootMeta_HasRecognizedRecord(void)
{
    BootMetaRecord_t candidate;
    uint32_t index;

    /*
     * A partially committed record must still suppress the factory fallback.
     * BEGIN writes metadata before erasing the Application, so seeing either
     * end of our record format means an OTA transaction has owned the image.
     */
    for (index = 0UL; index < BOOT_META_RECORD_COUNT; ++index)
    {
        BootMeta_Read(index, &candidate);
        if ((candidate.magic == BOOT_META_MAGIC) ||
            (candidate.commit_marker == BOOT_META_COMMIT_MARKER))
        {
            return 1U;
        }
    }

    return 0U;
}

uint8_t BootMeta_Append(BootMetaRecord_t *record)
{
    BootMetaRecord_t latest;
    uint32_t index;
    uint32_t target_address = 0UL;
    uint32_t next_sequence = 1UL;
    BootFlashResult_t flash_result;

    if ((record == NULL) ||
        (record->state < BOOT_META_STATE_DOWNLOADING) ||
        (record->state > BOOT_META_STATE_INVALID))
    {
        return 0U;
    }

    if (BootMeta_GetLatest(&latest) != 0U)
    {
        next_sequence = latest.sequence + 1UL;
        if (next_sequence == 0UL)
        {
            next_sequence = 1UL;
        }
    }

    for (index = 0UL; index < BOOT_META_RECORD_COUNT; ++index)
    {
        if (BootMeta_SlotIsErased(index) != 0U)
        {
            target_address = BootMeta_RecordAddress(index);
            break;
        }
    }

    if (target_address == 0UL)
    {
        if (BootFlash_EraseMetadata() != BOOT_FLASH_OK)
        {
            return 0U;
        }
        target_address = GRIPPER_OTA_META_BASE;
    }

    record->magic = BOOT_META_MAGIC;
    record->record_version = BOOT_META_RECORD_VERSION;
    record->sequence = next_sequence;
    record->record_crc32 = 0xFFFFFFFFUL;
    record->commit_marker = 0xFFFFFFFFUL;
    record->record_crc32 = BootCrc32_Calculate((const uint8_t *)record,
                                               BOOT_META_CRC_LENGTH);
    record->commit_marker = BOOT_META_COMMIT_MARKER;

    /* Program the CRC + commit-marker doubleword last. */
    flash_result = BootFlash_ProgramMetadata(target_address,
                                             (const uint8_t *)record,
                                             BOOT_META_CRC_LENGTH);
    if (flash_result != BOOT_FLASH_OK)
    {
        return 0U;
    }

    flash_result = BootFlash_ProgramMetadata(target_address + BOOT_META_CRC_LENGTH,
                                             ((const uint8_t *)record) +
                                                 BOOT_META_CRC_LENGTH,
                                             BOOT_META_RECORD_SIZE -
                                                 BOOT_META_CRC_LENGTH);
    if (flash_result != BOOT_FLASH_OK)
    {
        return 0U;
    }

    BootMeta_Read((target_address - GRIPPER_OTA_META_BASE) /
                      BOOT_META_RECORD_SIZE,
                  &latest);
    return BootMeta_RecordIsValid(&latest);
}
