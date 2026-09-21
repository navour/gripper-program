#ifndef BOOT_CRC_H
#define BOOT_CRC_H

#include <stdint.h>

uint16_t BootCrc16_Modbus(const uint8_t *data, uint32_t length);

uint32_t BootCrc32_Begin(void);
uint32_t BootCrc32_Update(uint32_t crc, const uint8_t *data, uint32_t length);
uint32_t BootCrc32_End(uint32_t crc);
uint32_t BootCrc32_Calculate(const uint8_t *data, uint32_t length);

#endif /* BOOT_CRC_H */
