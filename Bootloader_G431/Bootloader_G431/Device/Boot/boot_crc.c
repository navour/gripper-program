#include "boot_crc.h"

uint16_t BootCrc16_Modbus(const uint8_t *data, uint32_t length)
{
    uint16_t crc = 0xFFFFU;
    uint32_t index;
    uint8_t bit;

    for (index = 0U; index < length; ++index)
    {
        crc ^= data[index];
        for (bit = 0U; bit < 8U; ++bit)
        {
            if ((crc & 1U) != 0U)
            {
                crc = (uint16_t)((crc >> 1U) ^ 0xA001U);
            }
            else
            {
                crc >>= 1U;
            }
        }
    }

    return crc;
}

uint32_t BootCrc32_Begin(void)
{
    return 0xFFFFFFFFUL;
}

uint32_t BootCrc32_Update(uint32_t crc, const uint8_t *data, uint32_t length)
{
    uint32_t index;
    uint8_t bit;

    for (index = 0U; index < length; ++index)
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

    return crc;
}

uint32_t BootCrc32_End(uint32_t crc)
{
    return crc ^ 0xFFFFFFFFUL;
}

uint32_t BootCrc32_Calculate(const uint8_t *data, uint32_t length)
{
    return BootCrc32_End(BootCrc32_Update(BootCrc32_Begin(), data, length));
}
