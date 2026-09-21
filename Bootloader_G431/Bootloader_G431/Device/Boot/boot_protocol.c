#include "boot_protocol.h"

#include <string.h>

#include "main.h"
#include "usart.h"

#include "boot_crc.h"
#include "gripper_ota_layout.h"

#define BOOT_SOF_0                      (0x47U)
#define BOOT_SOF_1                      (0x42U)
#define BOOT_INTERBYTE_TIMEOUT_MS       (100UL)
#define BOOT_TX_TIMEOUT_MS              (1000UL)
#define BOOT_TX_COMPLETE_TIMEOUT_MS     (20UL)

uint16_t BootProtocol_GetU16(const uint8_t *bytes)
{
    return (uint16_t)((uint16_t)bytes[0] |
                      ((uint16_t)bytes[1] << 8U));
}

uint32_t BootProtocol_GetU32(const uint8_t *bytes)
{
    return (uint32_t)bytes[0] |
           ((uint32_t)bytes[1] << 8U) |
           ((uint32_t)bytes[2] << 16U) |
           ((uint32_t)bytes[3] << 24U);
}

void BootProtocol_PutU16(uint8_t *bytes, uint16_t value)
{
    bytes[0] = (uint8_t)(value & 0xFFU);
    bytes[1] = (uint8_t)((value >> 8U) & 0xFFU);
}

void BootProtocol_PutU32(uint8_t *bytes, uint32_t value)
{
    bytes[0] = (uint8_t)(value & 0xFFUL);
    bytes[1] = (uint8_t)((value >> 8U) & 0xFFUL);
    bytes[2] = (uint8_t)((value >> 16U) & 0xFFUL);
    bytes[3] = (uint8_t)((value >> 24U) & 0xFFUL);
}

static uint8_t BootProtocol_FindSof(uint8_t *raw, uint32_t timeout_ms)
{
    uint32_t start_tick = HAL_GetTick();
    uint32_t elapsed;
    uint32_t remaining;
    uint8_t byte;
    uint8_t have_first = 0U;
    HAL_StatusTypeDef status;

    do
    {
        elapsed = HAL_GetTick() - start_tick;
        if (elapsed >= timeout_ms)
        {
            return 0U;
        }
        remaining = timeout_ms - elapsed;

        status = HAL_UART_Receive(&huart2, &byte, 1U, remaining);
        if (status == HAL_TIMEOUT)
        {
            return 0U;
        }
        if (status != HAL_OK)
        {
            return 0U;
        }

        if (have_first == 0U)
        {
            have_first = (byte == BOOT_SOF_0) ? 1U : 0U;
        }
        else if (byte == BOOT_SOF_1)
        {
            raw[0] = BOOT_SOF_0;
            raw[1] = BOOT_SOF_1;
            return 1U;
        }
        else
        {
            have_first = (byte == BOOT_SOF_0) ? 1U : 0U;
        }
    } while (1);
}

static void BootProtocol_ParseHeader(const uint8_t *raw, BootFrame_t *frame)
{
    frame->protocol_version = raw[2];
    frame->command = raw[3];
    frame->session_id = BootProtocol_GetU32(&raw[4]);
    frame->sequence = BootProtocol_GetU16(&raw[8]);
    frame->flags_or_status = BootProtocol_GetU16(&raw[10]);
    frame->offset = BootProtocol_GetU32(&raw[12]);
    frame->payload_length = BootProtocol_GetU16(&raw[16]);
}

BootReceiveResult_t BootProtocol_Receive(BootFrame_t *frame,
                                         uint32_t first_byte_timeout_ms)
{
    uint8_t raw[BOOT_PROTOCOL_MAX_FRAME_SIZE];
    uint16_t received_crc;
    uint16_t calculated_crc;
    uint16_t tail_length;
    HAL_StatusTypeDef status;

    if (frame == NULL)
    {
        return BOOT_RX_IO_ERROR;
    }

    (void)memset(frame, 0, sizeof(*frame));

    if ((first_byte_timeout_ms == 0UL) ||
        (BootProtocol_FindSof(raw, first_byte_timeout_ms) == 0U))
    {
        return BOOT_RX_TIMEOUT;
    }

    status = HAL_UART_Receive(&huart2,
                              &raw[2],
                              BOOT_PROTOCOL_HEADER_SIZE - 2U,
                              BOOT_INTERBYTE_TIMEOUT_MS);
    if (status != HAL_OK)
    {
        return BOOT_RX_IO_ERROR;
    }

    BootProtocol_ParseHeader(raw, frame);
    if (frame->payload_length > BOOT_PROTOCOL_MAX_PAYLOAD)
    {
        return BOOT_RX_BAD_LENGTH;
    }

    tail_length = (uint16_t)(frame->payload_length + BOOT_PROTOCOL_CRC_SIZE);
    status = HAL_UART_Receive(&huart2,
                              &raw[BOOT_PROTOCOL_HEADER_SIZE],
                              tail_length,
                              BOOT_INTERBYTE_TIMEOUT_MS);
    if (status != HAL_OK)
    {
        return BOOT_RX_IO_ERROR;
    }

    if (frame->payload_length != 0U)
    {
        (void)memcpy(frame->payload,
                     &raw[BOOT_PROTOCOL_HEADER_SIZE],
                     frame->payload_length);
    }

    received_crc = BootProtocol_GetU16(
        &raw[BOOT_PROTOCOL_HEADER_SIZE + frame->payload_length]);
    calculated_crc = BootCrc16_Modbus(
        raw,
        BOOT_PROTOCOL_HEADER_SIZE + frame->payload_length);

    if (received_crc != calculated_crc)
    {
        return BOOT_RX_BAD_CRC;
    }

    return BOOT_RX_FRAME;
}

uint8_t BootProtocol_SendResponse(const BootFrame_t *request,
                                  BootStatus_t status,
                                  uint32_t next_offset,
                                  const uint8_t *payload,
                                  uint16_t payload_length)
{
    uint8_t raw[BOOT_PROTOCOL_MAX_FRAME_SIZE];
    uint16_t frame_crc;
    uint16_t frame_length;
    uint32_t wait_start;
    HAL_StatusTypeDef hal_status;

    if ((request == NULL) || (payload_length > BOOT_PROTOCOL_MAX_PAYLOAD) ||
        ((payload_length != 0U) && (payload == NULL)))
    {
        return 0U;
    }

    raw[0] = BOOT_SOF_0;
    raw[1] = BOOT_SOF_1;
    raw[2] = GRIPPER_PROTOCOL_VERSION;
    raw[3] = request->command | BOOT_COMMAND_RESPONSE_MASK;
    BootProtocol_PutU32(&raw[4], request->session_id);
    BootProtocol_PutU16(&raw[8], request->sequence);
    BootProtocol_PutU16(&raw[10], (uint16_t)status);
    BootProtocol_PutU32(&raw[12], next_offset);
    BootProtocol_PutU16(&raw[16], payload_length);

    if (payload_length != 0U)
    {
        (void)memcpy(&raw[BOOT_PROTOCOL_HEADER_SIZE], payload, payload_length);
    }

    frame_crc = BootCrc16_Modbus(raw,
                                 BOOT_PROTOCOL_HEADER_SIZE + payload_length);
    BootProtocol_PutU16(&raw[BOOT_PROTOCOL_HEADER_SIZE + payload_length],
                        frame_crc);
    frame_length = (uint16_t)(BOOT_PROTOCOL_HEADER_SIZE + payload_length +
                              BOOT_PROTOCOL_CRC_SIZE);

    HAL_GPIO_WritePin(RS485_DE_GPIO_Port, RS485_DE_Pin, GPIO_PIN_SET);
    HAL_Delay(1UL);
    hal_status = HAL_UART_Transmit(&huart2,
                                   raw,
                                   frame_length,
                                   BOOT_TX_TIMEOUT_MS);

    wait_start = HAL_GetTick();
    while ((__HAL_UART_GET_FLAG(&huart2, UART_FLAG_TC) == RESET) &&
           ((HAL_GetTick() - wait_start) < BOOT_TX_COMPLETE_TIMEOUT_MS))
    {
    }

    HAL_GPIO_WritePin(RS485_DE_GPIO_Port, RS485_DE_Pin, GPIO_PIN_RESET);

    if ((hal_status != HAL_OK) ||
        (__HAL_UART_GET_FLAG(&huart2, UART_FLAG_TC) == RESET))
    {
        return 0U;
    }

    return 1U;
}
