#ifndef PRESSURE_HISTORY_H
#define PRESSURE_HISTORY_H
#include <stdint.h>

#define MAT_REG_FIRST       0x0150u
#define MAT_REG_COMMAND     0x0160u
#define MAT_COMMAND_WORDS   8u
#define MAT_REG_HEAD        0x0170u
#define MAT_REG_RAW         0x0180u
#define MAT_REG_END         0x0200u
#define MAT_RAW_WORDS       128u
#define MAT_QUEUE_CAPACITY  4u
#define MAT_PROTOCOL_VER   1u
#define MAT_START           1u
#define MAT_STOP            2u
#define MAT_ACK_DURABLE     3u

/* Init before scheduling. BeginCycle and PushCycle: sole pressure producer.
 * SubmitCommand: UART ISR; Service: communication task. ReadRegs outputs
 * big-endian bytes and pins one head for the entire response range. */
void PressureHistory_Init(uint16_t slave, uint16_t layout);
void PressureHistory_BeginCycle(void);
void PressureHistory_PushCycle(const uint16_t *r0, const uint16_t *r1,
                              const uint16_t *r2, const uint16_t *r3,
                              uint32_t be_ms, uint32_t le_ms,
                              uint16_t flags, uint16_t cal_mask);
uint8_t PressureHistory_SubmitCommand(const uint16_t words[MAT_COMMAND_WORDS]);
void PressureHistory_Service(void);
void PressureHistory_ReadRegs(uint16_t start, uint16_t count, uint8_t *out);
#endif
