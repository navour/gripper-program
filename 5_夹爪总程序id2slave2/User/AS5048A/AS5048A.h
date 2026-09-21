#ifndef __AS5048A_H
#define __AS5048A_H

#include "stm32g4xx_hal.h"
#include <stdint.h>
#include <stdbool.h>

// AS5048A 命令/寄存器地址
#define AS5_CMD_NOP            0x0000U
#define AS5_CMD_CLR_ERR        0x0001U
#define AS5_CMD_PROG_CTRL      0x0003U
#define AS5_CMD_OTP_HIGH       0x0016U
#define AS5_CMD_OTP_LOW        0x0017U
#define AS5_REG_DIAG           0x3FFDU
#define AS5_REG_MAG            0x3FFEU
#define AS5_REG_ANGLE          0x3FFFU

// 兼容旧的宏定义
#define CMD_ANGLE          AS5_REG_ANGLE
#define CMD_READ_MAG       AS5_REG_MAG
#define CMD_READ_DIAG      AS5_REG_DIAG
#define CMD_NOP            AS5_CMD_NOP
#define CMD_CLEAR_ERROR    AS5_CMD_CLR_ERR
#define CMD_ProgramControl AS5_CMD_PROG_CTRL
#define CMD_OTPHigh        AS5_CMD_OTP_HIGH
#define CMD_OTPLow         AS5_CMD_OTP_LOW

// 全局错误标志（由驱动内部更新）
extern uint8_t error_flag;

// 基础 SPI 通信函数
void     AS5048A_QH_Init(void);
uint16_t SPI_WriteByte(uint16_t TxData);

// 寄存器读写函数
uint16_t Read_As5048A_Reg(uint16_t cmd);
void     Write_As5048A_Reg(uint16_t cmd, uint16_t value);
uint16_t Read_As5048A_QH_Value(uint16_t cmd);
uint8_t  Write_As5048A_ZeroPosition(void);

// 工具函数
uint8_t  parity_even(uint16_t v);

// 微秒延时（DWT 版）
void DWT_Delay_Init(void);
void DWT_Delay_us(uint32_t us);

extern uint16_t angle_val;
extern uint16_t mag_val;
extern float    angle_deg;
extern uint8_t  angle_valid;

void AS5048A_Update_All(void);

#endif
