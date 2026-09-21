/**
 * @file   AS5048A.c
 * @brief  AS5048A 磁编码器驱动
 * @note   移植自 STM32F10x 标准库版本，使用 SPI1 接口，CS 引脚为 PA4
 */

#include "AS5048A.h"
#include "spi.h"
#include "gpio.h"
#include <stdio.h>

// 全局错误标志
uint8_t error_flag = 0;
uint16_t angle_val = 0;   // 角度原始值
uint16_t mag_val   = 0;   // 磁信号幅值
float    angle_deg = 0.0f; // 角度（度）
uint8_t  angle_valid = 0;

// CS 引脚定义（根据硬件连接修改）
#define AS5048_CS_PORT  GPIOA
#define AS5048_CS_PIN   GPIO_PIN_4

/**
 * @brief  初始化 DWT 微秒延时
 * @param  None
 * @retval None
 */
void DWT_Delay_Init(void) {
    if (!(CoreDebug->DEMCR & CoreDebug_DEMCR_TRCENA_Msk)) {
        CoreDebug->DEMCR |= CoreDebug_DEMCR_TRCENA_Msk;
    }
    DWT->CYCCNT = 0;
    DWT->CTRL |= DWT_CTRL_CYCCNTENA_Msk;
}

/**
 * @brief  DWT 微秒延时
 * @param  us: 延时长度（微秒）
 * @retval None
 */
void DWT_Delay_us(uint32_t us) {
    uint32_t start = DWT->CYCCNT;
    uint32_t ticks = (SystemCoreClock / 1000000U) * us;
    while ((DWT->CYCCNT - start) < ticks) {
        __NOP();
    }
}

/**
 * @brief  计算奇偶校验位
 * @param  v: 16 位数据
 * @retval 1 表示奇数个 1，0 表示偶数个 1
 */
uint8_t parity_even(uint16_t v) {
    if (v == 0) return 0;
    v ^= v >> 8;
    v ^= v >> 4;
    v ^= v >> 2;
    v ^= v >> 1;
    return v & 1;
}

/**
 * @brief  AS5048A 初始化
 * @param  None
 * @retval None
 */
void AS5048A_QH_Init(void) {
    /* 初始化DWT延时 */
    DWT_Delay_Init();
    
    /* CS引脚设置为高电平(禁用) */
    HAL_GPIO_WritePin(AS5048_CS_PORT, AS5048_CS_PIN, GPIO_PIN_SET);
    DWT_Delay_us(500);
}

/**
 * @brief  SPI 发送并接收 16 位数据
 * @param  TxData: 要发送的 16 位数据
 * @retval 接收到的 16 位数据
 */
uint16_t SPI_WriteByte(uint16_t TxData) {
    uint16_t RxData = 0;
    
    /* CS拉低 */
    HAL_GPIO_WritePin(AS5048_CS_PORT, AS5048_CS_PIN, GPIO_PIN_RESET);
    DWT_Delay_us(1);
    
    /* SPI 传输：16 位模式直接传输 */
    HAL_SPI_TransmitReceive(&hspi1, (uint8_t*)&TxData, (uint8_t*)&RxData, 1, 100);
    
    DWT_Delay_us(1);
    
    /* CS拉高 */
    HAL_GPIO_WritePin(AS5048_CS_PORT, AS5048_CS_PIN, GPIO_PIN_SET);
    DWT_Delay_us(1);
    
    return RxData;
}

/**
 * @brief  读取 AS5048A 寄存器
 * @param  cmd: 寄存器地址
 * @retval 寄存器值（14 位）
 */
uint16_t Read_As5048A_Reg(uint16_t cmd) {
    uint16_t data = 0;
    uint16_t res;
    uint16_t command;
    
    // 构造读命令: [15]=Parity [14]=1(读) [13:0]=Address
    command = 0x4000 | (cmd & 0x3FFF);    // bit14=1 表示读操作
    command |= ((uint16_t)parity_even(command) << 15);
    
    // 第一阶段: 发送读命令
    SPI_WriteByte(command);
    DWT_Delay_us(10);  // 与原项目保持一致
    
    // 第二阶段: 发送 NOP 获取数据
    command = CMD_NOP & 0x3FFF;
    command |= ((uint16_t)parity_even(command) << 15);
    res = SPI_WriteByte(command);
    DWT_Delay_us(10);
    
    // 提取 14 位数据
    data = (res & 0x3FFF);
    
    // 检查错误标志（bit14）
    if ((res & (1 << 14)) != 0) {
        error_flag = 1;
        // 发送清除错误命令
        command = 0x4000 | CMD_CLEAR_ERROR;  // bit14=1 表示写操作
        command |= ((uint16_t)parity_even(command) << 15);
        SPI_WriteByte(command);
        DWT_Delay_us(10);
    } else {
        // 验证奇偶校验
        error_flag = (parity_even(res & 0x7FFF) != ((res >> 15) & 0x01));
    }
    
    return data;
}

/**
 * @brief  写 AS5048A 寄存器
 * @param  cmd: 寄存器地址
 * @param  value: 要写入的值（14 位）
 * @retval None
 */
void Write_As5048A_Reg(uint16_t cmd, uint16_t value) {
    uint16_t data = 0;
    uint16_t res;
    uint16_t command;
    
    // 第一阶段: 发送写命令（bit14=1 表示写操作）
    command = 0x4000 | (cmd & 0x3FFF);
    command |= ((uint16_t)parity_even(command) << 15);
    SPI_WriteByte(command);
    DWT_Delay_us(2);
    
    // 第二阶段: 发送数据
    command = 0x4000 | (value & 0x3FFF);
    command |= ((uint16_t)parity_even(command) << 15);
    SPI_WriteByte(command);
    DWT_Delay_us(2);
    
    // 第三阶段: 发送 NOP 读取响应
    command = CMD_NOP & 0x3FFF;
    command |= ((uint16_t)parity_even(command) << 15);
    res = SPI_WriteByte(command);
    
    error_flag = 1;
    if ((res & (1 << 14)) == 0) {
        data = (res & 0x3FFF);
        error_flag = (parity_even(data) ^ (res >> 15));
    } else {
        command = 0x4000 | CMD_CLEAR_ERROR;
        command |= ((uint16_t)parity_even(command) << 15);
        SPI_WriteByte(command);
    }
}

/**
 * @brief  读取 AS5048A 角度值（带错误检查）
 * @param  cmd: 寄存器地址
 * @retval 角度值，若有错误返回 0
 */
uint16_t Read_As5048A_QH_Value(uint16_t cmd) {
    uint16_t val;
    val = Read_As5048A_Reg(cmd);
    if (error_flag) {
        val = 0;
    }
    return val;
}

/**
 * @brief  写入 AS5048A 零点位置到 OTP（一次性可编程）
 * @param  None
 * @retval 0=成功，非 0=错误码
 * @note   此操作不可逆，请谨慎使用
 */
uint8_t Write_As5048A_ZeroPosition(void) {
    uint16_t Angle_val;
    uint8_t Angle_High, Angle_Low;
    uint16_t cmd;
    
    // 1. 读取当前角度
    Angle_val = Read_As5048A_QH_Value(CMD_ANGLE);
    cmd = Read_As5048A_QH_Value(CMD_ProgramControl);
    
    // 2. 设置编程使能位
    Write_As5048A_Reg(CMD_ProgramControl, 0x01);
    DWT_Delay_us(10);
    cmd = Read_As5048A_QH_Value(CMD_ProgramControl);
    DWT_Delay_us(10);
    
    // 3. 写入零点位置
    Write_As5048A_Reg(CMD_OTPHigh, Angle_val >> 8);
    DWT_Delay_us(10);
    Write_As5048A_Reg(CMD_OTPLow, Angle_val & 0xFF);
    DWT_Delay_us(10);
    
    // 4. 验证写入的数据
    Angle_High = Read_As5048A_QH_Value(CMD_OTPHigh);
    DWT_Delay_us(10);
    Angle_Low = Read_As5048A_QH_Value(CMD_OTPLow);
    DWT_Delay_us(10);
    
    if (Angle_High != (uint8_t)(Angle_val >> 8))
        return 1;
    if (Angle_Low != (uint8_t)(Angle_val & 0xFF))
        return 2;
    
    // 5. 开始烧录
    Write_As5048A_Reg(CMD_ProgramControl, 0x09);
    DWT_Delay_us(10);
    cmd = Read_As5048A_QH_Value(CMD_ProgramControl);
    DWT_Delay_us(10);
    Angle_val = Read_As5048A_QH_Value(CMD_ANGLE);
    DWT_Delay_us(10);
    
    if (Angle_val != 0)
        return 3;
    
    // 6. 验证烧录结果
    Write_As5048A_Reg(CMD_ProgramControl, 0x49);
    DWT_Delay_us(10);
    cmd = Read_As5048A_QH_Value(CMD_ProgramControl);
    DWT_Delay_us(10);
    Angle_val = Read_As5048A_QH_Value(CMD_ANGLE);
    DWT_Delay_us(10);
    
    if (Angle_val != 0)
        return 4;
    
    return 0;
}

/**
 * @brief  更新 AS5048A 角度与幅值
 * @param  None
 * @retval None
 */
void AS5048A_Update_All(void)
{
    uint16_t raw;

    // 先清掉旧错误标志
    error_flag = 0;

    // 1. 读取角度寄存器
    raw = Read_As5048A_Reg(CMD_ANGLE);
    if (error_flag)
    {
        angle_valid = 0;
        return; // 读取失败，直接返回，保留旧值
    }

    angle_val = raw;
    angle_deg = (float)raw * 360.0f / 16384.0f;
    angle_valid = 1;

    // 2. 读取幅值寄存器
    raw = Read_As5048A_Reg(CMD_READ_MAG);
    if (error_flag)
    {
        return; // 同样，如果失败就退出
    }

    mag_val = raw;
}
