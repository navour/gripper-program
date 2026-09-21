#ifndef DM_MODBUS_BRIDGE_H
#define DM_MODBUS_BRIDGE_H

#include "stm32g4xx_hal.h"
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

// ========= 对外接口（放到 Inc/ 目录）===========

/**
 * @brief  初始化 Modbus 桥接
 * @param  huartx: 用于 Modbus 的 UART 句柄
 * @param  hfdcan1: 用于电机的 FDCAN 句柄
 * @param  motor_id: 电机 ID（用于寄存器初值）
 * @retval None
 */
void DM_Bridge_Init(UART_HandleTypeDef *huartx, FDCAN_HandleTypeDef *hfdcan1, uint16_t motor_id);

/**
 * @brief  启动 UART Idle 接收
 * @param  None
 * @retval None
 */
void DM_Bridge_Start(void);

/**
 * @brief  主循环轮询处理（非阻塞发送兜底）
 * @param  None
 * @retval None
 */
void DM_Bridge_Poll(void);

/**
 * @brief  UART Idle 回调，解析 Modbus 帧
 * @param  huart: UART 句柄
 * @param  size: 接收长度
 * @retval None
 */
void DM_Bridge_OnUartIdle(UART_HandleTypeDef *huart, uint16_t size);

/**
 * @brief  UART 发送完成回调
 * @param  huart: UART 句柄
 * @retval None
 */
void DM_Bridge_OnUartTxDone(UART_HandleTypeDef *huart);

/**
 * @brief  CAN 接收回调，更新反馈寄存器
 * @param  rxh: CAN 帧头
 * @param  data: 数据区（8 字节）
 * @retval None
 */
void DM_Bridge_OnCanRx(const FDCAN_RxHeaderTypeDef *rxh, const uint8_t data[8]);

#ifdef __cplusplus
}
#endif

#endif /* DM_MODBUS_BRIDGE_H */
