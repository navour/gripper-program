#ifndef POS_PERSIST_H
#define POS_PERSIST_H

#include "stm32g4xx_hal.h"
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief  持久化模块初始化
 * @param  None
 * @retval None
 */
void PosPersist_Init(void);

/**
 * @brief  读取上一次保存的机械零点（AS5048A 绝对角度）
 * @param  pos_rad_out: 输出机械零点角度（rad, 0..2π）
 * @param  scale_out: 输出保存时的比例系数
 * @retval 1=读取到有效记录，0=无有效记录
 */
uint8_t PosPersist_Load(float *pos_rad_out, float *scale_out);

uint8_t PosPersist_LoadEx(float *pos_rad_out, float *scale_out,
                          float *bias_rad_out, uint8_t *bias_valid_out);

/**
 * @brief  写入机械零点到 Flash（仅在用户显式触发时写，避免频繁擦写）
 * @param  pos_rad: 机械零点绝对角度（rad, 0..2π）
 * @param  scale: 当前比例系数
 * @param  allow_erase: 1=页满允许擦除并重写，0=页满返回 HAL_BUSY
 * @retval HAL 状态
 */
HAL_StatusTypeDef PosPersist_Save(float pos_rad, float scale, uint8_t allow_erase);

HAL_StatusTypeDef PosPersist_SaveEx(float pos_rad, float scale,
                                    float bias_rad, uint8_t allow_erase);

#ifdef __cplusplus
}
#endif

#endif /* POS_PERSIST_H */
