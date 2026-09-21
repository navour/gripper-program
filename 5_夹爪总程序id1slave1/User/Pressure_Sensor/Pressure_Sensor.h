#ifndef __PRESSURE_SENSOR_H__
#define __PRESSURE_SENSOR_H__

#include "main.h" 

// 压力传感器数据结构定义
typedef struct {
    uint16_t BE_arrayC[32], BE_arrayD[32], BE_arrayE[32];
    uint16_t LE_arrayC[32];
} SensorData_t;

// 全局变量
extern SensorData_t sensor_data;

typedef enum {
    PRESSURE_REGION_TOTAL = 0,
    PRESSURE_REGION_BE_SUM,
    PRESSURE_REGION_BE_C,
    PRESSURE_REGION_BE_D,
    PRESSURE_REGION_BE_E,
    PRESSURE_REGION_LE_C,
    PRESSURE_REGION_COUNT
} PressureRegion_t;

typedef struct {
    uint8_t  calibrated;
    uint8_t  contact;
    uint32_t sum;
    uint16_t max;
    uint8_t  area;
} PressureFeature_t;

/**
 * @brief  压力传感器初始化
 * @param  None
 * @retval None
 */
void PressureSensor_Init(void);
/**
 * @brief  更新压力数据并打印
 * @param  None
 * @retval None
 */
void PressureSensor_UpdateAndPrint(void);

// ==================== 压力安全标志 ====================
// bit0: 已锁存（主机写 SAFE_UNLOCK=0xA55A 清除）
// bit1: 总和 >= 阈值（当前状态）
// bit2: 最大值 >= 阈值（当前状态）
// bit3: 解锁等待中（已写解锁命令，等待释放/重新装夹）

#define PRESS_SAFE_FLAG_LATCHED   (1u<<0)
#define PRESS_SAFE_FLAG_SUM_OVER  (1u<<1)
#define PRESS_SAFE_FLAG_MAX_OVER  (1u<<2)
#define PRESS_SAFE_FLAG_ARMING    (1u<<3)

uint8_t  PressureSafety_IsLatched(void);
uint8_t  PressureSafety_IsBlocked(void);
uint16_t PressureSafety_GetFlags(void);
void     PressureSafety_ClearLatch(void);

// 阈值接口
void     PressureSafety_SetThreshold(uint32_t sum_th, uint16_t max_th);
void     PressureSafety_GetThreshold(uint32_t *sum_th, uint16_t *max_th);

// 获取最新压力统计值（给 Modbus/上位机使用）
void     PressureSafety_GetLatest(uint32_t *sum, uint16_t *max);
void     PressureSensor_GetLatestFeature(PressureRegion_t region, PressureFeature_t *feat);

#endif /* __PRESSURE_SENSOR_H__ */
