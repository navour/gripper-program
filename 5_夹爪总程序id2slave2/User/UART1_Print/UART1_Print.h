#ifndef __UART1_PRINT_H__
#define __UART1_PRINT_H__

#include <stdint.h>
#include <stdbool.h>
#include "cmsis_os.h"

#ifdef __cplusplus
extern "C" {
#endif

typedef struct {
    uint8_t  ok;
    uint16_t raw;
    float    deg;
    uint16_t mag;
} AS5048A_Latest_t;

typedef struct {
    uint8_t  calibrated;
    uint8_t  contact;
    uint32_t sum;
    uint16_t max;
    uint8_t  max_r;
    uint8_t  max_c;
    uint8_t  area;
} PressureFeat_t;

// 压力区域编号（BE=0，LE=1，以及细分通道）
typedef enum {
    PRESSURE_SIDE_BE      = 0,  // 背面合并区域（BE_SUM）
    PRESSURE_SIDE_LE      = 1,  // 侧面区域（LE_C）

    PRESSURE_SIDE_BE_B    = 2,
    PRESSURE_SIDE_BE_C    = 3,
    PRESSURE_SIDE_BE_E    = 4,
    PRESSURE_SIDE_TOTAL   = 5,  // TOTAL = BE_SUM + LE_C
} PressureSide_t;

void Uart1_Print_UpdateAS5048A(bool ok, uint16_t raw, float deg, uint16_t mag);
void Uart1_Print_UpdatePressure(PressureSide_t side, const PressureFeat_t *feat);
void UART1Print(void *argument);

#ifdef __cplusplus
}
#endif

#endif /* __UART1_PRINT_H__ */
