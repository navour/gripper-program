#include "Uart1_Print.h"
#include "usart.h"

#include <stdio.h>
#include <string.h>

#include "FreeRTOS.h"
#include "task.h"

static volatile AS5048A_Latest_t g_as = {0};

// 各路压力特征缓存
static volatile PressureFeat_t g_be_c   = {0};
static volatile PressureFeat_t g_be_d   = {0};
static volatile PressureFeat_t g_be_e   = {0};
static volatile PressureFeat_t g_le_c   = {0};
static volatile PressureFeat_t g_be_sum = {0};
static volatile PressureFeat_t g_total  = {0};

/**
 * @brief  更新 AS5048A 最新数据
 * @param  ok: 读取是否成功
 * @param  raw: 原始角度值
 * @param  deg: 角度（度）
 * @param  mag: 磁场幅值
 * @retval None
 */
void Uart1_Print_UpdateAS5048A(bool ok, uint16_t raw, float deg, uint16_t mag)
{
    taskENTER_CRITICAL();
    g_as.ok  = ok ? 1U : 0U;
    g_as.raw = raw;
    g_as.deg = deg;
    g_as.mag = mag;
    taskEXIT_CRITICAL();
}

/**
 * @brief  更新压力特征缓存
 * @param  side: 压力区域编号
 * @param  feat: 压力特征数据
 * @retval None
 */
void Uart1_Print_UpdatePressure(PressureSide_t side, const PressureFeat_t *feat)
{
    if (feat == NULL) return;

    taskENTER_CRITICAL();
    switch (side)
    {
        case PRESSURE_SIDE_BE_C:   g_be_c = *feat; break;
        case PRESSURE_SIDE_BE_D:   g_be_d = *feat; break;
        case PRESSURE_SIDE_BE_E:   g_be_e = *feat; break;
        case PRESSURE_SIDE_TOTAL:  g_total = *feat; break;

        case PRESSURE_SIDE_LE:     g_le_c = *feat; break;   // LE 对应 LE_C
        case PRESSURE_SIDE_BE:     g_be_sum = *feat; break; // BE 对应 BE_SUM
        default: break;
    }
    taskEXIT_CRITICAL();
}

/**
 * @brief  UART1 打印任务
 * @param  argument: 任务参数（未使用）
 * @retval None
 */
void UART1Print(void *argument)
{
    (void)argument;

    char buf[420];

    for (;;)
    {
        AS5048A_Latest_t as;
        PressureFeat_t beC, beD, beE, leC, beSum, total;

        taskENTER_CRITICAL();
        as    = g_as;
        beC   = g_be_c;
        beD   = g_be_d;
        beE   = g_be_e;
        leC   = g_le_c;
        beSum = g_be_sum;
        total = g_total;
        taskEXIT_CRITICAL();

        int n = snprintf(buf, sizeof(buf),
            "AS:%s raw=%u deg=%.2f mag=%u | "
            "TOTAL:%s cal=%u sum=%lu max=%u area=%u | "
            "BE_SUM:%s cal=%u sum=%lu max=%u area=%u | "
            "BE_C:%s cal=%u sum=%lu max=%u area=%u | "
            "BE_D:%s cal=%u sum=%lu max=%u area=%u | "
            "BE_E:%s cal=%u sum=%lu max=%u area=%u | "
            "LE_C:%s cal=%u sum=%lu max=%u area=%u\r\n",
            (as.ok ? "OK" : "ERR"), (unsigned)as.raw, (double)as.deg, (unsigned)as.mag,

            (total.contact ? "ON" : "OFF"), (unsigned)total.calibrated, (unsigned long)total.sum, (unsigned)total.max, (unsigned)total.area,
            (beSum.contact ? "ON" : "OFF"), (unsigned)beSum.calibrated, (unsigned long)beSum.sum, (unsigned)beSum.max, (unsigned)beSum.area,

            (beC.contact ? "ON" : "OFF"), (unsigned)beC.calibrated, (unsigned long)beC.sum, (unsigned)beC.max, (unsigned)beC.area,
            (beD.contact ? "ON" : "OFF"), (unsigned)beD.calibrated, (unsigned long)beD.sum, (unsigned)beD.max, (unsigned)beD.area,
            (beE.contact ? "ON" : "OFF"), (unsigned)beE.calibrated, (unsigned long)beE.sum, (unsigned)beE.max, (unsigned)beE.area,

            (leC.contact ? "ON" : "OFF"), (unsigned)leC.calibrated, (unsigned long)leC.sum, (unsigned)leC.max, (unsigned)leC.area
        );

        if (n > 0) {
            (void)HAL_UART_Transmit(&huart1, (uint8_t*)buf, (uint16_t)strlen(buf), 100);
        }

        osDelay(500);
    }
}
