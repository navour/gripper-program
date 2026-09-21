#include "Pressure_Sensor.h"
#include "pressure_history.h"
#include "cmsis_os.h"
#include "Uart1_Print.h"

#include <string.h>
#include <stdint.h>

// ============================ 外部变量 ============================
extern SPI_HandleTypeDef hspi2;
extern SPI_HandleTypeDef hspi3;

extern osSemaphoreId_t SpiRxSemHandle;
SensorData_t sensor_data;

// ============================ SPI 接收缓冲区 ============================
static uint8_t SPI3_RxData[322];
static uint8_t SPI2_RxData[194];

// ============================ SPI 调试变量 ============================
// 记录 SPI 头/尾字节与超时/错误统计
static volatile uint8_t  g_spi3_head = 0, g_spi3_tail = 0, g_spi2_head = 0, g_spi2_tail = 0;
static volatile uint32_t g_spi3_to = 0, g_spi2_to = 0, g_spi3_bad = 0, g_spi2_bad = 0, g_spi3_err = 0, g_spi2_err = 0;
static uint8_t g_spi3_bad_run = 0;
static uint8_t g_spi2_bad_run = 0;


// ============================ 固定参数 ============================
#define PR_N           32U
#define PR_ROWS        8U
#define PR_COLS        4U

// 校准帧数
#ifndef CAL_FRAMES
#define CAL_FRAMES     100U
#endif

// IIR 滤波参数: y += (x - y) >> BETA_SHIFT
// 0 表示不滤波（y=x），1 表示 1/2，2 表示 1/4，3 表示 1/8...
#ifndef BETA_SHIFT
#define BETA_SHIFT     2U
#endif

// 基线跟踪（可选）: base += (raw - base) >> ALPHA_SHIFT
#ifndef ENABLE_BASE_TRACK
#define ENABLE_BASE_TRACK  0U
#endif

// 原始数据调试输出开关
#ifndef PRESS_DEBUG_RAW_PRINT
#define PRESS_DEBUG_RAW_PRINT  0
#endif

#ifndef ALPHA_SHIFT
#define ALPHA_SHIFT    11U
#endif

// 基线保持阈值（无接触时才允许更新）
#ifndef BASE_HOLD_SUM_TH
#define BASE_HOLD_SUM_TH   60U
#endif
#ifndef BASE_HOLD_MAX_TH
#define BASE_HOLD_MAX_TH   12U
#endif

// 需要连续 N 帧无接触才更新基线
#ifndef BASE_IDLE_FRAMES
#define BASE_IDLE_FRAMES   20U
#endif

// 单元触发阈值（用于面积统计）
#ifndef CELL_ON_TH
#define CELL_ON_TH         8U
#endif

// 接触判定阈值
#ifndef CONTACT_SUM_TH
#define CONTACT_SUM_TH     120U
#endif
#ifndef CONTACT_MAX_TH
#define CONTACT_MAX_TH     30U
#endif

// SPI 等待超时
#ifndef SPI_WAIT_TICKS
#define SPI_WAIT_TICKS     50U
#endif

#ifndef PRESS_SPI_RECOVER_BAD_FRAMES
#define PRESS_SPI_RECOVER_BAD_FRAMES  5U
#endif

#ifndef PRESS_SPI_RECOVER_IDLE_TICKS
#define PRESS_SPI_RECOVER_IDLE_TICKS  2U
#endif

// ============================ 压力安全逻辑 ============================
static volatile uint8_t  g_press_latched = 0;
static volatile uint8_t  g_unlock_pending = 0;
static volatile uint8_t  g_rearm_cnt = 0;
static volatile uint32_t g_safe_sum      = 0;
static volatile uint16_t g_safe_max      = 0;
static volatile uint32_t g_sum_th        = 4500;
static volatile uint16_t g_max_th        = 1000;
static volatile uint16_t g_safe_flags    = 0;
static PressureFeature_t g_latest_features[PRESSURE_REGION_COUNT];

/**
 * @brief  查询是否已锁存
 * @param  None
 * @retval 1=已锁存，0=未锁存
 */
uint8_t  PressureSafety_IsLatched(void){ return g_press_latched; }

/**
 * @brief  查询是否被阻止（锁存或解锁等待中）
 * @param  None
 * @retval 1=阻止，0=允许
 */
uint8_t  PressureSafety_IsBlocked(void){ return (uint8_t)(g_press_latched || g_unlock_pending); }

/**
 * @brief  获取安全标志位
 * @param  None
 * @retval 标志位组合值
 */
uint16_t PressureSafety_GetFlags(void){ return g_safe_flags; }

/**
 * @brief  清除锁存并进入解锁等待
 * @param  None
 * @retval None
 */
void PressureSafety_ClearLatch(void){
    g_press_latched = 0;
    g_unlock_pending = 1u; 
    g_rearm_cnt = 0u;
}

/**
 * @brief  设置压力阈值
 * @param  sum_th: 总和阈值
 * @param  max_th: 最大值阈值
 * @retval None
 */
void PressureSafety_SetThreshold(uint32_t sum_th, uint16_t max_th){
    g_sum_th = sum_th;
    g_max_th = max_th;
}

/**
 * @brief  读取压力阈值
 * @param  sum_th: 输出总和阈值
 * @param  max_th: 输出最大值阈值
 * @retval None
 */
void PressureSafety_GetThreshold(uint32_t *sum_th, uint16_t *max_th){
    if(sum_th) *sum_th = g_sum_th;
    if(max_th) *max_th = g_max_th;
}

/**
 * @brief  读取最新压力统计值
 * @param  sum: 输出总和
 * @param  max: 输出最大值
 * @retval None
 */
void PressureSafety_GetLatest(uint32_t *sum, uint16_t *max){
    if(sum) *sum = g_safe_sum;
    if(max) *max = g_safe_max;
}

void PressureSensor_GetLatestFeature(PressureRegion_t region, PressureFeature_t *feat)
{
    uint32_t primask;

    if((feat == 0) || (region >= PRESSURE_REGION_COUNT)){
        return;
    }

    primask = __get_PRIMASK();
    __disable_irq();
    *feat = g_latest_features[region];
    if(!primask){
        __enable_irq();
    }
}

// ============================ 内部状态 ============================
typedef struct {
    uint32_t acc[PR_N];
    uint16_t cal_cnt;
    uint8_t  calibrated;

    uint16_t base[PR_N];
    uint16_t filt[PR_N];

#if ENABLE_BASE_TRACK
    uint16_t idle_cnt;
#endif
} PressureProcState_t;

static PressureProcState_t g_be_c;
static PressureProcState_t g_be_d;
static PressureProcState_t g_be_e;
static PressureProcState_t g_le_c;

// ============================ 内部函数 ============================
static inline uint16_t u16_clamp_i32(int32_t v)
{
    if (v < 0) return 0;
    if (v > 65535) return 65535;
    return (uint16_t)v;
}

/**
 * @brief  SPI 阻塞式读取一帧数据
 * @param  hspi: SPI 句柄
 * @param  cs_port: 片选端口
 * @param  cs_pin: 片选引脚
 * @param  buff: 接收缓冲区
 * @param  size: 接收长度
 * @retval 1=成功，0=失败
 */
static uint8_t SPI_Read_Block(SPI_HandleTypeDef *hspi,
                              GPIO_TypeDef *cs_port,
                              uint16_t cs_pin,
                              uint8_t *buff,
                              uint16_t size)
{
    
    (void)hspi;

    // 清空历史信号量，避免上一次超时残留
    while (osSemaphoreAcquire(SpiRxSemHandle, 0) == osOK) {}

    // 若 SPI 处于 BUSY 状态，先中止并恢复
    if (HAL_SPI_GetState(hspi) != HAL_SPI_STATE_READY) {
        (void)HAL_SPI_Abort_IT(hspi);
        while (osSemaphoreAcquire(SpiRxSemHandle, 0) == osOK) {}
    }

    HAL_GPIO_WritePin(cs_port, cs_pin, GPIO_PIN_RESET);

    if (HAL_SPI_Receive_IT(hspi, buff, size) != HAL_OK) {
        HAL_GPIO_WritePin(cs_port, cs_pin, GPIO_PIN_SET);
        if (hspi->Instance == SPI3) { g_spi3_err++; } else { g_spi2_err++; }
        return 0;
    }

    osStatus_t st = osSemaphoreAcquire(SpiRxSemHandle, SPI_WAIT_TICKS);

    HAL_GPIO_WritePin(cs_port, cs_pin, GPIO_PIN_SET);

    if (st == osOK) {
        // 记录头/尾字节用于调试
        if (hspi->Instance == SPI3) { g_spi3_head = buff[0]; g_spi3_tail = buff[size-1]; }
        else { g_spi2_head = buff[0]; g_spi2_tail = buff[size-1]; }
        return 1;
    }

    // 超时则中止并恢复
    if (hspi->Instance == SPI3) { g_spi3_to++; g_spi3_head = buff[0]; g_spi3_tail = buff[size-1]; }
    else { g_spi2_to++; g_spi2_head = buff[0]; g_spi2_tail = buff[size-1]; }
    (void)HAL_SPI_Abort_IT(hspi);
    while (osSemaphoreAcquire(SpiRxSemHandle, 0) == osOK) {}
    return 0;
}

static void SPI_Recover_Block(SPI_HandleTypeDef *hspi,
                              GPIO_TypeDef *cs_port,
                              uint16_t cs_pin)
{
    HAL_GPIO_WritePin(cs_port, cs_pin, GPIO_PIN_SET);
    (void)HAL_SPI_Abort_IT(hspi);
    while (osSemaphoreAcquire(SpiRxSemHandle, 0) == osOK) {}

    (void)HAL_SPI_DeInit(hspi);
    (void)HAL_SPI_Init(hspi);
    osDelay(PRESS_SPI_RECOVER_IDLE_TICKS);
}

static void SPI_Record_Frame_Result(SPI_HandleTypeDef *hspi,
                                    GPIO_TypeDef *cs_port,
                                    uint16_t cs_pin,
                                    uint8_t frame_ok)
{
    uint8_t *bad_run = (hspi->Instance == SPI3) ? &g_spi3_bad_run : &g_spi2_bad_run;

    if(frame_ok){
        *bad_run = 0u;
        return;
    }

    if(*bad_run < 255u){
        (*bad_run)++;
    }

    if(*bad_run >= PRESS_SPI_RECOVER_BAD_FRAMES){
        SPI_Recover_Block(hspi, cs_port, cs_pin);
        *bad_run = 0u;
    }
}


// 32 路原始数据 -> 特征值
// 计算 (raw - base) 并限幅到 >=0，然后进行 IIR 滤波
static void Pressure_Process32(const uint16_t raw32[PR_N], PressureProcState_t *st, PressureFeat_t *out)
{
    if (!raw32 || !st || !out) return;

    // 0) 校准阶段：累加平均基线
    if (!st->calibrated) {
        if (st->cal_cnt < CAL_FRAMES) {
            for (uint32_t i = 0; i < PR_N; i++) st->acc[i] += raw32[i];
            st->cal_cnt++;
        }

        if (st->cal_cnt >= CAL_FRAMES) {
            for (uint32_t i = 0; i < PR_N; i++) {
                st->base[i] = (uint16_t)(st->acc[i] / (uint32_t)CAL_FRAMES);
                st->filt[i] = 0;
            }
            st->calibrated = 1u;
        }

        out->calibrated = st->calibrated;
#if PRESS_DEBUG_RAW_PRINT
        // RAW 调试：即使在校准期也输出原始 sum/max，便于确认采样是否正常
        {
            uint32_t sum_raw = 0;
            uint16_t max_raw = 0;
            uint8_t  area_raw = 0;
            for (uint32_t i = 0; i < PR_N; i++) {
                uint16_t v = raw32[i];
                sum_raw += (uint32_t)v;
                if (v > max_raw) max_raw = v;
                if (v >= (uint16_t)CELL_ON_TH) area_raw++;
            }
            out->contact = (max_raw > 0) ? 1u : 0u;
            out->sum = sum_raw;
            out->max = max_raw;
            out->area = area_raw;
        }
#else
        out->contact = 0;
        out->sum = 0;
        out->max = 0;
        out->area = 0;
#endif
        out->max_r = 0;
        out->max_c = 0;
        return;
    }

    // 1) 计算差值并滤波
    uint32_t sum = 0;
    uint16_t maxv = 0;
    uint8_t  max_r = 0, max_c = 0;
    uint8_t  area = 0;

    for (uint32_t i = 0; i < PR_N; i++) {
#if PRESS_DEBUG_RAW_PRINT
        // RAW 调试：跳过基线与滤波
        uint16_t y = raw32[i];
        st->filt[i] = y;
#else
        int32_t d = (int32_t)raw32[i] - (int32_t)st->base[i];
        if (d < 0) d = 0;

#if (BETA_SHIFT == 0)
        uint16_t y = (uint16_t)d;
#else
        int32_t y_i = (int32_t)st->filt[i] + ((d - (int32_t)st->filt[i]) >> (int32_t)BETA_SHIFT);
        if (y_i < 0) y_i = 0;
        if (y_i > 65535) y_i = 65535;
        uint16_t y = (uint16_t)y_i;
#endif
        st->filt[i] = y;
#endif

        sum += (uint32_t)y;
        if (y > maxv) {
            maxv = y;
            max_r = (uint8_t)(i / PR_COLS);
            max_c = (uint8_t)(i % PR_COLS);
        }
        if (y >= (uint16_t)CELL_ON_TH) area++;
    }

    uint8_t contact = (sum >= (uint32_t)CONTACT_SUM_TH || maxv >= (uint16_t)CONTACT_MAX_TH) ? 1u : 0u;

#if ENABLE_BASE_TRACK
    // 2) 基线跟踪
    if (sum < (uint32_t)BASE_HOLD_SUM_TH && maxv < (uint16_t)BASE_HOLD_MAX_TH) {
        if (st->idle_cnt < 0xFFFFu) st->idle_cnt++;
    } else {
        st->idle_cnt = 0u;
    }

    if (st->idle_cnt >= (uint16_t)BASE_IDLE_FRAMES) {
        for (uint32_t i = 0; i < PR_N; i++) {
            int32_t diff = (int32_t)raw32[i] - (int32_t)st->base[i];
            /* signed shift */
            int32_t step = (diff >> (int32_t)ALPHA_SHIFT);
            st->base[i] = u16_clamp_i32((int32_t)st->base[i] + step);
        }
    }
#endif

    out->calibrated = 1u;
    out->contact = contact;
    out->sum = sum;
    out->max = maxv;
    out->max_r = max_r;
    out->max_c = max_c;
    out->area = area;
}

// 聚合三路 BE 特征
static void Pressure_Agg3(const PressureFeat_t *c, const PressureFeat_t *d, const PressureFeat_t *e, PressureFeat_t *out)
{
    if (!c || !d || !e || !out) return;

    out->calibrated = (uint8_t)(c->calibrated && d->calibrated && e->calibrated);
    out->contact    = (uint8_t)(c->contact || d->contact || e->contact);
    out->sum        = c->sum + d->sum + e->sum;
    out->area       = (uint8_t)(c->area + d->area + e->area);

    // 取三路中的最大值
    const PressureFeat_t *m = c;
    if (d->max > m->max) m = d;
    if (e->max > m->max) m = e;

    out->max   = m->max;
    out->max_r = m->max_r;
    out->max_c = m->max_c;
}

static void Pressure_Agg2(const PressureFeat_t *be_sum, const PressureFeat_t *le_c, PressureFeat_t *out)
{
    if (!be_sum || !le_c || !out) return;

    out->calibrated = (uint8_t)(be_sum->calibrated && le_c->calibrated);
    out->contact    = (uint8_t)(be_sum->contact || le_c->contact);
    out->sum        = be_sum->sum + le_c->sum;
    out->area       = (uint8_t)(be_sum->area + le_c->area);

    if (le_c->max > be_sum->max) {
        out->max = le_c->max;
        out->max_r = le_c->max_r;
        out->max_c = le_c->max_c;
    } else {
        out->max = be_sum->max;
        out->max_r = be_sum->max_r;
        out->max_c = be_sum->max_c;
    }
}

static void Pressure_PublishFeature(PressureRegion_t region, const PressureFeat_t *src)
{
    PressureFeature_t feat;
    uint32_t primask;

    if((src == 0) || (region >= PRESSURE_REGION_COUNT)){
        return;
    }

    feat.calibrated = src->calibrated;
    feat.contact = src->contact;
    feat.sum = src->sum;
    feat.max = src->max;
    feat.area = src->area;

    primask = __get_PRIMASK();
    __disable_irq();
    g_latest_features[region] = feat;
    if(!primask){
        __enable_irq();
    }
}

// ============================ 对外接口 ============================
/**
 * @brief  压力传感器初始化
 * @param  None
 * @retval None
 */
void PressureSensor_Init(void)
{
    /* SPI3 CS: PB6, SPI2 CS: PB12 */
    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_6, GPIO_PIN_SET);
    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_12, GPIO_PIN_SET);
}

/**
 * @brief  更新压力数据并通过 UART1 打印
 * @param  None
 * @retval None
 */
void PressureSensor_UpdateAndPrint(void)
{
    static uint32_t history_be_ms, history_le_ms;
    PressureHistory_BeginCycle();
    PressureFeat_t be_c = {0}, be_d = {0}, be_e = {0};
    PressureFeat_t le_c = {0};
    PressureFeat_t be_sum = {0};
    PressureFeat_t total  = {0};

    // 若 SPI 异常，复用上一次有效数据
    static PressureFeat_t s_last_be_c = {0}, s_last_be_d = {0}, s_last_be_e = {0};
    static PressureFeat_t s_last_le_c = {0};
    static PressureFeat_t s_last_be_sum = {0};
    static uint8_t s_have_be = 0, s_have_le = 0;

    // ==================== SPI3: BE (5x32) ====================
    uint8_t spi3_ok = SPI_Read_Block(&hspi3, GPIOB, GPIO_PIN_6, SPI3_RxData, (uint16_t)sizeof(SPI3_RxData));
    uint8_t spi3_frame_ok;
    g_spi3_head = SPI3_RxData[0];
    g_spi3_tail = SPI3_RxData[321];
    if (spi3_ok && (SPI3_RxData[0] != 0xA5 || SPI3_RxData[321] != 0x5A)) { g_spi3_bad++; }
    spi3_frame_ok = (uint8_t)(spi3_ok && SPI3_RxData[0] == 0xA5 && SPI3_RxData[321] == 0x5A);
    SPI_Record_Frame_Result(&hspi3, GPIOB, GPIO_PIN_6, spi3_frame_ok);

    if (spi3_frame_ok)
    {
        history_be_ms = HAL_GetTick();
        memcpy(sensor_data.BE_arrayC, SPI3_RxData + 129, 64);
        memcpy(sensor_data.BE_arrayD, SPI3_RxData + 193, 64);
        memcpy(sensor_data.BE_arrayE, SPI3_RxData + 257, 64);

        Pressure_Process32(sensor_data.BE_arrayC, &g_be_c, &be_c);
        Pressure_Process32(sensor_data.BE_arrayD, &g_be_d, &be_d);
        Pressure_Process32(sensor_data.BE_arrayE, &g_be_e, &be_e);

        Pressure_Agg3(&be_c, &be_d, &be_e, &be_sum);

        s_last_be_c = be_c;
        s_last_be_d = be_d;
        s_last_be_e = be_e;
        s_last_be_sum = be_sum;
        s_have_be = 1u;
    }
    else
    {
        be_c.calibrated = g_be_c.calibrated;
        be_d.calibrated = g_be_d.calibrated;
        be_e.calibrated = g_be_e.calibrated;
        be_sum.calibrated = (uint8_t)(g_be_c.calibrated && g_be_d.calibrated && g_be_e.calibrated);

        if (s_have_be) {
            be_c = s_last_be_c;
            be_d = s_last_be_d;
            be_e = s_last_be_e;
            be_sum = s_last_be_sum;
        }
    }

    // ==================== SPI2: LE (3x32) ====================
    uint8_t spi2_ok = SPI_Read_Block(&hspi2, GPIOB, GPIO_PIN_12, SPI2_RxData, (uint16_t)sizeof(SPI2_RxData));
    uint8_t spi2_frame_ok;
    g_spi2_head = SPI2_RxData[0];
    g_spi2_tail = SPI2_RxData[193];
    if (spi2_ok && (SPI2_RxData[0] != 0xA5 || SPI2_RxData[193] != 0x5A)) { g_spi2_bad++; }
    spi2_frame_ok = (uint8_t)(spi2_ok && SPI2_RxData[0] == 0xA5 && SPI2_RxData[193] == 0x5A);
    SPI_Record_Frame_Result(&hspi2, GPIOB, GPIO_PIN_12, spi2_frame_ok);

    if (spi2_frame_ok)
    {
        history_le_ms = HAL_GetTick();
        memcpy(sensor_data.LE_arrayC, SPI2_RxData + 129, 64);

        Pressure_Process32(sensor_data.LE_arrayC, &g_le_c, &le_c);

        s_last_le_c = le_c;
        s_have_le = 1u;
    }
    else
    {
        le_c.calibrated = g_le_c.calibrated;
        if (s_have_le) {
            le_c = s_last_le_c;
        }
    }

    // ==================== 汇总 ====================
    Pressure_Agg2(&be_sum, &le_c, &total);

    // ==================== 安全判断 ====================
    g_safe_sum = total.sum;
    g_safe_max = total.max;

    uint8_t sum_over = (total.calibrated && (total.sum >= g_sum_th)) ? 1u : 0u;
    uint8_t max_over = (total.calibrated && (total.max >= g_max_th)) ? 1u : 0u;

    if (g_unlock_pending) {
        uint32_t re_sum = (g_sum_th >= 4u) ? (g_sum_th / 4u) : 0u;
        uint16_t re_max = (uint16_t)(g_max_th / 4u);

        if (total.calibrated && (total.sum < re_sum) && (total.max < re_max)) {
            if (++g_rearm_cnt >= 5u) {
                g_unlock_pending = 0u;
                g_rearm_cnt = 0u;
            }
        } else {
            g_rearm_cnt = 0u;
        }
    } else {
        if (!g_press_latched && (sum_over || max_over)) {
            g_press_latched = 1u;
        }
    }

    g_safe_flags = (uint16_t)((g_press_latched ? PRESS_SAFE_FLAG_LATCHED : 0u) |
                             (sum_over        ? PRESS_SAFE_FLAG_SUM_OVER : 0u) |
                             (max_over        ? PRESS_SAFE_FLAG_MAX_OVER : 0u) |
                             (g_unlock_pending? PRESS_SAFE_FLAG_ARMING   : 0u));

    // ==================== 通过 UART1 打印 ====================
    /* Archive raw counts after the existing pressure safety calculation. */
    PressureHistory_PushCycle(
        sensor_data.BE_arrayC, sensor_data.BE_arrayD, sensor_data.BE_arrayE, sensor_data.LE_arrayC,
        history_be_ms, history_le_ms,
        (uint16_t)(spi3_frame_ok | (spi2_frame_ok << 1) | (s_have_be << 2) | (s_have_le << 3)),
        (uint16_t)((g_be_c.calibrated << 0) | (g_be_d.calibrated << 1) | (g_be_e.calibrated << 2) | (g_le_c.calibrated << 3)));

    Pressure_PublishFeature(PRESSURE_REGION_TOTAL,  &total);
    Pressure_PublishFeature(PRESSURE_REGION_BE_SUM, &be_sum);
    Pressure_PublishFeature(PRESSURE_REGION_BE_C,   &be_c);
    Pressure_PublishFeature(PRESSURE_REGION_BE_D,   &be_d);
    Pressure_PublishFeature(PRESSURE_REGION_BE_E,   &be_e);
    Pressure_PublishFeature(PRESSURE_REGION_LE_C,   &le_c);

    Uart1_Print_UpdatePressure(PRESSURE_SIDE_BE_C,   &be_c);
    Uart1_Print_UpdatePressure(PRESSURE_SIDE_BE_D,   &be_d);
    Uart1_Print_UpdatePressure(PRESSURE_SIDE_BE_E,   &be_e);
    Uart1_Print_UpdatePressure(PRESSURE_SIDE_LE,     &le_c);
    Uart1_Print_UpdatePressure(PRESSURE_SIDE_BE,     &be_sum);
    Uart1_Print_UpdatePressure(PRESSURE_SIDE_TOTAL,  &total);
}
