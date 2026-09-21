#include "pressure_history.h"
#include "dm_modbus_bridge.h"
#include "dm_motor_ctrl.h"
#include "dm_motor_drv.h"
#include "pos_persist.h"
#include "AS5048A.h"
#include "Pressure_Sensor.h"
#include <string.h>
#include <stdint.h>
#include "rs485.h"
#include "force_estimator.h"
#include "gripper_ota_app.h"
#include "gripper_ota_layout.h"

#ifndef DEG2RADf
#define DEG2RADf(x) ((x) * 0.017453292519943295f)
#endif


#define BRIDGE_PI      (3.14159265359f)
#define BRIDGE_TWO_PI  (6.28318530718f)


/* ---------- 工具函数 ---------- */
static float f_abs(float x){ return (x < 0.0f) ? -x : x; }

#ifndef RS485_EN_W
#define RS485_EN_W ((void)0)
#define RS485_EN_R ((void)0)
#endif

/* ------------------------ 常量与参数 ------------------------ */
#define BRIDGE_SLAVE_ADDR   0x02u
#define RX_BUF_SZ           256
#define TX_BUF_SZ           260

#ifndef BRIDGE_EST_UPDATE_PERIOD_MS
#define BRIDGE_EST_UPDATE_PERIOD_MS    20u
#endif

#ifndef BRIDGE_FB_UPDATE_PERIOD_MS
#define BRIDGE_FB_UPDATE_PERIOD_MS     50u
#endif

#ifndef BRIDGE_GOAL_TRACK_PERIOD_MS
#define BRIDGE_GOAL_TRACK_PERIOD_MS    20u
#endif

#ifndef BRIDGE_GOAL_TRACK_DEADBAND_DEG
#define BRIDGE_GOAL_TRACK_DEADBAND_DEG (0.5f)
#endif

#ifndef BRIDGE_POS_SCALE_DEFAULT
#define BRIDGE_POS_SCALE_DEFAULT  (26.5f)
#endif

#ifndef BRIDGE_AS_HOST_MIN_DEG
#define BRIDGE_AS_HOST_MIN_DEG    (-45.0f)
#endif

#ifndef BRIDGE_AS_HOST_MAX_DEG
#define BRIDGE_AS_HOST_MAX_DEG    (8.0f)
#endif

#ifndef BRIDGE_AS_RANGE_BYPASS_MS
#define BRIDGE_AS_RANGE_BYPASS_MS (120000u)
#endif

#ifndef BRIDGE_FIRST_MOVE_VEL_DEG
#define BRIDGE_FIRST_MOVE_VEL_DEG (5.0f)
#endif

#ifndef BRIDGE_FIRST_MOVE_KP_MAX
#define BRIDGE_FIRST_MOVE_KP_MAX  (0.20f)
#endif

#ifndef BRIDGE_AS_MOTOR_STEP_MAX_RAD
#define BRIDGE_AS_MOTOR_STEP_MAX_RAD (2.0f)
#endif

#ifndef BRIDGE_USE_AS_VELOCITY_LOOP
#define BRIDGE_USE_AS_VELOCITY_LOOP 1
#endif

#ifndef BRIDGE_AS_VEL_KP
#define BRIDGE_AS_VEL_KP              (8.0f)
#endif

#ifndef BRIDGE_AS_VEL_MAX_DEG
#define BRIDGE_AS_VEL_MAX_DEG         (45.0f)
#endif

#ifndef BRIDGE_AS_POS_DEADBAND_DEG
#define BRIDGE_AS_POS_DEADBAND_DEG    (0.5f)
#endif

#ifndef BRIDGE_AS_HOLD_KD
#define BRIDGE_AS_HOLD_KD             (0.3f)
#endif

#ifndef BRIDGE_MOTOR_POS_LIMIT_MARGIN
#define BRIDGE_MOTOR_POS_LIMIT_MARGIN  (0.20f)
#endif

#ifndef POS_SAVE_PERIOD_MS
#define POS_SAVE_PERIOD_MS   (5000u)
#endif
#ifndef POS_SAVE_DELTA_RAD
#define POS_SAVE_DELTA_RAD   (0.02f)
#endif

#ifndef BRIDGE_CONTACT_HOLD_EPS_DEG
#define BRIDGE_CONTACT_HOLD_EPS_DEG  (0.5f)
#endif

#ifndef BRIDGE_GRIP_HOLD_VEL_TH
#define BRIDGE_GRIP_HOLD_VEL_TH      (0.30f)
#endif

#ifndef BRIDGE_GRIP_HOLD_CONFIRM_CNT
#define BRIDGE_GRIP_HOLD_CONFIRM_CNT (3u)
#endif

#ifndef BRIDGE_GRIP_HOLD_MIN_TRAVEL_DEG
#define BRIDGE_GRIP_HOLD_MIN_TRAVEL_DEG (1.5f)
#endif

#define BRIDGE_OTA_KEY1_VALUE       (0x4F54u)
#define BRIDGE_OTA_KEY2_VALUE       (0x4131u)
#define BRIDGE_OTA_KEY_WINDOW_MS    (1000u)
#define BRIDGE_OTA_ACK_TIMEOUT_MS   (200u)
#define BRIDGE_OTA_RESET_DELAY_MS   (3u)

/* 单位/缩放配置位：与上位机当前协议保持一致。 */
#define UNIT_POS_IS_DEG   (1u<<0)
#define UNIT_VEL_IS_DEG   (1u<<1)
#define SCALE_POS_X1      (1u<<2)
#define SCALE_VEL_X1      (1u<<3)

#define BRIDGE_AS_RANGE_BYPASS_MAGIC (0xA55Au)

/* ------------------------ 寄存器映射 ------------------------ */
enum {
    REG_DEV_ID        = 0x0000,
    REG_FW_VER        = 0x0001,
    REG_MOTOR_ID      = 0x0010,
    REG_MODE          = 0x0011,
    REG_ENABLE        = 0x0012,
    REG_CLEAR_FAULT   = 0x0013,
    REG_UNIT_CFG      = 0x0014,
    REG_SAVE_ZERO     = 0x0016,
    REG_SAVE_MOTOR_ZERO = 0x0017,

    REG_PERSIST_VALID            = 0x0018,
    REG_PERSIST_LOADED_POS_H     = 0x0019,
    REG_PERSIST_LOADED_POS_L     = 0x001A,
    REG_PERSIST_LOADED_SCALE     = 0x001B,
    REG_PERSIST_LAST_SAVE_ST     = 0x001C,
    REG_PERSIST_SAVE_OK_CNT      = 0x001D,
    REG_PERSIST_BIAS_MRAD_H      = 0x001E,
    REG_PERSIST_BIAS_MRAD_L      = 0x001F,

    REG_V_DES_H       = 0x0020,
    REG_V_DES_L       = 0x0021,
    REG_P_DES_H       = 0x0022,
    REG_P_DES_L       = 0x0023,
    REG_KP_1_100      = 0x0024,
    REG_KD_1_100      = 0x0025,
    REG_TFF_H         = 0x0026,
    REG_TFF_L         = 0x0027,

    REG_FB_POS_H      = 0x0100,
    REG_FB_POS_L      = 0x0101,
    REG_FB_ERR        = 0x0103,
    REG_FB_MOS_TEMP   = 0x0104,
    REG_FB_ROTOR_TEMP = 0x0105,

    REG_SAFE_SUM_TH_H = 0x0030,
    REG_SAFE_SUM_TH_L = 0x0031,
    REG_SAFE_MAX_TH   = 0x0032,
    REG_SAFE_UNLOCK   = 0x0034,

    REG_SAFE_FLAGS    = 0x0106,
    REG_SAFE_SUM_H    = 0x0107,
    REG_SAFE_SUM_L    = 0x0108,
    REG_SAFE_MAX      = 0x0109,

    REG_FB_TOR_RAW_H  = 0x010A,
    REG_FB_TOR_RAW_L  = 0x010B,
    REG_FB_TOR_FILT_H = 0x010C,
    REG_FB_TOR_FILT_L = 0x010D,
    REG_FB_TOR_EXT_H  = 0x010E,
    REG_FB_TOR_EXT_L  = 0x010F,
    REG_FB_FORCE_H    = 0x0110,
    REG_FB_FORCE_L    = 0x0111,
    REG_FB_CONTACT_FLAG = 0x0112,
    REG_FB_TOR_FRIC_H = 0x0113,
    REG_FB_TOR_FRIC_L = 0x0114,
    REG_FB_AS_ABS_H   = 0x0115,
    REG_FB_AS_ABS_L   = 0x0116,
    REG_FB_AS_STATUS  = 0x0117,
    REG_FB_MOTOR_POS_H  = 0x0118,
    REG_FB_MOTOR_POS_L  = 0x0119,
    REG_FB_MOTOR_CMD_H  = 0x011A,
    REG_FB_MOTOR_CMD_L  = 0x011B,
    REG_FB_MOTOR_RAW_H  = 0x011C,
    REG_FB_MOTOR_RAW_L  = 0x011D,
    REG_FB_MOTOR_PMAX   = 0x011E,
    REG_FB_MOTOR_CLAMP  = 0x011F,
    REG_FB_GOAL_STATUS = 0x0120,

    REG_EST_TOR_TH      = 0x0036,
    REG_EST_VEL_IDLE_TH = 0x0037,
    REG_EST_ALPHA       = 0x0038,
    REG_EST_BIAS_LEARN  = 0x0039,
    REG_EST_FRIC_B      = 0x003A,
    REG_EST_COULOMB_POS = 0x003B,
    REG_EST_COULOMB_NEG = 0x003C,
    REG_EST_STATIC_POS  = 0x003D,
    REG_EST_STATIC_NEG  = 0x003E,
    REG_EST_VEL_SIGN_TH = 0x003F,
    REG_EST_CONTACT_DIR = 0x0040,
    REG_DM_PMAX_MRAD    = 0x0041,
    REG_DM_PMAX_APPLY   = 0x0042,
    REG_EST_BASE_OPEN_H  = 0x0043,
    REG_EST_BASE_OPEN_L  = 0x0044,
    REG_EST_BASE_CLOSE_H = 0x0045,
    REG_EST_BASE_CLOSE_L = 0x0046,
    REG_EST_GRIP_TH      = 0x0047,
    REG_AS_RANGE_BYPASS  = 0x0048,

    REG_OTA_KEY1         = 0x00F0,
    REG_OTA_KEY2         = 0x00F1,
    REG_OTA_CTRL         = 0x00F2,
    REG_BOOT_VER         = 0x00F3,
    REG_OTA_CAPS         = 0x00F4,
    REG_APP_BUILD        = 0x00F5,
    REG_APP_PATCH        = 0x00F6,
    REG_APP_READY        = 0x00F7,
    REG_LAYOUT_VERSION   = 0x00F8,
    REG_APP_ROLE         = 0x00F9,

    REG_FB_FRIC_RAW_H    = 0x0121,
    REG_FB_FRIC_RAW_L    = 0x0122,
    REG_FB_BASELINE_H    = 0x0123,
    REG_FB_BASELINE_L    = 0x0124,
    REG_FB_CONTACT_RAW_H = 0x0125,
    REG_FB_CONTACT_RAW_L = 0x0126,
    REG_FB_CONTACT_NET_H = 0x0127,
    REG_FB_CONTACT_NET_L = 0x0128,
    REG_FB_VEL_EST_H     = 0x0129,
    REG_FB_VEL_EST_L     = 0x012A,
    REG_FB_POS_EST_H     = 0x012B,
    REG_FB_POS_EST_L     = 0x012C,
    REG_FB_PRESS_BASE    = 0x0130,
};

#define BRIDGE_PRESS_FEAT_REGS 5u
#define REG_FB_PRESS_LAST \
    (REG_FB_PRESS_BASE + (PRESSURE_REGION_COUNT * BRIDGE_PRESS_FEAT_REGS) - 1u)

/* ------------------------ 内部状态 ------------------------ */
static UART_HandleTypeDef *s_huart = NULL;
static FDCAN_HandleTypeDef *s_hcan  = NULL;

static uint8_t  s_rxbuf[RX_BUF_SZ];
static uint16_t s_rxlen = 0;

static uint8_t  s_txbuf[TX_BUF_SZ];
static uint16_t s_txlen = 0;
static volatile uint8_t s_tx_pending = 0;
static volatile uint8_t s_tx_busy     = 0;
static uint32_t s_tx_started_ms = 0u;

static uint16_t s_regs[0x200];

static float s_pos_scale = BRIDGE_POS_SCALE_DEFAULT;

/* ------------------------ AS5048A 零点与 host/motor 坐标映射 ------------------------ */
static uint8_t  s_bias_ready = 0;
static float    s_host_bias = 0.0f;        /* host 机械坐标 = motor/scale + bias */
static uint8_t  s_bias_from_as = 0;
static uint8_t  s_bias_from_persist = 0;
static uint8_t  s_boot_as_rebind_done = 0;
static uint8_t  s_as_range_ok = 0;
static uint8_t  s_as_range_bypass = 0;
static uint32_t s_as_range_bypass_started_ms = 0u;
static uint8_t  s_first_motion_soft_limit = 0;

/* AS5048A 机械零点为绝对角，来自 Flash，减少重复写入。 */
static uint8_t  s_zero_valid = 0;
static float    s_zero_abs_rad = 0.0f;     /* rad, 0..2*pi */

/* 保留从 Flash 加载的 scale，用于 host<->motor 坐标映射。 */
static float    s_loaded_scale = 1.0f;

/* Save 成功计数，同步到寄存器 0x001D。 */
static uint16_t s_save_ok_cnt = 0;

typedef struct {
    uint8_t valid;
    float v, p, kp, kd, tff;
} goal_cache_t;

typedef struct {
    float host_pos;
    float vel;
    float tor;
} estimator_sample_t;

static goal_cache_t s_goal = {0};
static uint8_t s_goal_cmd_seen = 0u;
static estimator_sample_t s_estimator_sample = {0};
static volatile uint8_t s_estimator_sample_pending = 0u;
static float s_dbg_motor_raw_target = 0.0f;
static float s_dbg_motor_cmd_target = 0.0f;
static uint8_t s_dbg_motor_clamped = 0;
static uint16_t s_dbg_goal_status = 0;
static uint8_t s_contact_hold_active = 0;
static float s_contact_hold_pos = 0.0f;
static uint8_t s_grip_hold_cnt = 0u;
static uint8_t s_closing_motion_seen = 0u;
static uint8_t s_grip_contact_armed = 0u;
static float s_grip_motion_start_pos = 0.0f;
static uint32_t s_enable_retry_until = 0u;
static uint32_t s_enable_retry_last = 0u;
static uint32_t s_estimator_last_update = 0u;
static uint32_t s_feedback_last_update = 0u;
static uint32_t s_goal_track_last_update = 0u;

static uint8_t s_ota_key_stage = 0u;
static uint32_t s_ota_key_started_ms = 0u;
static uint8_t s_ota_reset_requested = 0u;
static uint8_t s_ota_reply_sent = 0u;
static uint32_t s_ota_request_started_ms = 0u;
static uint32_t s_ota_reply_sent_ms = 0u;
static uint32_t s_ota_session_id = 0u;


static int32_t f_to_i32_x1000(float x);
static float u16_to_f_x1000(uint16_t v);
static void estimator_sync_param_to_regs(void);
static void persist_regs_on_bias_ready(float bias_rad);
static void bridge_apply_dm_pmax_setting(uint8_t mode);
static void estimator_sync_regs_to_param(void);
static void estimator_update_feedback_regs(void);
static inline void put_u32(uint16_t reg_h, uint32_t v);
static void put_i32(uint16_t reg_h, int32_t v);
static uint8_t as_is_valid(void);
static float as_abs_rad_now(void);
static float host_pos_from_as(void);
static uint8_t bridge_as_range_bypass_active(void);
static void bridge_set_as_range_bypass(uint8_t active);
static uint8_t bridge_as_ready_for_motion(void);
static void bridge_set_motor_passive(void);
static float motor_pos_limit_abs(void);
#if !BRIDGE_USE_AS_VELOCITY_LOOP
static void bridge_apply_goal(const goal_cache_t *g);
#endif
static void bridge_apply_goal_as_velocity(const goal_cache_t *g);
static void bridge_apply_goal_dispatch(const goal_cache_t *g);
static void bridge_parse_goal_from_regs_and_apply(void);
static void bridge_schedule_enable_retry(uint32_t now_ms);
static void bridge_queue_estimator_sample(float host_pos, float vel, float tor);
static uint8_t bridge_take_estimator_sample(estimator_sample_t *sample);
static uint8_t bridge_grip_hold_confirm(float host_now, float target_pos);
static void bridge_update_grip_hold(float host_pos);
static void bridge_update_extended_feedback_regs(void);
static void bridge_track_goal_from_as(uint32_t now_ms);
static void bridge_service_uart_tx(uint32_t now_ms);
static uint8_t bridge_ota_handle_write(uint16_t reg, uint16_t value);
static void bridge_ota_poll(uint32_t now_ms);

static void bridge_schedule_enable_retry(uint32_t now_ms)
{
    s_enable_retry_until = now_ms + 300u;
    s_enable_retry_last = 0u;
}

static void bridge_queue_estimator_sample(float host_pos, float vel, float tor)
{
    s_estimator_sample.host_pos = host_pos;
    s_estimator_sample.vel = vel;
    s_estimator_sample.tor = tor;
    s_estimator_sample_pending = 1u;
}

static uint8_t bridge_take_estimator_sample(estimator_sample_t *sample)
{
    uint8_t have_sample = 0u;
    uint32_t primask;

    if(sample == NULL) return 0u;

    primask = __get_PRIMASK();
    __disable_irq();
    if(s_estimator_sample_pending){
        *sample = s_estimator_sample;
        s_estimator_sample_pending = 0u;
        have_sample = 1u;
    }
    if(!primask){
        __enable_irq();
    }

    return have_sample;
}

static void bridge_update_grip_hold(float host_pos)
{
    if(s_goal.valid &&
       s_regs[REG_ENABLE] &&
       !s_contact_hold_active &&
       bridge_grip_hold_confirm(host_pos, s_goal.p))
    {
        s_contact_hold_active = 1u;
        s_contact_hold_pos = host_pos;
        s_dbg_goal_status |= (1u << 9);
        bridge_apply_goal_dispatch(&s_goal);
    }
}

static void bridge_update_extended_feedback_regs(void)
{
    uint16_t region;

    estimator_update_feedback_regs();
    put_i32(REG_FB_AS_ABS_H, (int32_t)(as_abs_rad_now() * 1000.0f));
    s_regs[REG_FB_AS_STATUS] = as_is_valid() ? 1u : 0u;
    put_i32(REG_FB_MOTOR_POS_H, f_to_i32_x1000(motor[Motor1].para.pos));
    put_i32(REG_FB_MOTOR_CMD_H, f_to_i32_x1000(s_dbg_motor_cmd_target));
    put_i32(REG_FB_MOTOR_RAW_H, f_to_i32_x1000(s_dbg_motor_raw_target));
    s_regs[REG_FB_MOTOR_PMAX] = (uint16_t)(motor_pos_limit_abs() * 1000.0f);
    s_regs[REG_FB_MOTOR_CLAMP] = s_dbg_motor_clamped;
    s_dbg_goal_status &= (uint16_t)~((1u << 10) | (1u << 11) | (1u << 12));
    if(!bridge_as_ready_for_motion()){
        s_dbg_goal_status |= (1u << 10);
    }
    if(s_first_motion_soft_limit){
        s_dbg_goal_status |= (1u << 11);
    }
    if(bridge_as_range_bypass_active()){
        s_dbg_goal_status |= (1u << 12);
    }
    s_regs[REG_FB_GOAL_STATUS] = s_dbg_goal_status;

    for(region = 0u; region < PRESSURE_REGION_COUNT; region++){
        PressureFeature_t feat;
        uint16_t base = (uint16_t)(REG_FB_PRESS_BASE + (region * BRIDGE_PRESS_FEAT_REGS));

        PressureSensor_GetLatestFeature((PressureRegion_t)region, &feat);
        s_regs[base] = (uint16_t)((feat.calibrated ? 1u : 0u) |
                                  (feat.contact ? 2u : 0u));
        put_u32((uint16_t)(base + 1u), feat.sum);
        s_regs[base + 3u] = feat.max;
        s_regs[base + 4u] = feat.area;
    }
}

static void bridge_track_goal_from_as(uint32_t now_ms)
{
    if((s_goal_track_last_update != 0u) &&
       ((now_ms - s_goal_track_last_update) < BRIDGE_GOAL_TRACK_PERIOD_MS)){
        return;
    }
    s_goal_track_last_update = now_ms;

    if(!s_goal.valid ||
       !s_regs[REG_ENABLE] ||
       s_contact_hold_active ||
       PressureSafety_IsBlocked() ||
       !bridge_as_ready_for_motion()){
        return;
    }

    bridge_apply_goal_dispatch(&s_goal);
}

static void bridge_grip_hold_reset(void)
{
    s_grip_hold_cnt = 0u;
    s_closing_motion_seen = 0u;
    s_grip_contact_armed = 0u;
    s_grip_motion_start_pos = 0.0f;
}

static uint8_t bridge_target_is_closing(float host_now, float target_pos)
{
    return (target_pos > (host_now + DEG2RADf(BRIDGE_CONTACT_HOLD_EPS_DEG))) ? 1u : 0u;
}

static uint8_t bridge_target_is_opening_from_hold(float target_pos)
{
    return (target_pos < (s_contact_hold_pos - DEG2RADf(BRIDGE_CONTACT_HOLD_EPS_DEG))) ? 1u : 0u;
}

static uint8_t bridge_grip_hold_confirm(float host_now, float target_pos)
{
    force_estimator_state_t s;
    force_estimator_param_t p;

    ForceEstimator_GetState(&s);
    ForceEstimator_GetParam(&p);

    if (!bridge_target_is_closing(host_now, target_pos)) {
        bridge_grip_hold_reset();
        return 0u;
    }

    if (f_abs(s.vel_rad_s) > BRIDGE_GRIP_HOLD_VEL_TH) {
        if (!s_closing_motion_seen) {
            s_grip_motion_start_pos = host_now;
        }
        s_closing_motion_seen = 1u;
    }

    if (s_closing_motion_seen &&
        (f_abs(host_now - s_grip_motion_start_pos) >= DEG2RADf(BRIDGE_GRIP_HOLD_MIN_TRAVEL_DEG)) &&
        s.contact_flag)
    {
        s_grip_contact_armed = 1u;
    }

    if ((p.grip_hold_th <= 0.0f) ||
        !s_grip_contact_armed ||
        (s.force_proxy < p.grip_hold_th))
    {
        s_grip_hold_cnt = 0u;
        return 0u;
    }

    if (s_grip_hold_cnt < 255u) {
        s_grip_hold_cnt++;
    }

    return (s_grip_hold_cnt >= BRIDGE_GRIP_HOLD_CONFIRM_CNT) ? 1u : 0u;
}

static float safe_scale(void){
    return (f_abs(s_pos_scale) < 1e-6f) ? 1.0f : s_pos_scale;
}

static float clampf_bridge(float x, float lo, float hi){
    if (x < lo) return lo;
    if (x > hi) return hi;
    return x;
}

static float motor_pos_limit_abs(void){
    float pmax = motor[Motor1].tmp.PMAX;
    if (pmax < 1.0f) pmax = 12.5f;
    if (pmax > BRIDGE_MOTOR_POS_LIMIT_MARGIN) {
        pmax -= BRIDGE_MOTOR_POS_LIMIT_MARGIN;
    }
    return pmax;
}

static void bridge_apply_dm_pmax_setting(uint8_t mode){
    float pmax = ((float)s_regs[REG_DM_PMAX_MRAD]) / 1000.0f;

    if (pmax < 1.0f) pmax = 1.0f;
    if (pmax > 20.0f) pmax = 20.0f;

    motor[Motor1].tmp.PMAX = pmax;
    s_regs[REG_DM_PMAX_MRAD] = (uint16_t)(pmax * 1000.0f + 0.5f);

    if (mode >= 2u) {
        float_type_u y;
        y.f_val = pmax;
        write_motor_data(motor[Motor1].id, RID_PMAX, y.b_val[0], y.b_val[1], y.b_val[2], y.b_val[3]);
        if (mode >= 3u) {
            save_motor_data(motor[Motor1].id, RID_PMAX);
        }
    }

    s_regs[REG_DM_PMAX_APPLY] = 0u;

    if (s_goal.valid && s_regs[REG_ENABLE]) {
        bridge_parse_goal_from_regs_and_apply();
    }
}

/* motor 坐标 -> host 机械坐标 */
static float motor_to_host_pos(float motor_pos_rad){
    return (motor_pos_rad / safe_scale()) + s_host_bias;
}

#if !BRIDGE_USE_AS_VELOCITY_LOOP
static float clamp_motor_pos_cmd(float motor_pos_rad){
    float lim = motor_pos_limit_abs();
    return clampf_bridge(motor_pos_rad, -lim, lim);
}

/* host 机械坐标 -> motor 坐标 */
static float host_to_motor_pos(float host_pos_rad){
    return safe_scale() * (host_pos_rad - s_host_bias);
}
#endif

/* -------- AS5048A 机械坐标 --------
 * 说明：angle_deg 由 AS5048ATask 周期更新。
 * host 机械坐标以保存的 s_zero_abs_rad 为 0，并包装到 [-pi, pi]。
 */
static float wrap_pm_pi(float x){
    while (x >  BRIDGE_PI)     x -= BRIDGE_TWO_PI;
    while (x < -BRIDGE_PI)     x += BRIDGE_TWO_PI;
    return x;
}

static uint8_t as_is_valid(void){
    return (angle_valid != 0u);
}

static float as_abs_rad_now(void){
    /* angle_deg: 0..360 */
    return DEG2RADf(angle_deg);
}

static float host_pos_from_as(void){
    float abs = as_abs_rad_now();
    return wrap_pm_pi(abs - s_zero_abs_rad);
}

static uint8_t bridge_as_host_in_range(float host_pos){
    return (host_pos >= DEG2RADf(BRIDGE_AS_HOST_MIN_DEG)) &&
           (host_pos <= DEG2RADf(BRIDGE_AS_HOST_MAX_DEG));
}

static uint8_t bridge_as_range_bypass_active(void){
    if(!s_as_range_bypass){
        s_regs[REG_AS_RANGE_BYPASS] = 0u;
        return 0u;
    }

    if((uint32_t)(HAL_GetTick() - s_as_range_bypass_started_ms) >= BRIDGE_AS_RANGE_BYPASS_MS){
        s_as_range_bypass = 0u;
        s_regs[REG_AS_RANGE_BYPASS] = 0u;
        return 0u;
    }

    s_regs[REG_AS_RANGE_BYPASS] = 1u;
    return 1u;
}

static void bridge_set_as_range_bypass(uint8_t active){
    s_as_range_bypass = active ? 1u : 0u;
    s_as_range_bypass_started_ms = HAL_GetTick();
    s_regs[REG_AS_RANGE_BYPASS] = s_as_range_bypass;
    if(!s_as_range_bypass){
        s_dbg_goal_status &= (uint16_t)~(1u << 12);
    }
}

static uint8_t bridge_as_ready_for_motion(void){
    float host_pos;

    if(!s_zero_valid || !as_is_valid()){
        s_as_range_ok = 0u;
        return 0u;
    }

    host_pos = host_pos_from_as();
    s_as_range_ok = bridge_as_host_in_range(host_pos);
    if(s_as_range_ok){
        return 1u;
    }

    return bridge_as_range_bypass_active();
}

static void bridge_set_motor_passive(void){
    motor[Motor1].ctrl.vel_set = 0.0f;
    motor[Motor1].ctrl.kp_set  = 0.0f;
    motor[Motor1].ctrl.kd_set  = 0.0f;
    motor[Motor1].ctrl.tor_set = 0.0f;
    motor[Motor1].ctrl.cur_set = 0.0f;
}

static void bridge_ota_clear_authorization(void){
    s_ota_key_stage = 0u;
    s_ota_key_started_ms = 0u;
    s_regs[REG_OTA_KEY1] = 0u;
    s_regs[REG_OTA_KEY2] = 0u;
    s_regs[REG_OTA_CTRL] = 0u;
}

static uint8_t bridge_ota_handle_write(uint16_t reg, uint16_t value){
    uint32_t now = HAL_GetTick();

    if(reg == REG_OTA_KEY1){
        bridge_ota_clear_authorization();
        if(value != BRIDGE_OTA_KEY1_VALUE || s_ota_reset_requested){
            return 0u;
        }
        s_ota_key_stage = 1u;
        s_ota_key_started_ms = now;
        return 1u;
    }

    if(reg == REG_OTA_KEY2){
        if(value != BRIDGE_OTA_KEY2_VALUE ||
           s_ota_key_stage != 1u ||
           (now - s_ota_key_started_ms) > BRIDGE_OTA_KEY_WINDOW_MS){
            bridge_ota_clear_authorization();
            return 0u;
        }
        s_ota_key_stage = 2u;
        return 1u;
    }

    if(reg == REG_OTA_CTRL){
        if(value != 1u ||
           s_ota_key_stage != 2u ||
           (now - s_ota_key_started_ms) > BRIDGE_OTA_KEY_WINDOW_MS ||
           s_regs[REG_ENABLE] != 0u ||
           s_huart == NULL || s_hcan == NULL ||
           s_ota_reset_requested){
            bridge_ota_clear_authorization();
            return 0u;
        }

        bridge_ota_clear_authorization();
        s_goal.valid = 0u;
        s_goal_cmd_seen = 0u;
        s_contact_hold_active = 0u;
        s_enable_retry_until = 0u;
        bridge_grip_hold_reset();
        bridge_set_motor_passive();
        dm_motor_disable(s_hcan, &motor[Motor1]);
        s_regs[REG_ENABLE] = 0u;

        s_ota_session_id = GripperOtaApp_CreateSessionId(now);
        s_ota_request_started_ms = now;
        s_ota_reply_sent_ms = 0u;
        s_ota_reply_sent = 0u;
        s_ota_reset_requested = 1u;
        return 1u;
    }

    return 0u;
}

static void bridge_service_uart_tx(uint32_t now_ms){
    uint32_t baud = s_huart->Init.BaudRate ? s_huart->Init.BaudRate : 115200u;
    uint32_t tx_timeout_ms = ((uint32_t)s_txlen * 10000u + baud - 1u) / baud + 10u;
    if(s_tx_busy && ((now_ms - s_tx_started_ms) > tx_timeout_ms)){
        (void)HAL_UART_AbortTransmit_IT(s_huart);
        s_tx_busy = 0u;
        s_tx_pending = 0u;
        s_tx_started_ms = 0u;
        RS485_EN_R;
        if(!s_ota_reset_requested){
            HAL_UARTEx_ReceiveToIdle_IT(s_huart, s_rxbuf, RX_BUF_SZ);
        }
    }

    if(s_tx_pending && !s_tx_busy){
        RS485_EN_W;
        if(HAL_OK == HAL_UART_Transmit_IT(s_huart, s_txbuf, s_txlen)){
            s_tx_busy = 1u;
            s_tx_started_ms = now_ms;
        }else{
            s_tx_pending = 0u;
            RS485_EN_R;
            if(!s_ota_reset_requested){
                HAL_UARTEx_ReceiveToIdle_IT(s_huart, s_rxbuf, RX_BUF_SZ);
            }
        }
    }
}

static void bridge_ota_poll(uint32_t now_ms){
    if(s_ota_key_stage != 0u &&
       (now_ms - s_ota_key_started_ms) > BRIDGE_OTA_KEY_WINDOW_MS){
        bridge_ota_clear_authorization();
    }

    if(!s_ota_reset_requested){
        return;
    }

    s_regs[REG_ENABLE] = 0u;
    s_enable_retry_until = 0u;
    bridge_set_motor_passive();

    if(s_ota_reply_sent &&
       (__HAL_UART_GET_FLAG(s_huart, UART_FLAG_TC) != RESET) &&
       ((now_ms - s_ota_reply_sent_ms) >= BRIDGE_OTA_RESET_DELAY_MS)){
        (void)HAL_UART_AbortReceive_IT(s_huart);
        RS485_EN_R;
        GripperOtaApp_ResetToBootloader(s_ota_session_id,
                                        GRIPPER_BOOT_REASON_MODBUS_OTA);
    }

    if((now_ms - s_ota_request_started_ms) > BRIDGE_OTA_ACK_TIMEOUT_MS){
        /* Never reset if the Modbus acceptance ACK was not completed. */
        s_ota_reset_requested = 0u;
        s_ota_reply_sent = 0u;
        s_ota_session_id = 0u;
        s_tx_busy = 0u;
        s_tx_pending = 0u;
        s_tx_started_ms = 0u;
        RS485_EN_R;
        (void)HAL_UART_AbortReceive_IT(s_huart);
        HAL_UARTEx_ReceiveToIdle_IT(s_huart, s_rxbuf, RX_BUF_SZ);
    }
}

/* ------------------------ CRC16 Modbus ------------------------ */
static uint16_t crc16_modbus(const uint8_t *buf, uint16_t len){
    uint16_t crc = 0xFFFF;
    uint16_t i;
    for(i=0;i<len;i++){
        uint8_t b = buf[i];
        uint8_t k;
        crc ^= b;
        for(k=0;k<8;k++){
            if(crc & 0x0001) crc = (crc >> 1) ^ 0xA001;
            else             crc >>= 1;
        }
    }
    return crc;
}

static inline int32_t get_s32(uint16_t reg_h){
    return ((int32_t)((uint32_t)s_regs[reg_h] << 16) | (uint32_t)s_regs[reg_h+1]);
}

static inline uint32_t get_u32(uint16_t reg_h){
    return ((uint32_t)s_regs[reg_h] << 16) | (uint32_t)s_regs[reg_h+1];
}

static inline void put_u32(uint16_t reg_h, uint32_t v){
    s_regs[reg_h]     = (uint16_t)((v >> 16) & 0xFFFF);
    s_regs[reg_h + 1] = (uint16_t)(v & 0xFFFF);
}

static inline void put_i32(uint16_t reg_h, int32_t v){
    put_u32(reg_h, (uint32_t)v);
}


/* ------------------------ 只读寄存器保护 ------------------------ */
static uint8_t is_readonly_reg(uint16_t reg){
    /* All history writes are handled by the dedicated atomic FC10 path. */
    if(reg >= MAT_REG_FIRST && reg < MAT_REG_END) return 1;
    /* 固定设备信息只读 */
    if(reg == REG_DEV_ID || reg == REG_FW_VER) return 1;

    /* OTA capability/version information is read-only. */
    if(reg >= REG_BOOT_VER && reg <= REG_APP_ROLE) return 1;

    /* 反馈寄存器只读 */
    if(reg >= REG_FB_POS_H && reg <= REG_FB_ROTOR_TEMP) return 1;
    if(reg >= REG_FB_TOR_RAW_H && reg <= REG_FB_POS_EST_L) return 1;
    if(reg >= REG_FB_PRESS_BASE && reg <= REG_FB_PRESS_LAST) return 1;
    /* 压力安全反馈只读 */
    if(reg >= REG_SAFE_FLAGS && reg <= REG_SAFE_MAX) return 1;

    /* 持久化状态只读 */
    if(reg >= REG_PERSIST_VALID && reg <= REG_PERSIST_BIAS_MRAD_L) return 1;

    return 0;
}

/* ------------------------ Pressure safety helpers ------------------------ */
static void safety_sync_threshold_from_regs(void){
    uint32_t sum_th = get_u32(REG_SAFE_SUM_TH_H);
    uint16_t max_th = s_regs[REG_SAFE_MAX_TH];
    PressureSafety_SetThreshold(sum_th, max_th);
}

static void safety_update_feedback_regs(void){
    uint32_t sum = 0;
    uint16_t max = 0;
    uint16_t flags = 0;
    PressureSafety_GetLatest(&sum, &max);
    flags = PressureSafety_GetFlags();

    s_regs[REG_SAFE_FLAGS] = flags;
    put_u32(REG_SAFE_SUM_H, sum);
    s_regs[REG_SAFE_MAX] = max;
}

static void safety_enforce_latch_stop(void){
    /*
     * 压力硬锁存时必须强制失能，防止继续夹持。
     * 写入 UNLOCK 后进入卸压等待，仅允许 OPEN 方向释放压力。
     */
    static uint8_t s_prev_latched = 0;
    uint8_t latched = PressureSafety_IsLatched();

    if(latched){
        s_regs[REG_ENABLE] = 0;
        s_contact_hold_active = 0u;
        bridge_grip_hold_reset();
        if(!s_prev_latched){
            if(s_hcan){ dm_motor_disable(s_hcan, &motor[Motor1]); }
        }
    }
    s_prev_latched = latched;
}



/* ------------------------ 持久化状态寄存器同步 ------------------------ */
static void persist_regs_on_load(void){
    int32_t pos_mrad = (int32_t)(s_zero_abs_rad * 1000.0f); /* LOADED_POS: 保存的零点绝对角 */
    uint16_t scale_milli = (uint16_t)(s_loaded_scale * 1000.0f + 0.5f);

    s_regs[REG_PERSIST_VALID] = (uint16_t)(s_zero_valid ? 1u : 0u);
    put_i32(REG_PERSIST_LOADED_POS_H, pos_mrad);
    s_regs[REG_PERSIST_LOADED_SCALE] = scale_milli;

    /* 初始状态：尚未保存 */
    s_regs[REG_PERSIST_LAST_SAVE_ST] = 0xFFFF;
    s_save_ok_cnt = 0;
    s_regs[REG_PERSIST_SAVE_OK_CNT] = s_save_ok_cnt;

    if(s_bias_ready){
        persist_regs_on_bias_ready(s_host_bias);
    }else{
        put_i32(REG_PERSIST_BIAS_MRAD_H, 0);
    }
}

static void persist_regs_on_save_result(HAL_StatusTypeDef st, float zero_abs_rad){
    int32_t pos_mrad = (int32_t)(zero_abs_rad * 1000.0f);
    uint16_t scale_milli = (uint16_t)(safe_scale() * 1000.0f + 0.5f);

    s_regs[REG_PERSIST_LAST_SAVE_ST] = (uint16_t)st;
    /* 同步当前 scale，便于确认保存时的比例参数。 */
    s_regs[REG_PERSIST_LOADED_SCALE] = scale_milli;
    /* 将最近一次保存的 zero_abs 写回 loaded_pos，便于上位机观测。 */
    put_i32(REG_PERSIST_LOADED_POS_H, pos_mrad);

    if(st == HAL_OK){
        s_save_ok_cnt++;
        s_regs[REG_PERSIST_SAVE_OK_CNT] = s_save_ok_cnt;
        s_regs[REG_PERSIST_VALID] = 1; /* 一旦成功写入，持久化参数即视为有效。 */
    }
}

static void persist_regs_on_bias_ready(float bias_rad){
    int32_t bias_mrad = (int32_t)(bias_rad * 1000.0f);
    put_i32(REG_PERSIST_BIAS_MRAD_H, bias_mrad);
}

/* ------------------------ 持久化保存接口 ------------------------ */
static void bridge_force_save(uint8_t allow_erase){
    if(!as_is_valid()) return;

    s_zero_abs_rad = as_abs_rad_now();
    s_zero_valid = 1;

    if(s_bias_ready){
        float scale_now = safe_scale();
        float motor_pos = motor[Motor1].para.pos;
        float host_now  = host_pos_from_as();
        s_host_bias = host_now - (motor_pos / scale_now);
        s_bias_from_as = 1;
        s_bias_from_persist = 1;
    }

    {
        HAL_StatusTypeDef st = PosPersist_SaveEx(
            s_zero_abs_rad,
            safe_scale(),
            s_bias_ready ? s_host_bias : 0.0f,
            allow_erase
        );
        persist_regs_on_save_result(st, s_zero_abs_rad);
        if(st == HAL_OK){
            s_regs[REG_PERSIST_VALID] = 1;
        }
    }

    if(s_bias_ready){
        persist_regs_on_bias_ready(s_host_bias);
    }

    bridge_set_as_range_bypass(0u);
}

static void bridge_save_motor_zero(void){
    HAL_StatusTypeDef st = HAL_ERROR;
    float host_now = 0.0f;

    if(s_hcan == NULL) return;

    save_pos_zero(s_hcan, motor[Motor1].id, MIT_MODE);

    if(as_is_valid()){
        host_now = host_pos_from_as();
        s_host_bias = host_now;
        s_bias_from_as = 1;
    }else{
        host_now = 0.0f;
        s_host_bias = 0.0f;
        s_bias_from_as = 0;
    }
    s_bias_ready = 1;
    s_bias_from_persist = 1;

    st = PosPersist_SaveEx(s_zero_abs_rad, safe_scale(), s_host_bias, 1u);
    persist_regs_on_save_result(st, s_zero_abs_rad);
    persist_regs_on_bias_ready(s_host_bias);

    if(s_regs[REG_ENABLE]){
        s_goal.valid = 1u;
        s_goal.p = host_now;
        s_goal.v = 0.0f;
        s_goal.kp = ((float)s_regs[REG_KP_1_100]) / 100.0f;
        s_goal.kd = ((float)s_regs[REG_KD_1_100]) / 100.0f;
        s_goal.tff = 0.0f;
        bridge_apply_goal_dispatch(&s_goal);
    }
}
/* 不再周期性保存当前位置，避免频繁写 Flash。 */

        

/* ------------------------ 目标解析与应用 ------------------------ */
#if !BRIDGE_USE_AS_VELOCITY_LOOP
static void bridge_apply_goal(const goal_cache_t *g){
    float raw_target;
    goal_cache_t applied = *g;
    float host_now = 0.0f;
    uint8_t host_ok = 0u;

    if(as_is_valid()){
        host_now = host_pos_from_as();
        host_ok = 1u;
    }else if(s_bias_ready){
        host_now = motor_to_host_pos(motor[Motor1].para.pos);
        host_ok = 1u;
    }

    if(!s_contact_hold_active && host_ok && s_regs[REG_ENABLE]){
        if(bridge_grip_hold_confirm(host_now, applied.p)){
            s_contact_hold_active = 1u;
            s_contact_hold_pos = host_now;
            s_dbg_goal_status |= (1u << 9);
        }
    }

    if(s_contact_hold_active){
        if(bridge_target_is_opening_from_hold(applied.p)){
            s_contact_hold_active = 0u;
            bridge_grip_hold_reset();
        }else{
            applied.p = s_contact_hold_pos;
            applied.v = 0.0f;
            applied.tff = 0.0f;
            s_dbg_goal_status |= (1u << 8);
        }
    }

    if(as_is_valid()){
        float motor_now = motor[Motor1].para.pos;
        float motor_delta;
        if(!host_ok){
            host_now = host_pos_from_as();
        }
        motor_delta = safe_scale() * (applied.p - host_now);
        motor_delta = clampf_bridge(
            motor_delta,
            -BRIDGE_AS_MOTOR_STEP_MAX_RAD,
            BRIDGE_AS_MOTOR_STEP_MAX_RAD
        );
        raw_target = motor_now + motor_delta;
    }else{
        raw_target = host_to_motor_pos(applied.p);
    }
    if(s_first_motion_soft_limit){
        const float soft_v = DEG2RADf(BRIDGE_FIRST_MOVE_VEL_DEG);
        if(applied.v > soft_v){
            applied.v = soft_v;
        }
        if(applied.kp > BRIDGE_FIRST_MOVE_KP_MAX){
            applied.kp = BRIDGE_FIRST_MOVE_KP_MAX;
        }
        applied.tff = 0.0f;
        s_dbg_goal_status |= (1u << 11);
        s_first_motion_soft_limit = 0u;
    }
    float cmd_target = clamp_motor_pos_cmd(raw_target);
    s_dbg_motor_raw_target = raw_target;
    s_dbg_motor_cmd_target = cmd_target;
    s_dbg_motor_clamped = (f_abs(raw_target - cmd_target) > 0.001f) ? 1u : 0u;
    s_dbg_goal_status |= (1u << 5);
    motor[Motor1].ctrl.pos_set = cmd_target;
    motor[Motor1].ctrl.vel_set = applied.v;
    motor[Motor1].ctrl.kp_set  = applied.kp;
    motor[Motor1].ctrl.kd_set  = applied.kd;
    motor[Motor1].ctrl.tor_set = applied.tff;
}
#endif

static void bridge_apply_goal_as_velocity(const goal_cache_t *g)
{
    goal_cache_t applied = *g;
    float host_now;
    float err;
    float vel_host_limit;
    float vel_motor_limit;
    float vel_motor_cmd = 0.0f;
    float pos_hold;

    if(!bridge_as_ready_for_motion()){
        bridge_set_motor_passive();
        return;
    }

    host_now = host_pos_from_as();

    if(!s_contact_hold_active && s_regs[REG_ENABLE]){
        if(bridge_grip_hold_confirm(host_now, applied.p)){
            s_contact_hold_active = 1u;
            s_contact_hold_pos = host_now;
            s_dbg_goal_status |= (1u << 9);
        }
    }

    if(s_contact_hold_active){
        if(bridge_target_is_opening_from_hold(applied.p)){
            s_contact_hold_active = 0u;
            bridge_grip_hold_reset();
        }else{
            applied.p = s_contact_hold_pos;
            applied.v = 0.0f;
            applied.tff = 0.0f;
            s_dbg_goal_status |= (1u << 8);
        }
    }

    err = applied.p - host_now;

    if(f_abs(err) <= DEG2RADf(BRIDGE_AS_POS_DEADBAND_DEG)){
        vel_motor_cmd = 0.0f;
    }else{
        vel_host_limit = DEG2RADf(BRIDGE_AS_VEL_MAX_DEG);
        if(applied.v > 0.0f && applied.v < vel_host_limit){
            vel_host_limit = applied.v;
        }
        if(s_first_motion_soft_limit){
            const float soft_v = DEG2RADf(BRIDGE_FIRST_MOVE_VEL_DEG);
            if(soft_v < vel_host_limit){
                vel_host_limit = soft_v;
            }
            applied.tff = 0.0f;
            s_dbg_goal_status |= (1u << 11);
            s_first_motion_soft_limit = 0u;
        }

        vel_motor_limit = safe_scale() * vel_host_limit;
        vel_motor_cmd = BRIDGE_AS_VEL_KP * safe_scale() * err;
        vel_motor_cmd = clampf_bridge(vel_motor_cmd, -vel_motor_limit, vel_motor_limit);
    }

    pos_hold = motor[Motor1].para.pos;

    s_dbg_motor_raw_target = pos_hold;
    s_dbg_motor_cmd_target = pos_hold;
    s_dbg_motor_clamped = 0u;
    s_dbg_goal_status |= (1u << 5);

    motor[Motor1].ctrl.pos_set = pos_hold;
    motor[Motor1].ctrl.vel_set = vel_motor_cmd;
    motor[Motor1].ctrl.kp_set  = 0.0f;
    motor[Motor1].ctrl.kd_set  = BRIDGE_AS_HOLD_KD;
    motor[Motor1].ctrl.tor_set = applied.tff;
}

static void bridge_apply_goal_dispatch(const goal_cache_t *g)
{
#if BRIDGE_USE_AS_VELOCITY_LOOP
    bridge_apply_goal_as_velocity(g);
#else
    bridge_apply_goal(g);
#endif
}

static void bridge_parse_goal_from_regs_and_apply(void){
    int32_t v_raw = get_s32(REG_V_DES_H);
    int32_t p_raw = get_s32(REG_P_DES_H);
    float   kp_in = (float)s_regs[REG_KP_1_100] / 100.0f;
    float   kd_in = (float)s_regs[REG_KD_1_100] / 100.0f;
    float   tf_in = (float)get_s32(REG_TFF_H) / 1000.0f;

    uint16_t unit = s_regs[REG_UNIT_CFG];

    float p_in = (unit & SCALE_POS_X1) ? (float)p_raw : ((float)p_raw / 1000.0f);
    float v_in = (unit & SCALE_VEL_X1) ? (float)v_raw : ((float)v_raw / 1000.0f);

    if (unit & UNIT_POS_IS_DEG) p_in = DEG2RADf(p_in);
    if (unit & UNIT_VEL_IS_DEG) v_in = DEG2RADf(v_in);
    s_dbg_goal_status = 0u;
    if(s_goal_cmd_seen || s_goal.valid){
        s_goal.valid = 1u;
        s_dbg_goal_status |= (1u << 0);
    }else{
        s_goal.valid = 0u;
    }
    if (s_bias_ready) s_dbg_goal_status |= (1u << 1);
    if (s_bias_from_as) s_dbg_goal_status |= (1u << 6);
    if (s_bias_from_persist) s_dbg_goal_status |= (1u << 7);
    if (s_regs[REG_ENABLE]) s_dbg_goal_status |= (1u << 2);
    if (PressureSafety_IsLatched()) s_dbg_goal_status |= (1u << 3);
    if (PressureSafety_IsBlocked()) s_dbg_goal_status |= (1u << 4);
    if (!bridge_as_ready_for_motion()) s_dbg_goal_status |= (1u << 10);
    if (s_first_motion_soft_limit) s_dbg_goal_status |= (1u << 11);
    if (bridge_as_range_bypass_active()) s_dbg_goal_status |= (1u << 12);
    s_goal.v = v_in;
    s_goal.p = p_in;
    s_goal.kp = kp_in;
    s_goal.kd = kd_in;
    s_goal.tff = tf_in;

    if(!s_goal.valid){
        bridge_set_motor_passive();
        return;
    }

    if (!s_regs[REG_ENABLE]) {
        return;
    }

#if BRIDGE_USE_AS_VELOCITY_LOOP
    if (!bridge_as_ready_for_motion()) {
        if (s_regs[REG_ENABLE]) {
            bridge_set_motor_passive();
        }
        return;
    }
#else
    if (!bridge_as_ready_for_motion() && !s_bias_ready) {
        bridge_set_motor_passive();
        return;
    }
#endif

    if (!s_regs[REG_ENABLE]) {
        return;
    }

    // 硬锁存状态下不应用目标。
    if (PressureSafety_IsLatched()) {
        return;
    }

    // 正常状态：应用完整目标。
    if (!PressureSafety_IsBlocked()) {
        bridge_apply_goal_dispatch(&s_goal);
        return;
    }

    /*
     * unlock_pending 表示已解锁但仍在释放确认中；此时只允许 OPEN 方向卸压，忽略闭合/保持指令。
     */
    {
        float host_now;
        if (as_is_valid()) {
            host_now = host_pos_from_as();
        } else {
            host_now = motor_to_host_pos(motor[Motor1].para.pos);
        }

        // OPEN means the target host position is smaller than the current host position.
        if (s_goal.p < host_now - DEG2RADf(0.5f)) {
            goal_cache_t g = s_goal;
            const float V_REL = DEG2RADf(30.0f);
            if (g.v > V_REL) g.v = V_REL;
            g.tff = 0.0f;
            bridge_apply_goal_dispatch(&g);
        } else {
            // 卸压等待期间忽略闭合/保持指令。
        }
    }

}

/* 模式映射 */
static void sync_mode_to_motor(void){
    uint16_t m = (uint16_t)(s_regs[REG_MODE] & 0x03);
    uint8_t  official = mit_mode;
    if(m==1) official = pos_mode;
    else if(m==2) official = spd_mode;
    motor[Motor1].ctrl.mode = official;
}

/* 写 CMode 并在已使能时重新使能 */
static void bridge_apply_mode_and_reenable(void){
    uint8_t official = motor[Motor1].ctrl.mode;
    write_motor_data(motor[Motor1].id, RID_CMODE, official, 0, 0, 0);
    if (s_regs[REG_ENABLE]) {
        if(PressureSafety_IsLatched()){
            /* 不允许模式切换绕过压力硬锁存。 */
            s_regs[REG_ENABLE] = 0;
            dm_motor_disable(s_hcan, &motor[Motor1]);
        }else{
            if(!bridge_as_ready_for_motion()){
                bridge_set_motor_passive();
                s_dbg_goal_status |= (1u << 10);
            }
            dm_motor_disable(s_hcan, &motor[Motor1]);
            dm_motor_enable (s_hcan, &motor[Motor1]);
            bridge_schedule_enable_retry(HAL_GetTick());
        }
    }
}

/* ------------------------ 异常响应 ------------------------ */
static void respond_exception(uint8_t func, uint8_t code){
    s_txbuf[0] = BRIDGE_SLAVE_ADDR;
    s_txbuf[1] = (uint8_t)(func | 0x80);
    s_txbuf[2] = code;
    {
        uint16_t tl = 3;
        uint16_t c = crc16_modbus(s_txbuf, tl);
        s_txbuf[tl++] = (uint8_t)(c & 0xFF);
        s_txbuf[tl++] = (uint8_t)(c >> 8);
        s_txlen = tl;
        s_tx_pending = 1;
    }
}

// ------------------------ 对外接口 ------------------------
/**
 * @brief  初始化 Modbus 桥接
 * @param  huartx: 用于 Modbus 的 UART 句柄
 * @param  hfdcan1: 用于电机的 FDCAN 句柄
 * @param  motor_id: 电机 ID，用于寄存器初值
 * @retval None
 */
void DM_Bridge_Init(UART_HandleTypeDef *huartx, FDCAN_HandleTypeDef *hfdcan1, uint16_t motor_id){
    (void)motor_id;
    s_huart = huartx;
    s_hcan  = hfdcan1;

    memset(s_regs, 0, sizeof(s_regs));
    s_regs[REG_DEV_ID]   = 0xD431;
    s_regs[REG_FW_VER]   = GRIPPER_APP_VERSION_U16;
    s_regs[REG_MOTOR_ID] = (uint16_t)(motor[Motor1].id & 0x7FF);
    s_regs[REG_MODE]     = 0;
    s_regs[REG_ENABLE]   = 0;
    s_regs[REG_DM_PMAX_MRAD] = (uint16_t)(motor[Motor1].tmp.PMAX * 1000.0f + 0.5f);
    s_regs[REG_DM_PMAX_APPLY] = 0u;
    bridge_set_as_range_bypass(0u);
    s_regs[REG_BOOT_VER] = GRIPPER_BOOT_VERSION_U16;
    s_regs[REG_OTA_CAPS] = GRIPPER_APP_OTA_CAPS;
    s_regs[REG_APP_BUILD] = GRIPPER_APP_VERSION_U16;
    s_regs[REG_APP_PATCH] = GRIPPER_APP_VERSION_PATCH;
    s_regs[REG_APP_READY] = 1u;
    s_regs[REG_LAYOUT_VERSION] = (uint16_t)GRIPPER_LAYOUT_VERSION;
    s_regs[REG_APP_ROLE] = GRIPPER_APP_ROLE;
    bridge_ota_clear_authorization();
    s_ota_reset_requested = 0u;
    s_ota_reply_sent = 0u;
    s_ota_session_id = 0u;


    s_pos_scale = BRIDGE_POS_SCALE_DEFAULT;
	

    /* 默认：位置=度，速度=度/s，并使用 x1 缩放。 */
    s_regs[REG_UNIT_CFG] = (UNIT_POS_IS_DEG | UNIT_VEL_IS_DEG | SCALE_POS_X1 | SCALE_VEL_X1);

    /* 压力保护默认阈值，可由上位机写 0x0030~0x0032 修改。 */
    put_u32(REG_SAFE_SUM_TH_H, 4500u);
    s_regs[REG_SAFE_MAX_TH] = 1000u;
    s_regs[REG_SAFE_UNLOCK] = 0;
    safety_sync_threshold_from_regs();
    s_regs[REG_EST_TOR_TH]      = 80u;
    s_regs[REG_EST_VEL_IDLE_TH] = 50u;
    s_regs[REG_EST_ALPHA]       = 100u;
    s_regs[REG_EST_BIAS_LEARN]  = 10u;
    s_regs[REG_EST_FRIC_B]      = 47u;
    s_regs[REG_EST_COULOMB_POS] = 250u;
    s_regs[REG_EST_COULOMB_NEG] = 120u;
    s_regs[REG_EST_STATIC_POS]  = 120u;
    s_regs[REG_EST_STATIC_NEG]  = 80u;
    s_regs[REG_EST_VEL_SIGN_TH] = 40u;
    s_regs[REG_EST_CONTACT_DIR] = 1u;
    put_i32(REG_EST_BASE_OPEN_H,  (int32_t)(-29.74f * 17.45329252f));
    put_i32(REG_EST_BASE_CLOSE_H, 0);
    s_regs[REG_EST_GRIP_TH] = 220u;
    estimator_sync_regs_to_param();
    estimator_sync_param_to_regs();
    estimator_update_feedback_regs();

    /* Load AS zero, scale and optional host/motor bias from Flash. */
    {
        float loaded_bias = 0.0f;
        uint8_t loaded_bias_valid = 0u;

        PosPersist_Init();
        s_zero_valid = PosPersist_LoadEx(
            &s_zero_abs_rad,
            &s_loaded_scale,
            &loaded_bias,
            &loaded_bias_valid
        );

        if(s_zero_valid && f_abs(s_loaded_scale) > 1e-6f){
            s_pos_scale = s_loaded_scale;
        }

        if(!s_zero_valid){
            if(as_is_valid()){
                s_zero_abs_rad = as_abs_rad_now();
            }else{
                s_zero_abs_rad = 0.0f;
            }
        }

        /*
         * 达妙电机只记住单圈零点。断电后多圈计数可能变化，所以启动时
         * 不再直接信任 Flash 中的 host/motor bias；等待第一帧电机反馈后，
         * 用当前 AS5048A 绝对角度重新建立 bias。
         */
        s_bias_ready = 0u;
        s_host_bias = loaded_bias_valid ? loaded_bias : 0.0f;
        s_bias_from_as = 0u;
        s_bias_from_persist = 0u;
        s_boot_as_rebind_done = 0u;
        s_first_motion_soft_limit = 0u;
        s_as_range_ok = 0u;
    }

    s_goal.valid = 0;
    s_goal_cmd_seen = 0u;

    persist_regs_on_load();
    sync_mode_to_motor();
}

/**
 * @brief  鍚姩 UART Idle 鎺ユ敹
 * @param  None
 * @retval None
 */
void DM_Bridge_Start(void){
    RS485_EN_R;
    s_tx_busy = 0u;
    s_tx_pending = 0u;
    s_tx_started_ms = 0u;
    s_ota_reset_requested = 0u;
    s_ota_reply_sent = 0u;
    s_ota_session_id = 0u;
    bridge_ota_clear_authorization();
    __HAL_UART_CLEAR_OREFLAG(s_huart);
    HAL_UARTEx_ReceiveToIdle_IT(s_huart, s_rxbuf, RX_BUF_SZ);
}

/**
 * @brief  主循环轮询处理，包含安全状态、估算器、反馈寄存器和发送队列。
 * @param  None
 * @retval None
 */
void DM_Bridge_Poll(void){
    uint32_t now = HAL_GetTick();
    estimator_sample_t estimator_sample;

    PressureHistory_Service();
    bridge_service_uart_tx(now);
    bridge_ota_poll(now);
    if(s_ota_reset_requested){
        return;
    }

    safety_update_feedback_regs();
    safety_enforce_latch_stop();
    if(s_regs[REG_ENABLE] && !bridge_as_ready_for_motion()){
        bridge_set_motor_passive();
        s_dbg_goal_status |= (1u << 10);
    }
    if(((s_estimator_last_update == 0u) ||
        ((now - s_estimator_last_update) >= BRIDGE_EST_UPDATE_PERIOD_MS)) &&
       bridge_take_estimator_sample(&estimator_sample)){
        ForceEstimator_Update(
            estimator_sample.host_pos,
            estimator_sample.vel,
            estimator_sample.tor
        );
        bridge_update_grip_hold(estimator_sample.host_pos);
        s_estimator_last_update = now;
    }

    if((s_feedback_last_update == 0u) ||
       ((now - s_feedback_last_update) >= BRIDGE_FB_UPDATE_PERIOD_MS)){
        bridge_update_extended_feedback_regs();
        s_feedback_last_update = now;
    }

    bridge_track_goal_from_as(now);

    if(s_regs[REG_ENABLE] && s_hcan && (now < s_enable_retry_until)){
        if((s_enable_retry_last == 0u) || ((now - s_enable_retry_last) >= 50u)){
            dm_motor_enable(s_hcan, &motor[Motor1]);
            s_enable_retry_last = now;
        }
    }

}

/**
 * @brief  UART 发送完成回调
 * @param  huart: UART 句柄
 */
void DM_Bridge_OnUartTxDone(UART_HandleTypeDef *huart){
    if(huart != s_huart) return;
    s_tx_busy = 0;
    s_tx_pending = 0;
    s_tx_started_ms = 0u;
    while(__HAL_UART_GET_FLAG(s_huart, UART_FLAG_TC) == RESET) {;}
    RS485_EN_R;
    if(s_ota_reset_requested){
        s_ota_reply_sent = 1u;
        s_ota_reply_sent_ms = HAL_GetTick();
    }else{
        HAL_UARTEx_ReceiveToIdle_IT(s_huart, s_rxbuf, RX_BUF_SZ);
    }
}

// ------------------------ Modbus 解析 ------------------------
/**
 * @brief  UART Idle 回调，解析 Modbus 帧
 * @param  huart: UART 句柄
 * @retval None
 */
void DM_Bridge_OnUartIdle(UART_HandleTypeDef *huart, uint16_t size){
    if(huart != s_huart) return;
    if(s_tx_busy || s_tx_pending) return;
    s_rxlen = size;
    (void)s_rxlen;

    if(size < 4){
        HAL_UARTEx_ReceiveToIdle_IT(s_huart, s_rxbuf, RX_BUF_SZ);
        return;
    }

    {
        const uint8_t *f = s_rxbuf;
        const uint16_t framelen = size;

        if(f[0] != BRIDGE_SLAVE_ADDR){
            HAL_UARTEx_ReceiveToIdle_IT(s_huart, s_rxbuf, RX_BUF_SZ);
            return;
        }

        /* CRC 校验 */
        {
            uint16_t crc = crc16_modbus(f, framelen - 2);
            uint16_t crc_rx = (uint16_t)(f[framelen-2] | ((uint16_t)f[framelen-1] << 8));
            if(crc != crc_rx){
                respond_exception(f[1], 0x03);
                return;
            }
        }

        {
            uint8_t func = f[1];
            uint16_t txlen = 0;

            if(func == 0x03u){ /* Read Holding */
                if(framelen < 8){ goto done; }
                {
                    uint16_t start = (uint16_t)((f[2] << 8) | f[3]);
                    uint16_t count = (uint16_t)((f[4] << 8) | f[5]);
                    if(count == 0u || count > 125u || (5u + 2u * count) > TX_BUF_SZ){
                        respond_exception(func, 0x03); return;
                    }
                    if(start >= 0x200u || count > (0x200u - start)){
                        respond_exception(func, 0x02); return;
                    }

                    s_txbuf[0] = BRIDGE_SLAVE_ADDR;
                    s_txbuf[1] = 0x03;
                    s_txbuf[2] = (uint8_t)(count * 2);
                    {
                        uint16_t i;
                        for(i=0;i<count;i++){
                            uint16_t v = s_regs[start + i];
                            s_txbuf[3 + 2*i]     = (uint8_t)(v >> 8);
                            s_txbuf[3 + 2*i + 1] = (uint8_t)(v & 0xFF);
                        }
                    }
                    /* One pinned history snapshot for the whole response, including
                     * requests crossing from legacy registers into the new range. */
                    if(start + count > MAT_REG_FIRST){
                        uint16_t first = start < MAT_REG_FIRST ? MAT_REG_FIRST : start;
                        PressureHistory_ReadRegs(first, (uint16_t)(start + count - first),
                                                 s_txbuf + 3u + 2u * (first - start));
                    }
                    txlen = (uint16_t)(3 + count*2);
                }

            }else if(func == 0x06u){ /* Write Single */
                if(framelen < 8){ goto done; }
                {
                    uint16_t reg = (uint16_t)((f[2] << 8) | f[3]);
                    uint16_t val = (uint16_t)((f[4] << 8) | f[5]);

                    if(reg >= (sizeof(s_regs)/sizeof(s_regs[0]))){
                        respond_exception(func, 0x02);
                        return;
                    }

                    if(is_readonly_reg(reg)){
                        respond_exception(func, 0x02); /* Illegal Data Address */
                        return;
                    }

                    if(reg >= REG_OTA_KEY1 && reg <= REG_OTA_CTRL){
                        if(!bridge_ota_handle_write(reg, val)){
                            respond_exception(func, 0x03); /* Illegal value/state */
                            return;
                        }
                    }else{
                        s_regs[reg] = val;
                    }

                    if(reg == REG_SAVE_ZERO){
                        if(val == 1u){
                            bridge_force_save(1);
                        }
                        s_regs[reg] = 0;

                    }else if(reg == REG_SAVE_MOTOR_ZERO){
                        if(val == 1u){
                            bridge_save_motor_zero();
                        }
                        s_regs[reg] = 0;

                    }else if(reg == REG_SAFE_UNLOCK){
                        if(val == 0xA55Au){
                            PressureSafety_ClearLatch();
                        }
                        s_regs[REG_SAFE_UNLOCK] = 0;

                    }else if(reg == REG_AS_RANGE_BYPASS){
                        if(val == BRIDGE_AS_RANGE_BYPASS_MAGIC){
                            bridge_set_as_range_bypass(1u);
                        }else if(val == 0u){
                            bridge_set_as_range_bypass(0u);
                        }else{
                            s_regs[REG_AS_RANGE_BYPASS] = bridge_as_range_bypass_active();
                        }
                        if(s_goal.valid && s_regs[REG_ENABLE]){
                            bridge_parse_goal_from_regs_and_apply();
                        }

                    }else if(reg == REG_SAFE_SUM_TH_H || reg == REG_SAFE_SUM_TH_L || reg == REG_SAFE_MAX_TH){
                        safety_sync_threshold_from_regs();
                    }else if((reg >= REG_EST_TOR_TH) && (reg <= REG_EST_GRIP_TH)){
                        estimator_sync_regs_to_param();
                        estimator_sync_param_to_regs();

                    }else if(reg == REG_DM_PMAX_APPLY){
                        if(val != 0u){
                            bridge_apply_dm_pmax_setting((uint8_t)val);
                        }

                    }else if(reg == REG_MOTOR_ID){
                        motor[Motor1].id = (uint16_t)(val & 0x7FF);

                    }else if(reg == REG_MODE){
                        sync_mode_to_motor();
                        bridge_apply_mode_and_reenable();

                    }else if(reg == REG_ENABLE){
                        sync_mode_to_motor();
                        if(val) {
                                     /* 只有硬锁存会禁止使能；unlock_pending 仍允许使能，以便主机执行 OPEN 卸压。 */
                            if(PressureSafety_IsLatched()){
                                s_regs[REG_ENABLE] = 0;
                                dm_motor_disable(s_hcan, &motor[Motor1]);
                            }else{
                                if(!bridge_as_ready_for_motion()){
                                    bridge_set_motor_passive();
                                    s_dbg_goal_status |= (1u << 10);
                                }
                                dm_motor_enable(s_hcan, &motor[Motor1]);
                                bridge_schedule_enable_retry(HAL_GetTick());
                                if (s_goal.valid) {
                bridge_parse_goal_from_regs_and_apply();
            }
                            }
                        } else {
                            s_contact_hold_active = 0u;
                            bridge_grip_hold_reset();
                            dm_motor_disable(s_hcan, &motor[Motor1]);
                        }

                    }else if(reg == REG_KP_1_100 || reg == REG_KD_1_100 ||
                   reg == REG_V_DES_H || reg == REG_V_DES_L ||
                             reg == REG_P_DES_H || reg == REG_P_DES_L ||
                             reg == REG_TFF_H   || reg == REG_TFF_L ||
                             reg == REG_UNIT_CFG){
                        if(reg == REG_P_DES_H || reg == REG_P_DES_L){
                            s_goal_cmd_seen = 1u;
                        }
                        bridge_parse_goal_from_regs_and_apply();

                    }else if(reg == REG_CLEAR_FAULT && val == 1){
                        /* 如果压力硬锁存，不允许清错后自动重新使能。 */
                        dm_motor_clear_err(s_hcan, &motor[Motor1]);
                        if(PressureSafety_IsLatched()){
                            dm_motor_disable(s_hcan, &motor[Motor1]);
                            s_regs[REG_ENABLE] = 0;
                        }else{
                            if(!bridge_as_ready_for_motion()){
                                bridge_set_motor_passive();
                                s_dbg_goal_status |= (1u << 10);
                            }
                            dm_motor_enable(s_hcan, &motor[Motor1]);
                            bridge_schedule_enable_retry(HAL_GetTick());
                            s_regs[REG_ENABLE] = 1;
                            if (s_goal.valid) {
                bridge_parse_goal_from_regs_and_apply();
            }
                        }
                    }

                    memcpy(s_txbuf, f, 6);
                    txlen = 6;
                }

            }else if(func == 0x10u){ /* Write Multiple */
                if(framelen < 9){ goto done; }
                {
                    uint16_t start = (uint16_t)((f[2] << 8) | f[3]);
                    uint16_t count = (uint16_t)((f[4] << 8) | f[5]);
                    uint8_t bytecnt = f[6];
                    uint16_t i;

                    if(count == 0u ||
                       start >= (sizeof(s_regs)/sizeof(s_regs[0])) ||
                       count > ((sizeof(s_regs)/sizeof(s_regs[0])) - start) ||
                       count > 123u ||
                       bytecnt != (uint8_t)(count*2u) ||
                       framelen < (uint16_t)(9u + bytecnt)) {
                        respond_exception(func, 0x03);
                        return;
                    }

                    if(start + count > MAT_REG_FIRST){
                        uint16_t words[MAT_COMMAND_WORDS];
                        uint8_t error;
                        if(start != MAT_REG_COMMAND || count != MAT_COMMAND_WORDS ||
                           bytecnt != 16u || framelen != 25u){
                            respond_exception(func, 0x02); return;
                        }
                        for(i = 0; i < MAT_COMMAND_WORDS; ++i)
                            words[i] = (uint16_t)(((uint16_t)f[7 + 2*i] << 8) | f[8 + 2*i]);
                        error = PressureHistory_SubmitCommand(words);
                        if(error){ respond_exception(func, error); return; }
                        memcpy(s_txbuf, f, 6u); txlen = 6u;
                        goto done;
                    }

                    /* OTA authorization must use three ordered 0x06 writes. */
                    if(start <= REG_OTA_CTRL && (start + count) > REG_OTA_KEY1){
                        respond_exception(func, 0x03);
                        return;
                    }

                    /* 只读区保护：只要写入范围包含只读寄存器，就拒绝整包写入。 */
                    for(i=0;i<count;i++){
                        if(is_readonly_reg((uint16_t)(start + i))){
                            respond_exception(func, 0x02);
                            return;
                        }
                    }

                    for(i=0;i<count;i++){
                        uint16_t v = (uint16_t)((f[7 + i*2] << 8) | f[8 + i*2]);
                        s_regs[start + i] = v;
                    }

                    /* 压力保护：阈值、解锁、使能拦截。 */
                    if((start <= REG_SAFE_SUM_TH_L) && ((start + count) > REG_SAFE_SUM_TH_H)){
                        safety_sync_threshold_from_regs();
                    }
                    if(start <= REG_SAFE_MAX_TH && (start + count) > REG_SAFE_MAX_TH){
                        safety_sync_threshold_from_regs();
                    }
                    if((start <= REG_EST_GRIP_TH) && ((start + count) > REG_EST_TOR_TH)){
                        estimator_sync_regs_to_param();
                        estimator_sync_param_to_regs();
                    }
                    if(start <= REG_DM_PMAX_APPLY && (start + count) > REG_DM_PMAX_APPLY){
                        if(s_regs[REG_DM_PMAX_APPLY] != 0u){
                            bridge_apply_dm_pmax_setting((uint8_t)s_regs[REG_DM_PMAX_APPLY]);
                        }
                    }
                    if(start <= REG_SAFE_UNLOCK && (start + count) > REG_SAFE_UNLOCK){
                        if(s_regs[REG_SAFE_UNLOCK] == 0xA55Au){
                            PressureSafety_ClearLatch();
                        }
                        s_regs[REG_SAFE_UNLOCK] = 0;
                    }
                    if(start <= REG_AS_RANGE_BYPASS && (start + count) > REG_AS_RANGE_BYPASS){
                        if(s_regs[REG_AS_RANGE_BYPASS] == BRIDGE_AS_RANGE_BYPASS_MAGIC){
                            bridge_set_as_range_bypass(1u);
                        }else if(s_regs[REG_AS_RANGE_BYPASS] == 0u){
                            bridge_set_as_range_bypass(0u);
                        }else{
                            s_regs[REG_AS_RANGE_BYPASS] = bridge_as_range_bypass_active();
                        }
                        if(s_goal.valid && s_regs[REG_ENABLE]){
                            bridge_parse_goal_from_regs_and_apply();
                        }
                    }
                    if(start <= REG_SAVE_MOTOR_ZERO && (start + count) > REG_SAVE_MOTOR_ZERO){
                        if(s_regs[REG_SAVE_MOTOR_ZERO] == 1u){
                            bridge_save_motor_zero();
                        }
                        s_regs[REG_SAVE_MOTOR_ZERO] = 0;
                    }
                    if(start <= REG_ENABLE && (start + count) > REG_ENABLE){
                        if(s_regs[REG_ENABLE] && PressureSafety_IsLatched()){
                            s_regs[REG_ENABLE] = 0;
                            dm_motor_disable(s_hcan, &motor[Motor1]);
                        }
                    }

                    /* 目标寄存器变化后重新解析。 */
                    if((start <= REG_TFF_L) && ((start + count) > REG_V_DES_H)){
                        if((start <= REG_P_DES_L) && ((start + count) > REG_P_DES_H)){
                            s_goal_cmd_seen = 1u;
                        }
                        bridge_parse_goal_from_regs_and_apply();
                    }
                    if(start <= REG_UNIT_CFG && (start + count) > REG_UNIT_CFG){
                        bridge_parse_goal_from_regs_and_apply();
                    }

                    if(start <= REG_MOTOR_ID && (start + count) > REG_MOTOR_ID){
                        motor[Motor1].id = (uint16_t)(s_regs[REG_MOTOR_ID] & 0x7FF);
                    }
                    if(start <= REG_MODE && (start + count) > REG_MODE){
                        sync_mode_to_motor();
                        bridge_apply_mode_and_reenable();
                    }
                    if(start <= REG_ENABLE && (start + count) > REG_ENABLE){
                        sync_mode_to_motor();
                        if(s_regs[REG_ENABLE]) {
                            if(PressureSafety_IsLatched()){
                                s_regs[REG_ENABLE] = 0;
                                dm_motor_disable(s_hcan, &motor[Motor1]);
                            }else{
                                if(!bridge_as_ready_for_motion()){
                                    bridge_set_motor_passive();
                                    s_dbg_goal_status |= (1u << 10);
                                }
                                dm_motor_enable(s_hcan, &motor[Motor1]);
                                bridge_schedule_enable_retry(HAL_GetTick());
                                if (s_goal.valid) {
                    bridge_parse_goal_from_regs_and_apply();
                }
                            }
                        } else {
                            s_contact_hold_active = 0u;
                            bridge_grip_hold_reset();
                            dm_motor_disable(s_hcan, &motor[Motor1]);
                        }
                    }

                    s_txbuf[0] = BRIDGE_SLAVE_ADDR;
                    s_txbuf[1] = 0x10;
                    s_txbuf[2] = f[2];
                    s_txbuf[3] = f[3];
                    s_txbuf[4] = f[4];
                    s_txbuf[5] = f[5];
                    txlen = 6;
                }

            }else{
                respond_exception(func, 0x01);
                return;
            }

done:
            {
                uint16_t c = crc16_modbus(s_txbuf, txlen);
                s_txbuf[txlen++] = (uint8_t)(c & 0xFF);
                s_txbuf[txlen++] = (uint8_t)(c >> 8);
                s_txlen = txlen;

                s_tx_pending = 1;
                if (!s_tx_busy) {
                    RS485_EN_W;
                    if (HAL_OK == HAL_UART_Transmit_IT(s_huart, s_txbuf, s_txlen)) {
                        s_tx_busy = 1;
                        s_tx_started_ms = HAL_GetTick();
                    }else{
                        s_tx_pending = 0u;
                        RS485_EN_R;
                    }
                }
                if(!s_ota_reset_requested){
                    HAL_UARTEx_ReceiveToIdle_IT(s_huart, s_rxbuf, RX_BUF_SZ);
                }
            }
        }
    }
}

// ------------------------ CAN 反馈 ------------------------
/**
 * @brief  CAN 接收回调，更新电机反馈寄存器
 * @param  rxh: CAN 帧头
 * @param  data: 数据区，8 字节
 */
void DM_Bridge_OnCanRx(const FDCAN_RxHeaderTypeDef *rxh, const uint8_t data[8]){
    (void)rxh;

    /*
     * 反馈解析始终执行，即使 AS5048A 暂时无效，也保证上位机能读取 MOS/ROTOR 温度。
     */
    dm_motor_fbdata(&motor[Motor1], (uint8_t*)data);


    // 基础反馈寄存器不依赖 bias/AS 有效性；ERR 位在反馈帧 D[0] 高 4 位。
    s_regs[REG_FB_ERR]        = (uint16_t)((data[0] >> 4) & 0x0F);
    s_regs[REG_FB_MOS_TEMP]   = (uint16_t)data[6];
    s_regs[REG_FB_ROTOR_TEMP] = (uint16_t)data[7];

    // 每次上电后用 AS5048A 和当前达妙反馈重建一次 host/motor bias。
    if (!s_boot_as_rebind_done && bridge_as_ready_for_motion()) {
        float scale_now = safe_scale();
        float motor_pos = motor[Motor1].para.pos;
        float host_pos = host_pos_from_as();

        s_host_bias = host_pos - (motor_pos / scale_now);
        s_bias_from_as = 1;
        s_bias_from_persist = 0;
        s_bias_ready = 1;
        s_boot_as_rebind_done = 1u;
        s_first_motion_soft_limit = 1u;
        persist_regs_on_bias_ready(s_host_bias);

        if (s_goal.valid && s_regs[REG_ENABLE]) {
            bridge_parse_goal_from_regs_and_apply();
        }
    }else if(!bridge_as_ready_for_motion()){
        s_dbg_goal_status |= (1u << 10);
    }

    // 位置反馈使用 host 机械坐标，单位 mrad。
    {
        float host_pos = as_is_valid() ? host_pos_from_as() : motor_to_host_pos(motor[Motor1].para.pos);
        bridge_queue_estimator_sample(
            host_pos,
            motor[Motor1].para.vel,
            motor[Motor1].para.tor
        );
        int32_t p_mrad = (int32_t)(host_pos * 1000.0f);
        put_i32(REG_FB_POS_H, p_mrad);
    }
}

static int32_t f_to_i32_x1000(float x)
{
    return (int32_t)(x * 1000.0f);
}

static float u16_to_f_x1000(uint16_t v)
{
    return ((float)v) / 1000.0f;
}

static void estimator_sync_param_to_regs(void)
{
    force_estimator_param_t p;
    ForceEstimator_GetParam(&p);

    s_regs[REG_EST_TOR_TH]      = (uint16_t)(p.torque_contact_th * 1000.0f);
    s_regs[REG_EST_VEL_IDLE_TH] = (uint16_t)(p.vel_idle_th * 1000.0f);
    s_regs[REG_EST_ALPHA]       = (uint16_t)(p.alpha * 1000.0f);
    s_regs[REG_EST_BIAS_LEARN]  = (uint16_t)(p.bias_learn_rate * 1000.0f);
    s_regs[REG_EST_FRIC_B]      = (uint16_t)(p.viscous_coeff * 1000000.0f);
    s_regs[REG_EST_COULOMB_POS] = (uint16_t)(p.coulomb_pos * 1000.0f);
    s_regs[REG_EST_COULOMB_NEG] = (uint16_t)(p.coulomb_neg * 1000.0f);
    s_regs[REG_EST_STATIC_POS]  = (uint16_t)(p.static_pos * 1000.0f);
    s_regs[REG_EST_STATIC_NEG]  = (uint16_t)(p.static_neg * 1000.0f);
    s_regs[REG_EST_VEL_SIGN_TH] = (uint16_t)(p.vel_sign_th * 1000.0f);
    s_regs[REG_EST_CONTACT_DIR] = (p.contact_dir < 0) ? 0xFFFFu : 1u;
    put_i32(REG_EST_BASE_OPEN_H,  f_to_i32_x1000(p.baseline_open_rad));
    put_i32(REG_EST_BASE_CLOSE_H, f_to_i32_x1000(p.baseline_close_rad));
    s_regs[REG_EST_GRIP_TH]     = (uint16_t)(p.grip_hold_th * 1000.0f);
}

static void estimator_sync_regs_to_param(void)
{
    force_estimator_param_t p;
    ForceEstimator_GetParam(&p);

    p.torque_contact_th = u16_to_f_x1000(s_regs[REG_EST_TOR_TH]);
    p.vel_idle_th       = u16_to_f_x1000(s_regs[REG_EST_VEL_IDLE_TH]);
    p.alpha             = u16_to_f_x1000(s_regs[REG_EST_ALPHA]);
    p.bias_learn_rate   = u16_to_f_x1000(s_regs[REG_EST_BIAS_LEARN]);
    p.viscous_coeff     = ((float)s_regs[REG_EST_FRIC_B]) / 1000000.0f;
    p.coulomb_pos       = u16_to_f_x1000(s_regs[REG_EST_COULOMB_POS]);
    p.coulomb_neg       = u16_to_f_x1000(s_regs[REG_EST_COULOMB_NEG]);
    p.static_pos        = u16_to_f_x1000(s_regs[REG_EST_STATIC_POS]);
    p.static_neg        = u16_to_f_x1000(s_regs[REG_EST_STATIC_NEG]);
    p.vel_sign_th       = u16_to_f_x1000(s_regs[REG_EST_VEL_SIGN_TH]);
    p.contact_dir       = (s_regs[REG_EST_CONTACT_DIR] == 0xFFFFu) ? -1 : 1;
    p.baseline_open_rad = ((float)get_s32(REG_EST_BASE_OPEN_H)) / 1000.0f;
    p.baseline_close_rad = ((float)get_s32(REG_EST_BASE_CLOSE_H)) / 1000.0f;
    p.grip_hold_th      = u16_to_f_x1000(s_regs[REG_EST_GRIP_TH]);

    ForceEstimator_SetParam(&p);
}

static void estimator_update_feedback_regs(void)
{
    force_estimator_state_t s;
    ForceEstimator_GetState(&s);

    put_i32(REG_FB_TOR_RAW_H,  f_to_i32_x1000(s.tor_raw));
    put_i32(REG_FB_TOR_FILT_H, f_to_i32_x1000(s.tor_filt));
    put_i32(REG_FB_TOR_EXT_H,  f_to_i32_x1000(s.tor_ext));
    put_i32(REG_FB_FORCE_H,    f_to_i32_x1000(s.force_proxy));
    s_regs[REG_FB_CONTACT_FLAG] = s.contact_flag;
    put_i32(REG_FB_TOR_FRIC_H, f_to_i32_x1000(s.tor_fric));
    put_i32(REG_FB_FRIC_RAW_H,    f_to_i32_x1000(s.fric_raw));
    put_i32(REG_FB_BASELINE_H,    f_to_i32_x1000(s.baseline_tau));
    put_i32(REG_FB_CONTACT_RAW_H, f_to_i32_x1000(s.contact_tau_raw));
    put_i32(REG_FB_CONTACT_NET_H, f_to_i32_x1000(s.contact_tau_after_baseline));
    put_i32(REG_FB_VEL_EST_H,     f_to_i32_x1000(s.vel_rad_s));
    put_i32(REG_FB_POS_EST_H,     f_to_i32_x1000(s.pos_rad));
}
