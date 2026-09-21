#include "force_estimator.h"
#include <string.h>

/**
 * @brief  力矩接触估算模块
 * @note   输入夹爪位置、速度和电机反馈力矩，输出摩擦补偿力矩、接触力矩和接触标志。
 *         该模块不直接控制电机，只提供上层夹紧保持和调参诊断所需的估算结果。
 */

static force_estimator_param_t g_param;
static force_estimator_state_t g_state;

/**
 * @brief  baseline 查表点
 * @note   ratio_pct 为开合度百分比，tau_nm 为该位置附近的无接触基准力矩。
 */
typedef struct {
    float ratio_pct;
    float tau_nm;
} baseline_point_t;

/* 位置 baseline 表：0% 为全开端，100% 为全闭端。 */
static const baseline_point_t k_baseline_table[] = {
    {  0.0f, 0.178f },
    { 10.0f, 0.166f },
    { 20.0f, 0.129f },
    { 30.0f, 0.129f },
    { 40.0f, 0.099f },
    { 50.0f, 0.096f },
    { 60.0f, 0.098f },
    { 70.0f, 0.100f },
    { 80.0f, 0.105f },
    {100.0f, 0.105f },
};

#define FORCE_EST_DEFAULT_OPEN_RAD  (-29.74f * 0.01745329252f)
#define FORCE_EST_DEFAULT_CLOSE_RAD (0.0f)
#define FORCE_EST_FRIC_ALPHA        (0.05f)
#define FORCE_EST_FRIC_TRANSITION_MUL (10.0f)
#define FORCE_EST_FORCE_PROXY_ALPHA (0.25f)
#define FORCE_EST_CONTACT_RELEASE_RATIO (0.60f)
#define FORCE_EST_CONTACT_RELEASE_CNT   (5u)

#ifndef FORCE_EST_USE_POSITION_BASELINE
#define FORCE_EST_USE_POSITION_BASELINE (0u)
#endif

/**
 * @brief  求浮点绝对值
 * @param  x: 输入值
 * @retval 输入值的绝对值
 */
static float f_abs(float x) { return (x < 0.0f) ? -x : x; }

/**
 * @brief  限幅到 0~1
 * @param  x: 输入值
 * @retval 限幅后的值
 */
static float clamp01(float x)
{
    if (x < 0.0f) return 0.0f;
    if (x > 1.0f) return 1.0f;
    return x;
}

/**
 * @brief  平滑插值曲线
 * @param  x: 输入值，函数内部会先限幅到 0~1
 * @retval 平滑后的 0~1 插值系数
 */
static float smoothstep01(float x)
{
    x = clamp01(x);
    return x * x * (3.0f - 2.0f * x);
}

/**
 * @brief  归一化运动/接触方向
 * @param  dir: 输入方向
 * @retval -1 或 +1
 */
static int8_t norm_dir(int8_t dir)
{
    return (dir < 0) ? -1 : 1;
}

/**
 * @brief  根据开合度百分比线性插值 baseline 力矩
 * @param  ratio_pct: 开合度百分比，0=全开，100=全闭
 * @retval baseline 力矩，单位 N*m
 */
static float interp_baseline(float ratio_pct)
{
    unsigned int i;
    const unsigned int n = (unsigned int)(sizeof(k_baseline_table) / sizeof(k_baseline_table[0]));

    if (ratio_pct <= k_baseline_table[0].ratio_pct) {
        return k_baseline_table[0].tau_nm;
    }
    if (ratio_pct >= k_baseline_table[n - 1u].ratio_pct) {
        return k_baseline_table[n - 1u].tau_nm;
    }

    for (i = 0u; i + 1u < n; i++) {
        const baseline_point_t *a = &k_baseline_table[i];
        const baseline_point_t *b = &k_baseline_table[i + 1u];
        if ((ratio_pct >= a->ratio_pct) && (ratio_pct <= b->ratio_pct)) {
            float span = b->ratio_pct - a->ratio_pct;
            float t = (span > 0.0f) ? ((ratio_pct - a->ratio_pct) / span) : 0.0f;
            return a->tau_nm + (b->tau_nm - a->tau_nm) * t;
        }
    }

    return k_baseline_table[n - 1u].tau_nm;
}

/**
 * @brief  根据当前位置计算 baseline 力矩
 * @param  pos_rad: 当前夹爪位置，单位 rad
 * @retval baseline 力矩，单位 N*m
 */
static float baseline_from_pos(float pos_rad)
{
    float span = g_param.baseline_close_rad - g_param.baseline_open_rad;
    float ratio_pct;

    if (f_abs(span) <= 0.0001f) {
        return 0.0f;
    }

    ratio_pct = 100.0f * (pos_rad - g_param.baseline_open_rad) / span;
    ratio_pct = clamp01(ratio_pct / 100.0f) * 100.0f;
    return interp_baseline(ratio_pct);
}

/**
 * @brief  计算干摩擦幅值
 * @param  abs_vel: 速度绝对值，单位 rad/s
 * @param  static_fric: 静摩擦幅值，单位 N*m
 * @param  coulomb_fric: 库仑摩擦幅值，单位 N*m
 * @retval 当前速度下的干摩擦幅值，单位 N*m
 */
static float dry_friction_mag(float abs_vel, float static_fric, float coulomb_fric)
{
    float v0 = g_param.vel_sign_th;
    float v1 = g_param.vel_sign_th * FORCE_EST_FRIC_TRANSITION_MUL;
    float t;

    if (static_fric <= 0.0f) {
        static_fric = coulomb_fric;
    }
    if (coulomb_fric <= 0.0f) {
        coulomb_fric = static_fric;
    }
    if (v1 <= v0) {
        return coulomb_fric;
    }

    t = smoothstep01((abs_vel - v0) / (v1 - v0));
    return static_fric + (coulomb_fric - static_fric) * t;
}

/**
 * @brief  摩擦力矩模型
 * @param  vel_rad_s: 当前速度，单位 rad/s
 * @retval 摩擦力矩估计值，单位 N*m
 * @note   方向足够明确时使用当前速度方向；低速区沿用最近一次有效运动方向，
 *         避免速度过零时摩擦方向频繁抖动。
 */
static float friction_model(float vel_rad_s)
{
    float abs_vel = f_abs(vel_rad_s);
    int8_t dir;

    if (abs_vel > (g_param.vel_sign_th * 1.5f)) {
        dir = (vel_rad_s > 0.0f) ? 1 : -1;
        g_state.last_motion_dir = dir;
    } else {
        dir = g_state.last_motion_dir;
        if (dir == 0) {
            dir = norm_dir(g_param.contact_dir);
            g_state.last_motion_dir = dir;
        }
    }

    if (dir > 0) {
        float dry = dry_friction_mag(abs_vel, g_param.static_pos, g_param.coulomb_pos);
        return g_param.viscous_coeff * vel_rad_s + dry;
    } else {
        float dry = dry_friction_mag(abs_vel, g_param.static_neg, g_param.coulomb_neg);
        return g_param.viscous_coeff * vel_rad_s - dry;
    }
}

/**
 * @brief  清空力矩估算运行状态
 * @param  None
 * @retval None
 */
void ForceEstimator_Reset(void)
{
    memset(&g_state, 0, sizeof(g_state));
    g_state.last_motion_dir = norm_dir(g_param.contact_dir);
}

/**
 * @brief  初始化力矩估算器默认参数并清空状态
 * @param  None
 * @retval None
 */
void ForceEstimator_Init(void)
{
    g_param.alpha = 0.10f;
    g_param.vel_idle_th = 0.05f;
    g_param.bias_learn_rate = 0.01f;
    /* 仅在接近空载时学习零偏，避免把静态夹持力误吸收到零偏中。 */
    g_param.bias_learn_window = 0.03f;
    g_param.torque_contact_th = 0.08f;
    g_param.grip_hold_th = 0.12f;
    g_param.force_gain = 1.0f;
    g_param.viscous_coeff = 0.0000465141f;
    g_param.coulomb_pos = 0.25f;
    g_param.coulomb_neg = 0.12f;
    g_param.static_pos = 0.12f;
    g_param.static_neg = 0.08f;
    g_param.vel_sign_th = 0.04f;
    g_param.baseline_open_rad = FORCE_EST_DEFAULT_OPEN_RAD;
    g_param.baseline_close_rad = FORCE_EST_DEFAULT_CLOSE_RAD;
    g_param.contact_dir = 1;
    g_param.contact_confirm_cnt = 3u;

    ForceEstimator_Reset();
}

/**
 * @brief  设置力矩估算参数
 * @param  p: 参数结构体指针
 * @retval None
 * @note   该函数会对关键参数做限幅，避免上位机写入异常值导致估算失效。
 */
void ForceEstimator_SetParam(const force_estimator_param_t *p)
{
    if (!p) return;
    g_param = *p;
    if (g_param.alpha < 0.0f) g_param.alpha = 0.0f;
    if (g_param.alpha > 1.0f) g_param.alpha = 1.0f;
    if (g_param.vel_idle_th < 0.0f) g_param.vel_idle_th = 0.0f;
    if (g_param.bias_learn_rate < 0.0f) g_param.bias_learn_rate = 0.0f;
    if (g_param.bias_learn_rate > 1.0f) g_param.bias_learn_rate = 1.0f;
    if (g_param.bias_learn_window < 0.0f) g_param.bias_learn_window = 0.0f;
    if (g_param.torque_contact_th < 0.0f) g_param.torque_contact_th = 0.0f;
    if (g_param.grip_hold_th < 0.0f) g_param.grip_hold_th = 0.0f;
    if (g_param.force_gain < 0.0f) g_param.force_gain = 0.0f;
    if (g_param.viscous_coeff < 0.0f) g_param.viscous_coeff = 0.0f;
    if (g_param.coulomb_pos < 0.0f) g_param.coulomb_pos = 0.0f;
    if (g_param.coulomb_neg < 0.0f) g_param.coulomb_neg = 0.0f;
    if (g_param.static_pos < 0.0f) g_param.static_pos = 0.0f;
    if (g_param.static_neg < 0.0f) g_param.static_neg = 0.0f;
    if (g_param.vel_sign_th <= 0.0f) g_param.vel_sign_th = 0.001f;
    if (f_abs(g_param.baseline_close_rad - g_param.baseline_open_rad) <= 0.0001f) {
        g_param.baseline_open_rad = FORCE_EST_DEFAULT_OPEN_RAD;
        g_param.baseline_close_rad = FORCE_EST_DEFAULT_CLOSE_RAD;
    }
    g_param.contact_dir = norm_dir(g_param.contact_dir);
    if (g_param.contact_confirm_cnt == 0u) g_param.contact_confirm_cnt = 1u;
}

/**
 * @brief  获取当前力矩估算参数
 * @param  p: 参数结构体输出指针
 * @retval None
 */
void ForceEstimator_GetParam(force_estimator_param_t *p)
{
    if (!p) return;
    *p = g_param;
}

/**
 * @brief  获取当前力矩估算状态
 * @param  s: 状态结构体输出指针
 * @retval None
 */
void ForceEstimator_GetState(force_estimator_state_t *s)
{
    if (!s) return;
    *s = g_state;
}

/**
 * @brief  更新一次力矩接触估算
 * @param  pos_rad: 当前夹爪位置，单位 rad
 * @param  vel_rad_s: 当前夹爪速度，单位 rad/s
 * @param  tor_nm: 当前电机反馈力矩，单位 N*m
 * @retval None
 * @note   主要流程：
 *         1. 对原始力矩做低通滤波；
 *         2. 在低速低载条件下更新零偏；
 *         3. 用摩擦模型估算摩擦力矩并计算外力矩；
 *         4. 生成接触判定信号 force_proxy；
 *         5. 通过确认计数完成接触/释放防抖。
 */
void ForceEstimator_Update(float pos_rad, float vel_rad_s, float tor_nm)
{
    float th_on;
    float th_off;

    g_state.pos_rad = pos_rad;
    g_state.vel_rad_s = vel_rad_s;
    g_state.tor_raw = tor_nm;

    g_state.tor_filt += g_param.alpha * (g_state.tor_raw - g_state.tor_filt);

    if ((f_abs(vel_rad_s) < g_param.vel_idle_th) &&
        (f_abs(g_state.tor_raw) < g_param.bias_learn_window))
    {
        g_state.tor_bias += g_param.bias_learn_rate * (g_state.tor_filt - g_state.tor_bias);
    }

    g_state.fric_raw = friction_model(vel_rad_s) + g_state.tor_bias;
    if (f_abs(g_state.tor_fric) < 1e-6f) {
        g_state.tor_fric = g_state.fric_raw;
    } else {
        g_state.tor_fric += FORCE_EST_FRIC_ALPHA * (g_state.fric_raw - g_state.tor_fric);
    }
    g_state.tor_ext = g_state.tor_filt - g_state.tor_fric;

    /*
     * 接触判定优先保证可用性：即使摩擦模型尚未完全辨识，也使用“滤波力矩 - 零偏”
     * 作为夹持方向信号。摩擦补偿后的 tor_ext 仍保留给上位机调参和诊断。
     */
    g_state.contact_tau_raw =
        (float)g_param.contact_dir * (g_state.tor_filt - g_state.tor_bias);
    if (g_state.contact_tau_raw < 0.0f) {
        g_state.contact_tau_raw = 0.0f;
    }
    if (FORCE_EST_USE_POSITION_BASELINE) {
        g_state.baseline_tau = baseline_from_pos(pos_rad);
    } else {
        g_state.baseline_tau = 0.0f;
    }
    g_state.contact_tau_after_baseline = g_state.contact_tau_raw - g_state.baseline_tau;
    if (g_state.contact_tau_after_baseline < 0.0f) {
        g_state.contact_tau_after_baseline = 0.0f;
    }

    g_state.force_proxy += FORCE_EST_FORCE_PROXY_ALPHA *
        ((g_state.contact_tau_after_baseline * g_param.force_gain) - g_state.force_proxy);

    th_on = g_param.torque_contact_th * g_param.force_gain;
    th_off = th_on * FORCE_EST_CONTACT_RELEASE_RATIO;

    /* 接触进入判定：force_proxy 连续超过开启阈值后置位 contact_flag。 */
    if (!g_state.contact_flag) {
        if (g_state.force_proxy > th_on) {
            if (g_state.contact_cnt < 255u) {
                g_state.contact_cnt++;
            }
        } else {
            g_state.contact_cnt = 0u;
        }

        if (g_state.contact_cnt >= g_param.contact_confirm_cnt) {
            g_state.contact_flag = 1u;
            g_state.contact_cnt = 0u;
        }
    } else {
        /* 接触释放判定：force_proxy 连续低于释放阈值后清除 contact_flag。 */
        if (g_state.force_proxy < th_off) {
            if (g_state.contact_cnt < 255u) {
                g_state.contact_cnt++;
            }
        } else {
            g_state.contact_cnt = 0u;
        }

        if (g_state.contact_cnt >= FORCE_EST_CONTACT_RELEASE_CNT) {
            g_state.contact_flag = 0u;
            g_state.contact_cnt = 0u;
        }
    }
}
