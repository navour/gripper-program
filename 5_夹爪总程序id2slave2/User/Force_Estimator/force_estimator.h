#ifndef FORCE_ESTIMATOR_H
#define FORCE_ESTIMATOR_H

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief  力矩接触估算参数
 * @note   参数由 Modbus 寄存器 0x0036~0x0047 同步，可通过上位机在线调节。
 */
typedef struct {
    float alpha;               // 低通滤波系数 0~1
    float vel_idle_th;         // 速度低于该值认为接近静止 rad/s
    float bias_learn_rate;     // 零偏学习率 0~1
    float bias_learn_window;   // 仅在 |tor_raw| 小于该值时学习零偏
    float torque_contact_th;   // 接触判定阈值 Nm
    float grip_hold_th;        // 夹紧保持阈值 Nm，达到后停止继续闭合
    float force_gain;          // 力矩到代理力缩放系数，当前默认 1
    float viscous_coeff;       // 粘滞摩擦系数，单位 N*m*s/rad
    float coulomb_pos;         // 正向运动库仑摩擦，单位 N*m
    float coulomb_neg;         // 反向运动库仑摩擦，单位 N*m
    float static_pos;          // 正向运动静摩擦，单位 N*m
    float static_neg;          // 反向运动静摩擦，单位 N*m
    float vel_sign_th;         // 运动方向判定速度阈值，单位 rad/s
    float baseline_open_rad;   // baseline 查表对应的全开角 rad
    float baseline_close_rad;  // baseline 查表对应的全闭角 rad
    int8_t contact_dir;        // 接触力矩方向，+1 表示正外力矩为夹持，-1 表示负外力矩为夹持
    uint8_t contact_confirm_cnt; // 连续多少帧超阈才判定接触
} force_estimator_param_t;

/**
 * @brief  力矩接触估算运行状态
 * @note   这些状态会被 Modbus 反馈寄存器 0x010A~0x012C 使用，用于上位机调参和诊断。
 */
typedef struct {
    float tor_raw;                 // 原始电机力矩，单位 N*m
    float tor_filt;                // 低通滤波后的电机力矩，单位 N*m
    float tor_bias;                // 自动学习的零偏力矩，单位 N*m
    float fric_raw;                // 摩擦模型原始输出，单位 N*m
    float tor_fric;                // 平滑后的摩擦估计，单位 N*m
    float tor_ext;                 // 扣除摩擦后的外力矩估计，单位 N*m
    float baseline_tau;            // 位置 baseline 力矩，单位 N*m
    float contact_tau_raw;         // 未扣 baseline 的接触力矩，单位 N*m
    float contact_tau_after_baseline; // 扣 baseline 后的接触力矩，单位 N*m
    float force_proxy;             // 接触判定用代理力信号
    float vel_rad_s;               // 当前估算速度，单位 rad/s
    float pos_rad;                 // 当前估算位置，单位 rad
    uint8_t contact_flag;          // 接触标志，1=已接触，0=未接触
    uint8_t contact_cnt;           // 接触/释放确认计数
    int8_t last_motion_dir;        // 最近一次有效运动方向，+1 或 -1
} force_estimator_state_t;

/**
 * @brief  初始化力矩估算器默认参数并清空状态
 * @param  None
 * @retval None
 */
void ForceEstimator_Init(void);

/**
 * @brief  清空力矩估算运行状态
 * @param  None
 * @retval None
 */
void ForceEstimator_Reset(void);

/**
 * @brief  更新一次力矩接触估算
 * @param  pos_rad: 当前夹爪位置，单位 rad
 * @param  vel_rad_s: 当前夹爪速度，单位 rad/s
 * @param  tor_nm: 当前电机反馈力矩，单位 N*m
 * @retval None
 */
void ForceEstimator_Update(float pos_rad, float vel_rad_s, float tor_nm);

/**
 * @brief  设置力矩估算参数
 * @param  p: 参数结构体指针
 * @retval None
 */
void ForceEstimator_SetParam(const force_estimator_param_t *p);

/**
 * @brief  获取当前力矩估算参数
 * @param  p: 参数结构体输出指针
 * @retval None
 */
void ForceEstimator_GetParam(force_estimator_param_t *p);

/**
 * @brief  获取当前力矩估算状态
 * @param  s: 状态结构体输出指针
 * @retval None
 */
void ForceEstimator_GetState(force_estimator_state_t *s);

#ifdef __cplusplus
}
#endif

#endif
