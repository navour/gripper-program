#ifndef __DM_MOTOR_DRV_H__
#define __DM_MOTOR_DRV_H__
#include "main.h"
#include "fdcan.h"
#include "bsp_fdcan.h"

#define MIT_MODE 			0x000
#define POS_MODE			0x100
#define SPD_MODE			0x200
#define PSI_MODE		  	0x300

#define KP_MIN 0.0f
#define KP_MAX 500.0f
#define KD_MIN 0.0f
#define KD_MAX 5.0f

typedef enum
{
    Motor1,
    num
} motor_num;

typedef enum
{
	mit_mode = 1,
	pos_mode = 2,
	spd_mode = 3,
	psi_mode = 4
} mode_e;

typedef enum {
    RID_UV_VALUE=0,    // 母线电压
    RID_KT_VALUE=1,    // 转矩系数
    RID_OT_VALUE=2,    // 过温报警值
    RID_OC_VALUE=3,    // 过流报警值
    RID_ACC		=4,    // 加速度
    RID_DEC		=5,    // 减速度
    RID_MAX_SPD	=6,    // 最大速度
    RID_MST_ID	=7,    // 主站 ID
    RID_ESC_ID	=8,    // 电机 ID
    RID_TIMEOUT	=9,    // 超时设置
    RID_CMODE	=10,   // 控制模式
    RID_DAMP	=11,   // 阻尼系数
    RID_INERTIA =12,   // 转动惯量
    RID_HW_VER	=13,   // 硬件版本
    RID_SW_VER	=14,   // 软件版本
    RID_SN		=15,   // 序列号
    RID_NPP		=16,   // 极对数
    RID_RS		=17,   // Rs
    RID_LS		=18,   // Ls
    RID_FLUX	=19,   // 磁链
    RID_GR		=20,   // 减速比
    RID_PMAX	=21,   // 位置映射范围
    RID_VMAX	=22,   // 速度映射范围
    RID_TMAX	=23,   // 转矩映射范围
    RID_I_BW	=24,   // 电流环带宽
    RID_KP_ASR	=25,   // 速度环 Kp
    RID_KI_ASR	=26,   // 速度环 Ki
    RID_KP_APR	=27,   // 位置环 Kp
    RID_KI_APR	=28,   // 位置环 Ki
    RID_OV_VALUE=29,   // 过压报警值
    RID_GREF	=30,   // 给定参考
    RID_DETA	=31,   // 速度环微分系数
    RID_V_BW	=32,   // 速度环带宽
    RID_IQ_CL	=33,   // Iq 限制系数
    RID_VL_CL	=34,   // 速度限制系数
    RID_CAN_BR	=35,   // CAN 波特率
    RID_SUB_VER	=36,   // 子版本号
    RID_U_OFF	=50,   // U 相偏置
    RID_V_OFF	=51,   // V 相偏置
    RID_K1		=52,   // 标定系数 1
    RID_K2		=53,   // 标定系数 2
    RID_M_OFF	=54,   // 角度偏置
    RID_DIR		=55,   // 方向
    RID_P_M		=80,   // 机械位置
    RID_X_OUT	=81    // 输出轴位置
} rid_e;




// 电机参数信息
typedef struct
{
	uint8_t read_flag;
	uint8_t write_flag;
	uint8_t save_flag;
	
    float UV_Value;     // 母线电压
    float KT_Value;     // 转矩系数
    float OT_Value;     // 过温报警值
    float OC_Value;     // 过流报警值
    float ACC;          // 加速度
    float DEC;          // 减速度
    float MAX_SPD;      // 最大速度
    uint32_t MST_ID;    // 主站 ID
    uint32_t ESC_ID;    // 电机 ID
    uint32_t TIMEOUT;   // 超时设置
    uint32_t cmode;     // 控制模式
    float    Damp;      // 阻尼系数
    float    Inertia;   // 转动惯量
    uint32_t hw_ver;    // 硬件版本
    uint32_t sw_ver;    // 软件版本
    uint32_t SN;        // 序列号
    uint32_t NPP;       // 极对数
    float    Rs;        // Rs
    float    Ls;        // Ls
    float    Flux;      // 磁链
    float    Gr;        // 减速比
    float    PMAX;      // 位置映射范围
    float    VMAX;      // 速度映射范围
    float    TMAX;      // 转矩映射范围
    float    I_BW;      // 电流环带宽
    float    KP_ASR;    // 速度环 Kp
    float    KI_ASR;    // 速度环 Ki
    float    KP_APR;    // 位置环 Kp
    float    KI_APR;    // 位置环 Ki
    float    OV_Value;  // 过压报警值
    float    GREF;      // 给定参考
    float    Deta;      // 速度环微分系数
    float    V_BW;      // 速度环带宽
    float    IQ_cl;     // Iq 限制系数
    float    VL_cl;     // 速度限制系数
    uint32_t can_br;    // CAN 波特率
    uint32_t sub_ver;   // 子版本号
	float    u_off;      // U 相偏置
	float	 v_off;      // V 相偏置
	float	 k1;         // 标定系数 1
	float    k2;         // 标定系数 2
	float    m_off;      // 角度偏置
	float    dir;        // 方向
	float	 p_m;        // 机械位置
	float	 x_out;      // 输出轴位置
} esc_inf_t;

// 电机反馈信息结构
typedef struct
{
    int id;
    int state;
    int p_int;
    int v_int;
    int t_int;
    int kp_int;
    int kd_int;
    float pos;
    float vel;
    float tor;
    float Kp;
    float Kd;
    float Tmos;
    float Tcoil;
} motor_fbpara_t;

// 电机控制参数结构
typedef struct
{
    uint8_t mode;
    float pos_set;
    float vel_set;
    float tor_set;
	float cur_set;
    float kp_set;
    float kd_set;
} motor_ctrl_t;

typedef struct
{
    uint16_t id;
	uint16_t mst_id;
    motor_fbpara_t para;
    motor_ctrl_t ctrl;
	esc_inf_t tmp;
} motor_t;



float uint_to_float(int x_int, float x_min, float x_max, int bits);
int float_to_uint(float x_float, float x_min, float x_max, int bits);
void dm_motor_ctrl_send(hcan_t* hcan, motor_t *motor);
void dm_motor_enable(hcan_t* hcan, motor_t *motor);
void dm_motor_disable(hcan_t* hcan, motor_t *motor);
void dm_motor_clear_para(motor_t *motor);
void dm_motor_clear_err(hcan_t* hcan, motor_t *motor);
void dm_motor_fbdata(motor_t *motor, uint8_t *rx_data);

void enable_motor_mode(hcan_t* hcan, uint16_t motor_id, uint16_t mode_id);
void disable_motor_mode(hcan_t* hcan, uint16_t motor_id, uint16_t mode_id);

void mit_ctrl(hcan_t* hcan, motor_t *motor, uint16_t motor_id, float pos, float vel,float kp, float kd, float tor);
void pos_ctrl(hcan_t* hcan, uint16_t motor_id, float pos, float vel);
void spd_ctrl(hcan_t* hcan, uint16_t motor_id, float vel);
void psi_ctrl(hcan_t* hcan, uint16_t motor_id, float pos, float vel, float cur);
	
void save_pos_zero(hcan_t* hcan, uint16_t motor_id, uint16_t mode_id);
void clear_err(hcan_t* hcan, uint16_t motor_id, uint16_t mode_id);

void read_motor_data(uint16_t id, uint8_t rid);
void read_motor_ctrl_fbdata(uint16_t id);
void write_motor_data(uint16_t id, uint8_t rid, uint8_t d0, uint8_t d1, uint8_t d2, uint8_t d3);
void save_motor_data(uint16_t id, uint8_t rid);

#endif /* __DM_MOTOR_DRV_H__ */

