#include "dm_motor_drv.h"
#include "fdcan.h"


/**
 * @brief  使能电机控制模式
 * @param  hcan: CAN 句柄
 * @param  motor: 电机结构体指针
 * @retval None
 */
void dm_motor_enable(hcan_t* hcan, motor_t *motor)
{
	switch(motor->ctrl.mode)
	{
		case mit_mode:
			enable_motor_mode(hcan, motor->id, MIT_MODE);
			break;
		case pos_mode:
			enable_motor_mode(hcan, motor->id, POS_MODE);
			break;
		case spd_mode:
			enable_motor_mode(hcan, motor->id, SPD_MODE);
			break;
		case psi_mode:
			enable_motor_mode(hcan, motor->id, PSI_MODE);
			break;
		default:
			motor->ctrl.mode = mit_mode;
			enable_motor_mode(hcan, motor->id, MIT_MODE);
			break;
	}	
}
/**
 * @brief  禁用电机控制模式
 * @param  hcan: CAN 句柄
 * @param  motor: 电机结构体指针
 * @retval None
 */
void dm_motor_disable(hcan_t* hcan, motor_t *motor)
{
	switch(motor->ctrl.mode)
	{
		case mit_mode:
			disable_motor_mode(hcan, motor->id, MIT_MODE);
			break;
		case pos_mode:
			disable_motor_mode(hcan, motor->id, POS_MODE);
			break;
		case spd_mode:
			disable_motor_mode(hcan, motor->id, SPD_MODE);
			break;
		case psi_mode:
			disable_motor_mode(hcan, motor->id, PSI_MODE);
			break;
		default:
			motor->ctrl.mode = mit_mode;
			disable_motor_mode(hcan, motor->id, MIT_MODE);
			break;
	}	
	dm_motor_clear_para(motor);
}
/**
 * @brief  发送电机控制命令
 * @param  hcan: CAN 句柄
 * @param  motor: 电机结构体指针
 * @retval None
 */
void dm_motor_ctrl_send(hcan_t* hcan, motor_t *motor)
{
	switch(motor->ctrl.mode)
	{
		case mit_mode:
			mit_ctrl(hcan, motor, motor->id, motor->ctrl.pos_set, motor->ctrl.vel_set, motor->ctrl.kp_set, motor->ctrl.kd_set, motor->ctrl.tor_set);
			break;
		case pos_mode:
			pos_ctrl(hcan, motor->id, motor->ctrl.pos_set, motor->ctrl.vel_set);
			break;
		case spd_mode:
			spd_ctrl(hcan, motor->id, motor->ctrl.vel_set);
			break;
		case psi_mode:
			psi_ctrl(hcan, motor->id,motor->ctrl.pos_set, motor->ctrl.vel_set, motor->ctrl.cur_set);
			break;
	}	
}

/**
 * @brief  清零电机控制参数
 * @param  motor: 电机结构体指针
 * @retval None
 */
void dm_motor_clear_para(motor_t *motor)
{
	motor->ctrl.kd_set 	= 0;
	motor->ctrl.kp_set	= 0;
	motor->ctrl.pos_set = 0;
	motor->ctrl.vel_set = 0;
	motor->ctrl.tor_set = 0;
	motor->ctrl.cur_set = 0;
}

/**
 * @brief  清除电机错误状态
 * @param  hcan: CAN 句柄
 * @param  motor: 电机结构体指针
 * @retval None
 */
void dm_motor_clear_err(hcan_t* hcan, motor_t *motor)
{
	switch(motor->ctrl.mode)
	{
		case mit_mode:
			clear_err(hcan, motor->id, MIT_MODE);
			break;
		case pos_mode:
			clear_err(hcan, motor->id, POS_MODE);
			break;
		case spd_mode:
			clear_err(hcan, motor->id, SPD_MODE);
			break;
		case psi_mode:
			clear_err(hcan, motor->id, PSI_MODE);
			break;
	}	
}
/**
 * @brief  解析电机反馈数据
 * @param  motor: 电机结构体指针
 * @param  rx_data: 接收数据缓冲区
 * @retval None
 */
void dm_motor_fbdata(motor_t *motor, uint8_t *rx_data)
{
	motor->para.id = (rx_data[0])&0x0F;
	motor->para.state = (rx_data[0])>>4;
	motor->para.p_int=(rx_data[1]<<8)|rx_data[2];
	motor->para.v_int=(rx_data[3]<<4)|(rx_data[4]>>4);
	motor->para.t_int=((rx_data[4]&0xF)<<8)|rx_data[5];
	motor->para.pos = uint_to_float(motor->para.p_int, -motor->tmp.PMAX, motor->tmp.PMAX, 16); // (-12.5,12.5)
	motor->para.vel = uint_to_float(motor->para.v_int, -motor->tmp.VMAX, motor->tmp.VMAX, 12); // (-45.0,45.0)
	motor->para.tor = uint_to_float(motor->para.t_int, -motor->tmp.TMAX, motor->tmp.TMAX, 12); // (-18.0,18.0)
	motor->para.Tmos = (float)(rx_data[6]);
	motor->para.Tcoil = (float)(rx_data[7]);
}

/**
 * @brief  浮点数映射为无符号整数
 * @param  x_float: 待转换的浮点数
 * @param  x_min: 范围最小值
 * @param  x_max: 范围最大值
 * @param  bits: 目标位宽
 * @retval 无符号整数结果
 */
int float_to_uint(float x_float, float x_min, float x_max, int bits)
{
	// Clamp before encoding; otherwise values above PMAX wrap after uint16_t cast.
	if (x_float < x_min) x_float = x_min;
	if (x_float > x_max) x_float = x_max;
	// Map the clamped value into the unsigned protocol range.
	float span = x_max - x_min;
	float offset = x_min;
	return (int) ((x_float-offset)*((float)((1<<bits)-1))/span);
}
/**
 * @brief  无符号整数映射为浮点数
 * @param  x_int: 待转换的无符号整数
 * @param  x_min: 范围最小值
 * @param  x_max: 范围最大值
 * @param  bits: 位宽
 * @retval 浮点数结果
 */
float uint_to_float(int x_int, float x_min, float x_max, int bits)
{
	// 按范围线性映射到浮点数
	float span = x_max - x_min;
	float offset = x_min;
	return ((float)x_int)*span/((float)((1<<bits)-1)) + offset;
}

/**
 * @brief  发送电机模式使能命令
 * @param  hcan: CAN 句柄
 * @param  motor_id: 电机 ID
 * @param  mode_id: 模式 ID
 * @retval None
 */
void enable_motor_mode(hcan_t* hcan, uint16_t motor_id, uint16_t mode_id)
{
	uint8_t data[8];
	uint16_t id = motor_id + mode_id;
	
	data[0] = 0xFF;
	data[1] = 0xFF;
	data[2] = 0xFF;
	data[3] = 0xFF;
	data[4] = 0xFF;
	data[5] = 0xFF;
	data[6] = 0xFF;
	data[7] = 0xFC;
	
	fdcanx_send_data(hcan, id, data, 8);
}
/**
 * @brief  发送电机模式禁用命令
 * @param  hcan: CAN 句柄
 * @param  motor_id: 电机 ID
 * @param  mode_id: 模式 ID
 * @retval None
 */
void disable_motor_mode(hcan_t* hcan, uint16_t motor_id, uint16_t mode_id)
{
	uint8_t data[8];
	uint16_t id = motor_id + mode_id;
	
	data[0] = 0xFF;
	data[1] = 0xFF;
	data[2] = 0xFF;
	data[3] = 0xFF;
	data[4] = 0xFF;
	data[5] = 0xFF;
	data[6] = 0xFF;
	data[7] = 0xFD;
	
	fdcanx_send_data(hcan, id, data, 8);
}
/**
 * @brief  保存位置零点
 * @param  hcan: CAN 句柄
 * @param  motor_id: 电机 ID
 * @param  mode_id: 模式 ID
 * @retval None
 */
void save_pos_zero(hcan_t* hcan, uint16_t motor_id, uint16_t mode_id)
{
	uint8_t data[8];
	uint16_t id = motor_id + mode_id;
	
	data[0] = 0xFF;
	data[1] = 0xFF;
	data[2] = 0xFF;
	data[3] = 0xFF;
	data[4] = 0xFF;
	data[5] = 0xFF;
	data[6] = 0xFF;
	data[7] = 0xFE;
	
	fdcanx_send_data(hcan, id, data, 8);
}
/**
 * @brief  清除电机错误
 * @param  hcan: CAN 句柄
 * @param  motor_id: 电机 ID
 * @param  mode_id: 模式 ID
 * @retval None
 */
void clear_err(hcan_t* hcan, uint16_t motor_id, uint16_t mode_id)
{
	uint8_t data[8];
	uint16_t id = motor_id + mode_id;
	
	data[0] = 0xFF;
	data[1] = 0xFF;
	data[2] = 0xFF;
	data[3] = 0xFF;
	data[4] = 0xFF;
	data[5] = 0xFF;
	data[6] = 0xFF;
	data[7] = 0xFB;
	
	fdcanx_send_data(hcan, id, data, 8);
}
/**
 * @brief  MIT 模式控制
 * @param  hcan: CAN 句柄
 * @param  motor: 电机结构体指针
 * @param  motor_id: 电机 ID
 * @param  pos: 位置给定值
 * @param  vel: 速度给定值
 * @param  kp: 位置比例系数
 * @param  kd: 位置微分系数
 * @param  tor: 转矩给定值
 * @retval None
 */
void mit_ctrl(hcan_t* hcan, motor_t *motor, uint16_t motor_id, float pos, float vel,float kp, float kd, float tor)
{
	uint8_t data[8];
	uint16_t pos_tmp,vel_tmp,kp_tmp,kd_tmp,tor_tmp;
	uint16_t id = motor_id + MIT_MODE;

	pos_tmp = float_to_uint(pos, -motor->tmp.PMAX, motor->tmp.PMAX, 16);
	vel_tmp = float_to_uint(vel, -motor->tmp.VMAX, motor->tmp.VMAX, 12);
	tor_tmp = float_to_uint(tor, -motor->tmp.TMAX, motor->tmp.TMAX, 12);
	kp_tmp  = float_to_uint(kp,  KP_MIN, KP_MAX, 12);
	kd_tmp  = float_to_uint(kd,  KD_MIN, KD_MAX, 12);

	data[0] = (pos_tmp >> 8);
	data[1] = pos_tmp;
	data[2] = (vel_tmp >> 4);
	data[3] = ((vel_tmp&0xF)<<4)|(kp_tmp>>8);
	data[4] = kp_tmp;
	data[5] = (kd_tmp >> 4);
	data[6] = ((kd_tmp&0xF)<<4)|(tor_tmp>>8);
	data[7] = tor_tmp;
	
	fdcanx_send_data(hcan, id, data, 8);
}
/**
 * @brief  位置-速度控制
 * @param  hcan: CAN 句柄
 * @param  motor_id: 电机 ID
 * @param  pos: 位置给定值
 * @param  vel: 速度给定值
 * @retval None
 */
void pos_ctrl(hcan_t* hcan,uint16_t motor_id, float pos, float vel)
{
	uint16_t id;
	uint8_t *pbuf, *vbuf;
	uint8_t data[8];
	
	id = motor_id + POS_MODE;
	pbuf=(uint8_t*)&pos;
	vbuf=(uint8_t*)&vel;
	
	data[0] = *pbuf;
	data[1] = *(pbuf+1);
	data[2] = *(pbuf+2);
	data[3] = *(pbuf+3);

	data[4] = *vbuf;
	data[5] = *(vbuf+1);
	data[6] = *(vbuf+2);
	data[7] = *(vbuf+3);
	
	fdcanx_send_data(hcan, id, data, 8);
}
/**
 * @brief  速度控制
 * @param  hcan: CAN 句柄
 * @param  motor_id: 电机 ID
 * @param  vel: 速度给定值
 * @retval None
 */
void spd_ctrl(hcan_t* hcan, uint16_t motor_id, float vel)
{
	uint16_t id;
	uint8_t *vbuf;
	uint8_t data[4];
	
	id = motor_id + SPD_MODE;
	vbuf=(uint8_t*)&vel;
	
	data[0] = *vbuf;
	data[1] = *(vbuf+1);
	data[2] = *(vbuf+2);
	data[3] = *(vbuf+3);
	
	fdcanx_send_data(hcan, id, data, 4);
}

/**
 * @brief  位置-速度-电流混控
 * @param  hcan: CAN 句柄
 * @param  motor_id: 电机 ID
 * @param  pos: 位置给定值
 * @param  vel: 速度给定值
 * @param  cur: 电流给定值
 * @retval None
 */
void psi_ctrl(hcan_t* hcan, uint16_t motor_id, float pos, float vel, float cur)
{
	uint16_t id;
	uint8_t *pbuf, *vbuf, *ibuf;
	uint8_t data[8];
	
	uint16_t u16_vel = vel*100;
	uint16_t u16_cur  = cur*10000;
	
	id = motor_id + PSI_MODE;
	pbuf=(uint8_t*)&pos;
	vbuf=(uint8_t*)&u16_vel;
	ibuf=(uint8_t*)&u16_cur;
	
	data[0] = *pbuf;
	data[1] = *(pbuf+1);
	data[2] = *(pbuf+2);
	data[3] = *(pbuf+3);

	data[4] = *vbuf;
	data[5] = *(vbuf+1);
	
	data[6] = *ibuf;
	data[7] = *(ibuf+1);
	
	fdcanx_send_data(hcan, id, data, 8);
}
/**
 * @brief  发送读取寄存器命令
 * @param  id: 电机 CAN ID
 * @param  rid: 寄存器地址
 * @retval None
 */
void read_motor_data(uint16_t id, uint8_t rid) 
{
	uint8_t can_id_l = id & 0x0F;
	uint8_t can_id_h = (id >> 4) & 0x0F;
	
	uint8_t data[4] = {can_id_l, can_id_h, 0x33, rid};
	fdcanx_send_data(&hfdcan1, 0x7FF, data, 4);
}
/**
 * @brief  发送读取电机控制反馈数据命令
 * @param  id: 电机 CAN ID
 * @retval None
 */
void read_motor_ctrl_fbdata(uint16_t id) 
{
	uint8_t can_id_l = id & 0xFF;       // 低 8 位
    uint8_t can_id_h = (id >> 8) & 0x07; // 高 3 位

	uint8_t data[4] = {can_id_l, can_id_h, 0xCC, 0x00};
	fdcanx_send_data(&hfdcan1, 0x7FF, data, 4);
}
/**
 * @brief  发送写寄存器命令
 * @param  id: 电机 CAN ID
 * @param  rid: 寄存器地址
 * @param  d0: 数据字节 0
 * @param  d1: 数据字节 1
 * @param  d2: 数据字节 2
 * @param  d3: 数据字节 3
 * @retval None
 */
void write_motor_data(uint16_t id, uint8_t rid, uint8_t d0, uint8_t d1, uint8_t d2, uint8_t d3)
{
	uint8_t can_id_l = id & 0xFF;       // 低 8 位
    uint8_t can_id_h = (id >> 8) & 0x07; // 高 3 位
	
	uint8_t data[8] = {can_id_l, can_id_h, 0x55, rid, d0, d1, d2, d3};
	fdcanx_send_data(&hfdcan1, 0x7FF, data, 8);
}
/**
 * @brief  发送保存参数命令
 * @param  id: 电机 CAN ID
 * @param  rid: 寄存器地址
 * @retval None
 */
void save_motor_data(uint16_t id, uint8_t rid) 
{
	uint8_t can_id_l = id & 0xFF;       // 低 8 位
    uint8_t can_id_h = (id >> 8) & 0x07; // 高 3 位
	
	uint8_t data[4] = {can_id_l, can_id_h, 0xAA, 0x01};
	fdcanx_send_data(&hfdcan1, 0x7FF, data, 4);
}

