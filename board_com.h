/**
 ******************************************************************************
 * Copyright (c) 2019 - ~, SCUT-RobotLab Development Team
 * @file    board_com.h
 * @author  mzh
 * @brief   板间通信头文件
 ******************************************************************************
 * @attention
 *
 * if you had modified this file, please make sure your code does not have any
 * bugs, update the version Number, write dowm your name and the date. The most
 * important thing is make sure the users will have clear and definite under-
 * standing through your new brief.
 *
 * <h2><center>&copy; Copyright (c) 2019 - ~, SCUT-RobotLab Development Team.
 * All rights reserved.</center></h2>
 ******************************************************************************
 */
#ifndef _BOARD_COM_H_
#define _BOARD_COM_H_

#ifdef __cplusplus
/* Includes ------------------------------------------------------------------*/
#include "srml.h"
#include "FreeRTOS.h"
#include "queue.h"
/* Private macros ------------------------------------------------------------*/
#define GIMBAL_RXPACK1 0x221
#define GIMBAL_RXPACK2 0x220
#define CHASSIS_RXPACK1 0x222
#define CHASSIS_RXPACK2 0x223
#define CHASSIS_RXPACK_STEER 0x113

/* 板间通信，小发射数据包 ID：0x221 */
#pragma pack(1)
typedef struct
{
	uint16_t bullet_speed;
	uint16_t cooling_rate;
	uint16_t heat_limit;
	uint16_t booster_heat;

} _gimbal_RxPack1;
#pragma pack()

/* 板间通信，状态数据包 ID：0x220*/
#pragma pack(1)
typedef struct
{
	uint8_t source_power_max;
	uint8_t booster_maxspeed;
	uint8_t robot_id;
	uint8_t cap_status;
	uint8_t cap_voltage;
	uint8_t booster_enable;
	uint16_t chassis_flags;

} _gimbal_RxPack2;
#pragma pack()

/* 板间通信，发送目标速度及控制状态 ID：0x222*/
#pragma pack(1)
typedef struct
{
	int16_t chassis_speed_y;
	int16_t chassis_speed_x;
	int16_t chassis_speed_z;
	uint16_t chassis_mode;

} _chassis_RxPack1;
#pragma pack()

#pragma pack(1)
typedef struct
{
	float chassis_rotation_angle;
}_chassis_RxPack2;
#pragma pack()

/* Private type --------------------------------------------------------------*/
/* Exported macros -----------------------------------------------------------*/
/* Exported types ------------------------------------------------------------*/
class BoardCom_Classdef
{
public:
	_gimbal_RxPack1 rx_pack1;
	_gimbal_RxPack2 rx_pack2;
	_chassis_RxPack1 tx_pack1; // 发送给底盘的数据
	_chassis_RxPack2 tx_pack2; // 发送第二个数据包

	void Booster_Receive(uint8_t *Rec_Data); // 接收小发射数据
	void Chassis_Receive(uint8_t *Rec_Data); // 接收底盘控制数据
	void Chassis_Send_Pack1(int16_t _speed_y, int16_t _speed_x,
					  int16_t _speed_z, uint8_t _reset_state);
	void Chassis_Send_Pack2(float _rotation_angle);

	void Set_BalanceInfanty_Flag(uint8_t _lost_ctrl,
								 uint8_t _unlimited_state,
								 uint8_t _ascent_state,
								 uint8_t _leap_state,
								 uint8_t _rotation_state,
								 uint8_t _bulletbay_state,
								 uint8_t _ui_reset_flag,
								 uint8_t _vision_mode_flag1,
								 uint8_t _vision_mode_flag2,
								 uint8_t _self_rescue_state,
								 uint8_t _enable_cmd,
								 uint8_t _sliding_remake,
								 uint8_t _turn90degrees,
								 uint8_t _gg_flag,
								 uint8_t _vision_can_shoot,
								 uint8_t _fri_state);
};
/* Exported function declarations --------------------------------------------*/
#endif

#ifdef __cplusplus
extern "C"
{
#endif
	/* Exported macros -----------------------------------------------------------*/
	/* Exported types ------------------------------------------------------------*/
	/* Exported function declarations --------------------------------------------*/
}
#endif
/************************ COPYRIGHT(C) SCUT-ROBOTLAB **************************/
