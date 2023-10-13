/**
 ******************************************************************************
 * Copyright (c) 2019 - ~, SCUT-RobotLab Development Team
 * @file    booster.h
 * @author  mzh
 * @brief   小发射控制代码头文件
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
#ifndef _BOOSTER_H_
#define _BOOSTER_H_

#ifdef __cplusplus
/* Includes ------------------------------------------------------------------*/
#include "SRML.h"
#include "srml_std_lib.h"

/* Private macros ------------------------------------------------------------*/
#define ID_FRI_LEFT (2)
#define ID_FRI_RIGHT (3)
#define ID_Turnplate (1)
/*摩擦轮射速宏定义*/
#define BULLET_SPEED_15 4300;
#define BULLET_SPEED_18 4750;
#define BULLET_SPEED_30 7800;
/*热量上限阈值宏定义*/
#define HEAT_OFFSET (30) // 20Hz射频时上限-38，15Hz、10Hz射频上限-28，2Hz射频上限-13
/*弹舱盖开关限位*/
#define BULLETBAY_ON (500)
#define BULLETBAY_OFF (2225)
/* 弹速自适应滤波长度 */
#define SHOOT_FILTER_STEP 20
/* Private type --------------------------------------------------------------*/
/* Exported macros -----------------------------------------------------------*/
/* Exported types ------------------------------------------------------------*/
class Booster_Classdef
{
public:
	/*创建电机对象*/
	Motor_C620 left_friWheel = Motor_C620(ID_FRI_LEFT);
	Motor_C620 right_friWheel = Motor_C620(ID_FRI_RIGHT);
	Motor_C610 turnplateMotor = Motor_C610(ID_Turnplate);
	/*创建pid对象*/
	myPID left_fri_speedloop;
	myPID right_fri_speedloop;
	myPID turnplate_angleloop;
	myPID turnplate_speedloop;
	// 射速自适应
	float over_bulletSpeed_punish = 50;// 超射速惩罚值
	float bulletSpeed_adapt_gain = 10; // 弹速自适应增益
	float bulletSpeed_filter_out = 0;//滤波后弹速
	float friRPM_accumulation = 0; // 自适应累积转速
	MeanFilter<SHOOT_FILTER_STEP> bulletSpeed_meanfilter;
	/*接口*/
	void Status_Update(bool *_friState,
					   bool *_laserState,
					   bool *_turnplateState,
					   bool *_bulletBayState,
					   bool *_resetState,
					   uint16_t *_bulletSpeed,
					   uint16_t *_heatRf,
					   uint16_t *_coolingRate,
					   uint16_t *_heatLimit,
					   uint8_t *_maxSpeed);
	void Adjust();
	void Set_TurnplateFrq(float _turnplate_frq); // 修改射频的接口
	void Pack_CAN(Motor_CAN_COB *_CANX_pack);	 // 打包函数

	/*摩擦轮相关变量*/
	int16_t friction_wheel_rpm, adaptive_fri_wheel_rpm; // 摩擦轮转速、自适应转速
	int8_t heat_offset = HEAT_OFFSET;					// 热量控制限制值
	/*拨盘相关变量*/
	uint16_t continuous_flag = 0;
	int32_t turnplate_targetAngle;
	float turnplate_frq = 10; // 拨盘射频
	/*打符相关*/
	uint8_t auto_fire;
	/*裁判系统相关变量*/
	float bulletSpeed, lastBulletSpeed;				  // 裁判系统传入
	float heat;										  // 当前热量
	int16_t heatRf, coolingRate, heatLimit, maxSpeed; // 裁判系统传入
	int16_t bulletcounter;							  // 计算发射弹丸数,未使用
private:
	/*小发射复位标志位*/
	bool booster_resetState;
	/* 弹仓盖开关PWM值 */
	uint16_t bulletBay_on = BULLETBAY_ON, bulletBay_off = BULLETBAY_OFF;


	/*摩擦轮相关函数*/
	bool fri_on(bool _fri_state);
	void laser_on(bool _laser_state); // 激光
	/*弹舱相关函数*/
	void bulletBay_ctrl(bool _bulletBay_state);
	/*拨盘相关函数*/
	void turnplate_ctrl(bool _turnplate_state);
	void bulletJam_protection(); // 防卡弹
	/*裁判系统相关函数*/
	bool heat_ctrl();			  // 热量控制
	void bulletVelocity_adjust(); // 摩擦轮自适应
	/*pid计算函数*/
	void booster_pid_calculate();
	/*小发射复位*/
	void booster_reset();
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
