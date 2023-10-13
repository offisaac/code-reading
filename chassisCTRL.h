/**
 ******************************************************************************
 * Copyright (c) 2019 - ~, SCUT-RobotLab Development Team
 * @file    chassisCTRL.h
 * @author  mzh
 * @brief   底盘控制文件
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
#ifndef _CHASSISCTRL_H_
#define _CHASSISCTRL_H_

#ifdef __cplusplus
/* Includes ------------------------------------------------------------------*/
#include "SRML.h"
#include "arm_math.h"
/* Private macros ------------------------------------------------------------*/
#define RAD (180) //弧度制，转角度制用
/* Private type --------------------------------------------------------------*/
/* Exported macros -----------------------------------------------------------*/
/* Exported types ------------------------------------------------------------*/
typedef enum Coordinate_System
{
	RECTANGULAR = 0U, //直角坐标
	SPHERICAL,				//球坐标
} Coordinate_System;

class ChassisCTRL_Classdef
{
public:
	ChassisCTRL_Classdef(Coordinate_System _coordinate_system)
	{
		coordinate_system = _coordinate_system;
	}
	/*接口*/
	void Status_Update(float _chassis_yaw_current,
										 float _y_data,
										 float _x_data,
										 float _y_back_data, //使用键鼠时需要用来判断向后和左
										 float _x_back_data,
										 bool *_chassis_resetState,
										 uint8_t *_rotationState,
										 uint8_t *_keyboardState,
										 bool *_bulletBayState);
	void Adjust();
	int16_t GetSpeed_X(); //返回底盘解算的速度或角度数据
	int16_t GetSpeed_Y();
	int16_t GetSpeed_Z();
	int16_t GetAlpha();
	int16_t GetTheta();
	int16_t GetR();
	void ChassisFollowOn(uint8_t _chassis_follow_flag); //开启底盘跟随

	myPID chassis_yawAngle;
	/*采用的坐标系*/
	Coordinate_System coordinate_system = RECTANGULAR;
	/*计算用的变量*/
	float chassis_yaw_current;			//底盘跟随pid当前值
	float chassis_yaw_current_full; //底盘360度角度当前值
	float y_data, x_data, y_back_data, x_back_data;
	int rotateCnt; //记录云台旋转圈数
	uint32_t yaw_encoder_count = 0;	//底盘跟随掉头编码器计数
	/*标志位*/
	bool chassis_resetState; //遥控保护
	uint8_t rotationState = 0;	 //小陀螺
	uint8_t keyboardState;	 //判断是否为键盘控制
	bool bulletBayState;		 //弹舱开启时灵敏度下降
private:
	/*下发到底盘的变量*/
	int16_t speed_X, speed_Y, speed_Z; //直角系
	int16_t alpha, theta, r;					 //球坐标系

	/*计算圈数*/
	void angleCNT();
	/*底盘解算*/
	void data_process();
	/*底盘复位函数*/
	void chassis_reset();
	/*球坐标系转换函数*/
	void convert_to_spherical();
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
