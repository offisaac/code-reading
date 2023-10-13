/**
 ******************************************************************************
 * Copyright (c) 2019 - ~, SCUT-RobotLab Development Team
 * @file chassisCTRL.cpp
 * @author mzh
 * @brief 底盘控制
 * @date 2022-02-14
 * @version 2.0
 * @par Change Log：
 * <table>
 * <tr><th>Date <th>Version <th>Author <th>Description
 * <tr><td>2019-06-12 <td> 1.0 <td>S.B. <td>Creator
 * </table>
 *
 ==============================================================================
 ##### How to use this driver #####
 ==============================================================================
 @note
 -#
 ******************************************************************************
 * @attention
 *
 * if you had modified this file, please make sure your code does not have many
 * bugs, update the version Number, write dowm your name and the date, the most
* important is make sure the users will have clear and definite understanding
 * through your new brief.
 *
 * <h2><center>&copy; Copyright (c) 2019 - ~, SCUT-RobotLab Development Team.
 * All rights reserved.</center></h2>
 ******************************************************************************
 */
/* Includes ------------------------------------------------------------------*/
#include "chassisCTRL.h"
/* Private define ------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
/* Private type --------------------------------------------------------------*/
/* Private function declarations ---------------------------------------------*/
/* function prototypes -------------------------------------------------------*/
/**
 * @brief 接口，更新底盘控制数据
 * @param None
 * @retval None
 */
void ChassisCTRL_Classdef::Status_Update(float _chassis_yaw_current,
										 float _y_data,
										 float _x_data,
										 float _y_back_data, // 使用键鼠时需要用来判断向后和左
										 float _x_back_data,
										 bool *_chassis_resetState,
										 uint8_t *_rotationState,
										 uint8_t *_keyboardState,
										 bool *_bulletBayState)
{
	y_data = _y_data;
	x_data = _x_data;
	y_back_data = _y_back_data;
	x_back_data = _x_back_data;
	chassis_yaw_current = _chassis_yaw_current;
	chassis_resetState = *_chassis_resetState;
	keyboardState = *_keyboardState;
	bulletBayState = *_bulletBayState;
	//rotationState = *_rotationState;
	if (*_rotationState == 1)
	{
		rotationState = 1;
	}
	else
	{
		if (chassis_yawAngle.Error < -120 && chassis_yawAngle.Error > -140)
		{
			rotationState = 0;
		}
	}
}
/**
 * @brief 接口，处理底盘控制数据
 * @param None
 * @retval None
 */
void ChassisCTRL_Classdef::Adjust()
{
	angleCNT();
	data_process();
	if (chassis_resetState)
	{
		chassis_reset();
	}
	/*球坐标系转换*/
	if (coordinate_system == SPHERICAL)
	{
		convert_to_spherical();
	}
}
/**
 * @brief 接口，返回底盘解算的速度或角度数据
 * @param None
 * @retval None
 */
int16_t ChassisCTRL_Classdef::GetSpeed_X()
{
	return speed_X;
}
int16_t ChassisCTRL_Classdef::GetSpeed_Y()
{
	return speed_Y;
}
int16_t ChassisCTRL_Classdef::GetSpeed_Z()
{
	return speed_Z;
}
int16_t ChassisCTRL_Classdef::GetAlpha()
{
	return alpha;
}
int16_t ChassisCTRL_Classdef::GetTheta()
{
	return theta;
}
int16_t ChassisCTRL_Classdef::GetR()
{
	return r;
}
/**
 * @brief 底盘跟随开关
 * @param None
 * @retval None
 */
void ChassisCTRL_Classdef::ChassisFollowOn(uint8_t _chassis_follow_flag)
{
	static float chassis_follow_para[6];
	/*当数组为空时存储原始pid数据*/
	if (chassis_follow_para[4] == 0)
	{
		chassis_follow_para[0] = chassis_yawAngle.Kp;
		chassis_follow_para[1] = chassis_yawAngle.Ki;
		chassis_follow_para[2] = chassis_yawAngle.Kd;
		chassis_follow_para[3] = chassis_yawAngle.I_Term_Max;
		chassis_follow_para[4] = chassis_yawAngle.PI_Term_Max;
		chassis_follow_para[5] = chassis_yawAngle.Out_Max;
	}
	/*关闭底盘跟随pid全置零*/
	if (_chassis_follow_flag)
		chassis_yawAngle.SetPIDParam(chassis_follow_para[0], chassis_follow_para[1], chassis_follow_para[2], chassis_follow_para[3], chassis_follow_para[4], chassis_follow_para[5]);
	else
		chassis_yawAngle.SetPIDParam(0, 0, 0, 0, 0, 0);
}
/**
 * @brief 计算小陀螺圈数
 * @param None
 * @retval None
 */
void ChassisCTRL_Classdef::angleCNT()
{
	if (chassis_yaw_current - rotateCnt * 360 > 360)
	{
		rotateCnt++;
	}
	if (chassis_yaw_current - rotateCnt * 360 < -360)
	{
		rotateCnt--;
	}
}
/**
 * @brief 底盘速度值更新（底盘跟随+小陀螺）
 * @param None
 * @retval None
 */
void ChassisCTRL_Classdef::data_process()
{
	static int16_t velocity[4]; // 键盘速度临时数据
	/*键盘控制*/
	if (keyboardState)
	{
		/*运动优化，使速度不会突变*/
		(y_data > 0.5f) ? (velocity[0] + 50 < 1023 ? velocity[0] += 50 : velocity[0] = 1023) : (velocity[0] - 50 > 0 ? velocity[0] -= 50 : velocity[0] = 0);
		(x_data > 0.5f) ? (velocity[2] + 50 < 1023 ? velocity[2] += 50 : velocity[2] = 1023) : (velocity[2] - 50 > 0 ? velocity[2] -= 50 : velocity[2] = 0);
		(y_back_data > 0.5f) ? (velocity[1] + 50 < 1023 ? velocity[1] += 50 : velocity[1] = 1023) : (velocity[1] - 50 > 0 ? velocity[1] -= 50 : velocity[1] = 0);
		(x_back_data > 0.5f) ? (velocity[3] + 50 < 1023 ? velocity[3] += 50 : velocity[3] = 1023) : (velocity[3] - 50 > 0 ? velocity[3] -= 50 : velocity[3] = 0);

		// speed_Y = (velocity[0] - velocity[1]) * arm_cos_f32(chassis_yaw_current / RAD * PI) + (velocity[2] - velocity[3]) * arm_sin_f32(chassis_yaw_current / RAD * PI);
		// speed_X = (velocity[2] - velocity[3]) * arm_cos_f32(chassis_yaw_current / RAD * PI) - (velocity[0] - velocity[1]) * arm_sin_f32(chassis_yaw_current / RAD * PI);
		speed_Y = velocity[0] - velocity[1];
		speed_X = velocity[2] - velocity[3];
		/*开弹舱时降低移动灵敏度*/
		if (bulletBayState)
		{
			speed_Y *= 0.25;
			speed_X *= 0.25;
		}
	}

	/*遙控器控制*/
	else
	{
		// speed_Y = y_data * 1023 * arm_cos_f32(chassis_yaw_current / RAD * PI) + x_data * 1023 * arm_sin_f32(chassis_yaw_current / RAD * PI);

		// speed_X = x_data * 1023 * arm_cos_f32(chassis_yaw_current / RAD * PI) - y_data * 1023 * arm_sin_f32(chassis_yaw_current / RAD * PI);

		speed_Y = y_data * 1023;
		speed_X = x_data * 1023;
	}

	/*使底盘总是从最小角度跟随*/
	if (chassis_yaw_current - rotateCnt * 360 > 180)
	{
		chassis_yawAngle.Current = chassis_yaw_current - rotateCnt * 360 - 360;
	}
	else if (chassis_yaw_current - rotateCnt * 360 < -180)
	{
		chassis_yawAngle.Current = chassis_yaw_current - rotateCnt * 360 + 360;
	}
	else
	{
		chassis_yawAngle.Current = chassis_yaw_current - rotateCnt * 360;
	}
	/*底盘跟随PID计算*/
	chassis_yawAngle.Adjust();

	if (rotationState)
	{
		speed_Z = 1023;
	}
	else
	{
		speed_Z = chassis_yawAngle.Out;
	}
	chassis_yaw_current_full = chassis_yaw_current - rotateCnt * 360;

	if (yaw_encoder_count % 2 == 1)
	{
		chassis_yaw_current_full += 180;
	}
	else
	{
	}
}
/**
 * @brief 底盘复位
 * @param None
 * @retval None
 */
void ChassisCTRL_Classdef::chassis_reset()
{
	speed_X = 0;
	speed_Y = 0;
	speed_Z = 0;
}
/**
 * @brief 将直角系转化成球坐标系
 * @param alpha为当前方向与Z轴方向夹角，theta为当前方向的水平面投影与X轴方向夹角
 * @retval None
 */
void ChassisCTRL_Classdef::convert_to_spherical()
{
	(sqrtf(powf(speed_X, 2) + powf(speed_Y, 2) + powf(speed_Z, 2)) > 1023) ? (r = 1023) : (r = sqrtf(powf(speed_X, 2) + powf(speed_Y, 2) + powf(speed_Z, 2)));
	alpha = acosf(speed_Z / sqrtf(powf(speed_X, 2) + powf(speed_Y, 2) + powf(speed_Z, 2))) / PI * 180 * 10;
	theta = int16_t(atanf(speed_Y / (float)speed_X) / PI * 180 * 5); // 强制转化成float时arctan角度小于1时能正常运算
	if (speed_X == 0)
	{
		theta = (speed_Y > 0) ? 90 * 5 : ((speed_Y < 0) ? 270 * 5 : theta);
	}
	else if (speed_X < 0)
	{
		theta += 180 * 5;
	}
	if (theta < 0)
	{
		theta += 360 * 5;
	}
}
/************************ COPYRIGHT(C) SCUT-ROBOTLAB**************************/
