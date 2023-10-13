/**
 ******************************************************************************
 * Copyright (c) 2019 - ~, SCUT-RobotLab Development Team
 * @file PCvision.cpp
 * @author mzh
 * @brief 用于执行视觉视觉对接相关操作,包括 接收数据 和 返回数据
 * @date 2022-02-13
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
 @warning
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
#include "PCvision.h"
/* Exported macros -----------------------------------------------------------*/
extern UART_HandleTypeDef huart1;
/* Private define ------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
/* Private type --------------------------------------------------------------*/
/* Private function declarations ---------------------------------------------*/
/* function prototypes -------------------------------------------------------*/
/**
 * @brief  串口回调函数，处理视觉回传数据，通过union共用内存的特性获取数据
 * @param  Recv_Data:接收缓存 ReceiveLen:接收缓存长度
 * @return NULL
 */

uint32_t PCvision_Classdef::GetViaionData(uint8_t *Recv_Data, uint16_t ReceiveLen) // ReceiveLen为实际数据长度
{
	lostDelay = 50;
	return 0;
}

/**
 * @brief  串口向视觉发送数据
 * @param  NULL
 * @return NULL
 */
float angle_delay_time = 1.f;
float anglevelocity_delay_time = 1.f;
void PCvision_Classdef::SendGimbleStatus()
{
	//	static PackToVision_Def PackToVision;
	PackToVision.head = VISION_HEAD;
	PackToVision.end = VISION_END;

//	PackToVision.pitch_speed = PitchAngularSpeed_PerviousData.GetPreviousData(anglevelocity_delay_time);
//	PackToVision.yaw_speed = YawAngularSpeed_PerviousData.GetPreviousData(anglevelocity_delay_time);

//	PackToVision.pitch_angle = PitchPerviousData.GetPreviousData(angle_delay_time);
//	PackToVision.yaw_angle = YawPerviousData.GetPreviousData(angle_delay_time);
	
	PackToVision.pitch_speed = pitch_angular_speed;
	PackToVision.yaw_speed = yaw_angular_speed;

	PackToVision.pitch_angle = pitch_current;
	PackToVision.yaw_angle = yaw_current;

	PackToVision.color = (robotID > 100) ? 0 : 2;
	if (maxSpeed == 0)
		maxSpeed = 15; // 防止发maxSpeed为0过去
	PackToVision.bullet_speed = (maxSpeed == 255 ? 30 : maxSpeed);
	PackToVision.mode = pcVisionMode; // 默认值0为打击地面装甲模式

	/*判断是否有接收到数据*/
	if (lostDelay == 0)
	{
		PCvisionStatus = Unconnected;
	}
	else
	{
		lostDelay--;
		PCvisionStatus = Connected;
	}
	HAL_UART_Transmit_DMA(&huart1, (uint8_t *)&PackToVision, sizeof(PackToVision));
}
/**
 * @brief  接口，获取参数
 * @param  NULL
 * @return NULL
 */
void PCvision_Classdef::Status_Update(uint8_t _pcVisionMode,
									  float _pitch_current,
									  float _yaw_current,
									  float _pitch_angular_speed,
									  float _yaw_angular_speed,
									  uint8_t *_maxSpeed,
									  uint8_t *_robotID
									  )
{
	pcVisionMode = _pcVisionMode;
	pitch_current = _pitch_current; // pitch下为正方向
	yaw_current = _yaw_current;		// yaw右为正方向
	maxSpeed = *_maxSpeed;
	/*防止接收到错误数据*/
	switch (maxSpeed)
	{
	case 15:
		maxSpeed = 15;
		break;
	case 18:
		maxSpeed = 18;
		break;
	case 30:
		maxSpeed = 30;
		break;
	default:
		maxSpeed = 15;
		break;
	}
	robotID = *_robotID;
	pitch_angular_speed = _pitch_angular_speed;
	yaw_angular_speed = _yaw_angular_speed;
	/*存放pitch、yaw数组*/
	PitchPerviousData.InputData(pitch_current);
	YawPerviousData.InputData(yaw_current);
	PitchAngularSpeed_PerviousData.InputData(pitch_angular_speed);
	YawAngularSpeed_PerviousData.InputData(yaw_angular_speed);
}
/************************ COPYRIGHT(C) SCUT-ROBOTLAB**************************/
