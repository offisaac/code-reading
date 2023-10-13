/**
 ******************************************************************************
 * Copyright (c) 2019 - ~, SCUT-RobotLab Development Team
 * @file booster.cpp
 * @author mzh
 * @brief 小发射控制代码
 * @date 2022-02-10
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
#include "booster.h"
#include <Middlewares/UpperMonitor/UpperMonitor.h>
/* Exported macros -----------------------------------------------------------*/
extern TIM_HandleTypeDef htim2;	  // 弹舱盖用
extern UART_HandleTypeDef huart3; // 小发射测试用
uint32_t bullect_speed_t = 0;
uint32_t turn_plate_t = 0;
/* Private define ------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
/* Private type --------------------------------------------------------------*/
/* Private function declarations ---------------------------------------------*/
/* function prototypes -------------------------------------------------------*/
/**
 * @brief 函数接口，获取状态标志位
 * @param None
 * @retval None
 */
void Booster_Classdef::Status_Update(bool *_friState,
																		 bool *_laserState,
																		 bool *_turnplateState,
																		 bool *_bulletBayState,
																		 bool *_resetState,
																		 uint16_t *_bulletSpeed,
																		 uint16_t *_heatRf,
																		 uint16_t *_coolingRate,
																		 uint16_t *_heatLimit,
																		 uint8_t *_maxSpeed)
{
	/*传入裁判系统数据，复位标志位*/
	booster_resetState = *_resetState;
	bulletSpeed = *_bulletSpeed / 1000.0f;
	heatRf = *_heatRf;
	coolingRate = *_coolingRate;
	heatLimit = *_heatLimit;
	maxSpeed = *_maxSpeed;
	turnplate_frq = std_lib::constrain(turnplate_frq, 0.1f, 20.0f); // 限制射频
	if(continuous_flag < USHRT_MAX)
	{
		continuous_flag++;
	}

	if (!booster_resetState) // 掉线保护
	{
		bulletBay_ctrl(*_bulletBayState); // 弹舱盖
		if (fri_on(*_friState))						// 摩擦轮
		{
			bulletVelocity_adjust(); // 射速自适应 debug时注释掉
			bulletJam_protection();	 // 防卡弹

			/*视觉打符，接管拨盘*/
			if (auto_fire)
			{
				turnplate_targetAngle -= 1296;
				turn_plate_t = Get_SystemTimer();
			}
			else
			{
				if (heat_ctrl())										// 热量控制	heat_ctrl(),debug时改为1
					turnplate_ctrl(*_turnplateState); // 拨盘
				else
					turnplate_ctrl(false);
			}
		}
		else
		{
		}
		laser_on(*_laserState); // 激光
	}
	else
	{
		fri_on(false);
		laser_on(false);
		bulletBay_ctrl(false);
	}
	/*debug,小发射测试*/
	static float last_bulletspeed;
	if (bulletSpeed != last_bulletspeed)
	{
		//Sent_Contorl(&huart3);
		bullect_speed_t = Get_SystemTimer();
	}
	last_bulletspeed = bulletSpeed;
}
/**
 * @brief 函数接口，处理小发射数据
 * @param None
 * @retval None
 */
void Booster_Classdef::Adjust()
{
	booster_pid_calculate();
	if (booster_resetState)
	{
		booster_reset();
	}
}
/**
 * @brief 修改射频
 * @param None
 * @retval None
 */
void Booster_Classdef::Set_TurnplateFrq(float _turnplate_frq)
{
	turnplate_frq = _turnplate_frq;
}
/**
 * @brief 打包函数
 * @param None
 * @retval None
 */
void Booster_Classdef::Pack_CAN(Motor_CAN_COB *_CANX_pack)
{
	(*_CANX_pack) = MotorMsgPack<Motor_C620, Motor_C620, Motor_C610>(*_CANX_pack, left_friWheel, right_friWheel, turnplateMotor);
}
/**
 * @brief 摩擦轮状态判断与处理
 * @param None
 * @retval None
 */
bool Booster_Classdef::fri_on(bool _fri_state)
{
	if (_fri_state)
	{
		friction_wheel_rpm = adaptive_fri_wheel_rpm;
		return true;
	}
	else
	{
		friction_wheel_rpm = 0;
		return false;
	}
}
/**
 * @brief 激光开关
 * @param None
 * @retval None
 */
void Booster_Classdef::laser_on(bool _laser_state)
{
	if (_laser_state)
	{
		HAL_GPIO_WritePin(GPIOC, GPIO_PIN_15, GPIO_PIN_SET);
	}
	else
	{
		HAL_GPIO_WritePin(GPIOC, GPIO_PIN_15, GPIO_PIN_RESET);
	}
}
/**
 * @brief 弹舱开关
 * @param None
 * @retval None
 */
void Booster_Classdef::bulletBay_ctrl(bool _bulletBay_state)
{
	static int16_t bulletBay_delay = 0;
	if (bulletBay_delay == 0)
	{
		if (_bulletBay_state)
		{
			__HAL_TIM_SetCompare(&htim2, TIM_CHANNEL_2, bulletBay_on);
		}
		else
		{
			__HAL_TIM_SetCompare(&htim2, TIM_CHANNEL_2, bulletBay_off);
		}
	}
	// 降低舵机控制频率
	bulletBay_delay++;
	bulletBay_delay %= 500;
}
/**
 * @brief 拨盘控制
 * @param None
 * @retval None
 */
void Booster_Classdef::turnplate_ctrl(bool _turnplate_state)
{
	if (continuous_flag >= (int)(1000 / turnplate_frq) && _turnplate_state)
	{
		turnplate_targetAngle -= 1296;
		continuous_flag = 0;
		turn_plate_t = Get_SystemTimer();
	}
}
/**
 * @brief 防卡弹
 * @param None
 * @retval None
 */
void Booster_Classdef::bulletJam_protection()
{
	if ((turnplate_angleloop.Current - turnplate_targetAngle) > 360 * 3.6 * 1.9)
	{
		turnplate_targetAngle = turnplate_angleloop.Current + 1296; // 方案一：拨盘回拨一格
																	//		Turnplate_targetAngle = TurnplateAngleloop.Current; //方案二：拨盘电流为零
	}
}
/**
 * @brief 热量控制
 * @param None
 * @retval None
 */
bool Booster_Classdef::heat_ctrl()
{
	/*一发弹丸增加10热量*/
	if (bulletSpeed != lastBulletSpeed)
	{
		heat += 10;
		bulletcounter++; // 小弹丸计数
	}
	lastBulletSpeed = bulletSpeed;
	/*裁判系统修正*/
	if (heatRf - heat > 9)
	{
		heat += 10;
	}
	/*热量冷却*/
	heat -= coolingRate / 1000.0;
	if (heat < 0)
	{
		heat = 0;
	}
	/*检测热量是否允许发射*/
	if (heat < heatLimit - heat_offset || heatLimit == -1)
	{
		return true;
	}
	else
	{
		return false;
	}
}
/**
 * @brief 摩擦轮转速自适应
 * @param None
 * @retval None
 */
uint16_t debug_15rpm = BULLET_SPEED_15;
uint16_t debug_18rpm = BULLET_SPEED_18;
uint16_t debug_30rpm = BULLET_SPEED_30;
void Booster_Classdef::bulletVelocity_adjust()
{
	static uint8_t shoot_cnt = 0;
	static int16_t lastMaxSpeed;
	float bulletSpeed_target = 0;
	float reference_friRPM /*参考转速*/;
	switch (maxSpeed)
	{
	case 15:
		reference_friRPM = debug_15rpm;
		bulletSpeed_target = maxSpeed - 1.4f;
		break;
	case 18:
		reference_friRPM = debug_18rpm;
		bulletSpeed_target = maxSpeed - 1.8f;
		break;
	case 30:
		reference_friRPM = debug_30rpm;
		bulletSpeed_target = maxSpeed - 5.f; // 30射速小发射方差较大，要降低平均射速
		break;
	default:
		reference_friRPM = debug_15rpm;
		bulletSpeed_target = maxSpeed - 1.4f;
		break;
	}

	if (lastMaxSpeed != maxSpeed) /*更新最大转速时重置累积值*/
	{
		friRPM_accumulation = 0;
		//清空滤波器
		shoot_cnt = 0;
		for (int i = 0;i<SHOOT_FILTER_STEP;i++)
		{
			bulletSpeed_meanfilter << 0;
		}
	}
	lastMaxSpeed = maxSpeed;

	if (bulletSpeed != lastBulletSpeed) /*自适应转速,发一发子弹更新一次*/
	{
		bulletSpeed_filter_out = bulletSpeed_meanfilter.f(bulletSpeed);
		if (bulletSpeed > maxSpeed && maxSpeed!=-1)
		{
			friRPM_accumulation -= over_bulletSpeed_punish;//超射速直接给予惩罚
			if (shoot_cnt < SHOOT_FILTER_STEP)
			{
				shoot_cnt++;
			}
		}
		else
		{
			if (shoot_cnt < SHOOT_FILTER_STEP)//滤波器未满时不进行修正
			{
				shoot_cnt++;
			}
			else
			{
				friRPM_accumulation += bulletSpeed_adapt_gain * (bulletSpeed_target - bulletSpeed_filter_out);
			}
		}
	}
	adaptive_fri_wheel_rpm = reference_friRPM + friRPM_accumulation;
}
/**
 * @brief pid计算
 * @param None
 * @retval None
 */
void Booster_Classdef::booster_pid_calculate()
{
	/*更新当前值*/
	left_fri_speedloop.Current = left_friWheel.getSpeed();
	right_fri_speedloop.Current = right_friWheel.getSpeed();
	turnplate_angleloop.Current = turnplateMotor.getAngle();
	turnplate_speedloop.Current = turnplateMotor.getSpeed();

	/*更新目标值*/
	friction_wheel_rpm = std_lib::constrain(friction_wheel_rpm, (int16_t)0, (int16_t)9000); /*摩擦轮转速限幅*/
	left_fri_speedloop.Target = friction_wheel_rpm;
	right_fri_speedloop.Target = -friction_wheel_rpm;
	turnplate_angleloop.Target = turnplate_targetAngle;
	turnplate_speedloop.Target = turnplate_angleloop.Adjust();

	/*计算输出值*/
	left_friWheel.Out = left_fri_speedloop.Adjust();
	right_friWheel.Out = right_fri_speedloop.Adjust();
	turnplateMotor.Out = turnplate_speedloop.Adjust();
}
/**
 * @brief 小发射复位
 * @param None
 * @retval None
 */
void Booster_Classdef::booster_reset()
{
	/*目标值复位*/
	friction_wheel_rpm = 0;
	turnplate_targetAngle = turnplate_angleloop.Current;
	/*输出置零*/
	turnplateMotor.Out = 0;
	/*关闭摩擦轮*/
	fri_on(false);
	laser_on(false);
}
/************************ COPYRIGHT(C) SCUT-ROBOTLAB**************************/
