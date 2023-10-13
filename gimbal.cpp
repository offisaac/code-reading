/**
 ******************************************************************************
 * Copyright (c) 2019 - ~, SCUT-RobotLab Development Team
 * @file gimbal.cpp
 * @author mzh
 * @brief 云台代码
 * @date 2022-02-11
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
#include "gimbal.h"
#include "math.h"
#include "DiffCalculator.h"
/* Private define ------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
/* Private type --------------------------------------------------------------*/
/* Private function declarations ---------------------------------------------*/
/* function prototypes -------------------------------------------------------*/
/**
 * @brief 函数接口，获取状态标志位，及遥感量
 * @param None
 * @retval None
 */
void Gimbal_Classdef::Status_Update(float *_pitchData, float *_yawData, bool *_resetState)
{
	gimbal_resetState = *_resetState;
	pitch_target += *_pitchData;
	yaw_target += *_yawData;
}
/**
 * @brief 函数接口，获取陀螺仪数据
 * @param None
 * @retval None
 */
void Gimbal_Classdef::MPUdata_Update(mpu_rec_s *p)
{
	angular_velocity_yaw = p->gyro[2];
	angular_velocity_pitch = p->gyro[1];
	current_yaw = p->yaw;
	current_pitch = p->roll;
}
/**
 * @brief 函数接口，处理云台数据
 * @param None
 * @retval None
 */
void Gimbal_Classdef::Adjust()
{
	yaw_calculate();
	if (gimbal_resetState)
	{
		gimbal_reset();
	}
	else
	{
		gimbal_pid_calculate();
	}
}
/**
 * @brief 打包函数
 * @param None
 * @retval None
 */
void Gimbal_Classdef::Pack_CAN(int8_t _CAN_numb, Motor_CAN_COB *_CANX_pack)
{
	switch (_CAN_numb)
	{
	case 1:
		(*_CANX_pack) = MotorMsgPack<Motor_GM6020>(*_CANX_pack, pitchMotor);
		break;
	case 2:
		(*_CANX_pack) = MotorMsgPack<Motor_GM6020>(*_CANX_pack, yawMotor);
		break;
	default:
		break;
	}
}
/**
 * @brief 获取当前值
 * @param None
 * @retval Pitch,Yaw当前值
 */
float Gimbal_Classdef::Get_PitchTarget()
{
	return pitch_target;
}

float Gimbal_Classdef::Get_PitchCurrent()
{
	return current_pitch;
}
float Gimbal_Classdef::Get_YawTotal()
{
	return total_yaw;
}
float Gimbal_Classdef::Get_YawTarget()
{
	return yaw_target;
}
float Gimbal_Classdef::Get_YawMotorAngle()
{
	return yawMotor.getAngle();
}
float Gimbal_Classdef::Get_Angular_Velocity_Pitch()
{
	return angular_velocity_pitch;
}
float Gimbal_Classdef::Get_Angular_Velocity_Yaw()
{
	return angular_velocity_yaw;
}
/**
 * @brief 设置目标值
 * @param None
 * @retval None
 */
void Gimbal_Classdef::Set_PitchTarget(float _pitch_target)
{
	pitch_target = _pitch_target;
}
void Gimbal_Classdef::Set_YawTarget(float _yaw_target)
{
	yaw_target = _yaw_target;
}
void Gimbal_Classdef::Set_FeedbackSource(E_PitchYaw_FeedbackBy _feedback_by)
{
	if (_feedback_by != feedback_by)
	{
		feedback_by = _feedback_by;
	}
}
void Gimbal_Classdef::Reset_YawUpdateFlag()
{
	yawUpdate_is_init = false;
}
/**
 * @brief pid参数初始化
 * @param None
 * @retval None
 */
void Gimbal_Classdef::PidParaInit(E_PitchYawPidType _type, float _kp, float _ki, float _kd, float _ki_max, float _pi_max, float _out_max, float _I_SeparThresh)
{
	pid_para[_type].kp = _kp;
	pid_para[_type].ki = _ki;
	pid_para[_type].kd = _kd;
	pid_para[_type].I_Term_Max = _ki_max;
	pid_para[_type].PI_Term_Max = _pi_max;
	pid_para[_type].Out_Max = _out_max;
	pid_para[_type].I_SeparThresh = _I_SeparThresh;
}

/**
 * @brief pid参数初始化
 * @param None
 * @retval None
 */
void Gimbal_Classdef::PidInit(S_PidPara *_para, float _kp, float _ki, float _kd, float _ki_max, float _pi_max, float _out_max, float _I_SeparThresh)
{
	_para->kp = _kp;
	_para->ki = _ki;
	_para->kd = _kd;
	_para->I_Term_Max = _ki_max;
	_para->PI_Term_Max = _pi_max;
	_para->Out_Max = _out_max;
	_para->I_SeparThresh = _I_SeparThresh;
}
/**
 * @brief pid对象加载pid参数
 * @param None
 * @retval None
 */
void Gimbal_Classdef::LoadPidPara(myPID *_pidloop, S_PidPara _pid_para)
{
	_pidloop->SetPIDParam(_pid_para.kp, _pid_para.ki, _pid_para.kd, _pid_para.I_Term_Max, _pid_para.PI_Term_Max, _pid_para.Out_Max);
	_pidloop->I_SeparThresh = _pid_para.I_SeparThresh;
}
/**
 * @brief 切换到视觉pid参数
 * @param None
 * @retval None
 */
void Gimbal_Classdef::Switch2VisionPid(uint8_t _vision_pid_mode)
{
	switch (_vision_pid_mode)
	{
	case 1:
	{
		/*载入地面目标pid参数*/
		LoadPidPara(&pitch_speedloop, pid_para[PITCH_SPEED_1]);
		LoadPidPara(&pitch_angleloop, pid_para[PITCH_ANGLE_1]);
		LoadPidPara(&yaw_speedloop, pid_para[YAW_SPEED_1]);
		LoadPidPara(&yaw_angleloop, pid_para[YAW_ANGLE_1]);
		break;
	}
	case 2:
	{
		/*载入神符pid参数*/
		LoadPidPara(&pitch_speedloop, pid_para[PITCH_SPEED_2]);
		LoadPidPara(&pitch_angleloop, pid_para[PITCH_ANGLE_2]);
		LoadPidPara(&yaw_speedloop, pid_para[YAW_SPEED_2]);
		LoadPidPara(&yaw_angleloop, pid_para[YAW_ANGLE_2]);
		break;
	}
	case 3:
	{
		/*载入哨兵pid参数*/
		LoadPidPara(&pitch_speedloop, pid_para[PITCH_SPEED_3]);
		LoadPidPara(&pitch_angleloop, pid_para[PITCH_ANGLE_3]);
		LoadPidPara(&yaw_speedloop, pid_para[YAW_SPEED_3]);
		LoadPidPara(&yaw_angleloop, pid_para[YAW_ANGLE_3]);
		break;
	}
	}
	/*清除I项积分*/
	//	 pitch_speedloop.I_Term = pitch_angleloop.I_Term = yaw_speedloop.I_Term = yaw_angleloop.I_Term = 0;
}
/**
 * @brief pid计算
 * @param None
 * @retval None
 */
float yaw_out = 0;
float sin_w = 3.1415926f;
float ta_o = 0;		 //三角波幅值
uint16_t ta_f = 1; //频率
int16_t ta_count = 0;
float t = 0;
uint8_t debug_switch = 0;

float feedforward_scale = 1.f;

//粘滞摩擦系数辨识
float safe_angle = 24.f; //安全区域，也就是openlog记录的范围
float stop_angle = 26.f; //停车位置，云台停转

float debug_ff_t = 0;
void Gimbal_Classdef::gimbal_pid_calculate()
{
	t = Get_SystemTimer() * 0.000001f;
	/*更新当前值，并考虑切换不同的反馈通路*/
	if (feedback_by == ENCODER)
	{
		pitch_angleloop.Current = -pitchMotor.getAngle();
		pitch_controller.update_current_state(-pitchMotor.getAngle(), -angular_velocity_pitch, pitchMotor.givenCurrent, yawMotor.getSpeed());
		/*以最小角度跟随*/
		yaw_controller.update_current_state(int(yawMotor.getAngle()) % 360, angular_velocity_yaw, yawMotor.givenCurrent, yawMotor.getSpeed());
		yaw_angleloop.Current = int(yawMotor.getAngle()) % 360;
		if (yaw_controller.angleLoop.Current > 180)
		{
			yaw_controller.angleLoop.Current -= 360;
		}
		else if (yaw_controller.angleLoop.Current < -180)
		{
			yaw_controller.angleLoop.Current += 360;
		}

		if (yaw_angleloop.Current > 180)
		{
			yaw_angleloop.Current -= 360;
		}
		else if (yaw_angleloop.Current < -180)
		{
			yaw_angleloop.Current += 360;
		}
	}
	else
	{
		pitch_angleloop.Current = current_pitch;
		pitch_controller.update_current_state(current_pitch, -angular_velocity_pitch, pitchMotor.givenCurrent, yawMotor.getSpeed());
		yaw_controller.update_current_state(total_yaw, angular_velocity_yaw, yawMotor.givenCurrent, yawMotor.getSpeed());
		yaw_angleloop.Current = total_yaw;
	}
	pitch_speedloop.Current = angular_velocity_pitch;

	/*更新目标值*/
	pitch_target = std_lib::constrain(pitch_target, -20.0f, 30.0f);
	pitch_angleloop.Target = pitch_target;
	pitch_controller.update_target_angle(pitch_target);
	yaw_angleloop.Target = yaw_target;
	yaw_controller.update_target_angle(yaw_target);

	if (PITCH_PARAM_STRUCTURE == 0)
	{
		pitchMotor.Out = pitch_angleloop.Adjust_importDiff(angular_velocity_pitch) + 116.7360f * current_pitch - 763.2905f; // old
	}
	else if (PITCH_PARAM_STRUCTURE == 1)
	{
		static bool angle_feedback_flag_p = 1;
		static uint8_t cnt_p = 0;
		if (last_angle_feedback_pitch == angle_feedback_pitch)
		{
			if (cnt_p < 15)
				cnt_p++;
			else
				angle_feedback_pitch = 0;
		}
		else
		{
			cnt_p = 0;
		}
		last_angle_feedback_pitch = angle_feedback_pitch;
		if (fabsf(pitch_angleloop.Error) > 6)
			angle_feedback_flag_p = 0;
		else if (fabsf(pitch_angleloop.Error) < 3)
			angle_feedback_flag_p = 1;

		pitch_angleloop.Target = pitchAngleLPF.f(pitch_angleloop.Target);
		pitch_currentloop.SetPIDParam(0.25, 50, 0, 3000, 30000, 30000);
		pitch_currentloop.I_SeparThresh = 120000;
		if(pitchMotor.getSpeed() == 0)
		{
			pitch_currentloop.Target = pitch_angleloop.Adjust_importDiff(angular_velocity_pitch) + 
															feedforward_scale * pitch_f_k * (pitch_angle_Diff[0].calc(-pitch_angleloop.Target) + 
															pitchMotor.getSpeed() * 6 - (-angular_velocity_pitch)) + pitch_g_k * pitch_angleloop.Current + pitch_g_c;
		}
		else
		{
			pitch_currentloop.Target = pitch_angleloop.Adjust_importDiff(angular_velocity_pitch) + 
															//feedforward_scale * pitch_f_c * pitchMotor.getSpeed()/fabsf(pitchMotor.getSpeed()) +
															feedforward_scale * pitch_f_k * (pitch_angle_Diff[0].calc(-pitch_angleloop.Target) + 
															pitchMotor.getSpeed() * 6 - (-angular_velocity_pitch)) + pitch_g_k * pitch_angleloop.Current + pitch_g_c;
		}
		pitch_currentloop.Target += angle_feedback_flag_p * angle_feedback_pitch * pitch_a_k; // 加入二阶微分前馈
		// debug_ff_t = angle_feedback_flag_p * angle_feedback_pitch * pitch_a_k;
		pitch_currentloop.Current = pitchMotor.givenCurrent;
		pitchMotor.Out = pitch_currentloop.Adjust() + (pitch_currentloop.Target + pitch_m_r * pitchMotor.getSpeed()) / (0.75f - pitch_m_l * pitchMotor.getSpeed());
	}
	else
	{
		pitchMotor.Out = pitch_controller.adjust();
	}

	if (YAW_PARAM_STRUCTURE == 0)
	{
		yawMotor.Out = yaw_angleloop.Adjust_importDiff(angular_velocity_yaw);
	}
	else if (YAW_PARAM_STRUCTURE == 1)
	{
		static bool angle_feedback_flag_y = 1;
		static uint8_t cnt_y = 0;
		if (last_angle_feedback_yaw == angle_feedback_yaw)
		{
			if (cnt_y < 15)
				cnt_y++;
			else
				angle_feedback_yaw = 0;
		}
		else
		{
			cnt_y = 0;
		}
		last_angle_feedback_yaw = angle_feedback_yaw;
		if (fabsf(yaw_angleloop.Error) > 6)
			angle_feedback_flag_y = 0;
		else if (fabsf(yaw_angleloop.Error) < 3)
			angle_feedback_flag_y = 1;

		yaw_angleloop.Target = yawAngleLPF.f(yaw_angleloop.Target);
		yaw_currentloop.SetPIDParam(0.25, 50, 0, 3000, 30000, 30000);
		yaw_currentloop.I_SeparThresh = 120000;
		yaw_currentloop.Target = yaw_angleloop.Adjust_importDiff(angular_velocity_yaw) + feedforward_scale * yaw_f_k * (yaw_angle_Diff[0].calc(yaw_angleloop.Target) + yawMotor.getSpeed() * 6 - angular_velocity_yaw);
		yaw_currentloop.Target += angle_feedback_flag_y * angle_feedback_yaw * yaw_a_k; // 加入二阶微分前馈
		// debug_ff_t = angle_feedback_flag_y * angle_feedback_yaw * yaw_a_k;
		yaw_currentloop.Current = yawMotor.givenCurrent;
		yawMotor.Out = yaw_currentloop.Adjust() + (yaw_currentloop.Target + yaw_m_r * yawMotor.getSpeed()) / (0.75f - yaw_m_l * yawMotor.getSpeed());
	}
	else
	{
		yawMotor.Out = yaw_controller.adjust();
	}

	/*计算输出值*/
	//	yaw_out = 200 + 50 * sinf(3 * sin_w * t) + 100 * sinf(sin_w * t);
	//	yaw_controller.SetSpeedloopParams(0,0,0,0,0);
	//	yaw_controller.update_current_state(total_yaw, angular_velocity_yaw, yawMotor.givenCurrent, yawMotor.getSpeed());
	//	if(debug_switch)
	//	{
	//		yaw_controller.speedLoop.Target = yaw_out;
	//	}
	//	else
	//	{
	//		yaw_controller.speedLoop.Target = 0;
	//	}

	// /*生成三角波*/
	// if(ta_o!=0)
	// {
	// 	ta_count++;
	// }
	// ta_count = std_lib::constrain(ta_count,(int16_t)0,(int16_t)(1000/ta_f));
	// if(ta_count>=(int16_t)(1000/ta_f))
	// {
	// 	ta_count = 0;
	// }
	// if(ta_count >= 0 && ta_count < (int16_t)(0.25*1000/ta_f))
	// {
	// 	yaw_out += ta_o/(float)(250/ta_f);
	// }
	// else if(ta_count >= (int16_t)(0.25*1000/ta_f) && ta_count < (int16_t)(0.75*1000/ta_f))
	// {
	// 	yaw_out -= ta_o/(float)(250/ta_f);
	// }
	// else
	// {
	// 	yaw_out += ta_o/(float)(250/ta_f);
	// }
	// if(ta_o == 0)
	// {
	// 	yaw_out = 0;
	// 	ta_count = 0;
	// }

	// static DiffCalculator<1> speed_target_Diff;
	// static SecondOrderButterworthLPF angle_t_lpf(5,1000);
	// static SecondOrderButterworthLPF angleOut_lpf(10,1000);
	/*生成正弦波*/
	// yaw_out = 200 + 50 * sinf(3 * sin_w * t) + 100 * sinf(sin_w * t);
	// yaw_out = 100 * sinf(2 * sin_w * t);//30 2	20 3
	//	if(debug_switch)
	//	{
	//		pitch_speedloop.Target = -yaw_out;
	//	}
	//	else
	//	{
	//		pitch_speedloop.Target = 0;
	//	}
	// pitch_angleloop.SetPIDParam(angle_kp,0,0,0,angle_omax,angle_omax);
	// pitch_angleloop.Target = angle_t_lpf.f(pitch_target);
	// //pitch_angleloop.Target = pitch_target;
	// pitch_angleloop.Current = current_pitch;
	// pitch_speedloop.Target = angleOut_lpf.f(pitch_angleloop.Adjust());
	// pitch_speedloop.Current = -angular_velocity_pitch;//陀螺仪与电机极性相反，参数都为正时，目标值和当前值为正输入取反
	// pitch_speedloop.SetPIDParam(speed_kp,speed_ki,0,speed_imax,speed_omax,speed_omax);
	// //pitch_speedloop.SetPIDParam(0,0,0,0,0,0);
	// if(pitchMotor.getSpeed() == 0)
	// {
	// 	pitch_currentloop.Target = pitch_speedloop.Adjust() +
	// 														116.2253f * current_pitch - 994.4664f + 231.1759f +	//重力矩补偿
	// 														f_k * (pitch_speedloop.Target + pitchMotor.getSpeed() * 6 - pitch_speedloop.Current) +	//动态粘滞摩擦补偿
	// 														a_k * (speed_target_Diff.calc(pitch_speedloop.Target));
	// }
	// else
	// {
	// 	pitch_currentloop.Target = pitch_speedloop.Adjust() +
	// 														116.2253f * current_pitch - 994.4664f + 231.1759f +	//重力矩补偿
	// 														f_c * pitchMotor.getSpeed()/abs(pitchMotor.getSpeed()) +	//库仑静摩擦补偿
	// 														f_k * (pitch_speedloop.Target + pitchMotor.getSpeed() * 6 - pitch_speedloop.Current) +	//动态粘滞摩擦补偿
	// 														a_k * (speed_target_Diff.calc(pitch_speedloop.Target));
	// }
	// pitch_currentloop.Current = pitchMotor.givenCurrent;
	// pitch_currentloop.SetPIDParam(0.25,50,0,3000,26000,26000);
	// pitchMotor.Out = pitch_currentloop.Adjust() + (pitch_currentloop.Target + m_r * pitchMotor.getSpeed()) / (0.75f - m_l * pitchMotor.getSpeed());
	// if(current_pitch > safe_angle)
	// {
	// 		debug_switch = 0;
	// 		pitch_angleloop.Target = stop_angle;
	// 		pitchMotor.Out = pitch_angleloop.Adjust_importDiff(angular_velocity_pitch) + 116.7360f * current_pitch + 668.5741f; // old
	// }
	// else
	// {
	// 		pitchMotor.Out = pitch_currentloop.Adjust() + (pitch_currentloop.Target + m_r * pitchMotor.getSpeed()) / (0.75f - m_l * pitchMotor.getSpeed());
	// }
}
/**
 * @brief yaw角度计算
 * @param None
 * @retval None
 */
void Gimbal_Classdef::yaw_calculate()
{
	static float last_yaw, yaw_offset;
	static int16_t yaw_cnt = 0;
	/* 若使用编码器赋值，则不使用陀螺仪计算pitch yaw角 */
	if ((feedback_by == ENCODER) && !(yawUpdate_is_init))
		return;
	/*云台复位之后交给陀螺仪控制获取offset*/
	if (yawUpdate_is_init)
	{
		if (current_yaw - last_yaw > 180)
			yaw_cnt--;
		else if (current_yaw - last_yaw < -180)
			yaw_cnt++;
	}
	else
	{
		yaw_cnt = 0;
		yaw_offset = current_yaw; //+(Missile_yaw_6020.getEncoder()-1925) / 8192.0f *360;
		yawUpdate_is_init = true;
	}
	last_yaw = current_yaw;
	total_yaw = yaw_cnt * 360.0f + current_yaw - yaw_offset;
}
/**
 * @brief 云台复位
 * @param None
 * @retval None
 */
void Gimbal_Classdef::gimbal_reset()
{
	/*目标值复位*/
	yaw_target = total_yaw;
	pitch_target = 0;
	/*输出置零*/
	pitchMotor.Out = yawMotor.Out = 0;
	pitch_speedloop.I_Term = pitch_angleloop.I_Term = yaw_speedloop.I_Term = yaw_angleloop.I_Term = 0;
}
/************************ COPYRIGHT(C) SCUT-ROBOTLAB**************************/
