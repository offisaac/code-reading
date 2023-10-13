/**
 ******************************************************************************
 * Copyright (c) 2019 - ~, SCUT-RobotLab Development Team
 * @file    gimbal.h
 * @author  mzh
 * @brief   云台代码头文件
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
#ifndef _GIMBAL_H_
#define _GIMBAL_H_

#ifdef __cplusplus
/* Includes ------------------------------------------------------------------*/
#include "SRML.h"
#include "srml_std_lib.h"
#include "math.h"
#include "DiffCalculator.h"
#include "SecondButterworthLPF.h"
/* Private macros ------------------------------------------------------------*/
#define ID_PITCH (2)
#define ID_YAW (4)

#define PITCH_PARAM_STRUCTURE 1 // PITCH轴参数结构：0为单环角度环	1为角度环+电流环	2为全补偿三环
#define YAW_PARAM_STRUCTURE 1		// YAW轴参数结构：0为单环角度环	1为角度环+电流环	2为全补偿三环
/* Private type --------------------------------------------------------------*/
/* Exported macros -----------------------------------------------------------*/
/* Exported types ------------------------------------------------------------*/
/*pitch、yaw反馈值类型*/
typedef enum E_PitchYaw_FeedbackBy
{
	ENCODER = 0U,
	GYRO,
} E_PitchYaw_FeedbackBy;
/*pitch、yaw pid各环,操作手模式下及自瞄下*/
enum E_PitchYawPidType
{
	PITCH_SPEED_1 = 0,
	PITCH_ANGLE_1 = 1,
	YAW_SPEED_1 = 2,
	YAW_ANGLE_1 = 3,
	YAW_CURRENT_1 = 4,
	PITCH_SPEED_2 = 5,
	PITCH_ANGLE_2 = 6,
	YAW_SPEED_2 = 7,
	YAW_ANGLE_2 = 8,
	YAW_CURRENT_2 = 9,
	PITCH_SPEED_3 = 10,
	PITCH_ANGLE_3 = 11,
	YAW_SPEED_3 = 12,
	YAW_ANGLE_3 = 13,
	YAW_CURRENT_3 = 14,
};
/*pid结构体*/
struct S_PidPara
{
	float kp;
	float ki;
	float kd;
	float I_Term_Max;
	float PI_Term_Max;
	float Out_Max;
	float I_SeparThresh;
};

class gimbal_motor_newController
{
public:
	DiffCalculator<1> angle_target_Diff;
	DiffCalculator<1> speed_target_Diff;
	SecondOrderButterworthLPF *angle_f_lpf = nullptr;
	SecondOrderButterworthLPF *angle_target_lpf = nullptr;
	SecondOrderButterworthLPF *speed_target_lpf = nullptr;
	float motor_speed = 0;
	float angle_feedback = 0;
	uint8_t angle_feedback_flag = 1;
	uint8_t cnt = 0;
	float last_angle_feedback = 0;

public:
	gimbal_motor_newController() {}

	myPID angleLoop;
	myPID speedLoop;
	myPID currentLoop;
	/** 控制器参数 **/
	//重力矩前馈
	float mg_k = 0; //斜率
	float mg_c = 0; //截距
	//电流环前馈
	float m_r = 55;			 //电机电阻对应线性化参数
	float m_l = 0.0004f; //电机电容对应线性化参数
	//速度环前馈
	float f_c = 536;			//补偿摩擦力
	float f_k = 1 / 3.72; //补偿摩擦力
	float a_k = 4.4;			//用于补偿转动惯量的加速力矩，调信号跟踪性能

	float Out = 0;

	void LoadAngleFLPF(SecondOrderButterworthLPF *_lpf)
	{
		angle_f_lpf = _lpf;
	}

	void LoadAngleLPF(SecondOrderButterworthLPF *_lpf)
	{
		angle_target_lpf = _lpf;
	}

	void LoadSpeedLPF(SecondOrderButterworthLPF *_lpf)
	{
		speed_target_lpf = _lpf;
	}

	void update_current_state(float current_angle, float current_angle_speed, float current_motor_current, float _motor_speed)
	{
		angleLoop.Current = current_angle;
		speedLoop.Current = current_angle_speed;
		currentLoop.Current = current_motor_current;
		motor_speed = _motor_speed;
	}

	void update_target_angle(float target_angle)
	{
		if (angle_target_lpf != nullptr)
		{
			angleLoop.Target = angle_target_lpf->f(target_angle);
		}
		else
		{
			angleLoop.Target = target_angle;
		}
	}

	void SetCurrentFeedForward(float _m_r, float _m_l)
	{
		m_r = _m_r;
		m_l = _m_l;
	}

	void SetSpeedFeedForward(float _f_c, float _f_k, float _a_k)
	{
		f_c = _f_c;
		f_k = _f_k;
		a_k = _a_k;
	}

	void SetMgFeedForward(float _mg_k, float _mg_c)
	{
		mg_k = _mg_k;
		mg_c = _mg_c;
	}

	void SetAngleloopParams(float kp, float omax)
	{
		angleLoop.SetPIDParam(kp, 0, 0, 0, omax, omax);
	}

	void SetSpeedloopParams(float kp, float ki, float imax, float omax, float ist)
	{
		speedLoop.SetPIDParam(kp, ki, 0, imax, omax, omax);
		speedLoop.I_SeparThresh = ist;
	}

	void SetCurrentloopParams(float kp, float ki, float imax, float omax, float ist)
	{
		currentLoop.SetPIDParam(kp, ki, 0, imax, omax, omax);
		currentLoop.I_SeparThresh = ist;
	}

	void AngleFeedForwardCalc(float _anglet)
	{
		angle_feedback = angle_target_Diff.calc(_anglet);
		if (angle_f_lpf != nullptr)
		{
			angle_feedback = angle_f_lpf->f(angle_feedback);
		}
	}

	float adjust()
	{
		if (last_angle_feedback == angle_feedback)
		{
			if (cnt < 15)
				cnt++;
			else
				angle_feedback = 0;
		}
		else
		{
			cnt = 0;
		}
		last_angle_feedback = angle_feedback;

		if (fabsf(angleLoop.Error) > 6)
			angle_feedback_flag = 0;
		else if (fabsf(angleLoop.Error) < 3)
			angle_feedback_flag = 1;

		if (speed_target_lpf != nullptr)
		{
			speedLoop.Target = speed_target_lpf->f(angleLoop.Adjust());
		}
		else
		{
			speedLoop.Target = angleLoop.Adjust();
		}
		//speedLoop.Target += angle_feedback * angle_feedback_flag * 0.5f;

		if (motor_speed == 0)
			currentLoop.Target = speedLoop.Adjust() +
													 f_k * (speedLoop.Target + motor_speed * 6 - speedLoop.Current) +
													 a_k * (speed_target_Diff.calc(speedLoop.Target)) +
													 mg_k * angleLoop.Current + mg_c;
		else
			currentLoop.Target = speedLoop.Adjust() +
													 f_c * motor_speed / fabsf(motor_speed) +
													 f_k * (speedLoop.Target + motor_speed * 6 - speedLoop.Current) +
													 a_k * (speed_target_Diff.calc(speedLoop.Target)) +
													 mg_k * angleLoop.Current + mg_c;
		Out = currentLoop.Adjust() + (currentLoop.Target + m_r * motor_speed) / (0.75f - m_l * motor_speed);
		return Out;
	}
};

class Gimbal_Classdef
{
public:
	Gimbal_Classdef(int16_t _yaw_offset /*底盘跟随目标角*/, int16_t _pitch_offset)
	{
		/*底盘跟随角度初始化*/
		yawMotor.setEncoderOffset(_yaw_offset);
		pitchMotor.setEncoderOffset(_pitch_offset);

		pitch_controller.LoadAngleFLPF(&pitchAngleFLPF);
		pitch_controller.LoadAngleLPF(&pitchAngleLPF);
		pitch_controller.LoadSpeedLPF(&pitchSpeedLPF);
		pitch_controller.SetCurrentFeedForward(pitch_m_r, pitch_m_l);
		pitch_controller.SetSpeedFeedForward(pitch_f_c, pitch_f_k, pitch_a_k); // 2.15
		// pitch_controller.SetSpeedFeedForward(0, 0, 0);//2.15
		pitch_controller.SetMgFeedForward(pitch_g_k, pitch_g_c);

		yaw_controller.LoadAngleFLPF(&yawAngleFLPF);
		yaw_controller.LoadAngleLPF(&yawAngleLPF);
		yaw_controller.LoadSpeedLPF(&yawSpeedLPF);
		yaw_controller.SetCurrentFeedForward(yaw_m_r, yaw_m_l);
		yaw_controller.SetSpeedFeedForward(yaw_f_c, yaw_f_k, yaw_a_k); // 5.5
																																	 // yaw_controller.SetSpeedFeedForward(0, 0, 0);//5.5
	}
	/*创建电机对象*/
	Motor_GM6020 pitchMotor = Motor_GM6020(ID_PITCH);
	Motor_GM6020 yawMotor = Motor_GM6020(ID_YAW);
	/*创建pid对象*/
	myPID pitch_currentloop;
	myPID pitch_speedloop;
	myPID pitch_angleloop;
	myPID yaw_speedloop;
	myPID yaw_angleloop;
	myPID yaw_currentloop; // 电流环

	gimbal_motor_newController yaw_controller;
	gimbal_motor_newController pitch_controller;

	/*控制滤波器*/
	SecondOrderButterworthLPF pitchAngleLPF = {SecondOrderButterworthLPF(20, 1000)};
	SecondOrderButterworthLPF pitchAngleFLPF = {SecondOrderButterworthLPF(20, 1000)};
	SecondOrderButterworthLPF pitchSpeedLPF = {SecondOrderButterworthLPF(20, 1000)};
	SecondOrderButterworthLPF pitchCurrentLPF = {SecondOrderButterworthLPF(20, 1000)};

	SecondOrderButterworthLPF yawAngleLPF = {SecondOrderButterworthLPF(20, 1000)};
	SecondOrderButterworthLPF yawAngleFLPF = {SecondOrderButterworthLPF(20, 1000)};
	SecondOrderButterworthLPF yawSpeedLPF = {SecondOrderButterworthLPF(20, 1000)};
	SecondOrderButterworthLPF yawCurrentLPF = {SecondOrderButterworthLPF(20, 1000)};

	DiffCalculator<1> pitch_angle_Diff[3];
	DiffCalculator<1> yaw_angle_Diff[3];

	float angle_feedback_pitch = 0;
	float last_angle_feedback_pitch = 0;
	float angle_feedback_yaw = 0;
	float last_angle_feedback_yaw = 0;

	void angle_ff_calc(float angle_t_pitch, float angle_t_yaw)
	{
		angle_feedback_pitch = pitch_angle_Diff[1].calc(pitch_angle_Diff[2].calc(angle_t_pitch));
		angle_feedback_pitch = pitchAngleFLPF.f(angle_feedback_pitch);

		angle_feedback_yaw = yaw_angle_Diff[1].calc(yaw_angle_Diff[2].calc(angle_t_yaw));
		angle_feedback_yaw = yawAngleFLPF.f(angle_feedback_yaw);
	}

	/*接口*/
	void Status_Update(float *_pitchData, float *_yawData, bool *_resetState);
	void MPUdata_Update(mpu_rec_s *p); // pitch yaw角度更新，包括初始角度偏置
	void Adjust();
	void Pack_CAN(int8_t _CAN_number, Motor_CAN_COB *_CANX_pack); // 打包函数
	/*获取当前值*/
	float Get_PitchTarget();						// pitch目标值
	float Get_PitchCurrent();						// pitch陀螺仪当前值
	float Get_YawTotal();								// 云台yaw当前值
	float Get_YawMotorAngle();					// yaw码盘电机值
	float Get_YawTarget();							// yaw目标值
	float Get_Angular_Velocity_Pitch(); // pitch角速度
	float Get_Angular_Velocity_Yaw();		// yaw角速度
	/*设置目标值*/
	void Set_PitchTarget(float _pitch_target);
	void Set_YawTarget(float _yaw_target);
	void Set_FeedbackSource(E_PitchYaw_FeedbackBy); // 设置反馈来源
	void Reset_YawUpdateFlag();											// 平衡步用，陀螺仪计算角度复位
	/*pid相关*/
	void PidParaInit(E_PitchYawPidType _type, float _kp, float _ki, float _kd, float _ki_max, float _pi_max, float _out_max, float _I_SeparThresh);
	void PidInit(S_PidPara *_para, float _kp, float _ki, float _kd, float _ki_max, float _pi_max, float _out_max, float _I_SeparThresh);
	// pid参数初始化
	void LoadPidPara(myPID *_pidloop, S_PidPara _pid_para); // 加载pid参数进pid对象
	void Switch2VisionPid(uint8_t _vision_pid_mode);				// 切换到视觉pid
	S_PidPara pid_para[12];																	// pid各环参数

	S_PidPara PitchHalfNormal[3], PitchHalfCar[3], PitchHalfRune[3]; //半补偿
	S_PidPara PitchFullNormal[3], PitchFullCar[3], PitchFullRune[3]; //全补偿
	S_PidPara YawHalfNormal[3], YawHalfCar[3], YawHalfRune[3];			 //半补偿
	S_PidPara YawFullNormal[3], YawFullCar[3], YawFullRune[3];			 //全补偿

	float pitch_target, yaw_target;

public:
	/*pitch、yaw相关变量*/
	bool yawUpdate_is_init;				// yaw复位标志位,平衡步倒地自救用
	bool gimbal_resetState;				// 云台复位标志位
	float pitch_scale, yaw_scale; // pitch、yaw灵敏度
	float total_yaw;

	E_PitchYaw_FeedbackBy feedback_by; // 控制pitch yaw的数据来源
	/*陀螺仪相关变量*/
	float angular_velocity_yaw, angular_velocity_pitch, current_pitch, current_yaw; // 陀螺仪传入

	/*pid计算函数*/
	void gimbal_pid_calculate();
	/*yaw角度计算函数*/
	void yaw_calculate();
	/*云台复位函数*/
	void gimbal_reset();

	float yaw_m_r = 55.f;
	float yaw_m_l = 0.0004f;
	float pitch_m_r = 53.f;
	float pitch_m_l = 0.0004f;

	float pitch_f_k = 4.8f;														 //粘滞摩擦力斜率补偿(4.6
	float pitch_f_c = 240;														 //静摩擦补偿(230
	float pitch_a_k = 2.15f;													 //转动惯量补偿
	float pitch_g_k = 116.7360f;											 //重力补偿斜率
	float pitch_g_c = -994.4664f + 231.1759f + 1000.f; //重力补偿截距

	float yaw_f_k = 0.28f;
	float yaw_f_c = 536;
	float yaw_a_k = 5.5f;
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
