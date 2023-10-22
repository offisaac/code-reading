#ifndef _BALANCE_CONTROLLER_H_
#define _BALANCE_CONTROLLER_H_

#include "SRML.h"
#include "math.h"
#include "arm_math.h"
#include "FuzzyPD_Ctrl.h"

extern float debug_out_A;
extern float debug_out_B;
extern float debug_out_C;
extern float debug_out_D;
extern float debug_out_E;
extern float debug_out_F;
extern float debug_out_G;
extern float debug_out_H;
extern float debug_out_I;

const float ratio_degree2rad = PI / 180.f;

//指定枚举类型作用域并且内部变量类型为uint8_t
enum Ctrl_Type
{
	PID, // pid控制器
	LQR	 // lqr控制器
};

typedef struct _Angular
{
	float yaw;	 //绕z轴(云台坐标系)
	float pitch; //绕x轴,朝y轴
	float roll;	 //绕y轴,朝x轴
} Angular;

typedef struct _Linear
{
	float z;
	float y;
	float x;
} Linear;

typedef struct _Controller_Out
{
	//输出变量
	float set_point_out;		 //定点环输出
	float distance_out;			 //距离环输出
	float stand_out;				 //直立环输出
	float turn_out;					 //转向环输出
	float speed_out;				 //速度环输出
	float feedforward_out;	 //前馈环输出
	float slider_out;				 //滑块环输出
	float sliderCtrl_out[2]; //滑块力矩输出
} Controller_Out;

class Ctrl_Base
{
public:
	Ctrl_Base(){};
	void Update_Target_Location(float z = 0, float y = 0, float x = 0);
	void Update_Current_Location(float z = 0, float y = 0, float x = 0);
	void Update_Target_Pos(float yaw = 0, float pitch = 0, float roll = 0);
	void Update_Current_Pos(float yaw = 0, float pitch = 0, float roll = 0);
	void Update_Target_LinearSpeed(float z = 0, float y = 0, float x = 0);
	void Update_Current_LinearSpeed(float z = 0, float y = 0, float x = 0);
	void Update_Target_AngularSpeed(float yaw = 0, float pitch = 0, float roll = 0);
	void Update_Current_AngularSpeed(float yaw = 0, float pitch = 0, float roll = 0);
	void Update_Target_LinearAcc(float z = 0, float y = 0, float x = 0);
	void Update_Current_LinearAcc(float z = 0, float y = 0, float x = 0);
	void Update_Target_AngularAcc(float yaw = 0, float pitch = 0, float roll = 0);
	void Update_Current_AngularAcc(float yaw = 0, float pitch = 0, float roll = 0);
	void Update_Flags(uint8_t _is_turn90degrees, uint8_t _is_rotation, uint8_t _is_reset, uint8_t _is_leap, uint8_t _is_unlimited, uint8_t _is_slope);

	float Value_Step(const float value, const float last_value, const float step, const float max, const float min); //防止数据跳变

	virtual void Controller_Adjust() = 0;

	const Controller_Out &Get_Data() { return output; } //读取输出数据

public:
	Linear target_location;
	Linear current_location;
	Angular target_pos;
	Angular current_pos;
	Linear target_linearSpeed;
	Linear current_linearSpeed;
	Angular target_angularSpeed;
	Angular current_angularSpeed;
	Linear target_linearAcc;
	Linear current_linearAcc;
	Angular target_angularAcc;
	Angular current_angularAcc;
	Linear target_sliderLocation;
	Linear current_sliderLocation[2];
	Linear target_sliderSpeed;
	Linear current_sliderSpeed[2];

	Linear last_target_location;
	Linear last_current_location;
	Angular last_target_pos;
	Angular last_current_pos;
	Linear last_target_linearSpeed;
	Linear last_current_linearSpeed;
	Angular last_target_angularSpeed;
	Angular last_current_angularSpeed;
	Linear last_target_linearAcc;
	Linear last_current_linearAcc;
	Angular last_target_angularAcc;
	Angular last_current_angularAcc;

	Controller_Out output; //控制器输出

	uint8_t is_turn90degrees = 0;
	uint8_t is_rotation = 0;
	uint8_t is_reset = 0;
	uint8_t is_leap = 0;
	uint8_t is_unlimited = 0;
	uint8_t is_slope = 0;
};

typedef struct _chassis_Params
{
	bool is_torqueOptimize;
	float wheel_max_speed; // 轮子所能达到的最大轮速，用于计算目标
	float motor_max_output;

	float max_linear_target;

	/* Time interval for speed resolve. */
	float task_run_interval;

	/* Acceleration for different stages(Unit: rpm/s) */
	float max_launch_acceleration; //起步速度
	float max_normal_acceleration; //正常加速度
	float max_brake_acceleration;	 //刹车速度
	float launch_speed;						 //起步速度范围

} chassis_Params;

template <Ctrl_Type Infantry_Ctrl_Type>
class Controller : public Ctrl_Base
{
public:
private:
};

//特化为pid控制器
template <>
class Controller<PID> : public Ctrl_Base
{
public:
	Controller(); //构造函数
	// void Pid_Parament_Config();		//参数整定
	virtual void Controller_Adjust(); //控制器结算

	void Set_Stand_Point(float point = 0) //设置静态定点
	{
		set_point = point;
	}

private:
	void torque_optimization(); //力矩优化
	float self_adaption();			//重心自适应
	float stand_adjust();				//直立环
	float turn_adjust();				//转向环
	float speed_adjust();				//速度环
	float stand_feedforward();	//抵消重力矩前馈

	void reset_adjust(); //控制器重置

	float set_point;		 //车身定点目标值
	myPID set_point_pid; //关乎参数调整
	myPID stand_pid;		 //直立环
	myPID turn_pid;			 //转向环
	myPID speed_pid;		 //速度环
	FuzzyPID_Classdef fuzzy_stand_pd;
	FuzzyPID_Classdef fuzzy_speed_pi;

	uint8_t speed_pid_cnt;				//速度环执行周期
	uint8_t speed_pid_delay = 20; //速度环执行周期

	chassis_Params params; //底盘优化参数

	// debug调参
	float debug_kp = 400.0f;
	float debug_kd = 100.0f;
	float feedforward_ratio = 130 * 0.1f / 2 * 2.8f / 20 * 16384;

	/*模糊规则表*/
	//直立环kp规则表
	int8_t stand_kp_rule[7][7] = {
			{PB, PS, PS, ZE, ZE, PS, PB},
			{PB, PB, PS, ZE, ZE, PS, PB},
			{PB, PS, PS, PS, ZE, PS, PB},
			{PS, PS, ZE, PS, ZE, PS, PS},
			{PB, PS, ZE, PS, PS, PS, PB},
			{PB, PS, ZE, ZE, PS, PB, PB},
			{PB, PS, ZE, ZE, PS, PS, PB}};
	//直立环kd规则表
	int8_t stand_kd_rule[7][7] = {
			{ZE, PS, PB, PB, PB, PS, ZE},
			{ZE, PS, PB, PS, PB, PS, ZE},
			{ZE, PS, PB, PS, PB, PS, ZE},
			{ZE, PS, PB, ZE, PB, PS, ZE},
			{ZE, PS, PB, PS, PB, PS, ZE},
			{ZE, PS, PB, PS, PB, PS, ZE},
			{ZE, PS, PB, PB, PB, PS, ZE}};
	//速度环kp规则表，ki先用固定参数
	int8_t speed_kp_rule[7][7] = {
			{PS, PS, PB, ZE, PB, PS, PS},
			{PS, PS, PB, ZE, PB, PS, PS},
			{PS, PS, PB, ZE, PB, PS, PS},
			{PS, PS, PB, ZE, PB, PS, PS},
			{PS, PS, PB, ZE, PB, PS, PS},
			{PS, PS, PB, ZE, PB, PS, PS},
			{PS, PS, PB, ZE, PB, PS, PS}};
};

//特化为lqr控制器
template <>
class Controller<LQR> : public Ctrl_Base
{
public:
	Controller(); //构造函数
	// void Pid_Parament_Config();		//参数整定
	virtual void Controller_Adjust(); //控制器结算

	void Set_Stand_Point(float point = 0) //设置静态定点
	{
		set_point = point;
	}

	bool sport_flag = false;
	bool break_flag = false;
	bool distance_flag = false;
	bool distance_enable = false;
	bool setpoint_adaption_flag = false;
	bool last_sport_flag = false;
	bool down_slope_flag = false;

	bool weightless_flag = false;	 //失重检测标志位
	int16_t weightless_delay = 80; //失重检测延时
	int16_t real_count = 0;

	bool idling_flag = false;	 //空转检测
	float motor_current = 0;	 //电机电流（用于空转检测）
	int16_t idling_count = 10; //空转计数器

	// float slider_pos[2] = {0, 0};

public:
	void slider_control();		 //滑块控制（纯滑块控制）
	float self_adaption();		 //重心自适应
	float distance_adjust();	 //距离环
	float stand_adjust();			 //直立环
	float turn_adjust();			 //转向环
	float speed_adjust();			 //速度环
	float slider_adjust();		 //滑块环（机体控制）
	float stand_feedforward(); //抵消重力矩前馈

	void weightless_check();		 //腾空检测
	void idling_check();				 //电机空转检测
	void rotation_crash_check(); //小陀螺撞墙检测

	void reset_adjust(); //控制器重置

	float set_point; //车身定点目标值

	myPID set_point_pid;			//关乎参数调整
	myPID rotation_point_pid; //小陀螺自适应pid
	myPID slider_follow_pid;	//滑块并行控制pid

	uint8_t speed_pid_cnt = 0;		//速度环执行周期
	uint8_t speed_pid_delay = 20; //速度环执行周期

	/*lqr参数*/
	float body_distance_kp = -2.f;
	float slider_distance_kp = 1.f;
	
float body_speed_kp = -2.680752e+00;
float body_pitch_kp = -1.327851e+01;
float body_pitchSpeed_kp = -2.271329e+00;
float body_sposition_kp = -1.032577e+01;
float body_sspeed_kp = -9.463333e-01;

float slider_speed_kp = 5.600548e-01;
float slider_pitch_kp = -4.549142e-01;
float slider_pitchSpeed_kp = -3.392322e-01;
float slider_sposition_kp = 1.419961e+01;
float slider_sspeed_kp = 1.623517e+00;

	float body_yaw_kp = 0.f;
	float body_yawSpeed_kp = 2.f;

	float slider_bias = 0;

	float feedforward_ratio = 0 * 14.48527f * 9.8f * 0.126807f; // 0.5*mg*l
	float distance_max = 1.5f;
	int distance_count = 50; //目标距离设置计数
	int distance_delay = 10; //距离环使能阈值，可用于距离环延迟执行（减小减速停车倒回问题）

	float torque_scale = 1.f; //力矩系数，用于抵消模型与实际
	float turn_scale = 1.f;

	float balance_point = 0.0f;
	float rotation_point = -2.f;

	/*小陀螺撞墙检测*/
	float last_speed_z = 0.f;
	bool is_rotation_crash = false;
};

extern myPID Cap_Charge_Pid;
extern myPID Power_Limit_Pid;
extern float Cap_Charge_Controller(const float current, const float target);
extern float Power_Limit_Controller(const float current, const float target);

#endif
