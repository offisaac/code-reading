/**
 ******************************************************************************
 * Copyright (c) 2023 - ~, SCUT-RobotLab Development Team
 * @file PIDmethod.h
 * @author 张至睿 (2231625449@qq.com)
 * @brief header file for PIDmethod.cpp
 * @version 1.0
 * @date 2023-03-01
 *
 ******************************************************************************
 * @attention
 *
 * if you had modified this file, please make sure your code does not have many
 * bugs, update the version Number, write dowm your name and the date, the most
 * important is make sure the users will have clear and definite understanding
 * through your new brief.
 *
 * <h2><center>&copy; Copyright (c) 2023 - ~, SCUT-RobotLab Development Team.
 * All rights reserved.</center></h2>
 ******************************************************************************
 */
#ifndef _PIDMETHOD_H_
#define _PIDMETHOD_H_

#include <stdint.h>
#include <cstdint>
#include <stddef.h>
#include <math.h>
#include <limits>//定义各种变量的储存最大值
#include "Upper_Public.h"
using namespace std;


typedef uint32_t(*SystemTick_Fun)(void);//函数指针，用于接入不同单片机获取时间栈的函数

typedef enum
{
    Common,
    Fit
}Params_Mode;

typedef enum class _PID_Mode
{
    IS_PI,
    IS_PD
}PID_Mode;

class PIDtimer
{
public:
    static uint8_t getMicroTick_regist(uint32_t(*getTick_fun)(void));//获取当前时间函数接口
    static SystemTick_Fun Get_SystemTick;   //获取时间的函数
    float dt = 0;				                //时间微分
    uint32_t last_time; 	                //记录上次时间
    uint8_t UpdataTimeStamp(void);          //时间栈更新

};

#ifdef __cplusplus
extern "C"
{
#endif
/*PID方法类*/
class PIDmethod : public PIDtimer
{
public:
    PIDmethod(){}
    PIDmethod(Params_Mode mode, float _timeStep = 0);//必须输入参数模式，时间栈选择输入
    void PID_Init(Params_Mode mode, float _timeStep = 0);
    //设置参数
    //拟合线性函数方法
    void Params_Config(Fit_Params _fun_p, Fit_Params _fun_i, Fit_Params _fun_d, float _I_Term_Max, float _Output_Max, float _Output_Min = numeric_limits<float>::max());
    void Params_Config(Fit_Params _fun_p, float _I_Term_Max, float _Output_Max, float _Output_Min = numeric_limits<float>::max());//纯p
    void Params_Config(PID_Mode mode, Fit_Params _fun_p, Fit_Params _fun_id, float _I_Term_Max, float _Output_Max, float _Output_Min = numeric_limits<float>::max());//pi或pd控制
    //普通参数方法
    void Params_Config(float _kp, float _ki, float _kd, float _I_Term_Max, float _Output_Max, float _Output_Min = numeric_limits<float>::max());
    void Params_Config(float _kp, float _I_Term_Max, float _Output_Max, float _Output_Min = numeric_limits<float>::max());//纯p
    void Params_Config(PID_Mode mode, float _kp, float _kid, float _I_Term_Max, float _Output_Max, float _Output_Min = numeric_limits<float>::max());//pi或pd控制
    //结算
    float Adjust(float _x);//线性自变量，使用自己算的d项
    float Adjust(float _x, float extern_d);//线性自变量，使用外部计算的d项

    Fit_Params fun_p, fun_i, fun_d;//三次拟合参数
    float kp = 0, ki = 0, kd = 0;//普通参数
    float fact_kp = 0, fact_ki = 0, fact_kd = 0;

    float Error_Max = numeric_limits<float>::max();//P项限幅
    float I_Term_Max = 0;//I项限幅
    float Output_Max = 0;//输出上限
    float Output_Min = 0;//输出下限

    float P_Term = 0;//比例项输出
    float I_Term = 0;//积分项输出
    float D_Term = 0;//微分项输出

    float I_SeparThresh = 400;   /*!< 积分分离阈值，需为正数。std::abs(error)大于该阈值取消积分作用。*/

    float target = 0;
    float current = 0;
    float error = 0;
    float out = 0;
		float integral = 0;

    float timeStep = 0;//如果被赋值，则以此为微分时间

    bool d_of_current = true;//是否使用微分先行
private:
    //线性拟合函数系数+自变量
    float fit_function(Fit_Params param, float x);
    Params_Mode params_mode = Common;
    float last_current = 0;
    float last_error = 0;
    float d_current = 0;
    float d_error = 0;
    

};

#ifdef __cplusplus
};
#endif


#endif
















