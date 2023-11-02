/**
 ******************************************************************************
 * Copyright (c) 2023 - ~, SCUT-RobotLab Development Team
 * @file manipulator.h
 * @author 张至睿 (2231625449@qq.com)
 * @brief header file for manipulator.cpp
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
#ifndef _MANIPULATOR_H_
#define _MANIPULATOR_H_

#include "DiffCalculator.h"
#define DEGREE_TO_RADIAN PI / 180.f

// 连杆长度
struct Bipe_Structdef
{
    float L1;
    float L2;
    float L3;
    float L4;
    float L5;
};

// 等效摆杆
struct Pendulum_Structdef
{
    float mass;      // 重量（质心）
    float length;    // 杆长
    float dlength;   // 杆长微分
    float ddlength;  //杆长微分的微分
    float angle = 0; // 摆角
    float dangle;    // 角速度
    float ddangle;   //角加速度
    float k;         // 重心长度系数length*(1-k)
};

// 等效关节
struct Joint_Structdef
{
    float f_angle;
    float b_angle;
    float position_x;
    float position_y;
    Pendulum_Structdef pendulum;
    float Fx;
    float Fy;
    float support_force;
};

// 力矩
struct Torque_Structdef
{
    float wheel;
    float f_joint;
    float b_joint;
};

#ifdef __cplusplus
extern "C"
{
#endif
    /*轮腿类*/
    class Manipulator_Classdef
    {
    public:
        Manipulator_Classdef(float _t = 0) { dt = _t; }
        void Init(); // 初始化机械参数

        void current_joint_update(float _f_angel, float _b_angel, float _f_torque, float _b_torque); // 更新关节参数
        void body_angle_update(float _angle, float _angleVel);                                       // 更新车身角度
        void timer_update(float _dt);                                                                // 更新时间微分

        void forward_kinematics_cal();                   // 正运动学解算
        void inverse_kinematics_cal(float _x, float _y); // 逆运动学解算
        void jacobian();                                 // 正雅可比VMC
        void overall_barycenter_cal();                   // 重心计算
        void support_force_cal(float body_vertical_acc); //末端轮子支持力解算

        Joint_Structdef target_joint;   // 目标轮腿参数
        Joint_Structdef current_joint;  // 当前轮腿参数
        Torque_Structdef torque_output; // 期望输出力矩参数
        Torque_Structdef torque_fact;   // 实际输出力矩参数
        Bipe_Structdef bipes;           // 五连杆参数
        Pendulum_Structdef body;        // 车身摆杆等效参数
        Pendulum_Structdef overall;     // 车身与轮腿拟合参数

    private:
        DiffCalculator<5> dLengthCal;
        DiffCalculator<20> ddLengthCal;
        DiffCalculator<5> dLegAngleCal;
        DiffCalculator<20> ddLegAngleCal;
        float dt;
    };
#ifdef __cplusplus
};
#endif

#endif
