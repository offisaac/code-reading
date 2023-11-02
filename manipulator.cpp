/**
 ******************************************************************************
 * Copyright (c) 2023 - ~, SCUT-RobotLab Development Team
 * @file manipulator.cpp
 * @author 张至睿 (2231625449@qq.com)
 * @brief 并联对称五连杆物理参数存储类与运动学解算
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
#include "manipulator.h"
#include "math.h"

#define END_QUALITY 0.644 + 0.96 //五连杆末端质量（轮子+9025）

// 初始化参数
void Manipulator_Classdef::Init()
{
    bipes.L1 = 0.15f;
    bipes.L2 = 0.288f;
    bipes.L3 = 0.288f;
    bipes.L4 = 0.15f;
    bipes.L5 = 0.15f;

    body.mass = 14.28f;
    body.length = 0.143f;
    body.k = 0;

    target_joint.pendulum.length = 0.15f;

    current_joint.pendulum.mass = 0.94f;
    current_joint.pendulum.k = 0.3989f;
}

/**
 * @brief  五连杆正向运动学
 * @note	 根据当前关节转动角度计算当前腿杆顶点的位置
 * @param
 * @return
 * @retval  None
 */
void Manipulator_Classdef::forward_kinematics_cal()
{
    float legAngle = 0;
    // body.angle = state->current_pos.pitch;
    float X3 = bipes.L5 / 2.f + bipes.L1 * cosf(current_joint.f_angle);
    float Y3 = -bipes.L1 * sinf(current_joint.f_angle);
    float X4 = -bipes.L5 / 2.f + bipes.L4 * cosf(current_joint.b_angle);
    float Y4 = -bipes.L4 * sinf(current_joint.b_angle);
    float X43 = X3 - X4;
    float Y43 = Y3 - Y4;
    float X6 = (X3 + X4) / 2.f;
    float Y6 = (Y3 + Y4) / 2.f;
    float D43 = sqrtf(powf(X43, 2) + powf(Y43, 2));
    current_joint.position_x = sqrtf((powf(bipes.L2, 2) - powf(D43, 2) / 4.f) / powf(D43, 2)) * Y43 + X6;
    current_joint.position_y = sqrtf((powf(bipes.L3, 2) - powf(D43, 2) / 4.f) / powf(D43, 2)) * (-X43) + Y6;

    current_joint.pendulum.length = sqrtf(powf(current_joint.position_x, 2) + powf(current_joint.position_y, 2));
    legAngle = atanf(current_joint.position_x / (float)current_joint.position_y);
    current_joint.pendulum.angle = legAngle + body.angle;

    if (dt > 0)
    {
        current_joint.pendulum.dlength = dLengthCal.calc(current_joint.pendulum.length, dt);    // 杆长变化率
        current_joint.pendulum.ddlength = ddLengthCal.calc(current_joint.pendulum.dlength, dt); //杆长变化率的微分
        current_joint.pendulum.dangle = dLegAngleCal.calc(legAngle, this->dt) + body.dangle;    // 求角速度
        current_joint.pendulum.ddangle = ddLegAngleCal.calc(current_joint.pendulum.dangle, dt); //求角加速度
    }
}

/**
 * @brief  五连杆逆向运动学
 * @note	 根据当前顶点坐标计算关节电机应该到达的位置
 * @param
 * @return
 * @retval  None
 */
void Manipulator_Classdef::inverse_kinematics_cal(float _x, float _y)
{
    if (_x < -0.5f * bipes.L5)
    {
        target_joint.b_angle = 3.1415926f - atanf(-_y / (-_x - bipes.L5 / 2.f)) + acosf((powf(bipes.L4, 2) - powf(bipes.L3, 2) + powf(-0.5f * bipes.L5 - _x, 2) + powf(_y, 2)) / (2.f * bipes.L1 * sqrtf(powf(-0.5f * bipes.L5 - _x, 2) + powf(_y, 2))));
    }
    else
    {
        target_joint.b_angle = atanf(-_y / (_x + bipes.L5 / 2.f)) + acosf((powf(bipes.L4, 2) - powf(bipes.L3, 2) + powf(-0.5f * bipes.L5 - _x, 2) + powf(_y, 2)) / (2.f * bipes.L1 * sqrtf(powf(-0.5f * bipes.L5 - _x, 2) + powf(_y, 2))));
    }
    if (_x > 0.5f * bipes.L5)
    {
        target_joint.f_angle = atanf(-_y / (_x - bipes.L5 / 2.f)) - acosf((powf(bipes.L1, 2) - powf(bipes.L2, 2) + powf(0.5f * bipes.L5 - _x, 2) + powf(_y, 2)) / (2.f * bipes.L1 * sqrtf(powf(0.5f * bipes.L5 - _x, 2) + powf(_y, 2))));
    }
    else
    {
        target_joint.f_angle = 3.1415926f - atanf(-_y / (-_x + bipes.L5 / 2.f)) - acosf((powf(bipes.L1, 2) - powf(bipes.L2, 2) + powf(0.5f * bipes.L5 - _x, 2) + powf(_y, 2)) / (2.f * bipes.L1 * sqrtf(powf(0.5f * bipes.L5 - _x, 2) + powf(_y, 2))));
    }
}

/**
 * @brief  五连杆正向雅可比（VMC）
 * @note
 * @param
 * @return 两个电机力矩
 * @retval  None
 */
void Manipulator_Classdef::jacobian()
{
    float A = powf(bipes.L1, 2) * powf((sinf(current_joint.b_angle) - sinf(current_joint.f_angle)), 2) + powf(bipes.L5 + bipes.L1 * (cosf(current_joint.f_angle) - cosf(current_joint.b_angle)), 2);
    float B = -2.f * powf(bipes.L1, 2) * (sinf(current_joint.b_angle) - sinf(current_joint.f_angle)) * cosf(current_joint.f_angle) - 2.f * (bipes.L5 + bipes.L1 * (cosf(current_joint.f_angle) - cosf(current_joint.b_angle))) * bipes.L1 * sinf(current_joint.f_angle);
    float C = 2.f * powf(bipes.L1, 2) * (sinf(current_joint.b_angle) - sinf(current_joint.f_angle)) * cosf(current_joint.b_angle) + 2.f * (bipes.L5 + bipes.L1 * (cosf(current_joint.f_angle) - cosf(current_joint.b_angle))) * bipes.L1 * sinf(current_joint.b_angle);
    float D = sqrtf(powf(bipes.L2, 2) / A - 0.25f);
    float E = (bipes.L1 * (sinf(current_joint.f_angle) - sinf(current_joint.b_angle)) * powf(bipes.L2, 2)) / (2.f * powf(A, 2) * D);
    float F = ((bipes.L5 + bipes.L1 * (cosf(current_joint.f_angle) - cosf(current_joint.b_angle))) * powf(bipes.L2, 2)) / (2 * powf(A, 2) * D);

    float x_a1 = E * B - bipes.L1 * cosf(current_joint.f_angle) * D - 0.5f * bipes.L1 * sinf(current_joint.f_angle);
    float x_a2 = E * C + bipes.L1 * cosf(current_joint.b_angle) * D - 0.5f * bipes.L1 * sinf(current_joint.b_angle);
    float y_a1 = F * B + bipes.L1 * sinf(current_joint.f_angle) * D - 0.5f * bipes.L1 * cosf(current_joint.f_angle);
    float y_a2 = F * C - bipes.L1 * sinf(current_joint.b_angle) * D - 0.5f * bipes.L1 * cosf(current_joint.b_angle);

    this->torque_output.f_joint = (x_a1 * target_joint.Fx + y_a1 * target_joint.Fy); // 符号根据建模和matlab结果综合整定
    this->torque_output.b_joint = (x_a2 * target_joint.Fx + y_a2 * target_joint.Fy);

    this->current_joint.Fy = (x_a2 * torque_fact.f_joint - x_a1 * torque_fact.b_joint) / (x_a2 * y_a1 - x_a1 * y_a2);
    this->current_joint.Fx = (y_a2 * torque_fact.f_joint - y_a1 * torque_fact.b_joint) / (y_a2 * x_a1 - y_a1 * x_a2);
}

/**
 * @brief  整车质心解算(仅对单腿)
 * @note    运用惯性矩原理
 * @param
 * @return
 * @retval  None
 */
void Manipulator_Classdef::overall_barycenter_cal()
{
    float X1 = current_joint.pendulum.length * (1.f - current_joint.pendulum.k) * sinf(current_joint.pendulum.angle);
    float Y1 = current_joint.pendulum.length * (1.f - current_joint.pendulum.k) * cosf(current_joint.pendulum.angle);
    float X2 = current_joint.pendulum.length * sinf(current_joint.pendulum.angle) + body.length * (1 - body.k) * sinf(body.angle);
    float Y2 = current_joint.pendulum.length * cosf(current_joint.pendulum.angle) + body.length * (1 - body.k) * cosf(body.angle);
    float X12 = X2 - X1;
    float Y12 = Y2 - Y1;
    float D12 = sqrtf(powf(X12, 2) + powf(Y12, 2));
    float Angle12 = atan(X12 / Y12);
    float k3 = current_joint.pendulum.mass / (body.mass / 2.f) / (1 + current_joint.pendulum.mass / (body.mass / 2.f)); // 这里相当于取一半的车体质量
    float X3 = X1 + D12 * (1 - k3) * sinf(Angle12);
    float Y3 = Y1 + D12 * (1 - k3) * cosf(Angle12);
    overall.mass = current_joint.pendulum.mass + body.mass / 2.f;
    overall.length = sqrtf(powf(X3, 2) + powf(Y3, 2));
    overall.angle = atan(X3 / Y3);
    overall.k = 0;
}

/**
 * @brief  末端轮子支持力解算
 * @note
 * @param   机体竖直方向加速度加速度(单位m/s^2)
 * @return
 * @retval  None
 */
float debug_force;
void Manipulator_Classdef::support_force_cal(float body_vertical_acc)
{
    //机器人作用于轮子竖直向下的力
    float vertical_force = current_joint.Fy;
    //驱动轮竖直方向的运动加速度
    float vertical_acc = body_vertical_acc -
                         current_joint.pendulum.ddlength * cosf(current_joint.pendulum.angle) +
                         2 * current_joint.pendulum.dlength * current_joint.pendulum.dangle * sinf(current_joint.pendulum.angle) +
                         current_joint.pendulum.length * current_joint.pendulum.ddangle * sinf(current_joint.pendulum.angle) +
                         current_joint.pendulum.length * powf(current_joint.pendulum.dangle, 2) * cosf(current_joint.pendulum.angle);

    current_joint.support_force = END_QUALITY * body_vertical_acc - vertical_force + END_QUALITY * 9.8;
    debug_force = END_QUALITY * body_vertical_acc - target_joint.Fy + END_QUALITY * 9.8;
}

void Manipulator_Classdef::current_joint_update(float _f_angel, float _b_angel, float _f_torque, float _b_torque)
{
    current_joint.f_angle = _f_angel;
    current_joint.b_angle = _b_angel;
    torque_fact.f_joint = _f_torque;
    torque_fact.b_joint = _b_torque;
}

void Manipulator_Classdef::body_angle_update(float _angle, float _angleVel)
{
    body.angle = _angle;
    body.dangle = _angleVel;
}

void Manipulator_Classdef::timer_update(float _dt)
{
    dt = _dt;
}
