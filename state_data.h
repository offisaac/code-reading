/*! @file state_data.h
 *  @brief Data structs for motion commands storage
 *
 *  This file contains the State_Data_Classdef and state_timer class,
 *  which provide ways to store motion commands.
 *  此文件用于储存车辆整体控制参量及标志位
 */
#ifndef _STATE_DATA_H_
#define _STATE_DATA_H_

#include <stdint.h>
#include <cstdint>
#include <stddef.h>
#include "math.h"
#include <limits> //定义各种变量的储存最大值
#include "Upper_Public.h"
#include "boardComProtocol.h"
#include "manipulator.h"

typedef uint32_t (*getSystemTick)(void); //函数指针，用于接入不同单片机获取时间栈的函数

class state_timer
{
public:
    static uint8_t getMicroTick_regist(uint32_t (*getTick_fun)(void)); //获取当前时间函数接口
    static getSystemTick Get_SystemTick;                               //获取时间的函数
    float dt = 0;                                                      //时间微分
    uint32_t last_time;                                                //记录上次时间
    uint8_t UpdataTimeStamp(void);                                     //时间栈更新
};

/*线性参数*/
struct Linear_Structdef
{
    float x;
    float y;
    float z;
};
/*角度参数(统一用弧度制)*/
struct Angular_Structdef
{
    float pitch;
    float yaw;
    float roll;
};
/*标志位*/
struct StateFlag_Structdef
{
    bool sport_flag = false; //移动标志位
    bool break_flag = false;
    bool leap_flag = false;
    bool slope_flag = false;
    bool pull_up_flag = false;  //采用滞回比较器
    bool distance_flag = false; //路程环的使能位
    bool r_single_roll = false; //是否是单边roll轴控制
    bool l_single_roll = false;
    bool weightlessness = false;
    bool tiny_weightlessness = false;
    bool is_weightless_check = true; //腾空检测开关
    bool jump_flag = false;          //弹跳开关
    bool kd_inhibition = false;      //抑制支撑环kd
    bool set_point_flag = false;     //是否开启重心自适应
    bool is_legforce = false;        //是否开腿力控
    bool is_jumping = false;         //是否正在跳跃
};

#ifdef __cplusplus
extern "C"
{
#endif
    /*状态类*/
    class State_Data_Classdef : public state_timer
    {
    public:
        State_Data_Classdef();
        State_Data_Classdef(Manipulator_Classdef *r_mp, Manipulator_Classdef *l_mp)
        {
            mp[RIGHT_JOINT] = r_mp;
            mp[LEFT_JOINT] = l_mp;
        }
        State_Data_Classdef(Manipulator_Classdef *r_mp, Manipulator_Classdef *l_mp, float _dt)
        {
            mp[RIGHT_JOINT] = r_mp;
            mp[LEFT_JOINT] = l_mp;
            dt = _dt;
        }

        /*状态更新函数*/
        void timeStamp_update(float _dt = 0);
        void load_leg_state(Manipulator_Classdef *r_mp, Manipulator_Classdef *l_mp)
        {
            mp[RIGHT_JOINT] = r_mp;
            mp[LEFT_JOINT] = l_mp;
        }
        void target_update(float target_speed, float target_turn);   //目标值更新
        void current_location_update(float x, float y, float z);     //位置更新
        void current_speed_update(float x, float y, float z);        //速度更新
        void current_acc_update(float x, float y, float z);          //加速度更新
        void current_pos_update(float pitch, float yaw, float roll); //角度更新
        void current_av_update(float pitch, float yaw, float roll);  //角速度更新
        void power_limit_update(float _scale);                       //功控系数更新
        void centripetal_force_update(float _force);                 //向心力系数更新
        void gimbal_state_update(chassis_state_structdef _data);     //云台下发标志位更新
        /*状态判断函数*/
        void sport_adaption();            //用于运动状态检测
        void weightlessness_check();      //用于检测跳台阶，飞坡等需要缓冲的腾空检测
        void tiny_weightlessness_check(); //用于上下坡，伸长缩短腿时保持平衡的腾空检测

        Linear_Structdef target_location = {0};
        Linear_Structdef current_location = {0};

        Linear_Structdef target_speed = {0};
        Linear_Structdef raw_target_speed = {0};
        Linear_Structdef current_speed = {0};

        Linear_Structdef target_acc = {0};
        Linear_Structdef current_acc = {0};

        Angular_Structdef target_pos = {0};
        Angular_Structdef current_pos = {0};

        Angular_Structdef target_av = {0};
        Angular_Structdef current_av = {0};

        StateFlag_Structdef flags; //标志位手动一个个配置

        chassis_state_structdef chassis_state = {};

        /*功控系数*/
        float power_scale = 1.f;
        /*向心力系数*/
        float centripetal_force_scale = 0.f;

    private:
        Manipulator_Classdef *mp[2];
    };
#ifdef __cplusplus
};
#endif

#endif
