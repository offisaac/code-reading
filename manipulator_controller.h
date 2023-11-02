/*! @file manipulator_controller.h
 *  @brief header file for manipulator.cpp
 *
 */
#ifndef _MANIPULATOR_CONTROLLER_H_
#define _MANIPULATOR_CONTROLLER_H_

#include "lqrCalculater.h"
#include "manipulator.h"
#include "user_data.h"
#include "Upper_Public.h"
#include "state_data.h"

extern float debug_Dterm;
extern float debug_Kterm;
extern float debug_pitch;
extern float debug_ff;

/*轮控制器参数列表*/
enum wheelCtrl_Enumdef
{
    W_DISTANCE,
    W_SPEED,
    W_ALPHA,
    W_BETA,
    W_FF,
    W_TURN
};
/*腿控制器参数列表*/
enum jointCtrl_Enumdef
{
    J_DISTANCE,
    J_SPEED,
    J_ALPHA,
    J_BETA,
    J_TURN,
    J_LENGTHKEEP
};

#ifdef __cplusplus
extern "C"
{
#endif
    class Manipulator_Controller_Classdef
    {
    public:
        Manipulator_Controller_Classdef() {}
        Manipulator_Controller_Classdef(State_Data_Classdef *_sd, Manipulator_Classdef *r_mp, Manipulator_Classdef *l_mp, UserData_Classdef *_user)
        {
            state = _sd;
            user = _user;
            mp[RIGHT_JOINT] = r_mp;
            mp[LEFT_JOINT] = l_mp;
        }
        //参数初始化
        void Init();
        //引用类型加载
        void Load_Reference_Type(State_Data_Classdef *_sd, Manipulator_Classdef *r_mp, Manipulator_Classdef *l_mp, UserData_Classdef *_user);
        //控制器加载
        void Load_Lqr_Controller(lqrCalculater<6, 2> *_lqrCal);
        void Load_Wheel_SubController(PIDmethod _wSubCtrl[3]);
        void Load_Joint_SubController(PIDmethod _jSubCtrl[5]);
        //控制器使能
        void Set_Enable_List(bool _w[6], bool _j[6]);
        //控制结算
        void controll_adjust();

        lqrCalculater<6, 2> *lqrCal[2]; //腿的极性和轮的极性是相反的
        State_Data_Classdef *state;
        UserData_Classdef *user;
        Manipulator_Classdef *mp[2];

    public:
        /*功能动作函数*/
        void jump_adjust();

        /*LQR闭环增益*/
        void lqr_adjust(Manipulator_Classdef *_mp, lqrCalculater<6, 2> *_lqr_cal);
        void lqr_state_config(Manipulator_Classdef *mp, float *state_target, float *state_current);

        float wheel_turn_adjust();                                 //转向环
        float wheel_feedforward_adjust(Manipulator_Classdef *_mp); //前馈环
        float set_point_adjust();
        float wheelBlance_adjust();

        float length_keep_adjust(Manipulator_Classdef *_mp, uint8_t leg_side);                    //轮腿腿长控制
        float joint_turn_adjust(Manipulator_Classdef *_mp_right, Manipulator_Classdef *_mp_left); //转向防劈叉
        float roll_keep_adjust();                                                                 //车体roll轴平衡
        float turn_adaption_adjust();                                                             //车体运动转向roll内倾
        float turn_adaption_adjust2();                                                            //车体运动转向力矩补偿2
        float turn_adaption_adjust3();                                                            //车体运动转向力矩补偿3

        float slope_keep_feedforward(Manipulator_Classdef *_mp); //上坡水平方向虚拟力前馈

        /****************************控制器********************************/

        PIDmethod *w_turn_pid;
        PIDmethod *wheelBalance_pid;
        PIDmethod *j_turn_pid;
        PIDmethod *set_point_pid;

        PIDmethod *j_follow_pid;
        PIDmethod *j_roll_keep_pid;
        PIDmethod *j_length_keep_pid[2];

        /*创建状态变量数组*/
        float lqr_target[6];
        float lqr_current[6];
        /*控制器使能数组*/
        bool wheel_enable_list[6] = {0};
        bool joint_enable_list[6] = {0};

        float wheel_output[2][6];
        float joint_output[2][6];
    };
#ifdef __cplusplus
};
#endif

#endif
