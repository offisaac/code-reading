/*! @file manipulator_controller.cpp
 *  @brief Controll methods of manipulator robots
 *         轮腿控制器
 */
#include "manipulator_controller.h"
#include "internal.h"

float debug_DisOut;
float debug_Dterm;
float debug_Kterm;
float debug_pitch;
float debug_ff;
float ff_gain = 0.25f;
float debug_rollkeep;
float debug_alphaleg;
float debug_wout;

extern const float ratio_rad_to_degree;

/****************************加载外部引用类型********************************/
void Manipulator_Controller_Classdef::Load_Reference_Type(State_Data_Classdef *_sd, Manipulator_Classdef *r_mp, Manipulator_Classdef *l_mp, UserData_Classdef *_user)
{
    state = _sd;
    user = _user;
    mp[RIGHT_JOINT] = r_mp;
    mp[LEFT_JOINT] = l_mp;
}

/****************************加载控制器********************************/
void Manipulator_Controller_Classdef::Load_Lqr_Controller(lqrCalculater<6, 2> *_lqrCal)
{
    lqrCal[RIGHT_JOINT] = &_lqrCal[RIGHT_JOINT];
    lqrCal[LEFT_JOINT] = &_lqrCal[LEFT_JOINT];
}

void Manipulator_Controller_Classdef::Load_Wheel_SubController(PIDmethod _wSubCtrl[3])
{
    w_turn_pid = &_wSubCtrl[0];
    w_turn_pid->PID_Init(Common);
    set_point_pid = &_wSubCtrl[1];
    set_point_pid->PID_Init(Common);
    wheelBalance_pid = &_wSubCtrl[2];
    wheelBalance_pid->PID_Init(Common);
}

void Manipulator_Controller_Classdef::Load_Joint_SubController(PIDmethod _jSubCtrl[5])
{
    j_turn_pid = &_jSubCtrl[0];
    j_follow_pid = &_jSubCtrl[1];
    j_roll_keep_pid = &_jSubCtrl[2];
    j_length_keep_pid[RIGHT_JOINT] = &_jSubCtrl[3];
    j_length_keep_pid[LEFT_JOINT] = &_jSubCtrl[4];
    j_turn_pid->PID_Init(Common);
    j_follow_pid->PID_Init(Common);
    j_roll_keep_pid->PID_Init(Common);
    j_length_keep_pid[RIGHT_JOINT]->PID_Init(Common);
    j_length_keep_pid[LEFT_JOINT]->PID_Init(Common);
}

void Manipulator_Controller_Classdef::Init()
{
    /*数据写入控制器*/
    lqrCal[0]->init(user->leg_lqr_params);
    lqrCal[1]->init(user->leg_lqr_params);

    w_turn_pid->Params_Config(user->lqr_yaw_kp, 0, numeric_limits<float>::max());

    wheelBalance_pid->Params_Config(user->wheelBalance_kp, 0, user->wheelBalance_O_Max);

    set_point_pid->Params_Config(PID_Mode::IS_PI, user->setPoint_kp, user->setPoint_ki, user->setPoint_I_Max, user->setPoint_O_Max);

    j_roll_keep_pid->Params_Config(user->roll_keep_kp, user->roll_keep_ki, user->roll_keep_kd, user->roll_keep_imax, user->roll_keep_omax);

    j_length_keep_pid[RIGHT_JOINT]->Params_Config(user->length_keep_kp, user->length_keep_ki, user->length_keep_kd, user->length_keep_imax, numeric_limits<float>::max());
    j_length_keep_pid[LEFT_JOINT]->Params_Config(user->length_keep_kp, user->length_keep_ki, user->length_keep_kd, user->length_keep_imax, numeric_limits<float>::max());

    j_turn_pid->Params_Config(user->j_turn_kp, 0, numeric_limits<float>::max());
    j_turn_pid->d_of_current = false;

    j_follow_pid->Params_Config(PID_Mode::IS_PD, user->j_follow_kp, user->j_follow_kd, 0, numeric_limits<float>::max());
    j_follow_pid->d_of_current = false;

    for (int i = 0; i < 6; i++) // 初始化使能列表为false
    {
        wheel_enable_list[i] = false;
        joint_enable_list[i] = false;
    }
}

/**
 * @brief  控制器使能函数
 * @note
 * @param   轮控制使能清单；腿控制使能清单；
 * @return
 * @retval  None
 */
void Manipulator_Controller_Classdef::Set_Enable_List(bool _w[6], bool _j[6])
{
    for (int i = 0; i < 6; i++)
    {
        if (_w[i] != false && _w[i] != true)
        {
            wheel_enable_list[i] = false;
        }
        else
        {
            wheel_enable_list[i] = _w[i];
        }
        if (_j[i] != false && _j[i] != true)
        {
            joint_enable_list[i] = false;
        }
        else
        {
            joint_enable_list[i] = _j[i];
        }
    }
}

/**
 * @brief  总控函数
 * @note
 * @param
 * @return
 * @retval  None
 */
// float rotation_roll_target = -1.2f;
float rotation_roll_target = 0.f;
void Manipulator_Controller_Classdef::controll_adjust()
{
    /*轮腿控制部分(v是轮杆方向，h是轮杆法向)*/
    float alpha[2];
    float fv[2], fh[2];
    float turn_fv = 0;
    static MeanFilter<100> turn_filter;

    for (int i = 0; i < 2; i++)
    {
        mp[i]->body_angle_update(state->current_pos.pitch, state->current_av.pitch); // 向腿杆更新车体倾角
        mp[i]->forward_kinematics_cal();                                             // 正运动学计算
        mp[i]->overall_barycenter_cal();                                             // 重心计算
        mp[i]->inverse_kinematics_cal(0, -mp[i]->target_joint.pendulum.length);      // 逆运动学计算
        mp[i]->support_force_cal(state->current_acc.z * 9.8f);                       // 支持力解算
    }
    jump_adjust();

    /*重心自适应*/
    if (state->flags.is_legforce == false)
    {
        state->target_pos.pitch += wheelBlance_adjust();
    }
    else
    {
        mp[RIGHT_JOINT]->target_joint.pendulum.angle = mp[LEFT_JOINT]->target_joint.pendulum.angle = set_point_adjust();
    }
    /*轮控制*/
    wheel_turn_adjust();
    for (int j = 0; j < 2; j++)
    {
        /*lqr解算*/
        lqr_adjust(mp[j], lqrCal[j]);
        /*轮子控制参数导出*/
        wheel_output[j][W_DISTANCE] = lqrCal[j]->single_out[0][lqrID::distance];
        wheel_output[j][W_SPEED] = lqrCal[j]->single_out[0][lqrID::speed];
        wheel_output[j][W_ALPHA] = lqrCal[j]->single_out[0][lqrID::alpha] + lqrCal[j]->single_out[0][lqrID::dalpha];
        wheel_output[j][W_BETA] = lqrCal[j]->single_out[0][lqrID::beta] + lqrCal[j]->single_out[0][lqrID::dbeta];
        wheel_output[j][W_FF] = wheel_feedforward_adjust(mp[j]);
        wheel_output[j][W_TURN] = w_turn_pid->out;
        if (state->chassis_state.rotation_state == true)
        {
            wheel_output[j][W_TURN] = upper::constrain(wheel_output[j][W_TURN], 2.5f);
        }
        else
        {
            wheel_output[j][W_TURN] = upper::constrain(wheel_output[j][W_TURN], 3.f);
        }
    }

    if (state->chassis_state.rotation_state == false)
    {
        state->target_pos.roll = turn_filter.f(turn_adaption_adjust()); // 第一类转向补偿
    }
    else
    {
        state->target_pos.roll = rotation_roll_target * upper::degree2rad_ratio;
    }

    // turn_fv = turn_adaption_adjust2();//第二类转向补偿（可用，但不如第一个方案）
    turn_fv = turn_adaption_adjust3(); // 第三类转向补偿（如果增益比较大，盲道转向容易起飞，如果增益比较小平地转向补偿不足）

    if (state->flags.weightlessness == true && state->flags.is_jumping == false)
    {
        mp[LEFT_JOINT]->target_joint.pendulum.length = mp[RIGHT_JOINT]->target_joint.pendulum.length = 0.26;
    }
    else
    {
        float length_error = roll_keep_adjust(); // roll计算
        mp[RIGHT_JOINT]->target_joint.pendulum.length += length_error;
        mp[LEFT_JOINT]->target_joint.pendulum.length -= length_error;

        mp[RIGHT_JOINT]->target_joint.pendulum.length = upper::constrain(mp[RIGHT_JOINT]->target_joint.pendulum.length, 0.14, 0.4);
        mp[LEFT_JOINT]->target_joint.pendulum.length = upper::constrain(mp[LEFT_JOINT]->target_joint.pendulum.length, 0.14, 0.4);
    }

    /*转向+劈叉控制*/
    joint_turn_adjust(mp[RIGHT_JOINT], mp[LEFT_JOINT]);
    for (int k = 0; k < 2; k++)
    {
        /*获得腿杆控制量*/
        joint_output[k][J_DISTANCE] = -lqrCal[k]->single_out[1][lqrID::distance] / mp[k]->current_joint.pendulum.length;
        joint_output[k][J_SPEED] = -lqrCal[k]->single_out[1][lqrID::speed] / mp[k]->current_joint.pendulum.length;
        joint_output[k][J_ALPHA] = -(lqrCal[k]->single_out[1][lqrID::alpha] + lqrCal[k]->single_out[1][lqrID::dalpha]) / mp[k]->current_joint.pendulum.length;
        joint_output[k][J_BETA] = -(lqrCal[k]->single_out[1][lqrID::beta] + lqrCal[k]->single_out[1][lqrID::dbeta]) / mp[k]->current_joint.pendulum.length;
        joint_output[k][J_TURN] = j_follow_pid->out - j_turn_pid->out;
        joint_output[k][J_LENGTHKEEP] = length_keep_adjust(mp[k], k);
        /*腿杆相对于地面的角度*/
        alpha[k] = mp[k]->current_joint.pendulum.angle - mp[k]->body.angle;
        /*根据使能清单关闭相应的控制模块*/
        for (int n = 0; n < 6; n++)
        {
            if (wheel_enable_list[n] == false)
            {
                wheel_output[k][n] = 0;
            }
            if (joint_enable_list[n] == false)
            {
                joint_output[k][n] = 0;
            }
        }

        /*力矩集成输出*/
        mp[k]->torque_output.wheel = wheel_output[k][W_DISTANCE] + wheel_output[k][W_SPEED] + wheel_output[k][W_ALPHA] + wheel_output[k][W_BETA] + wheel_output[k][W_FF];

        // 统一极性，以车辆前进方向为x轴，沿x轴向下角度增加旋转方向为力矩输出正方向
        if (k == RIGHT_JOINT)
        {
            fv[k] = turn_fv;
            fh[k] = (joint_output[k][J_DISTANCE] + joint_output[k][J_SPEED] + joint_output[k][J_ALPHA] + joint_output[k][J_BETA]) + joint_output[k][J_TURN];
        }
        else
        {
            fv[k] = -turn_fv;
            fh[k] = (joint_output[k][J_DISTANCE] + joint_output[k][J_SPEED] + joint_output[k][J_ALPHA] + joint_output[k][J_BETA]) - joint_output[k][J_TURN];
        }
        mp[k]->target_joint.Fx = fh[k] * cosf(alpha[k]) + fv[k] * sinf(alpha[k]);
        mp[k]->target_joint.Fy = -fh[k] * sinf(alpha[k]) + fv[k] * cosf(alpha[k]) + joint_output[k][J_LENGTHKEEP];
        mp[k]->jacobian(); // VMC运算
    }
    mp[RIGHT_JOINT]->torque_output.wheel += wheel_output[RIGHT_JOINT][W_TURN];
    mp[LEFT_JOINT]->torque_output.wheel -= wheel_output[LEFT_JOINT][W_TURN];

    debug_ff = fh[0];
    debug_Dterm = wheel_output[0][W_BETA];
    debug_Kterm = wheel_output[0][W_ALPHA];
}

/*******************************************LQR控制***********************************************/
/**
 * @brief  lqr总控
 * @note
 * @param
 * @return  力矩输出
 * @retval  None
 */
void Manipulator_Controller_Classdef::lqr_adjust(Manipulator_Classdef *_mp, lqrCalculater<6, 2> *_lqr_cal)
{
    /*数组打包*/
    lqr_state_config(_mp, lqr_target, lqr_current);
    /*变量写入控制器*/
    _lqr_cal->updateData(lqr_target, lqr_current, _mp->current_joint.pendulum.length);
    /*控制器结算*/
    _lqr_cal->adjust();
}
/**
 * @brief  lqr状态空间变量设置
 * @note    把状态空间变量打包到给定数组当中
 * @param   目标状态1X6向量，当前状态1X6向量，状态使能1X6向量
 * @return
 * @retval  None
 */
void Manipulator_Controller_Classdef::lqr_state_config(Manipulator_Classdef *mp, float *state_target, float *state_current)
{
    state_target[0] = state->target_location.y;
    state_current[0] = state->current_location.y;

    state_target[1] = state->target_speed.y;
    state_current[1] = state->current_speed.y;

    state_target[2] = mp->target_joint.pendulum.angle;
    state_current[2] = mp->current_joint.pendulum.angle;

    state_target[3] = 0;
    state_current[3] = mp->current_joint.pendulum.dangle;

    state_target[4] = state->target_pos.pitch;
    state_current[4] = state->current_pos.pitch;

    state_target[5] = 0;
    state_current[5] = state->current_av.pitch;
}

/*******************************************轮控制***********************************************/

/**
 * @brief  lqr转向环
 * @note
 * @param
 * @return  力矩输出
 * @retval  None
 */
float Manipulator_Controller_Classdef::wheel_turn_adjust()
{
    w_turn_pid->target = state->target_av.yaw;
    w_turn_pid->current = state->current_av.yaw;
    w_turn_pid->Adjust(0);
    return w_turn_pid->out;
}

/**
 * @brief  lqr前馈环
 * @note
 * @param
 * @return  力矩输出
 * @retval  None
 */
float Manipulator_Controller_Classdef::wheel_feedforward_adjust(Manipulator_Classdef *_mp)
{
    /*float error = 0 - _mp->overall.angle;*/
    // float debug_feedforward = -_mp->overall.mass * 9.8f * _mp->overall.length;
    float debug_feedforward = user->stand_ff * _mp->body.mass * 9.8f * (_mp->current_joint.pendulum.length + _mp->body.length);
    /*float debug_feedforward = -0.5 * _mp.body.mass * 9.8 * _mp->overall.length;*/
    /*float debug_feedforward = -_mp->overall.mass * 9.8 * (_mp.current_joint.pendulum.length + _mp.body.length);*/
    //    if (state->current_pos.pitch > 0)
    //    {
    //        return debug_feedforward * sinf(abs(state->current_pos.pitch));
    //    }
    //    else
    //    {
    //        return -debug_feedforward * sinf(abs(state->current_pos.pitch));
    //    }
    return debug_feedforward * sinf(state->current_pos.pitch - balance_point / 180.f * 3.1415926f);
}

/**
 * @brief  重心自适应
 * @note
 * @param
 * @return
 * @retval  None
 */
float Manipulator_Controller_Classdef::set_point_adjust()
{
    // if (state->chassis_state.ascent_state)
    // {
    //     set_point_pid->Params_Config(PID_Mode::IS_PI, user->setPoint_kp / 3.f, user->setPoint_ki / 2.f, user->setPoint_I_Max / 2.f, user->setPoint_O_Max / 3.f);
    // }
    // else
    // {
    //     set_point_pid->Params_Config(PID_Mode::IS_PI, user->setPoint_kp, user->setPoint_ki, user->setPoint_I_Max, user->setPoint_O_Max);
    // }

    set_point_pid->kp = user->setPoint_kp;
    set_point_pid->ki = user->setPoint_ki = 0.;
    set_point_pid->I_Term_Max = user->setPoint_I_Max = 0.12f;
    set_point_pid->Output_Max = user->setPoint_O_Max = 0.12f;

    set_point_pid->target = 0;
    set_point_pid->current = state->current_speed.y;
    set_point_pid->Adjust(0);
    if (state->flags.sport_flag || state->chassis_state.rotation_state)
    {
        return 0;
    }
    return set_point_pid->out;
}

/**
 * @brief  普平管理
 * @note
 * @param
 * @return
 * @retval  None
 */
float Manipulator_Controller_Classdef::wheelBlance_adjust()
{
    wheelBalance_pid->target = 0;
    wheelBalance_pid->current = state->current_speed.y;
    wheelBalance_pid->Adjust(0);
    if (state->flags.tiny_weightlessness || state->flags.weightlessness)
    {
        return 0;
    }
    if (!state->flags.set_point_flag)
    {
        if (state->target_speed.y > 0.3f)
        {
            if (abs(state->current_speed.y) > 0.9 * abs(state->target_speed.y))
            {
                return 0;
            }
            return 2.f * upper::degree2rad_ratio;
        }
        else if (state->target_speed.y < -0.3f)
        {
            if (abs(state->current_speed.y) > 0.9 * abs(state->target_speed.y))
            {
                return 0;
            }
            return -2.f * upper::degree2rad_ratio;
        }
    }
    return wheelBalance_pid->out;
}

/*********************************************腿控制****************************************************/

/**
 * @brief  车体转向内倾
 * @note    用角速度，速度，腿长三个变量拟合增益
 * @note    如果步做阈值控制和输出滤波，该策略将会导致过盲道震荡加剧
 * @param
 * @return  roll轴角度目标值
 * @retval  None
 */
float Manipulator_Controller_Classdef::turn_adaption_adjust()
{
    if (state->flags.sport_flag && abs(state->current_av.yaw) > 0.5f)
    {
        if (state->current_av.yaw == 0)
        {
            return 0;
        }
        float out;
        if (state->current_speed.y >= 0)
        {
            // out = -user->turn_adaption_kp * (state->current_av.yaw / abs(state->current_av.yaw)) * (0.4f + abs(state->current_av.yaw) * 1.1f / 1.5f) * powf((mp[RIGHT_JOINT]->current_joint.pendulum.length + mp[LEFT_JOINT]->current_joint.pendulum.length), 2) * (0.7f + abs(state->current_speed.y) * 0.6f / 3.f);
            out = -user->turn_adaption_kp * (state->current_av.yaw / abs(state->current_av.yaw)) * powf((mp[RIGHT_JOINT]->current_joint.pendulum.length + mp[LEFT_JOINT]->current_joint.pendulum.length), 2) * state->centripetal_force_scale;
        }
        else if (state->current_speed.y < 0)
        {
            // out = user->turn_adaption_kp * (state->current_av.yaw / abs(state->current_av.yaw)) * (0.4f + abs(state->current_av.yaw) * 1.1f / 1.5f) * powf((mp[RIGHT_JOINT]->current_joint.pendulum.length + mp[LEFT_JOINT]->current_joint.pendulum.length), 2) * (0.7f + abs(state->current_speed.y) * 0.6f / 3.f);
            out = user->turn_adaption_kp * (state->current_av.yaw / abs(state->current_av.yaw)) * powf((mp[RIGHT_JOINT]->current_joint.pendulum.length + mp[LEFT_JOINT]->current_joint.pendulum.length), 2) * state->centripetal_force_scale;
        }
        return upper::constrain(out, user->turn_adaption_omax);
    }
    else
    {
        return 0;
    }
}

/**
 * @brief  车体转向内倾2
 * @note    向心力拟合增益
 * @param
 * @return  两轮杆方向力矩差分
 * @retval  None
 */
float Manipulator_Controller_Classdef::turn_adaption_adjust2()
{
    if (state->flags.sport_flag && abs(state->current_av.yaw) > 0.2f)
    {
        if (state->current_av.yaw == 0)
        {
            return 0;
        }
        float out;
        if (state->current_speed.y >= 0)
        {
            out = -user->turn_adaption_kp2 * (state->current_av.yaw / abs(state->current_av.yaw)) * powf((mp[RIGHT_JOINT]->current_joint.pendulum.length + mp[LEFT_JOINT]->current_joint.pendulum.length), 2) * state->centripetal_force_scale;
        }
        else if (state->current_speed.y < 0)
        {
            out = user->turn_adaption_kp2 * (state->current_av.yaw / abs(state->current_av.yaw)) * powf((mp[RIGHT_JOINT]->current_joint.pendulum.length + mp[LEFT_JOINT]->current_joint.pendulum.length), 2) * state->centripetal_force_scale;
        }
        return out;
    }
    else
    {
        return 0;
    }
}

/**
 * @brief  车体转向内倾3
 * @note    用roll倾角拟合增益
 * @param
 * @return  两轮杆方向力矩差分
 * @retval  None
 */
float Manipulator_Controller_Classdef::turn_adaption_adjust3()
{
    // if (state->flags.sport_flag && abs(state->current_av.yaw) > 0.1f)
    // {
    // return -(current_pos.roll- target_pos.roll) * turn_adaption_kp3*(0.4 + abs(current_av.yaw) * 1.1 / 1.5) * powf((current_right_joint.pendulum.length + current_left_joint.pendulum.length), 2) * (0.7 + abs(current_speed.y) * 0.8 / 3.);
    return -(state->current_pos.roll - state->target_pos.roll) * user->turn_adaption_kp3 /* * (mp[RIGHT_JOINT]->current_joint.pendulum.length + mp[LEFT_JOINT]->current_joint.pendulum.length)*/;
    // }
    // else
    // {
    //     return 0;
    // }
}

/**
 * @brief  车体roll轴平衡
 * @note
 * @param
 * @return  高度双腿差分值
 * @retval  None
 */
float Manipulator_Controller_Classdef::roll_keep_adjust()
{
    j_roll_keep_pid->kp = user->roll_keep_kp;
    j_roll_keep_pid->ki = user->roll_keep_ki;
    j_roll_keep_pid->kd = user->roll_keep_kd;
    j_roll_keep_pid->I_Term_Max = user->roll_keep_imax;
    j_roll_keep_pid->Output_Max = user->roll_keep_omax;
    j_roll_keep_pid->target = state->target_pos.roll;
    j_roll_keep_pid->current = state->current_pos.roll;
    j_roll_keep_pid->Adjust(0, state->current_av.roll);
    return j_roll_keep_pid->out;
}

/**
 * @brief  轮腿高度自适应
 * @note
 * @param
 * @return  得出某个轮腿在y方向上的力
 * @retval  None
 */
float Manipulator_Controller_Classdef::length_keep_adjust(Manipulator_Classdef *_mp, uint8_t leg_side)
{
    float length_keep_feedforward = -user->length_ff_gain * mp[RIGHT_JOINT]->body.mass * 9.8f;

    if (state->flags.kd_inhibition)
    {
        j_length_keep_pid[leg_side]->kp = 1440;
        j_length_keep_pid[leg_side]->ki = 0;
        j_length_keep_pid[leg_side]->kd = 40;
    }
    else if (state->flags.weightlessness)
    {
        j_length_keep_pid[leg_side]->kp = user->length_keep_kp + 200.f;
        j_length_keep_pid[leg_side]->ki = user->length_keep_ki;
        j_length_keep_pid[leg_side]->kd = user->length_keep_kd + 15.f;
    }
    else
    {
        j_length_keep_pid[leg_side]->kp = user->length_keep_kp;
        j_length_keep_pid[leg_side]->ki = user->length_keep_ki;
        j_length_keep_pid[leg_side]->kd = user->length_keep_kd;
    }

    j_length_keep_pid[leg_side]->target = -upper::constrain(_mp->target_joint.pendulum.length, 0.13, 0.4);
    j_length_keep_pid[leg_side]->current = -_mp->current_joint.pendulum.length;
    j_length_keep_pid[leg_side]->Adjust(0, _mp->current_joint.pendulum.dlength);
    return length_keep_feedforward + j_length_keep_pid[leg_side]->out;
}

/**
 * @brief  关节转向环
 * @note    用于避免两腿开叉
 * @param
 * @return  末端力
 * @retval  None
 */
float Manipulator_Controller_Classdef::joint_turn_adjust(Manipulator_Classdef *_mp_right, Manipulator_Classdef *_mp_left)
{
    j_turn_pid->target = state->target_av.yaw;
    j_turn_pid->current = state->current_av.yaw;
    j_turn_pid->Adjust(0);
    j_follow_pid->target = _mp_right->current_joint.pendulum.angle;
    j_follow_pid->current = _mp_left->current_joint.pendulum.angle;
    j_follow_pid->Adjust(0);
    return j_follow_pid->out - j_turn_pid->out;
}

/**
 * @brief  腿杆上坡前馈
 * @note    避免趴坡(以腿杆摆角区分极性，目的是让腿杆有保持中置的趋势)
 * @param
 * @return  末端力
 * @retval  None
 */
float Manipulator_Controller_Classdef::slope_keep_feedforward(Manipulator_Classdef *_mp)
{
    if (state->target_speed.y > 0.3f)
    {
        return _mp->body.mass * cosf(_mp->current_joint.pendulum.angle) * sinf(abs(user->slope_angle)) * user->slope_ff_gain;
    }
    else if (state->target_speed.y < -0.3f)
    {
        return -_mp->body.mass * cosf(_mp->current_joint.pendulum.angle) * sinf(abs(user->slope_angle)) * user->slope_ff_gain;
    }
    else
    {
        return 0;
    }
}

/*******************************功能动作函数**************************************/
/**
 * @brief  车体弹跳函数
 * @note
 * @param
 * @return  末端力
 * @retval  None
 */
void Manipulator_Controller_Classdef::jump_adjust()
{
    static int count = 0;
    static bool is_build = false; // 蓄力标志位
    if (state->flags.jump_flag)
    {
        count += 480;
        if (mp[RIGHT_JOINT]->target_joint.pendulum.length > 0.16f || mp[LEFT_JOINT]->target_joint.pendulum.length > 0.16f)
        {
            is_build = true;
        }
        else
        {
            is_build = false;
        }
        state->flags.jump_flag = false;
    }
    else
    {
        count--;
    }
    count = upper::constrain(count, 0, 960);
    if (is_build)
    {
        if (count % 480 >= 320)
        {
            mp[RIGHT_JOINT]->target_joint.pendulum.length = 0.15f;
            mp[LEFT_JOINT]->target_joint.pendulum.length = 0.15f;
            state->flags.kd_inhibition = false;
        }
        else if (count % 480 < 320 && count % 480 >= 240)
        {
            mp[RIGHT_JOINT]->target_joint.pendulum.length = 0.4f;
            mp[LEFT_JOINT]->target_joint.pendulum.length = 0.4f;
            state->flags.kd_inhibition = true;
        }
        else if (count % 480 < 240 && count % 480 >= 120)
        {
            mp[RIGHT_JOINT]->target_joint.pendulum.length = 0.15f;
            mp[LEFT_JOINT]->target_joint.pendulum.length = 0.15f;
            state->flags.kd_inhibition = true;
        }
        else
        {
            state->flags.kd_inhibition = false;
        }
        if (count % 480 >= 120)
        {
            state->flags.is_jumping = true;
        }
        else
        {
            state->flags.is_jumping = false;
        }
    }
    else
    {
        if (count % 480 >= 400)
        {
            mp[RIGHT_JOINT]->target_joint.pendulum.length = 0.4f;
            mp[LEFT_JOINT]->target_joint.pendulum.length = 0.4f;
            state->flags.kd_inhibition = true;
        }
        else if (count % 480 < 400 && count % 480 >= 280)
        {
            mp[RIGHT_JOINT]->target_joint.pendulum.length = 0.15f;
            mp[LEFT_JOINT]->target_joint.pendulum.length = 0.15f;
            state->flags.kd_inhibition = true;
        }
        else
        {
            state->flags.kd_inhibition = false;
        }
        if (count % 480 >= 280)
        {
            state->flags.is_jumping = true;
        }
        else
        {
            state->flags.is_jumping = false;
        }
    }
    if (count <= 0)
    {
        is_build = false;
    }
}
