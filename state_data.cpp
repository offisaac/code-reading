#include "state_data.h"
#include "SRML.h"

#define DISTANCE_MAX 1.f

/******************************* timer *******************************/
getSystemTick state_timer::Get_SystemTick = NULL; //静态变量必须实现

uint8_t state_timer::UpdataTimeStamp(void)
{
    uint32_t now_time;

    /*Check `Get_SystemTick` */
    if (state_timer::Get_SystemTick != NULL)
    {
        /*Convert to system time*/
        if (last_time == 0)
        {
            last_time = state_timer::Get_SystemTick();
            return 1; //第一步先不跑
        }
        now_time = state_timer::Get_SystemTick();

        /*Overflow*/
        if (now_time < last_time)
            dt = (float)(now_time + (0xFFFFFFFF - last_time));
        else
            dt = (float)(now_time - last_time);

        last_time = now_time;

        dt *= (float)0.000001;

        return 0;
    }
    else
    {
        dt = 0.001f; //如果没有则默认1ms避免出现错误
        return 1;
    }
}

/**
 * @brief  Regist get time function(1Tick = 1us)
 * @param  realTime_fun: Pointer of function to get system real time
 * @retval 1: success
           0: error input param
 * @author
 */
uint8_t state_timer::getMicroTick_regist(uint32_t (*getTick_fun)(void))
{
    if (getTick_fun != NULL)
    {
        state_timer::Get_SystemTick = getTick_fun;
        return 1;
    }
    else
        return 0;
}

/*********************************** state ***************************************/
State_Data_Classdef::State_Data_Classdef()
{
}

/*时间栈更新*/
//如果输入参数则可以自定义控制周期
void State_Data_Classdef::timeStamp_update(float _dt)
{
    if (_dt == 0)
    {
        this->UpdataTimeStamp();
    }
    else
    {
        this->dt = _dt;
    }
}

/*数据更新*/
void State_Data_Classdef::target_update(float target_speed_, float target_turn_)
{
    target_speed.y = target_speed_;
    raw_target_speed.y = target_speed_;
    target_av.yaw = target_turn_;
}

void State_Data_Classdef::current_location_update(float x, float y, float z)
{
    current_location.x = x;
    current_location.y = y;
    current_location.z = z;
}

void State_Data_Classdef::current_speed_update(float x, float y, float z)
{
    current_speed.x = x;
    current_speed.y = y;
    current_speed.z = z;
}

void State_Data_Classdef::current_acc_update(float x, float y, float z)
{
    static MeanFilter<20> xAccF;
    static MeanFilter<20> yAccF;
    static MeanFilter<20> zAccF;
    current_acc.x = xAccF.f(x);
    current_acc.y = yAccF.f(y);
    current_acc.z = zAccF.f(z);
}

void State_Data_Classdef::current_av_update(float pitch, float yaw, float roll)
{
    current_av.pitch = pitch;
    current_av.yaw = yaw;
    current_av.roll = roll;
}

void State_Data_Classdef::current_pos_update(float pitch, float yaw, float roll)
{
    current_pos.pitch = pitch;
    current_pos.yaw = yaw;
    current_pos.roll = roll;
}

void State_Data_Classdef::power_limit_update(float _scale)
{
    power_scale = _scale;
}

void State_Data_Classdef::centripetal_force_update(float _force)
{
    centripetal_force_scale = _force;
}

/*状态更新函数*/
void State_Data_Classdef::sport_adaption()
{
    //先进行运动状态判断
    if (fabsf(current_speed.y) > 0.4f || fabsf(target_speed.y) > 0.2f)
    {
        flags.sport_flag = true;
        target_location.y = current_location.y;
        if (fabsf(raw_target_speed.y) < 0.1f * fabsf(current_speed.y))
        {
            flags.break_flag = true;
        }
        else
        {
            flags.break_flag = false;
        }
    }
    else
    {
        flags.sport_flag = false;
        flags.break_flag = false;
        if (fabsf(current_speed.y) < 0.6f && fabsf(target_speed.y) <= 0.05f)
        {
            flags.distance_flag = true;
        }
        else
        {
            flags.distance_flag = false;
        }
        target_location.y = upper::constrain(target_location.y, current_location.y - DISTANCE_MAX, current_location.y + DISTANCE_MAX);
    }

    /*路程环通断检测*/
    static int distance_delay = 80;
    static int distance_count = 0; //目标距离设置计数
    static bool distance_enable = false;
    if (flags.distance_flag == true)
    {                                 //每一次触发路程环都只设置一次目标值，延迟可以调
        if (distance_enable == false) //如果距离环被使能了，就不再更新距离目标值
        {
            distance_count++;
            if (distance_count == distance_delay)
            {
                distance_enable = true;
            }
            else
            {
            }
            target_location.y = current_location.y;
        }
        else
        {
        }
    }
    else
    {
        target_location.y = current_location.y;
        distance_count = 0;
        distance_enable = false;
    }

    if (chassis_state.rotation_state)
    {
        target_location.y = current_location.y;
        distance_count = 0;
        distance_enable = false;
    }

    if (flags.sport_flag && !flags.break_flag)
    {
        flags.set_point_flag = false;
    }
    else
    {
        flags.set_point_flag = true;
    }
    if (flags.weightlessness || flags.tiny_weightlessness)
    {
        flags.set_point_flag = false;
    }
}

/**
 * @brief  腾空检测
 * @note
 * @param
 * @return
 * @retval  None
 */
void State_Data_Classdef::weightlessness_check()
{
    static int16_t weightless_count = 0; //用于计数腾空时间
    // if (mp[RIGHT_JOINT]->current_joint.support_force < 30.f && mp[LEFT_JOINT]->current_joint.support_force < 30.f)
    // {
    //     flags.weightlessness = true;
    // }
    // else if(mp[RIGHT_JOINT]->current_joint.support_force > 55.f && mp[LEFT_JOINT]->current_joint.support_force > 55.f)
    // {
    //     flags.weightlessness = false;
    // }

    if (mp[RIGHT_JOINT]->current_joint.support_force < 30.f && mp[LEFT_JOINT]->current_joint.support_force < 30.f)
    {
        weightless_count += 8;
    }
    else if(mp[RIGHT_JOINT]->current_joint.support_force > 55.f && mp[LEFT_JOINT]->current_joint.support_force > 55.f)
    {
        weightless_count -= 8;
    }
    weightless_count = upper::constrain(weightless_count, 0, 1000);
    if (weightless_count > 120)
    {
        flags.weightlessness = true;
    }
    else
    {
        flags.weightlessness = false;
    }

    
}

/**
 * @brief  失重检测(小幅腾空)
 * @note
 * @param
 * @return
 * @retval  None
 */
void State_Data_Classdef::tiny_weightlessness_check()
{
    static int16_t weightless_count = 0; //用于计数腾空时间
    if (mp[RIGHT_JOINT]->current_joint.support_force < 30.f && mp[LEFT_JOINT]->current_joint.support_force < 30.f)
    {
        flags.tiny_weightlessness = true;
    }
    else if(mp[RIGHT_JOINT]->current_joint.support_force > 55.f && mp[LEFT_JOINT]->current_joint.support_force > 55.f)
    {
        flags.tiny_weightlessness = false;
    }

    // if (current_acc.z  / cos(fabsf(current_pos.pitch)) < -0.4f)
    // if (mp[RIGHT_JOINT]->current_joint.support_force < 20.f || mp[LEFT_JOINT]->current_joint.support_force < 20.f)
    // {
    //     weightless_count += 4;
    // }
    // else
    // {
    //     weightless_count -= 16;
    // }
    // weightless_count = upper::constrain(weightless_count, 0, 1000);
    // if (weightless_count > 70)
    // {
    //     flags.tiny_weightlessness = true;
    // }
    // else
    // {
    //     flags.tiny_weightlessness = false;
    // }
}

void State_Data_Classdef::gimbal_state_update(chassis_state_structdef _data)
{
    chassis_state = _data;
}
