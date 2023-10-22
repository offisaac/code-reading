#include "Balance_Chassis.h"
#include "internal.h"

//状态机逻辑实现变量
uint16_t Land_Delay = 200; //倒地自救延时
bool is_reset = 0;         //底盘重置

Lost_Ctrl_State lostctrl_state;
Pre_Balance_State prebalance_state;
Balance_State balance_state;

void Lost_Ctrl_State::State_Handler()
{
	context->machine_mode = 0;
    Land_Delay = 200; //计数器重置
    is_reset = 1;
    context->Reset_Adjust();
}

//预平衡状态只在程序重新开始的时候生效
void Pre_Balance_State::State_Handler()
{
	context->machine_mode = 1;

    if (absChassis.getCtrlData().enable_cmd)
    {
        context->Status_Switching(&balance_state); //底盘使能，进入平衡状态
    }
}

void Balance_State::State_Handler()
{
	context->machine_mode = 2;
    is_reset = 0;
    /* 底盘电机控制 */
    context->Chassis_Ctrl_Cal();
    /*进入软件复位*/
    if (absChassis.getCtrlData().gg_flag)
    {
        __set_FAULTMASK(1); //关闭所有中断
        NVIC_SystemReset();
    }
}

/**
 * @brief  构造函数
 * @note
 * @param
 * @return
 * @retval  None
 */
Balance_Infantry_Classdef::Balance_Infantry_Classdef()
{
    current_state = &lostctrl_state;
}

/**
 * @brief  状态切换判断函数
 * @note
 * @param
 * @return
 * @retval  None
 */
void Balance_Infantry_Classdef::Judge_State()
{
    if (!absChassis.getCtrlData().remote_ctrl_state)
    {
        Status_Switching(&lostctrl_state);
    }
    else if (current_state == &lostctrl_state)
    {
        Status_Switching(&prebalance_state);
    }
    else
    {
    }

    balance_controller.Update_Flags(absChassis.getCtrlData().turn90degrees, absChassis.getCtrlData().rotation_state, is_reset, absChassis.getCtrlData().leap_state, absChassis.getCtrlData().unlimited_state, absChassis.getCtrlData().ascent_state); //更新控制器标志位

    current_state->State_Handler();
}

/**
 * @brief  更新底盘运动目标值
 * @note
 * @param
 * @return
 * @retval  None
 */
void Balance_Infantry_Classdef::Update_Target(float _y_speed, float _z_speed, float _x_speed)
{
    float target_rotation_angle = 0;  //速度向量旋转角
    float target_rotation_r;      //目标速度模长
    float fact_target_speed;

    target_rotation_r = sqrtf(powf(_y_speed, 2) + powf(_x_speed, 2)); //计算平移矢量模长
    if (_y_speed == 0 && _x_speed > 0)
    {
        target_rotation_angle = 90. / 180. * PI; //计算平移矢量角度
    }
    else if (_y_speed == 0 && _x_speed < 0)
    {
        target_rotation_angle = -90. / 180. * PI; //计算平移矢量角度
    }
    else if (_y_speed < 0 && _x_speed > 0)
    {
        target_rotation_angle = atanf(_x_speed / _y_speed) + PI;
    }
    else if (_y_speed < 0 && _x_speed < 0)
    {
        target_rotation_angle = atanf(_x_speed / _y_speed) - PI;
    }
    else if (_y_speed == 0 && _x_speed == 0)
    {
        target_rotation_angle = 0;
    }
    else if (_x_speed == 0 && _y_speed > 0)
    {
        target_rotation_angle = 0;
    }
    else if (_x_speed == 0 && _y_speed < 0)
    {
        target_rotation_angle = -PI;
    }
    else
    {
        target_rotation_angle = atanf(_x_speed / (float)_y_speed); //计算平移矢量角度
    }

    if (absChassis.getCtrlData().rotation_state)
    {
        target_rotation_angle -= (0.25f * PI * (-balance_controller.current_angularSpeed.yaw - 2 * PI) / (2 * PI) + 0.5f * PI); //根据小陀螺转速进行速度向量相位补偿
    }

    float angle_error = rotation_chassis_angle - target_rotation_angle; //得出速度向量与底盘坐标系夹角
    fact_target_speed = target_rotation_r * cosf(angle_error);

    if (absChassis.getCtrlData().rotation_state)
    {
        balance_controller.Update_Target_LinearSpeed(0, fact_target_speed * rotation_move_gain, 0);
        balance_controller.Update_Target_AngularSpeed(_z_speed, 0, 0);
        balance_controller.down_slope_flag = false;
    }
    else
    {
        balance_controller.Update_Target_LinearSpeed(0, fact_target_speed, 0);
        balance_controller.Update_Target_AngularSpeed(_z_speed, 0, 0);
        if (absChassis.getCtrlData().ascent_state)
        {
            balance_controller.down_slope_flag = true;
        }
        else
        {
            balance_controller.down_slope_flag = false;
        }
    }
}

/**
 * @brief  更新底盘当前位姿
 * @note
 * @param
 * @return
 * @retval  None
 */
void Balance_Infantry_Classdef::Update_Current_Pos(float _yaw, float _pitch, float _chassis_angle)
{
    balance_controller.Update_Current_Pos(_yaw, _pitch, 0);
    rotation_chassis_angle = _chassis_angle / 180.f * PI;
}

/**
 * @brief  更新底盘当前速度
 * @note
 * @param
 * @return
 * @retval  None
 */
void Balance_Infantry_Classdef::Update_Current_Speed(float _y, float _yaw, float _pitch)
{
    static float distance = 0.0f;
    static uint32_t time_gap = 0;
    static uint32_t current_time = 0;
    static uint32_t last_time = 0;
    current_time = Get_SystemTimer();
    time_gap = current_time - last_time;

    distance += (_y + _pitch * absChassis.wheelRadius) * CTRL_INTERAL;
    balance_controller.Update_Current_LinearSpeed(0, (_y + _pitch * absChassis.wheelRadius), 0); //当前线速度需要减去因车体旋转造成的误差
    balance_controller.Update_Current_AngularSpeed(_yaw, _pitch, 0);
    if (time_gap && current_time && last_time)
    {
        balance_controller.Update_Current_Location(0, distance, 0); //更新走过的距离
    }
    else
    {
    }
    last_time = time_gap;
}

/**
 * @brief  更新底盘当前线性加速度
 * @note
 * @param
 * @return
 * @retval  None
 */
void Balance_Infantry_Classdef::Update_Current_Acc(float _x, float _y, float _z)
{
    static MeanFilter<20> xAccF;
    static MeanFilter<20> yAccF;
    static MeanFilter<20> zAccF;
    balance_controller.Update_Current_LinearAcc(xAccF.f(_x), yAccF.f(_y), zAccF.f(_z));
}

/**
 * @brief  更新电机输出电流
 * @note
 * @param
 * @return
 * @retval  None
 */
void Balance_Infantry_Classdef::Update_Motor_Current(float _current)
{
    balance_controller.motor_current = _current;
}

/**
 * @brief  更新滑块位置和转速
 * @note
 * @param
 * @return
 * @retval  None
 */
void Balance_Infantry_Classdef::Update_Slider_Params(float _s[2], float _sspeed[2])
{
    for (int i = 0; i < 2; i++)
    {
        balance_controller.current_sliderLocation[i].y = _s[i] / 360.f * 2.f * PI * 0.05f;
        balance_controller.current_sliderSpeed[i].y = _sspeed[i] / 60.f * 2.f * PI * 0.05f;
    }
}

/**
 * @brief  控制函数
 * @note
 * @param
 * @return
 * @retval  None
 */
void Balance_Infantry_Classdef::Chassis_Ctrl_Cal()
{
    static MeanFilter<20> turn_mf;
    static LowPassFilter turn_lf(0.5);
    float slider_s[2];
    float slider_sspeed[2];
    slider_s[LEFT] = Slider_Ctrl.slider[LEFT].getCurrentPos();
    slider_s[RIGHT] = Slider_Ctrl.slider[RIGHT].getCurrentPos();
    slider_sspeed[LEFT] = Slider_Ctrl.slider[LEFT].getCurrentSpeed();
    slider_sspeed[RIGHT] = Slider_Ctrl.slider[RIGHT].getCurrentSpeed();

    /*更新目标值*/
    Update_Target(absChassis.getCtrlData().target_speed_y, -absChassis.getCtrlData().target_speed_z, absChassis.getCtrlData().target_speed_x);
    /*更新当前位姿*/
    Update_Current_Pos(absChassis.absIMU.eularData.yaw, absChassis.absIMU.eularData.pitch, absChassis.getCtrlData().chassis_rotation_angle);
    /*更新当前速度*/
    Update_Current_Speed(absChassis.getLinerSpeed(), absChassis.absIMU.gyroData.yaw, absChassis.absIMU.gyroData.pitch);
    /*更新当前线性加速度*/
    Update_Current_Acc(absChassis.absIMU.accData.x, absChassis.absIMU.accData.y, absChassis.absIMU.accData.z);
    /*更新当前滑块状态*/
    Update_Slider_Params(slider_s, slider_sspeed);
    /*更新当前电机电流值*/
    Update_Motor_Current(absChassis.getChassisPower());
    /*底盘控制*/
    balance_controller.Controller_Adjust();

    /* 输出合并，其中直立环和速度环互为反号 */
    wheel_stand_out_theory[LEFT] = balance_controller.Get_Data().stand_out + balance_controller.Get_Data().feedforward_out + balance_controller.Get_Data().distance_out + balance_controller.Get_Data().slider_out; //作为最后输出的一部分，直接转换成整数
    wheel_stand_out_theory[RIGHT] = balance_controller.Get_Data().stand_out + balance_controller.Get_Data().feedforward_out + balance_controller.Get_Data().distance_out + balance_controller.Get_Data().slider_out;

    wheel_speed_out_theory[LEFT] = balance_controller.Get_Data().speed_out - balance_controller.Get_Data().turn_out; //还需要做功率控制，先不转换类型
    wheel_speed_out_theory[RIGHT] = balance_controller.Get_Data().speed_out + balance_controller.Get_Data().turn_out;
}

/**
 * @brief  控制量下发
 * @note
 * @param
 * @return
 * @retval  None
 */
void Balance_Infantry_Classdef::Chassis_Adjust()
{
    /* 需要修改  高速时转向环响应不够 */
    if (fabsf(absChassis.absIMU.eularData.pitch) < 45.0f)
    {
        /* 固连自救，取消直立环作用，只利用速度环	自救 */
        if (absChassis.getCtrlData().self_rescue_state)
        {
            wheel_stand_out_theory[LEFT] = 0;
            wheel_stand_out_theory[RIGHT] = 0;
            wheel_out[LEFT] = balance_controller.target_linearSpeed.y - balance_controller.Get_Data().turn_out;
            wheel_out[RIGHT] = balance_controller.target_linearSpeed.y + balance_controller.Get_Data().turn_out;
        }
        else
        {

            wheel_out[LEFT] = wheel_stand_out_theory[LEFT] + balance_controller.Get_Data().speed_out - balance_controller.Get_Data().turn_out;
            wheel_out[RIGHT] = wheel_stand_out_theory[RIGHT] + balance_controller.Get_Data().speed_out + balance_controller.Get_Data().turn_out;
        }
    }
    else
    {
        wheel_out[LEFT] = 0;
        wheel_out[RIGHT] = 0;
    }

    if (balance_controller.weightless_flag == true)
    {
    }
    else
    {
        if (balance_controller.idling_flag == true)
        {
            wheel_out[LEFT] = 0;
            wheel_out[RIGHT] = 0;
        }
    }

    wheel_out[LEFT] = std_lib::constrain(wheel_out[LEFT], -4.2f, 4.2f);
    wheel_out[RIGHT] = std_lib::constrain(wheel_out[RIGHT], -4.2f, 4.2f);
    absChassis.controlWheelMotor(wheel_out);

    float rotation_slider_pos[2] = {-10, -10};
    if (absChassis.getCtrlData().rotation_state)
    {
        rotation_slider_pos[0] = -10;
        rotation_slider_pos[1] = -10;
    }
    else if (absChassis.getCtrlData().turn90degrees)
    {
        rotation_slider_pos[0] = -40;
        rotation_slider_pos[1] = -40;
    }
    if (absChassis.getCtrlData().remote_ctrl_state == false)
    {
        wheel_out[LEFT] = 0;
        wheel_out[RIGHT] = 0;
        Slider_Ctrl.clear();
    }
    else
    {
        if (!absChassis.getCtrlData().enable_cmd)
        {
            balance_controller.output.sliderCtrl_out[RIGHT] = 0;
            balance_controller.output.sliderCtrl_out[LEFT] = 0;
        }
        if (absChassis.getCtrlData().rotation_state || absChassis.getCtrlData().turn90degrees)
        {
            Slider_Ctrl.update(rotation_slider_pos);
            Slider_Ctrl.adjust();
            Slider_Ctrl.acutate();
        }
        else
        {
            // Slider_Ctrl.setTorqueOut(balance_controller.output.sliderCtrl_out);
            Slider_Ctrl.setVoltageOut(balance_controller.output.sliderCtrl_out);
        }
    }
}

/**
 * @brief  底盘重置响应
 * @note
 * @param
 * @return
 * @retval  None
 */
void Balance_Infantry_Classdef::Reset_Adjust()
{
    //输出清零
    wheel_stand_out_theory[LEFT] = 0;
    wheel_stand_out_theory[RIGHT] = 0;
    wheel_speed_out_theory[LEFT] = 0;
    wheel_speed_out_theory[RIGHT] = 0;
    //目标值清零
    Update_Target(0, 0, 0);
    Update_Current_Pos(0, 0, 0);
    Update_Current_Speed(0, 0, 0);
    //控制器输出重置
    balance_controller.reset_adjust();
    //滑块重置
    Slider_Ctrl.clear();
}
