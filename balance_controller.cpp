#include "balance_controller.h"
#include "math.h"

myPID Cap_Charge_Pid;
myPID Power_Limit_Pid;

float debug_out_A;
float debug_out_B;
float debug_out_C;
float debug_out_D;
float debug_out_E;
float debug_out_F;
float debug_out_G;
float debug_out_H;
float debug_out_I;

//更新数据函数
void Ctrl_Base::Update_Target_Location(float z, float y, float x)
{
    last_target_location.z = target_location.z;
    last_target_location.y = target_location.y;
    last_target_location.x = target_location.x;

    target_location.z = z;
    target_location.y = y;
    target_location.x = x;
}

void Ctrl_Base::Update_Current_Location(float z, float y, float x)
{
    last_current_location.z = current_location.z;
    last_current_location.y = current_location.y;
    last_current_location.x = current_location.x;

    current_location.z = z;
    current_location.y = y;
    current_location.x = x;
}

void Ctrl_Base::Update_Target_Pos(float yaw, float pitch, float roll)
{
    last_target_pos.yaw = target_pos.yaw;
    last_target_pos.pitch = target_pos.pitch;
    last_target_pos.roll = target_pos.roll;

    target_pos.yaw = yaw;
    target_pos.pitch = pitch;
    target_pos.roll = roll;
}
void Ctrl_Base::Update_Current_Pos(float yaw, float pitch, float roll)
{
    last_current_pos.yaw = current_pos.yaw;
    last_current_pos.pitch = current_pos.pitch;
    last_current_pos.roll = current_pos.roll;

    current_pos.yaw = yaw;
    current_pos.pitch = pitch;
    current_pos.roll = roll;
}
void Ctrl_Base::Update_Target_LinearSpeed(float z, float y, float x)
{
    last_target_linearSpeed.z = target_linearSpeed.z;
    last_target_linearSpeed.y = target_linearSpeed.y;
    last_target_linearSpeed.x = target_linearSpeed.x;

    target_linearSpeed.z = z;
    target_linearSpeed.y = y;
    target_linearSpeed.z = z;
}
void Ctrl_Base::Update_Current_LinearSpeed(float z, float y, float x)
{
    last_current_linearSpeed.z = current_linearSpeed.z;
    last_current_linearSpeed.y = current_linearSpeed.y;
    last_current_linearSpeed.x = current_linearSpeed.x;

    current_linearSpeed.z = z;
    current_linearSpeed.y = y;
    current_linearSpeed.z = z;
}
void Ctrl_Base::Update_Target_AngularSpeed(float yaw, float pitch, float roll)
{
    last_target_angularSpeed.yaw = target_angularSpeed.yaw;
    last_target_angularSpeed.pitch = target_angularSpeed.pitch;
    last_target_angularSpeed.roll = target_angularSpeed.roll;

    target_angularSpeed.yaw = yaw;
    target_angularSpeed.pitch = pitch;
    target_angularSpeed.roll = roll;
}
void Ctrl_Base::Update_Current_AngularSpeed(float yaw, float pitch, float roll)
{
    last_current_angularSpeed.yaw = current_angularSpeed.yaw;
    last_current_angularSpeed.pitch = current_angularSpeed.pitch;
    last_current_angularSpeed.roll = current_angularSpeed.roll;

    current_angularSpeed.yaw = yaw;
    current_angularSpeed.pitch = pitch;
    current_angularSpeed.roll = roll;
}
void Ctrl_Base::Update_Target_LinearAcc(float z, float y, float x)
{
    last_target_linearAcc.z = target_linearAcc.z;
    last_target_linearAcc.y = target_linearAcc.y;
    last_target_linearAcc.x = target_linearAcc.x;

    target_linearAcc.z = z;
    target_linearAcc.y = y;
    target_linearAcc.x = x;
}
void Ctrl_Base::Update_Current_LinearAcc(float z, float y, float x)
{
    last_current_linearAcc.z = current_linearAcc.z;
    last_current_linearAcc.y = current_linearAcc.y;
    last_current_linearAcc.x = current_linearAcc.x;

    current_linearAcc.z = z;
    current_linearAcc.y = y;
    current_linearAcc.x = x;
}
void Ctrl_Base::Update_Target_AngularAcc(float yaw, float pitch, float roll)
{
    last_target_angularAcc.yaw = target_angularAcc.yaw;
    last_target_angularAcc.pitch = target_angularAcc.pitch;
    last_target_angularAcc.roll = target_angularAcc.roll;

    target_angularAcc.yaw = yaw;
    target_angularAcc.pitch = pitch;
    target_angularAcc.roll = roll;
}
void Ctrl_Base::Update_Current_AngularAcc(float yaw, float pitch, float roll)
{
    last_current_angularAcc.yaw = current_angularAcc.yaw;
    last_current_angularAcc.pitch = current_angularAcc.pitch;
    last_current_angularAcc.roll = current_angularAcc.roll;

    current_angularAcc.yaw = yaw;
    current_angularAcc.pitch = pitch;
    current_angularAcc.roll = roll;
}
void Ctrl_Base::Update_Flags(uint8_t _is_turn90degrees, uint8_t _is_rotation, uint8_t _is_reset, uint8_t _is_leap, uint8_t _is_unlimited, uint8_t _is_slope)
{
    is_turn90degrees = _is_turn90degrees;
    is_rotation = _is_rotation;
    is_reset = _is_reset;
    is_leap = _is_leap;
    is_unlimited = _is_unlimited;
    is_slope = _is_slope;
}
//用于限制变量增长率
float Ctrl_Base::Value_Step(const float value, const float last_value, const float step, const float max, const float min)
{
    float value_out = 0.0f;
    if (value >= last_value)
    {
        value_out = (fabsf(value - last_value) > fabsf(step)) ? (last_value + fabsf(step)) : value;
    }
    else
    {
        value_out = (fabsf(value - last_value) > fabsf(step)) ? (last_value - fabsf(step)) : value;
    }

    value_out = std_lib::constrain(value_out, min, max);

    return value_out;
}

/********************************************************pid控制器******************************************************/
/**
 * @brief  pid控制器构造函数
 * @note
 * @param
 * @return
 * @retval  None
 */
Controller<PID>::Controller() : fuzzy_stand_pd(-40, 40, -500, 500, 100, 650, 0, 0, 50, 110), fuzzy_speed_pi(-1000 / 435.71f, 1000 / 435.71f, -500 / 435.71f, 500 / 435.71f, 5 * 435.71f, 8 * 435.71f, 0, 0, 0.0, 0.0)
{
    set_point_pid.SetPIDParam(0.035f * 435.71f, 0.040f * 435.71f, 0.0f, 3.0f, 10.0f); // 0.045		3.0

    stand_pid.SetPIDParam(220.0f, 0, 55.0f, 0, 13.0f / 20.0f * 16384 * 0.7f);

    turn_pid.SetPIDParam(70.0f, 0, 0, 0, 13.0f / 20.0f * 16384);

    speed_pid.SetPIDParam(6.0f, 0.0f, 0.0f, 0, 13.0f / 20.0f * 16384 * 0.7f);

    fuzzy_stand_pd.Load_FuzzyRule(stand_kp_rule, NULL, stand_kd_rule); //加载直立环模糊pd规则表

    fuzzy_speed_pi.Load_FuzzyRule(speed_kp_rule, NULL, NULL);

    set_point = 0;

    speed_pid_cnt = 0;

    params.motor_max_output = 13.0f / 20.0f * 16384;

    params.max_linear_target = 1023.0f;

    params.task_run_interval = 0.005f;

    params.max_launch_acceleration = 6666.0f;

    params.max_normal_acceleration = 9999.0f;

    params.max_brake_acceleration = 23333.0f;

    params.launch_speed = 9000.0f;

    params.wheel_max_speed = 9000.0f;
}

/**
 * @brief  pid控制器结算
 * @note
 * @param
 * @return
 * @retval  None
 */
void Controller<PID>::Controller_Adjust()
{

    if (is_reset)
    {
        reset_adjust();
        return;
    }
    else
    {
        torque_optimization();

        output.set_point_out = self_adaption();
        debug_out_A = output.set_point_out; // setpoint输出
        // output.set_point_out=0;

        target_pos.pitch = output.set_point_out; //自适应的输出设置为目标角度
        output.stand_out = stand_adjust();
        debug_out_D = output.stand_out; //直立环输出

        output.speed_out = speed_adjust();
        debug_out_E = output.speed_out; //速度环输出
        // output.speed_out = 0;

        output.turn_out = turn_adjust();
        debug_out_F = output.turn_out; //转向环输出
        // output.turn_out=0;

        output.feedforward_out = stand_feedforward();
        // output.feedforward_out=0;
    }
}

/**
 * @brief  力矩优化
 * @note
 * @param
 * @return
 * @retval  None
 */
void Controller<PID>::torque_optimization()
{
    float max_accceleration = 0.0f;
    float dspeed = 0.0f;
    if (fabsf(target_linearSpeed.y) > fabsf(last_target_linearSpeed.y))
    { // accelerate
        if (fabsf(target_linearSpeed.y) <= params.launch_speed)
        {
            max_accceleration = params.max_launch_acceleration;
        }
        else
        {
            max_accceleration = params.max_normal_acceleration;
        }
    }
    else // decelerate
    {
        max_accceleration = params.max_brake_acceleration;
    }
    dspeed = max_accceleration * ((float)params.task_run_interval);

    target_linearSpeed.y = Value_Step(target_linearSpeed.y, last_target_linearSpeed.y, dspeed, params.max_linear_target, -params.max_linear_target);
}

/**
 * @brief  pid自适应环
 * @note
 * @param
 * @return
 * @retval  None
 */
float Controller<PID>::self_adaption()
{ //自适应环
    static bool setpoint_adaption_flag = false;
    float setpoint_adapt_out = 0.0f;
    float setpoint_ctrl_out = 0.0f;
    // debug_out_B=current_linearSpeed.y;
    if (is_rotation || fabsf(current_angularSpeed.yaw) > 100.0f)
    {
        //        set_point_pid.I_Term = 0.0f;
        fuzzy_stand_pd.params.kp_params[0] = 500.0f; //防止小陀螺坐船和转弯车体晃动
        fuzzy_stand_pd.params.kd_params[0] = 100.0f;
        return -2.0f;
    }
    else
    {
        fuzzy_stand_pd.params.kp_params[0] = 100.0f;
        fuzzy_stand_pd.params.kd_params[0] = 50.0f;
    }
    if (target_linearSpeed.y == 0 && setpoint_adaption_flag)
    {
        set_point_pid.Target = 0; //正常状态下以0为目标值
        set_point_pid.Current = current_linearSpeed.y;
        set_point_pid.Adjust();
        debug_out_G = set_point_pid.P_Term;
        debug_out_H = set_point_pid.I_Term;
        if (is_turn90degrees)
        {
            setpoint_adapt_out = -0.5f - set_point_pid.Out;
        }
        else
        {
            setpoint_adapt_out = -1.5f - set_point_pid.Out;
        }
    }

    /*加速后制动一段时间才进入零点自适应*/
    if (target_linearSpeed.y != 0)
    {
        setpoint_adaption_flag = false;
    }
    else
    {
        setpoint_adaption_flag = true;
    }
    //    else if (current_linearSpeed.y < 500.0f && current_linearSpeed.y > -500.0f)
    //    {
    //        setpoint_adaption_flag = true;
    //    }

    if (target_linearSpeed.y < -200)
    {
        setpoint_ctrl_out = 2.0f;
    }
    /* 根据操作手需求使用 */
    else if (target_linearSpeed.y > 200)
    {
        setpoint_ctrl_out = -1.5f;
    }
    /* 刹车 */
    else if (fabsf(current_linearSpeed.y) > 500.0f && fabsf(target_linearSpeed.y) < 500)
    {
        setpoint_adaption_flag = true;
    }

    if (setpoint_adaption_flag)
    {
        return std_lib::constrain(setpoint_adapt_out, -10.0f, 10.0f); //自适应输出
    }
    else
    {
        //        set_point_pid.I_Term = 0.0f;
        return std_lib::constrain(setpoint_ctrl_out, -10.0f, 10.0f); //定点输出
    }
}

/**
 * @brief  pid直立环
 * @note
 * @param
 * @return
 * @retval  None
 */
float Controller<PID>::stand_adjust()
{ //直立环
    /* 模糊PD直立环计算 */
    float error = target_pos.pitch - current_pos.pitch;
    fuzzy_stand_pd.Fuzzy_Adjust(error, current_angularSpeed.pitch);
    stand_pid.Kp = fuzzy_stand_pd.Kp_out;
    stand_pid.Kd = fuzzy_stand_pd.Kd_out;
    debug_out_B = target_pos.pitch;
    debug_out_C = current_pos.pitch;
    //    debug_out_G = stand_pid.Kp * error;
    //    debug_out_H = -stand_pid.Kd * current_angularSpeed.pitch;
    return stand_pid.Kp * error - stand_pid.Kd * current_angularSpeed.pitch;
}

/**
 * @brief  pid转向环
 * @note
 * @param
 * @return
 * @retval  None
 */
float Controller<PID>::turn_adjust()
{ //转向环
    /* 转向环 */
    turn_pid.Current = current_angularSpeed.yaw;
    debug_out_I = turn_pid.Current;
    turn_pid.Target = target_angularSpeed.yaw;
    return turn_pid.Adjust();
}

/**
 * @brief  pid速度环
 * @note
 * @param
 * @return
 * @retval  None
 */
float Controller<PID>::speed_adjust()
{ //速度环
    static LowPassFilter speed_filter(0.8);
    //防止刹车倾角过大
    if (fabsf(target_linearSpeed.y) < 30.0f)
    {
        speed_pid.Out_Max = 5000.0f; //直接限制速度环输出
    }
    else
    {
        speed_pid.Out_Max = 10000.0f;
    }
    /* 速度环 */
    if (speed_pid_cnt)
        speed_pid_cnt--;
    else
    {
        speed_pid_cnt = speed_pid_delay;

        speed_pid.Target = target_linearSpeed.y;   //速度环就是对速度做响应的环路，用速度环的输出自然能够扰动直立环
        speed_pid.Current = current_linearSpeed.y; //速度环要做速度积分，采用myPID进行结算
        speed_filter << speed_pid.Current;
        speed_filter >> speed_pid.Current;
        /*速度滤波*/
        float error_speed = speed_pid.Target - speed_pid.Current;
        fuzzy_speed_pi.Fuzzy_Adjust(error_speed, error_speed);
        speed_pid.Kp = fuzzy_speed_pi.Kp_out; //一阶模糊，只模糊Kp
    }
    return speed_pid.Adjust();
}

/**
 * @brief  pid前馈
 * @note
 * @param
 * @return
 * @retval  None
 */
float Controller<PID>::stand_feedforward()
{ //直立前馈
    float error = target_pos.pitch - current_pos.pitch;
    if (error > 0)
    {
        return feedforward_ratio * arm_sin_f32(fabsf(error) * 3.14f / 180.0f);
    }
    else
    {
        return -feedforward_ratio * arm_sin_f32(fabsf(error) * 3.14f / 180.0f);
    }
}

/**
 * @brief  pid重置
 * @note
 * @param
 * @return
 * @retval  None
 */
void Controller<PID>::reset_adjust()
{
    Update_Target_Location();
    Update_Current_Location();
    Update_Target_Pos();
    Update_Current_Pos();
    Update_Target_LinearSpeed();
    Update_Current_LinearSpeed();
    Update_Target_AngularSpeed();
    Update_Current_AngularSpeed();
    Update_Target_LinearAcc();
    Update_Current_LinearAcc();
    Update_Target_AngularAcc();
    Update_Current_AngularAcc();
    speed_pid_cnt = speed_pid_delay * 2.0f; //重置时给两倍的延时间隔

    set_point_pid.I_Term = 0.0f;
    stand_pid.I_Term = 0.0f;
    turn_pid.I_Term = 0.0f;
    speed_pid.I_Term = 0.0f;

    output = Controller_Out{}; //清零输出
}

/********************************************************lqr控制器******************************************************/
/**
 * @brief  lqr控制器构造函数
 * @note
 * @param
 * @return
 * @retval  None
 */
Controller<LQR>::Controller()
{
    set_point_pid.SetPIDParam(-10.0f, 0, 0.0f, 3.0f, 6.f);
    rotation_point_pid.SetPIDParam(0, 0.000001f, 0, 0.f, 3.f);
    // slider_follow_pid.SetPIDParam(1000.f, 0, 0.2f, 0, 20000.f);
    slider_follow_pid.SetPIDParam(-5.f, 0, 0.f, 0, 1.f);
}

/**
 * @brief  lqr控制结算
 * @note
 * @param
 * @return
 * @retval  None
 */
void Controller<LQR>::Controller_Adjust()
{
    if (is_reset)
    {
        reset_adjust();
        return;
    }
    else
    {
        // weightless_check();     //失重检测
        // idling_check();         //空转检测
        rotation_crash_check(); //小陀螺撞墙检测
                                /*setpoint倾角辅助*/
        slider_control();       //滑块控制
        output.set_point_out = self_adaption() * ratio_degree2rad;
        target_pos.pitch = output.set_point_out; //自适应的输出设置为目标角度
        /*直立环*/
        output.stand_out = torque_scale * stand_adjust();
        /*速度环*/
        output.speed_out = torque_scale * speed_adjust();
        if (break_flag)
        {
            output.speed_out = std_lib::constrain(output.speed_out, -3.f, 3.f);
        }
        else
        {
            output.speed_out = std_lib::constrain(output.speed_out, -3.0f, 3.0f);
        }
        /*距离环*/
        if (distance_flag && !is_rotation)
        {                                 //每一次触发路程环都只设置一次目标值，延迟可以调
            if (distance_enable == false) //如果距离环被使能了，就不再更新距离目标值
            {
                if (distance_count == distance_delay)
                {
                    distance_enable = true;
                }
                else
                {
                }
                target_location.y = current_location.y;
                distance_count++;
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
        output.distance_out = torque_scale * distance_adjust();
        /*转向环*/
        output.turn_out = turn_scale * turn_adjust();
        if (is_rotation == 0)
        {
            output.turn_out = std_lib::constrain(output.turn_out, -3.f, 3.f);
        }
        else
        {
            output.turn_out = std_lib::constrain(output.turn_out, -3.f, 3.f);
        }
        /*前馈*/
        output.feedforward_out = stand_feedforward();
        /*滑块环*/
        output.slider_out = slider_adjust();
    }
}

void Controller<LQR>::slider_control()
{
    static MeanFilter<30> distance_MF;   //距离滤波
    static MeanFilter<10> pitch_MF;      //直立滤波
    static MeanFilter<20> pitchSpeed_MF; //角速度滤波
    static MeanFilter<100> speed_MF1;    //速度滤波
    static MeanFilter<100> speed_MF2;
    static MedianFilter<50> speed_MIF1; //中值滤波
    static MedianFilter<50> speed_MIF2;
    static MeanFilter<20> turn_MF;
    static MeanFilter<10> s_MF[2];
    static MeanFilter<10> sspeed_MF[2];

    static SecondOrderButterworthLPF speed_lpf(10, 500);
    static SecondOrderButterworthLPF distance_lpf(10, 500);
    static SecondOrderButterworthLPF pitch_lpf(5, 500);
    static SecondOrderButterworthLPF pitchSpeed_lpf(5, 500);
    static SecondOrderButterworthLPF s_lpf[2] = {SecondOrderButterworthLPF(5, 500), SecondOrderButterworthLPF(5, 500)};
    static SecondOrderButterworthLPF sspeed_lpf[2] = {SecondOrderButterworthLPF(5, 500), SecondOrderButterworthLPF(5, 500)};

    static float last_speed_error = 0;

    /*distance*/
    float distance_error = distance_MF.f(target_location.y - current_location.y);
    /*speed*/
    float speed_error = std_lib::constrain(speed_lpf.f(target_linearSpeed.y - current_linearSpeed.y), -2.5f, 2.5f);
    //		if(fabsf(speed_error)<0.3f)
    //		{
    //			speed_error = 0;
    //		}
    // float speed_error = target_linearSpeed.y - speed_MIF1.f(this->current_linearSpeed.y);
    /*pitch*/
    float pitch_error = pitch_MF.f(target_pos.pitch - current_pos.pitch);
    /*pitchSpeed*/
    float pitchSpeed_error = pitchSpeed_MF.f(0 - current_angularSpeed.pitch);

    float s_error;
    float sspeed_error;

    if (is_rotation)
    {
        speed_error = 0;
        // pitch_error = 0;
    }
    else if (abs(current_linearSpeed.y) > 1.2f && abs(turn_MF.f(current_angularSpeed.yaw)) > 1.2f)
    {
        speed_error = last_speed_error;
        // speed_error = 0;
    }

    for (int i = 0; i < 2; i++)
    {
        /*s*/
        s_error = 0 - s_MF[i].f(current_sliderLocation[i].y);
        /*sspeed*/
        sspeed_error = 0 - sspeed_MF[i].f(current_sliderSpeed[i].y);
        output.sliderCtrl_out[i] = distance_error * slider_distance_kp +
                                   speed_error * slider_speed_kp +
                                   pitch_error * slider_pitch_kp +
                                   pitchSpeed_error * slider_pitchSpeed_kp +
                                   s_error * slider_sposition_kp +
                                   sspeed_error * slider_sspeed_kp;
    }
    last_speed_error = speed_error;

    slider_follow_pid.Target = 0;
    slider_follow_pid.Current = current_sliderLocation[0].y - current_sliderLocation[1].y;
    slider_follow_pid.Adjust();
    output.sliderCtrl_out[0] -= slider_follow_pid.Out;
    output.sliderCtrl_out[1] += slider_follow_pid.Out;

    debug_out_A = distance_error;
    debug_out_B = speed_error;
    debug_out_C = pitch_error;
    debug_out_D = pitchSpeed_error;
    debug_out_F = s_error;
    debug_out_G = sspeed_error;
}

/**
 * @brief  lqr自适应环
 * @note
 * @param
 * @return  角度输出
 * @retval  None
 */
bool leapState = false;
float Controller<LQR>::self_adaption()
{                                    //自适应环
    static MeanFilter<50> speed_MF1; //均值滤波
    static float setpoint_adapt_out = 0.0f;
    float setpoint_ctrl_out = 0.0f;
    float speed = current_linearSpeed.y;
    float speed_mf1 = 0.0f;
    speed_MF1 << speed;
    speed_MF1 >> speed_mf1; //大滤波

    if (fabsf(speed_mf1) > 0.6f && sport_flag == false)
    {
        sport_flag = true;
    }
    else if (fabsf(speed_mf1) < 0.6f && sport_flag == true)
    {
        sport_flag = false;
    }

    if (sport_flag)
    {
        if (fabsf(target_linearSpeed.y) < 0.1f * fabsf(speed_mf1) && abs(current_linearSpeed.y) < 2.5f)
        {
            break_flag = true;
        }
        else
        {
            break_flag = false;
        }
    }
    else
    {
        break_flag = false;
    }

    if (is_rotation)
    {
        distance_flag = false;
        return rotation_point;
        // rotation_point_pid.Target = rotation_point;
        // rotation_point_pid.Current = current_pos.pitch;
        // rotation_point_pid.Adjust();
        // return rotation_point_pid.Out;
    }
    else
    {
        rotation_point_pid.clean_intergral();
    }
    //根据情况判断当前平衡点
    if (is_turn90degrees)
    {
        balance_point = -0;
    }
    else
    {
        balance_point = -0.f;
    }

    /*负方向运动策略*/
    if (target_linearSpeed.y < -0.3f)
    {
        if (current_linearSpeed.y > 0.9f * target_linearSpeed.y)
        {
            setpoint_ctrl_out = -1.5f;
            if (is_unlimited)
            {
                setpoint_ctrl_out = -2.5f;
            }
            if (is_turn90degrees)
            {
                setpoint_ctrl_out = -0.f;
            }
        }
    }
    /* 正方向运动策略 */
    else if (target_linearSpeed.y > 0.3f)
    {
        if (current_linearSpeed.y < 0.9f * target_linearSpeed.y)
        {
            setpoint_ctrl_out = 1.5f;
            // setpoint_ctrl_out = 0.f;
            if (is_unlimited)
            {
                setpoint_ctrl_out = 2.5f;
                // setpoint_ctrl_out = 1.f;
            }
            if (is_turn90degrees)
            {
                setpoint_ctrl_out = 0.f;
            }
        }
    }
    /* 刹车 */
    else if (break_flag)
    {
        setpoint_ctrl_out = balance_point;
    }
    else
    {
    }

    //没有做前后适应
    if (is_leap)
    {
        if (fabsf(current_linearSpeed.y) > 3.1f && leapState == false)
        {
            leapState = true;
        }
        else if (fabsf(current_linearSpeed.y) < 1.4f && leapState == true)
        {
            leapState = false;
        }
        if (target_linearSpeed.y > 0.3f)
        {
            if (leapState == true && current_linearSpeed.y > 1.4f)
            {
                setpoint_ctrl_out = -6.f;
            }
            else
            {
                setpoint_ctrl_out = 4.f;
            }
        }
        else if (target_linearSpeed.y < -0.3f)
        {
            if (leapState == true && current_linearSpeed.y < -1.4f)
            {
                setpoint_ctrl_out = 6.f;
            }
            else
            {
                setpoint_ctrl_out = -4.f;
            }
        }
    }
    else if (is_slope)
    {
        setpoint_ctrl_out = 0.f;
    }
		
		debug_out_H = setpoint_ctrl_out;
		
    /*加速后制动一段时间才进入零点自适应*/
    if (target_linearSpeed.y != 0 && !break_flag)
    {
        setpoint_adaption_flag = false;
    }
    else
    {
        setpoint_adaption_flag = true; //在摇杆回0的时候直接打开自适应辅助刹车
    }

    if (setpoint_adaption_flag)
    {
        set_point_pid.Target = 0; //正常状态下以0（速度）为目标值
        set_point_pid.Current = speed_mf1;
        set_point_pid.Adjust();
        setpoint_adapt_out = balance_point - set_point_pid.Out;
    }
    else
    {
    }

    if (setpoint_adaption_flag)
    {
        if (break_flag == false && abs(speed_mf1) < 1.0f)
        {
            distance_flag = true;
        }
        else
        {
            distance_flag = false;
        }
        return std_lib::constrain(setpoint_adapt_out, -10.0f, 10.0f); //自适应输出
    }
    else
    {
        distance_flag = false;
        if (down_slope_flag == true)
        {
            return std_lib::constrain(setpoint_ctrl_out, -2.0f, 2.0f); //定角输出
        }
        else
        {
            return std_lib::constrain(setpoint_ctrl_out, -15.0f, 15.0f); //定角输出
        }
    }
}

/**
 * @brief  lqr距离环
 * @note
 * @param
 * @return  力矩输出
 * @retval  None
 */
float Controller<LQR>::distance_adjust()
{
    float distance_error = target_location.y - current_location.y;
    distance_error = std_lib::constrain(distance_error, -distance_max, distance_max);
    float distance_out = distance_error * body_distance_kp;
    return distance_out;
}

/**
 * @brief  lqr直立环
 * @note
 * @param
 * @return  力矩输出
 * @retval  None
 */
float Controller<LQR>::stand_adjust()
{
    static SecondOrderButterworthLPF pitch_lpf(20, 500);
    static SecondOrderButterworthLPF pitchSpeed_lpf(20, 500);
    float pitch_error = target_pos.pitch - current_pos.pitch;
    float pitchSpeed_error = 0 - current_angularSpeed.pitch;
    return pitch_lpf.f(pitch_error) * body_pitch_kp + pitchSpeed_lpf.f(pitchSpeed_error) * body_pitchSpeed_kp;
}

/**
 * @brief  lqr速度环
 * @note
 * @param
 * @return  力矩输出
 * @retval  None
 */
float Controller<LQR>::speed_adjust()
{
    static MeanFilter<10> speed_mf;
    float speed_error = speed_mf.f(target_linearSpeed.y - current_linearSpeed.y);
    return speed_error * body_speed_kp;
}

/**
 * @brief  lqr转向环
 * @note
 * @param
 * @return  力矩输出
 * @retval  None
 */
float Controller<LQR>::turn_adjust()
{
    static SecondOrderButterworthLPF turn_lpf(20, 500);
    float yaw_error = target_pos.yaw - current_pos.yaw;
    float yawSpeed_error = target_angularSpeed.yaw - current_angularSpeed.yaw;
    return yaw_error * body_yaw_kp + turn_lpf.f(yawSpeed_error) * body_yawSpeed_kp;
}

/**
 * @brief  lqr前馈
 * @note
 * @param
 * @return  电流输出
 * @retval  None
 */
float Controller<LQR>::stand_feedforward()
{ //直立前馈
    float error = 0 - current_pos.pitch;
    if (error > 0)
    {
        return -feedforward_ratio * arm_sin_f32(fabsf(error));
    }
    else
    {
        return feedforward_ratio * arm_sin_f32(fabsf(error));
    }
}

/**
 * @brief  滑块机体控制
 * @note
 * @param
 * @return  电流输出
 * @retval  None
 */
float Controller<LQR>::slider_adjust()
{
    static MeanFilter<10> s_mf;
    static MeanFilter<10> sspeed_mf;
    float s_error = 0 - 0.5f * (current_sliderLocation[0].y + current_sliderLocation[1].y);
    float sspeed_error = 0 - 0.5f * (current_sliderSpeed[0].y + current_sliderSpeed[1].y);
    return body_sposition_kp * s_mf.f(s_error) + body_sspeed_kp * sspeed_mf.f(sspeed_error);
}

/**
 * @brief  失重检测
 * @note
 * @param
 * @return
 * @retval  None
 */
void Controller<LQR>::weightless_check()
{
    if (current_linearAcc.z < -0.4f)
    {
        real_count += 2;
    }
    else
    {
        real_count -= 8;
    }
    real_count = std_lib::constrain(real_count, (int16_t)0, (int16_t)1000);
    if (real_count > weightless_delay)
    {
        weightless_flag = true;
    }
    else
    {
        weightless_flag = false;
    }
}

/**
 * @brief  空转检测
 * @note
 * @param
 * @return
 * @retval  None
 */
void Controller<LQR>::idling_check()
{
    if (abs(current_linearSpeed.y / motor_current) > 3.f && abs(current_linearSpeed.y) > 3.f)
    {
        idling_count--;
        idling_count = std_lib::constrain(idling_count, (int16_t)0, (int16_t)10);
        if (idling_count == 0)
        {
            idling_flag = true;
        }
        else
        {
            idling_flag = false;
        }
    }
    else
    {
        idling_flag = false;
        idling_count = 10;
    }
}

/**
 * @brief  小陀螺撞墙检测
 * @note
 * @param
 * @return
 * @retval  None
 */
void Controller<LQR>::rotation_crash_check()
{
    debug_out_I = current_angularSpeed.yaw - last_speed_z;
    if (is_rotation && ((current_angularSpeed.yaw - last_speed_z) > 3.f && (current_angularSpeed.yaw - target_angularSpeed.yaw) > 90.f))
    {
        is_rotation_crash = true;
    }
    else
    {
        is_rotation_crash = false;
    }
    last_speed_z = current_angularSpeed.yaw;
}

/**
 * @brief  lqr控制器重置
 * @note
 * @param
 * @return
 * @retval  None
 */
void Controller<LQR>::reset_adjust()
{
    Update_Target_Location();
    Update_Current_Location();
    Update_Target_Pos();
    Update_Current_Pos();
    Update_Target_LinearSpeed();
    Update_Current_LinearSpeed();
    Update_Target_AngularSpeed();
    Update_Current_AngularSpeed();
    Update_Target_LinearAcc();
    Update_Current_LinearAcc();
    Update_Target_AngularAcc();
    Update_Current_AngularAcc();
    speed_pid_cnt = speed_pid_delay * 2.0f; //重置时给两倍的延时间隔

    set_point_pid.I_Term = 0.0f;
    output = Controller_Out{}; //清零输出
    output.distance_out = 0;
    output.feedforward_out = 0;
    output.set_point_out = 0;
    output.speed_out = 0;
    output.stand_out = 0;
    output.turn_out = 0;
    target_location.y = current_location.y;

    sport_flag = false;
    break_flag = false;
    distance_flag = false;
    distance_enable = false;
    setpoint_adaption_flag = false;
    last_sport_flag = false;
    weightless_flag = false; //失重检测标志位
    idling_flag = false;
    weightless_delay = 0; //失重检测延时
    real_count = 0;
    motor_current = 0; //电机电流（用于空转检测）
    idling_count = 10; //空转计数器
}

/*****************************************************功率控制*************************************************/
float Cap_Charge_Controller(const float current, const float target)
{
    Cap_Charge_Pid.Target = target;
    Cap_Charge_Pid.Current = current;
    Cap_Charge_Pid.Adjust();
    return Cap_Charge_Pid.Out;
}

float Power_Limit_Controller(const float current, const float target)
{
    Power_Limit_Pid.Target = target;
    Power_Limit_Pid.Current = current;
    Power_Limit_Pid.Adjust();
    return Power_Limit_Pid.Out;
}
