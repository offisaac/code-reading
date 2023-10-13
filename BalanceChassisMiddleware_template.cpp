#include "BalanceChassisMiddleware_template.h"
#include "internal.h"

abstractBalanceChassis::abstractBalanceChassis(QueueHandle_t* USART_TxPort, uint8_t left_wheel_id, uint8_t right_wheel_id)
{
    wheelMotor[LEFT].bindMotor(new wheelMotorType(left_wheel_id));
    wheelMotor[RIGHT].bindMotor(new wheelMotorType(right_wheel_id));

    wheelMotor[LEFT].Polarity = 1;
    wheelMotor[RIGHT].Polarity = -1;

    /*电机减速比以及电流扭矩常数设置*/
    wheelMotor[LEFT].out_unit_convert = wheelMotor[RIGHT].out_unit_convert = 1.f / 0.32f * 2048.f / 16.5f;
    wheelMotor[LEFT].speed_unit_convert = wheelMotor[RIGHT].speed_unit_convert = 1;//通常设置为减速比的倒数

    /*陀螺仪抽象类初始化*/
    absIMU.bindIMU(new IMUType(USART_TxPort, 1));
    absIMU.imu->Data_Type_Config(DATA_16BIT);
    absIMU.imu->LPMS_BE2_Init();

    absIMU.bindAccCoordinate(abstractIMU::imuWorldAccZ, abstractIMU::imuWorldAccY, abstractIMU::imuWorldAccX);
    absIMU.bindEulerCoordinate(abstractIMU::imuWorldRoll, abstractIMU::imuWorldYaw, abstractIMU::imuWorldPitch);

    absIMU.accPolarity.x = 1;
    absIMU.accPolarity.y = 1;
    absIMU.accPolarity.z = 1;

    absIMU.eularPolarity.pitch = 1;
    absIMU.eularPolarity.roll = 1;
    absIMU.eularPolarity.yaw = 1;

    absIMU.eularBaseData.pitch = 0.f;
    absIMU.eularBaseData.roll = 0.f;
    absIMU.eularBaseData.yaw = 0;
}

abstractBalanceChassis::~abstractBalanceChassis()
{
    delete wheelMotor[LEFT].motor;
    delete wheelMotor[RIGHT].motor;
    delete absIMU.imu;
}

void abstractBalanceChassis::bindCanHandle(QueueHandle_t* _wheelMotorCanHandle)//绑定CAN总线句柄，用于电机控制
{
    wheelMotorCanHandle = _wheelMotorCanHandle;
    /* 9025底盘用 */
    for(int i = 0; i < 2; i++)
    {
        wheelMotor[i].motor->init(*wheelMotorCanHandle);
    }
    /* 9025底盘用 */
}

void abstractBalanceChassis::CTRL_Data_Update(CAN_COB &CAN_RxMsg)
{
    if (CAN_RxMsg.ID == TOCHASSIS_PACK1_ID)
    {
        memcpy((uint8_t *)&ctrl_data, CAN_RxMsg.Data, CAN_RxMsg.DLC);
    }
    else if (CAN_RxMsg.ID == TOCHASSIS_PACK2_ID)
    {
        memcpy((uint8_t *)&ctrl_data.chassis_rotation_angle, CAN_RxMsg.Data, CAN_RxMsg.DLC);
    }
    else
    {
    }
    Set_MaxSpeed(Referee.GameRobotState.classis_power_limit, digital_Power.unit_DPW_data.Vcap);
    ctrl_data.target_speed_y = ctrl_data.get_speed_y * speed_scale / 1024.f;
    ctrl_data.target_speed_x = ctrl_data.get_speed_x * speed_scale / 1024.f;

    if(ctrl_data.rotation_state)
        ctrl_data.target_speed_z = (360.f + rotation_scale * 180.f) * DEGREE_TO_RAD;
    else
        ctrl_data.target_speed_z = ctrl_data.get_speed_z * 6 * PI / 1024.f;

    if(ctrl_data.turn90degrees || ctrl_data.ascent_state)
    {
        ctrl_data.target_speed_y *= 0.5f;
        ctrl_data.target_speed_x *= 0.5f;
    }  
}

void abstractBalanceChassis::controlWheelMotor(float wheelTorque[2])//轮子力控，单位N*m
{
    if (motorLinkCount[RIGHT] < 20 && motorLinkCount[LEFT] < 20)
    {
        for(int i = 0; i < 2; i++)
        {
            wheelMotor[i].setMotorCurrentOut(wheelTorque[i]);//若是9025电机，则函数内部已经实现发包了
        }
    }
    else
    {
        for (int i = 0; i < 2; i++)
        {
            if (motorLinkCount[i] >= 20)
                wheelMotor[i].motor->startMotor();
            else
                wheelMotor[i].motor->iqCloseControl_Current(0);
        }
    }

    /* 3508底盘用 */
    // Motor_CAN_COB TxBuff;
    // MotorMsgPack(TxBuff, *wheelMotor[RIGHT].motor, *wheelMotor[LEFT].motor);
    // xQueueSend(wheelMotorCanHandle, &TxBuff, 0);
    /* 3508底盘用 */
}

float abstractBalanceChassis::getLinerSpeed()//获取整车的线速度，单位m/s，滤波在不同的车之间需要调整
{
    static MeanFilter<20> speed_mf;
    float originSpeed = (wheelMotor[RIGHT].getMotorSpeed() + wheelMotor[LEFT].getMotorSpeed()) / 2.f / 60.f * 2 * PI * wheelRadius;
    return speed_mf.f(originSpeed);
}

/**
 * @brief 9025电机状态获取
 * @parma None
 * @return None
 */
void abstractBalanceChassis::Motor_State_Check()
{
    static float last_current[2] = {0, 0};
    wheelMotor[RIGHT].motor->readMotorState1_errorState();
    wheelMotor[LEFT].motor->readMotorState1_errorState();
    vTaskDelay(2);
    for (int i = 0; i < 2; i++)
    {
        float motor_current = wheelMotor[i].motor->getData().current;
        if (wheelMotor[i].motor->getData().errorState)
        {
            motorErrorCnt[i]++;
            wheelMotor[i].motor->cleanErrorState();
            vTaskDelay(2);
            wheelMotor[i].motor->startMotor();
            vTaskDelay(2);
            continue;
        }
        else
            motorErrorCnt[i] = 0;

        if (motor_current == last_current[i])
            motorDeadCnt[i]++;
        else
            motorDeadCnt[i] = 0;

        if (motorDeadCnt[i] >= 10)
        {
            wheelMotor[i].motor->startMotor();
            vTaskDelay(2);
        }
        last_current[i] = motor_current;
    }
}

void abstractBalanceChassis::Set_MaxSpeed(uint16_t _powerMax, float Cap_Voltage)
{
    if (_powerMax <= 60)
    {
        speed_scale = 2.f;
    }
    else if (_powerMax > 60 && _powerMax < 100)
    {
        speed_scale = 2.2f;
    }
    else if (_powerMax >= 100)
    {
        speed_scale = 2.4f; // 0.5f
    }
    else
    {
        speed_scale = 2.f;
    }
    /*各种功率标志位*/
    if (ctrl_data.leap_state && Cap_Voltage > 17.0f)
    {
        speed_scale = 3.4f;
    }
    else if (ctrl_data.unlimited_state && Cap_Voltage > 17.0f)
    {
        speed_scale += 0.4f;
    }
    else if (ctrl_data.ascent_state && Cap_Voltage > 17.0f)
    {
        speed_scale = 3.4f;
    }
    else
    {
    }
    speed_scale = std_lib::constrain(speed_scale, -5.f, 5.f);
    if (Cap_Voltage <= 12.f)
    {
        rotation_scale = 0.1;
        speed_scale = 1.7f;
    }
    else if (Cap_Voltage <= 16.f && Cap_Voltage > 12.f)
    {
        rotation_scale = 0.4;
        speed_scale = 1.9f;
    }
    else if (Cap_Voltage > 16.f && Cap_Voltage <= 23)
    {
        rotation_scale = 0.7f;
    }
    else
    {
        rotation_scale = 1.f;
    }
}

