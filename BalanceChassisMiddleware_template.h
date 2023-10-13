#pragma once

#include "SRML.h"
#include "arm_math.h"

#ifdef __cplusplus
//左右电机编号
enum _chassis_WheelEnumdef
{
    LEFT = 1U,
    RIGHT = 0U
};

#define TOGIMBAL_PACK1_ID 			0x221							
#define TOGIMBAL_PACK2_ID			0x220							
#define TOCHASSIS_PACK1_ID			0x222
#define TOCHASSIS_PACK2_ID			0x223

//控制信息的数据结构体，在实车上为接收云台的数据
#pragma pack(1)
typedef struct _CTRL_Data_Structdef
{
    int16_t get_speed_y;
    int16_t get_speed_x;
    int16_t get_speed_z;
    /* 更新标志位 */
    uint8_t remote_ctrl_state : 1; //遥控状态
    uint8_t unlimited_state : 1;   //超功率
    uint8_t rotation_state : 1;    //小陀螺
    uint8_t leap_state : 1;        //飞坡
    uint8_t bulletbay_state : 1;   //弹舱盖开启
    uint8_t ascent_state : 1;      //上坡
    uint8_t ui_reset_flag : 1;     //手动复位
    uint8_t vision_mode_flag1 : 1; //视觉模式1
    uint8_t vision_mode_flag2 : 1; //视觉模式2
    uint8_t enable_cmd : 1;        //底盘使能
    uint8_t self_rescue_state : 1; //固连自救
    uint8_t sliding_remake : 1;    //滑块复位
    uint8_t turn90degrees : 1;     //转90度
    uint8_t gg_flag : 1;           //寄掉
    uint8_t vision_can_shoot : 1;  //视觉是否发射
    uint8_t fri_state : 1;         //摩擦轮是否打开

    /*底盘旋转角*/
    float chassis_rotation_angle;
    float target_speed_y;
    float target_speed_x;
    float target_speed_z;
} CTRL_Data_Structdef;
#pragma pack()

class abstractBalanceChassis
{
private:
    using wheelMotorType = MotorMF9025v2Classdef;//方便电机类型更换
    using IMUType = LPMS_BE2_Typedef;//方便IMU类型更换

    QueueHandle_t* wheelMotorCanHandle = NULL;
    CTRL_Data_Structdef ctrl_data = {};

    //电机状态检测
    uint16_t motorLinkCount[2] = {}; // 电机连接检测1
    uint16_t motorErrorCnt[2] = {}; // 电机出错统计
    uint16_t motorDeadCnt[2] = {};

    float speed_scale = 0;    //速度系数，用于限制当前功率下的速度
    float rotation_scale = 0; //小陀螺转速，用于限制不同功率状况下的转速
    void Set_MaxSpeed(uint16_t _powerMax, float Cap_Voltage); //设置最大速度
public:
    float wheelRadius = 0.11f;
    abstractMotor<wheelMotorType> wheelMotor[2];
    abstractIMUClassdef<IMUType> absIMU;

    abstractBalanceChassis(QueueHandle_t *USART_TxPort, uint8_t left_wheel_id, uint8_t right_wheel_id);
    ~abstractBalanceChassis();

    /* 与底层相关的函数，在webots的中间层中需要重写 */
    void bindCanHandle(QueueHandle_t *_wheelMotorCanHandle); // 绑定CAN总线句柄，用于电机控制
    void CTRL_Data_Update(CAN_COB &CAN_RxMsg);
    inline float getChassisPower() { return digital_Power.power.pow_motor; }

    /* 在抽象模块的支持下，迁移平台不用重写（迁移至DJI电机除外） */
    void controlWheelMotor(float wheelTorque[2]); // 轮子力控，单位N*m
    float getLinerSpeed(); // 获取整车的线速度，单位m/s，滤波在不同的车之间需要调整
    inline const CTRL_Data_Structdef& getCtrlData() { return ctrl_data; }
    
    /* 以下并非顶层调用的的api，在webots中间层中不用实现 */
    inline void MotorUpdate(CAN_COB& CAN_RxCOB)
    {
        if(wheelMotor[LEFT].update(CAN_RxCOB))
        {
            motorLinkCount[LEFT] = 0;
        }
        else if (wheelMotor[RIGHT].update(CAN_RxCOB))
        {
            motorLinkCount[RIGHT] = 0;
        }
    }
    inline void Link_Check()
    {
        motorLinkCount[0]++;
        motorLinkCount[1]++;
    }
    void Motor_State_Check();
};

#endif /* __cplusplus */

