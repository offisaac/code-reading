#ifndef _BALANCE_CHASSIS_H_
#define _BALANCE_CHASSIS_H_

#define DIGITAL_POWER 1

#include "SRML.h"
#include "FreeRTOS.h"
#include "task.h"
#include "queue.h"
#include "math.h"
#include "board_com.h"
#include "balance_controller.h"

#define DEGREE_TO_RAD PI / 180.f
#define CTRL_INTERAL 0.002f                          //控制周期


//声明平衡步兵类
class Balance_Infantry_Classdef;

//状态基类
class State_Base
{
protected:
    Balance_Infantry_Classdef *context; //上下文指针，指向要操作的类对象

public:
    //上下文切换，让实例化对象获取被控对象的指针（索引）
    void Set_Context(Balance_Infantry_Classdef *_context)
    {
        this->context = _context;
    }

    virtual void State_Handler() = 0; //状态机方法，子类中实现实例化
};

//失控状态机
class Lost_Ctrl_State : public State_Base
{
    virtual void State_Handler();
};

//预平衡状态机
class Pre_Balance_State : public State_Base
{
    virtual void State_Handler();
};

//平衡状态机
class Balance_State : public State_Base
{
    virtual void State_Handler();
};

extern Lost_Ctrl_State lostctrl_state;
extern Pre_Balance_State prebalance_state;
extern Balance_State balance_state;

//平衡步大类
class Balance_Infantry_Classdef
{
public:
    //引入类型
    Controller<LQR> balance_controller; //底盘控制器
    uint8_t machine_mode = 0;

    Balance_Infantry_Classdef();
    //总控函数
    void Chassis_Ctrl()
    {
        Judge_State();       // 执行状态机
        Chassis_Adjust();    //控制量下发
    }

    //切换状态函数
    inline void Status_Switching(State_Base *_state)
    {
        this->current_state = _state;           //状态指针指向新的状态
        this->current_state->Set_Context(this); //将当前数据挂载到状态机当中
    }
		
public:
    //使用友元类，使状态方法可以配置大类数据
    friend Lost_Ctrl_State;
    friend Pre_Balance_State;
    friend Balance_State;

    State_Base *current_state; //状态基类指针，用于更新状态

    //总控函数部分
    void Judge_State();       //转换状态
    void Chassis_Ctrl_Cal();  //控制器计算
    void Chassis_Adjust();    //控制量下发
    void Reset_Adjust();      //重置函数
    //控制器更新数据部分
    void Update_Target(float _y_speed, float _z_speed, float _x_speed);
    void Update_Current_Pos(float _yaw, float _pitch, float _chassis_angle);
    void Update_Current_Speed(float _y, float _yaw, float _pitch);
    void Update_Current_Acc(float _x, float _y, float _z);
    void Update_Motor_Current(float _current);
    void Update_Slider_Params(float _s[2], float _sspeed[2]);

    //底盘输出
    float wheel_stand_out_theory[2] = {0}; //直立环理论输出（相对于整车的前后而言）
    float wheel_speed_out_theory[2] = {0}; //速度环理论输出（相对于整车的前后而言）
	float wheel_out[2];

    /*小陀螺平移*/
    float rotation_move_gain = 0.3f;  //平移速度增益
    float rotation_chassis_angle = 0; //底盘坐标相对于云台转角
};

#endif
