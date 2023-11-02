/*! @file Chassis_Integration.h
 *
 *  @brief header file for Chassis_Integration.cpp
 *
 */
#ifndef _CHASSIS_INTEGRATION_H_
#define _CHASSIS_INTEGRATION_H_

#include "manipulator_controller.h"
#include "myMiddleware.h"
class WheelBipe_Infantry_Classdef;

#define BALANCE_DELAY 250  // 判断是否成功平衡的延时
#define FALLDOWN_DELAY 50  // 判断是否摔倒的延时
#define PROTECT_DELAY 1500 // 是否开始保护机制的延时

/*状态机模式*/
namespace sMachine
{
  enum E_StateMachine
  {
    RESET = 0,
    LOSTCTRL = 1,
    PREBALANCE = 2,
    BALANCE = 3,
    WEIGHTLESS = 4,
    FALLDOWN = 5,
    SELFRESCUE = 6
  };
}

// 状态基类
class State_Base
{
protected:
  WheelBipe_Infantry_Classdef *context; // 上下文指针，指向要操作的类对象

public:
  // 上下文切换，让实例化对象获取被控对象的指针（索引）
  void Set_Context(WheelBipe_Infantry_Classdef *_context)
  {
    this->context = _context;
  }
  virtual void State_Handler() = 0; // 状态机方法，子类中实现实例化
};
/*预平衡状态机*/
class PreBalance_State : public State_Base
{
  virtual void State_Handler();
};
/*平衡状态机*/
class Balance_State : public State_Base
{
  virtual void State_Handler();
};
/*失重状态机*/
class Weightless_State : public State_Base
{
  virtual void State_Handler();
};
/*复位状态机*/
class Reset_State : public State_Base
{
  virtual void State_Handler();
};
/*失控状态机*/
class LostCtrl_State : public State_Base
{
  virtual void State_Handler();
};
/*摔倒状态机*/
class FallDown_State : public State_Base
{
  virtual void State_Handler();
};
/*固连自救状态机*/
class SelfRescue_State : public State_Base
{
  virtual void State_Handler();
};

extern PreBalance_State prebalanceState;
extern Balance_State balanceState;
extern Weightless_State weightlessState;
extern Reset_State resetState;
extern LostCtrl_State lostctrlState;
extern FallDown_State falldownState;

/*轮足统筹类（最后创建）*/
class WheelBipe_Infantry_Classdef
{
public:
  // 切换状态函数
  void Status_Switching(State_Base *_state)
  {
    this->current_state = _state;           // 状态指针指向新的状态
    this->current_state->Set_Context(this); // 将当前数据挂载到状态机当中
  }

  WheelBipe_Infantry_Classdef(MyMiddlewareClassdef *_middleware, Manipulator_Controller_Classdef *_controller);             // 构造函数
  void Top_Init(lqrCalculater<6, 2> *_lqrCal, PIDmethod _wSubCtrl[2], PIDmethod _jSubCtrl[4]);                              // 顶层初始化操作                                                                       //顶层初始化操作
  void Middle_Init(QueueHandle_t _USART_TxPort, uint8_t port_num[2], MotorHT04Classdef _motor[4], LPMS_BE2_Typedef *_lpms); // 中间层初始化操作

  void Joint_Reset(); // 关节电机收腿校准

  void State_Data_Update(); // 状态数据更新
  void State_Judge();       // 状态判断
  void adjust();            // 控制量计算
  void acutate();           // 执行器输出

  void params_config(); // 参数修改

  float target_length = 0.15;
  float speed_scale = 2.f;

  void debug_remoteCtrl(); // 测试车底盘遥控
  void gimbal_ctrl();      // 云台控制底盘
  void check_balance();    // 判断是否真的平衡
  void check_overturn();   // 判断是否翻车

public:
  // 状态基类指针，用于更新状态
  State_Base *current_state;
  sMachine::E_StateMachine state_flag;
  // 使用友元类，使状态方法可以操作大类成员
  friend PreBalance_State;
  friend Balance_State;
  friend Weightless_State;
  friend Reset_State;
  friend LostCtrl_State;
  friend FallDown_State;
  // 操作类型指针
  MyMiddlewareClassdef *middleware;            // 中间层
  Manipulator_Controller_Classdef *controller; // 顶层控制器

  // 输出载体
  float WheelOut[2] = {0, 0};
  float JointOut[4] = {0, 0, 0, 0};

  // 状态机标志位
  bool is_reset = false;                  // 初始化操作标志位
  bool en_chassis = false;                // 是否使能底盘输出
  bool en_legForce = false;               // 腿杆力控使能（如果打开则腿部力控，如果关闭则腿部位控）
  bool en_wheel = false;                  // 轮子使能
  bool is_balance = false;                // 车体是否真的平衡
  bool is_overturn = false;               // 是否翻车
  int16_t isBalanceCount = BALANCE_DELAY; // 平衡判断计数器
  bool is_fallDown = false;               // 摔跤触发标志位
  int16_t fallDownCount = FALLDOWN_DELAY; // 摔跤判断计数器
  int16_t protectCount = PROTECT_DELAY;   // 保护机制计时器
  bool en_protect = false;                // 是否使能保护机制（主要为了应对从斜坡起立的情况）

  // 动作组指令
  bool last_jump_flag = false;
};

#endif
