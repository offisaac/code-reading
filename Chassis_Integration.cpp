/*! @file Chassis_Integration.cpp
 *
 *  @brief chassis state machine
 *        底盘控制状态机
 */
#include "Chassis_Integration.h"
#include "internal.h"

uint8_t state_num = 0;

PreBalance_State prebalanceState;
Balance_State balanceState;
Weightless_State weightlessState;
Reset_State resetState;
LostCtrl_State lostctrlState;
FallDown_State falldownState;
SelfRescue_State selfrescueState;

/**
 * @brief 重置状态机
 */
void Reset_State::State_Handler()
{
  context->state_flag = sMachine::RESET;
  context->is_balance = false;
  context->is_fallDown = false;
  context->Joint_Reset(); // 关节收腿重新校准
  for (int i = 0; i < 2; i++)
  {
    context->WheelOut[i] = 0;
  }

  for (int j = 0; j < 4; j++)
    context->JointOut[j] = 0;

  context->is_reset = true;
  context->Status_Switching(&lostctrlState);
}

/**
 * @brief 失控状态机
 */
void LostCtrl_State::State_Handler()
{
  context->state_flag = sMachine::LOSTCTRL;
  context->is_balance = false;
  context->is_fallDown = false;
  context->controller->state->flags.weightlessness = false;
  context->en_chassis = false;
  context->en_legForce = false;
  context->en_wheel = false;
  context->target_length = 0.15f;
  bool w_en[6] = {false, false, false, false, false, false};
  bool j_en[6] = {false, false, false, false, false, false};
  context->controller->Set_Enable_List(w_en, j_en);

  context->controller->state->target_location.y = context->controller->state->current_location.y;
}
/**
 * @brief 预平衡状态机
 */
void PreBalance_State::State_Handler()
{
  context->state_flag = sMachine::PREBALANCE;
  context->is_balance = false;
  context->is_fallDown = false;
  /*预平衡收腿准备*/
  context->en_chassis = true;
  context->en_legForce = false;
  context->en_wheel = false;
  context->target_length = 0.15f; // 预平衡状态必须收腿
  static bool w_en[6] = {false, false, false, false, false, false};
  static bool j_en[6] = {false, false, false, false, false, false};
  context->controller->Set_Enable_List(w_en, j_en);

  context->controller->state->target_location.y = context->controller->state->current_location.y;

  if (context->middleware->getRecData().chassis_state.enable_cmd == true)
  {
    context->Status_Switching(&balanceState); // 底盘使能，进入平衡状态
  }
}

/**
 * @brief 平衡状态机
 */
void Balance_State::State_Handler()
{
  context->state_flag = sMachine::BALANCE;
  context->en_chassis = true;
  context->check_balance();                                                 // 判断当前是否真的平衡
  context->target_length = context->middleware->getRecData().target_height; // 平衡状态取云台下发数据
  // context->debug_remoteCtrl();
  context->gimbal_ctrl();
  bool w_en[6] = {true, true, true, true, true, true};
  bool j_en[6] = {true, true, true, true, true, true};
  context->controller->Set_Enable_List(w_en, j_en);

  if (context->controller->state->flags.tiny_weightlessness == true)
  {
    context->Status_Switching(&weightlessState);
  }
}

/**
 * @brief 失重状态机
 */
void Weightless_State::State_Handler()
{
  context->state_flag = sMachine::WEIGHTLESS;
  context->en_chassis = true;
  context->en_legForce = true;
  bool w_en[6] = {false, false, false, false, false, false};
  bool j_en[6] = {false, false, true, false, false, true};
  context->controller->Set_Enable_List(w_en, j_en);

  if (context->controller->state->flags.tiny_weightlessness == false)
  {
    context->Status_Switching(&balanceState);
  }
}

/**
 * @brief 摔倒状态机
 */
void FallDown_State::State_Handler()
{
  static int16_t cnt = 50;
  context->state_flag = sMachine::FALLDOWN;
  context->is_balance = false;
  context->en_legForce = false;
  context->en_wheel = false;
  context->target_length = 0.15f;
  bool w_en[6] = {false, false, false, false, false, false};
  bool j_en[6] = {false, false, false, false, false, false};
  context->controller->Set_Enable_List(w_en, j_en);

  context->en_chassis = true;
  if (abs(context->controller->state->current_speed.y) < 0.5f)
  {
    cnt--;
    if (cnt < 0)
    {
      cnt = 50;
      context->Status_Switching(&prebalanceState); // 轮子速度较低时重新进入预平衡状态
      context->is_fallDown = false;
    }
  }
  else
  {
    cnt = 50;
  }
}

/**
 * @brief 固连自救状态机
 */
void SelfRescue_State::State_Handler()
{
  context->state_flag = sMachine::SELFRESCUE;
  context->is_balance = false;
  context->is_fallDown = false;
  /*预平衡收腿准备*/
  context->en_chassis = true;
  context->en_wheel = true;
  context->en_legForce = false;
  context->target_length = 0.15f; // 预平衡状态必须收腿
  static bool w_en[6] = {false, false, false, false, false, true};
  static bool j_en[6] = {false, false, false, false, false, false};
  context->controller->Set_Enable_List(w_en, j_en);

  context->controller->state->target_location.y = context->controller->state->current_location.y;

  if (context->controller->state->chassis_state.self_rescue_state == false)
  {
    context->Status_Switching(&prebalanceState);
    context->protectCount = PROTECT_DELAY;
  }
}

/*构造函数*/
WheelBipe_Infantry_Classdef::WheelBipe_Infantry_Classdef(MyMiddlewareClassdef *_middleware, Manipulator_Controller_Classdef *_controller)
{
  middleware = _middleware;
  controller = _controller;
  current_state = &resetState; // 默认为reset
}

/*顶层初始化操作*/
void WheelBipe_Infantry_Classdef::Top_Init(lqrCalculater<6, 2> *_lqrCal, PIDmethod _wSubCtrl[2], PIDmethod _jSubCtrl[4])
{
  controller->mp[RIGHT_JOINT]->Init();
  controller->mp[LEFT_JOINT]->Init();
  controller->Load_Lqr_Controller(_lqrCal);
  controller->Load_Wheel_SubController(_wSubCtrl);
  controller->Load_Joint_SubController(_jSubCtrl);
  controller->Init();
}

/*中层初始化操作*/
void WheelBipe_Infantry_Classdef::Middle_Init(QueueHandle_t _USART_TxPort, uint8_t port_num[2], MotorHT04Classdef _motor[4], LPMS_BE2_Typedef *_lpms)
{
  middleware->init(_USART_TxPort, port_num, _lpms);
  // vTaskDelay(1000);
  middleware->jointInit(_motor);
}

/*关节重置*/
void WheelBipe_Infantry_Classdef::Joint_Reset()
{
  middleware->jointReset();
  middleware->setJointOffset();
  middleware->jointCommandClear();
  vTaskDelay(10);
}

/*更新状态*/
float rotation_angle = -3.f;
void WheelBipe_Infantry_Classdef::State_Data_Update()
{
  static float distance = 0;

  middleware->Link_Check(); // 通信检测
  check_overturn();

  float *jointAngle = middleware->getMotorAngle();
  float *jointTorque = middleware->getMotorTorque();
  AngularDataStructdef *angleVel = middleware->getAngleVelData();
  AngularDataStructdef *eularData = middleware->getEularData();
  LinearDataStructdef *laccData = middleware->getAccData();

  controller->state->timeStamp_update();

  angleVel->pitch = -mpu_receive.gyro[0] / ratio_rad_to_degree;
  angleVel->roll = -mpu_receive.gyro[1] / ratio_rad_to_degree;
  angleVel->yaw = -mpu_receive.gyro[2] / ratio_rad_to_degree;
  if (abs(angleVel->pitch) < 0.01f)
  {
    angleVel->pitch = 0;
  }
  if (abs(angleVel->roll) < 0.01f)
  {
    angleVel->roll = 0;
  }
  if (abs(angleVel->yaw) < 0.01f)
  {
    angleVel->yaw = 0;
  }

  distance += (middleware->getRecData().linerSpeed + angleVel->pitch * wheelRadius) * controller->state->dt; // 计算路程

  controller->mp[RIGHT_JOINT]->target_joint.pendulum.length = target_length;
  controller->mp[LEFT_JOINT]->target_joint.pendulum.length = target_length;

  controller->mp[RIGHT_JOINT]->target_joint.pendulum.angle = 0;
  controller->mp[LEFT_JOINT]->target_joint.pendulum.angle = 0;

  controller->mp[RIGHT_JOINT]->timer_update(controller->state->dt);
  controller->mp[LEFT_JOINT]->timer_update(controller->state->dt);

  controller->mp[RIGHT_JOINT]->current_joint_update(jointAngle[RF], jointAngle[RB], jointTorque[RF], jointTorque[RB]);
  controller->mp[LEFT_JOINT]->current_joint_update(jointAngle[LF], jointAngle[LB], jointTorque[LF], jointTorque[LB]);

  controller->state->target_update(middleware->getRecData().target_speed_y, -middleware->getRecData().target_speed_z);

  float speed_error_acc, speed_error_break;
  if (middleware->getRecData().chassis_state.turn90degrees)
  {
    speed_error_acc = 0.9f;
    speed_error_break = 1.5f;
  }
  else
  {
    if (middleware->getRecData().chassis_state.ascent_state)
    {
      speed_error_acc = 1.0f;
      speed_error_break = 0.8f;
    }
    else if (middleware->getRecData().chassis_state.leap_state)
    {
      speed_error_acc = 1.0f;
      speed_error_break = 1.5f;
    }
    else if (middleware->getRecData().chassis_state.unlimited_state)
    {
      speed_error_acc = 1.0f;
      speed_error_break = 1.5f;
    }
    else
    {
      speed_error_acc = 0.9f;
      speed_error_break = 1.5f;
    }
  }

  if (en_legForce)
  {
    if (infantry_state.flags.break_flag)
    {
      controller->state->target_speed.y = upper::constrain(controller->state->target_speed.y, controller->state->current_speed.y - speed_error_break, controller->state->current_speed.y + speed_error_break);
    }
    else
    {
      controller->state->target_speed.y = upper::constrain(controller->state->target_speed.y, controller->state->current_speed.y - speed_error_acc, controller->state->current_speed.y + speed_error_acc);
    }
  }
  else
  {
  }

  if (middleware->getRecData().chassis_state.rotation_state)
  {
    controller->state->target_pos.pitch = (rotation_angle) / ratio_rad_to_degree;
  }
  else
  {
    controller->state->target_pos.pitch = balance_point / ratio_rad_to_degree;
  }

  controller->state->current_location_update(0, distance, 0);
  controller->state->current_speed_update(0, (middleware->getRecData().linerSpeed + angleVel->pitch * wheelRadius), 0);
  controller->state->current_pos_update(eularData->pitch, eularData->yaw, eularData->roll);
  controller->state->current_av_update(angleVel->pitch, angleVel->yaw, angleVel->roll);
  controller->state->current_acc_update(laccData->x, laccData->y, laccData->z);
  controller->state->power_limit_update(middleware->getRecData().power_limit_scale);
  controller->state->centripetal_force_update(middleware->getRecData().centripetal_force_scale);
  controller->state->gimbal_state_update(middleware->getRecData().chassis_state);

  controller->state->flags.is_legforce = en_legForce;

  /* 状态检测 */
  controller->state->sport_adaption();
  if (en_legForce == true) // 开腿力控
  {
    controller->state->tiny_weightlessness_check();
    controller->state->weightlessness_check();
  }
  else
  {
    controller->state->flags.weightlessness = false;
    controller->state->flags.tiny_weightlessness = false;
  }

  if (middleware->getRecData().chassis_state.jump_cmd != last_jump_flag)
  {
    if (en_legForce)
    {
      controller->state->flags.jump_flag = true;
    }
    else
    {
      controller->state->flags.jump_flag = false;
    }
  }
  last_jump_flag = middleware->getRecData().chassis_state.jump_cmd;
}

/*状态判断与执行*/
void WheelBipe_Infantry_Classdef::State_Judge()
{
  if (is_reset == false) // 未初始化时收腿重置
  {
    Status_Switching(&resetState);
  }
  else if (is_overturn || is_fallDown) // 翻车&摔倒
  {
    Status_Switching(&falldownState);
  }
  else if (middleware->getRecData().chassis_state.remote_ctrl_state == false) // 如果遥控没打开则进入失控模式，所有输出置零
  {
    Status_Switching(&lostctrlState);
  }
  else //以下为遥控打开时的状态判断
  {
    if (middleware->getRecData().chassis_state.self_rescue_state == true) // 倒地自救
    {
      Status_Switching(&selfrescueState);
    }
    else if (current_state == &lostctrlState || middleware->getRecData().chassis_state.enable_cmd == false)
    {
      Status_Switching(&prebalanceState);
    }
  }
  
  current_state->State_Handler();
}

/*控制量下发*/
void WheelBipe_Infantry_Classdef::adjust()
{
  /*控制结算*/
  controller->controll_adjust();
  /*输出控制*/
  if (en_chassis == true) // 底盘使能
  {
    if (en_legForce == true) // 开腿力控
    {
      JointOut[RF] = controller->mp[RIGHT_JOINT]->torque_output.f_joint;
      JointOut[RB] = controller->mp[RIGHT_JOINT]->torque_output.b_joint;
      JointOut[LF] = controller->mp[LEFT_JOINT]->torque_output.f_joint;
      JointOut[LB] = controller->mp[LEFT_JOINT]->torque_output.b_joint;
    }
    else // 关腿位控
    {
      controller->j_length_keep_pid[R]->integral = 0;
      controller->j_length_keep_pid[L]->integral = 0;
      controller->j_roll_keep_pid->integral = 0;
    }

    if (en_wheel == true)
    {
      WheelOut[R] = controller->mp[RIGHT_JOINT]->torque_output.wheel;
      WheelOut[L] = controller->mp[LEFT_JOINT]->torque_output.wheel;
      if (middleware->getRecData().chassis_state.self_rescue_state == true)
      {
        WheelOut[R] += middleware->getRecData().target_speed_y * 1.f;
        WheelOut[L] += middleware->getRecData().target_speed_y * 1.f;
      }
      WheelOut[R] = upper::constrain(WheelOut[R], 5.12f);
      WheelOut[L] = upper::constrain(WheelOut[L], 5.12f);
    }
    else
    {
      for (int i = 0; i < 2; i++)
      {
        WheelOut[i] = 0;
      }
    }
  }
  else // 底盘失能
  {
    for (int i = 0; i < 4; i++)
    {
      JointOut[i] = 0;
    }
    for (int j = 0; j < 2; j++)
    {
      WheelOut[j] = 0;
    }
    controller->j_length_keep_pid[R]->integral = 0;
    controller->j_length_keep_pid[L]->integral = 0;
    controller->j_roll_keep_pid->integral = 0;
  }
}

void WheelBipe_Infantry_Classdef::acutate()
{
  if (middleware->getRecData().chassis_state.gg_flag)
  {
    float gg_out[4] = {0, 0, 0, 0};
    middleware->setJointTorque(gg_out); // 关闭关节电机输出
    vTaskDelay(10);
    __set_FAULTMASK(1); // 关闭所有中断
    NVIC_SystemReset();
  }

  if (en_chassis == true) // 底盘使能
  {
    if (en_legForce == true) // 开腿力控
    {
      middleware->setJointTorque(JointOut);
    }
    else // 关腿位控
    {
      middleware->jointMotor[RF].setMotorAngle(controller->mp[RIGHT_JOINT]->target_joint.f_angle, 15, 2.5);
      middleware->jointMotor[RB].setMotorAngle(controller->mp[RIGHT_JOINT]->target_joint.b_angle, 15, 2.5);
      middleware->jointMotor[LF].setMotorAngle(controller->mp[LEFT_JOINT]->target_joint.f_angle, 15, 2.5);
      middleware->jointMotor[LB].setMotorAngle(controller->mp[LEFT_JOINT]->target_joint.b_angle, 15, 2.5);
    }
  }
  else
  {
    middleware->jointMotor[RF].setMotorAngle(controller->mp[RIGHT_JOINT]->target_joint.f_angle, 15, 2.5);
    middleware->jointMotor[RB].setMotorAngle(controller->mp[RIGHT_JOINT]->target_joint.b_angle, 15, 2.5);
    middleware->jointMotor[LF].setMotorAngle(controller->mp[LEFT_JOINT]->target_joint.f_angle, 15, 2.5);
    middleware->jointMotor[LB].setMotorAngle(controller->mp[LEFT_JOINT]->target_joint.b_angle, 15, 2.5);
  }
  middleware->sendDigitalBoard(WheelOut,
                               controller->state->current_av.yaw,
                               is_balance,
                               is_reset,
                               is_fallDown,
                               is_overturn,
                               right_manipulator.current_joint.f_angle,
                               right_manipulator.current_joint.b_angle,
                               right_manipulator.current_joint.position_x,
                               right_manipulator.current_joint.position_y);
}

/*云台控制底盘逻辑策略*/
void WheelBipe_Infantry_Classdef::gimbal_ctrl()
{
  if (target_length <= 0.16f)
  {
    en_legForce = false;
    en_wheel = true;
  }
  else
  {
    en_legForce = true;
    en_wheel = true;
  }
}

/*测试底盘遥控控制*/
void WheelBipe_Infantry_Classdef::debug_remoteCtrl()
{

  if (DR16.GetS1() == 2)
  {
    en_legForce = false;
    en_wheel = true;
    target_length = 0.15f;
  }
  else if (DR16.GetS1() == 3)
  {
    en_legForce = true;
    en_wheel = true;
    target_length = 0.22f;
  }
  else if (DR16.GetS1() == 1)
  {
    en_legForce = true;
    en_wheel = true;
    target_length = 0.3f;
  }
  else
  {
    en_legForce = false;
    en_wheel = true;
    target_length = 0.15f;
  }
  if (DR16.GetS2() == 1)
  {
    speed_scale = 3.f;
  }
  else if (DR16.GetS2() == 3)
  {
    speed_scale = 2.f;
  }
  else
  {
    speed_scale = 2.f;
  }

  // if(DR16.GetS2() == 1)
  // {
  //   jump_flag = true;
  // }
  // else
  // {
  //   jump_flag = false;
  // }

  // if(jump_flag == true && last_jump_flag == false)
  // {
  //   controller->state->flags.jump_flag = true;//拨挡上升沿会被触发一次
  // }
  // else
  // {
  //   controller->state->flags.jump_flag = false;
  // }

  // last_jump_flag = jump_flag;
}

/*判断是否真的平衡*/
void WheelBipe_Infantry_Classdef::check_balance()
{
  float body_pitch, leg_pitch;
  if (controller->state->flags.leap_flag)
  {
    body_pitch = 20.f;
    leg_pitch = 35.f;
  }
  else
  {
    body_pitch = 16.f;
    leg_pitch = 32.f;
  }
  if (is_balance == false)
  {
    if (abs(controller->state->current_pos.pitch) < 15.f * upper::degree2rad_ratio)
    {
      isBalanceCount--;
    }
    else
    {
      isBalanceCount = BALANCE_DELAY;
    }
    if (isBalanceCount < 0)
    {
      is_balance = true;
      isBalanceCount = BALANCE_DELAY;
    }
    else
    {
    }
  }
  else
  {
  }

  protectCount--;
  protectCount = upper::constrain(protectCount, 1, PROTECT_DELAY);

  if (is_fallDown == false && en_legForce == true)
  {
    if (protectCount == 1)
    {
      if (abs(controller->state->current_pos.roll) > 22.f * upper::degree2rad_ratio)
      {
        fallDownCount -= 5;
      }
      else if (abs(controller->state->current_pos.pitch) > body_pitch * upper::degree2rad_ratio || abs(controller->state->current_pos.roll) > 18.f * upper::degree2rad_ratio)
      {
        fallDownCount--;
      }
      else
      {
        fallDownCount = FALLDOWN_DELAY;
      }
    }
    else
    {
      if (abs(controller->state->current_pos.pitch) > body_pitch * upper::degree2rad_ratio)
      {
        fallDownCount--;
      }
    }
    if (fallDownCount < 0)
    {
      is_fallDown = true;
      fallDownCount = FALLDOWN_DELAY;
      protectCount = PROTECT_DELAY;
    }
  }
}

/*判断是否翻车*/
void WheelBipe_Infantry_Classdef::check_overturn()
{
  if (abs(controller->state->current_pos.pitch) > 50.f * upper::degree2rad_ratio || abs(controller->state->current_pos.roll) > 45.f * upper::degree2rad_ratio)
  {
    is_overturn = true;
  }
  else
  {
    is_overturn = false;
  }
}

void WheelBipe_Infantry_Classdef::params_config()
{
  user_params.roll_keep_kd = 0.03f + 0.0572f * 0.5f * (controller->mp[R]->current_joint.pendulum.length + controller->mp[R]->current_joint.pendulum.length);
  user_params.setPoint_kp = 0.33f - 0.47f * 0.5f * (controller->mp[R]->current_joint.pendulum.length + controller->mp[R]->current_joint.pendulum.length);
}
