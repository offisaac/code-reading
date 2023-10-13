/**
 ******************************************************************************
 * @file   task.cpp
 * @brief  freertos task running file.
 ******************************************************************************
 * @note
 *  - Before running your devices, just do what you want ~ !
 *  - More devices or using other classification is decided by yourself ~ !
 ===============================================================================
                                   Task List
 ===============================================================================
 * <table>
 * <tr><th>Task Name     <th>Priority          <th>Frequency/Hz    <th>Stack/Byte
 * <tr><td>              <td>                  <td>                <td>
 * </table>
 *
*/

/* Includes ------------------------------------------------------------------*/
#include "internal.h"
#include "Balance_Chassis.h"
#include <Middlewares/UpperMonitor/UpperMonitor.h>
/* Private define ------------------------------------------------------------*/
TaskHandle_t DjiMotor_Handle;
TaskHandle_t Source_Handle;
TaskHandle_t Referee_Handle;
TaskHandle_t Referee_UI;
TaskHandle_t Openlog_send_Handle;
TaskHandle_t Log_Handle;
TaskHandle_t Motor_State_Handle;
/* Private function declarations ---------------------------------------------*/
void tskDjiMotor(void *arg);
void tskSource(void *arg);
void tskRefereeRx(void *arg);
void tskRefereeUI(void *arg);
void tskOpenlog_send(void *arg);
void tskLog(void *arg);
void Motor_State(void *arg);
/* Function prototypes -------------------------------------------------------*/
void Send2Gimbal(QueueHandle_t *canHandle);

/**
 * @brief  Initialization of device management service
 * @param  None.
 * @return None.
 */
void Service_Devices_Init(void)
{
  xTaskCreate(tskSource, "App.Source", Normal_Stack_Size, NULL, PriorityAboveNormal, &Source_Handle);
  xTaskCreate(tskRefereeRx, "App.Referee", Normal_Stack_Size, NULL, PriorityNormal, &Referee_Handle);
  xTaskCreate(tskRefereeUI, "App.RefereeUI", Normal_Stack_Size, NULL, PriorityBelowNormal, &Referee_UI);
  //xTaskCreate(tskOpenlog_send, "App.Openlog send", Small_Stack_Size, NULL, PriorityAboveNormal, &Openlog_send_Handle);
  xTaskCreate(tskLog, "App.Log", Small_Stack_Size, NULL, PriorityAboveNormal, &Log_Handle);
	xTaskCreate(tskDjiMotor, "App.Motor", Large_Stack_Size, NULL, PriorityAboveNormal, &DjiMotor_Handle);
  xTaskCreate(Motor_State, "Motor_State_Check", Small_Stack_Size, NULL, PriorityAboveNormal, &Motor_State_Handle);
}

/**
 * @brief <freertos> 电源管理任务
 */
void tskSource(void *arg)
{
  digital_Power.digital_Power_Init();
  for (;;)
  {
    vTaskDelay(1); //不要改任务周期，也不要用vTaskDelayUntil
    // 传入1、底盘功率限制，2、当前缓冲能量
    digital_Power.Update(Referee.GameRobotState.classis_power_limit, Referee.PowerHeatData.chassis_power_buffer);
    // 传入当前血量，防止死亡后电容供电
    digital_Power.digital_Power_Control(Referee.GameRobotState.remain_HP);
  }
}

/**
 * @brief <freertos> 大疆电机控制任务
 */
void tskDjiMotor(void *arg)
{
  /*	pre load for task	*/
  HAL_UART_DeInit(&huart1);
  vTaskDelay(200);
  HAL_UART_Init(&huart1);

  absChassis.bindCanHandle(&CAN2_TxPort);

  absChassis.wheelMotor[LEFT].motor->writePidToRAM(50, 50, 75, 25, 125, 25);
  vTaskDelay(5);
  absChassis.wheelMotor[RIGHT].motor->writePidToRAM(50, 50, 75, 25, 125, 25);

  Slider_Ctrl.importQueueHander(CAN1_TxPort);
  Slider_Ctrl.init();

  TickType_t xLastWakeTime_t;
  xLastWakeTime_t = xTaskGetTickCount();

  for (;;)
  {
    /* wait for next circle */

    vTaskDelayUntil(&xLastWakeTime_t, 2);
    absChassis.absIMU.update();
    absChassis.Link_Check();
    balance_infantry.Chassis_Ctrl();
    
    Send2Gimbal(&CAN1_TxPort);

  }
}

/**
 * @brief  获取电机当前状态
 * @param  None.
 * @return None.
 */
void Motor_State(void *arg)
{
  for (;;)
  {
    /* wait for next circle */
    vTaskDelay(20);
    absChassis.Motor_State_Check();
  }
}

/**
 * @brief <freertos> 裁判系统数据读取任务
 */
void tskRefereeRx(void *arg)
{
  /* Pre-Load for task */
  static USART_COB *referee_pack;
  static TickType_t xLastWakeTime_t = xTaskGetTickCount();

  /* Infinite loop */
  for (;;)
  {
    if (xTaskNotifyWait(0x00000000, 0xFFFFFFFF, (uint32_t *)&referee_pack, 0) == pdTRUE)
    {
      Referee.unPackDataFromRF((uint8_t *)referee_pack->address, referee_pack->len);
    }
    /* Pass control to the next task */
    vTaskDelayUntil(&xLastWakeTime_t, 1);
  }
}

uint32_t get_refeeretime();
/**
 *	@brief	UI绘制
 */
void tskRefereeUI(void *arg)
{
  //起始绘制时的绘制次数。若服务器丢包率较高，该值可适当给大
  static uint8_t enable_cnt = 30;

  //初始化裁判系统
  Referee.Init(&huart6, get_refeeretime);

  //图传稳定需要一段时间的延时
  vTaskDelay(500);
  Referee.clean_all();

  vTaskDelay(2000);

  for (;;)
  {
     vTaskDelay(2000);
  //   if (balance_infantry.absChassis.getCtrlData().ui_reset_flag == 1) //重画UI标志位
  //   {
  //     enable_cnt = 20;
  //     balance_infantry.absChassis.getCtrlData().ui_reset_flag = 0;
  //   }
  //   else
  //   {
  //   }

  //   if (enable_cnt)
  //   {
  //     enable_cnt--;
  //     //车界线、下坠标尺绘制
  //     // balance_infantry.Referee.Draw_Robot_Limit(180, 80, 961, 3, YELLOW, ADD_PICTURE);

  //     //绘制电容剩余能量
  //     balance_infantry.Referee.Draw_Cap_Energy(balance_infantry.Source_Cap_Voltage, 28, 12, enable_cnt, UI_X_VeryLeft, UI_Y_High);

  //     //雷达站策略集部分
  //     balance_infantry.Referee.Draw_Auto_Lock_Range(balance_infantry.absChassis.getCtrlData().vision_can_shoot, balance_infantry.auto_mode, UI_X_Middle, UI_Y_Middle, 700, 500, 2);
  //     // balance_infantry.Referee.HP_UI(balance_infantry.Gimbal_ID, 960, 220, 150, 30, WHITE, enable_cnt);

  //     //平衡步停车区域绘制
  //     balance_infantry.Referee.Draw_Balance_Stop_Erea(0, enable_cnt, -balance_infantry.balance_controller.current_linearSpeed.y, 3.4, 0.1, 1);

  //     //平衡步姿态绘制
  //     balance_infantry.Referee.Draw_Balance_State(0, 0, PI/2.f, 700, 120, 100, GREEN);
  //   }
  //   else
  //   {
  //     //绘制电容剩余能量
  //     balance_infantry.Referee.Draw_Cap_Energy(balance_infantry.Source_Cap_Voltage, 28, 12, enable_cnt, UI_X_VeryLeft, UI_Y_High);
  //     balance_infantry.Referee.Draw_Boost(balance_infantry.absChassis.getCtrlData().unlimited_state | balance_infantry.absChassis.getCtrlData().leap_state, UI_X_VeryLeft, UI_Y_LittleHigh, 10, PINK);
  //     balance_infantry.Referee.Draw_Bullet(balance_infantry.absChassis.getCtrlData().bulletbay_state, 1800, 740, 8, GREEN); //绘制弹仓开启状态
  //     balance_infantry.Referee.Draw_BulletBay_Open(balance_infantry.absChassis.getCtrlData().bulletbay_state, UI_X_Middle, UI_Y_LittleHigh, PINK);
  //     balance_infantry.Referee.Draw_Spin(balance_infantry.absChassis.getCtrlData().rotation_state, 1400, 740, 10, BLUE); //绘制小陀螺开启状态
  //     balance_infantry.Referee.Draw_CoolingHeat(balance_infantry.Referee.PowerHeatData.shooter_id1_17mm_cooling_heat, balance_infantry.Referee.GameRobotState.shooter_id1_17mm_cooling_limit, UI_X_Middle, UI_Y_Middle, 50, 5);

  //     balance_infantry.Referee.Draw_Auto_Lock_Range(balance_infantry.absChassis.getCtrlData().vision_can_shoot, balance_infantry.auto_mode, UI_X_Middle, UI_Y_Middle, 700, 500, 2);

  //     //平衡步停车区域绘制
  //     balance_infantry.Referee.Draw_Balance_Stop_Erea(0, enable_cnt, -balance_infantry.balance_controller.current_linearSpeed.y, 4.5, 0.1, 1);
  //     //平衡步小陀螺撞墙提示
  //     // balance_infantry.Referee.Draw_Rotation_Crash(4,960,720,true);

  //     // // infantry.Referee.HP_UI(infantry.Gimbal_ID, 960, 220, 150, 30, WHITE, enable_cnt);
  //     balance_infantry.Referee.Draw_Fri_State(balance_infantry.absChassis.getCtrlData().fri_state, UI_X_VeryLeft, UI_Y_Middle + 75);
  //     // static uint8_t Data;
  //     // infantry.Referee.CV_ToOtherRobot(infantry.Gimbal_ID, &Data, ROBOT_COM_PACK);
  //     //     if (!balance_infantry.absChassis.getCtrlData().turn90degrees)
  //     balance_infantry.Referee.Draw_Balance_State(0,balance_infantry.balance_controller.current_pos.pitch, - balance_infantry.absChassis.getCtrlData().chassis_rotation_angle / 180.0f * PI + PI / 2.f, 700, 120, 100, GREEN);
  //     //      else
  //     //        balance_infantry.Referee.Draw_Balance_State(balance_infantry.balance_controller.current_pos.pitch/180.0f*3.14f,(balance_infantry.balance_controller.current_linearSpeed.y-90)/180*3.14f,700,120,100,GREEN);
  //   }
  //   //			Referee.Engineer_AutoMode_Update(1,0,100,500,50);
  //   //			Referee.Draw_Fri_State(4,300,540);
  //   //			Referee.Draw_Cap_Energy(SourceManage.capObj.Voltage, 24, 13, enable_cnt, 380,800);
  //   //			Referee.Draw_Balance_State(0,1.57,950,200,100,PINK);
  }
}

void tskOpenlog_send(void *arg)
{
  /* Pre-Load for task */
  TickType_t xLastWakeTime_t;
  xLastWakeTime_t = xTaskGetTickCount();
  /* Infinite loop */
	vTaskDelay(1000);
  for (;;)
  {
    /* wait for next circle */
    vTaskDelayUntil(&xLastWakeTime_t, 1000);
//    openlog.Send();
  }
}

void tskLog(void *arg)
{
  /* Pre-Load for task */
  vTaskDelay(1000 * 3);
  openlog.new_file("table_%d.csv", 2);
  openlog.append_file("table_%d.csv", 2);
  openlog.record("Time,ID,zSpeed_t,rotation,zSpeed_c,gimbalGetZ,state,bulletSpeed\r");
  openlog.push_buff();
	openlog.Send();
	TickType_t xLastWakeTime_t;
	xLastWakeTime_t = xTaskGetTickCount();
	vTaskDelay(1000);
  /* Infinite loop */
  for (;;)
  {
    /* wait for next circle */
    vTaskDelayUntil(&xLastWakeTime_t, 100);
    openlog.record("%d,%d,%d,%d,%d,%d,%d,%d\r",(int16_t)Get_SystemTimer()/1000000,
                                               (int16_t)Referee.GameRobotState.robot_id,
                                               (int16_t)(balance_infantry.balance_controller.target_angularSpeed.yaw),
                                               (int16_t)absChassis.getCtrlData().rotation_state,
                                               (int16_t)(balance_infantry.balance_controller.current_angularSpeed.yaw),
                                               (int16_t)(absChassis.getCtrlData().target_speed_z * 180 / PI),
                                               (int16_t)balance_infantry.machine_mode,
                                               (int16_t)Referee.ShootData.bullet_speed*1000.0f);
 		openlog.push_buff();
		openlog.Send();
  }
}

/**
 * @brief  得到s单位
 * @note
 * @param
 * @return
 * @retval  None
 */
uint32_t get_refeeretime()
{
  return xTaskGetTickCount() * 1000;
}

void Send2Gimbal(QueueHandle_t *canHandle)
{
    /* 发送CAN包到云台 */
    Board_Com.gimbal_rx_pack2.chassis_flags &= 0xFFFE; //发送标志位除了第一位全部置1
    if (balance_infantry.machine_mode == 2)
    {
        Board_Com.gimbal_rx_pack2.chassis_flags |= 0x0001; //第一位置1（底盘正常）
    }

    if (Referee.GameState.stage_remain_time < 240 && Referee.GameState.stage_remain_time != 0)
        Board_Com.gimbal_rx_pack2.chassis_flags |= 0x0001 << 3; //第四位置1
    else
        Board_Com.gimbal_rx_pack2.chassis_flags &= ~(0x1 << 3); //第四位置0
    Board_Com.Send_GimbalPack1(canHandle, &Referee);
    Board_Com.Send_GimbalPack2(canHandle, &Referee, digital_Power.unit_DPW_data.Vcap * 7.f);

}
