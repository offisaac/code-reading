/**
  ***********************************************************************************
  * @file   : Service_Communication.cpp
  * @brief  : Communication support file.This file provides access ports to interface
  *           with connected devices.
  ***********************************************************************************
                                 ##### Port List #####
  =================================================================================
  |Port Name     Physical-Layer     Data-Link-Layer    Application-Layer    Number
  |————————————————————————————————————————
  |EXAMPLE_Port       CAN1               CAN                CUSTOM            0
  |CAN2_Port          CAN2               CAN                Custom            1
  |EBUG_Port         USART1             Custom              Custom            2
  |USART2_Port       USART2              DBUS               DJI-DR16          3
  *
**/
/* Includes ------------------------------------------------------------------*/
#include "internal.h"
#include "Balance_Chassis.h"
/* Private define ------------------------------------------------------------*/
void Task_CAN1Transmit(void *arg);
void Task_CAN2Transmit(void *arg);
void Task_CAN1Receive(void *arg);
void Task_CAN2Receive(void *arg);
void Task_UsartTransmit(void *arg);
void Task_UsartReceive(void *arg);

extern TaskHandle_t IMU_Handle;
/**
 * @brief  Initialization of communication service
 * @param  None.
 * @return None.
 */
void Service_Communication_Init(void)
{
  xTaskCreate(Task_CAN1Transmit, "Com.CAN1 TxPort", Small_Stack_Size, NULL, PrioritySuperHigh, &CAN1SendPort_Handle);
  xTaskCreate(Task_CAN2Transmit, "Com.CAN2 TxPort", Small_Stack_Size, NULL, PrioritySuperHigh, &CAN2SendPort_Handle);
  xTaskCreate(Task_CAN1Receive, "Com.CAN1 RxPort", Small_Stack_Size, NULL, PrioritySuperHigh, &CAN1ReceivePort_Handle);
  xTaskCreate(Task_CAN2Receive, "Com.CAN2 RxPort", Small_Stack_Size, NULL, PrioritySuperHigh, &CAN2ReceivePort_Handle);
  xTaskCreate(Task_UsartTransmit, "Com.Usart TxPort", Small_Stack_Size, NULL, PrioritySuperHigh, &UartTransmitPort_Handle);
  xTaskCreate(Task_UsartReceive, "Com.Usart RxPort", Normal_Stack_Size, NULL, PriorityRealtime, &UartReceivePort_Handle);
}

/*----------------------------------------------- CAN Manager ---------------------------------------------*/
/*Task Define ---------------------------*/
TaskHandle_t CAN1SendPort_Handle;
TaskHandle_t CAN2SendPort_Handle;
TaskHandle_t CAN1ReceivePort_Handle;
TaskHandle_t CAN2ReceivePort_Handle;

/*Function Prototypes--------------------*/
/**
 * @brief  Tasks for CAN Management.
 * @param  None.
 * @return None.
 */
void Task_CAN1Transmit(void *arg)
{
  /* Cache for Task */
  uint8_t free_can_mailbox;
  static CAN_COB CAN_TxMsg;
  /* Pre-Load for task */

  /* Infinite loop */

  for (;;)
  {
    /* CAN1 Send Port */
    if (xQueueReceive(CAN1_TxPort, &CAN_TxMsg, portMAX_DELAY) == pdPASS)
    {
      free_can_mailbox = HAL_CAN_GetTxMailboxesFreeLevel(&hcan1);
      /* Avoid the unused warning*/
      UNUSED(&free_can_mailbox);
      CANx_SendData(&hcan1, CAN_TxMsg.ID, CAN_TxMsg.Data, CAN_TxMsg.DLC);
    }
  }
}

/*
 * can2 transmit
 */
void Task_CAN2Transmit(void *arg)
{
  /* Cache for Task */
  uint8_t free_can_mailbox;
  static CAN_COB CAN_TxMsg;
  /* Pre-Load for task */

  /* Infinite loop */

  for (;;)
  {
    /* CAN1 Send Port */
    if (xQueueReceive(CAN2_TxPort, &CAN_TxMsg, portMAX_DELAY) == pdPASS)
    {
      free_can_mailbox = HAL_CAN_GetTxMailboxesFreeLevel(&hcan2);
      /* Avoid the unused warning*/
      UNUSED(&free_can_mailbox);
      CANx_SendData(&hcan2, CAN_TxMsg.ID, CAN_TxMsg.Data, CAN_TxMsg.DLC);
    }
  }
}

/*
 * can1 receive
 */
void Task_CAN1Receive(void *arg)
{
  static CAN_COB CAN_RxCOB;

  /* Infinite loop */
  for (;;)
  {
    /* update motor data from CAN1_RxPort */
    if (xQueueReceive(CAN1_RxPort, &CAN_RxCOB, portMAX_DELAY) == pdPASS)
    {
      if (CAN_RxCOB.ID == TOCHASSIS_PACK1_ID || CAN_RxCOB.ID == TOCHASSIS_PACK2_ID)
      {
        absChassis.CTRL_Data_Update(CAN_RxCOB);
      }
      else
      {
        Slider_Ctrl.updateMotorData(CAN_RxCOB);
      }
    }
  }
}

/*
 * can2 receive
 */
void Task_CAN2Receive(void *arg)
{
  static CAN_COB CAN_RxCOB;

  /* Infinite loop */
  for (;;)
  {
    /* update motor data from CAN1_RxPort */
    if (xQueueReceive(CAN2_RxPort, &CAN_RxCOB, portMAX_DELAY) == pdPASS)
    {
      absChassis.MotorUpdate(CAN_RxCOB);
    }
  }
}

/**
 * @brief  Callback function in CAN2 Interrupt
 * @param  None.
 * @return None.
 */
void User_CAN2_RxCpltCallback(CAN_RxBuffer *CAN_RxMessage)
{
  static CAN_COB CAN_RxCOB;
  BaseType_t xHigherPriorityTaskWoken;
  // Send To CAN Receive Queue
  if (CAN2_RxPort != NULL)
  {
    CAN_RxCOB.ID = CAN_RxMessage->header.StdId;
    CAN_RxCOB.DLC = CAN_RxMessage->header.DLC;
    memcpy(&CAN_RxCOB.Data, CAN_RxMessage->data, CAN_RxCOB.DLC);
    xQueueSendFromISR(CAN2_RxPort, &CAN_RxCOB,
                      &xHigherPriorityTaskWoken);
  }
}

/**
 * @brief  Callback function in CAN1 Interrupt
 * @param  None.
 * @return None.
 */
void User_CAN1_RxCpltCallback(CAN_RxBuffer *CAN_RxMessage)
{
  static CAN_COB CAN_RxCOB;
  BaseType_t xHigherPriorityTaskWoken;
  // Send To CAN Receive Queue
  if (CAN1_RxPort != NULL)
  {
    CAN_RxCOB.ID = CAN_RxMessage->header.StdId;
    CAN_RxCOB.DLC = CAN_RxMessage->header.DLC;
    memcpy(&CAN_RxCOB.Data, CAN_RxMessage->data, CAN_RxCOB.DLC);
    xQueueSendFromISR(CAN1_RxPort, &CAN_RxCOB,
                      &xHigherPriorityTaskWoken);
  }
}

/*---------------------------------------------- USART Manager --------------------------------------------*/
/*Task Define ---------------------------*/
TaskHandle_t UartTransmitPort_Handle;
TaskHandle_t UartReceivePort_Handle;

/*Function Prototypes--------------------*/
/**
* @brief  Tasks for USART Management.
          Attention:In this version we passing the pointer of data not copying
          data itself and we only have one buff zone, so user need to process
          the data as soon as possible in case of over-write.
* @param  None.
* @return None.
*/
void Task_UsartTransmit(void *arg)
{
  /* Cache for Task */
  UART_HandleTypeDef *pUart_x = NULL;
  static USART_COB Usart_TxCOB;
  /* Pre-Load for task */
  /* Infinite loop */
  for (;;)
  {
    /* Usart Receive Port*/
    if (xQueueReceive(USART_TxPort, &Usart_TxCOB, portMAX_DELAY) == pdPASS)
    {
      /* User Code Begin Here -------------------------------*/
      switch (Usart_TxCOB.port_num)
      {
			case 1:
        pUart_x = &huart1;
        break;
      case 2:
        pUart_x = &huart2;
        break;
      case 3:
        pUart_x = &huart3;
        break;
      case 4:
        pUart_x = &huart4;
        break;
      case 6:
        pUart_x = &huart6;
        break;
      default:
        pUart_x = NULL;
        break;
      }
      /* User Code End Here ---------------------------------*/
      if (pUart_x != NULL)
        HAL_UART_Transmit_DMA(pUart_x, (uint8_t *)Usart_TxCOB.address, Usart_TxCOB.len);
    }
  }
}

void Task_UsartReceive(void *arg)
{
  /* Cache for Task */
  USART_COB Usart_RxCOB;
  /* Pre-Load for task */
  /* Infinite loop */
  for (;;)
  {
    /* Usart Receive Port*/
    if (xQueueReceive(USART_RxPort, &Usart_RxCOB, portMAX_DELAY) == pdPASS)
    {
      /* User Code Begin Here -------------------------------*/
      switch (Usart_RxCOB.port_num)
      {
      case 1:
        absChassis.absIMU.processRecData((uint8_t *)Usart_RxCOB.address);
        break;
      default:
        break;
      }
      /* User Code End Here ---------------------------------*/
    }
  }
}

/**
 * @brief  Callback function in USART1 Interrupt, UpperMonitor
 * @param  None.
 * @return None.
 */
uint32_t User_UART1_RxCpltCallback(uint8_t *Recv_Data, uint16_t ReceiveLen)
{
  static USART_COB Usart_RxCOB;

  if (USART_RxPort != NULL)
  {
    Usart_RxCOB.address = Recv_Data;
    Usart_RxCOB.len = ReceiveLen;
    Usart_RxCOB.port_num = 1;

    xQueueSendFromISR(USART_RxPort, &Usart_RxCOB, 0);
  }

  return 0;
}

/**
 * @brief  Callback function in USART2 Interrupt
 * @param  Recv_Data		接收数组
 *	@param	ReceiveLen	接收数据长度
 * @return None.
 */
/*
  DR16
*/
uint32_t User_UART2_RxCpltCallback(uint8_t *Recv_Data, uint16_t ReceiveLen)
{
  static USART_COB Usart_RxCOB;

  if (USART_RxPort != NULL)
  {
    Usart_RxCOB.address = Recv_Data;
    Usart_RxCOB.len = ReceiveLen;
    Usart_RxCOB.port_num = 2;

    xQueueSendFromISR(USART_RxPort, &Usart_RxCOB, 0);
  }

  return 0;
}

uint32_t User_UART3_RxCpltCallback(uint8_t *Recv_Data, uint16_t ReceiveLen)
{
  return 0;
}

uint32_t User_UART4_RxCpltCallback(uint8_t *Recv_Data, uint16_t ReceiveLen)
{
  return 0;
}

uint32_t User_UART5_RxCpltCallback(uint8_t *Recv_Data, uint16_t ReceiveLen)
{
  return 0;
}

/*裁判系统数据直接通过任务通知进行接收解包*/
uint32_t Referee_recv_Callback(uint8_t *Recv_Data, uint16_t ReceiveLen)
{
  static BaseType_t *pxHigherPriorityTaskWoken;
  static USART_COB referee_pack;

  if (Referee_Handle != NULL && ReceiveLen < 256)
  {
    referee_pack.address = Recv_Data;
    referee_pack.len = ReceiveLen;
    xTaskNotifyFromISR(Referee_Handle, (uint32_t)&referee_pack, eSetValueWithOverwrite, pxHigherPriorityTaskWoken);
  }
  return 0;
}

void User_VirtualComRecCpltCallback(uint8_t *Recv_Data, uint16_t ReceiveLen)
{
  /* Avoid warning */
  UNUSED(Recv_Data);
  UNUSED(ReceiveLen);
}
/*---------------------------------------------- EXTI Manager ---------------------------------------------*/
/************************ COPYRIGHT(C) SCUT-ROBOTLAB **************************/
