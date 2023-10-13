/**
  ******************************************************************************
  * @file   Gimbal_Com.h
  * @author Lingzi_Xie 1357657340@qq.com
  * @brief  Communicaiton with Gimbal
  
  ******************************************************************************
  * @note
	 -# 板间通信暂未进行优化，先与现有云台代码进行匹配
  * @note
  *  - Before running your devices, just do what you want ~ !
  *  - More devices or using other classification is decided by yourself ~ !
 */
 
#ifndef  __BOARD_COM_H_
#define  __BOARD_COM_H_

#ifdef  __cplusplus
extern "C"{
#endif
 
#ifdef  __cplusplus
}

/* Includes ------------------------------------------------------------------*/
#include "SRML.h"
#include "FreeRTOS.h"
#include "queue.h"

/* Private macros ------------------------------------------------------------*/
#define TOGIMBAL_PACK1_ID 			0x221							
#define TOGIMBAL_PACK2_ID				0x220							
#define TOCHASSIS_PACK1_ID				0x222
#define TOCHASSIS_PACK2_ID				0x223

/* Private type --------------------------------------------------------------*/
/* 板间通信，小发射数据包 */
#pragma pack(1)
typedef struct{	
	uint16_t bullet_speed;
	uint16_t cooling_rate;
	uint16_t cooling_limit;
	uint16_t shooter_heat;

}_gimbal_RxPack1;
#pragma pack()	

/* 板间通信，状态数据包 */
#pragma pack(1)	
typedef struct{	
	uint8_t source_power_max;
	uint8_t shooter_speed_limit;
	uint8_t robot_id;
	uint8_t cap_status;
	uint8_t cap_voltage;
	uint8_t shooter_enable;
	uint16_t chassis_flags;

}_gimbal_RxPack2;
#pragma pack()	

/* 板间通信，接收目标速度及控制状态 */
#pragma pack(1)	
typedef struct{
	int16_t chassis_speed_y;
	int16_t chassis_speed_x;
	int16_t chassis_speed_z;
	uint16_t chassis_mode;
	
}_chassis_RxPack;

#pragma pack()	
/* Exported macros -----------------------------------------------------------*/
/* Exported types ------------------------------------------------------------*/
class GimbalCom_Classdef
{
	public:
	_gimbal_RxPack1 gimbal_rx_pack1;
	_gimbal_RxPack2 gimbal_rx_pack2;
	_chassis_RxPack chassis_rx_pack;
	
	void Send_GimbalPack1(QueueHandle_t *can_port,referee_Classdef* referee);
	void Send_GimbalPack2(QueueHandle_t *can_port,referee_Classdef* referee,int8_t _cap_voltage);
	
	void Receive(CAN_COB* _pack);
	
	private:
	CAN_COB send_data;
};
/* Exported variables --------------------------------------------------------*/
/* Exported function declarations --------------------------------------------*/

#endif

#endif

/************************ COPYRIGHT(C) SCUT-ROBOTLAB **************************/
