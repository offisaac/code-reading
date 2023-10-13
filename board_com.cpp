#include "board_com.h"

/**
 * @brief  板间通信，发送云台数据包1
 * @param  NULL
 * @retval NULL
 * @author Lingzi_Xie
 */
 /*发送枪管热量之类的变量*/
void GimbalCom_Classdef::Send_GimbalPack1(QueueHandle_t *can_port,referee_Classdef* referee)
{
	send_data.ID = TOGIMBAL_PACK1_ID;
	send_data.DLC = 8;
	
	gimbal_rx_pack1.bullet_speed=(uint16_t)(referee->ShootData.bullet_speed*1000.0f);
	gimbal_rx_pack1.cooling_rate=referee->GameRobotState.shooter_id1_17mm_cooling_rate;
	gimbal_rx_pack1.cooling_limit=referee->GameRobotState.shooter_id1_17mm_cooling_limit;
	gimbal_rx_pack1.shooter_heat=referee->PowerHeatData.shooter_id1_17mm_cooling_heat;	
	
	memcpy(send_data.Data, (uint8_t*)&gimbal_rx_pack1, send_data.DLC);
	xQueueSend(*can_port, &send_data, 0);
}

/**
 * @brief  板间通信，发送云台数据包2
 * @param  NULL
 * @retval NULL
 * @author Lingzi_Xie
 */
void GimbalCom_Classdef::Send_GimbalPack2(QueueHandle_t *can_port,referee_Classdef* referee, int8_t _cap_voltage)
{
	send_data.ID = TOGIMBAL_PACK2_ID;
	send_data.DLC = 8;

	gimbal_rx_pack2.source_power_max		=referee->GameRobotState.classis_power_limit;
	gimbal_rx_pack2.shooter_speed_limit	=referee->GameRobotState.shooter_id1_17mm_speed_limit;
	gimbal_rx_pack2.robot_id						=referee->GameRobotState.robot_id;
	gimbal_rx_pack2.cap_status					=1;//sourcemanage类
	gimbal_rx_pack2.cap_voltage					=_cap_voltage;
	gimbal_rx_pack2.shooter_enable			=0;
	
	memcpy(send_data.Data, (uint8_t*)&gimbal_rx_pack2, send_data.DLC);
	xQueueSend(*can_port, &send_data, 0);
}

/**
 * @brief  板间通信，接收来自云台的数据包
 * @param  NULL
 * @retval NULL
 * @author Lingzi_Xie
 */
void GimbalCom_Classdef::Receive(CAN_COB* _pack)
{
	if(_pack->ID == TOCHASSIS_PACK1_ID)
	{
		memcpy((uint8_t*)&chassis_rx_pack, _pack->Data, _pack->DLC);
	}
}
