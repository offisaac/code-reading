/**
 ******************************************************************************
 * Copyright (c) 2019 - ~, SCUT-RobotLab Development Team
 * @file board_com.cpp
 * @author mzh
 * @brief 板间通信
 * @date 2022-02-15
 * @version 2.0
 * @par Change Log：
 * <table>
 * <tr><th>Date <th>Version <th>Author <th>Description
 * <tr><td>2019-06-12 <td> 1.0 <td>S.B. <td>Creator
 * </table>
 *
 ==============================================================================
 ##### How to use this driver #####
 ==============================================================================
 @note
 -#
 ******************************************************************************
 * @attention
 *
 * if you had modified this file, please make sure your code does not have many
 * bugs, update the version Number, write dowm your name and the date, the most
* important is make sure the users will have clear and definite understanding
 * through your new brief.
 *
 * <h2><center>&copy; Copyright (c) 2019 - ~, SCUT-RobotLab Development Team.
 * All rights reserved.</center></h2>
 ******************************************************************************
 */
/* Includes ------------------------------------------------------------------*/
#include "board_com.h"
/* Exported macros -----------------------------------------------------------*/
extern QueueHandle_t CAN2_TxPort;
/* Private define ------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
/* Private type --------------------------------------------------------------*/
/* Private function declarations ---------------------------------------------*/
/* function prototypes -------------------------------------------------------*/
/**
 * @brief 接收小发射数据
 * @param None
 * @retval None
 */
void BoardCom_Classdef::Booster_Receive(uint8_t *Rec_Data)
{
	memcpy((uint8_t *)&rx_pack1, Rec_Data, 8);
}
/**
 * @brief 接收底盘控制数据
 * @param None
 * @retval None
 */
void BoardCom_Classdef::Chassis_Receive(uint8_t *Rec_Data)
{
	memcpy((uint8_t *)&rx_pack2, Rec_Data, 8);
}
/**
 * @brief 发送数据给底盘
 * @param None
 * @retval None
 */
void BoardCom_Classdef::Chassis_Send_Pack1(int16_t _speed_y, int16_t _speed_x,
									 int16_t _speed_z, uint8_t _reset_state)
{
	CAN_COB send_data;
	send_data.ID = CHASSIS_RXPACK1;
	send_data.DLC = 8;
	if (!_reset_state)
	{
		// 平衡步兵不需要横向移动
		tx_pack1.chassis_speed_y = _speed_y;
		tx_pack1.chassis_speed_x = _speed_x;
		tx_pack1.chassis_speed_z = _speed_z;
	}
	else
	{
		tx_pack1.chassis_speed_x = 0;
		tx_pack1.chassis_speed_y = 0;
		tx_pack1.chassis_speed_z = 0;
	}
	memcpy(send_data.Data, (uint8_t *)&tx_pack1, 8);
	xQueueSend(CAN2_TxPort, &send_data, 0);
}

void BoardCom_Classdef::Chassis_Send_Pack2(float _rotation_angle)
{
	CAN_COB send_data;
	send_data.ID = CHASSIS_RXPACK2;
	send_data.DLC = 4;
	tx_pack2.chassis_rotation_angle = _rotation_angle;
	memcpy(send_data.Data, (uint8_t *)&tx_pack2,4);
	xQueueSend(CAN2_TxPort, &send_data, 0);
}
/**
 * @brief 设置不同类型步兵标志位
 * @param None
 * @retval None
 */
void BoardCom_Classdef::Set_BalanceInfanty_Flag(uint8_t _lost_ctrl,
												uint8_t _unlimited_state,
												uint8_t _ascent_state,
												uint8_t _leap_state,
												uint8_t _rotation_state,
												uint8_t _bulletbay_state,
												uint8_t _ui_reset_flag,
												uint8_t _vision_mode_flag1,
												uint8_t _vision_mode_flag2,
												uint8_t _self_rescue_state,
												uint8_t _enable_cmd,
												uint8_t _sliding_remake,
												uint8_t _turn90degrees,
												uint8_t _gg_flag,
												uint8_t _vision_can_shoot,
												uint8_t _fri_state)
{
	tx_pack1.chassis_mode = _lost_ctrl | (_unlimited_state << 1) |
						   (_rotation_state << 2) | (_leap_state << 3) |
						   (_bulletbay_state << 4) | (_ascent_state << 5) |
						   (_ui_reset_flag << 6) | (_vision_mode_flag1 << 7) |
						   (_vision_mode_flag2 << 8) | (_enable_cmd << 9) |
						   (_self_rescue_state << 10) | (_sliding_remake << 11) |
						   (_turn90degrees << 12) | (_gg_flag << 13) | (_vision_can_shoot << 14) | (_fri_state << 15);
}
/************************ COPYRIGHT(C) SCUT-ROBOTLAB**************************/
