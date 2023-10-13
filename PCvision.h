/**
 ******************************************************************************
 * Copyright (c) 2019 - ~, SCUT-RobotLab Development Team
 * @file    PCvision.h
 * @author  mzh
 * @brief   视觉通信头文件
 ******************************************************************************
 * @attention
 *
 * if you had modified this file, please make sure your code does not have any
 * bugs, update the version Number, write dowm your name and the date. The most
 * important thing is make sure the users will have clear and definite under-
 * standing through your new brief.
 *
 * <h2><center>&copy; Copyright (c) 2019 - ~, SCUT-RobotLab Development Team.
 * All rights reserved.</center></h2>
 ******************************************************************************
 */
#ifndef __PCvision_H__
#define __PCvision_H__

#ifdef __cplusplus
/* Includes ------------------------------------------------------------------*/
#include <stdint.h>
#include <math.h>
#include "SRML.h"
#include "Gimbal.h"
/* Private macros ------------------------------------------------------------*/
/*vision flag*/
#define NO_ARMOR 0
#define CAN_SHOOT 1
#define VISION_HEAD 0x44
#define VISION_END 0x55

#define VISION_PITCH_SCALE 1.0f
#define VISION_YAW_SCALE 1.0f

#define DELAY_TRANSIMISION_TIME 15.0f // 时序核对延时时间

#define VISION_DATA_MODE 1 //视觉数据类型：0为绝对式	1为增量式
/* Private type --------------------------------------------------------------*/

/**************************************************************************
 * @brief	视觉传过来的协商一致的数据包
 * @note	 	target_mode: 有目标 -- 1
 *											 无目标 -- 0
 *					shoot_mode:  允许射击 -- 1
 *											 禁止射击 -- 0
 **************************************************************************/
#pragma pack(1)
typedef struct
{
	uint8_t target_mode; // 视觉控制云台响应的标志位，有目标则接管控制（具体响应与否可由操作手选择）
	uint8_t shoot_mode;	 // 视觉接管发弹的标志位 (具体发弹策略可由电控进一步调整)
	float yawData;		 // yaw 角度 -- 正值表示枪口往右偏移
	float pitchData;	 // pitch 角度 -- 正值表示枪口往下偏移
	// uint8_t type;				// 目标车数字
	uint8_t End; // 结束位标志，每次发送的值应不同，电控可据此判断通信是否正常
} PackFromVisionDef;
#pragma pack()

/**************************************************************************
 * @brief	通过共用内存将视觉传过来的数据自动同步到PackFromVision
 * @note		使用UsartData接收视觉的数据，PackFromVision中对应的参数将被修改
 **************************************************************************/
typedef union
{
	uint8_t UsartData[11];
	PackFromVisionDef PackFromVision;
} PackFromVisionUnionDef;

/**************************************************************************
 * @brief	返回给视觉的协商一致的数据包
 * @note		Null
 **************************************************************************/
#pragma pack(1)
typedef struct
{
	uint8_t head; // 包头
	float pitch_angle;
	float yaw_angle;
	float pitch_speed;
	float yaw_speed;
	uint8_t mode;		  // 功能模式
	uint8_t R_key;		  // 右键
	uint8_t color;		  // 对方颜色
	uint8_t bullet_speed; // 弹速
	uint8_t end;		  // 包尾
} PackToVision_Def;
#pragma pack()
// #pragma pack(show)

/**************************************************************************
 * @brief	通过共用内存将反馈的数据自动同步到PackToVision
 * @note		通过修改PackToVision中的参数，再将UsartData数组发送出即可
 **************************************************************************/
typedef union
{
	uint8_t UsartData[22];
	PackToVision_Def PackToVision;
} PackToVisionUnionDef;

/**************************************************************************
 * @brief	通过共用内存及函数修改float类型的大小端
 * @note
 **************************************************************************/
typedef union
{
	float f;
	char c[4];
} Float_Conv;

/*用于指示灯判断连接状态*/
enum PCvisionStatusDef
{
	Connected = 1,
	Unconnected = 0,
};
/* Exported macros -----------------------------------------------------------*/
/* Exported types ------------------------------------------------------------*/
/*存放数组类*/
template <int Length>
class PreviousDataClass
{
public:
	/* 输入数据 */
	void InputData(float NowData)
	{
		uint8_t i;
		/*数据移位*/
		for (i = Length - 1; i > 0; i--)
			DataTab[i] = DataTab[i - 1];
		DataTab[0] = NowData;
	}
	/* 获取数据 */
	float GetPreviousData(uint8_t T)
	{
		/*防止地址访问越界*/
		if (T >= Length)
			return 0;
		return DataTab[T];
	}
	float GetPreviousData(float T)
	{
		uint8_t intTime = (uint8_t)T;	 // 时间的整数部分
		float decimalTime = T - intTime; // 时间的小数部分
		/*防止地址访问越界*/
		if (intTime + 1 >= Length)
			return 0;
		return DataTab[intTime] + decimalTime * (DataTab[intTime + 1] - DataTab[intTime]);
	}

private:
	float DataTab[Length];
};

/*数据对齐：循环队列*/
template <uint16_t length>
class Recorder
{
public:
	Recorder() : currentIndex(0) {}

	void addFrame(const float frame)
	{
		currentIndex = (currentIndex + 1) % length;
		frames[currentIndex] = frame;
	}

	float getFrames(uint16_t last_frame_num)
	{
		if (last_frame_num >= length)
		{
			return NULL;
		}

		uint16_t Index = (currentIndex >= last_frame_num) ? (currentIndex - last_frame_num) : (length + (currentIndex - last_frame_num));

		return frames[Index];
	}

private:
	float frames[length];
	uint16_t currentIndex;
};

/*视觉通信类*/
class PCvision_Classdef
{
public:
	/*接口函数*/
	uint32_t GetViaionData(uint8_t *Recv_Data, uint16_t ReceiveLen); // 接收中断函数
	void SendGimbleStatus();										 // 发给视觉
	void Status_Update(uint8_t _pcVisionMode,
					   float _pitch_current,
					   float _yaw_current,
					   float _pitch_angular_speed,
					   float _yaw_angular_speed,
					   uint8_t *_maxSpeed,
					   uint8_t *_robotID
					   /*uint8_t *_big_rune_mode*/);

	/*中断接收的数据*/
	PackFromVisionUnionDef PackFromVisionUnion;
	PackToVision_Def PackToVision;
	PCvisionStatusDef PCvisionStatus; // 状态指示灯用
	/*pitch、yaw数组*/
	PreviousDataClass<50> PitchPerviousData;
	PreviousDataClass<50> YawPerviousData;
	PreviousDataClass<50> PitchAngularSpeed_PerviousData;
	PreviousDataClass<50> YawAngularSpeed_PerviousData;
	/*陀螺仪数据*/
	Recorder<20> ImuYawRecord;
	Recorder<20> ImuPitchRecord;
	/*打符上升沿标志位*/
	uint8_t last_shoot_mode, shoot_mode, have_fire;
	int count; // 上升冷却沿计时
private:
	uint8_t lostDelay; // 判断自瞄是否连接
	/*传入的参数*/
	uint8_t pcVisionMode;
	float pitch_current, yaw_current;
	float pitch_angular_speed, yaw_angular_speed;
	int16_t maxSpeed;
	int8_t robotID;
};
/* Exported function declarations --------------------------------------------*/
#endif

#ifdef __cplusplus
extern "C"
{
#endif
	/* Exported macros -----------------------------------------------------------*/
	/* Exported types ------------------------------------------------------------*/

	/* Exported function declarations --------------------------------------------*/
}
#endif
/************************ COPYRIGHT(C) SCUT-ROBOTLAB **************************/
