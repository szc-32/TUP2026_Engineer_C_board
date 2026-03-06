/**
 * @file motor_def.h
 * @brief Public declarations for Modules layer.
 */

#ifndef __MOTOR_DEF_H
#define __MOTOR_DEF_H

#include "pid.h"
#include "ladrc_feedforward.h"
#include "device_monitor.h"

// 挂载电机实例上限（用于静态数组大小）
#define TOTAL_MOTOR_SUM 16
//挂载瓴控电机上限
#define TOTAL_LK_MOTOR_SUM 4
//挂载达妙电机上限
#define TOTAL_DM_MOTOR_SUM 6

// 常用减速比倒数（用乘法替代除法，减少实时环中的除法开销）
#define ONE_FOURTEEN 0.07142857f  //1÷14
#define ONE_NINETEEN 0.05263158f  //1÷19
#define ONE_OUT_OF_THIRTYSIX 0.02777778f  //1÷36
#define ONE_OUT_OF_SEVENTYONE 0.01408451f  //1÷71
#ifdef __cplusplus
extern "C"{
#endif

#ifdef __cplusplus
//电机旋转方向--(GM6020：逆时针为正)
enum Motor_Rotate_Direction_e
{
	STOP = 0,	POSITIVE_DIRECT = 1,
	NEGATIVE_DIRECT = -1,
};

//电机类型
enum Motor_Type_e
{
	NO_MOTOR,
	M3508,
	AM3508,
	GM6020,
	M2006,
	DM4310,
	DM4340,
	DM10010L,
	DM8009P
};

//控制器类型
enum Control_Type_e
{
	OPEN_LOOP,
	//PID
	SINGLE_LOOP,
	CASCADE_LOOP,
	//LADRC
	LADRC_CONTROL,
	LADRC_FDW_CONTROL,
};

//减速器类型
enum Reduction_Drive_Type_e
{
	DIRECT_DRIVE,
	RATIO_1_TO_14,
	RATIO_1_TO_19,
	RATIO_1_TO_36,
	RATIO_1_TO_71,
};

//控制器
typedef struct
{
	//PID
	PID_t speed_PID;
	PID_t angle_PID;
	//LADRC
	LADRC_t ladrc;
	LADRC_FDW_t ladrc_fdw;
	//最外环参数
	fp32 target_value,now_value,pid_speed;
	int16_t send_current;
}Motor_Controller_s;
		
#endif
	
#ifdef __cplusplus
}
#endif

#endif

