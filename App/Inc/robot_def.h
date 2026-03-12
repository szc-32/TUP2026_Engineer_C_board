#ifndef __ROBOT_DEF_H
#define __ROBOT_DEF_H

#include "board_def.h"

#define OFF_SET 0
#define ON_SET 1
/* 任务启动延时（ms） */
#define TASK_INIT_TIME 300

/* 弹丸类型 */
#define BULLET_17 0  // 17mm 弹丸
#define BULLET_42 1  // 42mm 弹丸
#define NO_BULLET 2  // 无弹丸
#define ROBOT_SHOOT_TYPE BULLET_42

/* 视觉输入滤波类型 */
#define HANDLE_LPF 0     // 低通滤波
#define HANDLE_KALMAN 1  // 卡尔曼滤波
#define Auto_Type HANDLE_LPF

/* 底盘结构类型 */
#define All_Mecanum 0         // 麦克纳姆底盘
#define All_Omnidirectional 1 // 全向轮底盘
#define CHASSIS_TYPE All_Mecanum

/* 功率控制开关 */
#define POWER_CONTROL_TYPE ON_SET

/* 云台/底盘共享运动参数 */
#define TURN_SPEED    0.04f

#define YAW_OFFSET 5400   // YAW 编码器零偏
#define PIT_OFFSET 7200   // PITCH 编码器零偏

#define MOTOR_TO_YAW_RADIO 2 // 电机到 YAW 传动比
#define MOTOR_TO_PIT_RADIO 1 // 电机到 PITCH 传动比
#define PIT_TRANSMISSION_RADIO 0.50f

#define MAX_YAW_RELATIVE  0.24906585f  
#define MIN_YAW_RELATIVE -0.24906585f 

/* PITCH 工作区间限幅（rad） */
#define MAX_PIT_RELATIVE 0.45f
#define MIN_PIT_RELATIVE -0.51f

/* 底盘速度限幅（m/s） */
#define NORMAL_MAX_CHASSIS_SPEED_X 2.0f
#define NORMAL_MAX_CHASSIS_SPEED_Y 1.5f

/* 底盘几何参数（mm） */
#define WHEEL_BASE 350
#define TRACK_WIDTH 300
#define CENTER_GIMBAL_OFFSET_X 0
#define CENTER_GIMBAL_OFFSET_Y 0
#define RADIUS_WHEEL 60
#define REDUCTION_RATIO_WHEEL 19.0f

/* 传感器使能开关 */
#define USE_SENSOR OFF_SET

/* 转盘参数 */
#define WHOLE_CIRCLE_BULLET_NUM 8 
/* 发射频率档位 */
#define LOW_FREQUENCE 2
#define MID_FREQUENCE 5
#define HIGH_FREQUENCE 8
#define INIT_FREQUENCE LOW_FREQUENCE

#define REVOLVER_NUM 1
#define MAIN_REVOLVER 0
#define ASSIST_REVOLVER 1

/* 二级供弹机构 */
#define FIRC_STAGE 1
#define FIRST_STAGE 0
#define SECONE_STAGE 1

/* 级间目标转速 */
#define FIRST_STAGE_RPM_SET  2000
#define SECOND_STAGE_RPM_SET 2000

/* 射手转速预设 */
#define LOW_RPM_SET 5000
#define MID_RPM_SET 7000
#define HIGH_RPM_SET 9000
#ifdef __cplusplus
extern "C"{
#endif
	
#ifdef __cplusplus

	
	
#endif
	
#ifdef __cplusplus
}
#endif

#endif
