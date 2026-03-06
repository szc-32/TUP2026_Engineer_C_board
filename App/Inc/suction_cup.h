/**
 * @file suction_cup.h
 * @brief Public declarations for App layer.
 */

#ifndef SUCTION_CUP_H
#define SUCTION_CUP_H

#include "message_center.h"
#include "user_lib.h"
#include "robot_def.h"
#include "PID.h"
#include "motor.h"

//任务初始�?空闲一段时�?
#define SUCTION_CUP_TASK_INIT_TIME 201

//电机码盘值最大以及中�?
#define HALF_ECD_RANGE  4096
#define ECD_RANGE       8191

//电机编码值转化成角度�?
#define MOTOR_ECD_TO_RAD 0.000766990394f   // 2*PI/8192

#define PITCH_SPEED_PID_KP 1500.0f
#define PITCH_SPEED_PID_KI 0
#define PITCH_SPEED_PID_KD 500.0f

#define YAW_SPEED_PID_KP 1200.0f
#define YAW_SPEED_PID_KI 0
#define YAW_SPEED_PID_KD 500

#define YAW_B_SPEED_PID_KP 3000.0f
#define YAW_B_SPEED_PID_KI 0
#define YAW_B_SPEED_PID_KD 1000

#define ROLL_SPEED_PID_KP 1500.0f
#define ROLL_SPEED_PID_KI 0
#define ROLL_SPEED_PID_KD 500.0f

#define PITCH_ECD_ANGLE_PID_KP 0.02
#define PITCH_ECD_ANGLE_PID_KI 0.0
#define PITCH_ECD_ANGLE_PID_KD 0.0

#define YAW_ECD_ANGLE_PID_KP 0.02
#define YAW_ECD_ANGLE_PID_KI 0.0
#define YAW_ECD_ANGLE_PID_KD 0.005

#define YAW_B_ECD_ANGLE_PID_KP 0.06
#define YAW_B_ECD_ANGLE_PID_KI 0.0
#define YAW_B_ECD_ANGLE_PID_KD 0.02


#define ROLL_ECD_ANGLE_PID_KP 0.01
#define ROLL_ECD_ANGLE_PID_KI 0.0
#define ROLL_ECD_ANGLE_PID_KD 0.0

#define MOTOR_SPEED_PID_MAX_OUT 8192.0f
#define MOTOR_SPEED_PID_MAX_IOUT 6000.0f

#define MOTOR_ECD_ANGLE_PID_MAX_OUT 10.0f//45.0f
#define MOTOR_ECD_ANGLE_PID_MAX_IOUT 1.0f

#define ENGINEER_PITCH_SPEED 200
#define ENGINEER_YAW_SPEED  300
#define ENGINEER_ROLL_SPEED  400



#ifdef __cplusplus
extern "C"{
#endif
	
#ifdef __cplusplus
	
enum Suction_Motor_Type_e
{
	YAW_B, YAW_F,  //0�?
	PITCH,	ROLL,  //2�?
};

class suction_cup_motor_t
{
	public:
	  PID_t motor_speed_pid;
	  PID_t motor_ecd_angle_pid;
    	  int16_t speed_rpm;
		int64_t ecd_angle;//累计编码�?
    int64_t ecd_angle_set;//累计编码值设定�?
	  int64_t set_ramp_ecd_angle_set;//累计编码值斜坡设定�?
	
	  uint16_t offset_ecd;//相对角度中�?
	  fp32 relative_angle;//相对角度
    fp32 relative_angle_set;//相对角度设定�?
	  fp32 max_relative_angle; //最大相对角�?
    fp32 min_relative_angle; //最小相对角�?
	  
    fp32 motor_speed;//电机速度
    int16_t given_current;//电流�?
};	
	
class suction_cup_t
{
	public:
	
		suction_cup_motor_t Yaw_B_motor;
		suction_cup_motor_t Yaw_F_motor;
	    suction_cup_motor_t Pitch_motor;
	    suction_cup_motor_t Roll_motor;

		uint8_t motor_flag;
		DJIMotorInstance suction_motor[4];
        fp32 suction_motor_angle[4];
        fp32 suction_motor_angle_set[4];

		fp32 add_yaw;
		fp32 add_pitch;
		fp32 add_roll;

        void SuctionInit();
        void SuctionInfoUpdate();
		void Control();
};
	
	
	
#endif
void SuctionCupTask(void);

#ifdef __cplusplus
}	
#endif

#endif





