/**
 * @file lifting.h
 * @brief Public declarations for App layer.
 */

#ifndef LIFTING_H
#define LIFTING_H

#include "message_center.h"
#include "user_lib.h"
#include "robot_def.h"
#include "PID.h"
#include "motor.h"

#define UPMOTOR_SPEED_PID_KP 4000.0f       
#define UPMOTOR_SPEED_PID_KI 0.0f
#define UPMOTOR_SPEED_PID_KD 1500.0f   //1500
#define UPMOTOR_SPEED_PID_MAX_OUT 8192.0f   //9000
#define UPMOTOR_SPEED_PID_MAX_IOUT 5000.0f

#define UPMOTOR_ANGLE_PID_KP 0.002f
#define UPMOTOR_ANGLE_PID_KI 0.0f
#define UPMOTOR_ANGLE_PID_KD 0.002f//0.01f
#define UPMOTOR_ANGLE_PID_MAX_OUT  50.0f
#define UPMOTOR_ANGLE_PID_MAX_IOUT 5.0f

#define FRONTMOTOR_SPEED_PID_KP 3000.0f       
#define FRONTMOTOR_SPEED_PID_KI 0
#define FRONTMOTOR_SPEED_PID_KD 1000    //2500
#define FRONTMOTOR_SPEED_PID_MAX_OUT 9000.0f   //8000
#define FRONTMOTOR_SPEED_PID_MAX_IOUT 5000.0f

#define FRONTMOTOR_ANGLE_PID_KP 0.005f         //-0.0009f
#define FRONTMOTOR_ANGLE_PID_KI 0.0f
#define FRONTMOTOR_ANGLE_PID_KD 0.01
#define FRONTMOTOR_ANGLE_PID_MAX_OUT  30.0f     //10
#define FRONTMOTOR_ANGLE_PID_MAX_IOUT 5.0f

#ifdef __cplusplus
extern "C"{
#endif

#ifdef __cplusplus
enum Lifting_Motor_Type_e
{
	FRONT_L, FRONT_R,  //0�?
	UP_L,	UP_R,  //2�?
    SIDE,
};

class lifting_motor_t
{
	public:
	  fp32 ecd_angle; //计算角度差�?
    int16_t speed_rpm;
	  int64_t motor_angle;
    int64_t motor_angle_set;   
	
    fp32 motor_speed;
    int16_t given_current;
	
};

class lifting_t
{
    public:
			
    //抬升电机*2 前伸电机*2 
      
      lifting_motor_t up_motor_l;//左抬升电�?
      lifting_motor_t up_motor_r;//右抬升电�?
      
      lifting_motor_t front_motor_l;//左前伸电�?
      lifting_motor_t front_motor_r;//右前伸电�?
      
      lifting_motor_t side_motor;//

      DJIMotorInstance lifting_motor[5];
      fp32 lifting_motor_angle[5];
      fp32 lifting_motor_angle_set[5];

      // lifting_control_t();
      void LiftingInit();
      void LiftingInfoUpdate();
      void Control();

};

#endif
void LiftingTask(void);

#ifdef __cplusplus
}
#endif


#endif

