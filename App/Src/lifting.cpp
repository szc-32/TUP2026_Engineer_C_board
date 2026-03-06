#include "lifting.h"
#include "cmsis_os.h"


lifting_t lifting;

Motor_Setting_t lifting_config[5] = 
{
    {"lifting_front_l",ON_CAN2,1,M3508,POSITIVE_DIRECT,RATIO_1_TO_19,CASCADE_LOOP},
    {"lifting_front_r",ON_CAN2,2,M3508,NEGATIVE_DIRECT,RATIO_1_TO_19,CASCADE_LOOP},
    {"lifting_up_l",ON_CAN2,3,M3508,POSITIVE_DIRECT,RATIO_1_TO_19,CASCADE_LOOP},
    {"lifting_up_r",ON_CAN2,4,M3508,NEGATIVE_DIRECT,RATIO_1_TO_19,CASCADE_LOOP},
    {"lifting_side",ON_CAN1,5,M2006,POSITIVE_DIRECT,RATIO_1_TO_36,CASCADE_LOOP},
};

void LiftingTask()
{
    lifting.LiftingInit();
    lifting.LiftingInfoUpdate();
    lifting.Control();
}

void lifting_t::LiftingInit()
{
	fp32 up_motor_speed_pid[3]={UPMOTOR_SPEED_PID_KP,UPMOTOR_SPEED_PID_KI,UPMOTOR_SPEED_PID_KD};
    fp32 up_motor_angle_pid[3]={UPMOTOR_ANGLE_PID_KP,UPMOTOR_ANGLE_PID_KI,UPMOTOR_ANGLE_PID_KD};
  
    fp32 front_motor_speed_pid[3]={FRONTMOTOR_SPEED_PID_KP,FRONTMOTOR_SPEED_PID_KI,FRONTMOTOR_SPEED_PID_KD};
	fp32 front_motor_angle_pid[3]={FRONTMOTOR_ANGLE_PID_KP,FRONTMOTOR_ANGLE_PID_KI,FRONTMOTOR_ANGLE_PID_KD};
	
	fp32 side_motor_speed_pid[3]={1500,0,500};
	fp32 side_motor_angle_pid[3]={0.01,0,0.005};

    /*****电机初始化*****/
    lifting_motor[FRONT_L].DJIMotorInit(&lifting_config[FRONT_L]);
    lifting_motor[FRONT_R].DJIMotorInit(&lifting_config[FRONT_R]);
    lifting_motor[UP_L].DJIMotorInit(&lifting_config[UP_L]);
    lifting_motor[UP_R].DJIMotorInit(&lifting_config[UP_R]);
    lifting_motor[SIDE].DJIMotorInit(&lifting_config[SIDE]);

    lifting_motor[FRONT_L].controller.speed_PID.Init(PID_POSITION,front_motor_speed_pid,FRONTMOTOR_SPEED_PID_MAX_OUT,FRONTMOTOR_SPEED_PID_MAX_IOUT);
    lifting_motor[FRONT_R].controller.speed_PID.Init(PID_POSITION,front_motor_speed_pid,FRONTMOTOR_SPEED_PID_MAX_OUT,FRONTMOTOR_SPEED_PID_MAX_IOUT);
    lifting_motor[UP_L].controller.speed_PID.Init(PID_POSITION,up_motor_speed_pid,UPMOTOR_SPEED_PID_MAX_OUT,UPMOTOR_SPEED_PID_MAX_IOUT);
    lifting_motor[UP_R].controller.speed_PID.Init(PID_POSITION,up_motor_speed_pid,UPMOTOR_SPEED_PID_MAX_OUT,UPMOTOR_SPEED_PID_MAX_IOUT);
    lifting_motor[SIDE].controller.speed_PID.Init(PID_POSITION,side_motor_speed_pid,8000,5000);

    lifting_motor[FRONT_L].controller.angle_PID.Init(PID_POSITION,front_motor_angle_pid,FRONTMOTOR_ANGLE_PID_MAX_OUT,FRONTMOTOR_ANGLE_PID_MAX_IOUT);
    lifting_motor[FRONT_R].controller.angle_PID.Init(PID_POSITION,front_motor_angle_pid,FRONTMOTOR_ANGLE_PID_MAX_OUT,FRONTMOTOR_ANGLE_PID_MAX_IOUT);
    lifting_motor[UP_L].controller.angle_PID.Init(PID_POSITION,up_motor_angle_pid,UPMOTOR_ANGLE_PID_MAX_OUT,UPMOTOR_ANGLE_PID_MAX_IOUT);
    lifting_motor[UP_R].controller.angle_PID.Init(PID_POSITION,up_motor_angle_pid,UPMOTOR_ANGLE_PID_MAX_OUT,UPMOTOR_ANGLE_PID_MAX_IOUT);
    lifting_motor[SIDE].controller.angle_PID.Init(PID_POSITION,side_motor_angle_pid,50,5);
    for (int i = 0;i<=4;i++)
    {
        lifting_motor_angle[i] = 0;
    }

}

void lifting_t::LiftingInfoUpdate()
{
    for (int i = 0;i<=4;i++)
    {
        lifting_motor_angle[i] = lifting_motor[i].CalcTotalecd();
    }
    lifting_motor_angle_set[FRONT_L] = MovingPointer()->front_l_motor.motor_angle_set;
    lifting_motor_angle_set[FRONT_R] = MovingPointer()->front_r_motor.motor_angle_set;
    lifting_motor_angle_set[UP_L] = MovingPointer()->up_l_motor.motor_angle_set;
    lifting_motor_angle_set[UP_R] = MovingPointer()->up_r_motor.motor_angle_set;
    lifting_motor_angle_set[SIDE] = MovingPointer()->side_motor.motor_angle_set;

}

void lifting_t::Control()
{
	if(SysPointer()->mode==ZERO_FORCE)
	{
        for (int i = 0;i<=4;i++)
        {
            lifting_motor[i].controller.send_current = 0;
        }
		return;
	} 

    for (int i = 0;i<=4;i++)
    {
        lifting_motor[i].DJIMotorControl(&lifting_motor_angle[i],&lifting_motor_angle_set[i],NULL,OFF_SET);
    }

}

