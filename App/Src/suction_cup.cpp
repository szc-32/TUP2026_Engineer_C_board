/**
 * @file suction_cup.cpp
 * @brief Suction-cup joint motor initialization, feedback update and closed-loop control.
 */

#include "suction_cup.h"
#include "cmsis_os.h"

suction_cup_t suction_cup;

Motor_Setting_t suction_config[4] = 
{
    {"Yaw_B_motor",ON_CAN1,1,M3508,POSITIVE_DIRECT,RATIO_1_TO_19,CASCADE_LOOP},
    {"Yaw_F_motor",ON_CAN1,2,M2006,NEGATIVE_DIRECT,RATIO_1_TO_36,CASCADE_LOOP},
    {"Pitch_motor",ON_CAN1,3,M2006,POSITIVE_DIRECT,RATIO_1_TO_36,CASCADE_LOOP},
    {"Roll_motor",ON_CAN1,4,M2006,NEGATIVE_DIRECT,RATIO_1_TO_36,CASCADE_LOOP},
};

void SuctionCupTask()
{
    // This task currently reuses one class instance for init/update/control.
    suction_cup.SuctionInit();
    suction_cup.SuctionInfoUpdate();
    suction_cup.Control();
    
}

void suction_cup_t::SuctionInit()
{
    //电机速度环pid参数
	fp32 yaw_speed_pid[3]={YAW_SPEED_PID_KP,YAW_SPEED_PID_KI,YAW_SPEED_PID_KD};
    fp32 pitch_speed_pid[3]={PITCH_SPEED_PID_KP,PITCH_SPEED_PID_KI,PITCH_SPEED_PID_KD};
    fp32 roll_speed_pid[3]={ROLL_SPEED_PID_KP,ROLL_SPEED_PID_KI,ROLL_SPEED_PID_KD};
    //电机累计编码值环pid参数
    fp32 yaw_ecd_angle_pid[3]={YAW_ECD_ANGLE_PID_KP,YAW_ECD_ANGLE_PID_KI,YAW_ECD_ANGLE_PID_KD};
    fp32 pitch_ecd_angle_pid[3]={PITCH_ECD_ANGLE_PID_KP,PITCH_ECD_ANGLE_PID_KI,PITCH_ECD_ANGLE_PID_KD};
    fp32 roll_ecd_angle_pid[3]={ROLL_ECD_ANGLE_PID_KP,ROLL_ECD_ANGLE_PID_KI,ROLL_ECD_ANGLE_PID_KD};
  
    suction_motor[YAW_B].DJIMotorInit(&suction_config[YAW_B]);
    suction_motor[YAW_F].DJIMotorInit(&suction_config[YAW_F]);
    suction_motor[PITCH].DJIMotorInit(&suction_config[PITCH]);
    suction_motor[ROLL].DJIMotorInit(&suction_config[ROLL]);

    suction_motor[YAW_B].controller.speed_PID.Init(PID_POSITION,yaw_speed_pid,10000,16000);
    suction_motor[YAW_F].controller.speed_PID.Init(PID_POSITION,yaw_speed_pid,10000,9000);
    suction_motor[PITCH].controller.speed_PID.Init(PID_POSITION,pitch_speed_pid,MOTOR_SPEED_PID_MAX_OUT,MOTOR_SPEED_PID_MAX_IOUT);
    suction_motor[ROLL].controller.speed_PID.Init(PID_POSITION,roll_speed_pid,MOTOR_SPEED_PID_MAX_OUT,MOTOR_SPEED_PID_MAX_IOUT);

    suction_motor[YAW_B].controller.angle_PID.Init(PID_POSITION,yaw_ecd_angle_pid,MOTOR_ECD_ANGLE_PID_MAX_OUT,MOTOR_ECD_ANGLE_PID_MAX_IOUT);
    suction_motor[YAW_F].controller.angle_PID.Init(PID_POSITION,yaw_ecd_angle_pid,MOTOR_ECD_ANGLE_PID_MAX_OUT,MOTOR_ECD_ANGLE_PID_MAX_IOUT);
    suction_motor[PITCH].controller.angle_PID.Init(PID_POSITION,pitch_ecd_angle_pid,MOTOR_ECD_ANGLE_PID_MAX_OUT,MOTOR_ECD_ANGLE_PID_MAX_IOUT);
    suction_motor[ROLL].controller.angle_PID.Init(PID_POSITION,roll_ecd_angle_pid,MOTOR_ECD_ANGLE_PID_MAX_OUT,MOTOR_ECD_ANGLE_PID_MAX_IOUT);

    for(int i = 0;i<=3;i++)
    {
        suction_motor_angle[i] = 0;

    }

}

void suction_cup_t::SuctionInfoUpdate()
{
    for (int i = 0;i<=4;i++)
    {
        suction_motor_angle[i] = suction_motor[i].CalcTotalecd();
    }
    suction_motor_angle_set[YAW_B] = MovingPointer()->yaw_b_motor.motor_angle_set;
    suction_motor_angle_set[YAW_F] = MovingPointer()->yaw_f_motor.motor_angle_set;
    suction_motor_angle_set[PITCH] = MovingPointer()->pitch_motor.motor_angle_set;
    suction_motor_angle_set[ROLL] = MovingPointer()->roll_motor.motor_angle_set;

}

void suction_cup_t::Control()
{
	if(SysPointer()->mode==ZERO_FORCE)
	{
        for (int i = 0;i<=4;i++)
        {
            suction_motor[i].controller.send_current = 0;
        }
		return;
	} 

    for (int i = 0;i<=4;i++)
    {
        suction_motor[i].DJIMotorControl(&suction_motor_angle[i],&suction_motor_angle_set[i],NULL,OFF_SET);
    }

}

