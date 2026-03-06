/**
 * @file message_center.h
 * @brief Public declarations for App layer.
 */

#ifndef __MESSAGE_CENTER_H
#define __MESSAGE_CENTER_H

#include "led.h"
#include "struct_typedef.h"
#include "moving_behavior.h"
// #include "remote_control.h"

//最大话题数
//#define MAX_SUBJECT 4

#ifdef __cplusplus
extern "C"{
#endif
	
#ifdef __cplusplus	
enum Msg_Type_e 
{
	SYSPUB,
	GIMBALPUB,
	INSPUB,
	CHASSISPUB,
	REVOLVERPUB,
	UIPUB,
	MONITORPUB,
	LIFTINGPUB,
	SUCTIONPUB,
	//add_more
	
	TOTALNUM,
};

enum MessageErrorCode
{
	NO_MSG_ERROR,
	INVALID_MSG_TYPE,
	NULL_POINTER,
};

//模式信息
enum Control_Mode_t
{
	ZERO_FORCE_MOVE = 0,	
	DT7_MISSING, 
	LOCK_WHEEL,
	INIT_MODE,	
	CHASSIS_NO_MOVE,
	NORMAL, 
	RELATIVE_ANGLE,
	SPIN, 
	AUTO, 
	SPIN_AUTO,
	NO_FOLLOW_YAW ,
};

enum Firc_Mode_t
{
	FORBID_FIRE, CLOSE , OPEN,
};

enum Stir_Mode_t
{
	NO_MOVE, ALLOW_MOVE,
	REVERSE_MOVE, //反转
	SHOOT_1_BULLET,  //单发
	SHOOT_3_BULLET, //三发
	SHOOT_FIXED_FIRE_RATE, //连发
};
enum Engineer_Mode_t
{
	ZERO_FORCE,     //无力模式
	INIT,           //初始化模式
	ANCILLARY,      //副机构控制
	LIFTING,        //抬升机构模式
	SUCTION_CUP,    //吸盘机构模式
	STOP_POSITION,  //静止原位模式
	CALIBRATE,      //限位校准
	KEYBOARD,		//键盘控制
	USER,			//自定义控制器
	TEST,           //动作组测试
  
};

//键盘标志位
typedef struct
{
	uint8_t up_thumb_flag,down_thumb_flag;
	//云台设置
	uint8_t auto_aim_flag,turn_round_flag;
	//运动设置
	uint8_t super_cap_flag,spin_flag;
	//开火设置
	uint8_t heat_limit_flag,auto_fir_flag,fir_flag;
	//其他设置
	//uint8_t	reset_flag;
	 //变速小陀螺
    bool_t spin_speed_change_flag;
}Key_Flag_t;

typedef struct
{
	Control_Mode_t mode;
	Firc_Mode_t fir_mode;
	Stir_Mode_t stir_mode;
	Engineer_Mode_t engineer_mode;
	Key_Flag_t key_flag;
	//设定值
	fp32 add_yaw,add_pit;
	fp32 vx_set,vy_set;
	uint8_t change_mode_flag;
	bool_t spin_reverse_flag;    //反向小陀螺
	bool_t arm_home_done;        //六轴机械臂归位完成标志
}Sys_Pub_Msg_t;

typedef struct
{
	fp32 yaw_relative_angle,pit_relative_angle,yaw_down_relative_angle;
	fp32 yaw_absolute_start_rad,yaw_absolute_set_rad,yaw_absolute_rad;//添加
}Gimbal_Pub_Msg_t;

typedef struct
{
	fp32 send_vx_set,send_vy_set,send_wz_set;
}Chassis_Pub_Msg_t;

typedef struct
{
	uint8_t rest_heat;  //剩余热量
	fp32 shoot_rate;  //射频
	uint8_t if_stuck;
	uint8_t shoot_finish,reserve_time;
	uint8_t shoot_mode_set;
}Revolver_Pub_Msg_t;

typedef struct
{
	fp32 x_coordinate,y_coordinate;
	fp32 pre_x_coordinate,pre_y_coordinate;
	fp32 follow_radius;
}UI_Pub_Msg_t;

typedef struct
{
	Led_State_e state;
	
}Monitor_Pub_Msg_t;

typedef struct
{
	fp32 lifting_motor_angle[5];

}Lifting_Pub_Msg_t;

typedef struct
{
	fp32 suction_motor_angle[4];

}Suction_Cup_Pub_Msg_t;

// typedef struct 
// {
// 	moving_motor_t up_r_motor,up_l_motor;//主抬升
// 	moving_motor_t front_r_motor,front_l_motor;//主前伸
// 	moving_motor_t side_motor;//主横移
// 	moving_motor_t yaw_f_motor,yaw_b_motor;//yaw轴
// 	moving_motor_t pitch_motor,roll_motor;//pitch,roll轴
// 	moving_motor_t ancillary_up_motor_l,ancillary_up_motor_r,ancillary_front_motor_l,ancillary_front_motor_r;//左右副抬升，前伸

// }Moving_Behavior_Pub_Msg_t;


class Message_Center_t
{
	public:
		//添加需初始化的结构体指针
		Sys_Pub_Msg_t *sys_msg;
		Gimbal_Pub_Msg_t *gimbal_msg;
		Chassis_Pub_Msg_t *chassis_msg;
		Revolver_Pub_Msg_t *revolver_msg;
		UI_Pub_Msg_t *ui_msg;
		Monitor_Pub_Msg_t *monitor_msg;
	    Lifting_Pub_Msg_t *lifting_msg;
		Suction_Cup_Pub_Msg_t *suction_msg;
		// Moving_Behavior_Pub_Msg_t *moving_behavoir_msg;

		//指针初始化
		MessageErrorCode PointerInit(void *msg,Msg_Type_e type);
		bool CheckPointerEmpty();
		void SetLedState(Led_State_e input_state,uint8_t priority);
};

bool CheckMessageCenter();
Message_Center_t *CenterPointer();
Sys_Pub_Msg_t *SysPointer();
Gimbal_Pub_Msg_t *GimbalPointer();
Chassis_Pub_Msg_t *ChassisPointer();
Revolver_Pub_Msg_t *RevolverPointer();
UI_Pub_Msg_t *UiPointer();
Monitor_Pub_Msg_t *MonitorPointer(); 
Lifting_Pub_Msg_t *LiftingPointer();
Suction_Cup_Pub_Msg_t *SuctionPointer();
// Moving_Behavior_Pub_Msg_t *MovingPointer();

#endif
	
#ifdef __cplusplus
}
#endif

#endif
 

