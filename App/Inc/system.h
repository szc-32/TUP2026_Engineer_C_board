#ifndef __SYSTEM_H
#define __SYSTEM_H

#include "message_center.h"
#include "remote_control.h"
#include "ladrc_feedforward.h"
#include "operation_def.h"
#include "robot_def.h"

#ifdef __cplusplus
extern "C"{
#endif
	
#ifdef __cplusplus

extern int16_t test_current;
class system_t
{	
	private:
		Control_Mode_t last_mode; //上一次的模式
		/*****低通滤波结构体*****/
		lpf_type_def yaw_lpf;  
	    lpf_type_def pitch_lpf;
	
		/*****遥控器控制量*****/
		fp32 vx_set_channel, vy_set_channel;
		fp32 vx_ramp_set,vy_ramp_set;
		fp32 mouse_yaw, mouse_pitch;
		int16_t yaw_channel, pitch_channel;
		int8_t key_yaw;
	
		/*****射击模式变量*****/
		uint8_t debug_shoot_flag,debug_shoot_mode_set; //调试使用变量
		uint8_t key_shoot_mode_set,key_shoot_flag; //键盘控制变量
		uint16_t press_shoot_time; //按键持续按下时间
	
		/*****拨轮状态设置*****/
		uint16_t postive_thumb_wheel_state_time,negative_thumb_wheel_state_time; //拨轮状态值时间
		uint8_t postive_thumb_wheel_state,negative_thumb_wheel_state;	//正负拨轮状态值
		/*****其余变量设置*****/
		uint8_t last_mouse_l_press;
		uint8_t last_key_g_press;
		 

	public:
		system_t();
		const RC_ctrl_t *system_rc_ctrl; //遥控器指针
		Sys_Pub_Msg_t sys_pub; //发送信息实例
	
		/***模式设置***/
		void RobotModeSet(); //拨杆设置
	  void ThumbWheelModeSet(); //状态设置
		void KeyBoardModeSet();
		/***按键和控制量设置***/
		void CalControlQuantity();
		void KeyBoardQuantitySet();
		void RemoteQuantitySet();
};
	
#endif

void SysInit(void);
void SystemTask(void);	
#ifdef __cplusplus
}
#endif

#endif	

