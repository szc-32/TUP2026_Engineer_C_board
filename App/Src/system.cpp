/**
 ******************************************************************************
 * @file    sys_task.cpp
 * @author  Xushuang
 * @version V1.0.0 Xushuang 基本完成
 *					V1.1.0 Xushuang 优化封装
 *					V2.0.0 Xushuang 优化模式设置
 * @date    2024/1/11
 * @brief		此处为模式设置和各按键处理
 ******************************************************************************
 * @attention
 *实际比赛有必要为飞坡单独写一个模式-----对准后禁止YAW轴移动
 *调试时
 *步兵、英雄统一规定遥控器对应模式：
 *		左↓	右↓：无力	摩擦轮不动
 *		左↓	右-：正常控制（底盘跟随云台）/单控底盘
 *		左↓ 右↑：小陀螺 摩擦轮不动
 *		左-	右-：开启摩擦轮
 *		左↑ 右-↑：拨盘转动打弹
 *		右-：左↙ 右↗：开启自瞄（单按模式）
 *		拨轮：向正方向开启自瞄（松开即退出自瞄）
 *					向负方向开启超级电容
 ******************************************************************************
 */
#include "arm_math.h"
#include "main.h"
#include "system.h"
#include "cmsis_os.h"

//系统实例
system_t sys;

/**
	* @brief          系统类构造函数
  * @param[in]      NULL
  * @retval         NULL
  */
system_t::system_t()
{
	/*****初始模式设置*****/
	sys_pub.mode = INIT_MODE;
	sys_pub.fir_mode = FORBID_FIRE;
	sys_pub.stir_mode = NO_MOVE;
	sys_pub.engineer_mode = CALIBRATE;
	sys_pub.arm_home_done = FALSE;
	last_mode = sys_pub.mode;
	/*****初始发弹模式设置*****/
	key_shoot_mode_set = 1;
	debug_shoot_mode_set = 1;
	/*****初始控制量设置*****/
	vx_ramp_set = vy_ramp_set = vx_set_channel = vy_set_channel =0.0f;
	mouse_yaw = mouse_pitch = key_yaw = 0.0f;
	yaw_channel = pitch_channel = 0.0f;
}

/**
	* @brief          系统初始化
  * @param[in]      NULL
  * @retval         NULL
  */
void SysInit()
{
	/******遥控器指针获取******/
	sys.system_rc_ctrl = GetRemoteControlPoint();
	/******信息中心实例建立******/
	CenterPointer()->PointerInit(&sys.sys_pub,SYSPUB);
}

/**
	* @brief          系统主任务
  * @param[in]      NULL
  * @retval         NULL
  */
void SystemTask()
{
	/*****模式设置*****/
	sys.KeyBoardModeSet(); //键盘设置
	sys.ThumbWheelModeSet(); 	 //拨轮设置
	sys.RobotModeSet();  //机器人模式设置
	/*****操作量设置*****/
	sys.CalControlQuantity();
}

/**
	* @brief          机器人模式设置
  * @param[in]      NULL
  * @retval         NULL
  */
void system_t::RobotModeSet()
{
	static bool_t boot_auto_calibrate_pending = TRUE;

	if (boot_auto_calibrate_pending == TRUE && sys_pub.engineer_mode != CALIBRATE)
	{
		boot_auto_calibrate_pending = FALSE;
	}

	if (boot_auto_calibrate_pending == TRUE)
	{
		sys_pub.mode = INIT_MODE;
		sys_pub.engineer_mode = CALIBRATE;
		if (last_mode != sys_pub.mode)
		{
			sys_pub.change_mode_flag = 1;
		}
		last_mode = sys_pub.mode;
		return;
	}

	if (sys_pub.engineer_mode == CALIBRATE)
	{
		// Keep CALIBRATE workflow in control; do not overwrite mode by RC switches.
		if (last_mode != sys_pub.mode)
		{
			sys_pub.change_mode_flag = 1;
		}
		last_mode = sys_pub.mode;
		return;
	}

	/*****遥控器失联模式判断*****/
	if(!MonitorRc())
	{
		//失联模式进行保护
		sys_pub.mode = DT7_MISSING;
		sys_pub.fir_mode = CLOSE;
		sys_pub.stir_mode = NO_MOVE;
		sys_pub.engineer_mode = ZERO_FORCE;
		return;
	}
	
	/*****正常模式控制*****/
	if(sys_pub.mode == INIT_MODE)
	{
		//防止拨杆突然到最底下的情况
		if(switch_is_down(system_rc_ctrl->rc.s[RIGTH_CHANNEL]))
			sys_pub.mode = ZERO_FORCE_MOVE;
		return;
		
	}
	else
	{
		static uint16_t calibrate_hold_cnt = 0;
		const int16_t rc_cali_hole = 600;
		const uint16_t calibrate_hold_need = 2000;
		const bool_t calibrate_gesture =
			system_rc_ctrl->rc.ch[0] < -rc_cali_hole &&
			system_rc_ctrl->rc.ch[1] < -rc_cali_hole &&
			system_rc_ctrl->rc.ch[2] > rc_cali_hole &&
			system_rc_ctrl->rc.ch[3] < -rc_cali_hole &&
			switch_is_down(system_rc_ctrl->rc.s[LEFT_CHANNEL]) &&
			switch_is_down(system_rc_ctrl->rc.s[RIGTH_CHANNEL]);

		if (calibrate_gesture)
		{
			if (calibrate_hold_cnt < calibrate_hold_need)
			{
				calibrate_hold_cnt++;
			}
		}
		else
		{
			calibrate_hold_cnt = 0;
		}

		// Legacy-style gesture entry: hold '\../' for 2s to enter new CALIBRATE workflow.
		if (calibrate_hold_cnt >= calibrate_hold_need)
		{
			sys_pub.mode = NORMAL;
			sys_pub.engineer_mode = CALIBRATE;
			if (last_mode != sys_pub.mode)
			{
				sys_pub.change_mode_flag = 1;
			}
			last_mode = sys_pub.mode;
			return;
		}

		const bool_t custom_controller_mode =
			switch_is_down(system_rc_ctrl->rc.s[LEFT_CHANNEL]) &&
			switch_is_up(system_rc_ctrl->rc.s[RIGTH_CHANNEL]);

		/**********运动模式设置**********/
		if(switch_is_down(system_rc_ctrl->rc.s[LEFT_CHANNEL]) && switch_is_down(system_rc_ctrl->rc.s[RIGTH_CHANNEL]))
		{  
			//左下右下 无力模式
			sys_pub.mode = ZERO_FORCE_MOVE;
			sys_pub.engineer_mode = ZERO_FORCE;
		}
		else if(switch_is_down(system_rc_ctrl->rc.s[LEFT_CHANNEL]) && switch_is_mid(system_rc_ctrl->rc.s[RIGTH_CHANNEL]))
		{ 
			//左下右中 正常控制模式-键盘同时可以控制
			sys_pub.mode = RELATIVE_ANGLE;
			sys_pub.engineer_mode = STOP_POSITION
			;
		}
		else if(custom_controller_mode)
		{  
			//左下右上 接入新的自定义控制器逻辑（USER）
			sys_pub.mode = RELATIVE_ANGLE;
			sys_pub.engineer_mode = USER;
		}
		
		/**********初始化模式判断**********/
		if(last_mode == ZERO_FORCE_MOVE && sys_pub.mode != ZERO_FORCE_MOVE)
			sys_pub.mode = INIT_MODE;

		/**********模式切换判断**********/
		if(last_mode != sys_pub.mode)
			sys_pub.change_mode_flag = 1;
		//保存上次模式，用以判断模式切换
		last_mode = sys_pub.mode; 
		
		/**********工程模式设置**********/
		if (switch_is_mid(system_rc_ctrl->rc.s[LEFT_CHANNEL]) && switch_is_down(system_rc_ctrl->rc.s[RIGTH_CHANNEL]))
		{
			//左中右下 小陀螺模式
			sys_pub.mode = SPIN;
			sys_pub.engineer_mode = KEYBOARD;
		}
		else if (switch_is_mid(system_rc_ctrl->rc.s[LEFT_CHANNEL]) && switch_is_mid(system_rc_ctrl->rc.s[RIGTH_CHANNEL]))
		{
			//左中右中 六轴机械臂控制模式
			sys_pub.mode = NORMAL;
			sys_pub.engineer_mode = KEYBOARD;
		}
		// else if (switch_is_mid(system_rc_ctrl->rc.s[LEFT_CHANNEL]) && switch_is_up(system_rc_ctrl->rc.s[RIGTH_CHANNEL]))
		// {
		// 	//左中右上 机械臂控制模式
		// 	sys_pub.mode = NORMAL;
		// 	sys_pub.engineer_mode = SUCTION_CUP;

		// }

		if (switch_is_up(system_rc_ctrl->rc.s[LEFT_CHANNEL]) && switch_is_down(system_rc_ctrl->rc.s[RIGTH_CHANNEL]))
		{
			//左上右下 六轴机械臂归位模式（回初始化位置）
			sys_pub.mode = NORMAL;
			sys_pub.engineer_mode = INIT;
		}
		// else if (switch_is_up(system_rc_ctrl->rc.s[LEFT_CHANNEL]) && switch_is_mid(system_rc_ctrl->rc.s[RIGTH_CHANNEL]))
		// {
		// 	//左上右中 动作组测试模式
		// 	sys_pub.mode = NORMAL;
		// 	sys_pub.engineer_mode = TEST;
		// }
		else if (switch_is_up(system_rc_ctrl->rc.s[LEFT_CHANNEL]) && switch_is_up(system_rc_ctrl->rc.s[RIGTH_CHANNEL]))
		{
			//左上右上 键盘控制模式
			sys_pub.mode = NORMAL;
			sys_pub.engineer_mode = KEYBOARD;
		}

		/**********小陀螺模式设置**********/
		if(sys_pub.key_flag.spin_flag && sys_pub.engineer_mode != USER)
		{
			if(sys_pub.mode == NORMAL || sys_pub.mode == RELATIVE_ANGLE)
			{
				sys_pub.mode = SPIN;
			}
			else if(sys_pub.mode == AUTO)
			{
				sys_pub.mode = SPIN_AUTO;
			}
		}

		/**********模式切换判断（最终模式）**********/
		if(last_mode != sys_pub.mode)
			sys_pub.change_mode_flag = 1;
		last_mode = sys_pub.mode;
	}

	  
}

/**
	* @brief          拨轮模式设置
  * @param[in]      NULL
  * @retval         NULL
  */
void system_t::ThumbWheelModeSet()
{
	/*****下半边拨轮的状态使用*****/
	//用于拨盘模式设置
	if(sys_pub.stir_mode == ALLOW_MOVE) //拨轮设置为此种模式下才可进行模式设置
	{
		//遥控器射击的设置（使用正半边——往下拨动）
		ThumbWheelSet(POSITIVE_DIRECTION,&postive_thumb_wheel_state);
		if(postive_thumb_wheel_state)
			postive_thumb_wheel_state_time++;
	}
	
	//200<拨动时间<1500：松掉回中后则为单发模式;波动时间>1500：则为连发模式
	if(postive_thumb_wheel_state_time>200 && debug_shoot_mode_set)
	{
		if(postive_thumb_wheel_state_time >= 1500)
		{
			debug_shoot_flag = 3;
			if(IF_THUMB_WHEEL_ZERO_STATE) //拨杆值回中
			{
				debug_shoot_flag = 0;
				postive_thumb_wheel_state_time = 0;
			}
		}else
		{
			if(IF_THUMB_WHEEL_ZERO_STATE)  //拨杆值回中 //IF_THUMB_WHEEL_ZERO_STATE需要改试试
			{
				debug_shoot_flag = 1;
				postive_thumb_wheel_state_time = 0;
			}
		}
	}else
	{
		debug_shoot_flag = 0;
	}
	
	/*****上半边拨轮的状态使用*****/
	//用于自瞄设置
	ThumbWheelSet(NEGATIVE_DIRECTION,&negative_thumb_wheel_state);	//遥控器自瞄的设置（使用负半边）
	if(negative_thumb_wheel_state)
		negative_thumb_wheel_state_time++;	
		
	if(negative_thumb_wheel_state_time>800)
	{
		sys_pub.key_flag.down_thumb_flag = !sys_pub.key_flag.down_thumb_flag;
		negative_thumb_wheel_state_time = 0;
	}
	
	if(system_rc_ctrl->rc.ch[4]> 300)
		sys_pub.key_flag.super_cap_flag = 1;
	else
		sys_pub.key_flag.super_cap_flag = 0;
}

/**
	* @brief          操作设置
  * @param[in]      NULL
  * @retval         NULL
  */
void system_t::CalControlQuantity()
{
	//遥控器控制量设置
	RemoteQuantitySet();
	//按键控制量设置
	KeyBoardQuantitySet();
	
	//发布的信息基础解算
	sys_pub.add_yaw = (yaw_channel * YAW_RC_SEN - 
												 mouse_yaw +
												 key_yaw*0.00003);
	
	sys_pub.add_pit = (pitch_channel * PITCH_RC_SEN - 
												 mouse_pitch 
												);
	
	vx_ramp_set = RAMP_float(vx_set_channel,vx_ramp_set,0.004);
	vy_ramp_set = RAMP_float(vy_set_channel,vy_ramp_set,0.004);
	
	//停止信号，不需要缓慢加速，直接减速到零
	if (vx_set_channel < CHASSIS_RC_DEADLINE * CHASSIS_VX_RC_SEN && vx_set_channel > -CHASSIS_RC_DEADLINE * CHASSIS_VX_RC_SEN)
		vx_ramp_set = 0.0f;

	if (vy_set_channel < CHASSIS_RC_DEADLINE * CHASSIS_VY_RC_SEN && vy_set_channel > -CHASSIS_RC_DEADLINE * CHASSIS_VY_RC_SEN)
		vy_ramp_set = 0.0f;
	
	//斜坡赋值
	sys_pub.vx_set = vx_ramp_set;
	sys_pub.vy_set = vy_ramp_set;
}

/**
	* @brief          遥控器控制量设置
  * @param[in]      NULL
  * @retval         NULL
  */
void system_t::RemoteQuantitySet()
{
	int16_t vx_channel, vy_channel;
	
	//将遥控器的数据处理死区 int16_t yaw_channel,pitch_channel
	rc_deadband_limit(system_rc_ctrl->rc.ch[YAW_CHANNEL], yaw_channel, RC_DEADBAND);
    rc_deadband_limit(system_rc_ctrl->rc.ch[PITCH_CHANNEL], pitch_channel, RC_DEADBAND);
	
	//遥控器死区限制，因为遥控器可能存在差异 摇杆在中间，其值不为0
    rc_deadband_limit(system_rc_ctrl->rc.ch[CHASSIS_X_CHANNEL], vx_channel, CHASSIS_RC_DEADLINE);
    rc_deadband_limit(system_rc_ctrl->rc.ch[CHASSIS_Y_CHANNEL], vy_channel, CHASSIS_RC_DEADLINE);
	
	//通道值设置，缓慢上升
	vx_set_channel = vx_channel * -CHASSIS_VX_RC_SEN;
	vy_set_channel = vy_channel * CHASSIS_VY_RC_SEN;
}

/**
	* @brief          键盘基础运动设置-不允许修改
  * @param[in]      NULL
  * @retval         NULL
  */
void system_t::KeyBoardQuantitySet()
{
	//鼠标对应YAW,PITCH的灵敏度
	mouse_yaw = LPF(&yaw_lpf ,0.002,system_rc_ctrl->mouse.x * YAW_MOUSE_SEN, 14);
	mouse_pitch = LPF(&pitch_lpf ,0.002,system_rc_ctrl->mouse.y * PITCH_MOUSE_SEN, 30);
	
	//按键Q、E进行YAW轴灵敏度
	if(IF_KEY_PRESSED_Q)
		key_yaw = 1;
	else if(IF_KEY_PRESSED_E)
		key_yaw = -1;
	
	/***基础运动设置前后左右运动W、S、A、D***/
	if(IF_KEY_PRESSED_W)
		vx_set_channel = -NORMAL_MAX_CHASSIS_SPEED_X;
	else if(IF_KEY_PRESSED_S)
		vx_set_channel = NORMAL_MAX_CHASSIS_SPEED_X;
	
	if(IF_KEY_PRESSED_D)
		vy_set_channel = NORMAL_MAX_CHASSIS_SPEED_Y;
	else if(IF_KEY_PRESSED_A)
		vy_set_channel = -NORMAL_MAX_CHASSIS_SPEED_Y;
	
}

/**
	* @brief          其余按键设置-不同车可依据操作手需要更改
  * @param[in]      NULL
  * @retval         NULL
  */
void system_t::KeyBoardModeSet()
{
	/*****所有按键操作设置都在此处定义（额外多加也需在此处）*****/
	/***小陀螺模式设置（单击SHIFT键）***/
	static uint16_t last_keyboard = 0;
	if((system_rc_ctrl->key.v & KEY_PRESSED_OFFSET_SHIFT) && !(last_keyboard & KEY_PRESSED_OFFSET_SHIFT))
		sys_pub.key_flag.spin_flag = !sys_pub.key_flag.spin_flag;
	
	/***一键掉头模式设置（单击G键）***/
	if((system_rc_ctrl->key.v & KEY_PRESSED_OFFSET_G) && !(last_keyboard & KEY_PRESSED_OFFSET_G))
		sys_pub.key_flag.turn_round_flag = !sys_pub.key_flag.turn_round_flag;
	/***超级电容是否开启设置（长按C键）***/
	if(IF_KEY_PRESSED_C) 
		sys_pub.key_flag.super_cap_flag = 1;
	else
		sys_pub.key_flag.super_cap_flag = 0;
	/***自瞄模式设置（鼠标右键）***/
	if(IF_MOUSE_PRESSED_RIGHT)
		sys_pub.key_flag.auto_aim_flag = 1;
	else
		sys_pub.key_flag.auto_aim_flag = 0;
	
	/***发射模式设置（鼠标左键射击、G键切换射击模式）***/
	//G键进行模式切换
	if(IF_KEY_PRESSED_G && !last_key_g_press) 
	{
		//0、1、2分别为单发、三发、连发模式
		if(key_shoot_mode_set++ >= 3)
			key_shoot_mode_set = 1;
	}
	
	if(key_shoot_mode_set == 1 && (IF_MOUSE_PRESSED_LEFT && !last_mouse_l_press)) //单击鼠标左键：单发射击
	{
		key_shoot_flag = 1;  
	}else if(key_shoot_mode_set == 2 && (IF_MOUSE_PRESSED_LEFT && !last_mouse_l_press)) //单击鼠标左键：三发射击
	{
		key_shoot_flag = 2;
	}else if(key_shoot_mode_set == 3 && IF_MOUSE_PRESSED_LEFT) //长按鼠标左键：连续射击
	{
		//原按键
		if(press_shoot_time++ >300)  //按键时间大于阈值进行连发防止误触
			key_shoot_flag = 3;
	}else
	{
		key_shoot_flag = 0;
	}
	//上次按键值，用于模式判断
	last_mouse_l_press = IF_MOUSE_PRESSED_LEFT;
	last_key_g_press = IF_KEY_PRESSED_G;
	last_keyboard = system_rc_ctrl->key.v;
	/***Reset按键设置（单击B键）***/
	if(IF_KEY_PRESSED_B)
		NVIC_SystemReset();
	
}

