/**
 ******************************************************************************
 * @file    gimbal.cpp
 * @author  Xushuang
 * @version V1.0.0 基本完成
 * @date    2023/9/20
 * @brief		此处为云台各模式控制
 ******************************************************************************
 * @attention
 *
 ******************************************************************************
 */
#include "arm_math.h"
#include "gimbal.h"
#include "stm32f4xx_hal.h"
#include "main.h"
#include "bsp_dwt.h"
#include "bsp_usart.h"
#include "remote_control.h"
#include "photogate.h"

#include <stdio.h>
#include <string.h>
//车的后方为X轴正方向，右方为Y轴正方向，上方为Z轴正方向

#define GIMBAL_PHOTOGATE_HOME_ENABLE (GIMBAL_POWERON_USE_PHOTOGATE || GIMBAL_SPIN_EXIT_REHOME_USE_PHOTOGATE)

//创建实例
gimbal_t gimbal;

static const fp32 GIMBAL_YAW_SAFE_TORQUE_SCALE = 0.5f;
static const fp32 GIMBAL_YAW_SAFE_TORQUE_LIMIT = 0.4f;

/********电机初始化设置*********/
DM_Motor_Setting_t yaw_setting = {0};
// LK_Motor_Setting_t test_setting = {"test",ON_CAN1,1,POSITIVE_DIRECT,SINGLE_LOOP};

/********电机控制器参数设置*********/
//WC-B0-WO-W-GAIN
fp32 yaw_init_config[]={25,0.0058f,100,0,0};
fp32 pit_init_config[]={15,0.0055f,60,0,0};
fp32 yaw_normal_config[]={23,0.007f,90,25,1};
fp32 pit_normal_config[]={30,0.008f,120,20,1};
fp32 yaw_auto_config[]={16,0.0065,80,25,0.2f};
fp32 pit_auto_config[]={17,0.0065,85,17,0.3f};

fp32 test_pid[]={1,0,0};

static Photogate_t yaw_photogate;
static fp32 yaw_encoder_absolute_rad = 0.0f;
volatile fp32 init_yaw_set_dbg = INIT_YAW_SET;
static fp32 yaw_find_dir = 1.0f;
static uint32_t yaw_find_time_cnt = 0U;
static uint8_t yaw_find_state_init = 0U;
#if GIMBAL_POWERON_USE_PHOTOGATE
static bool_t yaw_poweron_init_done = FALSE;
#endif
static bool_t yaw_spin_exit_rehome_pending = FALSE;
static Control_Mode_t yaw_last_raw_mode = ZERO_FORCE_MOVE;
static bool_t yaw_dm_enabled = FALSE;

static bool_t IsSpinMode(Control_Mode_t mode)
{
	return (mode == SPIN || mode == SPIN_AUTO) ? TRUE : FALSE;
}

static bool_t IsRehomeInjectMode(Control_Mode_t mode)
{
	return (mode == NORMAL || mode == RELATIVE_ANGLE || mode == LOCK_WHEEL || mode == NO_FOLLOW_YAW) ? TRUE : FALSE;
}

static void GimbalCenterDebugPrint(fp32 yaw_center_error)
{
#if GIMBAL_CENTER_DEBUG_PRINT_ENABLE
	static uint32_t tick = 0U;
	char line[128];
	UART_HandleTypeDef *print_uart = &huart1;

	tick++;
	if (tick < GIMBAL_CENTER_DEBUG_PRINT_PERIOD_MS)
	{
		return;
	}
	tick = 0U;

#if GIMBAL_CENTER_DEBUG_UART_PORT == 3
	print_uart = &huart3;
#elif GIMBAL_CENTER_DEBUG_UART_PORT == 6
	print_uart = &huart6;
#endif

	snprintf(line,
			 sizeof(line),
			 "[GIMBAL_CAL] yaw_enc=%.6f center_err=%.6f home=%u\r\n",
			 yaw_encoder_absolute_rad,
			 yaw_center_error,
			 (unsigned)PhotogateHomeValid(&yaw_photogate));
	USARTSend(print_uart, (uint8_t *)line, (uint16_t)strlen(line), USART_TRANSFER_BLOCKING);
#else
	(void)yaw_center_error;
#endif
}

static fp32 InitYawErrorRad(void)
{
	return rad_format(gimbal.gimbal_msg.yaw_relative_angle - INIT_YAW_SET);
}

static void ResetYawFindState()
{
	yaw_find_dir = (GIMBAL_INIT_YAW_FIND_DIR >= 0.0f) ? 1.0f : -1.0f;
	yaw_find_time_cnt = 0U;
	yaw_find_state_init = 1U;
}

static fp32 DMYawApplyDirection(fp32 torque)
{
	return (yaw_setting.direction == NEGATIVE_DIRECT) ? -torque : torque;
}

static hcan_t *DMYawCanHandle()
{
	return (yaw_setting.can_id == ON_CAN1) ? &hcan1 : &hcan2;
}

static void EnsureYawDMEnabled(DM_motor_t *yaw_dm)
{
	if (yaw_dm_enabled == FALSE)
	{
		enable_motor_mode(DMYawCanHandle(), yaw_dm->id, MIT_MODE);
		yaw_dm_enabled = TRUE;
	}
}
/**
	* @brief          gimbal类构造函数
  * @param[in]      NULL
  * @retval         NULL
  */
gimbal_t::gimbal_t()
{
	yaw_absolute_set_rad = yaw_absolute_rad = 0.0f;
	yaw_absolute_start_rad = 0.0f;
	yaw_home_absolute_rad = 0.0f;
	yaw_relative_set = 0.0f;
	pit_relative_set = 0.0f;
	add_yaw = 0.0f;
	add_pit = 0.0f;
	// pit_absolute_set_rad = pit_absolute_rad = 0.0f;
}

/**
  * @brief          云台初始化
  * @param[in]      NULL
  * @retval         NULL
  */
void GimbalInit()
{
	yaw_setting.motor_name = "yaw_dm";
	yaw_setting.can_id = ON_CAN2;
	yaw_setting.rx_id = 0x01;
	yaw_setting.mst_id = 0x00;
	yaw_setting.direction = POSITIVE_DIRECT;
	yaw_setting.dm_control_type = mit_mode;

	/******电机初始化******/
	gimbal.yaw_motor.DMMotorInit(&yaw_setting);
	gimbal.yaw_motor.DM_motor_para_init(&gimbal.yaw_motor.dm_motor[Motor1]);
	// Force first control cycle to send DM enable frame.
	yaw_dm_enabled = FALSE;
	PhotogateInit(&yaw_photogate,
				  PHOTOGATE_GPIO_Port,
				  PHOTOGATE_Pin,
				  PHOTOGATE_ACTIVE_LEVEL,
				  PHOTOGATE_STABLE_COUNT);
#if GIMBAL_POWERON_USE_PHOTOGATE
	yaw_poweron_init_done = FALSE;
#endif
	yaw_spin_exit_rehome_pending = FALSE;
	yaw_last_raw_mode = SysPointer()->mode;
	/******信息中心实例建立******/
	CenterPointer()->PointerInit(&gimbal.gimbal_msg,GIMBALPUB);
}

/**
	* @brief          云台控制器的初始化
  * @param[in]      type：控制器类型
  * @retval         NULL
  */
void gimbal_t::GimbalControllerInit(uint8_t type)
{
	if(SysPointer()->change_mode_flag)  //根据不同模式更改控制器参数
	{
		SysPointer()->change_mode_flag = 0;
		//各个模式控制器初始化
		if(type == INIT_PARAM)
		{
			ResetYawFindState();
			yaw_ladrc_fdw.Init(yaw_init_config, yaw_motor.dm_motor[Motor1].tmp.TMAX);
		}else if(type == NORMAL_PARAM)
		{
			// Enter NORMAL smoothly: lock absolute setpoint to current yaw to avoid follow-loop kick.
			yaw_absolute_set_rad = yaw_absolute_rad;
			yaw_absolute_start_rad = yaw_absolute_rad;
			// pit_absolute_set_rad = pit_absolute_rad;
			yaw_ladrc_fdw.Init(yaw_normal_config, yaw_motor.dm_motor[Motor1].tmp.TMAX);
			// pit_motor.controller.ladrc_fdw.Init(pit_normal_config,NULL);
		}else if(type == AUTO_PARAM)
		{
			yaw_ladrc_fdw.Init(yaw_auto_config, yaw_motor.dm_motor[Motor1].tmp.TMAX);
			// pit_motor.controller.ladrc_fdw.Init(pit_auto_config,NULL);
		}
	}
}

/**
	* @brief          云台主任务
  * @param[in]      NULL
  * @retval         NULL
  */
void GimbalTask()
{
	Control_Mode_t raw_mode;
	Control_Mode_t run_mode;

	//基础信息更新
	gimbal.BasicInfoUpdate();

	// RC offline protection: keep yaw motor disabled to avoid power-on self-rotation.
	if (MonitorRc() == FALSE)
	{
		DM_motor_t *yaw_dm = &gimbal.yaw_motor.dm_motor[Motor1];
		gimbal.yaw_motor.DMMotorZeroForce(DMYawCanHandle(), yaw_dm);
		yaw_dm_enabled = FALSE;
		BlinkLEDByCount(0xFFFF0000, 500);
		yaw_last_raw_mode = DT7_MISSING;
		return;
	}

	raw_mode = SysPointer()->mode;
	if (IsSpinMode(yaw_last_raw_mode) == TRUE && raw_mode == NORMAL)
	{
#if GIMBAL_SPIN_EXIT_REHOME_USE_PHOTOGATE
		yaw_spin_exit_rehome_pending = TRUE;
		PhotogateClearHome(&yaw_photogate);
		ResetYawFindState();
#endif
	}

	run_mode = raw_mode;
	if (yaw_spin_exit_rehome_pending == TRUE && IsRehomeInjectMode(raw_mode) == TRUE)
	{
		run_mode = INIT_MODE;
		SysPointer()->mode = INIT_MODE;
	}

	{
		DM_motor_t *yaw_dm = &gimbal.yaw_motor.dm_motor[Motor1];
		if (run_mode == DT7_MISSING || run_mode == ZERO_FORCE_MOVE)
		{
			if (yaw_dm_enabled == TRUE)
			{
				gimbal.yaw_motor.DMMotorZeroForce(DMYawCanHandle(), yaw_dm);
				yaw_dm_enabled = FALSE;
			}
		}
		else
		{
			// Non-zero-force modes: enable once when entering the mode.
			EnsureYawDMEnabled(yaw_dm);
		}
	}

	//根据不同模式选择不同控制量信息来源
	switch(run_mode)
	{
		//遥控器作为输入量（操作模式）
		case NORMAL:
		case LOCK_WHEEL:
		case SPIN:
			gimbal.GimbalControllerInit(NORMAL_PARAM); 					//控制器初始化
			gimbal.OperationInfoUpdate();      		   					//操作时信息更新
			gimbal.NormalControl();                    					//进入控制器
//			MonitorPointer()->state = SetLEDWorkType(GREEN_SLOW);
			BlinkLEDByCount(0xFF00FF00,500);							//绿色慢闪
			break;
		//USBCDC串口作为输入量（自瞄模式）
		case AUTO:
		case SPIN_AUTO:
			gimbal.GimbalControllerInit(AUTO_PARAM); 			       	//控制器初始化
			gimbal.AutoInfoUpdate(); 								   	//自瞄信息更新
			gimbal.NormalControl(); 									//进入控制器
//			MonitorPointer()->state = SetLEDWorkType(BLUE_SLOW); 		//设置灯颜色为蓝色慢闪
			BlinkLEDByCount(0xFF0000FF,500);							//蓝色慢闪
			break;
		//初始化模式
		case INIT_MODE:
			gimbal.GimbalControllerInit(INIT_PARAM); 					//控制器初始化
			gimbal.InitInfoUpdate();  									//初始化信息更新
			gimbal.JudgeInitState();									//判断初始化状态
			gimbal.RelativeControl();									//进入控制器
//			MonitorPointer()->state = SetLEDWorkType(BLUE_TWO_BLINK);	//设置灯颜色为蓝色双闪
			BlinkLEDByCount(0xFFFFFF00,500);							//黄色慢闪
			break;
		//无力及丢失遥控器失控模式
		case DT7_MISSING:
//			MonitorPointer()->state = SetLEDWorkType(YELLOW_TWO_BLINK); //设置灯颜色为黄色双闪
			gimbal.ZeroForceControl();									//电机无力控制
			BlinkLEDByCount(0xFFFF0000,500);							//红灯闪烁
			break;
		case ZERO_FORCE_MOVE:
			gimbal.ZeroForceControl();									//电机无力控制
//			MonitorPointer()->state = SetLEDWorkType(CYAN_SLOW);	    //设置灯颜色为青色闪烁
			BlinkLEDByCount(0xFF00FFFF,500);							//青色闪烁
			break;
		//相对角度控制模式（底盘），不对云台进行控制
		case RELATIVE_ANGLE:
		case NO_FOLLOW_YAW: 
			BlinkLEDByCount(0xFF00FF00,500);
			gimbal.OperationInfoUpdate();
			gimbal.RelativeControl();
			break;
	}

	yaw_last_raw_mode = raw_mode;
}

/**
	* @brief          基础信息更新
  * @param[in]      NULL
  * @retval         NULL
  */
fp32 LK_Motor_speed = 0;
void gimbal_t::BasicInfoUpdate()
{
//	int n = 0;
	/**************绝对角度（陀螺仪）信息更新****************/
	//陀螺仪绝对角度转化成弧度制
//	if ((rc_ctrl.rc.ch[0]<-600&&rc_ctrl.rc.ch[1]<-600&&rc_ctrl.rc.ch[2]>600&&rc_ctrl.rc.ch[3]<
//		-600)&&(n==0))//拨杆向下内八
//	{
//	yaw_absolute_start_rad = yaw_absolute_rad;                                                 //添加
//		n = 1;
//	}
	yaw_absolute_rad = *(get_INS_angle_point() + INS_YAW_ADDRESS_OFFSET);
	// Pitch loop disabled: keep pitch state fixed to avoid affecting yaw path.
	pit_absolute_rad = 0.0f;
	yaw_encoder_absolute_rad = yaw_motor.DMMotorEcdToAngle(0.0f);

#if GIMBAL_PHOTOGATE_HOME_ENABLE
	PhotogateUpdateWithYaw(&yaw_photogate, yaw_encoder_absolute_rad);
#endif
	//陀螺仪角速度
	roll_gyro = *(get_gyro_data_point() + INS_GYRO_X_ADDRESS_OFFSET);
	pitch_gyro = 0.0f;
	yaw_gyro = arm_cos_f32(gimbal_msg.pit_relative_angle) * (*(get_gyro_data_point() + INS_GYRO_Z_ADDRESS_OFFSET))-
						 arm_sin_f32(gimbal_msg.pit_relative_angle) * (*(get_gyro_data_point() + INS_GYRO_X_ADDRESS_OFFSET));
	
	/**************相对角度（电机）信息更新****************/
	//电机相对角度（优先使用光电门+编码值）

#if GIMBAL_PHOTOGATE_HOME_ENABLE
	if (PhotogateHomeValid(&yaw_photogate) == TRUE)
	{
		gimbal_msg.yaw_relative_angle = GIMBAL_YAW_SIGN * PhotogateYawRelativeRad(&yaw_photogate, yaw_encoder_absolute_rad);
	}
	else
#endif
	{
		// During INIT, close the loop directly with encoder angle so homing converges to
		// GIMBAL_POWERON_ENCODER_CENTER_RAD instead of mixing encoder target with IMU feedback.
		if (SysPointer()->mode == INIT_MODE)
		{
			gimbal_msg.yaw_relative_angle = GIMBAL_YAW_SIGN * rad_format(yaw_encoder_absolute_rad - GIMBAL_POWERON_ENCODER_CENTER_RAD);
		}
		else
		{
			gimbal_msg.yaw_relative_angle = GIMBAL_YAW_SIGN * rad_format(yaw_absolute_rad - yaw_absolute_start_rad);
		}
	}
	// gimbal_msg.pit_relative_angle = 1.0f * PIT_TRANSMISSION_RADIO * pit_motor.MotorEcdToAngle(NULL,PIT_OFFSET,MOTOR_TO_PIT_RADIO);//改动
	// test_motor.LKMotorControl(NULL,&LK_Motor_speed,NULL,OFF_SET);
	// test_motor.EncodeTorqueControlData(test_motor.controller.send_current);
	// test_motor.EncodeTorqueControlData(test_motor.controller.send_current);
	// test_motor.EncodeSpeedControlData(10);
	// gimbal_msg.yaw_down_relative_angle = test_motor.LKMotorEcdToAngle(5000,1);
}

/**
	* @brief          操作时信息更新
  * @param[in]      NULL
  * @retval         NULL
  */
void gimbal_t::OperationInfoUpdate()
{
    static fp32 gimbal_end_angle = 0.0f;
    const bool_t yaw_idle = (fabs(SysPointer()->add_yaw) <= GIMBAL_YAW_IDLE_INPUT_EPS) ? TRUE : FALSE;
    // 拨杆拨动时持续转动到限幅并保持，拨杆回到死区时回到INIT_YAW_SET
    if(SysPointer()->mode == NORMAL)
    {
        if (yaw_idle == TRUE)
        {
            // 死区：自动回到初始位置
            add_yaw = 0.0f;
            yaw_absolute_set_rad = INIT_YAW_SET;
        }
        else
        {
			fp32 next_relative_angle;

			// NORMAL 模式下限幅应基于云台相对底盘的角度，而不是陀螺仪世界绝对角。
			next_relative_angle = gimbal_msg.yaw_relative_angle + SysPointer()->add_yaw;
			next_relative_angle = fp32_constrain(next_relative_angle, MIN_YAW_RELATIVE, MAX_YAW_RELATIVE);
			yaw_absolute_set_rad = rad_format(yaw_absolute_start_rad + next_relative_angle);
            add_yaw = 0.0f; // 由MIT模式扭矩控制
        }
    }
    else if(SysPointer()->mode == SPIN)
    {
        add_yaw = SysPointer()->add_yaw;
    }
    else if(SysPointer()->mode == RELATIVE_ANGLE || SysPointer()->mode == NO_FOLLOW_YAW)
    {
        add_yaw = SysPointer()->add_yaw;
    }
    // 一键掉头判断
    if(!SysPointer()->key_flag.turn_round_flag)
    {
        gimbal_end_angle = rad_format(yaw_absolute_rad + PI);
    }
    if(SysPointer()->key_flag.turn_round_flag)
    {
        if (rad_format(gimbal_end_angle - yaw_absolute_set_rad) > 0.0f)
            add_yaw += TURN_SPEED;
        else
            add_yaw -= TURN_SPEED;
        if(fabs(rad_format(gimbal_end_angle - yaw_absolute_rad)) < 0.01f)
            SysPointer()->key_flag.turn_round_flag = 0;
    }
    add_pit = 0.0f;
    yaw_absolute_set_rad = rad_format(yaw_absolute_set_rad + add_yaw);
}

void gimbal_t::NormalControl()
{
    DM_motor_t *yaw_dm = &yaw_motor.dm_motor[Motor1];
    fp32 yaw_tor_cmd = 0.0f;
	const fp32 yaw_relative_angle = gimbal_msg.yaw_relative_angle;
    // 判断拨杆是否回中
    const bool_t yaw_idle = (fabs(SysPointer()->add_yaw) <= GIMBAL_YAW_IDLE_INPUT_EPS) ? TRUE : FALSE;
    if (yaw_idle == TRUE)
    {
        // 拨杆回中，采用MIT模式位置控制回INIT_YAW_SET
        yaw_dm->dm_ctrl_set.mode = mit_mode;
        yaw_dm->dm_ctrl_set.pos_set = INIT_YAW_SET;
        yaw_dm->dm_ctrl_set.vel_set = 1.0f;
        yaw_dm->dm_ctrl_set.kp_set = 13.50f;
        yaw_dm->dm_ctrl_set.kd_set = 4.50f;
        yaw_dm->dm_ctrl_set.tor_set = 0.0f;
    }
    else
    {
        // 拨杆拨动，MIT模式扭矩控制
        yaw_dm->dm_ctrl_set.mode = mit_mode;
        yaw_dm->dm_ctrl_set.pos_set = 0.0f;
        yaw_dm->dm_ctrl_set.vel_set = 0.0f;
        yaw_dm->dm_ctrl_set.kp_set = 0.0f;
        yaw_dm->dm_ctrl_set.kd_set = 0.0f;
		if (fabs(yaw_absolute_set_rad - yaw_absolute_rad) > 0.01f)
		{
			// 向右拨动摇杆时云台转矩为负，向左拨动时为正
			yaw_tor_cmd = (SysPointer()->add_yaw > 0) ? GIMBAL_YAW_SAFE_TORQUE_LIMIT : -GIMBAL_YAW_SAFE_TORQUE_LIMIT;
			if (SysPointer()->add_yaw > 0.0f && yaw_relative_angle >= MAX_YAW_RELATIVE)
				yaw_tor_cmd = 0.0f;
			if (SysPointer()->add_yaw < 0.0f && yaw_relative_angle <= MIN_YAW_RELATIVE)
				yaw_tor_cmd = 0.0f;
		}
		else
		{
			yaw_tor_cmd = 0.0f;
		}
        yaw_dm->dm_ctrl_set.tor_set = DMYawApplyDirection(yaw_tor_cmd);
    }
    yaw_motor.DMMotorControl(DMYawCanHandle(), yaw_dm);
    // pit_motor.DJIMotorControl(&pit_absolute_rad,&pit_absolute_set_rad,&pitch_gyro,OFF_SET);
}

/**
	* @brief          无力模式控制
  * @param[in]      NULL
  * @retval         NULL
  */
void gimbal_t::ZeroForceControl()
{
	DM_motor_t *yaw_dm = &yaw_motor.dm_motor[Motor1];
	yaw_motor.DMMotorZeroForce(DMYawCanHandle(), yaw_dm);
	yaw_dm_enabled = FALSE;
	// pit_motor.MotorZeroForce();
}

/**
	* @brief          相对角度控制
  * @param[in]      NULL
  * @retval         NULL
  */
void gimbal_t::RelativeControl()
{
	DM_motor_t *yaw_dm = &yaw_motor.dm_motor[Motor1];
	// Re-arm enable in case mode transition skipped the one-shot enable path.
	EnsureYawDMEnabled(yaw_dm);

	if (SysPointer()->mode == INIT_MODE)
	{
		// During INIT, directly command DM internal position loop to INIT_YAW_SET.
		yaw_dm->dm_ctrl_set.mode = mit_mode;
		yaw_dm->dm_ctrl_set.pos_set = INIT_YAW_SET;
		yaw_dm->dm_ctrl_set.vel_set = 1.0f;
		yaw_dm->dm_ctrl_set.kp_set = 40.0f;
		yaw_dm->dm_ctrl_set.kd_set = 1.0f;
		yaw_dm->dm_ctrl_set.tor_set = 0.0f;

		yaw_motor.DMMotorControl(DMYawCanHandle(), yaw_dm);
		return;
	}

	if (SysPointer()->mode == RELATIVE_ANGLE)
	{
		// In RELATIVE_ANGLE, hold fixed yaw target with DM internal position loop.
		yaw_dm->dm_ctrl_set.mode = mit_mode;
		yaw_dm->dm_ctrl_set.pos_set = INIT_YAW_SET;
		yaw_dm->dm_ctrl_set.vel_set = 1.0f;
		yaw_dm->dm_ctrl_set.kp_set = 15.0f;
		yaw_dm->dm_ctrl_set.kd_set = 4.50f;
		yaw_dm->dm_ctrl_set.tor_set = 0.0f;

		yaw_motor.DMMotorControl(DMYawCanHandle(), yaw_dm);
		return;
	}

	//正常相对角度控制角速度应该也使用电机的，不使用陀螺仪的角速度
	yaw_relative_set += add_yaw;
	if (yaw_relative_set > MAX_YAW_RELATIVE)
	{
		yaw_relative_set = MAX_YAW_RELATIVE;
	}
	else if (yaw_relative_set < MIN_YAW_RELATIVE)
	{
		yaw_relative_set = MIN_YAW_RELATIVE;
	}
	// pit_relative_set=pit_motor.MotorWorkSpaceLimit(pit_relative_set,add_pit,MAX_PIT_RELATIVE,MIN_PIT_RELATIVE);
	
	fp32 yaw_tor_cmd;

	yaw_dm->dm_ctrl_set.mode = mit_mode;
	yaw_dm->dm_ctrl_set.pos_set = 0.0f;
	yaw_dm->dm_ctrl_set.vel_set = 0.0f;
	yaw_dm->dm_ctrl_set.kp_set = 0.0f;
	yaw_dm->dm_ctrl_set.kd_set = 0.0f;
	yaw_tor_cmd = yaw_ladrc_fdw.FDW_Calc(gimbal_msg.yaw_relative_angle, yaw_relative_set, yaw_gyro);
	yaw_tor_cmd *= GIMBAL_YAW_SAFE_TORQUE_SCALE;
	if (yaw_tor_cmd > GIMBAL_YAW_SAFE_TORQUE_LIMIT)
	{
		yaw_tor_cmd = GIMBAL_YAW_SAFE_TORQUE_LIMIT;
	}
	else if (yaw_tor_cmd < -GIMBAL_YAW_SAFE_TORQUE_LIMIT)
	{
		yaw_tor_cmd = -GIMBAL_YAW_SAFE_TORQUE_LIMIT;
	}
	yaw_dm->dm_ctrl_set.tor_set = DMYawApplyDirection(yaw_tor_cmd);

	yaw_motor.DMMotorControl(DMYawCanHandle(), yaw_dm);
	// pit_motor.DJIMotorControl(&gimbal_msg.pit_relative_angle,&pit_relative_set,&pitch_gyro,OFF_SET);
}

/**
	* @brief          初始化信息更新
  * @param[in]      NULL
  * @retval         NULL
  */
void gimbal_t::InitInfoUpdate()
{
	bool_t need_photogate_find = FALSE;
	fp32 yaw_center_error = 0.0f;

#if GIMBAL_POWERON_USE_PHOTOGATE
	if (yaw_poweron_init_done == FALSE)
	{
		need_photogate_find = TRUE;
	}
#endif

	if (yaw_spin_exit_rehome_pending == TRUE)
	{
		need_photogate_find = TRUE;
	}

	yaw_center_error = InitYawErrorRad();
	GimbalCenterDebugPrint(yaw_center_error);

	if (need_photogate_find == TRUE && PhotogateHomeValid(&yaw_photogate) == FALSE) //按策略找光电门
	{
		if (yaw_find_state_init == 0U)
		{
			ResetYawFindState();
		}

		yaw_find_time_cnt++;
#if GIMBAL_INIT_YAW_BIDIR_SEARCH
		if (yaw_find_time_cnt >= GIMBAL_INIT_YAW_REVERSE_TICKS)
		{
			yaw_find_dir = -yaw_find_dir;
			yaw_find_time_cnt = 0U;
		}
#endif

		add_pit = 0.0f;
		add_yaw = GIMBAL_INIT_YAW_FIND_SPEED * yaw_find_dir;
	}
	else // 光电门有效时使用相对角；上电不找门时使用达妙绝对编码角回中
	{
		fp32 yaw_for_center;

		ResetYawFindState();
		add_pit = 0.0f;
		if (PhotogateHomeValid(&yaw_photogate) == TRUE)
		{
			yaw_for_center = gimbal_msg.yaw_relative_angle;
		}
		else
		{
			yaw_for_center = GIMBAL_YAW_SIGN * rad_format(yaw_encoder_absolute_rad - GIMBAL_POWERON_ENCODER_CENTER_RAD);
		}

		add_yaw = (INIT_YAW_SET - yaw_for_center) * GIMBAL_INIT_YAW_SPEED;
	}
}

/**
	* @brief          判断初始化状态
  * @param[in]      NULL
  * @retval         NULL
  */
void gimbal_t::JudgeInitState()
{
  static uint16_t init_time = 0;
  static uint16_t init_stop_time = 0;
	fp32 yaw_error = 0.0f;

	if (yaw_spin_exit_rehome_pending == TRUE && PhotogateHomeValid(&yaw_photogate) == FALSE)
	{
		return;
	}

	yaw_error = InitYawErrorRad();
        
	//目标值与当前值之差小于阈值超过一定时间，则判断初始化完成
	if(fabs(yaw_error) < GIMBAL_INIT_ANGLE_ERROR)
  {        
    if(init_stop_time < GIMBAL_INIT_STOP_TIME)
      init_stop_time++;
  }else
  {     
    if(init_time < GIMBAL_INIT_TIME)
	  init_time++;
  }

	//超过初始化最大时间，或者已经稳定到中值一段时间，退出初始化状态开关打下档，或者掉线
  if(init_time < GIMBAL_INIT_TIME && init_stop_time < GIMBAL_INIT_STOP_TIME)
  {
    return;
  }else  //初始化完成
  {
    init_stop_time = 0;
    init_time = 0;
		yaw_home_absolute_rad = yaw_absolute_rad;
#if !GIMBAL_PHOTOGATE_HOME_ENABLE
	// No photogate on power-on: freeze absolute-loop reference at current IMU yaw.
	yaw_absolute_start_rad = yaw_absolute_rad;
#endif
	// Synchronize setpoints before leaving init to avoid first-frame jump.
	yaw_absolute_set_rad = yaw_absolute_rad;
	yaw_relative_set = gimbal_msg.yaw_relative_angle;
#if GIMBAL_POWERON_USE_PHOTOGATE
	yaw_poweron_init_done = TRUE;
#endif
	yaw_spin_exit_rehome_pending = FALSE;
	ResetYawFindState();
	SysPointer()->mode = NORMAL;
	SysPointer()->change_mode_flag = 1;
  }
}

void gimbal_t::AutoInfoUpdate()
{
    // 空实现，防止链接错误
}
//LKMotorInstance *GetLKPointer()
//{
//	return &gimbal.test_motor;
//}
