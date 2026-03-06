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

//创建实例
gimbal_t gimbal;

/********电机初始化设置*********/
DM_Motor_Setting_t yaw_setting = {0};
Motor_Setting_t height_setting = {"height",ON_CAN2,5,M2006,POSITIVE_DIRECT,RATIO_1_TO_36,CASCADE_LOOP};
// LK_Motor_Setting_t test_setting = {"test",ON_CAN1,1,POSITIVE_DIRECT,SINGLE_LOOP};

/********电机控制器参数设置*********/
//WC-B0-WO-W-GAIN
fp32 yaw_init_config[]={25,0.0058f,100,0,0};
fp32 pit_init_config[]={15,0.0055f,60,0,0};
fp32 yaw_normal_config[]={23,0.007f,90,25,1};
fp32 pit_normal_config[]={30,0.008f,120,20,1};
fp32 yaw_auto_config[]={16,0.0065,80,25,0.2f};
fp32 pit_auto_config[]={17,0.0065,85,17,0.3f};

fp32 height_motor_speed_pid[3]={1500,0,500};
fp32 height_motor_angle_pid[3]={0.01,0,0.005};

fp32 test_pid[]={1,0,0};

static Photogate_t yaw_photogate;
static fp32 yaw_encoder_absolute_rad = 0.0f;
static fp32 yaw_find_dir = 1.0f;
static uint32_t yaw_find_time_cnt = 0U;
static uint8_t yaw_find_state_init = 0U;
#if GIMBAL_POWERON_USE_PHOTOGATE
static bool_t yaw_poweron_init_done = FALSE;
#endif
static bool_t yaw_spin_exit_rehome_pending = FALSE;
static Control_Mode_t yaw_last_raw_mode = ZERO_FORCE_MOVE;

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
	if (PhotogateHomeValid(&yaw_photogate) == TRUE)
	{
		return gimbal.gimbal_msg.yaw_relative_angle - INIT_YAW_SET;
	}

	// 上电不使用光电门时，使用达妙绝对编码角与标定中心值计算误差
	return (yaw_encoder_absolute_rad - GIMBAL_POWERON_ENCODER_CENTER_RAD) - INIT_YAW_SET;
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
/**
	* @brief          gimbal类构造函数
  * @param[in]      NULL
  * @retval         NULL
  */
gimbal_t::gimbal_t()
{
	yaw_absolute_set_rad = yaw_absolute_rad = 0.0f;
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
	yaw_setting.mst_id = 0x02;
	yaw_setting.direction = POSITIVE_DIRECT;
	yaw_setting.dm_control_type = mit_mode;

	/******电机初始化******/
	gimbal.yaw_motor.DMMotorInit(&yaw_setting);
	gimbal.yaw_motor.DM_motor_para_init(&gimbal.yaw_motor.dm_motor[Motor1]);
	gimbal.height_motor.DJIMotorInit(&height_setting);
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
			height_motor.controller.speed_PID.Init(PID_POSITION,height_motor_speed_pid,10000,5000);
			height_motor.controller.angle_PID.Init(PID_POSITION,height_motor_angle_pid,50,5);
		}else if(type == NORMAL_PARAM)
		{
			yaw_absolute_set_rad = yaw_absolute_start_rad;//= yaw_absolute_rad           改动
			yaw_absolute_start_rad = yaw_absolute_rad;                                 //添加
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
			//不进行控制
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
	pit_absolute_rad = *(get_INS_angle_point() + INS_PITCH_ADDRESS_OFFSET);
	yaw_encoder_absolute_rad = yaw_motor.DMMotorEcdToAngle(0.0f);
	PhotogateUpdateWithYaw(&yaw_photogate, yaw_encoder_absolute_rad);
	//陀螺仪角速度
	roll_gyro = *(get_gyro_data_point() + INS_GYRO_X_ADDRESS_OFFSET);
	pitch_gyro = *(get_gyro_data_point() + INS_GYRO_Y_ADDRESS_OFFSET);
	yaw_gyro = arm_cos_f32(gimbal_msg.pit_relative_angle) * (*(get_gyro_data_point() + INS_GYRO_Z_ADDRESS_OFFSET))-
						 arm_sin_f32(gimbal_msg.pit_relative_angle) * (*(get_gyro_data_point() + INS_GYRO_X_ADDRESS_OFFSET));
	
	/**************相对角度（电机）信息更新****************/
	//电机相对角度（优先使用光电门+编码值）
	if (PhotogateHomeValid(&yaw_photogate) == TRUE)
	{
		gimbal_msg.yaw_relative_angle = PhotogateYawRelativeRad(&yaw_photogate, yaw_encoder_absolute_rad);
	}
	else
	{
		gimbal_msg.yaw_relative_angle = yaw_absolute_rad - yaw_absolute_start_rad;
	}
	// gimbal_msg.pit_relative_angle = 1.0f * PIT_TRANSMISSION_RADIO * pit_motor.MotorEcdToAngle(NULL,PIT_OFFSET,MOTOR_TO_PIT_RADIO);//改动
	// test_motor.LKMotorControl(NULL,&LK_Motor_speed,NULL,OFF_SET);
//	test_motor.EncodeTorqueControlData(test_motor.controller.send_current);
	// test_motor.EncodeTorqueControlData(test_motor.controller.send_current);
//	test_motor.EncodeSpeedControlData(10);
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
	if(SysPointer()->mode == NORMAL)     //底盘跟随云台模式
	{
		//add值更新并限幅
		add_yaw = AbsoluteControlAddLimit((yaw_absolute_set_rad-yaw_absolute_rad),SysPointer()->add_yaw,gimbal_msg.yaw_relative_angle,MAX_YAW_RELATIVE,MIN_YAW_RELATIVE);
	}else if(SysPointer()->mode == SPIN) //小陀螺模式
	{
		//add值更新，不进行限幅
		add_yaw = SysPointer()->add_yaw;
	}
	
	//一键掉头判断
	if(!SysPointer()->key_flag.turn_round_flag)
	{
		gimbal_end_angle = rad_format(yaw_absolute_rad + PI);
	}
	if(SysPointer()->key_flag.turn_round_flag)
	{
		//不断控制到掉头的目标值，正转，反装是随机
        if (rad_format(gimbal_end_angle - yaw_absolute_set_rad) > 0.0f)
			add_yaw += TURN_SPEED;
        else
            add_yaw -= TURN_SPEED;
		
		//到达pi （180°）后停止
		if(fabs(rad_format(gimbal_end_angle - yaw_absolute_rad)) < 0.01f)
		    SysPointer()->key_flag.turn_round_flag = 0;
	}
	
	//add值更新并限幅
	add_pit=AbsoluteControlAddLimit((pit_absolute_set_rad-pit_absolute_rad),SysPointer()->add_pit,gimbal_msg.pit_relative_angle,MAX_PIT_RELATIVE,MIN_PIT_RELATIVE);
	
	//弧度制赋值并限制
	yaw_absolute_set_rad = rad_format(yaw_absolute_set_rad + add_yaw);
	// pit_absolute_set_rad = rad_format(pit_absolute_set_rad + add_pit);
}

/**
	* @brief          自瞄信息更新
  * @param[in]      NULL
  * @retval         NULL
  */
void gimbal_t::AutoInfoUpdate()
{
// 	fp32 auto_error_yaw,auto_error_pit;
// 	/*****获取相对角度值*****/
// 	VisionErrorAngleYaw(&auto_error_yaw);
// 	VisionErrorAnglePit(&auto_error_pit);
	
// 	/*****数据处理方式*****/
// #if Auto_Type == HANDLE_LPF
// 	//瞄准到目标进行控制
// 	if( VisionGetIfTarget() )
// 	{
// 		//一节低通滤波
// 		add_yaw = LPF(&yaw_vision_lpf ,0.001,auto_error_yaw,550); //800 812 875 750 843 781 769 687
// 		add_pit = LPF(&pitch_vision_lpf ,0.001,auto_error_pit,1300);
		
// 		//数据异常处理
// 		if(isnan(add_yaw) || isinf(add_yaw))
// 		{
// 			add_yaw = 0.0;
// 		}
// 		if(isnan(add_pit) || isinf(add_pit))
// 		{ 
// 			add_pit = 0.0;
// 		}
// 	}else
// 	{
// 		add_yaw = add_pit = 0.0f;
// 	}
// #elif Auto_Type == HANDLE_KALMAN
	
// #endif
	
// 	/*****处理数据后进行限幅处理*****/
// 	if(SysPointer()->mode == AUTO)  //自瞄模式
// 	{
// 		//add值更新并限幅
// 		add_yaw=AbsoluteControlAddLimit((yaw_absolute_set_rad-yaw_absolute_rad),add_yaw,gimbal_msg.yaw_relative_angle,MAX_YAW_RELATIVE,MIN_YAW_RELATIVE);
// 	}else if(SysPointer()->mode == SPIN_AUTO)  //小陀螺自瞄模式
// 	{
// 		//add值更新，不进行限幅
// 		add_yaw = SysPointer()->add_yaw;
// 	}
	
// 	//角度值更新及限制
// 	add_pit=AbsoluteControlAddLimit((pit_absolute_set_rad-pit_absolute_rad),add_pit,gimbal_msg.pit_relative_angle,MAX_PIT_RELATIVE,MIN_PIT_RELATIVE);
	
// 	//进行赋值
// 	yaw_absolute_set_rad = rad_format(yaw_absolute_rad + add_yaw);
// 	pit_absolute_set_rad = rad_format(pit_absolute_rad + add_pit);
}

/**
	* @brief          正常绝对角度控制--正常
  * @param[in]      NULL
  * @retval         NULL
  */
void gimbal_t::NormalControl()
{
	DM_motor_t *yaw_dm = &yaw_motor.dm_motor[Motor1];

	yaw_dm->dm_ctrl_set.pos_set = 0.0f;
	yaw_dm->dm_ctrl_set.vel_set = 0.0f;
	yaw_dm->dm_ctrl_set.kp_set = 0.0f;
	yaw_dm->dm_ctrl_set.kd_set = 0.0f;
	yaw_dm->dm_ctrl_set.tor_set = DMYawApplyDirection(
		yaw_ladrc_fdw.FDW_Calc(yaw_absolute_rad, yaw_absolute_set_rad, yaw_gyro));

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
	yaw_motor.DMMotorZeroForce(DMYawCanHandle(), &yaw_motor.dm_motor[Motor1]);
	// pit_motor.MotorZeroForce();
}

/**
	* @brief          相对角度控制
  * @param[in]      NULL
  * @retval         NULL
  */
void gimbal_t::RelativeControl()
{
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
	
	DM_motor_t *yaw_dm = &yaw_motor.dm_motor[Motor1];

	yaw_dm->dm_ctrl_set.pos_set = 0.0f;
	yaw_dm->dm_ctrl_set.vel_set = 0.0f;
	yaw_dm->dm_ctrl_set.kp_set = 0.0f;
	yaw_dm->dm_ctrl_set.kd_set = 0.0f;
	yaw_dm->dm_ctrl_set.tor_set = DMYawApplyDirection(
		yaw_ladrc_fdw.FDW_Calc(gimbal_msg.yaw_relative_angle, yaw_relative_set, yaw_gyro));

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

	if(fabs(INIT_PITCH_SET - pit_absolute_rad) > GIMBAL_INIT_ANGLE_ERROR)  //先PITCH轴初始化
	{
		ResetYawFindState();
		add_pit = (INIT_PITCH_SET - pit_absolute_rad) * GIMBAL_INIT_PITCH_SPEED;
		add_yaw = 0.0f;
	}
	else if (need_photogate_find == TRUE && PhotogateHomeValid(&yaw_photogate) == FALSE) //按策略找光电门
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

		add_pit = (INIT_PITCH_SET - pit_absolute_rad) * GIMBAL_INIT_PITCH_SPEED;
		add_yaw = GIMBAL_INIT_YAW_FIND_SPEED * yaw_find_dir;
	}
	else // 光电门有效时使用相对角；上电不找门时使用达妙绝对编码角回中
	{
		fp32 yaw_for_center;

		ResetYawFindState();
		add_pit = (INIT_PITCH_SET - pit_absolute_rad) * GIMBAL_INIT_PITCH_SPEED;
		if (PhotogateHomeValid(&yaw_photogate) == TRUE)
		{
			yaw_for_center = gimbal_msg.yaw_relative_angle;
		}
		else
		{
			yaw_for_center = yaw_encoder_absolute_rad - GIMBAL_POWERON_ENCODER_CENTER_RAD;
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
	if((fabs(yaw_error) < GIMBAL_INIT_ANGLE_ERROR &&
      fabs(pit_absolute_rad - INIT_PITCH_SET) < GIMBAL_INIT_ANGLE_ERROR))
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
#if GIMBAL_POWERON_USE_PHOTOGATE
	yaw_poweron_init_done = TRUE;
#endif
	yaw_spin_exit_rehome_pending = FALSE;
	ResetYawFindState();
	SysPointer()->mode = NORMAL;
  }
}

//LKMotorInstance *GetLKPointer()
//{
//	return &gimbal.test_motor;
//}
