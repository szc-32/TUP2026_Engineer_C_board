/**
 ******************************************************************************
 * @file    chassis.cpp
 * @author  Xushuang
 * @version V1.0.0 基本完成
 * @date    2023/9/20
 * @brief		此处为底盘各模式控制
 ******************************************************************************
 * @attention
 *	底盘四个电机顺序已经规定好，LF、LB、RB、RF顺序为0、1、2、3。即chassis_motor
 *数组中0-3分别为左前、左后、右前、右后的顺序，每个速度的变量亦是此顺序。
 *	故移植到新底盘时仅需在程序的电机实例初始化处更改底盘每个轮子的ID即可。
 ******************************************************************************
 */
#include "arm_math.h"
#include "chassis.h"
#include "referee_data.h"
#include "gimbal.h"
#include "cmsis_os.h"
#include "gpio.h"
#include "struct_typedef.h"
#include "user_lib.h"
#include "bsp_can.h"
#include "pid.h"
#include "Info_update_Task.h"
#include "stm32f407xx.h"
//车的后方为X轴正方向，右方为Y轴正方向，上方为Z轴正方向

//底盘实例创建
chassis_t chassis;
chassis_msg_t chassis_msg = {0};

// 0-100随机数序列
uint8_t random_series[50] = {17, 50, 76, 58, 36, 22, 45, 12, 93, 97, 11, 36, 65, 75, 39, 51, 52, 16, 15, 80, 84, 19, 17, 66, 63,
                             75, 47, 77, 80, 33, 92, 7, 58, 11, 87, 78, 88, 56, 50, 49, 58, 38, 44, 15, 2, 80, 34, 75, 58, 43};


/**
  * @brief          底盘类指针
  * @param[in]      null
  * @retval         底盘对象
  */
chassis_t* chassispoint(void)
{
	return &chassis;
}

/**
 * @brief 电机初始化设置
 */
Motor_Setting_t chassis_config[4] = 
{
	{"chassis_lf",ON_CAN2,1,M3508,POSITIVE_DIRECT,RATIO_1_TO_19,SINGLE_LOOP},    //LF
	{"chassis_lb",ON_CAN2,2,M3508,POSITIVE_DIRECT,RATIO_1_TO_19,SINGLE_LOOP},	 //LB
	{"chassis_rb",ON_CAN2,3,M3508,POSITIVE_DIRECT,RATIO_1_TO_19,SINGLE_LOOP},	 //RB
	{"chassis_rf",ON_CAN2,4,M3508,POSITIVE_DIRECT,RATIO_1_TO_19,SINGLE_LOOP},	 //RF
};

/**
	* @brief          底盘类构造函数
  * @param[in]      NULL
  * @retval         NULL
  */
chassis_t::chassis_t()
{
	chassis_max_power = 60;
}

/**
	* @brief          底盘初始化
  * @param[in]      NULL
  * @retval         NULL
  */
void chassis_t::ChassisInit()
{
//获取陀螺仪地址
	chassis_INS_angle = get_INS_angle_point();
	//获取控制量
	chassis_order = get_Gimbal_Oreder();

//IFSHIFT = get_UI_measure();
	chassis_cap_masure = get_cap_measure_point();
	robo_cap_measure = get_robo_cap_measure_point();
	//Ina226_Chassis_Power = get_ina226_data();
  //初始化底盘电机pid 旋转环pid
  const static fp32 chassis_yaw_pid[3] = {CHASSIS_MOTOR_YAW_PID_KP,CHASSIS_MOTOR_YAW_PID_KI,CHASSIS_MOTOR_YAW_PID_KD};  
	chassis_angle_pid.Init(PID_POSITION,chassis_yaw_pid, CHASSIS_MOTOR_YAW_PID_MAX_OUT, CHASSIS_MOTOR_YAW_PID_MAX_IOUT);
	
	const static fp32 motor_speed_pid[3] ={CHASSIS_MOTOR_SPEED_PID_KP, CHASSIS_MOTOR_SPEED_PID_KI, CHASSIS_MOTOR_SPEED_PID_KD};
	const static fp32 motor_current_pid[3] = {10,0,2};
	for(int i=0;i<=3;i++)
   {
	    chassis_motor_speed_pid[i].Init(PID_POSITION,motor_speed_pid,M3508_MOTOR_SPEED_PID_MAX_OUT, M3508_MOTOR_SPEED_PID_MAX_IOUT);
		chassis_current_pid[i].Init(PID_POSITION,motor_current_pid,16384, 1000);
	}
	updata=1;
	Get_Information();


#if BOARD_NUM == TWO_BOARD && BOARD_PLACE == ON_GIMBAL  //在云台上并且双板控制时
	/******初始化底盘电机旋转环PID******/
	chassis.chassis_angle_pid.Init(PID_POSITION,chassis_yaw_pid,CHASSIS_MOTOR_YAW_PID_MAX_OUT,CHASSIS_MOTOR_YAW_PID_MAX_IOUT);
	/******信息中心实例建立******/
	CenterPointer()->PointerInit(&chassis.chassis_msg,CHASSISPUB);
#endif

#if (BOARD_NUM == TWO_BOARD && BOARD_PLACE == ON_CHASSIS) || BOARD_NUM == ONE_BOARD //单板或者双板且在底盘上时
	/*****登记超级电容实例*****/
//	RegisterSuperCap(ON_CAN1,0x211);
	
	//初始化底盘电机pid 旋转环pid
	chassis.chassis_angle_pid.Init(PID_POSITION,chassis_yaw_pid,CHASSIS_MOTOR_YAW_PID_MAX_OUT,CHASSIS_MOTOR_YAW_PID_MAX_IOUT);
	/*****电机初始化*****/
	chassis.chassis_motor[LF].DJIMotorInit(&chassis_config[LF]);
	chassis.chassis_motor[LB].DJIMotorInit(&chassis_config[LB]);
	chassis.chassis_motor[RB].DJIMotorInit(&chassis_config[RB]);
	chassis.chassis_motor[RF].DJIMotorInit(&chassis_config[RF]);
	for(int i = 0; i < 4; i++)
	{
		chassis_motor_info[i].chassis_motor_measure = chassis.chassis_motor[i].GetMotorMeasure();
	}
		
	/*****控制器初始化*****/
	chassis.chassis_motor[LF].controller.speed_PID.Init(PID_POSITION,motor_speed_pid,NULL,M3508_MOTOR_SPEED_PID_MAX_IOUT);
	chassis.chassis_motor[LB].controller.speed_PID.Init(PID_POSITION,motor_speed_pid,NULL,M3508_MOTOR_SPEED_PID_MAX_IOUT);
	chassis.chassis_motor[RB].controller.speed_PID.Init(PID_POSITION,motor_speed_pid,NULL,M3508_MOTOR_SPEED_PID_MAX_IOUT);
	chassis.chassis_motor[RF].controller.speed_PID.Init(PID_POSITION,motor_speed_pid,NULL,M3508_MOTOR_SPEED_PID_MAX_IOUT);
#endif
}

void chassis_t::Get_Information()
{
	if(chassis_order->if_open)
	{
		HAL_GPIO_WritePin(GPIOE,GPIO_PIN_6,GPIO_PIN_SET);
	}
	else
	{
		HAL_GPIO_WritePin(GPIOE,GPIO_PIN_6,GPIO_PIN_RESET);
	}
	// chassis_order = buff_order
	//底盘角度更新
    chassis_angle = *(chassis_INS_angle + INS_PITCH_ADDRESS_OFFSET);
	//处理云台发送的数据
    vx_set = 0.5f*chassis_msg.vx;
    vy_set = 0.5f*chassis_msg.vy;
	wz_set = 1.5f*chassis_msg.wz;
	//获取电机速度，换算成底盘速度
	for(uint8_t i=0;i<=3;i++)
	{
		chassis_motor_info[i].speed = CHASSIS_MOTOR_RPM_TO_VECTOR_SEN *chassis_motor_info[i].chassis_motor_measure->speed_rpm;
	}
	//获取底盘前进速度x，平移速度y，旋转速度wz，坐标系为右手系
	vx = ( chassis_motor_info[0].speed + chassis_motor_info[1].speed - chassis_motor_info[2].speed - chassis_motor_info[3].speed) * MOTOR_SPEED_TO_CHASSIS_SPEED_VX;
	vy = (-chassis_motor_info[0].speed + chassis_motor_info[1].speed - chassis_motor_info[2].speed + chassis_motor_info[3].speed) * MOTOR_SPEED_TO_CHASSIS_SPEED_VY;
	wz = (-chassis_motor_info[0].speed - chassis_motor_info[1].speed - chassis_motor_info[2].speed - chassis_motor_info[3].speed) * MOTOR_SPEED_TO_CHASSIS_SPEED_WZ / MOTOR_DISTANCE_TO_CENTER;

    if(chassis_msg.mode==ZERO_FORCE)
    {
    chassis_mode=ZERO_FORCE_MOVE;
    }
	else if(chassis_msg.mode==ANCILLARY)
    {
	chassis_mode=CHASSIS_NO_MOVE;
    }
    else
    {
    chassis_mode=NORMAL;
    }
}

/**
	* @brief          底盘主任务
  * @param[in]      NULL
  * @retval         NULL
  */
void ChassisTask()
{
	// Only delay once after startup; delaying every cycle makes control loop too slow and unstable.
	static uint8_t chassis_task_started = 0;
	if (chassis_task_started == 0)
	{
		vTaskDelay(CHASSIS_TASK_INIT_TIME);
		chassis_task_started = 1;
	}

#if BOARD_NUM == ONE_BOARD
	chassis.ChassisInfoUpdate(); //信息更新
	chassis.ControlSet(); //设定值
	chassis.ControlLoop(); //控制回环
#elif BOARD_NUM == TWO_BOARD
	#if BOARD_PLACE == ON_GIMBAL
	chassis.ControlSet();  //设定值
	chassis.SendMsgUpdate(); //发送信息
	#elif BOARD_PLACE == ON_CHASSIS
	chassis.ChassisInfoUpdate(); //信息更新
	chassis.ControlLoop();
	#endif
#endif
}

void ChassisInit(void)
{
	chassis.ChassisInit();
}



/**
	* @brief          控制量设置
  * @param[in]      NULL
  * @retval         NULL
  */
void chassis_t::ControlSet()
{
	switch(SysPointer()->mode)
	{
		case NORMAL:
		case AUTO:
		{
			fp32 yaw_follow_err;
			//平移设置
			RobotTranslationSet(); 
			//旋转设置
			yaw_follow_err = rad_format(GimbalPointer()->yaw_relative_angle);
			// Limit large-angle follow error to keep 180-degree transitions from exciting sway.
			yaw_follow_err = fp32_constrain(yaw_follow_err, -1.2f, 1.2f);
			if (fabs(yaw_follow_err) < 0.02f)
			{
				yaw_follow_err = 0.0f;
			}
			robot_wz_set = chassis_angle_pid.Calc(yaw_follow_err, 0.0f);
			// Keep yaw-follow command conservative to avoid high-speed lateral oscillation.
			robot_wz_set = fp32_constrain(robot_wz_set, -2.5f, 2.5f);
			if (fabs(robot_wz_set) < 0.05f)
			{
				robot_wz_set = 0.0f;
			}
			break;
		}
		case SPIN:
		case SPIN_AUTO:
			// SpinTranslationSet();
			static uint8_t index = 0;             //随机数序列索引
            static fp32 final_coefficient = 0.0f; //最终系数
            static fp32 coefficient = 0.0f;       //当前系数
            //平移设置
            RobotTranslationSet();
			//变速小陀螺
        if (SysPointer()->key_flag.spin_speed_change_flag)
        {
            if (xTaskGetTickCount() % SPIN_SPEED_CHANGE_TIME == 0)
            {
                // index在0-50之间循环
                index = (index + 1) % 50;
                final_coefficient = random_series[index] / 20.0f;
            }
            coefficient = RAMP_float(final_coefficient, coefficient, SPIN_SPEED_CHANGE_RAMP);

            robot_wz_set = (17.5f + coefficient) * (8 - fabs(SysPointer()->vx_set) - fabs(SysPointer()->vy_set)) / 5.0f;
        }
        //普通小陀螺
        else
        {
            robot_wz_set = 20.0f * (8 - fabs(SysPointer()->vx_set) - fabs(SysPointer()->vy_set)) / 5.0f;
        }

        //反向小陀螺
        if (SysPointer()->spin_reverse_flag)
        {
            robot_wz_set = -robot_wz_set;
        }
			break;
		case INIT_MODE:
		case ZERO_FORCE_MOVE:
		case DT7_MISSING:
			//三个速度设定值为零
			robot_vx_set = robot_vy_set = robot_wz_set = 0.0f;
		
		#if (BOARD_NUM == TWO_BOARD && BOARD_PLACE == ON_CHASSIS) || BOARD_NUM == ONE_BOARD //单板或者双板且在底盘上时
		for(int i =0;i<4;i++)
		{
			chassis_motor[i].MotorZeroForce();
		}
		#endif
			break;
			 case LOCK_WHEEL:
        for (uint8_t i = 0; i < 4; i++)
        {
		            chassis_motor[i].controller.send_current = chassis_motor[i].controller.speed_PID.Calc(speed[i], 0);
        }
        break;
		//相对角度模式
		case RELATIVE_ANGLE:
		case NO_FOLLOW_YAW:
			//平移设置
			NoFollowTranslationSet(); 
			//旋转设置
			robot_wz_set =  -CHASSIS_WZ_RC_SEN *	SysPointer()->add_yaw / YAW_RC_SEN;
			break;
		default:
			break;
	}
}

/**
	* @brief          平移运动设置
  * @param[in]      NULL
  * @retval         NULL
  */
void chassis_t::RobotTranslationSet()
{
	fp32 sin_yaw=0.0f,cos_yaw=0.0f;
	//旋转控制底盘速度方向，保证前进方向是云台方向，有利于运动平稳
	sin_yaw = arm_sin_f32(-GimbalPointer()->yaw_relative_angle);//-GimbalPointer()->yaw_relative_angle
	cos_yaw = arm_cos_f32(-GimbalPointer()->yaw_relative_angle);//改动
	//解算vx、vy速度
	robot_vx_set =  cos_yaw * SysPointer()->vx_set - sin_yaw * SysPointer()->vy_set;//+  +  -  +
	robot_vy_set = sin_yaw * SysPointer()->vx_set + cos_yaw * SysPointer()->vy_set;                        //改动
	//速度限幅
	robot_vx_set = fp32_constrain(robot_vx_set, -NORMAL_MAX_CHASSIS_SPEED_X, NORMAL_MAX_CHASSIS_SPEED_X);
	robot_vy_set = fp32_constrain(robot_vy_set, -NORMAL_MAX_CHASSIS_SPEED_Y, NORMAL_MAX_CHASSIS_SPEED_Y);
}

/**
	* @brief          不跟随时平移设置
  * @param[in]      NULL
  * @retval         NULL
  */
void chassis_t::NoFollowTranslationSet()
{
	robot_vx_set =  SysPointer()->vx_set;
	robot_vy_set = 	SysPointer()->vy_set;
	//速度限幅
	robot_vx_set = fp32_constrain(robot_vx_set, -NORMAL_MAX_CHASSIS_SPEED_X, NORMAL_MAX_CHASSIS_SPEED_X);
	robot_vy_set = fp32_constrain(robot_vy_set, -NORMAL_MAX_CHASSIS_SPEED_Y, NORMAL_MAX_CHASSIS_SPEED_Y);
}

/**
	* @brief          自旋行进设置(平移正常 + 固定角速度)
  * @param[in]      NULL
  * @retval         NULL
  */
// void chassis_t::SpinTranslationSet()
// {
// 	RobotTranslationSet();
// 	robot_wz_set = CHASSIS_SPIN_WZ_SET;
// }
//双板控制并且为此版为C板
#if BOARD_NUM == TWO_BOARD && BOARD_PLACE == ON_GIMBAL
/**
	* @brief          发送信息更新
  * @param[in]      NULL
  * @retval         NULL
  */
void chassis_t::SendMsgUpdate()
{
	chassis_msg.send_vx_set = robot_vx_set;
	chassis_msg.send_vy_set = robot_vy_set;
	chassis_msg.send_wz_set = robot_wz_set;
}

#elif BOARD_NUM == ONE_BOARD || (BOARD_NUM == TWO_BOARD && BOARD_PLACE == ON_CHASSIS)
/**
	* @brief          底盘信息更新
  * @param[in]      NULL
  * @retval         NULL
  */
void chassis_t::ChassisInfoUpdate()
{
	// 单板模式下使用系统模式驱动底盘状态，避免未更新的chassis_msg导致误判为无力。
	if (SysPointer()->mode == ZERO_FORCE_MOVE || SysPointer()->mode == DT7_MISSING || SysPointer()->mode == INIT_MODE)
	{
		chassis_mode = ZERO_FORCE_MOVE;
	}
	else
	{
		chassis_mode = NORMAL;
	}

	//获取电机速度，将电机的反馈的速度换算成底盘的速度
	for (uint8_t i = 0; i <= 3; i++)
		speed[i] = CHASSIS_MOTOR_RPM_TO_VECTOR_SEN *chassis_motor[i].GetRotorRpm();	   
	
	//获取底盘前进速度 x， 平移速度y，旋转速度wz，坐标系为右手系
	vx = (speed[LF] + speed[LB] - speed[RB] - speed[RF]) * MOTOR_SPEED_TO_CHASSIS_SPEED_VX;
	vy = (-speed[LF] + speed[LB]- speed[RB] + speed[RF]) * MOTOR_SPEED_TO_CHASSIS_SPEED_VY;
	wz = (-speed[LF] - speed[LB] - speed[RB] - speed[RF]) * MOTOR_SPEED_TO_CHASSIS_SPEED_WZ / MOTOR_DISTANCE_TO_CENTER;
	
	//裁判系统信息更新
	chassis_max_power    =  JUDGE_usGetChassisPowerLimit();
	chassis_real_power 	 =  JUDGE_fGetChassisPower();
	chassis_power_buffer =  JUDGE_fGetRemainEnergy();
	//田旭超电专用
	// cap_v_out = GetCapPointer()->tianxu_cap.Cap_voltage;
	// GetCapPointer()->EncodeControlData(chassis_max_power);
}

/**
	* @brief          底盘控制回环
  * @param[in]      NULL
  * @retval         NULL
  */
void chassis_t::ControlLoop()
{
	#if POWER_CONTROL_TYPE == OFF_SET  //关闭功控
	 
	#elif POWER_CONTROL_TYPE == ON_SET //加上功控
	PowerLimit();
	
	#endif

	if (chassis_mode == ZERO_FORCE_MOVE)
	{
		for(int i=0;i<4;i++)
		{
			chassis_motor_info[i].give_current=0;

		}
		return;
	}

	if(chassis_mode==CHASSIS_NO_MOVE)
	{
		vx_set=0;
		vy_set=0;
		wz_set=0;
	}

	//麦轮解算
	VectorToWheelSpeed(&wheel_speed[LF],&wheel_speed[LB],&wheel_speed[RB],&wheel_speed[RF],1);
	
for (int i = 0; i <=3; i++)
  {
		chassis_motor_info[i].speed_set=wheel_speed[i];
	}
	for (int i = 0; i <=3; i++)
  {
		chassis_motor_speed_pid[i].Calc( chassis_motor_info[i].speed,chassis_motor_info[i].speed_set);
  }    
	//赋值电流值
	for (int i = 0; i <= 3; i++)
	{
		chassis_motor_info[i].give_current = (int16_t)( chassis_motor_speed_pid[i].out);
	}

	//速度限幅
	SpeedLimit();
}

/**
	* @brief          速度解算
  * @param[in]      NULL
  * @retval         NULL
  */
void chassis_t::VectorToWheelSpeed(fp32 *lf_wheel_speed,fp32 *lb_wheel_speed,fp32 *rb_wheel_speed,fp32 *rf_wheel_speed,fp32 scale_k)
{
	#if CHASSIS_TYPE == All_Mecanum  //麦轮解算
	wheel_speed[LF] = (-scale_k*robot_vx_set + scale_k*robot_vy_set + (1.0f + CHASSIS_WZ_SET_SCALE) * MOTOR_DISTANCE_TO_CENTER * scale_k*robot_wz_set);
	wheel_speed[LB] = (-scale_k*robot_vx_set - scale_k*robot_vy_set + (1.0f + CHASSIS_WZ_SET_SCALE) * MOTOR_DISTANCE_TO_CENTER * scale_k*robot_wz_set);
	wheel_speed[RB] = (scale_k*robot_vx_set - scale_k*robot_vy_set + (1.0f + CHASSIS_WZ_SET_SCALE) * MOTOR_DISTANCE_TO_CENTER * scale_k*robot_wz_set);
	wheel_speed[RF] = (scale_k*robot_vx_set + scale_k*robot_vy_set + (1.0f + CHASSIS_WZ_SET_SCALE) * MOTOR_DISTANCE_TO_CENTER * scale_k*robot_wz_set);
	#elif CHASSIS_TYPE == All_Omnidirectional  //全向轮解算
	wheel_speed[LF] = (-scale_k*robot_vx_set + scale_k*robot_vy_set + (1.0f + CHASSIS_WZ_SET_SCALE) * MOTOR_DISTANCE_TO_CENTER * scale_k*robot_wz_set);
	wheel_speed[LB] = (-scale_k*robot_vx_set - scale_k*robot_vy_set + (1.0f + CHASSIS_WZ_SET_SCALE) * MOTOR_DISTANCE_TO_CENTER * scale_k*robot_wz_set);
	wheel_speed[RB] = (scale_k*robot_vx_set - scale_k*robot_vy_set + (1.0f + CHASSIS_WZ_SET_SCALE) * MOTOR_DISTANCE_TO_CENTER * scale_k*robot_wz_set);
	wheel_speed[RF] = (scale_k*robot_vx_set + scale_k*robot_vy_set + (1.0f + CHASSIS_WZ_SET_SCALE) * MOTOR_DISTANCE_TO_CENTER * scale_k*robot_wz_set);
	#endif
}

/**
	* @brief          速度限制
  * @param[in]      NULL
  * @retval         NULL
  */
void chassis_t::SpeedLimit()
{
	fp32 max_vector = 0.0f, vector_rate = 0.0f;
	fp32 temp = 0.0f;
	
	//计算轮子控制最大速度，并限制其最大速度i
	for (int i = 0; i <= 3; i++)
	{
		speed_set[i] = wheel_speed[i];
		temp = fabs(speed_set[i]);
		symbol[i] = speed_set[i]/temp;
		
		if (max_vector < temp)
		{
			max_vector = temp;
		}
	}

	//若最大速度大于设定值，则所有底盘电机速度等比例降低
	if (max_vector > MAX_WHEEL_SPEED)
	{
		vector_rate = MAX_WHEEL_SPEED / max_vector;
		for (int i = 0; i <= 3; i++)
		{
			speed_set[i] *= vector_rate;
		}
	}
		
	#if BOARD_TYPE == DJI_ABOARD  //A板上
	UphillPowerRedistribution();  //进行爬坡功率再分配
	#endif
		
	#if BOARD_NUM == ONE_BOARD || (BOARD_NUM == TWO_BOARD && BOARD_PLACE == ON_CHASSIS)
	for(int i =0;i<4;i++)  //进入控制器
	{
		chassis_motor[i].DJIMotorControl(&speed[i],&speed_set[i],NULL,OFF_SET);
	}
	#endif
}

/**
	* @brief          功率限制
  * @param[in]      NULL
  * @retval         NULL
  */
void chassis_t::PowerLimit()
{
	/******功率控制******/
	if((chassis_max_power>120))
    {
        chassis_max_power=120;  
    }else if(chassis_max_power==0)
    {
        chassis_max_power=80;
    }else
    {
        chassis_max_power = JUDGE_usGetChassisPowerLimit();
    }

    // fp32 a = cap_v_out;
    // fp32 b = 24.0f;

//    if(SysPointer()->key_flag.super_cap_flag)
//    {
//        if(GetCapPointer()->robofuture_cap.warning)
//			real_scale_k =0.6f;             
//        else if(SysPointer()->mode == SPIN || SysPointer()->mode == SPIN_AUTO)
//			real_scale_k = 1.5f;   //(40.0f+chassis_max_power)/150.0f;
//        else                
//			real_scale_k = (a/b)*(a/b)*(a/b)*((20.0f+chassis_max_power)/140.0f);
//    }else
//    {
//         if(SysPointer()->mode == SPIN || SysPointer()->mode == SPIN_AUTO)
//			real_scale_k = (40.0f+chassis_max_power)/165.0f;
//         else
//			real_scale_k = (chassis_power_buffer/60.0f)*(chassis_power_buffer/60.0f)*(chassis_power_buffer/60.0f)*(30.0f+chassis_max_power)/140.0f;
//    }
	// if(SysPointer()->key_flag.super_cap_flag)
    // {
    //     if(SysPointer()->mode == SPIN || SysPointer()->mode == SPIN_AUTO)
	// 		real_scale_k = (40.0f+chassis_max_power)/150.0f;
    //     else                
	// 		real_scale_k = (a/b)*(a/b)*(a/b)*((20.0f+chassis_max_power)/50.0f);
    // }else
    // {
    //      if(SysPointer()->mode == SPIN || SysPointer()->mode == SPIN_AUTO)
	// 		real_scale_k = (40.0f+chassis_max_power)/165.0f;
    //      else
	// 		real_scale_k = (chassis_power_buffer/60.0f)*(chassis_power_buffer/60.0f)*(chassis_power_buffer/60.0f)*(30.0f+chassis_max_power)/180.0f;
    // }
}

float Watch_Power_Max,Watch_Power,Watch_Buffer;
float Power,Power_Buffer;
uint16_t Power_Max;
float Klimit=1;
float Plimit=0;
float Chassis_pidout_max;
double Chassis_pid_out;
double Scaling1=0,Scaling2=0,Scaling3=0,Scaling4=0;

void chassis_t::new_chassis_power_limit()
{
	//读取状态
	Power_Buffer = (float)JUDGE_fGetRemainEnergy();
	Power = JUDGE_fGetChassisPower();
	Power_Max = JUDGE_usGetChassisPowerLimit();
	//纯软
//	Chassis_pidout_max=61536;
	//加超级电容
	Chassis_pidout_max = (Power_Max/Ina226_Chassis_Power->BusV)*10000;
	if(Power > Power_Max) 
	{
		fp32_constrain(chassis_motor_speed_pid[0].out,-4096,4096);
		fp32_constrain(chassis_motor_speed_pid[1].out,-4096,4096);
		fp32_constrain(chassis_motor_speed_pid[2].out,-4096,4096);
		fp32_constrain(chassis_motor_speed_pid[3].out,-4096,4096);		
	}else
	{
	//纯软
	Chassis_pid_out = fabs(chassis_motor_info[0].speed_set - chassis_motor_info[0].speed)+
	                  fabs(chassis_motor_info[1].speed_set - chassis_motor_info[1].speed)+
	                  fabs(chassis_motor_info[2].speed_set - chassis_motor_info[2].speed)+
	                  fabs(chassis_motor_info[3].speed_set - chassis_motor_info[3].speed);
	//加超级电容
//	Chassis_pid_out = Ina226_Chassis_Power->Current/100.0f+1;
		
	Scaling1 = (chassis_motor_info[0].speed_set - chassis_motor_info[0].speed)/Chassis_pid_out;
	Scaling2 = (chassis_motor_info[1].speed_set - chassis_motor_info[1].speed)/Chassis_pid_out;
	Scaling3 = (chassis_motor_info[2].speed_set - chassis_motor_info[2].speed)/Chassis_pid_out;
	Scaling4 = (chassis_motor_info[3].speed_set - chassis_motor_info[3].speed)/Chassis_pid_out;
	Klimit = Chassis_pid_out/16;
	fp32_constrain(Klimit,-1,1);
	
	if(Power_Buffer<50&&Power_Buffer>=40)	Plimit=0.9;//15
	else if(Power_Buffer<40&&Power_Buffer>=35)	Plimit=0.75;
	else if(Power_Buffer<35&&Power_Buffer>=30)	Plimit=0.5;
	else if(Power_Buffer<30&&Power_Buffer>=20)	Plimit=0.25;
	else if(Power_Buffer<20&&Power_Buffer>=10)	Plimit=0.125;
	else if(Power_Buffer<10&&Power_Buffer>=0)	Plimit=0.05;
	else if(Power_Buffer==60)					Plimit=1;
	
	chassis_motor_speed_pid[0].out = Scaling1*Chassis_pidout_max*Klimit*Plimit;
	chassis_motor_speed_pid[1].out = Scaling2*Chassis_pidout_max*Klimit*Plimit;
	chassis_motor_speed_pid[2].out = Scaling3*Chassis_pidout_max*Klimit*Plimit;
	chassis_motor_speed_pid[3].out = Scaling4*Chassis_pidout_max*Klimit*Plimit;
	
  }
}

/**
	* @brief          上坡功率再分配
  * @param[in]      NULL
  * @retval         NULL
  */
void chassis_t::UphillPowerRedistribution(fp32 *angle,uint8_t flag)
{
	fp32 all_pid_out;
	all_pid_out =fabs(speed_set[0])+
							 fabs(speed_set[1])+
							 fabs(speed_set[2])+
							 fabs(speed_set[3]);
	if(flag == 0)
	{
		if(*angle < -0.18f)  //上坡的角度
		{
			speed_set[1] = symbol[1]*all_pid_out/3;
			speed_set[2] = symbol[2]*all_pid_out/3;
			speed_set[0] = symbol[0]*all_pid_out/6;
			speed_set[3] = symbol[3]*all_pid_out/6;
		}
	}
}
#endif
