/**
 ******************************************************************************
 * @file    LKMotor.cpp
 * @author  Xushuang
 * @version V1.0.0 Xushuang 基本完成 2024/3/30
 * @date    2023/10/30
 * @brief	此处为瓴控电机类
 ******************************************************************************
 * @attention
 *			包含了电机数据读取的协议，集合了PID控制器和LADRC，可在初始化时自行选择电
 *	机类型等信息以及采取的控制器，此电机设备挂载在CAN线上，可根据两行代码，多加
 *  电机时无需再到CAN线接收回调函数中增添新的case，其设备信息会在CAN线自动登记，
 *  仅调用DJIMotorInit(**)函数后就可正常读取电机数据和控制
 ******************************************************************************
 */
#include "LKMotor.h"

#define get_lk9025_motor_measure(ptr, data)							\
{																	\
	(ptr)->last_ecd = (ptr)->ecd;									\
    (ptr)->ecd = (uint16_t)((data)[7] << 8 | (data)[6]);			\
    (ptr)->speed_rpm = (int16_t)((data)[5] << 8 | (data)[4]);	    \
    (ptr)->given_current = (int16_t)((data)[3] << 8 | (data)[2]);	\
    (ptr)->temperate = (data)[1];								    \
}

		
#define motor_measure_MF9025_PI(ptr,data)							\
		{															\
			(ptr)->anglePidKp = (data)[2];							\
			(ptr)->anglePidKi = (data)[3];							\
			(ptr)->speedPidKp = (data)[4];							\
			(ptr)->speedPidKi = (data)[5];							\
			(ptr)->iqPidKp		= (data)[6];						\
			(ptr)->iqPidKi		= (data)[7];						\
		}																															

#define motor_measure_MF9025_error1(ptr,data)							\
		{																\
			(ptr)->temperature = (data)[1];						        \
			(ptr)->voltage = (uint16_t)((data)[3] << 8 | (data)[2])	;	\
			(ptr)->error_flag = (data)[7];								\
		}

//瓴控电机指针
LKMotorInstance *LKMotorInstances[TOTAL_LK_MOTOR_SUM];		
		
/**
  * @brief          瓴控电机数据解码
  * @param[in]      *rx_instance：接收的实例
  * @retval         Null
  */
static void DecodeLK9025Motor(CAN_Rx_Instance_t *rx_instance)
{
	uint8_t *rxbuff = rx_instance->rx_buff;
	//address是void*需强制转化
	LKMotorInstance *motor = (LKMotorInstance *)rx_instance->module_address;
	LK_Motor_Measure_t *measure = &motor->motor_measure;
	get_lk9025_motor_measure(measure,rxbuff);
}

//构造函数
LKMotorInstance::LKMotorInstance()
{
	
}

/**
  * @brief          瓴控电机初始化函数
  * @param[in]     	config：传入的初始化参数
  * @retval         NULL
  */
void LKMotorInstance::LKMotorInit(LK_Motor_Setting_t *config)
{
	uint32_t rx_id;
	if(config == NULL)
	{
		return;
	}
	motor_settings = config;
	LKMotorInstances[motor_settings->set_id - 1] = this;
	rx_id = 0x140 + motor_settings->set_id;
	//注册对应的id
	CANRxInitSet(&motor_settings->motor_can,motor_settings->can_id,rx_id,this,DecodeLK9025Motor);
	CANRxRegister(&motor_settings->motor_can);
	switch(motor_settings->control_type)
	{
		case LADRC_FDW_CONTROL:
			controller.ladrc_fdw.MaxOutInit(1500);
			break;
		case LADRC_CONTROL:
			controller.ladrc.MaxOutInit(1500);
			break;
		case CASCADE_LOOP:
			controller.angle_PID.MaxOutInit(1500);
			controller.speed_PID.MaxOutInit(1500);
			break;
		case SINGLE_LOOP:
			controller.speed_PID.MaxOutInit(1500);
			break;
		case OPEN_LOOP:
			break;
	}
	motor_watch.InitDeviceStatus(1000);
}

/**
  * @brief          获得瓴控电机对应ID的地址
  * @param[in]     	target_id：需要获取信息的电机ID
  * @retval         NULL
  */
LKMotorInstance *GetLKPointer(uint8_t target_id)
{
	return LKMotorInstances[target_id-1];
}

/**
  * @brief          瓴控电机控制函数
  * @param[in]     	ref：反馈值
  * @param[in]     	set：设定值
  * @param[in]     	motor_gyro：反馈的角速度
  * @param[in]     	filter_flag：滤波标志位
  * @retval         NULL
  */
void LKMotorInstance::LKMotorControl(fp32 *ref,fp32 *set,fp32 *motor_gyro,uint8_t filter_flag)
{
	fp32 output;
	motor_watch.RecordStartTime(); //开始计时
	//在这赋值仅为方便调试观察
	controller.target_value = *set * motor_settings->direction;  //此处负号时仍在测试中
	controller.now_value = *ref;
	switch(motor_settings->control_type)
	{
		case LADRC_FDW_CONTROL:
			//错误处理
			if(motor_gyro == NULL)
			{
				motor_watch.error_code = INPUT_PARAM_ERROR;
			}else
			{
				output=controller.ladrc_fdw.FDW_Calc(controller.now_value,controller.target_value,*motor_gyro);
			}
			break;
		case LADRC_CONTROL:
			//错误处理
			if(motor_gyro == NULL)
			{
				motor_watch.error_code = INPUT_PARAM_ERROR;
			}else
			{
				output=controller.ladrc.Calc(controller.now_value,controller.target_value,*motor_gyro);
			}
			break;
		case SINGLE_LOOP:
		case CASCADE_LOOP:
			output=PIDControl(ref,set,filter_flag);
			break;
		case OPEN_LOOP:
			break;
	}
	controller.send_current = (int16_t)output;
	motor_watch.CalcExcutePeriod(); //结束计时
}

/**
  * @brief          瓴控电机的PID控制函数
  * @param[in]     	ref：反馈值
  * @param[in]     	set：设定值
  * @param[in]     	filter_flag：滤波标志位
  * @retval         fp32
  */
fp32 LKMotorInstance::PIDControl(fp32 *ref,fp32 *set,uint8_t filter_flag)
{
	fp32 angle_out,speed_set,pid_out;
	//速度单环时直接赋值设定值地址
	speed_set = *set * motor_settings->direction;
	//角度环
	if(motor_settings->control_type == CASCADE_LOOP )
	{
		angle_out	=	controller.angle_PID.Calc(*ref,*set);
		speed_set = angle_out;
	}
	//单环时直接进入
	//速度反馈是否需要滤波
	if(filter_flag)
	{
		pid_out	=	controller.speed_PID.Calc(controller.pid_speed,speed_set);
	}else
	{
		if(ref != NULL)
			pid_out	=	controller.speed_PID.Calc(*ref,speed_set);
		else
			pid_out	=	controller.speed_PID.Calc(motor_measure.speed_rpm,speed_set);
	}
	return pid_out;
}

/**
  * @brief          速度滤波函数
  * @param[in]     	fliter_num：滤波的系数
  * @retval         fp32
  */
fp32 LKMotorInstance::FilterSpeed(fp32 *fliter_num)
{
	if(fliter_num == NULL)
	{
		return false;
	}else
	{
		//二阶低通滤波
		motor_measure.speed_fliter_1 = motor_measure.speed_fliter_2;
		motor_measure.speed_fliter_2 = motor_measure.speed_fliter_3;
		motor_measure.speed_fliter_3 = motor_measure.speed_fliter_2 * fliter_num[0] + motor_measure.speed_fliter_1 * fliter_num[1] + (motor_measure.speed_rpm * RPM_TO_RAD_S) * fliter_num[2];
		controller.pid_speed = motor_measure.speed_fliter_3;
		return motor_measure.speed_fliter_3;
	}
}

/**
  * @brief          瓴控电机计算角度函数
  * @param[in]     	offset_ecd：中值ECD
  * @param[in]     	drive_radio：驱动的减速比
  * @retval         fp32
  */
fp32 LKMotorInstance::LKMotorEcdToAngle(uint16_t offset_ecd,int8_t drive_radio)
{
	int32_t relative_ecd;
	relative_ecd = motor_measure.ecd - offset_ecd;

    if(relative_ecd > HALF_LK_ECD_RANGE * drive_radio)
    {
	  relative_ecd -= LK_ECD_RANGE * drive_radio;
    }else if (relative_ecd < -HALF_LK_ECD_RANGE * drive_radio)
    {
	  relative_ecd += LK_ECD_RANGE * drive_radio;
    }
	
    return relative_ecd * LK_MOTOR_ECD_TO_RAD / drive_radio;
}

/**
  * @brief          编码关机指令
  * @retval         NULL
  */
void LKMotorInstance::EncodeOffData()
{
	send_data[0] = 0x80;
	for(int i = 1; i <= 7; i++)
	{
		send_data[i] = 0;
	}
}

/**
  * @brief          编码运行指令
  * @retval         NULL
  */
void LKMotorInstance::EncodeRunData()
{
	send_data[0] = 0x88;
	for(int i = 1; i <= 7; i++)
	{
		send_data[i] = 0;
	}
}

/**
  * @brief          编码停止运行指令
  * @retval         NULL
  */
void LKMotorInstance::EncodeStopData()
{
	send_data[0] = 0x81;
	for(int i = 1; i <= 7; i++)
	{
		send_data[i] = 0;
	}
}

/**
  * @brief          编码转矩电流控制指令
  * @param[in]     	iqcontrol：发送的转矩电流
  * @retval         NULL
  */
void LKMotorInstance::EncodeTorqueControlData(int16_t iqcontrol)
{
	send_data[0] = 0xA1;
	send_data[1] = send_data[2] = send_data[3] = 0;
	send_data[6] = send_data[7] = 0;
	send_data[4] = iqcontrol;
	send_data[5] = iqcontrol >> 8;
}

/**
  * @brief          编码速度控制指令（暂时还不好使）
  * @param[in]     	speed_control：发送的旋转速度指令
  * @retval         NULL
  */
void LKMotorInstance::EncodeSpeedControlData(int32_t speed_control)
{
	send_data[0] = 0xA2;
	send_data[1] = send_data[2] = send_data[3] = 0;
	send_data[4] = speed_control;
	send_data[5] = speed_control >> 8;
	send_data[6] = speed_control >> 16;
	send_data[7] = speed_control >> 24;
}
