#ifndef __GIMBAL_H
#define __GIMBAL_H

#include "motor.h"
#include "DMmotor.h"
#include "LKMotor.h"
#include "message_center.h"
#include "robot_def.h"
#include "struct_typedef.h"
#include "vision.h"
#include "led.h"
#include "debug.h"
#include "imu.h"

//云台初始化回中值，允许的误差,并且在误差范围内停止一段时间以及最大时间6s后解除初始化状态，
#define GIMBAL_INIT_ANGLE_ERROR     0.12f
#define GIMBAL_INIT_STOP_TIME       150
#define GIMBAL_INIT_TIME            6000
#define GIMBAL_CALI_REDUNDANT_ANGLE 0.1f
//云台初始化回中值的速度以及控制到的角度
#define GIMBAL_INIT_PITCH_SPEED     0.003f
#define GIMBAL_INIT_YAW_SPEED       0.0003f
#define GIMBAL_INIT_YAW_FIND_SPEED  0.0012f

// 光电门找门策略（用于 yaw 初始化）
#define GIMBAL_INIT_YAW_FIND_DIR         1.0f
#define GIMBAL_INIT_YAW_BIDIR_SEARCH     1
#define GIMBAL_TASK_PERIOD_MS            1U
#define GIMBAL_INIT_YAW_REVERSE_MS       1000U
#define GIMBAL_INIT_YAW_REVERSE_TICKS    ((GIMBAL_INIT_YAW_REVERSE_MS + GIMBAL_TASK_PERIOD_MS - 1U) / GIMBAL_TASK_PERIOD_MS)

// 双方案开关：上电初始化可不依赖光电门；小陀螺结束后可强制走光电门回中
#define GIMBAL_POWERON_USE_PHOTOGATE          0
#define GIMBAL_SPIN_EXIT_REHOME_USE_PHOTOGATE 0
// 上电不找光电门时，使用达妙绝对编码角作为回中参考（按机械安装标定）
#define GIMBAL_POWERON_ENCODER_CENTER_RAD    -2.3f

// 上电回中标定调试打印
#define GIMBAL_CENTER_DEBUG_PRINT_ENABLE      1
#define GIMBAL_CENTER_DEBUG_PRINT_PERIOD_MS   100U
#define GIMBAL_CENTER_DEBUG_UART_PORT         1
//初始角度值设定
#define INIT_YAW_SET    -1.8f
#define INIT_PITCH_SET  0.0f
// Yaw sign convention for both control input and chassis follow compensation.
// Set to -1.0f if installed direction is opposite.
#define GIMBAL_YAW_SIGN  1.0f
//控制器初始化参数
#define INIT_PARAM 0
#define NORMAL_PARAM 1
#define AUTO_PARAM 2

//云台正常控制参数
#define YAW_NORMAL_WC 23
#define YAW_NORMAL_B0 0.007f
#define YAW_NORMAL_WO 90
#define YAW_NORMAL_W  25
#define YAW_NORMAL_GAIN 1

#define PIT_NORMAL_WC 30
#define PIT_NORMAL_B0 0.008f
#define PIT_NORMAL_WO 120
#define PIT_NORMAL_W  20
#define PIT_NORMAL_GAIN 1

//云台初始化参数
#define YAW_INIT_WC 25
#define YAW_INIT_B0 0.0058f
#define YAW_INIT_WO 100

#define PIT_INIT_WC 15
#define PIT_INIT_B0 0.0055f
#define PIT_INIT_WO 60

//云台自瞄参数
#define YAW_AUTO_WC 16
#define YAW_AUTO_B0 0.0065f
#define YAW_AUTO_WO 80
#define YAW_AUTO_W  25
#define YAW_AUTO_GAIN 0.2f

#define PIT_AUTO_WC 17
#define PIT_AUTO_B0 0.0065f
#define PIT_AUTO_WO 85
#define PIT_AUTO_W  17
#define PIT_AUTO_GAIN 0.3f
#ifdef __cplusplus
extern "C"{
#endif
	
#ifdef __cplusplus

class gimbal_t
{
	private:
		/*****陀螺仪信息*****/
		fp32 roll_gyro,pitch_gyro,yaw_gyro;						//三轴角速度——陀螺仪获取
		fp32 yaw_absolute_set_rad,pit_absolute_set_rad; 		//绝对角度设定值（rad）
		fp32 yaw_absolute_rad,pit_absolute_rad;					//绝对角度值（rad）——陀螺仪获取
		/*****控制量信息*****/
		fp32 add_yaw,add_pit;									//控制量
	
		/*****相对角度设定值信息（不使用陀螺仪时使用），若需配合LADRC控制器，角速度信息应从电机类的API中获取*****/
		fp32 yaw_relative_set,pit_relative_set;  				//YAW轴、PIT轴相对角度设定值
		/*****其他信息*****/
		uint16_t yaw_ecd_angle;                  				//YAW轴累积编码值（因为同步带1：2所以需要，否则可以删）
	
		/*****视觉角度获取处理方式选择*****/
#if Auto_Type == HANDLE_LPF
		lpf_type_def yaw_vision_lpf,pitch_vision_lpf;			//一阶低通滤波结构体
#elif Auto_Type == HANDLE_KALMAN
			
#endif
	public:
		gimbal_t();
		/*****云台测试单元*****/
		Test_Module_t gimbal_test;
		/*****YAW轴、PIT轴电机实例*****/
		DMMotorInstance yaw_motor;
		DJIMotorInstance height_motor;

		/*****信息实例*****/
		Gimbal_Pub_Msg_t gimbal_msg;
		
		/*****信息获取函数*****/
		void BasicInfoUpdate();
		void OperationInfoUpdate();
		void AutoInfoUpdate();
		/*****初始化函数*****/
		void GimbalControllerInit(uint8_t type);
		void InitInfoUpdate();
		void JudgeInitState();
		/*****控制函数*****/
		void NormalControl();
		void RelativeControl();
		void ZeroForceControl();
		fp32 yaw_absolute_start_rad;

	private:
		LADRC_FDW_t yaw_ladrc_fdw;
		

};	
//LKMotorInstance *GetLKPointer();
#endif
void GimbalInit(void);
void GimbalTask(void);
#ifdef __cplusplus
}
#endif

#endif
