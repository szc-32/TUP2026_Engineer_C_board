#ifndef __LKMOTOR_H
#define __LKMOTOR_H

#include "struct_typedef.h"
#include "motor_def.h"
//#include "buzzer.h"
//#include "device_monitor.h"
#include "bsp_can.h"
#include "debug.h"
//电机编码值最大以及中值
#define HALF_LK_ECD_RANGE  32768
#define LK_ECD_RANGE       65535
//电机编码值转化成角度值
#define LK_MOTOR_ECD_TO_RAD 0.0000958737992428526f   // 2*  PI  /65536
#define RPM_TO_RAD_S 0.10471976f  //r/m -> rad/s

#ifdef __cplusplus
extern "C"{
#endif

#ifdef __cplusplus
	
////电机监测类型
//enum Motor_Monitor_Type_e
//{
//	MOTOR_PARAM = 0,
//	MOTOR_TEM = 1,
//};

//大疆电机信息包
typedef struct
{
	uint16_t ecd;
	int16_t speed_rpm;
	int16_t given_current;
	uint8_t temperate;
    
	uint16_t last_ecd;
	int64_t difference_num; //差值计算的旋转圈数
	
	//用于滤波
	fp32 speed_fliter_1;
	fp32 speed_fliter_2;
    fp32 speed_fliter_3;
}LK_Motor_Measure_t;	

//电机设定参数
typedef struct
{
	const char *motor_name; //电机名称
	CAN_NAME_e can_id;      //CAN线ID
	uint8_t set_id;		    //电机的ID号
	Motor_Rotate_Direction_e direction; //电机旋转方向
	Control_Type_e control_type;        //控制器选择类型
	//通用数据
	CAN_Rx_Instance_t motor_can;        //电机CAN实例
}LK_Motor_Setting_t;

//为每个注册电机加字符标注
class LKMotorInstance
{
	private:
		Motor_Watch_t motor_watch;	        //电机观测器
		LK_Motor_Setting_t *motor_settings;    //电机设置参数
		LK_Motor_Measure_t motor_measure;	//电机数据
	public:
		Motor_Controller_s controller;		  //控制器
		uint8_t	send_data[8];
		
		LKMotorInstance();
		void LKMotorInit(LK_Motor_Setting_t *config);
		void LKMotorControl(fp32 *ref,fp32 *set,fp32 *motor_gyro,uint8_t filter_flag);
		fp32 PIDControl(fp32 *ref,fp32 *set,uint8_t filter_flag);
		fp32 FilterSpeed(fp32 *fliter_num);
		fp32 LKMotorEcdToAngle(uint16_t offset_ecd,int8_t drive_radio);
	
		void EncodeOffData();
		void EncodeRunData();
		void EncodeStopData();
		void EncodeTorqueControlData(int16_t iqcontrol);
		void EncodeSpeedControlData(int32_t speed_control);
		/**友元函数进行信息解码**/
		friend void DecodeLK9025Motor(CAN_Rx_Instance_t *rx_instance);
};

LKMotorInstance *GetLKPointer(uint8_t target_id);
#endif
	
#ifdef __cplusplus
}
#endif

#endif
