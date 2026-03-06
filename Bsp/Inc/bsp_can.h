#ifndef __BSP_CAN_H
#define __BSP_CAN_H

#include "can.h"
#include "struct_typedef.h"
#include "stm32f4xx_hal.h"
#include "stm32f4xx_hal_can.h"
#include "main.h"
#include <stdint.h>

#if !defined(STM32F4xx_HAL_CAN_H)
/* 如果 HAL CAN 头文件对编辑器不可见，提供受保护的前向声明以消除未定义标识符警告 */
typedef struct __CAN_HandleTypeDef CAN_HandleTypeDef;
#endif
#define MAX_CAN_NUM 2
#define MAX_REGISTER_NUM 8
#ifdef __cplusplus
extern "C"{
#endif

#ifdef __cplusplus
	//底盘控制指令联合体
class Dji_Motor_Measure_t;

typedef union 
{
    uint8_t s[4];
    fp32 f;
} Chassis_Order;

typedef struct
{
uint8_t if_shoot;
}chassis_to_gimbal_t;

//底盘控制指令结构体
typedef struct
{
  Chassis_Order vx_set;          
  Chassis_Order vy_set;                 
  Chassis_Order wz_set;
  uint8_t if_open;
  uint8_t mode;
  uint8_t id;  
} chassis_order_measure_t;

/****不同电机挂载在不同CAN线上时需修改*****/
enum CAN1_msg_ID
{
	CAN1_3508_LF_ID = 0x201,
	CAN1_3508_LB_ID = 0x202,
	CAN1_3508_RB_ID = 0x203,
 	CAN1_3508_RF_ID = 0x204,
};

enum CAN2_msg_ID
{
	CAN2_Lifting_Motor_ID = 0x201,
	CAN2_Front_Motor_ID = 0x202,
	CAN2_GOLD_Motor_ID = 0x203,
	CAN2_Side_Motor_ID = 0x204,
	CAN2_Ore_Motor_ID = 0x205,
};

//可在此处加入所需电机编号
enum motor_ID
{
	LF,  //=0
	LB,  //=1
	RB,  //=2
	RF,  //=3 
//	LIFTING, //=4
//	GOLD,//5
//	FRONT,  //=6
//	SIDE,  //=7
//	ORE,  //=8
	UPMOTOR_L,
	UPMOTOR_R,
	FRONTMOTOR_L,
	FRONTMOTOR_R,
};

extern void CAN1_200_cmd_motor(int16_t can1_motor1, int16_t can1_motor2, int16_t can1_motor3, int16_t can1_motor4);
extern void CAN1_1FF_cmd_motor(int16_t can1_motor5, int16_t can1_motor6, int16_t can1_motor7, int16_t can1_motor8);
extern void CAN2_200_cmd_motor(int16_t can2_motor1, int16_t can2_motor2, int16_t can2_motor3, int16_t can2_motor4);
extern void CAN2_1FF_cmd_motor(int16_t can2_motor5, int16_t can2_motor6, int16_t can2_motor7, int16_t can2_motor8);
extern void CAN_cmd_chassis_reset_ID(void);
extern void CAN1_1FF_cmd_motor_send_wz_info(int16_t can1_motor5, int16_t can1_motor6, fp32 info);
extern void CAN1_301_send_vxvy_info(fp32 vx_set, fp32 vy_set);
void CAN_CMD_CAP(uint16_t motor1);
void CAN_CMD_ROBO_CAP(uint8_t dischargecmd,uint8_t ctrlmode,uint16_t chassis_power_buff,uint16_t chassis_power_buff_limit,uint16_t chassis_power_limit);
extern const chassis_to_gimbal_t  *get_if_revolver_point(void);
extern void CAN1_send_state_info(uint8_t reset,uint8_t Auto, uint8_t Spin ,uint8_t IfShift ,uint8_t OBLI);
extern const chassis_order_measure_t *get_Gimbal_Oreder();
extern bool_t if_speed_update;
extern fp32 speed_offest;

#endif
//CAN线种类名称
typedef enum 
{
	ON_CAN1, ON_CAN2,
}CAN_NAME_e;

//CAN线接收类结构体
typedef struct _
{
	CAN_NAME_e can_id;
	uint32_t rx_id;
	uint8_t rx_buff[8];     // 接收缓存,最大消息长度为8
	void (*CANMsgCallback)(struct _ *); // 增加额外参数
	void *module_address;
}CAN_Rx_Instance_t;



typedef union
{
		uint8_t s[4];
    fp32 f;
}coordinate_system;

typedef union 
{
    uint8_t s[4];
    fp32 f;
} Yaw_Pitch;

typedef union 
{
    uint8_t s[4];
    fp32 f;
}Shoot;

typedef union
{
	uint8_t s[4];
	fp32 f;
}rate_t;

typedef union
{
	uint8_t s[2];
	uint16_t u;
}speed_set_t;

//底盘接收的指令
typedef struct
{
	Yaw_Pitch yaw_angle;
    Yaw_Pitch pitch_angle;
	coordinate_system x_coordinate;
	coordinate_system y_coordinate;
	coordinate_system pre_x_coordinate;
	coordinate_system pre_y_coordinate;
	rate_t radius;//fir_speed;
	uint8_t reset;
	uint8_t IfFirc;
	uint8_t IfShift;
	uint8_t Auto;                 
    uint8_t Spin;  
	speed_set_t speed_set;
	uint8_t vision_mode;
} UI_order_to_chassis;

typedef struct
{   
	fp32 Cap_input_vol;
	fp32 Cap_voltage;
	fp32 Cap_current;
	uint16_t Cap_power;
} Chassis_Power_t;

typedef struct
{   
	uint8_t warning;
	uint8_t DischargeStop;
	uint16_t chassis_power;
	uint16_t ele_quantity;
} robofuture_cap_measure_t;

//CAN线接收的初始化函数
typedef void ModuleCallback(CAN_Rx_Instance_t *rx_instance);
void CANRxInitSet(CAN_Rx_Instance_t *input_init,CAN_NAME_e can_num,uint32_t module_rx_id,void *module_address,ModuleCallback *DecodeCallback);
CAN_Rx_Instance_t *CANRxRegister(CAN_Rx_Instance_t *input_rx);

//CAN线发送的结构体
typedef struct
{
	CAN_HandleTypeDef *can_handle;
	CAN_TxHeaderTypeDef tx_message_data;
	int16_t *set_current[4];
}CANTxInstance_t;

void CANTxRegister(CANTxInstance_t *input_tx,CAN_Rx_Instance_t *input_rx);
void CANFilterInit(void);

// BSP兼容别名与底层CAN操作函数（从 达妙电机例程 bsp_can.h 整合）
typedef CAN_HandleTypeDef hcan_t;

void bsp_can_init(void);
void can_filter_init(void);
uint8_t canx_send_data(hcan_t *hcan, uint16_t id, uint8_t *data, uint32_t len);
uint8_t canx_receive(hcan_t *hcan, uint16_t *recid, uint8_t *buf);
void can1_rx_callback(void);
void can2_rx_callback(void);

extern void (*CANMonitor)(uint8_t type);

//发送函数
void CANSendToMotor(CANTxInstance_t *tx_message,int16_t motor1,int16_t motor2,int16_t motor3,int16_t motor4);
void CANSend16Message(CANTxInstance_t *tx_message,uint16_t message1,uint16_t message2,uint16_t message3,uint16_t message4);
void CANSendFpMessage(CANTxInstance_t *tx_message,fp32 *message1,fp32 *message2);
void CANSendU8Message(CANTxInstance_t *tx_message,uint8_t message[8]);

extern UI_order_to_chassis UI_Measure;
extern const UI_order_to_chassis *get_UI_measure(void);
extern const Chassis_Power_t *get_cap_measure_point(void);
extern const robofuture_cap_measure_t *get_robo_cap_measure_point(void);

#ifdef __cplusplus
}
#endif

#endif
