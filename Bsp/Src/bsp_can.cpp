/**
 ******************************************************************************
 * @file    bsp_can.cpp
 * @author  Xushuang
 * @version V1.0.0 Xushuang 基本完成 2023/8/25
 *          V2.0.0 Xushuang 重构接收发送 2023/10/28
 * @date    2023/10/28
 * @brief		CAN线支持包，过滤器设置，CAN发送函数，CAN接收回调函数
 *					CAN接收回调函数定义不用在此处修改，正常此文件禁止修改
 ******************************************************************************
 * @attention
 *	在添加新的需挂载在CAN线上的设备时，仅需要两步即可实现，无需在接收中添加新的case。
 *	正常需扩展增加设备挂载时，仅需在该设备的类中进行如下操作即可！！！
 *
 *	第一步：在需要的结构体或类中加入CAN_Rx_Instance_t*类和CAN_Rx_Init_t类。
 * 	第二步：调用CANRxInitSet()函数对CAN_Rx_Init_t类进行初始化。
 *	第三步：调用CANRxRegister()函数进行注册登记和初始化设置。
 *
 *	特别注意：需定义一个static void Decode***(CAN_Rx_Instance_t *rx_instance)类型的
 *	信息解码函数，并在CANRxInitSet()中作为形参传入
 *
 *	例：在motor类中，需将电机注册到CAN线中
 *	1.class DJIMotorInstance
 *	{
 *		public:
 *			CAN_Rx_Instance_t *motor_can;				//电机CAN实例指针
 *			CAN_Rx_Init_t motor_can_init;				//电机CAN实例初始化
 *	}
 *	2.void DJIMotorInstance::DJIMotorInit()
 *	{
 *		CANRxInitSet(&motor_can_init,can_num,motor_rx_id,motor_address,DecodeDJIMotor);
 *		motor_can = CANRxRegister(&motor_can_init);
 *	}
 ******************************************************************************
 */
 //CAN新信息接收或者通信的说明未写
#include "bsp_can.h"
#include "string.h"
#include "stm32f4xx_hal_can.h"
#include "stm32f4xx.h"
#include "struct_typedef.h"
#include "main.h"
#include "cmsis_os.h"


//CAN总线接收指针数组，用于存储接收数据
CAN_Rx_Instance_t *rx_instance[MAX_CAN_NUM][MAX_REGISTER_NUM] = {NULL};
//记录CAN1、CAN2上的挂在设备个数
uint8_t can1_register_id,can2_register_id;

extern CAN_HandleTypeDef hcan1;
extern CAN_HandleTypeDef hcan2;

/********************通用***************************/
//电机数据读取
#define get_motor_measure(ptr, data)                                    \
    {                                                                   \
        (ptr)->last_ecd = (ptr)->ecd;                                   \
        (ptr)->ecd = (uint16_t)((data)[0] << 8 | (data)[1]);            \
        (ptr)->speed_rpm = (uint16_t)((data)[2] << 8 | (data)[3]);      \
        (ptr)->given_current = (uint16_t)((data)[4] << 8 | (data)[5]);  \
        (ptr)->temperate = (data)[6];                                   \
			if (abs((ptr)->ecd - (ptr)->last_ecd) > 4096)											\
			{																																	\
				if ((ptr)->ecd  >= 7500 && (ptr)->last_ecd <= 500)							\
				{																																\
						(ptr)->num += 1;																						\
				}																																\
				else	if((ptr)->ecd  <= 500 && (ptr)->last_ecd >= 7500)					\
				{																																\
						(ptr)->num -= 1;																						\
				}																															  \
			}																																	\
    }				
		
//获取超级电容数据
#define	get_cap_measure(ptr, data)                                      \
{   uint16_t *buf =    (uint16_t*) data;                                \
		(ptr)->Cap_input_vol = ((fp32)buf[0]/100.f);                        \
		(ptr)->Cap_voltage = ((fp32)buf[1]/100.f);                          \
		(ptr)->Cap_current = ((fp32)buf[2]/100.f);                          \
		(ptr)->Cap_power = ((fp32)buf[3]/100.f);                            \
}

/**********************双板时（底盘）使用********************************/		
//获取UI数据		
#define get_ui_measure(ptr, data)                      		 		 			  \
{    int16_t i = 0;          																					\
	   for(i=0;i<2;i++)                                                 \
	   {																																\
			 (ptr)->speed_set.s[i] = data[i];															  \
		 }																																\
		(ptr)->reset = data[2];																						\
	  (ptr)->IfFirc = data[3];																					\
		(ptr)->IfShift = data[4];																					\
		(ptr)->Auto = data[5];																						\
		(ptr)->Spin = data[6];																						\
}
/**********************双板时（底盘）使用********************************/		
//获取vx_set，vy_Set
#define get_gimabl_measure(ptr, data)                      		 	      \
{   int16_t i =0;                                                     \
		for(i=0;i<4;i++)																								 	\
		{																																	\
			(ptr)->vx_set.s[i]= data[i];														        \
			(ptr)->vy_set.s[i]= data[i+4];																	\
		}																																	\
}

/**********************双板时（底盘）使用********************************/		
//获取wz_set
#define get_wz_measure(ptr, data)                      		 	 		      \
{   int16_t i =0;                                                     \
		for(i=0;i<4;i++)																								 	\
		{																																	\
			(ptr)->wz_set.s[i]= data[i+4];			                            \
		}		                                                              \
		(ptr)->if_open=data[0];		                                        \
		(ptr)->mode=data[1]	;                                             \
		(ptr)->id=data[2];	                                              \
}
//				for(i=0;i<4;i++)																								 	\
//				{																																	\
//				 (ptr)->pitch.s[i]= data[i];																			\
//				}			

/**********************双板时（底盘）使用********************************/	
//获取yaw,pitch角度值
#define get_angle_info(ptr, data)																	\
		{		int16_t i =0;																											\
			  for(i=0;i<4;i++)																								 	\
				{																																	\
				 (ptr)->yaw_angle.s[i]= data[i];																	\
				 (ptr)->pitch_angle.s[i]= data[i+4];															\
				}																																	\
		}
/**********************双板时（底盘）使用********************************/		
//获取x,y坐标		
#define get_x_y_info(ptr, data)																	\
		{		int16_t i =0;																											\
			  for(i=0;i<4;i++)																								 	\
				{																																	\
				 (ptr)->x_coordinate.s[i]= data[i];																	\
				 (ptr)->y_coordinate.s[i]= data[i+4];															\
				}																																	\
		}
/**********************双板时（底盘）使用********************************/			
//获取预测x,y坐标		
		#define get_pre_x_y_info(ptr, data)																	\
		{		int16_t i =0;																											\
			  for(i=0;i<4;i++)																								 	\
				{																																	\
				 (ptr)->pre_x_coordinate.s[i]= data[i];																	\
				 (ptr)->pre_y_coordinate.s[i]= data[i+4];															\
				}																																	\
		}
		
/**********************双板时（底盘）使用********************************/			
//获取预测x,y坐标		
		#define get_r_info(ptr, data)																	\
		{		int16_t i =0;																											\
			  for(i=0;i<4;i++)																								 	\
				{																																	\
				 (ptr)->radius.s[i]= data[i];																	\
				}																																	\
				(ptr)->vision_mode = data[4];                                     \
		}		
/**********************双板时（云台）使用********************************/		
#define get_revolver_limit_measure(ptr, data)                   \
{                                                                   \
  (ptr)->if_shoot = (uint8_t)(data)[0] ;       										\
}

/**********************双板时（云台）使用********************************/		
#define get_bullet_speed_measure(ptr, data)														\
{   int16_t i =0;                                                     \
		for(i=0;i<4;i++)																								 	\
		{																																	\
			(ptr)->s[i]= data[i];																						\
		}																																	\
}

/**********************双板时（云台）使用********************************/	
#define gimbal_get_cap_measure(ptr, data)                                  \
{                                                                   \
 (ptr)->power = (uint16_t)((data)[0] << 8 | (data)[1]);          \
 (ptr)->voltage = (uint16_t)((data)[2] << 8 | (data)[3]);      	\
 (ptr)->current = (uint16_t)((data)[4] << 8 | (data)[5]);  			\
}

//浙纺超电
#define get_robofuture_cap_measure(ptr, data)			\
{																									\
	(ptr)->warning = data[0];           \
	(ptr)->DischargeStop = (data)[1];     \
	(ptr)->chassis_power = (uint16_t)((data)[2] << 8 | (data)[3]); \
	(ptr)->ele_quantity = (uint16_t)((data)[4] << 8 | (data)[5]); \
}
/********************CAN线发送的结构体定义***************************/
/*****************发送特定信息专用*************************/
static CAN_TxHeaderTypeDef  chassis_tx_message;
static uint8_t              chassis_can_send_data[8];	

/*************上下双板时（云台）使用**************/
Chassis_Order chassis_order_vx;
Chassis_Order chassis_order_vy;
Chassis_Order chassis_order_wz;
Yaw_Pitch yaw;
Yaw_Pitch pitch;
//static Chassis_Order bullet_speed;

fp32 speed_offest;
bool_t if_speed_update;
/*************上下双板时（底盘）使用**************/
UI_order_to_chassis UI_Measure;
static chassis_order_measure_t chassis_order_measure;
Shoot shoot_speed;

/********************通用***************************/
Chassis_Power_t		cap_measure;
robofuture_cap_measure_t robo_cap_measure;

/*********************通用********************/
/**
  * @brief          返回超级电容数据
  * @param[in]      Null
  * @retval         Null
  */
const Chassis_Power_t *get_cap_measure_point(void)
{
    return &cap_measure;
}


/**
  * @brief          返回超级电容数据
  * @param[in]      Null
  * @retval         Null
  */
const robofuture_cap_measure_t *get_robo_cap_measure_point(void)
{
    return &robo_cap_measure;
}

/**
  * @brief          返回电机对应地址
  * @param[in]      Null
  * @retval         Null
  */
/*************上下双板时使用**************/
/**
  * @brief          返回云台对底盘命令
  * @param[in]      Null
  * @retval         Null
  */
const chassis_order_measure_t *get_Gimbal_Oreder()
{
	return &chassis_order_measure;
}

/**
  * @brief          返回UI数据
  * @param[in]      Null
  * @retval         Null
  */
const UI_order_to_chassis *get_UI_measure()
{
	return &UI_Measure;
}


/**
************************************************************************
* @brief:      	can_bsp_init(void)
* @param:       void
* @retval:     	void
* @details:    	CAN 使能
************************************************************************
**/
void bsp_can_init(void)
{ 
	CANFilterInit();
	HAL_CAN_Start(&hcan1);   

	HAL_CAN_ActivateNotification(&hcan1, CAN_IT_RX_FIFO0_MSG_PENDING);	
}

/**
  * @brief          CAN线过滤器初始化
  * @param[in]      Null
  * @retval         Null
  */
void CANFilterInit(void)
{
		//此部分后续进行优化
    CAN_FilterTypeDef can_filter_st;
    can_filter_st.FilterActivation = ENABLE;
    can_filter_st.FilterMode = CAN_FILTERMODE_IDMASK;
    can_filter_st.FilterScale = CAN_FILTERSCALE_32BIT;
    can_filter_st.FilterIdHigh = 0x0000;
    can_filter_st.FilterIdLow = 0x0000;
    can_filter_st.FilterMaskIdHigh = 0x0000;
    can_filter_st.FilterMaskIdLow = 0x0000;
    can_filter_st.FilterBank = 0;
    can_filter_st.FilterFIFOAssignment = CAN_RX_FIFO0;
    HAL_CAN_ConfigFilter(&hcan1, &can_filter_st);
    HAL_CAN_Start(&hcan1);
    HAL_CAN_ActivateNotification(&hcan1, CAN_IT_RX_FIFO0_MSG_PENDING);


    can_filter_st.SlaveStartFilterBank = 14;
    can_filter_st.FilterBank = 14;
    HAL_CAN_ConfigFilter(&hcan2, &can_filter_st);
    HAL_CAN_Start(&hcan2);
    HAL_CAN_ActivateNotification(&hcan2, CAN_IT_RX_FIFO0_MSG_PENDING);

}

/**
	* @brief          CAN线发送电机输出值
	* @param[in]      CANTxInstance_t *：发送结构体
	* @param[in]      motor1：发送信息
	* @param[in]      motor2：发送信息
	* @param[in]      motor3：发送信息
	* @param[in]      motor4：发送信息
  * @retval         Null
  */
 
void CANSendToMotor(CANTxInstance_t *tx_message,int16_t motor1,int16_t motor2,int16_t motor3,int16_t motor4)
{
	uint32_t send_mail_box;
	uint8_t tx_buffer[8];
	
	tx_buffer[0] =	motor1 >> 8;
	tx_buffer[1] =	motor1 ;
	tx_buffer[2] =	motor2 >> 8;
	tx_buffer[3] =	motor2 ;
	tx_buffer[4] =	motor3 >> 8;
	tx_buffer[5] =	motor3 ;
	tx_buffer[6] =	motor4 >> 8;
	tx_buffer[7] =	motor4 ;
	
	HAL_CAN_AddTxMessage(tx_message->can_handle,&tx_message->tx_message_data,tx_buffer,&send_mail_box);
}

/**
	* @brief          CAN线发送uint16型数据进行通信
	* @param[in]      CANTxInstance_t *：发送结构体
	* @param[in]      message1：发送信息
	* @param[in]      message2：发送信息
	* @param[in]      message3：发送信息
	* @param[in]      message4：发送信息
  * @retval         Null
  */
void CANSend16Message(CANTxInstance_t *tx_message,uint16_t message1,uint16_t message2,uint16_t message3,uint16_t message4)
{
	uint32_t send_mail_box;
	uint8_t tx_buffer[8];
	
	tx_buffer[0] =	message1 >> 8;
	tx_buffer[1] =	message1 ;
	tx_buffer[2] =	message2 >> 8;
	tx_buffer[3] =	message2 ;
	tx_buffer[4] =	message3 >> 8;
	tx_buffer[5] =	message3 ;
	tx_buffer[6] =	message4 >> 8;
	tx_buffer[7] =	message4 ;
	
	HAL_CAN_AddTxMessage(tx_message->can_handle,&tx_message->tx_message_data,tx_buffer,&send_mail_box);
}

/**
	* @brief          CAN线发送float型数据通信
	* @param[in]      CANTxInstance_t *：发送结构体
	* @param[in]      *message1：发送信息
	* @param[in]      *message2：发送信息
  * @retval         Null
  */
void CANSendFpMessage(CANTxInstance_t *tx_message,fp32 *message1,fp32 *message2)
{
	uint32_t send_mail_box;
	uint16_t i;
	uint8_t tx_buffer[8];
	Un1 in_un1,in_un2; 
	
	in_un1.f = *message1;
	in_un2.f = *message2;
	for(i=0;i<4;i++)
	{
		tx_buffer[i] = 	in_un1.s[i];
		tx_buffer[i+4] = in_un2.s[i];
	}
	
	HAL_CAN_AddTxMessage(tx_message->can_handle,&tx_message->tx_message_data,tx_buffer,&send_mail_box);
}

/**
	* @brief          发送uint8_t类型数据
  * @param[in]      *message[8]：数据指针
  * @retval         Null
  */
void CANSendU8Message(CANTxInstance_t *tx_message,uint8_t message[8])
{
	uint32_t send_mail_box;
	uint16_t i;
	uint8_t tx_buffer[8];
	
	for(i=0;i<8;i++)
	{
		tx_buffer[i] = message[i];
	}
	
	HAL_CAN_AddTxMessage(tx_message->can_handle,&tx_message->tx_message_data,tx_buffer,&send_mail_box);
}

/**
	* @brief          RxInit的初始化设置
  * @param[in]      *input_init：需要初始化的CAN_Rx_Init_t结构体
	* @param[in]      can_num：挂载CAN线号
	* @param[in]      module_rx_id：设置的ID
  * @param[in]      *module_address：模块地址
	* @param[in]      *DecodeCallback：解码函数
  * @retval         CAN_Rx_Instance_t*：新建的CAN接收实例地址
  */
void CANRxInitSet(CAN_Rx_Instance_t *input_init,CAN_NAME_e can_num,uint32_t module_rx_id,void *module_address,ModuleCallback *DecodeCallback)
{
	input_init->can_id = can_num;
	input_init->rx_id = module_rx_id;
	input_init->module_address = module_address;
	input_init->CANMsgCallback = DecodeCallback;
}

/**
	* @brief          CAN总线接收实例登记函数
  * @param[in]      *input_rx：CAN接收初始化结构体
  * @retval         CAN_Rx_Instance_t*：新建的CAN接收实例地址
  */
CAN_Rx_Instance_t *CANRxRegister(CAN_Rx_Instance_t *input_rx)
{
	//给实例指针进行地址记录-登记
	if(input_rx->can_id == ON_CAN1){
		rx_instance[0][can1_register_id++] = input_rx;
	}else if(input_rx->can_id == ON_CAN2){
		rx_instance[1][can2_register_id++] = input_rx;
	}
	return input_rx;
}

/**
  * @brief          CAN1接收回调函数
  * @param[in]      CAN_RxHeaderTypeDef*：接收结构体
  * @retval         (*buffer_data)[8]：存储接收数据
  */
void CAN1Callback(CAN_RxHeaderTypeDef *buffer_rx_header,uint8_t *buffer_data)
{
	for (size_t i = 0; i < can1_register_id; ++i){ // 两者相等说明这是要找的实例
		if (buffer_rx_header->StdId == rx_instance[0][i]->rx_id){
			if (rx_instance[0][i]->CANMsgCallback != NULL) {// 回调函数不为空就调用
				memcpy(rx_instance[0][i]->rx_buff, buffer_data, 8); // 消息拷贝到对应实例
				rx_instance[0][i]->CANMsgCallback(rx_instance[0][i]);     // 触发回调进行数据解析和处理
			}
			return;
		}
	}
}

/**
  * @brief          CAN2接收回调函数
  * @param[in]      CAN_RxHeaderTypeDef*：接收结构体
  * @retval         (*buffer_data)[8]：存储接收数据
  */
void CAN2Callback(CAN_RxHeaderTypeDef *buffer_rx_header,uint8_t *buffer_data)
{
	for (size_t i = 0; i < can2_register_id; ++i){ // 两者相等说明这是要找的实例
		if (buffer_rx_header->StdId == rx_instance[1][i]->rx_id){
			if (rx_instance[1][i]->CANMsgCallback != NULL) {// 回调函数不为空就调用
				memcpy(rx_instance[1][i]->rx_buff, buffer_data, 8); // 消息拷贝到对应实例
				rx_instance[1][i]->CANMsgCallback(rx_instance[1][i]);     // 触发回调进行数据解析和处理
			}
			return;
		}
	}
}

/**
  * @brief          hal库CAN回调函数,接收电机数据
  * @param[in]      hcan:CAN句柄指针
  * @retval         Null
  */
void HAL_CAN_RxFifo0MsgPendingCallback(CAN_HandleTypeDef *hcan)
{
	CAN_RxHeaderTypeDef rx_header;
	uint8_t rx_data1[8];
	uint8_t rx_data2[8];
	
	if(hcan->Instance == CAN1){
			HAL_CAN_GetRxMessage(hcan,CAN_RX_FIFO0,&rx_header,rx_data1);
			CAN1Callback(&rx_header, rx_data1);
	}else if(hcan->Instance == CAN2){
			HAL_CAN_GetRxMessage(hcan,CAN_RX_FIFO0,&rx_header,rx_data2);
			CAN2Callback(&rx_header, rx_data2);
	}
}

/**
************************************************************************
* @brief:      	canx_bsp_send_data(FDCAN_HandleTypeDef *hfdcan, uint16_t id, uint8_t *data, uint32_t len)
* @param:       hcan: CAN句柄
* @param:       id: 	CAN设备ID
* @param:       data: 发送的数据
* @param:       len:  发送的数据长度
* @retval:     	void
* @details:    	发送数据
************************************************************************
**/
uint8_t canx_send_data(CAN_HandleTypeDef *hcan, uint16_t id, uint8_t *data, uint32_t len)
{	
	CAN_TxHeaderTypeDef	tx_header;
	
	tx_header.StdId = id;
	tx_header.ExtId = 0;
	tx_header.IDE   = 0;
	tx_header.RTR   = 0;
	tx_header.DLC   = len;
  /*找到空的发送邮箱，把数据发送出去*/
	if(HAL_CAN_AddTxMessage(hcan, &tx_header, data, (uint32_t*)CAN_TX_MAILBOX0) != HAL_OK) {
		if(HAL_CAN_AddTxMessage(hcan, &tx_header, data, (uint32_t*)CAN_TX_MAILBOX1) != HAL_OK) {
			HAL_CAN_AddTxMessage(hcan, &tx_header, data, (uint32_t*)CAN_TX_MAILBOX2);
    }
  }
  return 0;
}
/**
************************************************************************
* @brief:      	canx_bsp_receive(CAN_HandleTypeDef *hcan, uint8_t *buf)
* @param:       hcan: CAN句柄
* @param[out]:  rec_id: 	接收到数据的CAN设备ID
* @param:       buf：接收数据缓存
* @retval:     	接收的数据长度
* @details:    	接收数据
************************************************************************
**/
uint8_t canx_receive(hcan_t *hcan, uint16_t *rec_id, uint8_t *buf)
{	
	CAN_RxHeaderTypeDef rx_header;
  if (HAL_CAN_GetRxMessage(hcan, CAN_RX_FIFO0, &rx_header, buf) == HAL_OK)
	{
		*rec_id = rx_header.StdId;
		return rx_header.DLC; //接收数据长度
	}
	else
		return 0;
}
/**
************************************************************************
* @brief:      	HAL_CAN_RxFifo0MsgPendingCallback(CAN_HandleTypeDef *hcan)
* @param:       hfdcan；FDCAN句柄
* @param:       RxFifo0ITs：中断标志位
* @retval:     	void
* @details:    	HAL库的CAN中断回调函数
************************************************************************
**/
void CAN_CMD_CAP(uint16_t motor1)
{
	  uint32_t send_mail_box;
    chassis_tx_message.StdId = 0x210;
    chassis_tx_message.IDE = CAN_ID_STD;
    chassis_tx_message.RTR = CAN_RTR_DATA;
    chassis_tx_message.DLC = 0x08;
    chassis_can_send_data[0] = motor1 >> 8;
    chassis_can_send_data[1] = motor1;
    
		HAL_CAN_AddTxMessage(&hcan2, &chassis_tx_message, chassis_can_send_data, &send_mail_box);
}

void CAN_CMD_ROBO_CAP(uint8_t dischargecmd,uint8_t ctrlmode,uint16_t chassis_power_buff,uint16_t chassis_power_buff_limit,uint16_t chassis_power_limit)
{
	  uint32_t send_mail_box;
    chassis_tx_message.StdId = 0x100;
    chassis_tx_message.IDE = CAN_ID_STD;
    chassis_tx_message.RTR = CAN_RTR_DATA;
    chassis_tx_message.DLC = 0x08;

		chassis_can_send_data[0]=dischargecmd;
		chassis_can_send_data[1]=ctrlmode;
		chassis_can_send_data[2]=chassis_power_buff >> 8;
		chassis_can_send_data[3]=chassis_power_buff;
    chassis_can_send_data[4] = chassis_power_buff_limit >> 8;
    chassis_can_send_data[5] = chassis_power_buff_limit;
		chassis_can_send_data[6] = chassis_power_limit >> 8;
    chassis_can_send_data[7] = chassis_power_limit;
    
		HAL_CAN_AddTxMessage(&hcan1, &chassis_tx_message, chassis_can_send_data, &send_mail_box);
} 
__weak void can1_rx_callback(void)
{
}

__weak void can2_rx_callback(void)
{
}
