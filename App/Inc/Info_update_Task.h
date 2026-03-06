/**
 * @file Info_update_Task.h
 * @brief Public declarations for App layer.
 */

#ifndef __INFO_UPDATE_TASK_H
#define __INFO_UPDATE_TASK_H

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "stm32f4xx_hal.h"
#include "struct_typedef.h"
#include "chassis.h"

//#include "CUSUI.h"	
	
//#define VisionBufferLength_SH       150
#define    JUDGE_BUFFER_LEN          200
#define	USART_RX_BUF_LENGHT 		512

typedef struct __PACKED 
{
	uint8_t cmd;
  fp32 vx;
  fp32 vy;
  fp32 wz;
	uint8_t mode;
	uint8_t chassis_mode;
	uint8_t keyboard_mode;
	
char step_1;
char step_2;
	uint8_t user_control;
char user_y;
char user_z;
	uint8_t A_Board_init;
	uint8_t check_sum;
} chassis_msg_t;

//1+4+4+4+4+4+1=22
typedef struct __PACKED 
{
	uint8_t cmd;
	fp32 chassis_angle;
	fp32 ore;
	uint8_t check_sum;
}ancillary_msg_t;
//	
//#ifdef __cplusplus	
//	
//	
//#endif	
extern void Info_update_Task(void const * argument);
const ina226_t *get_ina226_data(void);
extern void Cap_Update_Task(void const * argumt);
extern void CUSUI_TASK(void const *argumt);
uint8_t JUDGE_usGetJudge_chassis_Power(void);
extern void Send_ancillary_msg();
#ifdef __cplusplus
}
#endif

#endif 

