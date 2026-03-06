/**
 * @file super_cap.h
 * @brief Public declarations for Modules layer.
 */

#ifndef __SUPER_CAP_H
#define __SUPER_CAP_H

#include "struct_typedef.h"
#include "bsp_can.h"

//超电类型选择
#define WULIE_CAP 0
#define ROBOFUTURE_CAP 1
#define TIANXUCAP 2
#define SUPER_CAP_TYPE TIANXUCAP
#ifdef __cplusplus
extern "C"{
#endif
	
#ifdef __cplusplus
//浙纺超电
typedef struct
{
	uint8_t warning;
	uint8_t charge_sign;
	uint16_t chassis_power;
	uint16_t ele_quantity;
}Robofuture_Cap_Measure_t;

//雾列超电
typedef struct
{
	fp32 Cap_input_vol;
	fp32 Cap_voltage;
	fp32 Cap_current;
	uint16_t Cap_power;
}WuLie_Cap_Measure_t;

//雾列超电
typedef struct
{
	fp32 Cap_input_vol;
	fp32 Cap_input_current;
	fp32 Cap_voltage;
	uint16_t Cap_power;
}TianXu_Cap_Measure_t;

class Cap_t
{
	public:
		Robofuture_Cap_Measure_t robofuture_cap;
		WuLie_Cap_Measure_t 	 wulie_cap;
		TianXu_Cap_Measure_t     tianxu_cap;
		
		Cap_t();
		CAN_Rx_Instance_t 		 super_cap_can;				//超电CAN实例指针
		uint8_t					 send_data[8];
	
		void SuperCapInit(CAN_NAME_e can_num,uint32_t input_id);
		void EncodeControlData(uint8_t dischargecmd,uint8_t ctrlmode,uint16_t chassis_power_buff,uint16_t chassis_power_limit,uint16_t chassis_power_buff_limit);
		void EncodeControlData(uint16_t chassis_power_limit);
};

void RegisterSuperCap(CAN_NAME_e can_num,uint32_t input_id);
Cap_t *GetCapPointer();

#endif
	
#ifdef __cplusplus
}
#endif

#endif

