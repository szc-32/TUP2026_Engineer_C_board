#include "DMmotor.h"
#include "string.h"
/*
目前代码不够完善，添加电机需要通过定义结构体数组（枚举类型定义）
*/
//保存实例地址
DMMotorInstance *DMMotorInstances[1];

uint8_t dm_motor_num=0;


//电机反馈帧
#define get_DM4310_motor_measure(ptr, rx_data) \
{ \
     \
    (ptr)->dm_motor_measure.id  = (rx_data[0]) & 0x0F;  /* 控制器ID（对应手册MST_ID） */ \
    (ptr)->dm_motor_measure .state = (rx_data[0]) >> 4;       /* 故障码（对应手册ERR） */ \
    \
    /* 位置POS */ \
    (ptr)->dm_motor_measure .p_int  = (uint16_t)((rx_data[1] << 8) | rx_data[2]);  /* POS[15:8] + POS[7:0] */ \
    \
    /* 速度VEL */ \
    (ptr)->dm_motor_measure .v_int  = (uint16_t)((rx_data[3] << 4) | (rx_data[4] >> 4));  /* VEL[11:4] + VEL[3:0] */ \
    \
    /* 扭矩T*/ \
    (ptr)->dm_motor_measure .t_int  = (uint16_t)(((rx_data[4] & 0x0F) << 8) | rx_data[5]);  /* T[11:8] + T[7:0] */ \
     \
    (ptr)->dm_motor_measure .pos  = uint_to_float((ptr)->dm_motor_measure .p_int, -(ptr)->tmp .PMAX , (ptr)->tmp.PMAX, 16); \
    (ptr)->dm_motor_measure .vel  = uint_to_float((ptr)->dm_motor_measure .v_int, -(ptr)->tmp .VMAX, (ptr)->tmp.VMAX, 12); \
    (ptr)->dm_motor_measure .tor = uint_to_float((ptr)->dm_motor_measure .t_int, -(ptr)->tmp .TMAX, (ptr)->tmp.TMAX, 12); \
    \
    (ptr)->dm_motor_measure .Tmos = (float)(rx_data[6]);    /* 驱动MOS管温度（ */ \
    (ptr)->dm_motor_measure .Tcoil = (float)(rx_data[7]);  /* 电机线圈温度 */ \
}



/**
************************************************************************
* @brief:      	enable_motor_mode: 启用电机模式函数
* @param[in]:   hcan:     指向CAN_HandleTypeDef结构的指针
* @param[in]:   motor_id: 电机ID，指定目标电机
* @param[in]:   mode_id:  模式ID，指定要开启的模式
* @retval:     	void
* @details:    	通过CAN总线向特定电机发送启用特定模式的命令
************************************************************************
**/
void enable_motor_mode(hcan_t* hcan, uint16_t motor_id, uint16_t mode_id)
{
	uint8_t data[8];
	uint16_t id = motor_id + mode_id;
	
	data[0] = 0xFF;
	data[1] = 0xFF;
	data[2] = 0xFF;
	data[3] = 0xFF;
	data[4] = 0xFF;
	data[5] = 0xFF;
	data[6] = 0xFF;
	data[7] = 0xFC;
	
	canx_send_data(hcan, id, data, 8);
}

//仿照大疆电机解码
static void DecodeDMMotor(CAN_Rx_Instance_t *rx_instance)
{
	uint8_t *rxbuff = rx_instance->rx_buff;
	//address是void*需强制转化
	DMMotorInstance *dm_motor_pit = (DMMotorInstance *)rx_instance->module_address;
	DM_motor_t *dm_motor_para = &(dm_motor_pit->dm_motor[Motor1]) ;
		switch (dm_motor_para->mst_id)
	{
			case 0x00:
			case 0x02:
				get_DM4310_motor_measure(dm_motor_para, rxbuff);
				break;
			default:
				break;
	}

}

/**
  * @brief          达妙电机示例初始化
  * @param[in]      input_type：电机类型
  * @param[in]      can_num：CAN线挂载的ID
  * @param[in]      input_id：电调ID号设置
  * @param[in]      input_control_type：控制方式
  * @retval         Null
  */
void DMMotorInstance::DMMotorInit(DM_Motor_Setting_t *config)
{
//	fp32 max_out;
//	uint32_t rx_id,am_rx_id;
//	//检查一下有没有问题
//	//ID错误不在范围内ErrorCode
//	if(config->set_id <= 0 || config->set_id > 8)
//		motor_watch.error_code =  ID_RANGE_ERROR;
//	
	  dm_motor_settings = config;
	//初始化电机设置
	switch(dm_motor_settings->dm_control_type)
	{
		case mit_mode:
			memset(&dm_motor[Motor1], 0, sizeof(dm_motor[Motor1]));
	// 设置Motor1的电机信息
		
			dm_motor[Motor1].id = (dm_motor_settings->rx_id != 0U) ? dm_motor_settings->rx_id : 0x01U;
	    dm_motor[Motor1].mst_id  = (dm_motor_settings->mst_id != 0U) ? dm_motor_settings->mst_id : 0x02U;
	    dm_motor[Motor1].tmp.read_flag = 1;
	    dm_motor[Motor1].dm_ctrl_set.mode 	= mit_mode;
		  dm_motor[Motor1].tmp.PMAX = 3.14f;
		  dm_motor[Motor1].tmp.VMAX = 10.0f;
			dm_motor[Motor1].tmp.TMAX = 10.0f;//仅初始化一次的参数在这里赋值，注意必须与上位机保持一致
			break;
		case pos_mode:
			dm_motor[Motor1].dm_ctrl_set.mode 	= pos_mode;//错误码处理！！！！！！！！达妙电机可以回传错误码（仿真里的state，建议利用一下，代码中不含有错误码处理
			break;
		case spd_mode:
			dm_motor[Motor1].dm_ctrl_set.mode 	= spd_mode;
			break;
		case psi_mode:
			dm_motor[Motor1].dm_ctrl_set.mode 	= psi_mode;
			break;
		default:
			//ErrorCode
//			motor_watch.error_code = MOTOR_TYPE_ERROR;
			break;
	}
   
	//进行CAN线登记注册
		CANRxInitSet(&dm_motor_settings->dm_motor_can,(CAN_NAME_e)dm_motor_settings->can_id,dm_motor[Motor1].mst_id,this,DecodeDMMotor);
	
	  CANRxRegister(&dm_motor_settings->dm_motor_can);
	
//	motor_watch.InitDeviceStatus(1000);
}



/**
************************************************************************
* @brief:      	float_to_uint: 浮点数转换为无符号整数函数
* @param[in]:   x_float:	待转换的浮点数
* @param[in]:   x_min:		范围最小值
* @param[in]:   x_max:		范围最大值
* @param[in]:   bits: 		目标无符号整数的位数
* @retval:     	无符号整数结果
* @details:    	将给定的浮点数 x 在指定范围 [x_min, x_max] 内进行线性映射，映射结果为一个指定位数的无符号整数
************************************************************************
**/
int float_to_uint(float x_float, float x_min, float x_max, int bits)
{
	/* Converts a float to an unsigned int, given range and number of bits */
	float span = x_max - x_min;
	float offset = x_min;
	return (int) ((x_float-offset)*((float)((1<<bits)-1))/span);
}


/**
************************************************************************
* @brief:      	uint_to_float: 无符号整数转换为浮点数函数
* @param[in]:   x_int: 待转换的无符号整数
* @param[in]:   x_min: 范围最小值
* @param[in]:   x_max: 范围最大值
* @param[in]:   bits:  无符号整数的位数
* @retval:     	浮点数结果
* @details:    	将给定的无符号整数 x_int 在指定范围 [x_min, x_max] 内进行线性映射，映射结果为一个浮点数
************************************************************************
**/
float uint_to_float(int x_int, float x_min, float x_max, int bits)
{
	/* converts unsigned int to float, given range and number of bits */
	float span = x_max - x_min;
	float offset = x_min;
	return ((float)x_int)*span/((float)((1<<bits)-1)) + offset;
}



/**
************************************************************************
* @brief:      	mit_ctrl: MIT模式下的电机控制函数
* @param[in]:   hcan:			指向CAN_HandleTypeDef结构的指针，用于指定CAN总线
* @param[in]:   motor_id:	电机ID，指定目标电机
* @param[in]:   pos:			位置给定值
* @param[in]:   vel:			速度给定值
* @param[in]:   kp:				位置比例系数
* @param[in]:   kd:				位置微分系数
* @param[in]:   torq:			转矩给定值
* @retval:     	void
* @details:    	通过CAN总线向电机发送MIT模式下的控制帧。
************************************************************************
**/
void mit_ctrl(hcan_t* hcan, DM_motor_t *motor, uint16_t motor_id, float pos, float vel,float kp, float kd, float tor)
{
	uint8_t data[8];
	uint16_t pos_tmp,vel_tmp,kp_tmp,kd_tmp,tor_tmp;
	uint16_t id = motor_id + MIT_MODE;

	pos_tmp = float_to_uint(pos, -motor->tmp.PMAX, motor->tmp.PMAX, 16);
	vel_tmp = float_to_uint(vel, -motor->tmp.VMAX, motor->tmp.VMAX, 12);
	tor_tmp = float_to_uint(tor, -motor->tmp.TMAX, motor->tmp.TMAX, 12);
	kp_tmp  = float_to_uint(kp,  KP_MIN, KP_MAX, 12);
	kd_tmp  = float_to_uint(kd,  KD_MIN, KD_MAX, 12);

	data[0] = (pos_tmp >> 8);
	data[1] = pos_tmp;
	data[2] = (vel_tmp >> 4);
	data[3] = ((vel_tmp&0xF)<<4)|(kp_tmp>>8);
	data[4] = kp_tmp;
	data[5] = (kd_tmp >> 4);
	data[6] = ((kd_tmp&0xF)<<4)|(tor_tmp>>8);
	data[7] = tor_tmp;
	
	canx_send_data(hcan, id, data, 8);
}



/**
************************************************************************
* @brief:      	pos_speed_ctrl: 位置速度控制函数
* @param[in]:   hcan:			指向CAN_HandleTypeDef结构的指针，用于指定CAN总线
* @param[in]:   motor_id:	电机ID，指定目标电机
* @param[in]:   vel:			速度给定值
* @retval:     	void
* @details:    	通过CAN总线向电机发送位置速度控制命令
************************************************************************
**/
void pos_ctrl(hcan_t* hcan,uint16_t motor_id, float pos, float vel)
{
	uint16_t id;
	uint8_t *pbuf, *vbuf;
	uint8_t data[8];
	
	id = motor_id + POS_MODE;
	pbuf=(uint8_t*)&pos;
	vbuf=(uint8_t*)&vel;
	
	data[0] = *pbuf;
	data[1] = *(pbuf+1);
	data[2] = *(pbuf+2);
	data[3] = *(pbuf+3);

	data[4] = *vbuf;
	data[5] = *(vbuf+1);
	data[6] = *(vbuf+2);
	data[7] = *(vbuf+3);
	
	canx_send_data(hcan, id, data, 8);
}



/**
************************************************************************
* @brief:      	speed_ctrl: 速度控制函数
* @param[in]:   hcan: 		指向CAN_HandleTypeDef结构的指针，用于指定CAN总线
* @param[in]:   motor_id: 电机ID，指定目标电机
* @param[in]:   vel: 			速度给定值
* @retval:     	void
* @details:    	通过CAN总线向电机发送速度控制命令
************************************************************************
**/
void spd_ctrl(hcan_t* hcan, uint16_t motor_id, float vel)
{
	uint16_t id;
	uint8_t *vbuf;
	uint8_t data[4];
	
	id = motor_id + SPD_MODE;
	vbuf=(uint8_t*)&vel;
	
	data[0] = *vbuf;
	data[1] = *(vbuf+1);
	data[2] = *(vbuf+2);
	data[3] = *(vbuf+3);
	
	canx_send_data(hcan, id, data, 4);
}



/**
************************************************************************
* @brief:      	pos_speed_ctrl: 混控模式
* @param[in]:   hcan:			指向CAN_HandleTypeDef结构的指针，用于指定CAN总线
* @param[in]:   motor_id:	电机ID，指定目标电机
* @param[in]:   pos:			位置给定值
* @param[in]:   vel:			速度给定值
* @param[in]:   i:				电流给定值
* @retval:     	void
* @details:    	通过CAN总线向电机发送位置速度控制命令
************************************************************************
**/
void psi_ctrl(hcan_t* hcan, uint16_t motor_id, float pos, float vel, float cur)
{
	uint16_t id;
	uint8_t *pbuf, *vbuf, *ibuf;
	uint8_t data[8];
	
	uint16_t u16_vel = vel*100;
	uint16_t u16_cur  = cur*10000;
	
	id = motor_id + PSI_MODE;
	pbuf=(uint8_t*)&pos;
	vbuf=(uint8_t*)&u16_vel;
	ibuf=(uint8_t*)&u16_cur;
	
	data[0] = *pbuf;
	data[1] = *(pbuf+1);
	data[2] = *(pbuf+2);
	data[3] = *(pbuf+3);

	data[4] = *vbuf;
	data[5] = *(vbuf+1);
	
	data[6] = *ibuf;
	data[7] = *(ibuf+1);
	
	canx_send_data(hcan, id, data, 8);
}



/**
************************************************************************
* @brief:      	dm4310_clear: 清除DM4310电机控制参数函数
* @param[in]:   motor:   指向DM_motor_t结构的指针，包含电机相关信息和控制参数
* @retval:     	void
* @details:    	将DM4310电机的命令参数和控制参数清零，包括位置、速度、
*               比例增益(KP)、微分增益(KD)和扭矩
************************************************************************
**/
void dm_motor_clear_para(DM_motor_t *motor)
{
	motor->dm_ctrl_set.kd_set 	= 0;
	motor->dm_ctrl_set.kp_set	= 0;
	motor->dm_ctrl_set.pos_set = 0;
	motor->dm_ctrl_set.vel_set = 0;
	motor->dm_ctrl_set.tor_set = 0;
	motor->dm_ctrl_set.cur_set = 0;
}



/**
************************************************************************
* @brief:      	clear_err: 清除电机错误函数
* @param[in]:   hcan:     指向CAN_HandleTypeDef结构的指针
* @param[in]:   motor_id: 电机ID，指定目标电机
* @param[in]:   mode_id:  模式ID，指定要清除错误的模式
* @retval:     	void
* @details:    	通过CAN总线向特定电机发送清除错误的命令。
************************************************************************
**/
void clear_err(hcan_t* hcan, uint16_t motor_id, uint16_t mode_id)
{
	uint8_t data[8];
	uint16_t id = motor_id + mode_id;
	
	data[0] = 0xFF;
	data[1] = 0xFF;
	data[2] = 0xFF;
	data[3] = 0xFF;
	data[4] = 0xFF;
	data[5] = 0xFF;
	data[6] = 0xFF;
	data[7] = 0xFB;
	
	canx_send_data(hcan, id, data, 8);
}


/**
************************************************************************
* @brief:      	dm4310_clear_err: 清除DM4310电机错误函数
* @param[in]:   hcan: 	 指向CAN控制结构体的指针
* @param[in]:  	motor:   指向电机结构体的指针
* @retval:     	void
* @details:    	根据电机的控制模式，调用对应模式的清除错误函数
************************************************************************
**/
void dm_motor_clear_err(hcan_t* hcan, DM_motor_t *motor)
{
	switch(motor->dm_ctrl_set.mode)
	{
		case mit_mode:
			clear_err(hcan, motor->id, MIT_MODE);
			break;
		case pos_mode:
			clear_err(hcan, motor->id, POS_MODE);
			break;
		case spd_mode:
			clear_err(hcan, motor->id, SPD_MODE);
			break;
		case psi_mode:
			clear_err(hcan, motor->id, PSI_MODE);
			break;
	}	
}

/**
************************************************************************
* @brief:      	disable_motor_mode: 禁用电机模式函数
* @param[in]:   hcan:     指向CAN_HandleTypeDef结构的指针
* @param[in]:   motor_id: 电机ID，指定目标电机
* @param[in]:   mode_id:  模式ID，指定要禁用的模式
* @retval:     	void
* @details:    	通过CAN总线向特定电机发送禁用特定模式的命令
************************************************************************
**/
void disable_motor_mode(hcan_t* hcan, uint16_t motor_id, uint16_t mode_id)
{
	uint8_t data[8];
	uint16_t id = motor_id + mode_id;
	
	data[0] = 0xFF;
	data[1] = 0xFF;
	data[2] = 0xFF;
	data[3] = 0xFF;
	data[4] = 0xFF;
	data[5] = 0xFF;
	data[6] = 0xFF;
	data[7] = 0xFD;
	
	canx_send_data(hcan, id, data, 8);
}


/**
  * @brief          获得转子反馈信息的API
  * @param[in]      NULL
  * @retval         fp32
  */
DMmotor_fbpara_t *DMMotorInstance::GetDMMotorMeasure()
{
	return &dm_motor[Motor1].dm_motor_measure ;
}


fp32 DMMotorInstance::DMMotorEcdToAngle(fp32 offset_rad)
{
  fp32 relative_rad;
	relative_rad = dm_motor[Motor1].dm_motor_measure.pos - offset_rad;
 
  return relative_rad;
}

/**
************************************************************************
* @brief:      	dm4310_ctrl_send: 发送DM4310电机控制命令函数
* @param[in]:   hcan:    指向CAN_HandleTypeDef结构的指针
* @param[in]:   motor:   指向DM_motor_t结构的指针，包含电机相关信息和控制参数
* @retval:     	void
* @details:    	根据电机控制模式发送相应的命令到DM4310电机
*               支持的控制模式包括位置模式、位置速度控制模式和速度控制模式
************************************************************************
**/
void DMMotorInstance::DMMotorControl(hcan_t* hcan, DM_motor_t *motor)
{

		switch(motor->dm_ctrl_set.mode)
	{
		case mit_mode:
			mit_ctrl(hcan,motor, motor->id, motor->dm_ctrl_set.pos_set, motor->dm_ctrl_set.vel_set, motor->dm_ctrl_set.kp_set, motor->dm_ctrl_set.kd_set, motor->dm_ctrl_set.tor_set);
			break;
		case pos_mode:
			pos_ctrl(hcan, motor->id, motor->dm_ctrl_set.pos_set, motor->dm_ctrl_set.vel_set);
			break;
		case spd_mode:
			spd_ctrl(hcan, motor->id, motor->dm_ctrl_set.vel_set);
			break;
		case psi_mode:
			psi_ctrl(hcan, motor->id,motor->dm_ctrl_set.pos_set, motor->dm_ctrl_set.vel_set, motor->dm_ctrl_set.cur_set);
			break;
	}	
}

/**
************************************************************************
* @brief:      	dm4310_disable: 禁用DM4310电机控制模式函数
* @param[in]:   hcan:    指向CAN_HandleTypeDef结构的指针
* @param[in]:   motor:   指向DM_motor_t结构的指针，包含电机相关信息和控制参数
* @retval:     	void
* @details:    	根据电机控制模式禁用相应的模式，通过CAN总线发送禁用命令
*               支持的控制模式包括位置模式、位置速度控制模式和速度控制模式
************************************************************************
**/

void DMMotorInstance::DMMotorZeroForce(hcan_t* hcan, DM_motor_t *motor)
{
	
	disable_motor_mode(hcan, motor->id, MIT_MODE);

	dm_motor_clear_para(motor);
}

/**
  * @brief          电机类构造函数
  * @param[in]     	Null
  * @retval         Null
  */
DMMotorInstance::DMMotorInstance()
{
  dm_motor_settings = NULL;
  //一系列地址存储
  // 将当前实例的指针添加到数组中
  for (int i = 0; i < TOTAL_DMMOTOR_SUM; i++) 
  {
	if (DMMotorInstances[i] == NULL) 
	{
		DMMotorInstances[i] = this;
		dm_motor_num++;
    break;
    }
  }
}


/**
  * @brief          限制电机相对角度运动空间
  * @param[in]     	relative_angle_set：相对角度设定值
  * @param[in]     	add：添加值
  * @param[in]     	max_limit：最大角度
  * @param[in]     	min_limit：最小角度
  * @retval         Null
  */

fp32 DMMotorInstance::DMMotorWorkSpaceLimit(fp32 relative_angle_set,fp32 add,fp32 max_limit,fp32 min_limit)
{
	relative_angle_set -= add;//由于英雄pit轴抬高时pos值减小，反之可以改为加
	if(relative_angle_set < max_limit)
	{
		relative_angle_set = max_limit;
	}else if(relative_angle_set > min_limit)
	{
		relative_angle_set = min_limit;
	}
	return relative_angle_set;
}


/**
  * @brief          获取电机指针
  * @retval         null
  */
DMMotorInstance *DMMotorInstancePointer(uint8_t cnt)
{
	return DMMotorInstances[cnt];
}

void DMMotorInstance::DM_motor_para_init(DM_motor_t *motor)//在控制器参数中调用的
{
    
			motor->dm_ctrl_set.vel_set 	= 6.0f;//感觉和电机响应速度无关，我是修改的遥控器灵敏度增加的响应速度
			motor->dm_ctrl_set.pos_set 	= 0.0f;
			motor->dm_ctrl_set.tor_set  = 0.0f;
			motor->dm_ctrl_set.cur_set 	= 0.0f;
			motor->dm_ctrl_set.kp_set 	= 40.0f;
			motor->dm_ctrl_set.kd_set 	= 1.0f;
			hcan_t *dm_hcan = (dm_motor_settings->can_id == ON_CAN1) ? &hcan1 : &hcan2;
			enable_motor_mode(dm_hcan, dm_motor[Motor1].id, MIT_MODE );

}
