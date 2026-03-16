#ifndef __DMmotor_H
#define __DMmotor_H

#include "struct_typedef.h"
#include "motor.h"

#define MIT_MODE 			0x000
#define POS_MODE			0x100
#define SPD_MODE			0x200
#define PSI_MODE		  	0x300

#define KP_MIN 0.0f
#define KP_MAX 500.0f
#define KD_MIN 0.0f
#define KD_MAX 50.0f

#define TOTAL_DMMOTOR_SUM 1

#ifdef __cplusplus
extern "C"{
#endif

#ifdef __cplusplus
typedef enum
{
	Motor1,
	Motor2,
	Motor3,
	Motor4,
	Motor5,
	Motor6,
	Motor7,
	Motor8,
	Motor9,
	Motor10,
    num
} motor_num;


typedef enum
{
	mit_mode ,
	pos_mode ,
	spd_mode ,
	psi_mode
} DM_Control_Type_e;


// 电机回传信息结构体
typedef struct
{
	
    int id;
    int state;
    int p_int;
    int v_int;
    int t_int;
    int kp_int;
    int kd_int;
    float pos;//位置
    float vel;//速度
    float tor;//扭矩
    float Kp;
    float Kd;
    float Tmos;
    float Tcoil;

} DMmotor_fbpara_t;


// 电机参数设置结构体
typedef struct
{
    uint8_t mode;
    float pos_set;
    float vel_set;
    float tor_set;
	  float cur_set;
    float kp_set;
    float kd_set;
} DMmotor_ctrl_t;


// 电机参数
typedef struct
{
	uint8_t read_flag;
	uint8_t write_flag;
	uint8_t save_flag;
	
    float UV_Value;		// 低压保护值
    float KT_Value;		// 扭矩系数
    float OT_Value;		// 过温保护值
    float OC_Value;		// 过流保护值
    float ACC;			// 加速度
    float DEC;			// 减速度
    float MAX_SPD;		// 最大速度
    uint32_t MST_ID;	// 反馈ID
    uint32_t ESC_ID;	// 接收ID
    uint32_t TIMEOUT;	// 超时警报时间
    uint32_t cmode;		// 控制模式
    float  	 Damp;		// 电机粘滞系数
    float    Inertia;	// 电机转动惯量
    uint32_t hw_ver;	// 保留
    uint32_t sw_ver;	// 软件版本号
    uint32_t SN;		// 保留
    uint32_t NPP;		// 电机极对数
    float    Rs;		// 电阻
    float    Ls;		// 电感
    float    Flux;		// 磁链
    float    Gr;		// 齿轮减速比
    float    PMAX;		// 位置映射范围
    float    VMAX;		// 速度映射范围
    float    TMAX;		// 扭矩映射范围
    float    I_BW;		// 电流环控制带宽
    float    KP_ASR;	// 速度环Kp
    float    KI_ASR;	// 速度环Ki
    float    KP_APR;	// 位置环Kp
    float    KI_APR;	// 位置环Ki
    float    OV_Value;	// 过压保护值
    float    GREF;		// 齿轮力矩效率
    float    Deta;		// 速度环阻尼系数
    float 	 V_BW;		// 速度环滤波带宽
    float 	 IQ_cl;		// 电流环增强系数
    float    VL_cl;		// 速度环增强系数
    uint32_t can_br;	// CAN波特率代码
    uint32_t sub_ver;	// 子版本号
	float 	 u_off;		// u相偏置
	float	 v_off;		// v相偏置
	float	 k1;		// 补偿因子1
	float 	 k2;		// 补偿因子2
	float 	 m_off;		// 角度偏移
	float  	 dir;		// 方向
	float	 p_m;		// 电机位置
	float	 x_out;		// 输出轴位置
} esc_inf_t;

typedef struct
{
    uint16_t id;
	uint16_t mst_id;
    DMmotor_fbpara_t dm_motor_measure;
    DMmotor_ctrl_t dm_ctrl_set;
	  esc_inf_t tmp;
}DM_motor_t;

typedef struct
{
	const char *motor_name; //电机名称
    uint8_t can_id; //CAN线ID
	uint16_t rx_id;		//电机的ID号?
	uint16_t mst_id;//接收ID?
	Motor_Rotate_Direction_e direction; //电机旋转方向
	DM_Control_Type_e dm_control_type; //控制器选择类型
	//通用数据
	CAN_Rx_Instance_t dm_motor_can;				//电机CAN实例

}DM_Motor_Setting_t;



float uint_to_float(int x_int, float x_min, float x_max, int bits);
int float_to_uint(float x_float, float x_min, float x_max, int bits);

void dm_motor_enable(hcan_t* hcan, DM_motor_t *motor);
void dm_motor_disable(hcan_t* hcan, DM_motor_t *motor);
void dm_motor_clear_para(DM_motor_t *motor);
void dm_motor_clear_err(hcan_t* hcan, DM_motor_t *motor);
//void dm_motor_fbdata(motor_t *motor, uint8_t *rx_data);

void enable_motor_mode(hcan_t* hcan, uint16_t motor_id, uint16_t mode_id);
void disable_motor_mode(hcan_t* hcan, uint16_t motor_id, uint16_t mode_id);

void mit_ctrl(hcan_t* hcan, DM_motor_t *motor, uint16_t motor_id, float pos, float vel,float kp, float kd, float tor);
void pos_ctrl(hcan_t* hcan, uint16_t motor_id, float pos, float vel);
void spd_ctrl(hcan_t* hcan, uint16_t motor_id, float vel);
void psi_ctrl(hcan_t* hcan, uint16_t motor_id, float pos, float vel, float cur);

class DMMotorInstance
{
	private:
//		Motor_Watch_t motor_watch;	//电机观测器
		DM_Motor_Setting_t *dm_motor_settings;    //电机设置参数
		
	public:
		DM_motor_t dm_motor[num];

	
		/**构造函数**/
		DMMotorInstance();
		/**初始化函数**/
		void DMMotorInit(DM_Motor_Setting_t *config);
		fp32 DMMotorEcdToAngle(fp32 offset_rad);
		fp32 DMMotorWorkSpaceLimit(fp32 relative_rad_set,fp32 add_pit,fp32 max,fp32 min);
//		void ChangeControlMode(Control_Type_e change_type);
		/**控制函数**/
	  void DMMotorControl(hcan_t* hcan, DM_motor_t *motor);
		void DMMotorZeroForce(hcan_t* hcan, DM_motor_t *motor);
	  		void DM_motor_para_init(DM_motor_t *motor);
		/**电机状态信息接口函数**/
//		fp32 CalcTotalecd();
//		fp32 FilterSpeed(fp32 *fliter_num);
//		fp32 GetRotorW(); //获得角速度
//		fp32 GetOutputShaftW(); //获得输出轴角速度
//		fp32 GetRotorRad();  //获得转子的角度
//		fp32 GetOutputShaftRad(); //获得输出轴的角度
//		int16_t GetRotorRpm();  //获得转子RPM
//    int16_t GetGivenCurrent();

		DMmotor_fbpara_t *GetDMMotorMeasure();
		

//		/**指针地址函数**/
//		Motor_Setting_t *SetParamPointer();
//		int16_t *SendCurrentPointer();
//		/**CAN信息接口函数**/
//		uint32_t GetRxID();
//		CAN_NAME_e GetCANID();
//		/**监测函数**/
//		void MonitorInitState();
//		void MonitorTem();
//		void MonitorOnlineState();
		/**友元函数进行信息解码**/
    friend void DecodeDMMotor(CAN_Rx_Instance_t *rx_instance);

};


#endif

#ifdef __cplusplus
}
#endif

#endif

