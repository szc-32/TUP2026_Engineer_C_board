#ifndef __CHASSIS_H
#define __CHASSIS_H

#include "message_center.h"
#include "user_lib.h"
#include "robot_def.h"
#include "PID.h"
#include "motor.h"
#include "super_cap.h"
#include "operation_def.h"
#include "bsp_can.h"
#include "INS_task.h"
#include "pid.h"

#define rc_deadband_limit(input, output, dealine)        \
    {                                                    \
        if ((input) > (dealine) || (input) < -(dealine)) \
        {                                                \
            (output) = (input);                          \
        }                                                \
        else                                             \
        {                                                \
            (output) = 0;                                \
        }                                                \
    }
#define CHASSIS_TASK_INIT_TIME 200 //任务开始空闲一段时间

#define CHASSIS_CONTROL_TIME_MS 2  //底盘任务控制间隔 2ms

//底盘角度环参数
#define CHASSIS_MOTOR_YAW_PID_KP -12.0f
#define CHASSIS_MOTOR_YAW_PID_KI 0.0f 
#define CHASSIS_MOTOR_YAW_PID_KD -32.0f
#define CHASSIS_MOTOR_YAW_PID_MAX_OUT 7.0f
#define CHASSIS_MOTOR_YAW_PID_MAX_IOUT 2000.0f			

//底盘速度解算参数
#define CHASSIS_WZ_RC_SEN      0.01f      //不跟随云台的时候 遥控器的yaw遥杆（max 660）转化成车体旋转速度的比例
#define CHASSIS_WZ_CHANNEL     2          //在特殊模式下，可以通过遥控器控制旋转
#define CHASSIS_WZ_SET_SCALE   -0.072f    //底盘设置旋转速度，设置前后左右轮不同设定速度的比例分权 0为在几何中心，不需要补偿
#define MOTOR_DISTANCE_TO_CENTER 0.2f
#define MAX_WHEEL_SPEED 3.5f	//底盘电机最大速度（调试安全档）

#define CHASSIS_SPIN_WZ_SET 8.0f          //自旋行进固定角速度(rad/s)

#define M3508_MOTOR_RPM_TO_VECTOR 0.000415809748903494517209f //m3508转化成底盘速度(m/s)的比例，做两个宏 是因为可能换电机需要更换比例
#define CHASSIS_MOTOR_RPM_TO_VECTOR_SEN M3508_MOTOR_RPM_TO_VECTOR


// yaw/pitch control channels (shared in operation_def.h)
#ifndef YAW_CHANNEL
#define YAW_CHANNEL   0
#endif
#ifndef PITCH_CHANNEL
#define PITCH_CHANNEL 1
#endif
#define GIMBAL_MODE_CHANNEL 0
#define REVOLVER_MODE_CHANNEL 1
// chassis translation channels: left stick (forward/back and left/right)
#ifndef CHASSIS_X_CHANNEL
#define CHASSIS_X_CHANNEL 3
#endif
#ifndef CHASSIS_Y_CHANNEL
#define CHASSIS_Y_CHANNEL 2
#endif
//底盘控制模式
#define GIMBAL_CALC 0
#define CHASSIS_CALC 1
//底盘鼠标控制灵敏度
#define YAW_MOUSE_SEN   0.00005f
#define PITCH_MOUSE_SEN 0.00005f
//遥控器输入转化比例
#ifdef CHASSIS_VX_RC_SEN
#undef CHASSIS_VX_RC_SEN
#endif
#define CHASSIS_VX_RC_SEN 0.0055f  //遥控器前进摇杆（max 660）转化成车体前进速度（m/s）的比例
#ifdef CHASSIS_VY_RC_SEN
#undef CHASSIS_VY_RC_SEN
#endif
#define CHASSIS_VY_RC_SEN 0.0055f  //遥控器左右摇杆（max 660）转化成车体左右速度（m/s）的比例
//底盘yaw,pitch遥杆转化比例
#ifndef YAW_RC_SEN
#define YAW_RC_SEN    -0.000005f
#endif
#ifdef PITCH_RC_SEN
#undef PITCH_RC_SEN
#endif
#define PITCH_RC_SEN  -0.00000222f //0.005
//YAW左右转控制按钮
#define GIMBAL_LEFT_KEY KEY_PRESSED_OFFSET_Q
#define GIMBAL_RIGHT_KEY KEY_PRESSED_OFFSET_E

//底盘前后左右控制按键
#define CHASSIS_FRONT_KEY KEY_PRESSED_OFFSET_W
#define CHASSIS_BACK_KEY KEY_PRESSED_OFFSET_S
#define CHASSIS_LEFT_KEY KEY_PRESSED_OFFSET_A
#define CHASSIS_RIGHT_KEY KEY_PRESSED_OFFSET_D

//遥控器输入死区，因为遥控器存在差异，摇杆在中间，其值不一定为零
#ifndef RC_DEADBAND
#define RC_DEADBAND   2
#endif
//摇杆死区（底盘）
#define CHASSIS_RC_DEADLINE 10

//底盘运动过程最大前进速度
#ifdef NORMAL_MAX_CHASSIS_SPEED_X
#undef NORMAL_MAX_CHASSIS_SPEED_X
#endif
#define NORMAL_MAX_CHASSIS_SPEED_X 8.0f
//底盘运动过程最大平移速度
#ifdef NORMAL_MAX_CHASSIS_SPEED_Y
#undef NORMAL_MAX_CHASSIS_SPEED_Y
#endif
#define NORMAL_MAX_CHASSIS_SPEED_Y 6.0f

//变速小陀螺转速切换的间隔时间，单位ms
#define SPIN_SPEED_CHANGE_TIME 500
//变速小陀螺转速变化的斜坡量
#define SPIN_SPEED_CHANGE_RAMP 0.01f

//A板或者C板单板控制
#if BOARD_NUM == ONE_BOARD || (BOARD_NUM == TWO_BOARD && BOARD_PLACE == ON_CHASSIS)

	#define MAX_WHEEL_SPEED 3.5f	//底盘电机最大速度（调试安全档）

	#define M3508_MOTOR_RPM_TO_VECTOR 0.000415809748903494517209f //m3508转化成底盘速度(m/s)的比例，做两个宏 是因为可能换电机需要更换比例
	#define CHASSIS_MOTOR_RPM_TO_VECTOR_SEN M3508_MOTOR_RPM_TO_VECTOR
	//四轮速度转化为车体
	#define MOTOR_SPEED_TO_CHASSIS_SPEED_VX 0.25f
	#define MOTOR_SPEED_TO_CHASSIS_SPEED_VY 0.25f
	#define MOTOR_SPEED_TO_CHASSIS_SPEED_WZ 0.25f
	//底盘速度解算参数
    #define CHASSIS_MOTOR_SPEED_PID_KP 8000.0f//10000.0f
    #define CHASSIS_MOTOR_SPEED_PID_KI 0.0f	
    #define CHASSIS_MOTOR_SPEED_PID_KD  2.0f//2.7f
	#define CHASSIS_MOTOR_SPEED_PID_LF_KP 9200.0f
	#define CHASSIS_MOTOR_SPEED_PID_LF_KI 0.0f
	#define CHASSIS_MOTOR_SPEED_PID_LF_KD 2.5f
	#define CHASSIS_MOTOR_SPEED_PID_LB_KP 9200.0f
	#define CHASSIS_MOTOR_SPEED_PID_LB_KI 0.0f
	#define CHASSIS_MOTOR_SPEED_PID_LB_KD 2.5f
	#define CHASSIS_MOTOR_SPEED_PID_RB_KP 9200.0f
	#define CHASSIS_MOTOR_SPEED_PID_RB_KI 0.0f
	#define CHASSIS_MOTOR_SPEED_PID_RB_KD 2.5f
	#define CHASSIS_MOTOR_SPEED_PID_RF_KP 9200.0f
	#define CHASSIS_MOTOR_SPEED_PID_RF_KI 0.0f
	#define CHASSIS_MOTOR_SPEED_PID_RF_KD 2.5f
	#define M3508_MOTOR_SPEED_PID_MAX_OUT    6000.0f
    #define M3508_MOTOR_SPEED_PID_MAX_IOUT   1000.0f

	
//电压电流功率结构体
typedef struct
{
  fp32 BusV;
  fp32 Current;
  fp32 Power;
} ina226_t;

#endif
#ifdef __cplusplus
extern "C"{
#endif
	
#ifdef __cplusplus
typedef struct
{
  const  Dji_Motor_Measure_t*chassis_motor_measure; 
  fp32 accel;
  fp32 speed;
  fp32 speed_set;
  int16_t give_current;
  fp32 power;
  fp32 current_set;
  fp32 current;
  fp32 current_scale;
}chassis_motor_t;


class chassis_t
{
	private:
		/*****机器人平移旋转速度设定值*****/
		fp32 	robot_vx_set,robot_vy_set,robot_wz_set;
		/*****四轮速度（顺序依次为：LF、LB、RB、RF）*****/
		fp32 	wheel_speed[4];   							//机器人速度解算出单个轮的速度（四轮）
		fp32 	speed_set[4],speed[4];						//速度设定值和当前速度值
		int16_t symbol[4];  								//四轮旋转方向标志
		uint16_t 	chassis_max_power;							//底盘功率上限
		fp32 	chassis_power_buffer;						//底盘缓冲能量
		fp32 	chassis_real_power;							//底盘实际功率
		fp32    cap_v_out;									//输出电压
		fp32    real_scale_k;
		const chassis_order_measure_t *chassis_order;
	    const UI_order_to_chassis *IFSHIFT;
		const fp32 *chassis_INS_angle;  //底盘陀螺仪角度
	    fp32 chassis_angle;
	    Control_Mode_t chassis_mode;

		PID_t chassis_motor_speed_pid[4];
		PID_t chassis_angle_pid;	//底盘跟随角度pid
		PID_t chassis_current_pid[4];
        first_order_filter_type_t chassis_cmd_slow_set_vx;
        first_order_filter_type_t chassis_cmd_slow_set_vy;

        uint8_t updata; 

	    fp32 vx;                         //底盘速度 前进方向 前为正，单位 m/s
        fp32 vy;                         //底盘速度 左右方向 左为正  单位 m/s
        fp32 wz;                         //底盘旋转角速度，逆时针为正 单位 rad/s
	
        fp32 vx_set;                     //底盘设定速度 前进方向 前为正，单位 m/s
        fp32 vy_set;                     //底盘设定速度 左右方向 左为正，单位 m/s
	    fp32 wz_set;                     //底盘旋转角速度，逆时针为正 单位 rad/s

	    fp32 vx_follow_set;                     //底盘设定速度 前进方向 前为正，单位 m/s
        fp32 vy_follow_set;                     //底盘设定速度 左右方向 左为正，单位 m/s
	    fp32 wz_follow_set;                     //底盘旋转角速度，逆时针为正 单位 rad/s

	    fp32 vx_max_speed;               //前进方向最大速度 单位m/s 
        fp32 vx_min_speed;               //前进方向最小速度 单位m/s
        fp32 vy_max_speed;               //左右方向最大速度 单位m/s
        fp32 vy_min_speed;               //左右方向最小速度 单位m/s

        fp32 chassis_relative_angle_set;
        bool shift_flag;
	    fp32 scale_k;
	    fp32 sin_yaw;
	    fp32 cos_yaw;
	    fp32 angle_set;
	    fp32 all_pid_out;

	public:
	const Chassis_Power_t *chassis_cap_masure;
	const robofuture_cap_measure_t *robo_cap_measure;
	const ina226_t *Ina226_Chassis_Power;
	public:
		
	chassis_motor_t chassis_motor_info[4];
		chassis_t();
		
#if BOARD_NUM == TWO_BOARD && BOARD_PLACE == ON_GIMBAL //双板控制-C板为云台
		/*****信息实例*****/
		Chassis_Pub_Msg_t chassis_msg;  
	void SendMsgUpdate(); 								//发送信息更新
#elif BOARD_NUM == ONE_BOARD || (BOARD_NUM == TWO_BOARD && BOARD_PLACE == ON_CHASSIS) //单板控制 || 双板控制-A板作为底盘
		DJIMotorInstance chassis_motor[4];	//底盘电机实例（按顺序）
	
		/*****信息更新函数*****/
	
		void ChassisInfoUpdate();
		/*****控制及限幅函数*****/
		void ControlLoop();									//控制回环函数
		void VectorToWheelSpeed(fp32 *lf_wheel_speed,fp32 *lb_wheel_speed,fp32 *rb_wheel_speed,fp32 *rf_wheel_speed,fp32 scale_k);  //解算函数
		void SpeedLimit(); //速度限制函数
		void PowerLimit(); //功率限制函数
		void UphillPowerRedistribution(fp32 *angle,uint8_t flag); //上坡功率再分配函数
#endif
	  /*****控制设置函数*****/
		void ControlSet();
		void RobotTranslationSet();
		void NoFollowTranslationSet();
		void SpinTranslationSet();
		void ChassisInit();
		void Get_Information();
		void new_chassis_power_limit();
};
	chassis_t* chassispoint(void);	
#endif

void ChassisTask(void);
void ChassisInit(void);
#ifdef __cplusplus
}
#endif

#endif
