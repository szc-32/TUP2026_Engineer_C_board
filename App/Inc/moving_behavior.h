/**
 * @file moving_behavior.h
 * @brief Public declarations for App layer.
 */

#ifndef MOVING_BEHAVIOR_H
#define MOVING_BEHAVIOR_H

#include "lifting.h"
#include "suction_cup.h"
#include "struct_typedef.h"
#include "forward_kinematics.h"

#define CUSTOM_CTRL_AS5600L_AXIS_NUM ARM_DOF
#define CUSTOM_CTRL_AS5600L_BYTES_PER_AXIS 2U
#define CUSTOM_CTRL_AS5600L_PAYLOAD_LEN (CUSTOM_CTRL_AS5600L_AXIS_NUM * CUSTOM_CTRL_AS5600L_BYTES_PER_AXIS)

enum DMArmAxisOrder_e
{
	DM_ARM_AXIS_J1 = 0,
	DM_ARM_AXIS_J2,
	DM_ARM_AXIS_J3,
	DM_ARM_AXIS_J4,
	DM_ARM_AXIS_J5,
	DM_ARM_AXIS_J6,
};

#ifdef __cplusplus
extern "C"{
#endif
	

#ifdef __cplusplus
enum NewRobotBehaviorMode_e
{
	NEW_BEHAVIOR_IDLE = 0,
	NEW_BEHAVIOR_HOME,
	NEW_BEHAVIOR_DM_ARM_MANUAL,
	NEW_BEHAVIOR_DM_ARM_AUTO,
	NEW_BEHAVIOR_DM_ARM_CALIBRATE,
	NEW_BEHAVIOR_DM_ARM_IK_WORK,
};

class moving_motor_t
{
	public:
      fp32 add_angle;               //增加角度
	  fp32 motor_angle;             //当前角度
      fp32 motor_angle_set;         //设定角度
	  fp32 motor_angle_goal;
      fp32 min_angle;               //最小角�?
      fp32 max_angle;               //最大角�?
      fp32 to_angle;
	  uint32_t init_time;
	

	void set_angle_limit();
	void init(float angle_lenght,float lenght);
	void init_limit(fp32 max,fp32 min);

};

class moving_t 
{
  public:

	//电机控制�?
	moving_motor_t up_r_motor,up_l_motor;//主抬�?
	moving_motor_t front_r_motor,front_l_motor;//主前�?
	moving_motor_t side_motor;//主横�?
	moving_motor_t yaw_f_motor,yaw_b_motor;//yaw�?
	moving_motor_t pitch_motor,roll_motor;//pitch,roll�?
	moving_motor_t ancillary_up_motor_l,ancillary_up_motor_r,ancillary_front_motor_l,ancillary_front_motor_r;//左右副抬升，前伸
	
	char pump_mode;
	char pump1_mode;
	char pump2_mode;
	uint8_t anci_select_flag;
	uint8_t anci_moving_behavior_flag;
	int anci_time;
	uint8_t anci_flag;

	uint8_t ore_0_used,ore_1_used,ore_2_used;

    float yaw_f_rad,yaw_b_rad,pitch_rad,roll_rad;
	float main_lifting_cm,main_front_cm,main_side_cm;
	float moving_height_fix;
	

	uint8_t sys_behaviour,last_sys_behaviour,last_keyboard_mode,keyboard_mode;
	NewRobotBehaviorMode_e behavior_mode,last_behavior_mode;
	bool_t dm_arm_hold_valid;
	fp32 dm_arm_hold_gimbal_yaw;
	fp32 dm_arm_hold_delta_yaw;

	char task_step_1,last_task_step_1,task_work_1,task_step_2,last_task_step_2,task_work_2;
    int timer1,timer2;
	void NewRobotBehaviorTask();
	void UpdateBehaviorMode();
	void OnBehaviorModeChange();
	void RunBehaviorMode();
	void BehaviorIdle();
	void BehaviorHome();
	void BehaviorDMArmManual();
	void BehaviorDMArmAuto();
	void BehaviorDMArmCalibrate();
	void BehaviorDMArmIKWork();
	bool_t DMArmHasInputCommand();
	void HoldDMArmRelativeToGimbal();
	void ClearDMArmHold();
	void ResetBehaviorSteps();

	void main_init(float front_cm,float side_cm,float lifing_cm,char *step,char value);//初始�?
	void main_moving_position(float front_cm,float side_cm,float lifing_cm,float speed,char *step,char value);//位置控制
	void main_moving_speed(float front_add,float side_add,float lifing_add,char *step,char value);//速度控制
	void main_moving_rc(char*step,char value);
	void main_moving_stop(char*step,char value);
	
//	void ancillary_moving_init(float front_cm,float side_cm,float lifing_cm,float speed,char *step,char value);//初始�?
	void ancillary_moving_position(float front_l_cm,float front_r_cm,float up_l_cm,float up_r_cm,float speed,char *step,char value);//位置控制
	void ancillary_moving_speed(float front_l_add,float front_r_add,float up_l_add,float up_r_add,char *step,char value);//速度控制
	void ancillary_moving_rc(char*step,char value);
	void ancillary_moving_stop(char*step,char value);
    void ancillary_pitch_position(int pitch_l,int pitch_r,char *step,char value);
	
	void arm_moving_init(float yaw_f,float yaw_b,float pitch,float roll,char *step,char value);//初始�?
	void arm_moving_position(float yaw_f_rad,float yaw_b_rad,float pitch_rad,float roll_rad,float speed,char *step,char value);//位置控制
	void arm_moving_speed(float yaw_f_add,float yaw_b_add,float pitch_add,float roll_add,char *step,char value);//速度控制
	void arm_moving_smooth(float yaw_f,float yaw_,float pitch,float roll,char *step,char value);//速度控制
	void arm_moving_rc(char*step,char value);
	void arm_moving_stop(char*step,char value);
	
	void set_pump(char id,char mode,char*step,char value);
	void step_delay(int* timer,int time,char *step,char value);
	void waiting_key(uint8_t key,char *step,char value);
	void step_jump(int jump_to,uint8_t key,char *step,char value);
	
	void Init();
    void Get_info();
	void Get_set();
	
	void main_moving_user(float front_cm,float side_cm,float lifting_cm,float speed);
	void arm_moving_user(float yaw_f_rad,float yaw_b_rad,float pitch_rad,float roll_rad,float speed);
	void moving_motor_position(moving_motor_t moving_motor,float motor_cm,float motor_speed);

	//debug
	int debug_front_l_angle,debug_front_r_angle,debug_lifting_l_angle,debug_lifting_r_angle,debug_side_angle;


};


extern  moving_t *MovingPointer(void);

typedef struct
{
	bool_t fk_inited;
	bool_t raw_valid;
	uint16_t raw12[ARM_DOF];
	fp32 q_joint[ARM_DOF];
	ArmPose_t pose;
	uint8_t last_fk_status;
} DMArmMasterFKDebug_t;

void DMArmMasterSetEncoderRaw12AS5600L(const uint16_t raw12[ARM_DOF]);
void DMArmMasterFKLoadDefaultTemplate(void);
bool_t DMArmMasterFKSetAxisConfig(uint8_t axis_id, const FK_AxisConfig_t *cfg);
bool_t DMArmMasterFKSetDH(const ArmDHParam_t dh[ARM_DOF]);
bool_t DMArmMasterFKGetDebug(DMArmMasterFKDebug_t *debug_out);

#endif
extern  void moving_task(void const *pvParameters);
#ifdef __cplusplus
}

#endif




#endif








