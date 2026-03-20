/**
 * @file arm.h
 * @brief 六轴机械臂对外声明。
 */

#ifndef __ARM_H
#define __ARM_H

#include "struct_typedef.h"

#ifdef __cplusplus
extern "C" {
#endif

// 机械臂自由度数量（固定为6轴）
#define ARM_DOF 6
// 初始化时是否自动打印机械臂参数
#define ARM_AUTO_PRINT_CONFIG_ON_INIT 1
// 参数打印串口端口（1/3/6）
#define ARM_PRINT_UART_PORT 1
// 机械臂控制任务开关：0关闭，1开启
#define ARM_CONTROL_TASK_ENABLE 0
// 六轴达妙关节默认挂载CAN
#define ARM_CONTROL_DM_CAN ON_CAN1
// 六轴达妙关节控制ID（ESC_ID）
#define ARM_CONTROL_DM_J1_RX_ID 0x01
#define ARM_CONTROL_DM_J2_RX_ID 0x02
#define ARM_CONTROL_DM_J3_RX_ID 0x03
#define ARM_CONTROL_DM_J4_RX_ID 0x04
#define ARM_CONTROL_DM_J5_RX_ID 0x05
#define ARM_CONTROL_DM_J6_RX_ID 0x06
// 六轴达妙关节反馈ID（MST_ID）
#define ARM_CONTROL_DM_J1_MST_ID 0x11
#define ARM_CONTROL_DM_J2_MST_ID 0x21
#define ARM_CONTROL_DM_J3_MST_ID 0x31
#define ARM_CONTROL_DM_J4_MST_ID 0x41
#define ARM_CONTROL_DM_J5_MST_ID 0x51
#define ARM_CONTROL_DM_J6_MST_ID 0x61
// 保留给旧逻辑使用（如部分行为层按连续ID遍历）
#define ARM_CONTROL_DM_RX_ID_BASE ARM_CONTROL_DM_J1_RX_ID

// 六轴初始化关节角（单位rad）
#define ARM_HOME_Q1_RAD -0.42f
#define ARM_HOME_Q2_RAD -0.15f
#define ARM_HOME_Q3_RAD 0.0f
#define ARM_HOME_Q4_RAD 0.0f
#define ARM_HOME_Q5_RAD 0.0f
#define ARM_HOME_Q6_RAD 0.0f

// 归位完成判据：单轴误差阈值 + 连续稳定计数
#define ARM_HOME_DONE_JOINT_TOL      0.035f
#define ARM_HOME_DONE_STABLE_COUNT   8U

// 标准DH参数
typedef struct
{
	fp32 a;
	fp32 alpha;
	fp32 d;
	fp32 theta_offset;
} ArmDHParam_t;

// 末端位姿（XYZ + 欧拉角，单位：m/rad）
typedef struct
{
	fp32 x;
	fp32 y;
	fp32 z;
	fp32 roll;
	fp32 pitch;
	fp32 yaw;
} ArmPose_t;

// 机械臂关节指令（逆解输出）
typedef struct
{
	fp32 q[ARM_DOF];
	fp32 gripper;
} ArmCommand_t;

// 单关节“关节角 -> 达妙电机角”映射配置
typedef struct
{
	// 方向：+1 或 -1
	fp32 direction;
	// 减速比：电机角 = 关节角 * reduction_ratio
	fp32 reduction_ratio;
	// 电机零位偏置
	fp32 zero_offset;
	// MIT模式位置限幅（电机侧，rad）
	fp32 pos_min;
	fp32 pos_max;
	// MIT模式速度前馈
	fp32 vel_ff;
	// MIT模式刚度/阻尼
	fp32 kp;
	fp32 kd;
	// MIT模式转矩前馈
	fp32 tor_ff;
} ArmDMJointMap_t;

// 单关节达妙MIT目标
typedef struct
{
	fp32 pos;
	fp32 vel;
	fp32 kp;
	fp32 kd;
	fp32 tor;
} ArmDMMITJointCmd_t;

// 六关节达妙MIT目标
typedef struct
{
	ArmDMMITJointCmd_t joint[ARM_DOF];
	fp32 gripper;
} ArmDMCommand_t;

// IK求解器参数
typedef struct
{
	uint16_t max_iter;
	fp32 pos_tol;
	fp32 ori_tol;
	fp32 damping;
	fp32 step_scale;
	fp32 jacobian_eps;
} ArmIKConfig_t;

#include "arm_kinematics.h"

// 文本输出回调（用于串口打印配置）
typedef void (*ArmPrintCallback_t)(const char *msg);

// 载入默认达妙关节映射参数
void ArmDMJointMapDefault(ArmDMJointMap_t map[ARM_DOF]);

// 机械臂模块初始化（DH、关节限位、IK参数、DM映射）
void ArmInit(void);
// 设置/获取六关节达妙映射参数
void ArmSetDMJointMap(const ArmDMJointMap_t map[ARM_DOF]);
void ArmGetDMJointMap(ArmDMJointMap_t map_out[ARM_DOF]);
// 仅做“关节指令 -> 达妙MIT指令”转换
bool_t ArmConvertJointToDMCommand(const ArmCommand_t *joint_cmd, ArmDMCommand_t *dm_cmd_out);
// 直接执行“位姿逆解 -> 达妙MIT指令”
bool_t ArmSolveIKToDMCommand(const ArmPose_t *target,
						 fp32 gripper_target,
						 const fp32 q_init[ARM_DOF],
						 ArmDMCommand_t *dm_cmd_out,
						 const ArmIKConfig_t *cfg,
						 uint16_t *iter_used);

// 机械臂控制任务相关接口
void ArmControlTaskInit(void);
void ArmControlTask(void);
void ArmControlSetEnable(bool_t enable);
void ArmControlEnableHomeJointsJ1J3(void);
bool_t ArmControlGetEnable(void);
bool_t ArmControlSetTargetPose(const ArmPose_t *target, fp32 gripper_target);
bool_t ArmControlHoldInitJointQ(const fp32 q_target[ARM_DOF], fp32 gripper_target);
bool_t ArmControlHoldJointQ(const fp32 q_target[ARM_DOF], fp32 gripper_target);
void ArmControlSetSeedQ(const fp32 q_seed[ARM_DOF]);
bool_t ArmControlGetJointFeedbackQ(fp32 q_feedback[ARM_DOF]);
bool_t ArmControlCalibrateZeroOffsetFromCurrent(const fp32 q_ref[ARM_DOF]);

// 打印IK、DH、关节限位、DM映射配置
void ArmPrintConfig(ArmPrintCallback_t print_cb);
void ArmPrintToUart1(const char *msg);

#ifdef __cplusplus
}

class DMMotorInstance;

bool_t ArmSolveIKToDMCommand(const ArmPose_t *target,
						 fp32 gripper_target,
						 const fp32 q_init[ARM_DOF],
						 ArmDMCommand_t *dm_cmd_out,
						 const ArmIKConfig_t *cfg,
						 uint16_t *iter_used = 0);

bool_t ArmWriteDMJointMIT(DMMotorInstance *joint_inst[ARM_DOF], const ArmDMCommand_t *dm_cmd);

#endif

#endif

