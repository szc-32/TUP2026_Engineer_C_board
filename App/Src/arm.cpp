/**
 * @file arm.cpp
 * @brief 6-DOF arm kinematics/control glue and calibration persistence.
 */

#include "arm.h"
#include "bsp_flash.h"
#include "bsp_usart.h"
#include "DMmotor.h"

#ifdef abs
#undef abs
#endif

#include <math.h>
#include <stdio.h>
#include <string.h>

// 全局：6轴运动学求解器与IK参数
static ArmKinematics6D g_arm_solver;
static ArmIKConfig_t g_arm_ik_cfg;
// 全局：home关节角（逆解初值和上电默认姿态）
static fp32 g_arm_home_q[ARM_DOF] = {
	ARM_HOME_Q1_RAD,
	ARM_HOME_Q2_RAD,
	ARM_HOME_Q3_RAD,
	ARM_HOME_Q4_RAD,
	ARM_HOME_Q5_RAD,
	ARM_HOME_Q6_RAD,
};

// 全局：关节角到达妙MIT命令的映射参数
static ArmDMJointMap_t g_arm_dm_map[ARM_DOF];

#define ARM_ZERO_OFFSET_FLASH_ADDR    ADDR_FLASH_SECTOR_10
#define ARM_ZERO_OFFSET_FLASH_MAGIC   ((uint32_t)0x41524D5A) // 'ARMZ'
#define ARM_ZERO_OFFSET_FLASH_VERSION ((uint32_t)0x00010002)

typedef struct
{
	uint32_t magic;
	uint32_t version;
	fp32 home_q[ARM_DOF];
	fp32 zero_offset[ARM_DOF];
	uint32_t checksum;
} ArmZeroOffsetFlash_t;

static uint32_t ArmZeroOffsetChecksum(const fp32 home_q[ARM_DOF], const fp32 zero_offset[ARM_DOF])
{
	union
	{
		fp32 f;
		uint32_t u;
	} conv;

	uint32_t sum = ARM_ZERO_OFFSET_FLASH_MAGIC ^ ARM_ZERO_OFFSET_FLASH_VERSION;
	for (uint8_t i = 0; i < ARM_DOF; i++)
	{
		conv.f = home_q[i];
		sum ^= conv.u + (0x9E3779B9u + (sum << 6) + (sum >> 2));
	}
	for (uint8_t i = 0; i < ARM_DOF; i++)
	{
		conv.f = zero_offset[i];
		sum ^= conv.u + (0x9E3779B9u + (sum << 6) + (sum >> 2));
	}
	return sum;
}

static bool_t ArmLoadZeroOffsetFromFlash(void)
{
	ArmZeroOffsetFlash_t data;
	flash_read(ARM_ZERO_OFFSET_FLASH_ADDR,
			  (uint32_t *)&data,
			  (uint32_t)(sizeof(data) / 4));

	if (data.magic != ARM_ZERO_OFFSET_FLASH_MAGIC ||
		data.version != ARM_ZERO_OFFSET_FLASH_VERSION)
	{
		ArmPrintToUart1("[ARM] zero_offset flash empty");
		return FALSE;
	}

	if (data.checksum != ArmZeroOffsetChecksum(data.home_q, data.zero_offset))
	{
		ArmPrintToUart1("[ARM] zero_offset flash checksum mismatch");
		return FALSE;
	}

	memcpy(g_arm_home_q, data.home_q, sizeof(g_arm_home_q));

	for (uint8_t i = 0; i < ARM_DOF; i++)
	{
		g_arm_dm_map[i].zero_offset = data.zero_offset[i];
	}

	ArmPrintToUart1("[ARM] zero_offset loaded from flash");
	return TRUE;
}

static bool_t ArmSaveZeroOffsetToFlash(void)
{
	ArmZeroOffsetFlash_t data;

	data.magic = ARM_ZERO_OFFSET_FLASH_MAGIC;
	data.version = ARM_ZERO_OFFSET_FLASH_VERSION;
	memcpy(data.home_q, g_arm_home_q, sizeof(data.home_q));
	for (uint8_t i = 0; i < ARM_DOF; i++)
	{
		data.zero_offset[i] = g_arm_dm_map[i].zero_offset;
	}
	data.checksum = ArmZeroOffsetChecksum(data.home_q, data.zero_offset);

	flash_erase_address(ARM_ZERO_OFFSET_FLASH_ADDR, 1);
	if (flash_write_single_address(ARM_ZERO_OFFSET_FLASH_ADDR,
					   (uint32_t *)&data,
					   (uint32_t)(sizeof(data) / 4)) != 0)
	{
		ArmPrintToUart1("[ARM] zero_offset flash save failed");
		return FALSE;
	}

	ArmPrintToUart1("[ARM] zero_offset saved to flash");
	return TRUE;
}

// 机械臂运行时上下文（控制任务专用）
typedef struct
{
	// 是否已完成内部电机与参数初始化
	bool_t inited;
	// 控制任务使能
	bool_t enable;
	// 当前目标末端位姿
	ArmPose_t target_pose;
	// 夹爪目标透传
	fp32 gripper_target;
	// IK迭代初值（每周期更新为上一帧结果）
	fp32 q_seed[ARM_DOF];
	// 六个达妙关节实例
	DMMotorInstance joint_motor[ARM_DOF];
	// 六个达妙关节初始化配置
	DM_Motor_Setting_t joint_setting[ARM_DOF];
} ArmControlRuntime_t;

static ArmControlRuntime_t g_arm_ctrl = {
	FALSE,
#if ARM_CONTROL_TASK_ENABLE
	TRUE,
#else
	FALSE,
#endif
	{0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f},
	0.0f,
	{0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f},
};
extern "C" volatile uint8_t g_arm_ctrl_enable_dbg =
#if ARM_CONTROL_TASK_ENABLE
	1U;
#else
	0U;
#endif
extern "C" volatile uint32_t g_arm_task_tick_dbg = 0U;
extern "C" volatile uint32_t g_arm_task_enable_true_tick_dbg = 0U;
extern "C" volatile uint32_t g_arm_j2_mit_ctrl_tx_cnt_dbg = 0U;
extern "C" volatile uint8_t g_arm_set_enable_arg_dbg = 0U;
extern "C" volatile uint8_t g_arm_set_enable_src_dbg = 0U;
extern "C" volatile uint8_t g_arm_set_enable_arg_norm_dbg = 0U;
volatile uint8_t g_arm_j1_state_dbg = 0U;
volatile uint8_t g_arm_j2_state_dbg = 0U;
volatile fp32 g_arm_j1_pos_dbg = 0.0f;
volatile fp32 g_arm_j2_pos_dbg = 0.0f;

// Whether arm DM joints have been explicitly enabled in current enable session.
static bool_t g_arm_dm_enabled = FALSE;
static bool_t g_arm_init_joint_enabled = FALSE;
static uint16_t g_arm_enable_refresh_tick = 0;

static const char *g_arm_joint_name[ARM_DOF] = {
	"arm_j1",
	"arm_j2",
	"arm_j3",
	"arm_j4",
	"arm_j5",
	"arm_j6",
};

//机械臂DH参数默认值
static const ArmDHParam_t g_arm_dh_default[ARM_DOF] = {
	{0.0f, 1.5707963f, 0.072f, 0.0f},
	{0.39f, 0.0f, 0.0f, 1.5707963f},
	{0.0895f, 1.5707963f, 0.0f, 3.1415926f},
	{0.0f, 0.0f, 0.371f, 0.0f},
	{0.0f, 1.5707963f, 0.0f, 0.0f},
	{0.0f, 1.5707963f, 0.0f, 0.0f},
};

static const fp32 g_arm_q_min_default[ARM_DOF] = {
	-3.1415926f, -0.9467733f, 0.3f, -1.5707963f, -1.5707963f, 0.0f
};

static const fp32 g_arm_q_max_default[ARM_DOF] = {
	3.1415926f, 1.5707963f, 3.5f, 1.5707963f, 1.5707963f, 3.1415926f
};

static const ArmDMJointMap_t g_arm_dm_map_default[ARM_DOF] = {
	{1.0f, 1.0f, 0.0f, -3.1415926f, 3.1415926f, 0.0f, 30.0f, 5.0f, 0.0f},
	{1.0f, 1.0f, 0.0f, -0.9467733f, 1.5707963f, 0.0f, 25.0f, 2.0f, 0.0f},
	{1.0f, 1.0f, 0.0f, 0.3f,        3.5f,       0.0f, 4.0f, 1.0f, 0.0f},
	{1.0f, 1.0f, 0.0f, -1.5707963f, 1.5707963f, 0.0f, 15.0f, 1.0f, 0.0f},
	{1.0f, 1.0f, 0.0f, -1.5707963f, 1.5707963f, 0.0f, 15.0f, 1.0f, 0.0f},
	{1.0f, 1.0f, 0.0f, 0.0f, 3.1415926f, 0.0f, 15.0f, 1.0f, 0.0f},
};

static const uint16_t g_arm_dm_rx_id_default[ARM_DOF] = {
	ARM_CONTROL_DM_J1_RX_ID,
	ARM_CONTROL_DM_J2_RX_ID,
	ARM_CONTROL_DM_J3_RX_ID,
	ARM_CONTROL_DM_J4_RX_ID,
	ARM_CONTROL_DM_J5_RX_ID,
	ARM_CONTROL_DM_J6_RX_ID,
};

static const uint16_t g_arm_dm_mst_id_default[ARM_DOF] = {
	ARM_CONTROL_DM_J1_MST_ID,
	ARM_CONTROL_DM_J2_MST_ID,
	ARM_CONTROL_DM_J3_MST_ID,
	ARM_CONTROL_DM_J4_MST_ID,
	ARM_CONTROL_DM_J5_MST_ID,
	ARM_CONTROL_DM_J6_MST_ID,
};

// 关节值通用限幅
static fp32 ClampValue(fp32 val, fp32 low, fp32 high)
{
	if (val < low)
	{
		return low;
	}
	if (val > high)
	{
		return high;
	}
	return val;
}

// 根据CAN编号获取CAN句柄
static hcan_t *ArmGetCanHandle(CAN_NAME_e can_id)
{
	return (can_id == ON_CAN1) ? &hcan1 : &hcan2;
}

static void ArmControlEnsureInit(void);
static void ArmUpdateJoint12DebugMirror(void);

static void ArmUpdateJoint12DebugMirror(void)
{
	if (g_arm_ctrl.inited == FALSE)
	{
		return;
	}

	DMmotor_fbpara_t *j1 = g_arm_ctrl.joint_motor[0].GetDMMotorMeasure();
	DMmotor_fbpara_t *j2 = g_arm_ctrl.joint_motor[1].GetDMMotorMeasure();

	if (j1 != 0)
	{
		g_arm_j1_state_dbg = (uint8_t)j1->state;
		g_arm_j1_pos_dbg = j1->pos;
	}

	if (j2 != 0)
	{
		g_arm_j2_state_dbg = (uint8_t)j2->state;
		g_arm_j2_pos_dbg = j2->pos;
	}
}

// 流程说明：按“前N轴”发送一次达妙使能帧。
// 用途：支持分阶段启用（例如 INIT 先启 J1~J4），避免一次性拉起全部关节。
static void ArmEnableDMJointRange(uint8_t joint_count)
{
	if (joint_count > ARM_DOF)
	{
		joint_count = ARM_DOF;
	}

	ArmControlEnsureInit();

	for (uint8_t i = 0; i < joint_count; i++)
	{
		DM_motor_t *motor = &g_arm_ctrl.joint_motor[i].dm_motor[Motor1];
		hcan_t *hcan = ArmGetCanHandle((CAN_NAME_e)g_arm_ctrl.joint_setting[i].can_id);
		dm_motor_clear_err(hcan, motor);
		enable_motor_mode(hcan, motor->id, MIT_MODE);
	}
}

// Send a one-shot enable frame for all six DM joints when arm control becomes active.
// 流程说明：控制任务正常运行时，确保六轴都已进入 MIT 模式。
static void ArmEnsureDMEnabled(void)
{
	if (g_arm_dm_enabled == TRUE)
	{
		return;
	}

	ArmEnableDMJointRange(ARM_DOF);
	g_arm_dm_enabled = TRUE;
	g_arm_init_joint_enabled = TRUE;
}

// 延迟初始化：仅在控制任务首次运行时注册六个关节达妙实例
// 流程说明：首次进入控制链时完成“参数加载 + 电机实例注册 + 初值准备”。
// 这样可避免上电初始化阶段占用过多时间，也便于分模式调试。
static void ArmControlEnsureInit(void)
{
	if (g_arm_ctrl.inited == TRUE)
	{
		return;
	}

	ArmInit();

	const fp32 *home_q = ArmGetHomeQ();
	memcpy(g_arm_ctrl.q_seed, home_q, sizeof(g_arm_ctrl.q_seed));
	g_arm_solver.ForwardKinematics(home_q, &g_arm_ctrl.target_pose);

	for (uint8_t i = 0; i < ARM_DOF; i++)
	{
		memset(&g_arm_ctrl.joint_setting[i], 0, sizeof(DM_Motor_Setting_t));
		g_arm_ctrl.joint_setting[i].motor_name = g_arm_joint_name[i];
		g_arm_ctrl.joint_setting[i].can_id = ARM_CONTROL_DM_CAN;
		g_arm_ctrl.joint_setting[i].rx_id = g_arm_dm_rx_id_default[i];
		g_arm_ctrl.joint_setting[i].mst_id = g_arm_dm_mst_id_default[i];
		g_arm_ctrl.joint_setting[i].direction = POSITIVE_DIRECT;
		g_arm_ctrl.joint_setting[i].dm_control_type = mit_mode;

		g_arm_ctrl.joint_motor[i].DMMotorInit(&g_arm_ctrl.joint_setting[i]);
		g_arm_ctrl.joint_motor[i].DM_motor_para_init(&g_arm_ctrl.joint_motor[i].dm_motor[Motor1]);
	}

	g_arm_ctrl.inited = TRUE;
	ArmUpdateJoint12DebugMirror();
}

// 加载达妙映射默认参数
void ArmDMJointMapDefault(ArmDMJointMap_t map[ARM_DOF])
{
	if (map == 0)
	{
		return;
	}
	memcpy(map, g_arm_dm_map_default, sizeof(g_arm_dm_map_default));
}

void ArmInit(void)
{
	// 初始化运动学与逆解参数
	g_arm_solver.SetDH(g_arm_dh_default);
	g_arm_solver.SetJointLimit(g_arm_q_min_default, g_arm_q_max_default);
	ArmIKDefaultConfig(&g_arm_ik_cfg);
	// 初始化关节->达妙映射参数
	ArmDMJointMapDefault(g_arm_dm_map);
	ArmLoadZeroOffsetFromFlash();
#if ARM_AUTO_PRINT_CONFIG_ON_INIT
	ArmPrintConfig(ArmPrintToUart1);
#endif
}

ArmKinematics6D *ArmGetSolver(void)
{
	return &g_arm_solver;
}

ArmIKConfig_t *ArmGetIKConfig(void)
{
	return &g_arm_ik_cfg;
}

const fp32 *ArmGetHomeQ(void)
{
	return g_arm_home_q;
}

void ArmSetHomeQ(const fp32 home_q[ARM_DOF])
{
	if (home_q == 0)
	{
		return;
	}

	memcpy(g_arm_home_q, home_q, sizeof(g_arm_home_q));
}

void ArmSetDMJointMap(const ArmDMJointMap_t map[ARM_DOF])
{
	if (map == 0)
	{
		return;
	}
	memcpy(g_arm_dm_map, map, sizeof(g_arm_dm_map));
}

void ArmGetDMJointMap(ArmDMJointMap_t map_out[ARM_DOF])
{
	if (map_out == 0)
	{
		return;
	}
	memcpy(map_out, g_arm_dm_map, sizeof(g_arm_dm_map));
}

bool_t ArmConvertJointToDMCommand(const ArmCommand_t *joint_cmd, ArmDMCommand_t *dm_cmd_out)
{
	if (joint_cmd == 0 || dm_cmd_out == 0)
	{
		return FALSE;
	}

	for (uint8_t i = 0; i < ARM_DOF; i++)
	{
		const ArmDMJointMap_t *map = &g_arm_dm_map[i];
		const fp32 q_joint = joint_cmd->q[i];
		// 电机角 = 方向 * 减速比 * 关节角 + 零位偏置
		const fp32 q_motor = map->direction * map->reduction_ratio * q_joint + map->zero_offset;

		// 位置限幅并写入MIT控制参数
		dm_cmd_out->joint[i].pos = ClampValue(q_motor, map->pos_min, map->pos_max);
		dm_cmd_out->joint[i].vel = map->vel_ff;
		dm_cmd_out->joint[i].kp = map->kp;
		dm_cmd_out->joint[i].kd = map->kd;
		dm_cmd_out->joint[i].tor = map->tor_ff;
	}

	dm_cmd_out->gripper = joint_cmd->gripper;
	return TRUE;
}

// 位姿逆解后直接得到达妙MIT控制命令
// 流程说明：target_pose -> IK 关节角 -> DM MIT 命令，一步给出可下发指令。
bool_t ArmSolveIKToDMCommand(const ArmPose_t *target,
						 fp32 gripper_target,
						 const fp32 q_init[ARM_DOF],
						 ArmDMCommand_t *dm_cmd_out,
						 const ArmIKConfig_t *cfg,
						 uint16_t *iter_used)
{
	if (target == 0 || q_init == 0 || dm_cmd_out == 0)
	{
		return FALSE;
	}

	ArmCommand_t joint_cmd;
	const bool_t ik_ok = g_arm_solver.SolveIKWithGripper(target, gripper_target, q_init, &joint_cmd, cfg, iter_used);
	ArmConvertJointToDMCommand(&joint_cmd, dm_cmd_out);
	return ik_ok;
}

// 将六轴达妙命令写入各轴控制缓存（不在此函数内发CAN）
bool_t ArmWriteDMJointMIT(DMMotorInstance *joint_inst[ARM_DOF], const ArmDMCommand_t *dm_cmd)
{
	if (joint_inst == 0 || dm_cmd == 0)
	{
		return FALSE;
	}

	for (uint8_t i = 0; i < ARM_DOF; i++)
	{
		if (joint_inst[i] == 0)
		{
			return FALSE;
		}

		DM_motor_t *motor = &joint_inst[i]->dm_motor[Motor1];
		motor->dm_ctrl_set.mode = mit_mode;
		motor->dm_ctrl_set.pos_set = dm_cmd->joint[i].pos;
		motor->dm_ctrl_set.vel_set = dm_cmd->joint[i].vel;
		motor->dm_ctrl_set.kp_set = dm_cmd->joint[i].kp;
		motor->dm_ctrl_set.kd_set = dm_cmd->joint[i].kd;
		motor->dm_ctrl_set.tor_set = dm_cmd->joint[i].tor;
	}

	return TRUE;
}

// 手动触发控制任务初始化
void ArmControlTaskInit(void)
{
	g_arm_ctrl.inited = FALSE;
	ArmControlEnsureInit();
}

void ArmControlSetEnable(bool_t enable)
{
	const uint8_t enable_raw = (uint8_t)enable;
	const bool_t enable_norm = (enable_raw != 0U) ? TRUE : FALSE;
	g_arm_set_enable_arg_dbg = enable_raw;
	g_arm_set_enable_arg_norm_dbg = (uint8_t)enable_norm;
	if (g_arm_set_enable_src_dbg == 0U)
	{
		// 0 means caller did not tag source before invocation.
		g_arm_set_enable_src_dbg = 3U;
	}
	// 仅切换使能状态，不重置目标和初值
	g_arm_ctrl.enable = enable_norm;
	g_arm_ctrl_enable_dbg = (uint8_t)enable_norm;

	if (enable_norm == FALSE)
	{
		// Re-send enable when next time arm is enabled.
		g_arm_dm_enabled = FALSE;
		g_arm_init_joint_enabled = FALSE;
		g_arm_enable_refresh_tick = 0;
	}
}

// HOME 阶段专用：直接使能 J1~J3，进入行为函数即触发，不再走额外层级。
void ArmControlEnableHomeJointsJ1J3(void)
{
	if (g_arm_init_joint_enabled == TRUE)
	{
		return;
	}

	g_arm_ctrl.enable = TRUE;
	g_arm_ctrl_enable_dbg = 1U;
	ArmEnableDMJointRange(3);
	g_arm_init_joint_enabled = TRUE;
}

bool_t ArmControlGetEnable(void)
{
	return g_arm_ctrl.enable;
}

bool_t ArmControlSetTargetPose(const ArmPose_t *target, fp32 gripper_target)
{
	if (target == 0)
	{
		return FALSE;
	}

	g_arm_ctrl.target_pose = *target;
	g_arm_ctrl.gripper_target = gripper_target;
	return TRUE;
}

// 流程说明：INIT/HOME 阶段关节控制。
// 1) 保留当前 seed 里的全轴值，避免未参与初始化的轴被误改。
// 2) 仅更新 J1~J4 到目标角，并仅对这四轴下发 MIT 命令。
// 3) 更新 seed/target_pose，保证后续 IK 初值连续。
bool_t ArmControlHoldInitJointQ(const fp32 q_target[ARM_DOF], fp32 gripper_target)
{
	if (q_target == 0)
	{
		return FALSE;
	}

	if (g_arm_ctrl.enable == FALSE)
	{
		return FALSE;
	}

	ArmControlEnsureInit();

	fp32 q_min[ARM_DOF];
	fp32 q_max[ARM_DOF];
	g_arm_solver.GetJointLimit(q_min, q_max);

	ArmCommand_t joint_cmd;
	for (uint8_t i = 0; i < ARM_DOF; i++)
	{
		joint_cmd.q[i] = g_arm_ctrl.q_seed[i];
	}
	for (uint8_t i = 0; i < 3; i++)
	{
		joint_cmd.q[i] = ClampValue(q_target[i], q_min[i], q_max[i]);
	}
	joint_cmd.gripper = gripper_target;

	ArmDMCommand_t dm_cmd;
	if (ArmConvertJointToDMCommand(&joint_cmd, &dm_cmd) == FALSE)
	{
		return FALSE;
	}

	for (uint8_t i = 0; i < 3; i++)
	{
		DM_motor_t *motor = &g_arm_ctrl.joint_motor[i].dm_motor[Motor1];
		hcan_t *hcan = ArmGetCanHandle((CAN_NAME_e)g_arm_ctrl.joint_setting[i].can_id);
		motor->dm_ctrl_set.mode = mit_mode;
		motor->dm_ctrl_set.pos_set = dm_cmd.joint[i].pos;
		motor->dm_ctrl_set.vel_set = dm_cmd.joint[i].vel;
		motor->dm_ctrl_set.kp_set = dm_cmd.joint[i].kp;
		motor->dm_ctrl_set.kd_set = dm_cmd.joint[i].kd;
		motor->dm_ctrl_set.tor_set = dm_cmd.joint[i].tor;
		g_arm_ctrl.joint_motor[i].DMMotorControl(hcan, motor);
	}

	g_arm_ctrl.gripper_target = gripper_target;
	memcpy(g_arm_ctrl.q_seed, joint_cmd.q, sizeof(g_arm_ctrl.q_seed));
	g_arm_solver.ForwardKinematics(joint_cmd.q, &g_arm_ctrl.target_pose);

	return TRUE;
}

// 流程说明：通用六轴关节位置保持。
// 该接口会使能六轴并完整下发 J1~J6 MIT 指令，适用于正常运行阶段。
bool_t ArmControlHoldJointQ(const fp32 q_target[ARM_DOF], fp32 gripper_target)
{
	if (q_target == 0)
	{
		return FALSE;
	}

	if (g_arm_ctrl.enable == FALSE)
	{
		return FALSE;
	}

	ArmControlEnsureInit();
	ArmEnsureDMEnabled();

	fp32 q_min[ARM_DOF];
	fp32 q_max[ARM_DOF];
	g_arm_solver.GetJointLimit(q_min, q_max);

	ArmCommand_t joint_cmd;
	for (uint8_t i = 0; i < ARM_DOF; i++)
	{
		joint_cmd.q[i] = ClampValue(q_target[i], q_min[i], q_max[i]);
	}
	joint_cmd.gripper = gripper_target;

	ArmDMCommand_t dm_cmd;
	if (ArmConvertJointToDMCommand(&joint_cmd, &dm_cmd) == FALSE)
	{
		return FALSE;
	}

	DMMotorInstance *joint_inst[ARM_DOF];
	for (uint8_t i = 0; i < ARM_DOF; i++)
	{
		joint_inst[i] = &g_arm_ctrl.joint_motor[i];
	}

	if (ArmWriteDMJointMIT(joint_inst, &dm_cmd) == FALSE)
	{
		return FALSE;
	}

	for (uint8_t i = 0; i < ARM_DOF; i++)
	{
		DM_motor_t *motor = &g_arm_ctrl.joint_motor[i].dm_motor[Motor1];
		hcan_t *hcan = ArmGetCanHandle((CAN_NAME_e)g_arm_ctrl.joint_setting[i].can_id);
		if (i == 1U)
		{
			g_arm_j2_mit_ctrl_tx_cnt_dbg++;
		}
		g_arm_ctrl.joint_motor[i].DMMotorControl(hcan, motor);
	}

	g_arm_ctrl.gripper_target = gripper_target;
	memcpy(g_arm_ctrl.q_seed, joint_cmd.q, sizeof(g_arm_ctrl.q_seed));
	g_arm_solver.ForwardKinematics(joint_cmd.q, &g_arm_ctrl.target_pose);

	return TRUE;
}

void ArmControlSetSeedQ(const fp32 q_seed[ARM_DOF])
{
	if (q_seed == 0)
	{
		return;
	}
	memcpy(g_arm_ctrl.q_seed, q_seed, sizeof(g_arm_ctrl.q_seed));
}

bool_t ArmControlGetJointFeedbackQ(fp32 q_feedback[ARM_DOF])
{
	if (q_feedback == 0)
	{
		return FALSE;
	}

	ArmControlEnsureInit();

	fp32 q_min[ARM_DOF];
	fp32 q_max[ARM_DOF];
	g_arm_solver.GetJointLimit(q_min, q_max);

	for (uint8_t i = 0; i < ARM_DOF; i++)
	{
		DMmotor_fbpara_t *measure = g_arm_ctrl.joint_motor[i].GetDMMotorMeasure();
		const ArmDMJointMap_t *map = &g_arm_dm_map[i];
		const fp32 denom = map->direction * map->reduction_ratio;

		if (measure == 0)
		{
			return FALSE;
		}
		if (denom < 0.0001f && denom > -0.0001f)
		{
			return FALSE;
		}

		const fp32 q_joint = (measure->pos - map->zero_offset) / denom;
		q_feedback[i] = ClampValue(q_joint, q_min[i], q_max[i]);
	}

	return TRUE;
}

// 流程说明：以当前反馈位置反算 zero_offset，并写入 Flash。
// 用途：完成机械臂零位标定的持久化，重启后可直接恢复映射参数。
bool_t ArmControlCalibrateZeroOffsetFromCurrent(const fp32 q_ref[ARM_DOF])
{
	const fp32 *q_target = q_ref;
	ArmDMJointMap_t map_new[ARM_DOF];

	ArmControlEnsureInit();

	if (q_target == 0)
	{
		q_target = ArmGetHomeQ();
	}

	memcpy(map_new, g_arm_dm_map, sizeof(map_new));

	for (uint8_t i = 0; i < ARM_DOF; i++)
	{
		DMmotor_fbpara_t *measure = g_arm_ctrl.joint_motor[i].GetDMMotorMeasure();
		const fp32 denom = map_new[i].direction * map_new[i].reduction_ratio;

		if (measure == 0)
		{
			ArmPrintToUart1("[ARM] calibrate zero_offset failed: null feedback");
			return FALSE;
		}

		if ((denom < 0.0001f) && (denom > -0.0001f))
		{
			ArmPrintToUart1("[ARM] calibrate zero_offset failed: invalid direction/ratio");
			return FALSE;
		}

		// pos = direction * ratio * q_joint + zero_offset
		map_new[i].zero_offset = measure->pos - denom * q_target[i];
	}

	ArmSetDMJointMap(map_new);
	ArmControlSetSeedQ(q_target);
	g_arm_solver.ForwardKinematics(q_target, &g_arm_ctrl.target_pose);
	ArmSaveZeroOffsetToFlash();
	ArmPrintToUart1("[ARM] calibrate zero_offset done");
	ArmPrintConfig(ArmPrintToUart1);

	return TRUE;
}

// 机械臂控制主循环（1ms级）
// 执行链：target_pose -> IK -> joint_cmd -> DM MIT -> CAN 下发 -> seed 更新。
// 其中 seed 连续更新是数值 IK 稳定收敛的关键。
void ArmControlTask(void)
{
	g_arm_task_tick_dbg++;

	if (g_arm_ctrl.inited == TRUE)
	{
		ArmUpdateJoint12DebugMirror();
	}

	if (g_arm_ctrl.enable == FALSE)
	{
		return;
	}

	g_arm_task_enable_true_tick_dbg++;

	// 确保首次进入时完成达妙关节实例注册
	ArmControlEnsureInit();
	ArmEnsureDMEnabled();

	// 周期性补发使能帧，避免单次丢帧导致个别关节长期未进MIT模式。
	if (g_arm_enable_refresh_tick < 50U)
	{
		g_arm_enable_refresh_tick++;
	}
	else
	{
		g_arm_enable_refresh_tick = 0;
		ArmEnableDMJointRange(ARM_DOF);
	}

	ArmCommand_t joint_cmd;
	uint16_t iter_used = 0;
	// 执行逆解：target_pose -> joint_cmd
	g_arm_solver.SolveIKWithGripper(
		&g_arm_ctrl.target_pose,
		g_arm_ctrl.gripper_target,
		g_arm_ctrl.q_seed,
		&joint_cmd,
		&g_arm_ik_cfg,
		&iter_used);

	ArmDMCommand_t dm_cmd;
	// 关节角转换为达妙MIT命令
	ArmConvertJointToDMCommand(&joint_cmd, &dm_cmd);

	DMMotorInstance *joint_inst[ARM_DOF];
	for (uint8_t i = 0; i < ARM_DOF; i++)
	{
		joint_inst[i] = &g_arm_ctrl.joint_motor[i];
	}

	if (ArmWriteDMJointMIT(joint_inst, &dm_cmd) == TRUE)
	{
		// 将每轴MIT命令下发到对应CAN
		for (uint8_t i = 0; i < ARM_DOF; i++)
		{
			DM_motor_t *motor = &g_arm_ctrl.joint_motor[i].dm_motor[Motor1];
			hcan_t *hcan = ArmGetCanHandle((CAN_NAME_e)g_arm_ctrl.joint_setting[i].can_id);
			if (i == 1U)
			{
				g_arm_j2_mit_ctrl_tx_cnt_dbg++;
			}
			if (i == 1U)
			{
				g_arm_j2_mit_ctrl_tx_cnt_dbg++;
			}
			g_arm_ctrl.joint_motor[i].DMMotorControl(hcan, motor);
		}
	}

	ArmUpdateJoint12DebugMirror();

	// 将本次解作为下一次迭代初值，提高收敛速度和稳定性
	memcpy(g_arm_ctrl.q_seed, joint_cmd.q, sizeof(g_arm_ctrl.q_seed));
}

// 打印当前IK、DH和达妙映射参数
void ArmPrintConfig(ArmPrintCallback_t print_cb)
{
	if (print_cb == 0)
	{
		return;
	}

	char line[160];
	ArmDHParam_t dh_now[ARM_DOF];
	fp32 qmin_now[ARM_DOF];
	fp32 qmax_now[ARM_DOF];

	g_arm_solver.GetDH(dh_now);
	g_arm_solver.GetJointLimit(qmin_now, qmax_now);

	snprintf(line, sizeof(line), "[ARM] IK cfg: iter=%u pos_tol=%.6f ori_tol=%.6f damp=%.6f step=%.6f eps=%.6f",
		(unsigned)g_arm_ik_cfg.max_iter,
		g_arm_ik_cfg.pos_tol,
		g_arm_ik_cfg.ori_tol,
		g_arm_ik_cfg.damping,
		g_arm_ik_cfg.step_scale,
		g_arm_ik_cfg.jacobian_eps);
	print_cb(line);

	for (uint8_t i = 0; i < ARM_DOF; i++)
	{
		snprintf(line, sizeof(line),
			"[ARM] DH[%u]: a=%.6f alpha=%.6f d=%.6f offset=%.6f qlim=[%.6f, %.6f] home=%.6f",
			(unsigned)(i + 1),
			dh_now[i].a,
			dh_now[i].alpha,
			dh_now[i].d,
			dh_now[i].theta_offset,
			qmin_now[i],
			qmax_now[i],
			g_arm_home_q[i]);
		print_cb(line);

		snprintf(line, sizeof(line),
			"[ARM] DM[%u]: dir=%.3f ratio=%.3f offset=%.6f pos=[%.6f, %.6f] kp=%.3f kd=%.3f",
			(unsigned)(i + 1),
			g_arm_dm_map[i].direction,
			g_arm_dm_map[i].reduction_ratio,
			g_arm_dm_map[i].zero_offset,
			g_arm_dm_map[i].pos_min,
			g_arm_dm_map[i].pos_max,
			g_arm_dm_map[i].kp,
			g_arm_dm_map[i].kd);
		print_cb(line);
	}
}

void ArmPrintToUart1(const char *msg)
{
	if (msg == 0)
	{
		return;
	}

	UART_HandleTypeDef *print_uart = &huart1;
#if ARM_PRINT_UART_PORT == 3
	print_uart = &huart3;
#elif ARM_PRINT_UART_PORT == 6
	print_uart = &huart6;
#endif

	const uint16_t len = (uint16_t)strlen(msg);
	if (len > 0U)
	{
		USARTSend(print_uart, (uint8_t *)msg, len, USART_TRANSFER_BLOCKING);
	}

	uint8_t line_end[2] = {'\r', '\n'};
	USARTSend(print_uart, line_end, 2, USART_TRANSFER_BLOCKING);
}
