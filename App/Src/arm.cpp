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
	{1.0f, 1.0f, 0.0f, -3.1415926f, 3.1415926f, 0.0f, 40.0f, 1.0f, 0.0f},
	{1.0f, 1.0f, 0.0f, -0.9467733f, 1.5707963f, 0.0f, 40.0f, 1.0f, 0.0f},
	{1.0f, 1.0f, 0.0f, 0.3f, 3.5f, 1.0f, 4.0f, 0.5f, 0.0f},
	{1.0f, 1.0f, 0.0f, -1.5707963f, 1.5707963f, 0.0f, 40.0f, 1.0f, 0.0f},
	{1.0f, 1.0f, 0.0f, -1.5707963f, 1.5707963f, 0.0f, 40.0f, 1.0f, 0.0f},
	{1.0f, 1.0f, 0.0f, 0.0f, 3.1415926f, 0.0f, 40.0f, 1.0f, 0.0f},
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

// 延迟初始化：仅在控制任务首次运行时注册六个关节达妙实例
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
		g_arm_ctrl.joint_setting[i].rx_id = ARM_CONTROL_DM_RX_ID_BASE + i;
		g_arm_ctrl.joint_setting[i].mst_id = ARM_CONTROL_DM_MST_ID_BASE + i;
		g_arm_ctrl.joint_setting[i].direction = POSITIVE_DIRECT;
		g_arm_ctrl.joint_setting[i].dm_control_type = mit_mode;

		g_arm_ctrl.joint_motor[i].DMMotorInit(&g_arm_ctrl.joint_setting[i]);
		g_arm_ctrl.joint_motor[i].DM_motor_para_init(&g_arm_ctrl.joint_motor[i].dm_motor[Motor1]);
	}

	g_arm_ctrl.inited = TRUE;
}

// 4x4单位矩阵
static void Mat4Identity(fp32 T[4][4])
{
	memset(T, 0, sizeof(fp32) * 16);
	T[0][0] = 1.0f;
	T[1][1] = 1.0f;
	T[2][2] = 1.0f;
	T[3][3] = 1.0f;
}

// 4x4矩阵乘法
static void Mat4Mul(const fp32 A[4][4], const fp32 B[4][4], fp32 C[4][4])
{
	fp32 tmp[4][4];
	for (uint8_t i = 0; i < 4; i++)
	{
		for (uint8_t j = 0; j < 4; j++)
		{
			tmp[i][j] = 0.0f;
			for (uint8_t k = 0; k < 4; k++)
			{
				tmp[i][j] += A[i][k] * B[k][j];
			}
		}
	}
	memcpy(C, tmp, sizeof(tmp));
}

// 根据标准DH参数构造单关节齐次变换矩阵
static void BuildStdDHTransform(fp32 a, fp32 alpha, fp32 d, fp32 theta, fp32 A[4][4])
{
	const fp32 ct = cosf(theta);
	const fp32 st = sinf(theta);
	const fp32 ca = cosf(alpha);
	const fp32 sa = sinf(alpha);

	A[0][0] = ct;
	A[0][1] = -st * ca;
	A[0][2] = st * sa;
	A[0][3] = a * ct;

	A[1][0] = st;
	A[1][1] = ct * ca;
	A[1][2] = -ct * sa;
	A[1][3] = a * st;

	A[2][0] = 0.0f;
	A[2][1] = sa;
	A[2][2] = ca;
	A[2][3] = d;

	A[3][0] = 0.0f;
	A[3][1] = 0.0f;
	A[3][2] = 0.0f;
	A[3][3] = 1.0f;
}

// 位姿（XYZ+RPY）转齐次变换矩阵
static void PoseToTransform(const ArmPose_t *pose, fp32 T[4][4])
{
	const fp32 cr = cosf(pose->roll);
	const fp32 sr = sinf(pose->roll);
	const fp32 cp = cosf(pose->pitch);
	const fp32 sp = sinf(pose->pitch);
	const fp32 cy = cosf(pose->yaw);
	const fp32 sy = sinf(pose->yaw);

	Mat4Identity(T);

	T[0][0] = cy * cp;
	T[0][1] = cy * sp * sr - sy * cr;
	T[0][2] = cy * sp * cr + sy * sr;

	T[1][0] = sy * cp;
	T[1][1] = sy * sp * sr + cy * cr;
	T[1][2] = sy * sp * cr - cy * sr;

	T[2][0] = -sp;
	T[2][1] = cp * sr;
	T[2][2] = cp * cr;

	T[0][3] = pose->x;
	T[1][3] = pose->y;
	T[2][3] = pose->z;
}

// 齐次变换矩阵转位姿（XYZ+RPY）
static void TransformToPose(const fp32 T[4][4], ArmPose_t *pose)
{
	pose->x = T[0][3];
	pose->y = T[1][3];
	pose->z = T[2][3];

	pose->pitch = asinf(-T[2][0]);
	if (fabsf(cosf(pose->pitch)) > 1e-6f)
	{
		pose->roll = atan2f(T[2][1], T[2][2]);
		pose->yaw = atan2f(T[1][0], T[0][0]);
	}
	else
	{
		pose->roll = 0.0f;
		pose->yaw = atan2f(-T[0][1], T[1][1]);
	}
}

// 6维向量二范数
static fp32 VecNorm6(const fp32 v[6])
{
	fp32 s = 0.0f;
	for (uint8_t i = 0; i < 6; i++)
	{
		s += v[i] * v[i];
	}
	return sqrtf(s);
}

// 3维向量二范数
static fp32 VecNorm3(const fp32 v[3])
{
	return sqrtf(v[0] * v[0] + v[1] * v[1] + v[2] * v[2]);
}

// 6x6矩阵求逆（高斯-约旦）
static bool_t Inverse6x6(const fp32 M[6][6], fp32 invM[6][6])
{
	fp32 aug[6][12];
	for (uint8_t i = 0; i < 6; i++)
	{
		for (uint8_t j = 0; j < 6; j++)
		{
			aug[i][j] = M[i][j];
			aug[i][j + 6] = (i == j) ? 1.0f : 0.0f;
		}
	}

	for (uint8_t col = 0; col < 6; col++)
	{
		uint8_t pivot = col;
		fp32 max_abs = fabsf(aug[col][col]);
		for (uint8_t r = (uint8_t)(col + 1); r < 6; r++)
		{
			const fp32 val = fabsf(aug[r][col]);
			if (val > max_abs)
			{
				max_abs = val;
				pivot = r;
			}
		}

		if (max_abs < 1e-9f)
		{
			return FALSE;
		}

		if (pivot != col)
		{
			for (uint8_t k = 0; k < 12; k++)
			{
				const fp32 tmp = aug[col][k];
				aug[col][k] = aug[pivot][k];
				aug[pivot][k] = tmp;
			}
		}

		const fp32 diag = aug[col][col];
		for (uint8_t k = 0; k < 12; k++)
		{
			aug[col][k] /= diag;
		}

		for (uint8_t r = 0; r < 6; r++)
		{
			if (r == col)
			{
				continue;
			}
			const fp32 factor = aug[r][col];
			for (uint8_t k = 0; k < 12; k++)
			{
				aug[r][k] -= factor * aug[col][k];
			}
		}
	}

	for (uint8_t i = 0; i < 6; i++)
	{
		for (uint8_t j = 0; j < 6; j++)
		{
			invM[i][j] = aug[i][j + 6];
		}
	}

	return TRUE;
}

void ArmIKDefaultConfig(ArmIKConfig_t *cfg)
{
	if (cfg == 0)
	{
		return;
	}

	cfg->max_iter = 120;
	cfg->pos_tol = 1e-3f;
	cfg->ori_tol = 2e-3f;
	cfg->damping = 0.05f;
	cfg->step_scale = 0.7f;
	cfg->jacobian_eps = 1e-4f;
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

ArmKinematics6D::ArmKinematics6D()
{
	for (uint8_t i = 0; i < ARM_DOF; i++)
	{
		dh_[i].a = 0.0f;
		dh_[i].alpha = 0.0f;
		dh_[i].d = 0.0f;
		dh_[i].theta_offset = 0.0f;
		q_min_[i] = -3.1415926f;
		q_max_[i] = 3.1415926f;
	}
}

void ArmKinematics6D::SetDH(const ArmDHParam_t dh[ARM_DOF])
{
	memcpy(dh_, dh, sizeof(dh_));
}

void ArmKinematics6D::SetJointLimit(const fp32 q_min[ARM_DOF], const fp32 q_max[ARM_DOF])
{
	memcpy(q_min_, q_min, sizeof(q_min_));
	memcpy(q_max_, q_max, sizeof(q_max_));
}

void ArmKinematics6D::GetDH(ArmDHParam_t dh_out[ARM_DOF]) const
{
	if (dh_out == 0)
	{
		return;
	}
	memcpy(dh_out, dh_, sizeof(dh_));
}

void ArmKinematics6D::GetJointLimit(fp32 q_min_out[ARM_DOF], fp32 q_max_out[ARM_DOF]) const
{
	if (q_min_out != 0)
	{
		memcpy(q_min_out, q_min_, sizeof(q_min_));
	}
	if (q_max_out != 0)
	{
		memcpy(q_max_out, q_max_, sizeof(q_max_));
	}
}

void ArmKinematics6D::BuildFKMatrix(const fp32 q[ARM_DOF], fp32 T[4][4]) const
{
	Mat4Identity(T);
	for (uint8_t i = 0; i < ARM_DOF; i++)
	{
		fp32 A[4][4];
		const fp32 theta = q[i] + dh_[i].theta_offset;
		BuildStdDHTransform(dh_[i].a, dh_[i].alpha, dh_[i].d, theta, A);
		Mat4Mul(T, A, T);
	}
}

void ArmKinematics6D::ForwardKinematics(const fp32 q[ARM_DOF], ArmPose_t *pose_out) const
{
	if (pose_out == 0)
	{
		return;
	}
	fp32 T[4][4];
	BuildFKMatrix(q, T);
	TransformToPose(T, pose_out);
}

void ArmKinematics6D::PoseError(const fp32 T_cur[4][4], const fp32 T_tar[4][4], fp32 err[6]) const
{
	err[0] = T_tar[0][3] - T_cur[0][3];
	err[1] = T_tar[1][3] - T_cur[1][3];
	err[2] = T_tar[2][3] - T_cur[2][3];

	fp32 Rerr[3][3];
	for (uint8_t i = 0; i < 3; i++)
	{
		for (uint8_t j = 0; j < 3; j++)
		{
			Rerr[i][j] = 0.0f;
			for (uint8_t k = 0; k < 3; k++)
			{
				Rerr[i][j] += T_tar[i][k] * T_cur[j][k];
			}
		}
	}

	err[3] = 0.5f * (Rerr[2][1] - Rerr[1][2]);
	err[4] = 0.5f * (Rerr[0][2] - Rerr[2][0]);
	err[5] = 0.5f * (Rerr[1][0] - Rerr[0][1]);
}

void ArmKinematics6D::BuildNumericalJacobian(const fp32 q[ARM_DOF], fp32 J[6][ARM_DOF], fp32 eps) const
{
	fp32 T0[4][4];
	BuildFKMatrix(q, T0);

	for (uint8_t col = 0; col < ARM_DOF; col++)
	{
		fp32 q_pert[ARM_DOF];
		memcpy(q_pert, q, sizeof(fp32) * ARM_DOF);
		q_pert[col] += eps;

		fp32 Tp[4][4];
		BuildFKMatrix(q_pert, Tp);

		J[0][col] = (Tp[0][3] - T0[0][3]) / eps;
		J[1][col] = (Tp[1][3] - T0[1][3]) / eps;
		J[2][col] = (Tp[2][3] - T0[2][3]) / eps;

		fp32 Rdelta[3][3];
		for (uint8_t i = 0; i < 3; i++)
		{
			for (uint8_t j = 0; j < 3; j++)
			{
				Rdelta[i][j] = 0.0f;
				for (uint8_t k = 0; k < 3; k++)
				{
					Rdelta[i][j] += Tp[i][k] * T0[j][k];
				}
			}
		}

		J[3][col] = 0.5f * (Rdelta[2][1] - Rdelta[1][2]) / eps;
		J[4][col] = 0.5f * (Rdelta[0][2] - Rdelta[2][0]) / eps;
		J[5][col] = 0.5f * (Rdelta[1][0] - Rdelta[0][1]) / eps;
	}
}

bool_t ArmKinematics6D::SolveIK(const ArmPose_t *target,
							const fp32 q_init[ARM_DOF],
							fp32 q_sol[ARM_DOF],
							const ArmIKConfig_t *cfg,
							uint16_t *iter_used) const
{
	if (target == 0 || q_init == 0 || q_sol == 0)
	{
		return FALSE;
	}

	ArmIKConfig_t local_cfg;
	if (cfg == 0)
	{
		ArmIKDefaultConfig(&local_cfg);
		cfg = &local_cfg;
	}

	fp32 q[ARM_DOF];
	memcpy(q, q_init, sizeof(q));

	fp32 T_tar[4][4];
	PoseToTransform(target, T_tar);

	for (uint16_t iter = 0; iter < cfg->max_iter; iter++)
	{
		fp32 T_cur[4][4];
		BuildFKMatrix(q, T_cur);

		fp32 err[6];
		PoseError(T_cur, T_tar, err);

		const fp32 pos_n = VecNorm3(err);
		const fp32 ori_n = VecNorm3(&err[3]);
		if (pos_n < cfg->pos_tol && ori_n < cfg->ori_tol)
		{
			memcpy(q_sol, q, sizeof(q));
			if (iter_used != 0)
			{
				*iter_used = iter;
			}
			return TRUE;
		}

		fp32 J[6][ARM_DOF];
		BuildNumericalJacobian(q, J, cfg->jacobian_eps);

		fp32 A[6][6];
		for (uint8_t r = 0; r < 6; r++)
		{
			for (uint8_t c = 0; c < 6; c++)
			{
				A[r][c] = 0.0f;
				for (uint8_t k = 0; k < ARM_DOF; k++)
				{
					A[r][c] += J[r][k] * J[c][k];
				}
			}
			A[r][r] += cfg->damping * cfg->damping;
		}

		fp32 invA[6][6];
		if (Inverse6x6(A, invA) == FALSE)
		{
			break;
		}

		fp32 y[6] = {0};
		for (uint8_t r = 0; r < 6; r++)
		{
			for (uint8_t c = 0; c < 6; c++)
			{
				y[r] += invA[r][c] * err[c];
			}
		}

		fp32 dq[ARM_DOF] = {0};
		for (uint8_t j = 0; j < ARM_DOF; j++)
		{
			for (uint8_t r = 0; r < 6; r++)
			{
				dq[j] += J[r][j] * y[r];
			}
		}

		for (uint8_t j = 0; j < ARM_DOF; j++)
		{
			q[j] += cfg->step_scale * dq[j];
			if (q[j] > q_max_[j])
			{
				q[j] = q_max_[j];
			}
			if (q[j] < q_min_[j])
			{
				q[j] = q_min_[j];
			}
		}

		if (VecNorm6(dq) < 1e-6f)
		{
			break;
		}
	}

	memcpy(q_sol, q, sizeof(q));
	if (iter_used != 0)
	{
		*iter_used = cfg->max_iter;
	}
	return FALSE;
}

bool_t ArmKinematics6D::SolveIKWithGripper(const ArmPose_t *target,
									   fp32 gripper_target,
									   const fp32 q_init[ARM_DOF],
									   ArmCommand_t *cmd_out,
									   const ArmIKConfig_t *cfg,
									   uint16_t *iter_used) const
{
	if (cmd_out == 0)
	{
		return FALSE;
	}

	const bool_t ok = SolveIK(target, q_init, cmd_out->q, cfg, iter_used);
	cmd_out->gripper = gripper_target;
	return ok;
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
	// 仅切换使能状态，不重置目标和初值
	g_arm_ctrl.enable = enable;
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

void ArmControlTask(void)
{
	if (g_arm_ctrl.enable == FALSE)
	{
		return;
	}

	// 确保首次进入时完成达妙关节实例注册
	ArmControlEnsureInit();

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
			g_arm_ctrl.joint_motor[i].DMMotorControl(hcan, motor);
		}
	}

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
