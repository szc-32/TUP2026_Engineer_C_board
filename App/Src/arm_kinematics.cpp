/**
 * @file arm_kinematics.cpp
 * @brief 六轴机械臂正逆解算实现。
 */

#include "arm.h"

#ifdef abs
#undef abs
#endif

#include <math.h>
#include <string.h>

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
