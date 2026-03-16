/**
 * @file arm_kinematics.h
 * @brief 六轴机械臂解算算法声明（求解器/IK/home状态）。
 *
 * 说明：本头文件依赖 arm.h 中已定义的 ARM_DOF、fp32、ArmPose_t、
 * ArmDHParam_t、ArmIKConfig_t 与 ArmCommand_t，请在 arm.h 内部或其后包含。
 */

#ifndef __ARM_KINEMATICS_H
#define __ARM_KINEMATICS_H

#ifdef __cplusplus
extern "C" {
#endif

// 加载解算器默认参数（DH、关节限位、IK参数）
void ArmKinematicsLoadDefaults(void);

// 载入默认IK参数
void ArmIKDefaultConfig(ArmIKConfig_t *cfg);

// 获取全局IK参数指针（可在线调参）
ArmIKConfig_t *ArmGetIKConfig(void);

// 获取/设置机械臂home关节角
const fp32 *ArmGetHomeQ(void);
void ArmSetHomeQ(const fp32 home_q[ARM_DOF]);

#ifdef __cplusplus
}

class ArmKinematics6D
{
public:
	ArmKinematics6D();

	// 设置/获取运动学参数
	void SetDH(const ArmDHParam_t dh[ARM_DOF]);
	void SetJointLimit(const fp32 q_min[ARM_DOF], const fp32 q_max[ARM_DOF]);
	void GetDH(ArmDHParam_t dh_out[ARM_DOF]) const;
	void GetJointLimit(fp32 q_min_out[ARM_DOF], fp32 q_max_out[ARM_DOF]) const;

	// 正解：关节角 -> 末端位姿
	void ForwardKinematics(const fp32 q[ARM_DOF], ArmPose_t *pose_out) const;

	// 逆解：末端位姿 -> 关节角
	bool_t SolveIK(const ArmPose_t *target,
				   const fp32 q_init[ARM_DOF],
				   fp32 q_sol[ARM_DOF],
				   const ArmIKConfig_t *cfg,
				   uint16_t *iter_used = 0) const;

	// 逆解并携带夹爪目标透传
	bool_t SolveIKWithGripper(const ArmPose_t *target,
						  fp32 gripper_target,
						  const fp32 q_init[ARM_DOF],
						  ArmCommand_t *cmd_out,
						  const ArmIKConfig_t *cfg,
						  uint16_t *iter_used = 0) const;

private:
	ArmDHParam_t dh_[ARM_DOF];
	fp32 q_min_[ARM_DOF];
	fp32 q_max_[ARM_DOF];

	void BuildFKMatrix(const fp32 q[ARM_DOF], fp32 T[4][4]) const;
	void BuildNumericalJacobian(const fp32 q[ARM_DOF], fp32 J[6][ARM_DOF], fp32 eps) const;
	void PoseError(const fp32 T_cur[4][4], const fp32 T_tar[4][4], fp32 err[6]) const;
};

// 获取全局求解器实例
ArmKinematics6D *ArmGetSolver(void);

#endif

#endif
