#include "moving_behavior.h"
#include "arm.h"
#include "imu.h"
#include "remote_control.h"
#include "referee_data.h"
#include "buzzer.h"

static bool_t g_dm_arm_target_inited = FALSE;
static ArmPose_t g_dm_arm_target_pose = {0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f};
static fp32 g_dm_arm_locked_pitch = 0.0f;
static fp32 g_dm_arm_locked_roll = 0.0f;
static bool_t g_dm_arm_custom_input_active = FALSE;
static bool_t g_dm_arm_init_pose_inited = FALSE;
static fp32 g_dm_arm_init_q[ARM_DOF] = {0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f};
static uint8_t g_dm_arm_home_stable_count = 0;
static uint16_t g_dm_gyro_cali_count = 0;
static const uint16_t g_dm_gyro_cali_need = 20000;
static uint16_t g_dm_arm_cali_retry_tick = 0;
static uint16_t g_dm_arm_cali_retry_cnt = 0;
static const uint16_t g_dm_arm_cali_retry_interval = 50;
static const uint16_t g_dm_arm_cali_retry_max = 40;
static uint16_t g_cali_beep_total_tick = 0;
static uint16_t g_cali_beep_tick = 0;
static uint16_t g_cali_beep_on_tick = 0;
static uint16_t g_cali_beep_period_tick = 0;
static uint16_t g_cali_beep_psc = MOTOR_PSC;
static fp32 g_dm_gyro_cali_scale[3] = {1.0f, 1.0f, 1.0f};
static fp32 g_dm_gyro_cali_offset[3] = {0.0f, 0.0f, 0.0f};
static forward_kinematics_t g_master_fk_solver;
static bool_t g_master_fk_inited = FALSE;
static bool_t g_master_fk_data_valid = FALSE;
static uint16_t g_master_fk_raw12[ARM_DOF] = {0U, 0U, 0U, 0U, 0U, 0U};
static DMArmMasterFKDebug_t g_master_fk_debug = {FALSE, FALSE, {0U, 0U, 0U, 0U, 0U, 0U}, {0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f}, {0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f}, 0U};

static const fp32 g_master_fk_default_zero_raw[ARM_DOF] = {0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f};
static const int8_t g_master_fk_default_direction[ARM_DOF] = {1, 1, 1, 1, 1, 1};
static const fp32 g_master_fk_default_q_min[ARM_DOF] = {-1000.0f, -1000.0f, -1000.0f, -1000.0f, -1000.0f, -1000.0f};
static const fp32 g_master_fk_default_q_max[ARM_DOF] = {1000.0f, 1000.0f, 1000.0f, 1000.0f, 1000.0f, 1000.0f};

static fp32 DMArmClampValue(fp32 value, fp32 low, fp32 high);

static void CalibrateBeepStart(uint16_t total_tick, uint16_t on_tick, uint16_t period_tick, uint16_t psc)
{
	g_cali_beep_total_tick = total_tick;
	g_cali_beep_tick = 0;
	g_cali_beep_on_tick = on_tick;
	g_cali_beep_period_tick = period_tick;
	g_cali_beep_psc = psc;
}

static void CalibrateBeepUpdate(void)
{
	if (g_cali_beep_tick >= g_cali_beep_total_tick)
	{
		return;
	}

	if (g_cali_beep_period_tick == 0U)
	{
		buzzer.BuzzerWarn(0, 0, g_cali_beep_psc, 10000);
		g_cali_beep_tick++;
		return;
	}

	if ((g_cali_beep_tick % g_cali_beep_period_tick) < g_cali_beep_on_tick)
	{
		buzzer.BuzzerWarn(0, 0, g_cali_beep_psc, 10000);
	}

	g_cali_beep_tick++;
}

static bool_t g_legacy_end_axis_inited = FALSE;
static const fp32 g_legacy_end_cmd_gain = 0.003f;
static const fp32 g_legacy_pitch_range_deg = 180.0f;
static const fp32 g_legacy_roll_range_deg = 360.0f;
static const fp32 g_legacy_deg_to_rad = 0.01745329252f;
static fp32 g_legacy_pitch_center_ecd = 0.0f;
static fp32 g_legacy_roll_center_ecd = 0.0f;

static fp32 DMArmLegacyDegToEcd(fp32 degree)
{
	return (degree * g_legacy_deg_to_rad) / MOTOR_ECD_TO_RAD;
}

static void DMArmLegacyEndEnsureInit(void)
{
	if (g_legacy_end_axis_inited == TRUE)
	{
		return;
	}

	moving_t *mv = MovingPointer();
	if (mv == 0)
	{
		return;
	}

	mv->pitch_motor.motor_angle_set = mv->pitch_motor.motor_angle;
	mv->roll_motor.motor_angle_set = mv->roll_motor.motor_angle;
	g_legacy_pitch_center_ecd = mv->pitch_motor.motor_angle;
	g_legacy_roll_center_ecd = mv->roll_motor.motor_angle;
	g_legacy_end_axis_inited = TRUE;
}

static void DMArmLegacyEndApplyDelta(fp32 pitch_delta_rad, fp32 roll_delta_rad)
{
	moving_t *mv = MovingPointer();
	if (mv == 0)
	{
		return;
	}

	DMArmLegacyEndEnsureInit();

	const fp32 pitch_half_range_ecd = 0.5f * DMArmLegacyDegToEcd(g_legacy_pitch_range_deg);
	const fp32 roll_half_range_ecd = 0.5f * DMArmLegacyDegToEcd(g_legacy_roll_range_deg);

	mv->pitch_motor.motor_angle_set += pitch_delta_rad / MOTOR_ECD_TO_RAD;
	mv->roll_motor.motor_angle_set += roll_delta_rad / MOTOR_ECD_TO_RAD;

	mv->pitch_motor.motor_angle_set = DMArmClampValue(mv->pitch_motor.motor_angle_set,
		g_legacy_pitch_center_ecd - pitch_half_range_ecd,
		g_legacy_pitch_center_ecd + pitch_half_range_ecd);
	mv->roll_motor.motor_angle_set = DMArmClampValue(mv->roll_motor.motor_angle_set,
		g_legacy_roll_center_ecd - roll_half_range_ecd,
		g_legacy_roll_center_ecd + roll_half_range_ecd);
}

static void DMArmLegacyEndUpdateFromLatestCustom(void)
{
	ext_robot_interactive_data_t custom_ctrl_data;
	if (RefereeGetLatestCustomControllerData(&custom_ctrl_data) == TRUE)
	{
		DMArmLegacyEndApplyDelta(custom_ctrl_data.pitch_rad * g_legacy_end_cmd_gain,
			custom_ctrl_data.roll_rad * g_legacy_end_cmd_gain);
	}
}

void DMArmMasterFKLoadDefaultTemplate(void)
{
	const fp32 raw_to_rad = 6.28318530718f / (fp32)FK_AS5600L_RAW_MAX;
	for (uint8_t i = 0; i < ARM_DOF; i++)
	{
		FK_AxisConfig_t cfg;
		cfg.zero_raw = g_master_fk_default_zero_raw[i];
		cfg.scale = raw_to_rad;
		cfg.direction = g_master_fk_default_direction[i];
		cfg.q_min = g_master_fk_default_q_min[i];
		cfg.q_max = g_master_fk_default_q_max[i];
		cfg.dof_index = i;
		g_master_fk_solver.ConfigureAxis(i, cfg);
	}
}

static void DMArmMasterFKEnsureInit(void)
{
	if (g_master_fk_inited == TRUE)
	{
		return;
	}

	g_master_fk_solver.Reset();
	g_master_fk_solver.SetAxisNum(ARM_DOF);
	g_master_fk_solver.BindSolver(ArmGetSolver());

	ArmKinematics6D *solver = ArmGetSolver();
	if (solver != 0)
	{
		ArmDHParam_t dh[ARM_DOF];
		solver->GetDH(dh);
		g_master_fk_solver.SetDH(dh);
	}

	DMArmMasterFKLoadDefaultTemplate();

	g_master_fk_inited = TRUE;
	g_master_fk_debug.fk_inited = TRUE;
}

void DMArmMasterSetEncoderRaw12AS5600L(const uint16_t raw12[ARM_DOF])
{
	if (raw12 == 0)
	{
		return;
	}

	for (uint8_t i = 0; i < ARM_DOF; i++)
	{
		g_master_fk_raw12[i] = raw12[i] & FK_AS5600L_RAW_MASK;
		g_master_fk_debug.raw12[i] = g_master_fk_raw12[i];
	}
	g_master_fk_data_valid = TRUE;
	g_master_fk_debug.raw_valid = TRUE;
}

bool_t DMArmMasterFKSetAxisConfig(uint8_t axis_id, const FK_AxisConfig_t *cfg)
{
	if (cfg == 0)
	{
		return FALSE;
	}

	DMArmMasterFKEnsureInit();
	return (g_master_fk_solver.ConfigureAxis(axis_id, *cfg) == FK_OK) ? TRUE : FALSE;
}

bool_t DMArmMasterFKSetDH(const ArmDHParam_t dh[ARM_DOF])
{
	if (dh == 0)
	{
		return FALSE;
	}

	DMArmMasterFKEnsureInit();
	return (g_master_fk_solver.SetDH(dh) == FK_OK) ? TRUE : FALSE;
}

bool_t DMArmMasterFKGetDebug(DMArmMasterFKDebug_t *debug_out)
{
	if (debug_out == 0)
	{
		return FALSE;
	}

	*debug_out = g_master_fk_debug;
	return TRUE;
}

static bool_t DMArmTryUpdateTargetFromMasterFK(void)
{
	if (g_master_fk_data_valid == FALSE)
	{
		return FALSE;
	}

	DMArmMasterFKEnsureInit();

	for (uint8_t i = 0; i < ARM_DOF; i++)
	{
		FK_Status_e update_ret = g_master_fk_solver.UpdateAxisRaw(i, g_master_fk_raw12[i]);
		if (update_ret != FK_OK)
		{
			g_master_fk_debug.last_fk_status = (uint8_t)update_ret;
			return FALSE;
		}
	}

	ArmPose_t pose_out;
	FK_Status_e pose_ret = g_master_fk_solver.SolvePose(&pose_out);
	if (pose_ret != FK_OK)
	{
		g_master_fk_debug.last_fk_status = (uint8_t)pose_ret;
		return FALSE;
	}

	fp32 q_dbg[ARM_DOF] = {0.0f};
	if (g_master_fk_solver.GetJointQ(q_dbg) == FK_OK)
	{
		for (uint8_t i = 0; i < ARM_DOF; i++)
		{
			g_master_fk_debug.q_joint[i] = q_dbg[i];
		}
		ArmControlSetSeedQ(q_dbg);
	}

	g_dm_arm_target_pose = pose_out;
	g_dm_arm_target_pose.pitch = g_dm_arm_locked_pitch;
	g_dm_arm_target_pose.roll = g_dm_arm_locked_roll;
	g_master_fk_debug.pose = pose_out;
	g_master_fk_debug.last_fk_status = (uint8_t)FK_OK;
	ArmControlSetTargetPose(&g_dm_arm_target_pose, 0.0f);
	g_dm_arm_custom_input_active = TRUE;
	return TRUE;
}

static fp32 DMArmAbsValue(fp32 value)
{
	if (value < 0.0f)
	{
		return -value;
	}
	return value;
}

static fp32 DMArmClampValue(fp32 value, fp32 low, fp32 high)
{
	if (value < low)
	{
		return low;
	}
	if (value > high)
	{
		return high;
	}
	return value;
}

static void DMArmSetInitPoseWithJointLimit(void)
{
	if (g_dm_arm_init_pose_inited == TRUE)
	{
		return;
	}

	ArmKinematics6D *solver = ArmGetSolver();
	if (solver == 0)
	{
		return;
	}

	fp32 q_min[ARM_DOF];
	fp32 q_max[ARM_DOF];
	fp32 q_init[ARM_DOF];
	const fp32 *home_q = ArmGetHomeQ();

	solver->GetJointLimit(q_min, q_max);
	for (uint8_t i = 0; i < ARM_DOF; i++)
	{
		q_init[i] = DMArmClampValue(home_q[i], q_min[i], q_max[i]);
		g_dm_arm_init_q[i] = q_init[i];
	}

	solver->ForwardKinematics(q_init, &g_dm_arm_target_pose);
	ArmControlSetSeedQ(q_init);
	ArmControlSetTargetPose(&g_dm_arm_target_pose, 0.0f);
	g_dm_arm_target_inited = TRUE;
	g_dm_arm_init_pose_inited = TRUE;
}

static void DMArmUpdateHomeDoneFlag(void)
{
	const fp32 joint_tol = ARM_HOME_DONE_JOINT_TOL;
	const uint8_t stable_count_need = (uint8_t)ARM_HOME_DONE_STABLE_COUNT;
	fp32 q_feedback[ARM_DOF];
	bool_t is_home = FALSE;

	if (g_dm_arm_init_pose_inited == TRUE && ArmControlGetJointFeedbackQ(q_feedback) == TRUE)
	{
		is_home = TRUE;
		for (uint8_t i = 0; i < ARM_DOF; i++)
		{
			if (DMArmAbsValue(q_feedback[i] - g_dm_arm_init_q[i]) > joint_tol)
			{
				is_home = FALSE;
				break;
			}
		}
	}

	if (is_home == TRUE)
	{
		if (g_dm_arm_home_stable_count < stable_count_need)
		{
			g_dm_arm_home_stable_count++;
		}
	}
	else
	{
		g_dm_arm_home_stable_count = 0;
	}

	SysPointer()->arm_home_done = (g_dm_arm_home_stable_count >= stable_count_need) ? TRUE : FALSE;
}

static void DMArmEnsureTargetInit(void)
{
	if (g_dm_arm_target_inited == TRUE)
	{
		return;
	}

	ArmKinematics6D *solver = ArmGetSolver();
	if (solver != 0)
	{
		solver->ForwardKinematics(ArmGetHomeQ(), &g_dm_arm_target_pose);
	}

	g_dm_arm_locked_pitch = g_dm_arm_target_pose.pitch;
	g_dm_arm_locked_roll = g_dm_arm_target_pose.roll;

	ArmControlSetTargetPose(&g_dm_arm_target_pose, 0.0f);
	g_dm_arm_target_inited = TRUE;
}

static void DMArmUpdateTargetFromInput(void)
{
	const fp32 pose_xy_gain = 0.0008f;
	const fp32 pose_z_gain = 0.0006f;
	const fp32 pose_yaw_gain = 0.00012f;
	const fp32 pose_pitch_gain = 0.0001f;
	const fp32 custom_pos_gain = 0.001f;
	const fp32 custom_rot_gain = 0.003f;
	ext_robot_interactive_data_t custom_ctrl_data;

	DMArmEnsureTargetInit();
	g_dm_arm_custom_input_active = FALSE;

	if (IF_KEY_PRESSED_Z)
	{
		g_dm_arm_target_pose.x += pose_xy_gain;
	}
	if (IF_KEY_PRESSED_X)
	{
		g_dm_arm_target_pose.x -= pose_xy_gain;
	}
	if (IF_KEY_PRESSED_R)
	{
		g_dm_arm_target_pose.y += pose_xy_gain;
	}
	if (IF_KEY_PRESSED_F)
	{
		g_dm_arm_target_pose.y -= pose_xy_gain;
	}

	g_dm_arm_target_pose.z += (fp32)rc_ctrl.rc.ch[4] * pose_z_gain;
	g_dm_arm_target_pose.yaw += (fp32)rc_ctrl.mouse.x * pose_yaw_gain;
	DMArmLegacyEndApplyDelta(-(fp32)rc_ctrl.mouse.y * pose_pitch_gain, 0.0f);

	if (RefereePopCustomControllerData(&custom_ctrl_data) == TRUE)
	{
		g_dm_arm_target_pose.x += custom_ctrl_data.x_dis * custom_pos_gain;
		g_dm_arm_target_pose.y += custom_ctrl_data.y_dis * custom_pos_gain;
		g_dm_arm_target_pose.z += custom_ctrl_data.z_dis * custom_pos_gain;
		g_dm_arm_target_pose.yaw += custom_ctrl_data.yaw_rad * custom_rot_gain;
		DMArmLegacyEndApplyDelta(custom_ctrl_data.pitch_rad * custom_rot_gain,
			custom_ctrl_data.roll_rad * custom_rot_gain);
		g_dm_arm_custom_input_active = TRUE;
	}

	g_dm_arm_target_pose.pitch = g_dm_arm_locked_pitch;
	g_dm_arm_target_pose.roll = g_dm_arm_locked_roll;

	ArmControlSetTargetPose(&g_dm_arm_target_pose, 0.0f);
}

moving_t moving;

moving_t *MovingPointer(void)
{
	return &moving;

}

void moving_t::Get_info()
{
  last_sys_behaviour = sys_behaviour;
  sys_behaviour = SysPointer()->engineer_mode;

  front_l_motor.motor_angle = LiftingPointer()->lifting_motor_angle[FRONT_L];
  front_r_motor.motor_angle = LiftingPointer()->lifting_motor_angle[FRONT_R];
  up_l_motor.motor_angle = LiftingPointer()->lifting_motor_angle[UP_L];
  up_r_motor.motor_angle = LiftingPointer()->lifting_motor_angle[UP_R];
  side_motor.motor_angle = LiftingPointer()->lifting_motor_angle[SIDE];

  yaw_f_motor.motor_angle = SuctionPointer()->suction_motor_angle[YAW_F];
  yaw_b_motor.motor_angle = SuctionPointer()->suction_motor_angle[YAW_B];
  pitch_motor.motor_angle = SuctionPointer()->suction_motor_angle[PITCH];
  roll_motor.motor_angle = SuctionPointer()->suction_motor_angle[ROLL];

}

void moving_t::Get_set()
{
	NewRobotBehaviorTask();
}

void moving_t::NewRobotBehaviorTask()
{
	UpdateBehaviorMode();
	if (behavior_mode != last_behavior_mode)
	{
		OnBehaviorModeChange();
		last_behavior_mode = behavior_mode;
	}
	RunBehaviorMode();
}

void moving_t::UpdateBehaviorMode()
{
	switch (SysPointer()->engineer_mode)
	{
		case INIT:
			behavior_mode = NEW_BEHAVIOR_HOME;
			break;
		case KEYBOARD:
			behavior_mode = NEW_BEHAVIOR_DM_ARM_MANUAL;
			break;
		case USER:
			behavior_mode = NEW_BEHAVIOR_DM_ARM_AUTO;
			break;
		case CALIBRATE:
			behavior_mode = NEW_BEHAVIOR_DM_ARM_CALIBRATE;
			break;
		case TEST:
			behavior_mode = NEW_BEHAVIOR_DM_ARM_IK_WORK;
			break;
		default:
			behavior_mode = NEW_BEHAVIOR_IDLE;
			break;
	}
}

void moving_t::OnBehaviorModeChange()
{
	ResetBehaviorSteps();
	ClearDMArmHold();
	g_dm_arm_init_pose_inited = FALSE;
	g_dm_arm_home_stable_count = 0;
	g_dm_gyro_cali_count = 0;
	SysPointer()->arm_home_done = FALSE;
}

void moving_t::RunBehaviorMode()
{
	switch (behavior_mode)
	{
		case NEW_BEHAVIOR_HOME:
			BehaviorHome();
			break;
		case NEW_BEHAVIOR_DM_ARM_MANUAL:
			BehaviorDMArmManual();
			break;
		case NEW_BEHAVIOR_DM_ARM_AUTO:
			BehaviorDMArmAuto();
			break;
		case NEW_BEHAVIOR_DM_ARM_CALIBRATE:
			BehaviorDMArmCalibrate();
			break;
		case NEW_BEHAVIOR_DM_ARM_IK_WORK:
			BehaviorDMArmIKWork();
			break;
		case NEW_BEHAVIOR_IDLE:
		default:
			BehaviorIdle();
			break;
	}
}

void moving_t::BehaviorIdle()
{
	ArmControlSetEnable(FALSE);
	SysPointer()->arm_home_done = FALSE;
}

void moving_t::BehaviorHome()
{
	ArmControlSetEnable(TRUE);
	DMArmSetInitPoseWithJointLimit();
	DMArmUpdateHomeDoneFlag();
	if (task_step_1 == 0)
	{
		task_step_1 = 1;
	}
}

void moving_t::BehaviorDMArmManual()
{
	ArmControlSetEnable(TRUE);
	SysPointer()->arm_home_done = FALSE;
	DMArmEnsureTargetInit();
	DMArmUpdateTargetFromInput();

	if (SysPointer()->mode != NORMAL)
	{
		SysPointer()->mode = NORMAL;
	}

	if (DMArmHasInputCommand())
	{
		ClearDMArmHold();
		return;
	}

	HoldDMArmRelativeToGimbal();
}

void moving_t::BehaviorDMArmAuto()
{
	ArmControlSetEnable(TRUE);
	SysPointer()->arm_home_done = FALSE;
	DMArmEnsureTargetInit();
	DMArmLegacyEndUpdateFromLatestCustom();
	if (DMArmTryUpdateTargetFromMasterFK() == FALSE)
	{
		DMArmUpdateTargetFromInput();
	}

	if (SysPointer()->mode != SPIN)
	{
		SysPointer()->mode = SPIN;
	}

	if (DMArmHasInputCommand())
	{
		ClearDMArmHold();
		return;
	}

	HoldDMArmRelativeToGimbal();
}

void moving_t::BehaviorDMArmCalibrate()
{
	static uint8_t last_log_step = 0xFFU;

	if (task_step_1 == 0)
	{
		task_step_1 = 1;
		task_work_1 = 0;
		g_dm_gyro_cali_count = 0;
		g_dm_arm_cali_retry_tick = 0;
		g_dm_arm_cali_retry_cnt = 0;
		last_log_step = 0xFFU;
		CalibrateBeepStart(350, 350, 0, 65); // Stage1: long low beep
	}

	if (last_log_step != task_step_1)
	{
		if (task_step_1 == 1)
		{
			ArmPrintToUart1("[CALI] stage1 gimbal init start");
		}
		else if (task_step_1 == 2)
		{
			ArmPrintToUart1("[CALI] stage2 arm zero_offset start");
			CalibrateBeepStart(320, 60, 160, 50); // Stage2: double mid beep
		}
		else if (task_step_1 == 3)
		{
			ArmPrintToUart1("[CALI] stage3 gyro calibrate start");
			CalibrateBeepStart(420, 50, 120, 35); // Stage3: triple high beep
		}
		else if (task_step_1 == 4)
		{
			ArmPrintToUart1("[CALI] finish, switch to KEYBOARD");
			CalibrateBeepStart(450, 450, 0, 28); // Finish: long high beep
		}
		last_log_step = task_step_1;
	}

	CalibrateBeepUpdate();

	if (task_step_1 == 1)
	{
		// Stage 1: gimbal encoder-center calibration in INIT_MODE.
		if (SysPointer()->mode != INIT_MODE)
		{
			SysPointer()->mode = INIT_MODE;
			SysPointer()->change_mode_flag = 1;
		}

		// GimbalTask() will set mode back to NORMAL after center is stable.
		if (SysPointer()->mode == NORMAL)
		{
			task_step_1 = 2;
		}
	}

	if (task_step_1 == 2)
	{
		fp32 q_feedback[ARM_DOF];
		const fp32 *home_q = ArmGetHomeQ();
		const fp32 *q_ref = home_q;

		if (g_dm_arm_cali_retry_tick < g_dm_arm_cali_retry_interval)
		{
			g_dm_arm_cali_retry_tick++;
		}
		else
		{
			g_dm_arm_cali_retry_tick = 0;

			if (ArmControlGetJointFeedbackQ(q_feedback) == TRUE)
			{
				ArmSetHomeQ(q_feedback);
				q_ref = ArmGetHomeQ();
			}

			task_work_1 = (ArmControlCalibrateZeroOffsetFromCurrent(q_ref) == TRUE) ? 1 : 0;
			if (task_work_1 == 1)
			{
				task_step_1 = 3;
				g_dm_gyro_cali_count = 0;
				g_dm_arm_cali_retry_cnt = 0;
			}
			else
			{
				if (g_dm_arm_cali_retry_cnt < 0xFFFFU)
				{
					g_dm_arm_cali_retry_cnt++;
				}

				if (g_dm_arm_cali_retry_cnt >= g_dm_arm_cali_retry_max)
				{
					ArmPrintToUart1("[CALI] arm zero_offset failed, abort calibrate");
					CalibrateBeepStart(700, 80, 160, 75); // Fail: low double repeating
					SysPointer()->mode = NORMAL;
					SysPointer()->engineer_mode = KEYBOARD;
					SysPointer()->change_mode_flag = 1;
					task_step_1 = 5;
				}
			}
		}
	}

	if (task_step_1 == 3)
	{
		INS_cali_gyro(g_dm_gyro_cali_scale, g_dm_gyro_cali_offset, &g_dm_gyro_cali_count);
		if (g_dm_gyro_cali_count >= g_dm_gyro_cali_need)
		{
			INS_set_cali_gyro(g_dm_gyro_cali_scale, g_dm_gyro_cali_offset);
			task_step_1 = 4;
		}
	}

	if (task_step_1 == 4)
	{
		SysPointer()->mode = NORMAL;
		SysPointer()->engineer_mode = KEYBOARD;
		SysPointer()->change_mode_flag = 1;
		task_step_1 = 5;
	}

	ArmControlSetEnable(FALSE);
	SysPointer()->arm_home_done = FALSE;
}

void moving_t::BehaviorDMArmIKWork()
{
	ArmControlSetEnable(TRUE);
	SysPointer()->arm_home_done = FALSE;
	DMArmEnsureTargetInit();
	DMArmUpdateTargetFromInput();

	if (SysPointer()->mode != LOCK_WHEEL)
	{
		SysPointer()->mode = LOCK_WHEEL;
	}

	if (DMArmHasInputCommand())
	{
		ClearDMArmHold();
		return;
	}

	HoldDMArmRelativeToGimbal();
}

bool_t moving_t::DMArmHasInputCommand()
{
	if (g_dm_arm_custom_input_active == TRUE)
	{
		return TRUE;
	}

	if (IF_KEY_PRESSED_Z || IF_KEY_PRESSED_X || IF_KEY_PRESSED_R || IF_KEY_PRESSED_F)
	{
		return TRUE;
	}

	if ((rc_ctrl.mouse.x > 8) || (rc_ctrl.mouse.x < -8) ||
		(rc_ctrl.mouse.y > 8) || (rc_ctrl.mouse.y < -8))
	{
		return TRUE;
	}

	if ((rc_ctrl.rc.ch[4] > 80) || (rc_ctrl.rc.ch[4] < -80))
	{
		return TRUE;
	}

	return FALSE;
}

void moving_t::HoldDMArmRelativeToGimbal()
{
	if (dm_arm_hold_valid == FALSE)
	{
		dm_arm_hold_gimbal_yaw = GimbalPointer()->yaw_relative_angle;
		dm_arm_hold_delta_yaw = 0.0f;
		dm_arm_hold_valid = TRUE;
		return;
	}

	dm_arm_hold_delta_yaw = GimbalPointer()->yaw_relative_angle - dm_arm_hold_gimbal_yaw;
}

void moving_t::ClearDMArmHold()
{
	dm_arm_hold_valid = FALSE;
	dm_arm_hold_gimbal_yaw = 0.0f;
	dm_arm_hold_delta_yaw = 0.0f;
}

void moving_t::ResetBehaviorSteps()
{
	task_step_1 = 0;
	last_task_step_1 = 0;
	task_work_1 = 0;
	task_step_2 = 0;
	last_task_step_2 = 0;
	task_work_2 = 0;
}


void moving_motor_t::init(float angle_lenght,float lenght)
{
	to_angle=angle_lenght/lenght;
	max_angle=angle_lenght;
	min_angle=0;
}

void moving_motor_t::set_angle_limit()
{

	if(max_angle>0)
	{
		if(motor_angle_set<0)
			motor_angle_set=0;
		if(motor_angle_set>max_angle)
			motor_angle_set=max_angle;
	}

	if(max_angle<0)
	{
		if(motor_angle_set>0)
			motor_angle_set=0;
		if(motor_angle_set<max_angle)
			motor_angle_set=max_angle;
		}
}


void moving_t::main_init(float front_cm,float side_cm,float lifting_cm,char *step,char value)
{
	if(*step!=value)
	{
		return;
	}
	uint8_t front_ok=0,side_ok=0,lifting_ok=0;
	//前伸
	if(front_cm>0)//判断是否生效
	{
		//获取速度
		front_l_motor.add_angle=front_cm*front_l_motor.to_angle;
		front_r_motor.add_angle=front_cm*front_r_motor.to_angle;
		//判断设定是否到达目标值
		front_l_motor.motor_angle_set-=front_l_motor.add_angle;
		front_r_motor.motor_angle_set-=front_r_motor.add_angle;
	}
	//横移
	if(side_cm>0)
	{
		side_motor.add_angle=side_cm*side_motor.to_angle;
		side_motor.motor_angle_set-=side_motor.add_angle;

	}
	//抬升
	if(lifting_cm>0)
	{

		up_l_motor.add_angle=lifting_cm*up_l_motor.to_angle;
		up_r_motor.add_angle=lifting_cm*up_r_motor.to_angle;

		up_l_motor.motor_angle_set-=up_l_motor.add_angle;
		up_r_motor.motor_angle_set-=up_r_motor.add_angle;
	}

	//判断动作是否完成
	if((abs(front_l_motor.motor_angle-front_l_motor.motor_angle_set)>10000&&
		abs(front_r_motor.motor_angle-front_r_motor.motor_angle_set)>10000)||front_cm<=0)
	{
		front_ok=1;
	}
	if(abs(side_motor.motor_angle-side_motor.motor_angle_set)>8000||side_cm<=0)
	{
		side_ok=1;
	}
	if((abs(up_l_motor.motor_angle-up_l_motor.motor_angle_set)>10000&&
		abs(up_r_motor.motor_angle-up_r_motor.motor_angle_set)>10000)||lifting_cm<=0)
	{
		lifting_ok=1;
	}
	//全部完成，步骤值加1
	if(front_ok&&side_ok&&lifting_ok)
	{
if(front_cm>0)
{
	front_l_motor.motor_angle_set=0;
	front_r_motor.motor_angle_set=0;
	LiftingPointer()->lifting_motor_angle[FRONT_L]=2000;
	LiftingPointer()->lifting_motor_angle[FRONT_R]=2000;
}
if(side_cm>0)
{
	LiftingPointer()->lifting_motor_angle[SIDE]=4000;
	side_motor.motor_angle_set=0;
}
if(lifting_cm>0)
{
	up_l_motor.motor_angle_set=0;
	up_r_motor.motor_angle_set=0;
	LiftingPointer()->lifting_motor_angle[UP_L]=3000;
	LiftingPointer()->lifting_motor_angle[UP_R]=-3000;
}
	*step+=1;
	}
}


/**
  * @brief       主机构移动指定位置
	* @param[in]   前伸位置，横移位置，抬升位置，速度（add值），步骤指针，步骤值
  */	
 void moving_t::main_moving_position(float front_cm,float side_cm,float lifting_cm,float speed,char *step,char value)
 {
	 if(*step!=value)
	 {
		 return;
	 }
	 uint8_t front_ok=0,side_ok=0,lifting_ok=0;
	 //前伸
	 if(front_cm>-1)//判断是否生效
	 {
		 //获取目标值
		 front_l_motor.motor_angle_goal=front_cm*front_l_motor.to_angle;
		 front_r_motor.motor_angle_goal=front_cm*front_r_motor.to_angle;
		 //获取速度
		 front_l_motor.add_angle=abs(speed*front_l_motor.to_angle);
		 front_r_motor.add_angle=abs(speed*front_r_motor.to_angle);
		 //判断设定是否到达目标值
		 if(front_l_motor.motor_angle_set==front_l_motor.motor_angle_goal||
			 abs(front_l_motor.motor_angle_set-front_l_motor.motor_angle_goal)<abs(front_l_motor.add_angle))
		 {
			 front_l_motor.motor_angle_set=front_l_motor.motor_angle_goal;
		 }
		 else 
		 {
			 if(front_l_motor.motor_angle_set<front_l_motor.motor_angle_goal)
			 {
				 front_l_motor.motor_angle_set+=front_l_motor.add_angle;
			 }
			 else if(front_l_motor.motor_angle_set>front_l_motor.motor_angle_goal)
			 {
				 front_l_motor.motor_angle_set-=front_l_motor.add_angle;
			 }
		 }
		 //判断是否到达目标值
		 if(front_r_motor.motor_angle_set==front_r_motor.motor_angle_goal||
			 abs(front_r_motor.motor_angle_set-front_r_motor.motor_angle_goal)<abs(front_r_motor.add_angle))
		 {
			 front_r_motor.motor_angle_set=front_r_motor.motor_angle_goal;
		 }
		 else 
		 {
			 if(front_r_motor.motor_angle_set<front_r_motor.motor_angle_goal)
			 {
				 front_r_motor.motor_angle_set+=front_r_motor.add_angle;
			 }
			 else if(front_r_motor.motor_angle_set>front_r_motor.motor_angle_goal)
			 {
				 front_r_motor.motor_angle_set-=front_r_motor.add_angle;
			 }
		 }
	 }
	 //横移
	 if(side_cm>-1)
	 {
		 //获取目标值
		 side_motor.motor_angle_goal=side_cm*side_motor.to_angle;
		 //获取速度
		 side_motor.add_angle=abs(speed*side_motor.to_angle);
		 //判断是否到达目标值
		 if(side_motor.motor_angle_set==side_motor.motor_angle_goal||
			 abs(side_motor.motor_angle_set-side_motor.motor_angle_goal)<abs(side_motor.add_angle))
		 {
			 side_motor.motor_angle_set=side_motor.motor_angle_goal;
		 }
		 else 
		 {
			 if(side_motor.motor_angle_set<side_motor.motor_angle_goal)
			 {
				 side_motor.motor_angle_set+=side_motor.add_angle;
			 }
			 else if(side_motor.motor_angle_set>side_motor.motor_angle_goal)
			 {
				 side_motor.motor_angle_set-=side_motor.add_angle;
			 }
		 }
	 }
	 //抬升
	 if(lifting_cm>-1)
	 {
		 //获取目标值
		 up_l_motor.motor_angle_goal=lifting_cm*up_l_motor.to_angle;
		 up_r_motor.motor_angle_goal=lifting_cm*up_r_motor.to_angle;
		 //获取速度
		 up_l_motor.add_angle=abs(speed*up_l_motor.to_angle);
		 up_r_motor.add_angle=abs(speed*up_r_motor.to_angle);
		 //判断是否到达目标值
		 if(up_l_motor.motor_angle_set==up_l_motor.motor_angle_goal||
			 abs(up_l_motor.motor_angle_set-up_l_motor.motor_angle_goal)<abs(up_l_motor.add_angle))
		 {
			 up_l_motor.motor_angle_set=up_l_motor.motor_angle_goal;
		 }
		 else 
		 {
			 if(up_l_motor.motor_angle_set<up_l_motor.motor_angle_goal)
			 {
				 up_l_motor.motor_angle_set+=up_l_motor.add_angle;
			 }
			 else if(up_l_motor.motor_angle_set>up_l_motor.motor_angle_goal)
			 {
				 up_l_motor.motor_angle_set-=up_l_motor.add_angle;
			 }
		 }
		 //判断是否到达目标值
		 if(up_r_motor.motor_angle_set==up_r_motor.motor_angle_goal||
			 abs(up_r_motor.motor_angle_set-up_r_motor.motor_angle_goal)<abs(up_r_motor.add_angle))
		 {
			 up_r_motor.motor_angle_set=up_r_motor.motor_angle_goal;
		 }
		 else 
		 {
			 if(up_r_motor.motor_angle_set<up_r_motor.motor_angle_goal)
			 {
				 up_r_motor.motor_angle_set+=up_r_motor.add_angle;
			 }
			 else if(up_r_motor.motor_angle_set>up_r_motor.motor_angle_goal)
			 {
				 up_r_motor.motor_angle_set-=up_r_motor.add_angle;
			 }
		 }
	 }
 
	 up_l_motor.set_angle_limit();
	 up_r_motor.set_angle_limit();
	 front_l_motor.set_angle_limit();
	 front_r_motor.set_angle_limit();
	 side_motor.set_angle_limit();
 
	 //判断动作是否完成
	 if((abs(front_l_motor.motor_angle-front_l_motor.motor_angle_goal)<2000&&
		 abs(front_r_motor.motor_angle-front_r_motor.motor_angle_goal)<2000)||front_cm<0)
	 {
		 front_ok=1;
	 }
	 if(abs(side_motor.motor_angle-side_motor.motor_angle_goal)<1000||side_cm<0)
	 {
		 side_ok=1;
	 }
	 if((abs(up_l_motor.motor_angle-up_l_motor.motor_angle_goal)<2000&&
		 abs(up_r_motor.motor_angle-up_r_motor.motor_angle_goal)<2000)||lifting_cm<0)
	 {
		 lifting_ok=1;
	 }
	 //全部完成，步骤值加1
	 if(front_ok&&side_ok&&lifting_ok)
	 {
	 *step+=1;
	 }
 }

 
/**
  * @brief       主机构指定速度移动
	* @param[in]   前伸位置，横移位置，抬升位置，速度（add值），步骤指针，步骤值
  */
 void moving_t::main_moving_speed(float front_add,float side_add,float lifing_add,char *step,char value)
 {
 if(*step!=value)
     {
         return;
     }
     //前伸
     front_l_motor.add_angle=front_add*front_l_motor.to_angle;
     front_r_motor.add_angle=front_add*front_r_motor.to_angle;
     front_l_motor.motor_angle_set+=front_l_motor.add_angle;
     front_r_motor.motor_angle_set+=front_r_motor.add_angle;
     //横移
     side_motor.add_angle=side_add*side_motor.to_angle;
     side_motor.motor_angle_set+=side_motor.add_angle;
     //抬升	
     up_l_motor.add_angle=lifing_add*up_l_motor.to_angle;
     up_r_motor.add_angle=lifing_add*up_r_motor.to_angle;
     up_l_motor.motor_angle_set+=up_l_motor.add_angle;
     up_r_motor.motor_angle_set+=up_r_motor.add_angle;
         
     up_l_motor.set_angle_limit();
     up_r_motor.set_angle_limit();
     front_l_motor.set_angle_limit();
     front_r_motor.set_angle_limit();
     side_motor.set_angle_limit();
 
 
 }
 
void moving_t::main_moving_rc(char*step,char value)
{
	if(*step!=value)
	{
		return;
	}
	float front_speed,side_speed,lifting_speed;
	// front_speed=moving_rc_ctrl->rc.ch[1]*0.00003;
	// side_speed=moving_rc_ctrl->rc.ch[0]*0.00003;
	// lifting_speed=moving_rc_ctrl->rc.ch[2]*0.00004;
	main_moving_speed(front_speed,side_speed,lifting_speed,step,value);
	
}

/**
  * @brief       主机构停止移动
	* @param[in]   步骤指针，步骤值
  */
 void moving_t::main_moving_stop(char*step,char value)
 {
 if(*step!=value)
	 {
		 return;
	 }
 moving.up_l_motor.add_angle=0;
 moving.up_l_motor.motor_angle_goal=0;
 moving.up_l_motor.motor_angle_set=moving.up_l_motor.motor_angle;
 moving.up_r_motor.add_angle=0;
 moving.up_r_motor.motor_angle_goal=0;
 moving.up_r_motor.motor_angle_set=moving.up_r_motor.motor_angle;
 
 moving.front_l_motor.add_angle=0;
 moving.front_l_motor.motor_angle_goal=0;
 moving.front_l_motor.motor_angle_set=moving.front_l_motor.motor_angle;
 moving.front_r_motor.add_angle=0;
 moving.front_r_motor.motor_angle_goal=0;
 moving.front_r_motor.motor_angle_set=moving.front_r_motor.motor_angle;
 
 moving.side_motor.add_angle=0;
 moving.side_motor.motor_angle_goal=0;
 moving.side_motor.motor_angle_set=moving.side_motor.motor_angle;
 
 *step+=1;
 }
 

 /**
  * @brief       副机构移动指定位置
	* @param[in]   前伸位置，横移位置，抬升位置，速度（add值），步骤指针，步骤值
  */
void moving_t::ancillary_moving_position(float front_l_cm,float front_r_cm,float up_l_cm,float up_r_cm,float speed,char *step,char value)
{

if(*step!=value)
	{
		return;
	}
	uint8_t front_l_ok=0,front_r_ok = 0,up_l_ok=0,up_r_ok=0;
	
	//左前伸
	if(front_l_cm>-1)//判断是否生效
	{
		ancillary_front_motor_l.motor_angle_goal=front_l_cm*ancillary_front_motor_l.to_angle;
		//获取速度
		ancillary_front_motor_l.add_angle=abs(speed*ancillary_front_motor_l.to_angle);
		//判断是否到达目标值
		if(ancillary_front_motor_l.motor_angle_set==ancillary_front_motor_l.motor_angle_goal||
			abs(ancillary_front_motor_l.motor_angle_set-ancillary_front_motor_l.motor_angle_goal)<abs(ancillary_front_motor_l.add_angle))
		{
			ancillary_front_motor_l.motor_angle_set=ancillary_front_motor_l.motor_angle_goal;
		}
		else 
		{
			if(ancillary_front_motor_l.motor_angle_set<ancillary_front_motor_l.motor_angle_goal)
			{
				ancillary_front_motor_l.motor_angle_set+=ancillary_front_motor_l.add_angle;
			}
			else if(ancillary_front_motor_l.motor_angle_set>ancillary_front_motor_l.motor_angle_goal)
			{
				ancillary_front_motor_l.motor_angle_set-=ancillary_front_motor_l.add_angle;
			}
		}
	}	
			//右前伸
	if(front_r_cm>-1)//判断是否生效
	{
		ancillary_front_motor_r.motor_angle_goal=front_r_cm*ancillary_front_motor_r.to_angle;
		//获取速度
		ancillary_front_motor_r.add_angle=abs(speed*ancillary_front_motor_r.to_angle);
		//判断是否到达目标值
		if(ancillary_front_motor_r.motor_angle_set==ancillary_front_motor_r.motor_angle_goal||
			abs(ancillary_front_motor_r.motor_angle_set-ancillary_front_motor_r.motor_angle_goal)<abs(ancillary_front_motor_r.add_angle))
		{
			ancillary_front_motor_r.motor_angle_set=ancillary_front_motor_r.motor_angle_goal;
		}
		else 
		{
			if(ancillary_front_motor_r.motor_angle_set<ancillary_front_motor_r.motor_angle_goal)
			{
				ancillary_front_motor_r.motor_angle_set+=ancillary_front_motor_r.add_angle;
			}
			else if(ancillary_front_motor_r.motor_angle_set>ancillary_front_motor_r.motor_angle_goal)
			{
				ancillary_front_motor_r.motor_angle_set-=ancillary_front_motor_r.add_angle;
			}
		}
	}	

	//左抬升
	if(up_l_cm>-1)
	{
		ancillary_up_motor_l.motor_angle_goal=up_l_cm*ancillary_up_motor_l.to_angle;		
		ancillary_up_motor_l.add_angle=abs(speed*ancillary_up_motor_l.to_angle);		
		if(ancillary_up_motor_l.motor_angle_set==ancillary_up_motor_l.motor_angle_goal||
			abs(ancillary_up_motor_l.motor_angle_set-ancillary_up_motor_l.motor_angle_goal)<abs(ancillary_up_motor_l.add_angle))
		{
			ancillary_up_motor_l.motor_angle_set=ancillary_up_motor_l.motor_angle_goal;
		}
		else 
		{
			if(ancillary_up_motor_l.motor_angle_set<ancillary_up_motor_l.motor_angle_goal)
			{
				ancillary_up_motor_l.motor_angle_set+=ancillary_up_motor_l.add_angle;
			}
			else if(ancillary_up_motor_l.motor_angle_set>ancillary_up_motor_l.motor_angle_goal)
			{
				ancillary_up_motor_l.motor_angle_set-=ancillary_up_motor_l.add_angle;
			}
		}
	}
	//右抬升
	if(up_r_cm>-1)
	{
		ancillary_up_motor_r.motor_angle_goal=up_r_cm*ancillary_up_motor_r.to_angle;	
		ancillary_up_motor_r.add_angle=abs(speed*ancillary_up_motor_r.to_angle);	
		if(ancillary_up_motor_r.motor_angle_set==ancillary_up_motor_r.motor_angle_goal||
			abs(ancillary_up_motor_r.motor_angle_set-ancillary_up_motor_r.motor_angle_goal)<abs(ancillary_up_motor_r.add_angle))
		{
			ancillary_up_motor_r.motor_angle_set=ancillary_up_motor_r.motor_angle_goal;
		}
		else 
		{
			if(ancillary_up_motor_r.motor_angle_set<ancillary_up_motor_r.motor_angle_goal)
			{
				ancillary_up_motor_r.motor_angle_set+=ancillary_up_motor_r.add_angle;
			}
			else if(ancillary_up_motor_r.motor_angle_set>ancillary_up_motor_r.motor_angle_goal)
			{
				ancillary_up_motor_r.motor_angle_set-=ancillary_up_motor_r.add_angle;
			}
		}
	}

	ancillary_up_motor_r.set_angle_limit();
	ancillary_up_motor_l.set_angle_limit();
	ancillary_front_motor_l.set_angle_limit();
	
	if(abs(ancillary_front_motor_l.motor_angle-ancillary_front_motor_l.motor_angle_goal)<1500||front_l_cm<0)
	{
		front_l_ok=1;
	}
	if(abs(ancillary_front_motor_r.motor_angle-ancillary_front_motor_r.motor_angle_goal)<1500||front_r_cm<0)
	{
		front_r_ok=1;
	}
	if(abs(ancillary_up_motor_l.motor_angle-ancillary_up_motor_l.motor_angle_goal)<1500||up_l_cm<0)
	{
		up_l_ok=1;
	}
	if(abs(ancillary_up_motor_r.motor_angle-ancillary_up_motor_r.motor_angle_goal)<2000||up_r_cm<0)
	{
		up_r_ok=1;
	}
	if(front_l_ok&&front_r_ok&&up_l_ok&&up_r_ok)
	{
	*step+=1;
	}
}

// void moving_t::arm_moving_init(float yaw_f,float yaw_b,float pitch,float roll,char *step,char value)
// {
// 	if(*step!=value)
// 	{
// 		return;
// 	}
// 	uint8_t yaw_f_ok=0,yaw_b_ok=0,pitch_ok=0,roll_ok=0;
// 	//前伸
// 	if(yaw_f>0)//判断是否生效
// 	{
// 		//获取速度
// 		yaw_f_motor.add_angle=yaw_f*yaw_f_motor.to_angle;
// 		//判断设定是否到达目标值
// 		yaw_f_motor.motor_angle_set-=yaw_f_motor.add_angle;
// 	}
// 	//横移
// 	if(yaw_b>0)
// 	{
// 		yaw_b_motor.add_angle=yaw_b*yaw_b_motor.to_angle;
// 		yaw_b_motor.motor_angle_set-=yaw_b_motor.add_angle;
// 	}
// 	if(pitch>0)
// 	{
// 		pitch_motor.add_angle=pitch*pitch_motor.to_angle;
// 		pitch_motor.motor_angle_set+=pitch_motor.add_angle;
// 	}
// 	//抬升
// 	if(roll>0)
// 	{

// 	}
// roll_ok=1;
// 	//判断动作是否完成
// 	if(abs(yaw_f_motor.motor_angle-yaw_f_motor.motor_angle_set)>16000||yaw_f<=0)
// 	{
// 		yaw_f_ok=1;
// 	}
// 	if(abs(yaw_b_motor.motor_angle-yaw_b_motor.motor_angle_set)>8000||yaw_b<=0)
// 	{
// 		yaw_b_ok=1;
// 	}
// 	if(abs(pitch_motor.motor_angle-pitch_motor.motor_angle_set)>2500||pitch<=0)
// 	{
// 		pitch_ok=1;
// 	}
// 	//全部完成，步骤值加1
// 	if(yaw_f_ok&&yaw_b_ok&&pitch_ok&&roll_ok)
// 	{
// 		if(yaw_f>0)
// 		{
// 			yaw_f_motor.motor_angle_set=40000;
// 			SuctionPointer()->suction_motor_angle[YAW_F]=19200;
// 		}
// 		if(yaw_b>0)
// 		{
// 			yaw_b_motor.motor_angle_set=0;
// 			SuctionPointer()->suction_motor_angle[YAW_B]=0;
// 		}
// 		if(pitch>0)
// 		{
// 			pitch_motor.motor_angle_set=0;
// 			SuctionPointer()->suction_motor_angle[PITCH]=-100000;
// 			SuctionPointer()->suction_motor_angle[ROLL]=100000;
// 		}
// 		*step+=1;
// 	}
// }


// void moving_t::arm_moving_position(float yaw_f_rad,float yaw_b_rad,float pitch_rad,float roll_rad,float speed,char *step,char value)
// {
// if(*step!=value)
// 	{
// 		return;
// 	}
// 	uint8_t yaw_f_ok=0,yaw_b_ok=0,pitch_ok=0,roll_ok=0;
	
// 	//yaw


// 	if(yaw_b_rad>-1)//判断是否生效
// 	{
// 		yaw_b_motor.motor_angle_goal=yaw_b_rad*yaw_b_motor.to_angle;
// 		//获取速度
// 		yaw_b_motor.add_angle=abs(speed*yaw_b_motor.to_angle);
// 		//判断是否到达目标值
// 		if(yaw_b_motor.motor_angle_set==yaw_b_motor.motor_angle_goal||
// 			abs(yaw_b_motor.motor_angle_set-yaw_b_motor.motor_angle_goal)<abs(yaw_b_motor.add_angle))
// 		{
// 			yaw_b_motor.motor_angle_set=yaw_b_motor.motor_angle_goal;
// 		}
// 		else 
// 		{
// 			if(yaw_b_motor.motor_angle_set<yaw_b_motor.motor_angle_goal)
// 			{
// 				yaw_b_motor.motor_angle_set+=yaw_b_motor.add_angle;
// 			}
// 			else if(yaw_b_motor.motor_angle_set>yaw_b_motor.motor_angle_goal)
// 			{
// 				yaw_b_motor.motor_angle_set-=yaw_b_motor.add_angle;
// 			}
// 		}
// 	}	

// 	if(yaw_f_rad>-1)//判断是否生效
// 	{
// 		yaw_f_motor.motor_angle_goal=(yaw_f_rad+moving.yaw_b_rad)*yaw_f_motor.to_angle;
// 		//获取速度
// 		yaw_f_motor.add_angle=abs(speed*yaw_f_motor.to_angle);
// 		//判断是否到达目标值
// 		if(yaw_f_motor.motor_angle_set==yaw_f_motor.motor_angle_goal||
// 			abs(yaw_f_motor.motor_angle_set-yaw_f_motor.motor_angle_goal)<abs(yaw_f_motor.add_angle))
// 		{
// 			yaw_f_motor.motor_angle_set=yaw_f_motor.motor_angle_goal;
// 		}
// 		else 
// 		{
// 			if(yaw_f_motor.motor_angle_set<yaw_f_motor.motor_angle_goal)
// 			{
// 				yaw_f_motor.motor_angle_set+=yaw_f_motor.add_angle;
// 			}
// 			else if(yaw_f_motor.motor_angle_set>yaw_f_motor.motor_angle_goal)
// 			{
// 				yaw_f_motor.motor_angle_set-=yaw_f_motor.add_angle;
// 			}
// 		}
// 	}
		
// 	//pitch
// 	if(pitch_rad>-1)
// 	{
// 		pitch_motor.motor_angle_goal=pitch_rad*pitch_motor.to_angle;
// 		//获取速度
// 		pitch_motor.add_angle=abs(speed*pitch_motor.to_angle);
// 		//判断是否到达目标值
// 		if(pitch_motor.motor_angle_set==pitch_motor.motor_angle_goal||
// 			abs(pitch_motor.motor_angle_set-pitch_motor.motor_angle_goal)<abs(pitch_motor.add_angle))
// 		{
// 			pitch_motor.motor_angle_set=pitch_motor.motor_angle_goal;
// 		}
// 		else 
// 		{
// 			if(pitch_motor.motor_angle_set<pitch_motor.motor_angle_goal)
// 			{
// 				pitch_motor.motor_angle_set+=pitch_motor.add_angle;
// 			}
// 			else if(pitch_motor.motor_angle_set>pitch_motor.motor_angle_goal)
// 			{
// 				pitch_motor.motor_angle_set-=pitch_motor.add_angle;
// 			}
// 		}
// 	}
// 	//roll
// 	if(roll_rad>-1)
// 	{
// 		roll_motor.motor_angle_goal=roll_rad*roll_motor.to_angle;
		
// 		roll_motor.add_angle=abs(speed*roll_motor.to_angle);
		
// 		if(roll_motor.motor_angle_set==roll_motor.motor_angle_goal||
// 			abs(roll_motor.motor_angle_set-roll_motor.motor_angle_goal)<abs(roll_motor.add_angle))
// 		{
// 			roll_motor.motor_angle_set=roll_motor.motor_angle_goal;
// 		}
// 		else 
// 		{
// 			if(roll_motor.motor_angle_set<roll_motor.motor_angle_goal)
// 			{
// 				roll_motor.motor_angle_set+=roll_motor.add_angle;
// 			}
// 			else if(roll_motor.motor_angle_set>roll_motor.motor_angle_goal)
// 			{
// 				roll_motor.motor_angle_set-=roll_motor.add_angle;
// 			}
// 		}
// 	}

// 	yaw_b_motor.set_angle_limit();
// 	yaw_f_motor.set_angle_limit();
// 	pitch_motor.set_angle_limit();
// 	roll_motor.set_angle_limit();
	
// 	if(abs(yaw_b_motor.motor_angle-yaw_b_motor.motor_angle_goal)<800||yaw_b_rad<0)
// 	{
// 		yaw_b_ok=1;
// 	}
// 	if(abs(yaw_f_motor.motor_angle-yaw_f_motor.motor_angle_goal)<800||yaw_f_rad<0)
// 	{
// 		yaw_f_ok=1;
// 	}
// 	if(abs(pitch_motor.motor_angle-pitch_motor.motor_angle_goal)<800||pitch_rad<0)
// 	{
// 		pitch_ok=1;
// 	}
// 	if(abs(roll_motor.motor_angle-roll_motor.motor_angle_goal)<800||roll_rad<0)
// 	{
// 		roll_ok=1;
// 	}
// 	if(yaw_f_ok&&yaw_b_ok&&pitch_ok&&roll_ok)
// 	{
// 	*step+=1;
// 	}
// }

// void moving_t::arm_moving_speed(float yaw_f_add,float yaw_b_add,float pitch_add,float roll_add,char *step,char value)
// {
// if(*step!=value)
// 	{
// 		return;
// 	}
// 	//yaw
// 	yaw_b_motor.add_angle=yaw_b_add*yaw_b_motor.to_angle;
// 	yaw_b_motor.motor_angle_set+=yaw_b_motor.add_angle;

// 	yaw_f_motor.add_angle=(yaw_f_add+yaw_b_add)*yaw_f_motor.to_angle;
// 	yaw_f_motor.motor_angle_set+=yaw_f_motor.add_angle;
// 	//pitch
// 	pitch_motor.add_angle=pitch_add*pitch_motor.to_angle;
// 	pitch_motor.motor_angle_set+=pitch_motor.add_angle;
// 	//roll
// 	roll_motor.add_angle=roll_add*roll_motor.to_angle;
// 	roll_motor.motor_angle_set+=roll_motor.add_angle;

// 	yaw_b_motor.set_angle_limit();
// 	yaw_f_motor.set_angle_limit();
// 	pitch_motor.set_angle_limit();
// 	roll_motor.set_angle_limit();
// }


// void moving_t::arm_moving_rc(char*step,char value)
// {
// if(*step!=value)
// 	{
// 		return;
// 	}
// 	float yaw_b_speed,yaw_f_speed,pitch_speed,roll_speed;
// 	// yaw_f_speed=moving_rc_ctrl->rc.ch[1]*0.000003;
// 	// yaw_b_speed=moving_rc_ctrl->rc.ch[0]*0.000002;
// 	// pitch_speed=moving_rc_ctrl->rc.ch[3]*0.00005;
// 	// roll_speed=moving_rc_ctrl->rc.ch[2]*0.00001;	
// 	arm_moving_speed(yaw_f_speed,yaw_b_speed,pitch_speed,roll_speed,step,value);
// }

void moving_t::arm_moving_init(float yaw_f,float yaw_b,float pitch,float roll,char *step,char value)
{
	(void)yaw_f;
	(void)yaw_b;
	(void)pitch;
	(void)roll;
	if(step == NULL || *step != value)
	{
		return;
	}
	*step += 1;
}

void moving_t::arm_moving_position(float yaw_f_rad,float yaw_b_rad,float pitch_rad,float roll_rad,float speed,char *step,char value)
{
	(void)yaw_f_rad;
	(void)yaw_b_rad;
	(void)pitch_rad;
	(void)roll_rad;
	(void)speed;
	if(step == NULL || *step != value)
	{
		return;
	}
	*step += 1;
}

void moving_t::arm_moving_rc(char*step,char value)
{
	if(step == NULL || *step != value)
	{
		return;
	}
	*step += 1;
}



