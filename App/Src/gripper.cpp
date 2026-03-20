/**
 * @file gripper.cpp
 * @brief Servo gripper control for USER and mouse input paths.
 */

#include "gripper.h"

#include "bsp_pwm.h"
#include "operation_def.h"
#include "remote_control.h"

// 舵机使用 TIM1_CH1（PE9）输出 PWM
#define GRIPPER_SERVO_CHANNEL TIM_CHANNEL_1
// 舵机角度与脉宽换算（按 0.5ms~2.5ms 对应 0°~180°）：
// pulse_us = 500 + angle_deg * (2000 / 180)
// angle_deg = (pulse_us - 500) * (180 / 2000)
// 若你的舵机量程是 1.0ms~2.0ms 对应 0°~180°，则改为：
// pulse_us = 1000 + angle_deg * (1000 / 180)
// 在这里修改“夹爪打开角度”（单位: us，常见范围约 500~2500）
#define GRIPPER_SERVO_OPEN_PULSE_US  3000U
// 在这里修改“夹爪关闭角度”（单位: us，常见范围约 500~2500）
#define GRIPPER_SERVO_CLOSE_PULSE_US 1000U

// 保存当前目标脉宽，便于调试或上层读取状态
static uint16_t g_gripper_pulse_us = GRIPPER_SERVO_CLOSE_PULSE_US;

typedef enum
{
	GRIPPER_THUMB_IDLE = 0,
	GRIPPER_THUMB_OPEN,
	GRIPPER_THUMB_CLOSE,
} GripperThumbAction_e;

// 内部函数：写入脉宽到定时器比较寄存器
static void GripperApplyPulse(uint16_t pulse_us)
{
	g_gripper_pulse_us = pulse_us;
	TIMSetPWM(&htim1, GRIPPER_SERVO_CHANNEL, pulse_us);
}

static bool_t GripperIsThumbControlMode(const RC_ctrl_t *rc_ctrl)
{
	if (rc_ctrl == 0)
	{
		return FALSE;
	}

	return (switch_is_down(rc_ctrl->rc.s[LEFT_CHANNEL]) &&
		switch_is_mid(rc_ctrl->rc.s[RIGTH_CHANNEL])) ? TRUE : FALSE;
}

static GripperThumbAction_e GripperGetThumbAction(const RC_ctrl_t *rc_ctrl)
{
	if (rc_ctrl == 0)
	{
		return GRIPPER_THUMB_IDLE;
	}

	if (rc_ctrl->rc.ch[4] < -(int16_t)RC_THUMB_USE_VALUE_MIN)
	{
		return GRIPPER_THUMB_OPEN;
	}

	if (rc_ctrl->rc.ch[4] > (int16_t)RC_THUMB_USE_VALUE_MIN)
	{
		return GRIPPER_THUMB_CLOSE;
	}

	return GRIPPER_THUMB_IDLE;
}

// 上电初始化时默认关闭夹爪
void GripperInit(void)
{
	GripperSetClose();
}

// 周期任务：左下右中模式下，拨轮上开、拨轮下关
// 若不在该模式或拨轮回中，则保持当前状态
void GripperTask(void)
{
	const RC_ctrl_t *rc_ctrl = GetRemoteControlPoint();
	GripperThumbAction_e thumb_action;

	if (GripperIsThumbControlMode(rc_ctrl) == FALSE)
	{
		return;
	}

	thumb_action = GripperGetThumbAction(rc_ctrl);
	if (thumb_action == GRIPPER_THUMB_OPEN)
	{
		GripperSetOpen();
	}
	else if (thumb_action == GRIPPER_THUMB_CLOSE)
	{
		GripperSetClose();
	}
}

// 打开夹爪
void GripperSetOpen(void)
{
	GripperApplyPulse(GRIPPER_SERVO_OPEN_PULSE_US);
}

// 关闭夹爪
void GripperSetClose(void)
{
	GripperApplyPulse(GRIPPER_SERVO_CLOSE_PULSE_US);
}

// 获取当前脉宽（单位: us）
uint16_t GripperGetPulseUs(void)
{
	return g_gripper_pulse_us;
}
