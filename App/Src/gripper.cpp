/**
 * @file gripper.cpp
 * @brief Servo gripper control for USER and mouse input paths.
 */

#include "gripper.h"

#include "bsp_pwm.h"
#include "message_center.h"
#include "referee_data.h"
#include "remote_control.h"

// 舵机使用 TIM1_CH1（PE9）输出 PWM
#define GRIPPER_SERVO_CHANNEL TIM_CHANNEL_1
// 舵机角度与脉宽换算（按 0.5ms~2.5ms 对应 0°~180°）：
// pulse_us = 500 + angle_deg * (2000 / 180)
// angle_deg = (pulse_us - 500) * (180 / 2000)
// 若你的舵机量程是 1.0ms~2.0ms 对应 0°~180°，则改为：
// pulse_us = 1000 + angle_deg * (1000 / 180)
// 在这里修改“夹爪打开角度”（单位: us，常见范围约 500~2500）
#define GRIPPER_SERVO_OPEN_PULSE_US  2000U
// 在这里修改“夹爪关闭角度”（单位: us，常见范围约 500~2500）
#define GRIPPER_SERVO_CLOSE_PULSE_US 1000U

// 自定义控制器按键映射：keys[0]=左实体按键（打开），keys[1]=右实体按键（关闭）
#define CUSTOM_GRIPPER_LEFT_KEY_INDEX  0U
#define CUSTOM_GRIPPER_RIGHT_KEY_INDEX 1U

// 保存当前目标脉宽，便于调试或上层读取状态
static uint16_t g_gripper_pulse_us = GRIPPER_SERVO_CLOSE_PULSE_US;

// 内部函数：写入脉宽到定时器比较寄存器
static void GripperApplyPulse(uint16_t pulse_us)
{
	g_gripper_pulse_us = pulse_us;
	TIMSetPWM(&htim1, GRIPPER_SERVO_CHANNEL, pulse_us);
}

// 上电初始化时默认关闭夹爪
void GripperInit(void)
{
	GripperSetClose();
}

// 周期任务：左键打开，右键关闭
// 若左右键都未按下，则保持当前状态
void GripperTask(void)
{
	ext_robot_interactive_data_t custom_ctrl_data;

	if (SysPointer()->engineer_mode == USER)
	{
		if (RefereeGetLatestCustomControllerData(&custom_ctrl_data) == TRUE)
		{
			if (custom_ctrl_data.keys[CUSTOM_GRIPPER_LEFT_KEY_INDEX] != 0U)
			{
				GripperSetOpen();
			}
			else if (custom_ctrl_data.keys[CUSTOM_GRIPPER_RIGHT_KEY_INDEX] != 0U)
			{
				GripperSetClose();
			}
		}
		return;
	}

	if (IF_MOUSE_PRESSED_LEFT)
	{
		GripperSetOpen();
	}
	else if (IF_MOUSE_PRESSED_RIGHT)
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
