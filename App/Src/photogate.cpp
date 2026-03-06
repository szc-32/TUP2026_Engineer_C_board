
/**
 * @file photogate.cpp
 * @brief Debounced photogate state machine with yaw-home latching.
 */

#include "photogate.h"

// 读取原始电平并映射为“遮挡/未遮挡”状态
static uint8_t PhotogateReadRaw(const Photogate_t *pg)
{
	GPIO_PinState state = GPIORead(pg->port, pg->pin);
	return (state == pg->active_level) ? PHOTOGATE_BLOCKED : PHOTOGATE_CLEAR;
}

// 初始化光电门去抖状态机与home缓存
void PhotogateInit(Photogate_t *pg,
				   GPIO_TypeDef *port,
				   uint16_t pin,
				   GPIO_PinState active_level,
				   uint32_t stable_threshold)
{
	if (pg == NULL)
	{
		return;
	}

	pg->port = port;
	pg->pin = pin;
	pg->active_level = active_level;
	pg->stable_threshold = (stable_threshold == 0U) ? PHOTOGATE_STABLE_COUNT : stable_threshold;

	pg->last_sample = PhotogateReadRaw(pg);
	pg->stable_state = pg->last_sample;
	pg->stable_count = 0U;
	pg->edge_latched = 0U;
	pg->yaw_home_offset_rad = 0.0f;
	pg->home_valid = 0U;
}

// 去抖更新逻辑：状态稳定到阈值后才改变stable_state
// 当检测到“遮挡沿”时置位edge_latched供上层处理
void PhotogateUpdate(Photogate_t *pg)
{
	uint8_t sample;

	if (pg == NULL)
	{
		return;
	}

	sample = PhotogateReadRaw(pg);

	if (sample == pg->last_sample)
	{
		if (pg->stable_count < pg->stable_threshold)
		{
			pg->stable_count++;
		}
	}
	else
	{
		pg->stable_count = 0U;
		pg->last_sample = sample;
	}

	if (pg->stable_count >= pg->stable_threshold && pg->stable_state != sample)
	{
		pg->stable_state = sample;
		if (pg->stable_state == PHOTOGATE_BLOCKED)
		{
			pg->edge_latched = 1U;
		}
	}
}

// 在基础更新后，若检测到遮挡沿，则记录当前yaw为home
void PhotogateUpdateWithYaw(Photogate_t *pg, fp32 yaw_encoder_rad)
{
	if (pg == NULL)
	{
		return;
	}

	PhotogateUpdate(pg);

	if (pg->edge_latched != 0U)
	{
		pg->yaw_home_offset_rad = yaw_encoder_rad;
		pg->home_valid = 1U;
		pg->edge_latched = 0U;
	}
}

// 获取当前稳定状态
PhotogateState_e PhotogateGetState(const Photogate_t *pg)
{
	if (pg == NULL)
	{
		return PHOTOGATE_CLEAR;
	}

	return (PhotogateState_e)pg->stable_state;
}

// 查询home是否有效
bool_t PhotogateHomeValid(const Photogate_t *pg)
{
	if (pg == NULL)
	{
		return FALSE;
	}

	return (pg->home_valid != 0U) ? TRUE : FALSE;
}

// 清除home有效标志（不改偏移值）
void PhotogateClearHome(Photogate_t *pg)
{
	if (pg == NULL)
	{
		return;
	}

	pg->home_valid = 0U;
}

// 外部手动设置home
void PhotogateSetHome(Photogate_t *pg, fp32 yaw_encoder_rad)
{
	if (pg == NULL)
	{
		return;
	}

	pg->yaw_home_offset_rad = yaw_encoder_rad;
	pg->home_valid = 1U;
}

// 计算Yaw相对角：当前编码角 - home偏移
fp32 PhotogateYawRelativeRad(const Photogate_t *pg, fp32 yaw_encoder_rad)
{
	if (pg == NULL || pg->home_valid == 0U)
	{
		return 0.0f;
	}

	return yaw_encoder_rad - pg->yaw_home_offset_rad;
}

