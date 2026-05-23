#include "bsp_buzzer.h"

void sentry_open(void)
{
    __HAL_TIM_PRESCALER(&htim12, 195);
    __HAL_TIM_SetCompare(&htim12, TIM_CHANNEL_1, 600);
    osDelay(250);
    __HAL_TIM_PRESCALER(&htim12, 177);
    __HAL_TIM_SetCompare(&htim12, TIM_CHANNEL_1, 600);
    osDelay(250);
    __HAL_TIM_PRESCALER(&htim12, 131);
    __HAL_TIM_SetCompare(&htim12, TIM_CHANNEL_1, 600);
    osDelay(400);
    __HAL_TIM_SetCompare(&htim12, TIM_CHANNEL_1,   0);
    osDelay(550);
}

void normal_status(uint32_t normal_tick, uint32_t tick)
{
    __HAL_TIM_PRESCALER(&htim12, normal_tick);
    __HAL_TIM_SetCompare(&htim12, TIM_CHANNEL_1, 400);
    osDelay(200);

    __HAL_TIM_SetCompare(&htim12, TIM_CHANNEL_1,   0);
    osDelay(tick);
}

void attack_status(uint32_t normal_tick, uint32_t tick)
{
    __HAL_TIM_PRESCALER(&htim12, normal_tick);
    __HAL_TIM_SetCompare(&htim12, TIM_CHANNEL_1, 600);
    osDelay(50);

  	__HAL_TIM_SetCompare(&htim12, TIM_CHANNEL_1,   0);
	osDelay(50);

	__HAL_TIM_SetCompare(&htim12, TIM_CHANNEL_1, 600);
    osDelay(100);

    __HAL_TIM_SetCompare(&htim12, TIM_CHANNEL_1,   0);
    osDelay(tick);
}
