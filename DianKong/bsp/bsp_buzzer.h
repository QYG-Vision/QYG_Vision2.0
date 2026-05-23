#ifndef __BSP_BUZZER_H_
#define __BSP_BUZZER_H_

#ifdef __cplusplus
extern "C" {
#endif

#include "stm32f4xx.h"
#include "tim.h"
#include "cmsis_os2.h"


void sentry_open(void);
void normal_status(uint32_t normal_tick, uint32_t tick);
void attack_status(uint32_t normal_tick, uint32_t tick);

#ifdef __cplusplus
}
#endif

#endif
