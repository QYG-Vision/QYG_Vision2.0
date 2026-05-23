#ifndef __CTRL_NODE_H_
#define __CTRL_NODE_H_

#ifdef __cplusplus
extern "C" {
#endif

#include "stm32f4xx.h"
#include "can.h"

#define ctrlNodeHandle_t   uint8_t

#define Cinderella         0x202

#define CTRL_NODE_RUN_OK   0x01
#define CTRL_NODE_RUN_ERR  0x00

ctrlNodeHandle_t set_sentry_chassis_vel(float vx, float vy, float w, int16_t yaw_current);
static ctrlNodeHandle_t Pluck_FSM_fcn(void);
	
#ifdef __cplusplus
}
#endif

#endif
