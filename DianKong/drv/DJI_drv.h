#ifndef __DJI_DRV_H_
#define __DJI_DRV_H_

#ifdef __cplusplus
extern "C" {
#endif
	
#include "stm32f4xx.h"
#include "can.h"

#define  DJI_StatusType  uint8_t

#define  DJI_OK          0x01
#define  DJI_ERROR       0x00

DJI_StatusType YAW_ctrl(int16_t yaw_current);
DJI_StatusType PITCH_ctrl(int16_t pitch_current);
DJI_StatusType FIRE_ctrl(int16_t current1, int16_t current2);
DJI_StatusType GET_FIRE_CMD(int16_t pluck_current);
	
#ifdef __cplusplus
}
#endif


#endif
