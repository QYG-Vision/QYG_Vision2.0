#ifndef __BSP_WIT_H_
#define __BSP_WIT_H_

#ifdef __cplusplus
extern "C" {
#endif

#include "stm32f4xx.h"
#include "math.h"
#include "arm_math.h"
#include "usart.h"
#include "dma.h"

#define Omega        0x52
#define Euler_Angle  0x53
#define Quaternion   0x59

typedef struct
{
    uint8_t Rxfifo[11];
    uint8_t frame_Header;
    uint8_t frame_ID;
    uint8_t Check_Sum;
    uint8_t data[8];
}wit_rx_info_t;

typedef struct
{
    wit_rx_info_t rx_info;

    float omega[3];
    float euler_angle[3];
    float quaternion[4];
}wit_imu_t;

#ifdef __cplusplus
}
#endif

#endif
