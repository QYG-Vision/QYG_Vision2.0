#ifndef __BSP_CAN_H_
#define __BSP_CAN_H_

#ifdef __cplusplus
extern "C" {
#endif

#include "main.h"
#include "can.h"

#define M2006_ID1       0x201
#define M2006_ID2       0x202
#define M2006_ID3       0x203
#define M2006_ID4       0x204
#define M2006_ID5       0x205
#define M2006_ID6       0x206
#define M2006_ID7       0x207
#define M2006_ID8       0x208

#define M3508_ID1       0x201
#define M3508_ID2       0x202
#define M3508_ID3       0x203
#define M3508_ID4       0x204
#define M3508_ID5       0x205
#define M3508_ID6       0x206
#define M3508_ID7       0x207
#define M3508_ID8       0x208

#define GM6020_ID1      0x205
#define GM6020_ID2      0x206
#define GM6020_ID3      0x207
#define GM6020_ID4      0x208
#define GM6020_ID5      0x209
#define GM6020_ID6      0x20A
#define GM6020_ID7      0x20B

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define Yaw             GM6020_ID5
#define Pitch           GM6020_ID1
#define r_fric          M3508_ID1
#define l_fric          M3508_ID2
#define Pluck           M2006_ID1

#define Shining_Colors  0x283
#define chassis2        0x200
#define chassis1        0x203
#define chassis         0x208

#define GM6020          0xAA
#define M3508           0xBB
#define M2006           0xCC

#define Current_MAX_Enc 16384
#define Current_MAX     20.0f

#define Pitch_err       0x02
#define Pitch_normal    0x01
/* USER CODE END PD */

typedef struct
{
    uint16_t angle_enc;
    int16_t  speed_enc;
    int16_t  torque_enc;
    int16_t  temp;
}Motor_info_enc;

typedef struct
{
    float angle;
    float speed;
    float torqueCurrent;
    float temp;
}Motor_info_real;

typedef struct
{
    Motor_info_enc  enc_info;
    Motor_info_real real_info;
    uint8_t         motorType;
    uint8_t         motor_FSM;

    uint32_t        Ctrl_Freq;
}Motor_info_t;

typedef struct
{
		uint8_t  id;
		uint8_t  game_status;
		uint16_t HP;
		uint16_t heat;
}sentry_status_t;	

void can1_filter_init(void);
void can2_filter_init(void);

static void Motor_info_Updata(Motor_info_t *motor, uint8_t data[8]);
static void enc2real(Motor_info_t *motor);

#ifdef __cplusplus
}
#endif


#endif
