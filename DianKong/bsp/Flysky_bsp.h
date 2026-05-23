#ifndef __FLYSKY_BSP_H_
#define __FLYSKY_BSP_H_

#ifdef __cplusplus
extern "C" {
#endif
	
#include "stm32f4xx.h"
#include "math.h"
#include "arm_math.h"

////
/*
--------------------------------哨兵遥控器功能--------------------------------
-----------------左摇杆：哨兵底盘前后左右（遥控模式）
-----------------右摇杆：哨兵云台上下左右（yaw、pitch，遥控模式）
-----------------左上拨杆：哨兵摩擦轮开关（遥控模式）
-----------------左中拨杆：哨兵拨弹轮开关（遥控模式）
-----------------右中拨杆：哨兵模式开关：失能模式、遥控模式、自动化模式
-----------------右上拨杆：哨兵小陀螺开关（遥控模式）
-----------------左旋扭：哨兵底盘灵敏度
-----------------右旋扭：哨兵云台灵敏度
-----------------------------------------------------------------------------
*/

#define BUFF_SIZE	   25
#define sbus_OK      0x01
#define sbus_ERR     0x00
#define mapping_OK   0x01
#define mapping_ERR  0x00

#define Flysky_max   1807
#define Flysky_mid   1024
#define Flysky_min    240

#define switch_ON       1
#define switch_OFF      0

#define DISABLE_MODE    0
#define REMOTE_MODE     1
#define AUTOMATION_MODE 2

typedef struct 
{
    uint8_t sentry_sbus_rx[BUFF_SIZE];
    int16_t channel[16];
    uint8_t sbus_check;
}remote_channel_t;

typedef struct 
{
    float spd_limit;
    float spd_factor;
    float spd;
}speed_t;

typedef struct
{
    float yaw_add_factor;
    float pitch_factor;

    float gimbal_ctrl_param;

    float yaw_add;
    float pitch_angle;
}gimbal_factor_t;

typedef struct 
{
    speed_t  spd_X;
    speed_t  spd_Y;

    float chassis_omega;

    gimbal_factor_t gimbal_factor;

    uint8_t friction_wheel_switch;
    uint8_t pluck_wheel_switch;
    uint8_t mode_switch;
    uint8_t gyro_switch;

    float chassis_sensitivity;
    float gimbal_sensitivity;
}sentry_ctrl_t;

typedef struct 
{
    remote_channel_t channel;
    sentry_ctrl_t    ctrl;

    float            epsilon;

    uint8_t sentry_remote_mapping_flag;
}sentry_remote;

void Flysky_init(sentry_remote *sentry_Rm,float spd_X_max, float spd_Y_max, float eps, float gimbal_param);
void sbus_data_unpacking(sentry_remote *rxdata,  uint8_t *inbuffer);
void sentry_remote_mapping(sentry_remote *RM);
	
#ifdef __cplusplus
}
#endif


#endif
