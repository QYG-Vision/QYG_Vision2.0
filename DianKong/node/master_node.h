#ifndef __MASTER_NODE_H_
#define __MASTER_NODE_H_

#ifdef __cplusplus
extern "C" {
#endif
	
#include "stm32f4xx.h"
#include "main.h"
#include "math.h"
#include "arm_math.h"
#include "FreeRTOS.h"
#include "cmsis_os2.h"
#include "task.h"
#include "queue.h"

#include "auto_mode.h"

#define tick_t                uint32_t

#define sentry_pitch_min      100.0f
#define sentry_pitch_mid      115.0f
#define sentry_pitch_max      130.0f

#define sentry_abs_zero_pitch 125.0f

typedef struct 
{
    float shoot_vel;
    float pluck_vel;
    
    float chassis_vel_x;
    float chassis_vel_y;

    float chassis_omega;

    float yaw_add_angle;
    float pitch_angle;

}_sentry_remote_data_t;



typedef struct
{
    float shoot_vel;
    float pluck_vel;
    
    float chassis_vel_x;
    float chassis_vel_y;

    float chassis_omega;

    float yaw_add_angle;
    float pitch_angle;

}_sentry_data_t;

typedef struct 
{
    float vel_for_World[2];
    float vel_for_Gimbal[2];
}Gimbal_Rot_to_World_t;


static float uint_to_float(int X_int, float X_min, float X_max, int Bits);
float GM6020_angle_Rot(float angle, float init_angle);
float GM6020_angle_Rot_inv(float angle, float init_angle);

#ifdef __cplusplus
}
#endif

#endif
