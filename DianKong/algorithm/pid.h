#ifndef __PID_H_
#define __PID_H_

#ifdef __cplusplus
extern "C" {
#endif

#include "main.h"
#include "math.h"
#include "arm_math.h"
#include "lpf.h"

#define PID_StatusTypeDef uint8_t
#define PID_CalcTypeDef   float

#define PID_OK            0x01
#define PID_ERROR         0x00

typedef struct {
    float kp;
    float ki;
    float kd;

    float err;
    float err_last;
    float err_last_last;

    float Intergral_max;
    float u_max;
}__PID_Typedef;

typedef struct {
    float Pout;
    float Iout;
    float Dout;

    float u;

    float ref;
    float fdb;

    __PID_Typedef pid;
    LowPassFilter1p_Info_TypeDef lpf;
}PID_param_t;

extern PID_StatusTypeDef PID_init(PID_param_t *PID, float kp, float ki, float kd, float alpha, float I_limit, float u_limit);
extern PID_CalcTypeDef PID_Calc(PID_param_t *PID, float ref, float fdb);

static float pid_sign(float x);
static float __LIMIT(float x, float limit);
	
#ifdef __cplusplus
}
#endif

#endif
