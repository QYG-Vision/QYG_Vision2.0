#include "pid.h"

static float pid_sign(float x)
{
    if (x > 0)
        return 1;
    else if (x < 0)
        return -1;
    else
        return 0;
}

static float __LIMIT(float x, float limit)
{
    if (x > limit)
        return  limit;
    else if (x < -limit)
        return -limit;
    else
        return x;
}

PID_StatusTypeDef PID_init(PID_param_t *PID, float kp, float ki, float kd, float alpha, float I_limit, float u_limit)
{
    PID ->pid.kd = kd;
    PID ->pid.ki = ki;
    PID ->pid.kp = kp;

    PID ->pid.Intergral_max = I_limit;
    PID ->pid.u_max         = u_limit;

    LPF_Init(&PID ->lpf, alpha);

    PID ->ref = 0;
    PID ->fdb = 0;

    PID ->Pout = 0;
    PID ->Iout  = 0;
    PID ->Dout  = 0;

    PID ->pid.err = 0;
    PID ->pid.err_last = 0;
    PID ->pid.err_last_last = 0;
   
    return PID_OK;
}

PID_CalcTypeDef PID_Calc(PID_param_t *PID, float ref, float fdb)
{
    PID ->ref = ref;
    PID ->fdb = fdb;

    PID ->pid.err = PID ->ref - PID ->fdb;

    PID ->Pout = PID ->pid.kp * PID ->pid.err;

    PID ->Iout = PID ->pid.ki * PID ->pid.err;
    __LIMIT(PID ->Iout, PID ->pid.Intergral_max);

    PID ->Dout = PID ->pid.kd * (PID ->pid.err - PID ->pid.err_last);
    PID ->Dout = LPF_Calc(&PID ->lpf, PID ->Dout);

    PID ->u    = PID ->Pout + PID ->Iout + PID ->Dout;
    PID ->u    = __LIMIT(PID ->u, PID ->pid.u_max);

    PID ->pid.err_last_last = PID ->pid.err_last;
    PID ->pid.err_last = PID ->pid.err;

    return PID ->u;
}
