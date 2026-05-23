#include "NLADRC_ver1.h"

/* NLADRC code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
static float fhan(float x1, float x2, float r, float h)
{
    float d,d0,y,a0,a;

    d  = r * h;
    d0 = h * d;
    y  = x1 + h * x2;
    a0 = sqrt(d0 * d0 + 8.0 * r * fabs(y));

    if(fabs(y) > d0)
    {
        a = x2 + 0.5 * (a0 - d) * (y / fabs(y));
    }else
    {
        a = a0 + y / h;
    }

    if(fabs(a) > d)
    {
        return -r * (a / fabs(a));
    }else
    {
        return -r * (a / d);
    }
}

static float fal(float e, float alpha, float delta)
{
    if(fabs(e) <= delta)
    {
        return e / (powf(delta, (1 - alpha)));
    }else
    {
        return (fabs(e) * alpha) * (e / fabs(e));
    }
}

NLADRC_HandleTypedef NLADRC_init(nladrc_t *adrc, float b0, 
                                                 float r, 
                                                 float h, 
                                                 float w0, 
                                                 float wc, 
                                                 float alpha, 
                                                 float delta
                                                )
{
    adrc->param.b0    = b0;
    adrc->param.r     = r;
    adrc->param.h     = h;
    adrc->param.w0    = w0;
    adrc->param.wc    = wc;
    adrc->param.alpha = alpha;
    adrc->param.delta = delta;

    adrc ->TD.ref     = 0;
    adrc ->TD.v1      = 0;
    adrc ->TD.v2      = 0;

    adrc ->NLSEF.u0   = 0;
    adrc ->NLSEF.u    = 0;
    
    adrc ->NLESO.fdb  = 0;
    adrc ->NLESO.z1   = 0;
    adrc ->NLESO.z2   = 0;
    adrc ->NLESO.z3   = 0;

    return NLADRC_func_OK;
}

NLADRC_HandleTypedef NLADRC_TD(nladrc_t *adrc, float ref)
{
    adrc ->TD.ref = ref;

    adrc ->TD.v1 += adrc ->param.h * adrc ->TD.v2;
    adrc ->TD.v2 += adrc ->param.h * fhan(adrc ->TD.v1 - adrc ->TD.ref, adrc ->TD.v2, adrc ->param.r, adrc ->param.h);

    return NLADRC_func_OK;
}

NLADRC_HandleTypedef NLADRC_NLSEF_angle(nladrc_t *adrc)
{
    float err1,err2;
    float beta[2];

    err1 = adrc ->TD.v1 - adrc ->NLESO.z1;
    err2 = adrc ->TD.v2 - adrc ->NLESO.z2;

    err1 = (err1 > 180? err1 - 360 : ( err1 < -180 ? err1 + 360 : err1));

    beta[0] = adrc ->param.wc * adrc ->param.wc;
    beta[1] = 2 * adrc ->param.wc;

    adrc ->NLSEF.u0 =  beta[0] * fal(err1, adrc ->param.alpha, adrc ->param.delta) + beta[1] * fal(err2, adrc ->param.alpha, adrc ->param.delta);

    adrc ->NLSEF.u  = (adrc ->NLSEF.u0 - adrc ->NLESO.z3) / adrc ->param.b0;

    return NLADRC_func_OK;
}

NLADRC_HandleTypedef NLADRC_NLESO(nladrc_t *adrc, float fdb)
{
    adrc ->NLESO.fdb = fdb;

    float error;

    float beta1, beta2, beta3;

    error = adrc ->NLESO.z1 - adrc ->NLESO.fdb;

    beta1 = 3  * adrc ->param.w0;
    beta2 = 3  * adrc ->param.w0 * adrc ->param.w0;
    beta3 = 1  * adrc ->param.w0 * adrc ->param.w0 * adrc ->param.w0;

    adrc ->NLESO.z1 += adrc ->param.h * (adrc ->NLESO.z2 - beta1 * error                                   );
    adrc ->NLESO.z2 += adrc ->param.h * (adrc ->NLESO.z3 - beta2 * error + adrc ->param.b0 * adrc ->NLSEF.u);
    adrc ->NLESO.z3 += adrc ->param.h * (                - beta3 * error                                   );

    return NLADRC_func_OK;
}

NLADRC_HandleTypedef NLADRC(nladrc_t *adrc, float ref, float fdb)
{
    NLADRC_TD(adrc, ref);
    NLADRC_NLSEF_angle(adrc);
    NLADRC_NLESO(adrc, fdb);

    return NLADRC_func_OK;
}

/* USER CODE END 0 */
