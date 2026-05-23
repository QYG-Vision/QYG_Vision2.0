#include "ADRC2311.h"

static ADRC_25_11_CalcTypedef nearby_pos(float x)
{
    if(x >  180) x = x - 360;
    if(x < -180) x = x + 360;
    return x;
}

static ADRC_25_11_CalcTypedef __LIMITadrc(ADRC2511_t *adrc, float u, uint8_t flag)
{
    if(flag == ADRC_POS_Flag)
    {
        if(u >  adrc ->pos_lsef.limit)  u =   adrc ->pos_lsef.limit;
        if(u < -adrc ->pos_lsef.limit)  u = - adrc ->pos_lsef.limit;
    }else if (flag == ADRC_SPD_Flag)
    {
        if(u >  adrc ->spd_lsef.limit)  u =   adrc ->spd_lsef.limit;
        if(u < -adrc ->spd_lsef.limit)  u = - adrc ->spd_lsef.limit;
    }
    return u;
}

ADRC_25_11_HandleTypedef adrc_init(ADRC2511_t * adrc, float sample_time, float pos_b0, float kp_pos, float pos_limit, float wc, float spd_b0, float kp_spd, float spd_limit)
{
    adrc ->w0 = sample_time;
    adrc ->wc = wc;

    adrc ->pos_param.adrc_flag = ADRC_POS_Flag;
    adrc ->pos_param.b0        = pos_b0;
    adrc ->pos_param.Kp        = kp_pos;

    adrc ->pos_leso.beta[0]    = 2 * adrc ->wc;
    adrc ->pos_leso.beta[1]    = adrc ->wc * adrc ->wc;
    adrc ->pos_leso.fdb        = 0;
    adrc ->pos_leso.z_now[0]   = 0;
    adrc ->pos_leso.z_now[1]   = 0;
    adrc ->pos_leso.z_pre[0]   = 0;
    adrc ->pos_leso.z_pre[1]   = 0;

    adrc ->pos_lsef.err_sef    = 0;
    adrc ->pos_lsef.ref        = 0;
    adrc ->pos_lsef.u          = 0;
    adrc ->pos_lsef.limit      = pos_limit;

    adrc ->spd_param.adrc_flag = ADRC_SPD_Flag;
    adrc ->spd_param.b0        = spd_b0;
    adrc ->spd_param.Kp        = kp_spd;

    adrc ->spd_leso.beta[0]    = 2 * adrc ->wc;
    adrc ->spd_leso.beta[1]    = adrc ->wc * adrc ->wc;
    adrc ->spd_leso.fdb        = 0;
    adrc ->spd_leso.z_now[0]   = 0;
    adrc ->spd_leso.z_now[1]   = 0;
    adrc ->spd_leso.z_pre[0]   = 0;
    adrc ->spd_leso.z_pre[1]   = 0;

    adrc ->spd_lsef.err_sef    = 0;
    adrc ->spd_lsef.ref        = 0;
    adrc ->spd_lsef.u          = 0;
    adrc ->spd_lsef.limit      = spd_limit;

    return ADRC_25_11_OK;
}

ADRC_25_11_CalcTypedef ADRC_LSEF_POS(ADRC2511_t *adrc, float ref)
{
    adrc ->pos_lsef.ref = ref;
    adrc ->pos_lsef.err_sef = nearby_pos(adrc ->pos_lsef.ref - adrc ->pos_leso.z_pre[0]);
    adrc ->pos_lsef.u = ((adrc ->pos_param.Kp * adrc ->pos_lsef.err_sef) - adrc ->pos_leso.z_pre[1]) / adrc ->pos_param.b0;
    adrc ->pos_lsef.u = __LIMITadrc(adrc, adrc ->pos_lsef.u, adrc ->pos_param.adrc_flag);
    return adrc ->pos_lsef.u;
}

ADRC_25_11_CalcTypedef ADRC_LSEF_SPD(ADRC2511_t *adrc, float ref)
{
    adrc ->spd_lsef.ref = ref;
    adrc ->spd_lsef.err_sef = adrc ->spd_lsef.ref - adrc ->spd_leso.z_pre[0];
    adrc ->spd_lsef.u = ((adrc ->spd_param.Kp * adrc ->spd_lsef.err_sef) - adrc ->spd_leso.z_pre[1]) / adrc ->spd_param.b0;
    adrc ->spd_lsef.u = __LIMITadrc(adrc, adrc ->spd_lsef.u, adrc ->spd_param.adrc_flag);
    return adrc ->spd_lsef.u;
}

ADRC_25_11_MultiOutput ADRC_LESO_POS(ADRC2511_t *adrc, float fdb)
{
    adrc ->pos_leso.fdb = fdb;
    adrc ->pos_leso.z_now[0] = adrc ->pos_leso.z_pre[0] + adrc ->w0 * (adrc ->pos_param.b0 * adrc ->pos_lsef.u - adrc ->pos_leso.beta[0] * (adrc ->pos_leso.z_pre[0] - adrc ->pos_leso.fdb));
    adrc ->pos_leso.z_now[1] = adrc ->pos_leso.z_pre[1] + adrc ->w0 * (adrc ->pos_leso.beta[1] * (adrc ->pos_leso.z_pre[0] - adrc ->pos_leso.fdb));

    adrc ->pos_leso.z_pre[0] = adrc ->pos_leso.z_now[0];
    adrc ->pos_leso.z_pre[1] = adrc ->pos_leso.z_now[1];

    return ADRC_25_11_Calc_OK;
}

ADRC_25_11_MultiOutput ADRC_LESO_SPD(ADRC2511_t *adrc, float fdb)
{
    adrc ->spd_leso.fdb = fdb;
    adrc ->spd_leso.z_now[0] = adrc ->spd_leso.z_pre[0] + adrc ->w0 * (adrc ->spd_param.b0 * adrc ->spd_lsef.u - adrc ->spd_leso.beta[0] * (adrc ->spd_leso.z_pre[0] - adrc ->spd_leso.fdb));
    adrc ->spd_leso.z_now[1] = adrc ->spd_leso.z_pre[1] + adrc ->w0 * (adrc ->spd_leso.beta[1] * (adrc ->spd_leso.z_pre[0] - adrc ->spd_leso.fdb));

    adrc ->spd_leso.z_pre[0] = adrc ->spd_leso.z_now[0];
    adrc ->spd_leso.z_pre[1] = adrc ->spd_leso.z_now[1];

    return ADRC_25_11_Calc_OK;
}

ADRC_25_11_CalcTypedef ADRC_25_11_Circle(ADRC2511_t *adrc, float ref, float fdb_pos, float fdb_spd)
{
    ADRC_LESO_POS(adrc, fdb_pos);
    ADRC_LESO_SPD(adrc, fdb_spd);
    return ADRC_LSEF_SPD(adrc, ADRC_LSEF_POS(adrc, ref));
}
