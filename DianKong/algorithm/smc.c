#include "smc.h"

// 添加快速数学函数
static inline float fast_pow(float x, float y) {
    // 使用对数-指数近似的快速幂计算
    return expf(y * logf(x));
}

static inline float fast_tanh(float x) {
    // 快速tanh近似，比标准库快3-5倍
    float x2 = x * x;
    return x * (27.0f + x2) / (27.0f + 9.0f * x2);
}

static inline float fast_fabs(float x) {
    // 使用位操作快速绝对值
    uint32_t tmp = *(uint32_t*)&x;
    tmp &= 0x7FFFFFFF;
    return *(float*)&tmp;
}

void ntsmc_init(ntsmc_param_t *smc)
{
    smc ->K         = 0;
    smc ->J         = 0;
    smc ->a         = 0;
    smc ->b         = 0;
    smc ->p         = 0;
    smc ->q         = 0;
    smc ->m         = 0;
    smc ->n         = 0;

    smc ->k_current = 0;
    smc ->alpha     = 0;
}

void smc_init(smc_param_t *smc, float u_limit)
{
    smc ->u_limit = u_limit;

    smc ->ref = 0;
    smc ->ref_last = 0;

    smc ->pos_err = 0;
    smc ->spd_err = 0;
    smc ->dref    = 0;

    smc ->sliding_surface = 0;
    smc ->u = 0;
}

void ntsmc_param_set(ntsmc_param_t *smc, float K, float J, float a, float b, float p, float q, float m, float n, float k_current, float alpha)
{
    smc ->K         = K;
    smc ->J         = J;
    smc ->a         = a;
    smc ->b         = b;
    smc ->p         = p;
    smc ->q         = q;
    smc ->m         = m;
    smc ->n         = n;
    smc ->k_current = k_current;
    smc ->alpha     = alpha;
}

static Sliding_model_calc Differential_calc(smc_param_t *smc)
{
    return (smc ->ref - smc ->ref_last);
}

static Sliding_model_calc pos_err_Update(float ref, float fdb)
{
    return NEARBY_POS(fdb - ref);
}

static Sliding_model_calc spd_err_Update(float ref, float fdb)
{
    return fdb - ref;
}

Sliding_model_handle NTSMC_Sliding_Surface_set(smc_param_t *smc, float ref, float fdb_pos, float fdb_spd)
{
    smc ->ref = ref;

    smc ->pos_err = pos_err_Update(ref       , fdb_pos);

    smc ->dref    = Differential_calc(smc);

    smc ->spd_err = spd_err_Update(smc ->dref, fdb_spd);

    smc ->ref_last = smc ->ref;

    smc ->sliding_surface = smc ->spd_err + smc ->ntsmc_t.a * smc_sign(smc ->pos_err) * fast_pow(fast_fabs(smc ->pos_err), smc ->ntsmc_t.p / smc ->ntsmc_t.q) 
                                          + smc ->ntsmc_t.b * smc_sign(smc ->pos_err) * fast_pow(fast_fabs(smc ->pos_err), smc ->ntsmc_t.m / smc ->ntsmc_t.n);

    smc ->ntsmc_t.G_out =   smc ->ntsmc_t.a * (smc ->ntsmc_t.p / smc ->ntsmc_t.q) * smc_sign(smc ->pos_err) * fast_pow(fast_fabs(smc ->pos_err), ((smc ->ntsmc_t.p / smc ->ntsmc_t.q) - 1)) 
                          + smc ->ntsmc_t.b * (smc ->ntsmc_t.m / smc ->ntsmc_t.n) * smc_sign(smc ->pos_err) * fast_pow(fast_fabs(smc ->pos_err), ((smc ->ntsmc_t.m / smc ->ntsmc_t.n) - 1));

    return SMC_OK;
}

Sliding_model_handle Sliding_Surface_set(smc_param_t *smc, float ref, float fdb_pos, float fdb_spd, uint8_t Sliding_surface_mode)
{
    switch (Sliding_surface_mode)
    {
        case NTSMC :
        {
            NTSMC_Sliding_Surface_set(smc, ref, fdb_pos, fdb_spd);
            break;
        }
        default : {
            break;
        }
    }
    return SMC_OK;
}

Sliding_model_handle SMC_Controller(smc_param_t *smc, uint8_t Sliding_surface_mode)
{
    switch (Sliding_surface_mode)
    {
        case NTSMC :
        {
            smc ->u = (smc ->ntsmc_t.J / smc ->ntsmc_t.k_current) * (0.f - smc ->spd_err * smc ->ntsmc_t.G_out 
                                                                         - smc ->ntsmc_t.K * smc ->sliding_surface 
                                                                         - smc ->ntsmc_t.K * fast_pow(fast_fabs(smc ->sliding_surface), smc ->ntsmc_t.alpha * fast_tanh(smc ->sliding_surface)));
            smc ->u = Limit(smc ->u, smc ->u_limit);
            break;
        }
        default    : 
        {
            break;
        }
    }
    return SMC_OK;
}

Sliding_model_handle SMC_Update(smc_param_t *smc, float ref, float fdb_pos, float fdb_spd, uint8_t Sliding_surface_mode)
{
    Sliding_Surface_set(smc, ref, fdb_pos, fdb_spd, Sliding_surface_mode);
    if(SMC_Controller(smc, Sliding_surface_mode) != SMC_OK) return SMC_ERR;
    else return SMC_OK;
}

