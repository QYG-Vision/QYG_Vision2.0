#ifndef __SMC_H_
#define __SMC_H_

#ifdef __cplusplus
extern "C" {
#endif

#include "stm32f4xx_hal.h"
#include "arm_math.h"

#define smc_sign(x)          ((x) < 0.0f ? -1.0f : 1.0f)

#define NTSMC  0xAA

#define Sliding_model_handle uint8_t
#define Sliding_model_calc   float

#define SMC_OK               0x01
#define SMC_ERR              0x00

#define NEARBY_POS(x) (x > 180? x - 360 : (x < -180? x + 360 : x))
#define Limit(x, max) (x > max? max : (x < -max? -max : x))

//#define dt                   0.001

#define SMC_1KHz             1000.f
#define SMC_500Hz             500.f
#define SMC_250Hz             250.f
#define SMC_200Hz             200.f
#define SMC_100Hz             100.f
#define SMC_50Hz               50.f
#define SMC_25Hz               25.f
#define SMC_10Hz               10.f
#define SMC_5Hz                 5.f
#define SMC_1Hz                 1.f

typedef struct {
    float K;
    float J;

    float a;
    float b;

    float p;
    float q;

    float m;
    float n;

    float k_current;
    float alpha;

    float G_out;
}ntsmc_param_t;

typedef struct {
    ntsmc_param_t ntsmc_t;
    float ref;
    float ref_last;
    float dref;
    float pos_err;
    float spd_err;

    float sliding_surface;

    float u;
    float u_limit;
}smc_param_t;

static inline float fast_pow(float x, float y);
static inline float fast_tanh(float x);
static inline float fast_fabs(float x);

static Sliding_model_calc Differential_calc(smc_param_t *smc);
static Sliding_model_calc pos_err_Update(float ref, float fdb);
static Sliding_model_calc spd_err_Update(float ref, float fdb);

extern void ntsmc_init(ntsmc_param_t *smc);
extern void ntsmc_param_set(ntsmc_param_t *smc, float K, float J, float a, float b, float p, float q, float m, float n, float k_current, float alpha);

Sliding_model_handle NTSMC_Sliding_Surface_set(smc_param_t *smc, float ref, float fdb_pos, float fdb_spd);

void smc_init(smc_param_t *smc, float u_limit);
extern Sliding_model_handle Sliding_Surface_set(smc_param_t *smc, float ref, float fdb_pos, float fdb_spd, uint8_t Sliding_surface_mode);
extern Sliding_model_handle SMC_Controller(smc_param_t *smc, uint8_t Sliding_surface_mode);

extern Sliding_model_handle SMC_Update(smc_param_t *smc, float ref, float fdb_pos, float fdb_spd, uint8_t Sliding_surface_mode);

#ifdef __cplusplus
}
#endif

#endif
