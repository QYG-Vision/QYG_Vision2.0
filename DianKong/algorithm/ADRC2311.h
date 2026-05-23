#ifndef __ADRC2311_H_
#define __ADRC2311_H_

#ifdef __cplusplus
extern "C" {
#endif

#include "main.h"
#include "math.h"
#include "arm_math.h"

#define  SampleTime    0.001f
#define  WIDTH         0.3f

#define ADRC_POS_Flag  0x0A
#define ADRC_SPD_Flag  0x0B

#define ADRC_25_11_HandleTypedef uint8_t
#define ADRC_25_11_CalcTypedef   float
#define ADRC_25_11_MultiOutput   uint8_t

#define ADRC_25_11_OK            0x01
#define ADRC_25_11_ERROR         0x00

#define ADRC_25_11_Calc_OK       0x01
#define ADRC_25_11_Calc_ERROR    0x00

typedef struct 
{
    float   Kp;
    float   b0;
    uint8_t adrc_flag;
}ADRC2511_HandleTypeDef;

typedef struct
{
    float ref;
    float err_sef;
    float u;
    float limit;
}ADRC2511_LSEF_t;

typedef struct 
{
    float   fdb;

    float z_now[2];
    float z_pre[2];

    float beta[2];
}ADRC2511_LESO_t;

typedef struct
{
    float w0;
    float wc;

    ADRC2511_HandleTypeDef pos_param;
    ADRC2511_LSEF_t        pos_lsef;
    ADRC2511_LESO_t        pos_leso;

    ADRC2511_HandleTypeDef spd_param;
    ADRC2511_LSEF_t        spd_lsef;
    ADRC2511_LESO_t        spd_leso;

}ADRC2511_t;

static ADRC_25_11_CalcTypedef nearby_pos(float x);
static ADRC_25_11_CalcTypedef __LIMITadrc(ADRC2511_t *adrc, float u, uint8_t flag);

extern ADRC_25_11_HandleTypedef adrc_init(ADRC2511_t * adrc, float sample_time, float pos_b0, float kp_pos, float pod_limit, float wc, float spd_b0, float kp_spd, float spd_limit);

extern ADRC_25_11_CalcTypedef ADRC_LSEF_POS(ADRC2511_t *adrc, float ref);
extern ADRC_25_11_CalcTypedef ADRC_LSEF_SPD(ADRC2511_t *adrc, float ref);

extern ADRC_25_11_MultiOutput ADRC_LESO_POS(ADRC2511_t *adrc, float fdb);
extern ADRC_25_11_MultiOutput ADRC_LESO_SPD(ADRC2511_t *adrc, float fdb);

extern ADRC_25_11_CalcTypedef ADRC_25_11_Circle(ADRC2511_t *adrc, float ref, float fdb_pos, float fdb_spd);

#ifdef __cplusplus
}
#endif

#endif
