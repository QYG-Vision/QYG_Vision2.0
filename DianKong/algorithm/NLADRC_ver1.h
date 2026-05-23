#ifndef __NLADRC_VER1_H_
#define __NLADRC_VER1_H_

#ifdef __cplusplus
extern "C" {
#endif

#include "main.h"
#include "math.h"
#include "arm_math.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "stm32f4xx_hal.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */
typedef struct
{
    float b0;
    float r;
    float h;
    float w0;
    float wc;
    float alpha;
    float delta;
}nladrc_param_t;

typedef struct
{
    float ref;
    float v1;
    float v2;
}nladrc_TD_t;

typedef struct
{
    float u0;
    float u;  
}nladrc_SEF_t;

typedef struct
{
    float fdb;
    float z1;
    float z2;
    float z3;
}nladrc_ESO_t;

typedef struct
{
    nladrc_param_t param;
    nladrc_TD_t    TD;
    nladrc_SEF_t   NLSEF;
    nladrc_ESO_t   NLESO;
}nladrc_t;
/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define NLADRC_HandleTypedef uint8_t

#define NLADRC_func_OK       0x01
#define NLADRC_func_ERROR    0x00
/* USER CODE END PD */

static float fhan(float x1, float x2, float r, float h);
static float fal(float e, float alpha, float delta);

extern NLADRC_HandleTypedef NLADRC_init(nladrc_t *adrc, float b0, float r, float h, float w0,float wc, float alpha, float delta);
extern NLADRC_HandleTypedef NLADRC_TD(nladrc_t *adrc, float ref);
extern NLADRC_HandleTypedef NLADRC_NLSEF_angle(nladrc_t *adrc);
extern NLADRC_HandleTypedef NLADRC_NLESO(nladrc_t *adrc, float fdb);

extern NLADRC_HandleTypedef NLADRC(nladrc_t *adrc, float ref, float fdb);

#ifdef __cplusplus
}
#endif

#endif
