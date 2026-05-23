#ifndef __LPF_H_
#define __LPF_H_

#ifdef __cplusplus
extern "C" {
#endif

#include "main.h"
#include "stm32f4xx_hal.h"
#include "math.h"
#include "stdbool.h"
#include "arm_math.h"

#define LPF_StatusTypeDef  uint8_t
#define LPF_CalcTypeDef    float

#define LPF_Param_Init_OK  0x01
#define LPF_Param_Init_ERR 0x00

typedef struct
{
    bool    isInit;
    float   alpha;
    float   input;
    float   output;
}LowPassFilter1p_Info_TypeDef;

/* 一阶低通滤波 */
extern LPF_StatusTypeDef LPF_Init(LowPassFilter1p_Info_TypeDef *lpf, float alpha);
extern LPF_CalcTypeDef LPF_Calc(LowPassFilter1p_Info_TypeDef *lpf, float input);

#ifdef __cplusplus
}
#endif

#endif
