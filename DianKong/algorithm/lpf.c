#include "lpf.h"

LPF_StatusTypeDef LPF_Init(LowPassFilter1p_Info_TypeDef *lpf, float alpha)
{
    lpf ->isInit = false;
    lpf ->alpha  = alpha;

    lpf ->input  = 0.0;
    lpf ->output = 0.0;
    return LPF_Param_Init_OK;
}

LPF_CalcTypeDef LPF_Calc(LowPassFilter1p_Info_TypeDef *lpf, float input)
{
    if(lpf ->isInit == false)
    {
        lpf ->output = input;
        lpf ->isInit = true;
    }
    else
    {
        lpf ->output = lpf ->alpha * input + (1.f - lpf ->alpha) * lpf ->output;
    }
    return lpf ->output;
}

