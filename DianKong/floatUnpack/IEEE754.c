#include "IEEE754.h"

ieee754_32_info_t unpack_ieee754_32(uint32_t x)
{
    ieee754_32_info_t info;

    info.sign = (x >> 31) & 1;

    info.exponent = (x >> 23) & 0xFF;

    info.mantissa = x & 0x7FFFFF;

    if (info.exponent == 0xFF) {
        if(info.mantissa == 0)
        {
            info.value = info.sign ? -HUGE_VALF : HUGE_VALF; 
        }else{
            info.value = 0.0f / 0.0f;
        }
    }else if (info.exponent == 0)
    {
        if(info.mantissa == 0)
        {
            info.value = info.sign ? -0.0 : 0.0;
        }else{
            info.value =  (info.sign ? -1.0f : 1.0f) * (info.mantissa / (float)(1 << 23)) * powf(2.0f, -126);
        }
    }else {  
        float mantissa_with_leading = (info.mantissa | (1 << 23)) / (float)(1 << 23);

        info.value = (info.sign ? -1.0f : 1.0f) * mantissa_with_leading * powf(2.0f, info.exponent - 127);
    }

    return info;
}
