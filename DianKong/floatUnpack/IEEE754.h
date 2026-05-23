#ifndef __IEEE754_H_
#define __IEEE754_H_

#ifdef __cplusplus
extern "C" {
#endif

#include "main.h"
#include "stdbool.h"
#include "math.h"

typedef struct {
    float    value;

    uint32_t sign;
    uint32_t exponent;
    uint32_t mantissa;
}ieee754_32_info_t;
extern ieee754_32_info_t unpack_ieee754_32(uint32_t x);

#ifdef __cplusplus
}
#endif

#endif
