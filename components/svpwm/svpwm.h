#ifndef SVPWM_H
#define SVPWM_H

#ifdef __cplusplus
extern "C" {
#endif

#include "math.h"
#include "IQmathLib.h"

# define PWM_PIN_1 GPIO_NUM_32
# define PWM_PIN_2 GPIO_NUM_33
# define PWM_PIN_3 GPIO_NUM_25

typedef struct foc_uvw_t{
    _iq u;
    _iq v;
    _iq w;
}foc_uvw_t;

typedef struct foc_ab_t{
    _iq alpha;
    _iq beta;
}foc_ab_t;

void svpwm_generate(foc_uvw_t *uvw, const foc_ab_t *ab);

#ifdef __cplusplus
}
#endif

#endif