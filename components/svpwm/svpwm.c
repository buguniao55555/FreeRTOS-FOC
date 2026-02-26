#include "svpwm.h"

static const _iq M_SQRT3 = _IQ(1.7320508075688772f);
static const _iq ONE_IQ = _IQ(1.0f);
static const _iq SQRT3_IQ = _IQ(1.7320508075688772f);
static const _iq INV_SQRT3_IQ = _IQ(0.5773502691896258f);         // 1/sqrt(3)
static const _iq TWO_INV_SQRT3_IQ = _IQ(1.1547005383792517f);     // 2/sqrt(3)

static inline _iq clamp_0_1(_iq x)
{
    if (x < 0) return 0;
    if (x > ONE_IQ) return ONE_IQ;
    return x;
}

void svpwm_generate(foc_uvw_t *uvw, const foc_ab_t *ab)
{
    int sextant = 1;

    if (ab->beta >= 0) {
        if (ab->alpha >= 0) {
            // 
            sextant = (ab->beta > _IQmpy(ab->alpha, SQRT3_IQ)) ? 2 : 1;
        } else {
            //  (alpha < 0)
            sextant = (ab->beta > _IQmpy( -ab->alpha, SQRT3_IQ)) ? 2 : 3;
        }
    } else {
        if (ab->alpha >= 0) {
           // (beta < 0, alpha > 0)
            sextant = ((-ab->beta > _IQmpy(ab->alpha, SQRT3_IQ)) ? 5 : 6);
        } else {
            // (beta < 0, alpha < 0)
            sextant = (ab->beta > _IQmpy(ab->alpha, SQRT3_IQ)) ? 4 : 5;
        }
    }

    switch (sextant) {
        // sextant v1-v2
        case 1: {
            _iq t1 = -_IQmpy(ab->alpha, M_SQRT3) + ab->beta;
            _iq t2 = -_IQmpy2(ab->beta);
    
            // PWM timings
            uvw->u = _IQdiv2(_IQ(1.F) - t1 - t2);
            uvw->v = uvw->u + t1;
            uvw->w = uvw->v + t2;
        } break;
    
        // sextant v2-v3
        case 2: {
            _iq t2 = -_IQmpy(ab->alpha, _IQ(M_SQRT3)) - ab->beta;
            _iq t3 =  _IQmpy(ab->alpha, _IQ(M_SQRT3)) - ab->beta;
    
            // PWM timings
            uvw->v = _IQdiv2(_IQ(1.F) - t2 - t3);
            uvw->u = uvw->v + t3;
            uvw->w = uvw->u + t2;
        } break;
    
        // sextant v3-v4
        case 3: {
            _iq t3 = -_IQmpy2(ab->beta);
            _iq t4 = _IQmpy(ab->alpha, _IQ(M_SQRT3)) + ab->beta;
    
            // PWM timings
            uvw->v = _IQdiv2(_IQ(1.F) - t3 - t4);
            uvw->w = uvw->v + t3;
            uvw->u = uvw->w + t4;
        } break;
    
        // sextant v4-v5
        case 4: {
            _iq t4 = _IQmpy(ab->alpha, _IQ(M_SQRT3)) - ab->beta;
            _iq t5 = _IQmpy2(ab->beta);
    
            // PWM timings
            uvw->w = _IQdiv2(_IQ(1.F) - t4 - t5);
            uvw->v = uvw->w + t5;
            uvw->u = uvw->v + t4;
        } break;
    
        // sextant v5-v6
        case 5: {
            _iq t5 =  _IQmpy(ab->alpha, _IQ(M_SQRT3)) + ab->beta;
            _iq t6 = -_IQmpy(ab->alpha, _IQ(M_SQRT3)) + ab->beta;
    
            // PWM timings
            uvw->w = _IQdiv2(_IQ(1.F) - t5 - t6);
            uvw->u = uvw->w + t5;
            uvw->v = uvw->u + t6;
        } break;
    
        // sextant v6-v1
        case 6: {
            _iq t6 = _IQmpy2(ab->beta);
            _iq t1 = -_IQmpy(ab->alpha, _IQ(M_SQRT3)) - ab->beta;
    
            // PWM timings
            uvw->u = _IQdiv2(_IQ(1.F) - t6 - t1);
            uvw->w = uvw->u + t1;
            uvw->v = uvw->w + t6;
        } break;
        }

    // _iq T1 = 0, T2 = 0;
    // _iq beta_over_sqrt3 = _IQmpy(ab->beta, INV_SQRT3_IQ);

    // switch (sextant) {
    // case 1: // 0~60 : V1-V2
    //     // T1 = alpha - beta/sqrt3
    //     // T2 = 2*beta/sqrt3
    //     T1 = ab->alpha - beta_over_sqrt3;
    //     T2 = _IQmpy(ab->beta, TWO_INV_SQRT3_IQ);
    //     break;

    // case 2: // 60~120 : V2-V3
    //     // T1 = alpha + beta/sqrt3
    //     // T2 = -alpha + beta/sqrt3
    //     T1 = ab->alpha + beta_over_sqrt3;
    //     T2 = -ab->alpha + beta_over_sqrt3;
    //     break;

    // case 3: // 120~180 : V3-V4
    //     // T1 = 2*beta/sqrt3
    //     // T2 = -alpha - beta/sqrt3
    //     T1 = _IQmpy(ab->beta, TWO_INV_SQRT3_IQ);
    //     T2 = -ab->alpha - beta_over_sqrt3;
    //     break;

    // case 4: // 180~240 : V4-V5
    //     // beta<0
    //     // T1 = -alpha + beta/sqrt3
    //     // T2 = -2*beta/sqrt3   
    //     T1 = -ab->alpha + beta_over_sqrt3;
    //     T2 = -_IQmpy(ab->beta, TWO_INV_SQRT3_IQ);
    //     break;

    // case 5: // 240~300 : V5-V6
    //     // T1 = -alpha - beta/sqrt3
    //     // T2 =  alpha - beta/sqrt3
    //     T1 = -ab->alpha - beta_over_sqrt3;
    //     T2 =  ab->alpha - beta_over_sqrt3;
    //     break;

    // case 6: // 300~360 : V6-V1
    //     // T1 = -2*beta/sqrt3
    //     // T2 =  alpha + beta/sqrt3
    //     T1 = -_IQmpy(ab->beta, TWO_INV_SQRT3_IQ);
    //     T2 =  ab->alpha + beta_over_sqrt3;
    //     break;
    // }

    // // -----------------------------
    // // 3) 保护：T1/T2 不应为负；否则说明 αβ 超界/或 Clarke 定义不一致
    // // -----------------------------
    // if (T1 < 0) T1 = 0;
    // if (T2 < 0) T2 = 0;

    // _iq T0 = ONE_IQ - T1 - T2;
    // if (T0 < 0) T0 = 0; // 超过线性区时，T0 会变负；这里硬夹住避免 duty 出界

    // _iq half_T0 = _IQdiv2(T0);


    // _iq Ta=0, Tb=0, Tc=0;

    // switch (sextant) {
    // case 1:
    //     Ta = T1 + T2 + half_T0;
    //     Tb = T2 + half_T0;
    //     Tc = half_T0;
    //     break;
    // case 2:
    //     Ta = T1 + half_T0;
    //     Tb = T1 + T2 + half_T0;
    //     Tc = half_T0;
    //     break;
    // case 3:
    //     Ta = half_T0;
    //     Tb = T1 + T2 + half_T0;
    //     Tc = T2 + half_T0;
    //     break;
    // case 4:
    //     Ta = half_T0;
    //     Tb = T1 + half_T0;
    //     Tc = T1 + T2 + half_T0;
    //     break;
    // case 5:
    //     Ta = T2 + half_T0;
    //     Tb = half_T0;
    //     Tc = T1 + T2 + half_T0;
    //     break;
    // case 6:
    //     Ta = T1 + T2 + half_T0;
    //     Tb = half_T0;
    //     Tc = T1 + half_T0;
    //     break;
    // }

    // // duty clamp [0,1]
    // uvw->u = clamp_0_1(Ta);
    // uvw->v = clamp_0_1(Tb);
    // uvw->w = clamp_0_1(Tc);
}

