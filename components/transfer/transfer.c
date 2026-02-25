#include "transfer.h"
#include <math.h>

// // Clarke 变换：abc → αβ （2/3 系数 保持功率等价）
// // 公式：alpha = 2/3 * ia - 1/3 * ib - 1/3 * ic
// // beta = 1/sqrt(3) * (ib - ic)
// void ClarkeTransform(_iq ia, _iq ib, _iq ic, _iq *alpha, _iq *beta)
// {

//     const _iq k1 = _IQ(2.0/3.0);
//     const _iq k2 = _IQ(1.0/3.0);
//     const _iq k3 = _IQ(M_SQRT3/3.0);
//     *alpha = _IQmpy(ia, k1) - _IQmpy(ib, k2) - _IQmpy(ic, k2);
//     *beta = _IQmpy( (ib - ic), k3);
// }

// // 反Clarke变换：αβ → abc
// // 公式：ia = alpha
// // ib = -1/2 * alpha + sqrt(3)/2 * beta
// // ic = -ia - ib
// void InverseClarkeTransform(_iq alpha, _iq beta, _iq *ia, _iq *ib, _iq *ic)
// {

//     *ia = alpha;
//     *ib = _IQdiv2(-alpha + _IQmpy(beta, M_SQRT3));
//     *ic = -*ia - *ib;
// }


// 两相采样 Clarke：只用 ia, ib（隐含 ic = -ia - ib）
// alpha = ia
// beta  = (ia + 2*ib) / sqrt(3)
void Clarke2Phase(_iq ia, _iq ib, _iq *alpha, _iq *beta)
{
    *alpha = ia;

    // 1/sqrt(3)
    const _iq inv_sqrt3 = _IQ(0.5773502691896258f);

    // ia + 2*ib
    _iq sum = ia + (ib << 1);

    *beta = _IQmpy(sum, inv_sqrt3);
}

// 两相反 Clarke：从 alpha,beta 还原 ia, ib（同样隐含 ic = -ia - ib）
// ia = alpha
// ib = -0.5*alpha + (sqrt(3)/2)*beta
void InverseClarke2Phase(_iq alpha, _iq beta, _iq *ia, _iq *ib)
{
    *ia = alpha;

    // sqrt(3)/2
    const _iq sqrt3_over_2 = _IQ(0.8660254037844386f);

    *ib = _IQdiv2(-alpha) + _IQmpy(beta, sqrt3_over_2);
}

// Park 变换：αβ → dq
// 公式：d = alpha * cos(theta) + beta * sin(theta)
// q = -alpha * sin(theta) + beta * cos(theta)
void ParkTransform(_iq alpha, _iq beta, _iq theta, _iq *d, _iq *q)
{
    _iq sin = _IQsin(theta);
    _iq cos = _IQcos(theta);
    
    *d = _IQmpy(alpha, cos) + _IQmpy(beta, sin);
    *q = -_IQmpy(alpha, sin) + _IQmpy(beta, cos);
}

// 反Park变换：dq → αβ
// 公式：alpha = d * cos(theta) - q * sin(theta)
// beta = d * sin(theta) + q * cos(theta)
void InverseParkTransform(_iq d, _iq q, _iq theta, _iq *alpha, _iq *beta)
{
    _iq cos = _IQcos(theta);
    _iq sin = _IQsin(theta);
    
    *alpha = _IQmpy(d, cos) - _IQmpy(q, sin);
    *beta = _IQmpy(d, sin) + _IQmpy(q, cos);
}