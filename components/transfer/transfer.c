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

// Clarke 变换：abc → αβ （2/3 系数 保持功率等价）
// 公式：alpha = 2/3 * ia - 1/3 * ib - 1/3 * ic
// beta = 1/sqrt(3) * (ib - ic)
void ClarkeTransform(_iq ia, _iq ib, _iq ic, _iq *alpha, _iq *beta)
{

    const _iq k1 = _IQ(2.0/3.0);
    const _iq k2 = _IQ(1.0/3.0);
    const _iq k3 = _IQ(M_SQRT3/3.0);
    *alpha = _IQmpy(ia, k1) - _IQmpy(ib, k2) - _IQmpy(ic, k2);
    *beta = _IQmpy( (ib - ic), k3);
}

// 反Clarke变换：αβ → abc
// 公式：ia = alpha
// ib = -1/2 * alpha + sqrt(3)/2 * beta
// ic = -ia - ib
void InverseClarkeTransform(_iq alpha, _iq beta, _iq *ia, _iq *ib, _iq *ic)
{

    *ia = alpha;
    *ib = _IQdiv2(-alpha + _IQmpy(beta, M_SQRT3));
    *ic = -*ia - *ib;
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