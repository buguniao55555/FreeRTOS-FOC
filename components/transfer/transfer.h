#include "IQmathLib.h"

#ifndef TRANSFORM_H
#define TRANSFORM_H

#ifdef __cplusplus
extern "C" {
#endif

// Clarke 变换：三相 (abc) → 两相 (αβ)
void ClarkeTransform(_iq ia, _iq ib, _iq *alpha, _iq *beta);

// 反Clarke变换：两相 (αβ) → 三相 (abc)
void InverseClarkeTransform(_iq alpha, _iq beta, _iq *ia, _iq *ib);

// Park 变换：静止坐标系 (αβ) → 旋转坐标系 (dq)
void ParkTransform(_iq alpha, _iq beta, _iq theta, _iq *d, _iq *q);

// 反Park变换：旋转坐标系 (dq) → 静止坐标系 (αβ)
void InverseParkTransform(_iq d, _iq q, _iq theta, _iq *alpha, _iq *beta);

#ifdef __cplusplus
}
#endif

#endif

