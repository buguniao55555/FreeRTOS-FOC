#pragma once

#ifdef __cplusplus
extern "C" {
#endif

#include "IQmathLib.h"

/**
 * @brief Initialize 3-phase PWM outputs (U/V/W) using MCPWM.
 *
 * This function configures the MCPWM timer/operators/comparators/generators
 * and starts the timer.
 */
void pwm_3phase_init(void);

/**
 * @brief Set 3-phase PWM duty cycles.
 *
 * Duty is in _iq format, expected range [0, 1] (Q = GLOBAL_IQ).
 *
 * @param du Duty for phase U in range [0, 1] (IQ)
 * @param dv Duty for phase V in range [0, 1] (IQ)
 * @param dw Duty for phase W in range [0, 1] (IQ)
 */
void pwm_3phase_set_duty_iq(_iq du, _iq dv, _iq dw);

#ifdef __cplusplus
}
#endif

