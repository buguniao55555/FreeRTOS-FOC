#pragma once

#ifdef __cplusplus
extern "C" {
#endif

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
 * @param du Duty for phase U in range [0.0, 1.0]
 * @param dv Duty for phase V in range [0.0, 1.0]
 * @param dw Duty for phase W in range [0.0, 1.0]
 */
void pwm_3phase_set_duty(float du, float dv, float dw);

#ifdef __cplusplus
}
#endif

