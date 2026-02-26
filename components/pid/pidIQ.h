// IQmath (fixed-point) PID controller for ISR use (no float, no esp_timer)

#ifndef PID_IQ_H
#define PID_IQ_H

#ifdef __cplusplus
extern "C" {
#endif

#include "IQmathLib.h"

typedef struct {
    // PID parameters (IQ)
    _iq P;
    _iq I;
    _iq D;
    _iq output_ramp;   // output units per second (same unit as output / s)
    _iq limit;         // output limit (+/-)

    // fixed sample time (seconds, IQ) and its reciprocal
    _iq Ts;
    _iq inv_Ts;

    // internal state
    _iq error_prev;
    _iq output_prev;
    _iq integral_prev;
} PIDControllerIQ_t;

// Initialize with fixed sample time Ts (seconds, IQ). Ts must be > 0.
void PIDControllerIQ_init(PIDControllerIQ_t *pid, _iq P, _iq I, _iq D, _iq ramp, _iq limit, _iq Ts);

// Update sample time Ts (seconds, IQ). Ts must be > 0.
void PIDControllerIQ_set_Ts(PIDControllerIQ_t *pid, _iq Ts);

// Compute PID output for current error (IQ). Safe for ISR (no float, no blocking).
_iq PIDControllerIQ_compute(PIDControllerIQ_t *pid, _iq error);

// Reset controller internal state (keeps parameters/Ts).
void PIDControllerIQ_reset(PIDControllerIQ_t *pid);

#ifdef __cplusplus
}
#endif

#endif
