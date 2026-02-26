#include "pidIQ.h"

static inline _iq iq_constrain(_iq x, _iq lo, _iq hi)
{
    if (x < lo) return lo;
    if (x > hi) return hi;
    return x;
}

void PIDControllerIQ_set_Ts(PIDControllerIQ_t *pid, _iq Ts)
{
    if (!pid) {
        return;
    }
    if (Ts <= 0) {
        // Default to 1ms if caller passes 0/negative.
        Ts = _IQ(0.001f);
    }
    pid->Ts = Ts;
    pid->inv_Ts = _IQdiv(_IQ(1.0f), Ts);
}

void PIDControllerIQ_init(PIDControllerIQ_t *pid, _iq P, _iq I, _iq D, _iq ramp, _iq limit, _iq Ts)
{
    if (!pid) {
        return;
    }

    pid->P = P;
    pid->I = I;
    pid->D = D;
    pid->output_ramp = ramp;
    pid->limit = limit;

    PIDControllerIQ_set_Ts(pid, Ts);

    pid->error_prev = 0;
    pid->output_prev = 0;
    pid->integral_prev = 0;
}

_iq PIDControllerIQ_compute(PIDControllerIQ_t *pid, _iq error)
{
    // Caller must ensure pid is valid and Ts is configured.
    // Safe for ISR: all fixed-point ops, no blocking calls.

    // P term
    _iq proportional = _IQmpy(pid->P, error);

    // I term (trapezoidal integration): I += I_gain * Ts * (e + e_prev)/2
    _iq e_sum_half = _IQdiv2(error) + _IQdiv2(pid->error_prev);
    _iq i_gain_ts = _IQmpy(pid->I, pid->Ts);
    pid->integral_prev += _IQmpy(i_gain_ts, e_sum_half);

    // Integral clamp (anti-windup)
    pid->integral_prev = iq_constrain(pid->integral_prev, -pid->limit, pid->limit);

    // D term: D * (e - e_prev) / Ts  == D * (e - e_prev) * inv_Ts
    _iq derivative = 0;
    if (pid->D != 0) {
        derivative = _IQmpy(_IQmpy(pid->D, (error - pid->error_prev)), pid->inv_Ts);
    }

    // Output
    _iq output = proportional + pid->integral_prev + derivative;
    output = iq_constrain(output, -pid->limit, pid->limit);

    // Output ramp limiting (optional)
    if (pid->output_ramp > 0) {
        _iq out_rate = _IQmpy((output - pid->output_prev), pid->inv_Ts);  // output per second
        if (out_rate > pid->output_ramp) {
            output = pid->output_prev + _IQmpy(pid->output_ramp, pid->Ts);
        } else if (out_rate < -pid->output_ramp) {
            output = pid->output_prev - _IQmpy(pid->output_ramp, pid->Ts);
        }
    }

    // Save state
    pid->error_prev = error;
    pid->output_prev = output;

    return output;
}

void PIDControllerIQ_reset(PIDControllerIQ_t *pid)
{
    if (!pid) {
        return;
    }
    pid->error_prev = 0;
    pid->output_prev = 0;
    pid->integral_prev = 0;
}
