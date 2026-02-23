#include "pid.h"
#include <string.h>
#include "esp_timer.h"

#define _constrain(amt,low,high) ((amt)<(low)?(low):((amt)>(high)?(high):(amt)))

// 初始化 PID 控制器
void PIDController_init(PIDController_t *pid, float P, float I, float D, float ramp, float limit)
{
    // 设置 PID 参数
    pid->P = P;
    pid->I = I;
    pid->D = D;
    pid->output_ramp = ramp;
    pid->limit = limit;
    
    // 初始化内部状态
    pid->error_prev = 0.0f;
    pid->output_prev = 0.0f;
    pid->integral_prev = 0.0f;
    pid->timestamp_prev = esp_timer_get_time(); // 微秒
}

// 计算 PID 输出
float PIDController_compute(PIDController_t *pid, float error)
{
    // 计算时间差（秒）
    unsigned long timestamp_now = esp_timer_get_time();
    float Ts = (timestamp_now - pid->timestamp_prev) * 1e-6f;
    
    // 避免除零
    if (Ts <= 0.0f || Ts > 0.5f) {
        Ts = 1e-3f;
    }
    
    // P 项
    float proportional = pid->P * error;
    
    // I 项
    pid->integral_prev += pid->I * Ts * (error + pid->error_prev) * 0.5f;
    
    // I 项限幅（抗积分饱和）
    pid->integral_prev = _constrain(pid->integral_prev, -pid->limit, pid->limit);
    
    // 微分项
    float derivative = pid->D * (error - pid->error_prev) / Ts;
    
    // 计算输出
    float output = proportional + pid->integral_prev + derivative;
    // 输出限幅
    output = _constrain(output, -pid->limit, pid->limit);

    // 输出斜坡限制（防止突变）
    if (pid->output_ramp > 0) {
        float output_rate = (output - pid->output_prev) / Ts;
        if (output_rate > pid->output_ramp) {
            output = pid->output_prev + pid->output_ramp * Ts;
        } else if (output_rate < -pid->output_ramp) {
            output = pid->output_prev - pid->output_ramp * Ts;
        }
    }
    
    // 保存状态
    pid->error_prev = error;
    pid->output_prev = output;
    pid->timestamp_prev = timestamp_now;
    
    return output;
}

// 重置 PID 控制器
void PIDController_reset(PIDController_t *pid)
{
    pid->error_prev = 0.0f;
    pid->output_prev = 0.0f;
    pid->integral_prev = 0.0f;
    pid->timestamp_prev = esp_timer_get_time();
}
