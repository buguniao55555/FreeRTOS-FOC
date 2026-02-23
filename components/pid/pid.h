#ifndef PID_H
#define PID_H

#ifdef __cplusplus
extern "C" {
#endif

typedef struct {
    // PID 参数
    float P;              
    float I;              
    float D;              
    float output_ramp;    // 输出斜坡限制 （防止输出突变）
    float limit;          // 输出限幅 （防止输出过大）
    
    // 内部状态
    float error_prev;     // 最后的跟踪误差值
    float output_prev;    // 最后一个 pid 输出值 （防止输出爆炸）
    float integral_prev;  // 最后一个积分分量值 （防止积分饱和）
    unsigned long timestamp_prev; // 上次执行时间戳 （用于计算微分项）
} PIDController_t;

// 函数声明
void PIDController_init(PIDController_t *pid, float P, float I, float D, float ramp, float limit);
float PIDController_compute(PIDController_t *pid, float error);
void PIDController_reset(PIDController_t *pid);

#ifdef __cplusplus
}
#endif

#endif