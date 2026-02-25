#include <stdio.h>
#include <math.h>
#include "AS5600.h"
#include "current_sensor.h"
#include "transfer.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/semphr.h"
#include "esp_err.h"
#include "esp_timer.h"
#include "esp_log.h"
#include "pid.h"
#include "esp_attr.h"
#include "driver/gptimer.h"


/* -------------------------------------------------------------------------- */
/*                                 PID频率参数                                     */
/* -------------------------------------------------------------------------- */
#define POS_DIV 50
#define VEL_DIV 10

void AS5600_setup();
int32_t read_data();
current_readings read_current();
void current_sensor_setup();
// current pid
static PIDController_t g_current_q_pid;
static PIDController_t g_current_d_pid;
static PIDController_t g_position_pid;
static PIDController_t g_velocity_pid;

// 目标位置 & 实测位置
static volatile float g_p_ref = 0.0f;
static volatile float g_p_meas = 0.0f;

// 目标速度 & 实测速度
static volatile float g_v_ref = 0.0f;
static volatile float g_v_meas = 0.0f;

// 目标电流 & 实测电流
static volatile float g_i_q_ref  = 0.0f;
static volatile float g_i_d_ref  = 0.0f;
static volatile float g_i_q_meas = 0.0f;
static volatile float g_i_d_meas = 0.0f;

// PID 输出
static volatile float g_u_q = 0.0f;
static volatile float g_u_d = 0.0f;

static float iq_ref;
static float id_ref;
static float err_q;
static float err_d;

// // 10kHz -> 100us
// static const float g_dt = 0.0001f;


static void init_all_pids(void)
{
    // 电流环（d/q）
    const float cur_kp = 0.1f;
    const float cur_ki = 10.0f;
    const float cur_kd = 0.0f;
    const float cur_ramp = 0.0f;
    const float cur_limit = 1.0f;

    PIDController_init(&g_current_q_pid, cur_kp, cur_ki, cur_kd, cur_ramp, cur_limit);
    PIDController_init(&g_current_d_pid, cur_kp, cur_ki, cur_kd, cur_ramp, cur_limit);

    // 位置环（输出：速度参考 g_v_ref，单位建议 rad/s）
    const float pos_kp = 0.1f;
    const float pos_ki = 10.0f;
    const float pos_kd = 0.0f;
    const float pos_ramp = 0.0f;
    const float pos_limit = 1.0f;
    PIDController_init(&g_position_pid, pos_kp, pos_ki, pos_kd, pos_ramp, pos_limit);

    // 速度环（输出：Iq参考 g_i_q_ref）
    const float vel_kp = 0.1f;
    const float vel_ki = 10.0f;
    const float vel_kd = 0.0f;
    const float vel_ramp = 0.0f;
    const float vel_limit = 1.0f;
    PIDController_init(&g_velocity_pid, vel_kp, vel_ki, vel_kd, vel_ramp, vel_limit);
}
/* -------------------------------------------------------------------------- */
/*                                10kHz ISR                                   */
/* -------------------------------------------------------------------------- */
static bool IRAM_ATTR current_loop_isr_cb(gptimer_handle_t timer,
    const gptimer_alarm_event_data_t *edata,
    void *user_ctx)
{
    (void)timer;
    (void)edata;
    (void)user_ctx;

    // static uint32_t pos_count = POS_DIV;
    // static uint32_t vel_count = VEL_DIV;

    // if(--pos_count == 0){
    //     pos_count = POS_DIV;

    //     float pos_ref = g_p_ref;
    //     float pos_meas = g_p_meas;
    //     float pos_err = pos_ref - pos_meas;
    //     g_v_ref = PIDController_compute(&g_position_pid, pos_err);
    // }

    // if(--vel_count == 0){
    //     vel_count = VEL_DIV;
        
    //     float vel_ref = g_v_ref;
    //     float vel_meas = g_v_meas;

    //     float vel_err = vel_ref - vel_meas;
    //     g_i_q_ref = PIDController_compute(&g_velocity_pid, vel_err);
    //     g_i_d_ref = 0.0f;
    // }
    /* -------------------------------------------------------------------------- */
    /*              TODO： 读输入（后续应该把 g_i_*_meas 用 ADC/park 的结果更新）    */
    /* -------------------------------------------------------------------------- */
    iq_ref  = g_i_q_ref;
    id_ref  = g_i_d_ref;
    current_readings output = read_current();
    output.Ia;

    // 2) 误差
    err_q = iq_ref - iq_meas;
    err_d = id_ref - id_meas;

    // 3) 两个 PID（同一周期内完成）
    // float uq = PIDController_compute(&g_current_q_pid, err_q);
    // float ud = PIDController_compute(&g_current_d_pid, err_d);

    // 4) 保存输出
    // g_u_q = uq;
    // g_u_d = ud;
    // int64_t t0 = esp_timer_get_time();
    // read_data();
    // int64_t t1 = esp_timer_get_time();
    // current_readings output = read_current();
    // int64_t t2 = esp_timer_get_time();

    // ESP_LOGI("TIMING", "read_data=%lld us, read_current=%lld us, total=%lld us", (long long)(t1 - t0), (long long)(t2 - t1), (long long)(t2 - t0));
    // ESP_LOGI("Current Sensor", "read_data = %ld, %ld", output.Ia, output.Ib);

    return false; // 不触发任务切换
}




// 初始化 GPTimer 10kHz 中断
static gptimer_handle_t init_10khz_timer_isr(void)
{   
    
    gptimer_handle_t timer = NULL;

    // 1MHz 分辨率 => 1 tick = 1 us，10kHz => alarm_count = 100
    gptimer_config_t tcfg = {
        .clk_src = GPTIMER_CLK_SRC_DEFAULT,
        .direction = GPTIMER_COUNT_UP,
        .resolution_hz = 1000000, // 1 MHz
    };
    ESP_ERROR_CHECK(gptimer_new_timer(&tcfg, &timer));

    gptimer_alarm_config_t alarm_cfg = {
        .reload_count = 0,
        .alarm_count = 100,      // 100 us
        .flags.auto_reload_on_alarm = true,
    };
    ESP_ERROR_CHECK(gptimer_set_alarm_action(timer, &alarm_cfg));

    gptimer_event_callbacks_t cbs = {
        .on_alarm = current_loop_isr_cb,
    };
    ESP_ERROR_CHECK(gptimer_register_event_callbacks(timer, &cbs, NULL));

    ESP_ERROR_CHECK(gptimer_enable(timer));
    ESP_ERROR_CHECK(gptimer_start(timer));
    return timer;
}

void vTaskReadSensor()
{
    TickType_t xLastWakeTime;
    const TickType_t xFrequency = pdMS_TO_TICKS(10);
    xLastWakeTime = xTaskGetTickCount();
    while(1)
    {
        vTaskDelayUntil(&xLastWakeTime, xFrequency);
        int64_t t0 = esp_timer_get_time();
        read_data();
        int64_t t1 = esp_timer_get_time();
        current_readings output = read_current();
        int64_t t2 = esp_timer_get_time();

        ESP_LOGI("TIMING", "read_data=%lld us, read_current=%lld us, total=%lld us", (long long)(t1 - t0), (long long)(t2 - t1), (long long)(t2 - t0));
        ESP_LOGI("Current Sensor", "read_data = %ld, %ld", output.Ia, output.Ib);
    }
}

void app_main(void)
{
    printf("hello world!");
    // 初始化 current pid
    // init_current_Q_pid();
    // init_current_D_pid();

    // 初始化 AS5600
    AS5600_setup();
    current_sensor_setup();

    // 初始化 PID（位置/速度/电流）
    init_all_pids();

    // 启动 10kHz 单 ISR（电流环每次跑；位置/速度用分频）
    (void)init_10khz_timer_isr();


    static uint8_t ucParameterToPass;
    TaskHandle_t xHandle = NULL;
    // xTaskCreate(vTaskReadSensor, "NAME", 4000, &ucParameterToPass, 5, &xHandle);
    // configASSERT( xHandle );
}
