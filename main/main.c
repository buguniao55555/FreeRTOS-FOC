#include <stdio.h>
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

void AS5600_setup();
int32_t read_data();
void read_current();
void current_sensor_setup();
// current pid
static PIDController_t g_current_q_pid;
static PIDController_t g_current_d_pid;

// 目标电流 & 实测电流
static volatile float g_i_q_ref  = 0.0f;
static volatile float g_i_d_ref  = 0.0f;
static volatile float g_i_q_meas = 0.0f;
static volatile float g_i_d_meas = 0.0f;

// PID 输出
static volatile float g_u_q = 0.0f;
static volatile float g_u_d = 0.0f;

// // 10kHz -> 100us
// static const float g_dt = 0.0001f;

// 10kHz ISR
static bool IRAM_ATTR current_loop_isr_cb(gptimer_handle_t timer,
    const gptimer_alarm_event_data_t *edata,
    void *user_ctx)
{
(void)timer;
(void)edata;
(void)user_ctx;

/* -------------------------------------------------------------------------- */
/*              TODO： 读输入（后续应该把 g_i_*_meas 用 ADC/park 的结果更新）    */
/* -------------------------------------------------------------------------- */
float iq_ref  = g_i_q_ref;
float iq_meas = g_i_q_meas;
float id_ref  = g_i_d_ref;
float id_meas = g_i_d_meas;

// 2) 误差
float err_q = iq_ref - iq_meas;
float err_d = id_ref - id_meas;

// 3) 两个 PID（同一周期内完成）
float uq = PIDController_compute(&g_current_q_pid, err_q);
float ud = PIDController_compute(&g_current_d_pid, err_d);

// 4) 保存输出
g_u_q = uq;
g_u_d = ud;

return false; // 不触发任务切换
}

// 初始化 current pid
static void init_current_Q_pid(void)
{
    const float Kp = 0.1f;
    const float Ki = 10.0f;
    const float Kd = 0.0f;

    const float ramp  = 0.0f;
    const float limit = 1.0f;   // 假设输出限幅 [-1, 1]

    PIDController_init(&g_current_q_pid, Kp, Ki, Kd, ramp, limit);

}

static void init_current_D_pid(void){
    const float Kp = 0.1f;
    const float Ki = 10.0f;
    const float Kd = 0.0f;

    const float ramp = 0.0f;
    const float limit = 1.0f;

    PIDController_init(&g_current_d_pid, Kp, Ki, Kd, ramp, limit);

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

    gptimer_event_callbacks_t cbs = {
        .on_alarm = current_loop_isr_cb,
    };
    ESP_ERROR_CHECK(gptimer_register_event_callbacks(timer, &cbs, NULL));

    gptimer_alarm_config_t alarm_cfg = {
        .reload_count = 0,
        .alarm_count = 100,      // 100 us
        .flags.auto_reload_on_alarm = true,
    };
    ESP_ERROR_CHECK(gptimer_set_alarm_action(timer, &alarm_cfg));

    ESP_ERROR_CHECK(gptimer_enable(timer));
    ESP_ERROR_CHECK(gptimer_start(timer));
    return timer;
}

void vTaskReadSensor()
{
    TickType_t xLastWakeTime;
    const TickType_t xFrequency = 10 / portTICK_PERIOD_MS;
    xLastWakeTime = xTaskGetTickCount();
    while(1)
    {
        vTaskDelayUntil(&xLastWakeTime, xFrequency);
        int64_t t0 = esp_timer_get_time();
        read_data();
        int64_t t1 = esp_timer_get_time();
        read_current();
        int64_t t2 = esp_timer_get_time();

        ESP_LOGI("TIMING", "read_data=%lld us, read_current=%lld us, total=%lld us", (long long)(t1 - t0), (long long)(t2 - t1), (long long)(t2 - t0));
    }
}

void app_main(void)
{
    printf("hello world!");
    // 初始化 current pid
    init_current_Q_pid();
    init_current_D_pid();
    // 初始化 GPTimer 10kHz 中断
    gptimer_handle_t timer = init_10khz_timer_isr();

    // 初始化 AS5600
    AS5600_setup();
    current_sensor_setup();


    static uint8_t ucParameterToPass;
    TaskHandle_t xHandle = NULL;
    xTaskCreate(vTaskReadSensor, "NAME", 4000, &ucParameterToPass, 5, &xHandle);
    configASSERT( xHandle );
}
