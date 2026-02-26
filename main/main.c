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
#include "pidIQ.h"
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
// static PIDController_t g_current_q_pid;
// static PIDController_t g_current_d_pid;
// static PIDController_t g_position_pid;
// static PIDController_t g_velocity_pid;
static PIDControllerIQ_t g_current_q_pid;
static PIDControllerIQ_t g_current_d_pid;
static PIDControllerIQ_t g_position_pid;
static PIDControllerIQ_t g_velocity_pid;

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

// static float iq_ref;
// static const float id_ref = 0;
static _iq iq_ref;
static const _iq id_ref = _IQ(0);
static _iq iq_meas;
static _iq id_meas;
static _iq i_alpha_meas;
static _iq i_beta_meas;
static _iq err_q;
static _iq err_d;
// static float uq_list[10];
// static float ud_list[10];
static _iq uq_list[10];
static _iq ud_list[10];
static _iq iq_list[10];
static _iq id_list[10];
static _iq i_alpha_list[10];
static _iq i_beta_list[10];
static _iq ia_list[10];
static _iq ib_list[10];
volatile bool print_flag = 0;
static _iq motor_angle = 0;
static _iq motor_angle_list[10];

static _iq v_alpha;
static _iq v_beta;
static _iq v_alpha_list[10];
static _iq v_beta_list[10];

static _iq v_a;
static _iq v_b;
static _iq v_c;
static _iq v_a_list[10];
static _iq v_b_list[10];
static _iq v_c_list[10];

// // 10kHz -> 100us
// static const float g_dt = 0.0001f;


// static void init_all_pids(void)
// {
//     // 电流环（d/q）
//     const float cur_kp = 0.1f;
//     const float cur_ki = 10.0f;
//     const float cur_kd = 0.0f;
//     const float cur_ramp = 0.0f;
//     const float cur_limit = 1.0f;

//     PIDController_init(&g_current_q_pid, cur_kp, cur_ki, cur_kd, cur_ramp, cur_limit);
//     PIDController_init(&g_current_d_pid, cur_kp, cur_ki, cur_kd, cur_ramp, cur_limit);

//     // 位置环（输出：速度参考 g_v_ref，单位建议 rad/s）
//     const float pos_kp = 0.1f;
//     const float pos_ki = 10.0f;
//     const float pos_kd = 0.0f;
//     const float pos_ramp = 0.0f;
//     const float pos_limit = 1.0f;
//     PIDController_init(&g_position_pid, pos_kp, pos_ki, pos_kd, pos_ramp, pos_limit);

//     // 速度环（输出：Iq参考 g_i_q_ref）
//     const float vel_kp = 0.1f;
//     const float vel_ki = 10.0f;
//     const float vel_kd = 0.0f;
//     const float vel_ramp = 0.0f;
//     const float vel_limit = 1.0f;
//     PIDController_init(&g_velocity_pid, vel_kp, vel_ki, vel_kd, vel_ramp, vel_limit);
// }


static void init_all_pids(void)
{
    // 电流环（d/q）
    const float cur_kp = 0.1f;
    const float cur_ki = 0.0f;
    const float cur_kd = 0.0f;
    const float cur_ramp = 0.0f;
    const float cur_limit = 1.0f;

    // 电流环采样周期（比如 10kHz -> 0.0001s）
    const float cur_Ts = 0.0001f;

    PIDControllerIQ_init(&g_current_q_pid,
                         _IQ(cur_kp), _IQ(cur_ki), _IQ(cur_kd),
                         _IQ(cur_ramp), _IQ(cur_limit),
                         _IQ(cur_Ts));

    PIDControllerIQ_init(&g_current_d_pid,
                         _IQ(cur_kp), _IQ(cur_ki), _IQ(cur_kd),
                         _IQ(cur_ramp), _IQ(cur_limit),
                         _IQ(cur_Ts));

    // 位置环（输出：速度参考 g_v_ref，单位建议 rad/s）
    const float pos_kp = 0.1f;
    const float pos_ki = 0.0f;
    const float pos_kd = 0.0f;
    const float pos_ramp = 0.0f;
    const float pos_limit = 1.0f;

    // 位置环采样周期（例如 1kHz -> 0.001s）
    const float pos_Ts = 0.001f;

    PIDControllerIQ_init(&g_position_pid,
                         _IQ(pos_kp), _IQ(pos_ki), _IQ(pos_kd),
                         _IQ(pos_ramp), _IQ(pos_limit),
                         _IQ(pos_Ts));

    // 速度环（输出：Iq参考 g_i_q_ref）
    const float vel_kp = 0.1f;
    const float vel_ki = 0.0f;
    const float vel_kd = 0.0f;
    const float vel_ramp = 0.0f;
    const float vel_limit = 1.0f;

    // 速度环采样周期（例如 1kHz -> 0.001s）
    const float vel_Ts = 0.001f;

    PIDControllerIQ_init(&g_velocity_pid,
                         _IQ(vel_kp), _IQ(vel_ki), _IQ(vel_kd),
                         _IQ(vel_ramp), _IQ(vel_limit),
                         _IQ(vel_Ts));
}



/* -------------------------------------------------------------------------- */
/*                                10kHz ISR                                   */
/* -------------------------------------------------------------------------- */
static bool IRAM_ATTR current_loop_isr_cb(gptimer_handle_t timer,
    const gptimer_alarm_event_data_t *edata,
    void *user_ctx)
{

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
    static uint32_t isr_cnt = 0;
    iq_ref = 100;
    // id_ref  = g_i_d_ref;
    current_readings output = read_current();
    // ia_list[isr_cnt % 10] = _IQ(output.Ia);
    // ib_list[isr_cnt % 10] = _IQ(output.Ib);
    ia_list[isr_cnt % 10] = _IQ(2);
    ib_list[isr_cnt % 10] = _IQ(2);

    // ClarkeTransform(_IQ(output.Ia), _IQ(output.Ib), &i_alpha_meas, &i_beta_meas);
    ClarkeTransform(_IQ(2), _IQ(2), &i_alpha_meas, &i_beta_meas);
    i_alpha_list[isr_cnt % 10] = i_alpha_meas;
    i_beta_list[isr_cnt % 10] = i_beta_meas;

    ParkTransform(i_alpha_meas, i_beta_meas, motor_angle, &id_meas, &iq_meas);
    motor_angle_list[isr_cnt % 10] = motor_angle;
    
    iq_list[isr_cnt % 10] = iq_meas;
    id_list[isr_cnt % 10] = id_meas;

    // 2) 误差
    err_q = _IQ(iq_ref) - iq_meas;
    err_d = _IQ(id_ref) - id_meas;

    // 3) 两个 PID（同一周期内完成）
    _iq uq = PIDControllerIQ_compute(&g_current_q_pid, err_q);
    _iq ud = PIDControllerIQ_compute(&g_current_d_pid, err_d);

    uq_list[isr_cnt % 10] = uq;
    ud_list[isr_cnt % 10] = ud;

    InverseParkTransform(ud, uq, motor_angle, &v_alpha, &v_beta);

    v_alpha_list[isr_cnt % 10] = v_alpha;
    v_beta_list[isr_cnt % 10] = v_beta;

    InverseClarkeTransform(v_alpha, v_beta, &v_a, &v_b);
    v_c = - v_a - v_b;
    v_a_list[isr_cnt % 10] = v_a;
    v_b_list[isr_cnt % 10] = v_b;
    v_c_list[isr_cnt % 10] = v_c;

    if (isr_cnt % 10 == 9)
    {
        print_flag = 1;
    }

    ++ isr_cnt;
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
        // int64_t t0 = esp_timer_get_time();
        int raw_angle = read_data();
        motor_angle = get_angle(raw_angle);
        // int64_t t1 = esp_timer_get_time();
        // current_readings output = read_current();
        // int64_t t2 = esp_timer_get_time();

        // ESP_LOGI("TIMING", "read_data=%lld us, read_current=%lld us, total=%lld us", (long long)(t1 - t0), (long long)(t2 - t1), (long long)(t2 - t0));
        // ESP_LOGI("Current Sensor", "read_data = %ld, %ld", output.Ia, output.Ib);
    }
}

void vTaskPrintData()
{
    TickType_t xLastWakeTime;
    const TickType_t xFrequency = pdMS_TO_TICKS(100);
    xLastWakeTime = xTaskGetTickCount();
    while(1)
    {
        vTaskDelayUntil(&xLastWakeTime, xFrequency);
        if (print_flag)
        {
            print_flag = 0;
            for (int i = 0; i < 10; ++i)
            {
                ESP_LOGI("DATA", "ia = %f, ib = %f, i_alpha = %f, i_beta = %f, iq = %f, id = %f, uq = %f, ud = %f, angle = %f, v_alpha = %f, v_beta = %f, va = %f, vb = %f, vc = %f", _IQtoF(ia_list[i]), _IQtoF(ib_list[i]), _IQtoF(i_alpha_list[i]), _IQtoF(i_beta_list[i]), _IQtoF(iq_list[i]), _IQtoF(id_list[i]), _IQtoF(uq_list[i]), _IQtoF(ud_list[i]), _IQtoF(motor_angle_list[i]), _IQtoF(v_alpha_list[i]), _IQtoF(v_beta_list[i]), _IQtoF(v_a_list[i]), _IQtoF(v_b_list[i]), _IQtoF(v_c_list[i]));
            }
        }
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
    TaskHandle_t xReadSensorHandle = NULL;
    xTaskCreate(vTaskReadSensor, "ReadSensor", 4000, &ucParameterToPass, 5, &xReadSensorHandle);
    configASSERT( xReadSensorHandle );
    TaskHandle_t xPrintData = NULL;
    xTaskCreate(vTaskPrintData, "PrintData", 4000, &ucParameterToPass, 5, &xPrintData);
    configASSERT( xPrintData );
}
