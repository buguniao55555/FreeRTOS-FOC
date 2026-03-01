#include "driver/mcpwm_timer.h"
#include "driver/mcpwm_prelude.h"
#include "esp_check.h"
#include "mcpwm.h"

#define MCPWM_GROUP_ID   0
#define PWM_GPIO_U       32
#define PWM_GPIO_V       33
#define PWM_GPIO_W       25

// pwm 频率
#define PWM_FREQ_HZ      30000
// timer 分辨率
#define TIMER_RES_HZ     10000000

static mcpwm_timer_handle_t      s_timer;
static mcpwm_oper_handle_t       s_oper_u, s_oper_v, s_oper_w;
static mcpwm_cmpr_handle_t       s_cmpr_u, s_cmpr_v, s_cmpr_w;
static mcpwm_gen_handle_t        s_gen_u,  s_gen_v,  s_gen_w;
static const uint32_t period_ticks = TIMER_RES_HZ / PWM_FREQ_HZ;
static inline _iq clamp_0_1_iq(_iq x)
{
    const _iq one = (_iq)(1L << GLOBAL_IQ);
    if (x < 0) return 0;
    if (x > one) return one;
    return x;
}

static inline uint32_t duty_iq_to_ticks(_iq duty, uint32_t period_ticks)
{
    // duty: Q(GLOBAL_IQ) in [0,1] -> ticks in [0, period_ticks-1]
    duty = clamp_0_1_iq(duty);
    int64_t prod = (int64_t)duty * (int64_t)period_ticks;
    int64_t ticks = (prod >> GLOBAL_IQ);
    if (ticks < 0) {
        return 0;
    }
    if (period_ticks == 0) {
        return 0;
    }
    if ((uint64_t)ticks >= period_ticks) {
        return period_ticks - 1;
    }
    return (uint32_t)ticks;
}

void make_phase(mcpwm_timer_handle_t timer,
    mcpwm_oper_handle_t *oper,
    mcpwm_cmpr_handle_t *cmpr,
    mcpwm_gen_handle_t  *gen,
    int gpio_num){
        // 2. Operator配置
        mcpwm_operator_config_t oper_cfg = {
            .group_id = MCPWM_GROUP_ID,
        };
        ESP_ERROR_CHECK(mcpwm_new_operator(&oper_cfg, oper));
        ESP_ERROR_CHECK(mcpwm_operator_connect_timer(*oper, timer));

         // 3. Comparator配置
        mcpwm_comparator_config_t comp_cfg={
            .flags.update_cmp_on_tez = 1,
        };
        ESP_ERROR_CHECK(mcpwm_new_comparator(*oper, &comp_cfg, cmpr));

        // 4. Generator 配置
        mcpwm_generator_config_t gen_cfg = {
            .gen_gpio_num = gpio_num,
        };
        ESP_ERROR_CHECK(mcpwm_new_generator(*oper, &gen_cfg, gen));

        ESP_ERROR_CHECK(mcpwm_generator_set_action_on_timer_event(*gen,
            MCPWM_GEN_TIMER_EVENT_ACTION( 
                MCPWM_TIMER_DIRECTION_UP,   // 向上计数
                MCPWM_TIMER_EVENT_EMPTY,    // 空事件
                MCPWM_GEN_ACTION_HIGH)));   // 高电平

        ESP_ERROR_CHECK(mcpwm_generator_set_action_on_compare_event(*gen,      
            MCPWM_GEN_COMPARE_EVENT_ACTION(
                MCPWM_TIMER_DIRECTION_UP,   // 向上计数
                *cmpr,
                MCPWM_GEN_ACTION_LOW)));   // 低电平

        mcpwm_comparator_set_compare_value(*cmpr, 0);
    
    }

void pwm_3phase_init(void)
{
    

    // 1. Timer配置
   
    mcpwm_timer_config_t timer_cfg = {
        .group_id = MCPWM_GROUP_ID,
        .clk_src = MCPWM_TIMER_CLK_SRC_DEFAULT,
        .resolution_hz = TIMER_RES_HZ,
        .period_ticks = period_ticks,
        .count_mode = MCPWM_TIMER_COUNT_MODE_UP,
    };
    ESP_ERROR_CHECK(mcpwm_new_timer(&timer_cfg, &s_timer));

    make_phase(s_timer, &s_oper_u, &s_cmpr_u, &s_gen_u, 32);
    make_phase(s_timer, &s_oper_v, &s_cmpr_v, &s_gen_v, 33);
    make_phase(s_timer, &s_oper_w, &s_cmpr_w, &s_gen_w, 25);
    
    ESP_ERROR_CHECK(mcpwm_timer_enable(s_timer));
    ESP_ERROR_CHECK(mcpwm_timer_start_stop(s_timer, MCPWM_TIMER_START_NO_STOP));
}

void pwm_3phase_set_duty_iq(_iq du, _iq dv, _iq dw)
{
    uint32_t cu = duty_iq_to_ticks(du, period_ticks);
    uint32_t cv = duty_iq_to_ticks(dv, period_ticks);
    uint32_t cw = duty_iq_to_ticks(dw, period_ticks);

    mcpwm_comparator_set_compare_value(s_cmpr_u, cu);
    mcpwm_comparator_set_compare_value(s_cmpr_v, cv);
    mcpwm_comparator_set_compare_value(s_cmpr_w, cw);
}