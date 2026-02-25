#include "current_sensor.h"
#include <stdio.h>
#include "driver/gpio.h"
#include "driver/i2c_master.h"
#include "esp_adc/adc_oneshot.h"
#include "esp_adc/adc_continuous.h"
#include "esp_log.h"
#include "esp_err.h"
#include "esp_adc/adc_cali.h"
#include "esp_adc/adc_cali_scheme.h"
#include <string.h>
#include <stdio.h>

static adc_continuous_handle_t handle = NULL;
static adc_channel_t channel[2] = {ADC_IN_1, ADC_IN_2};
static uint8_t channel_num = 2;
uint8_t result[ADC_READ_LEN] = {0};
uint32_t ret_num = 0;
bool cali_ok = false;
adc_cali_handle_t cali;
current_readings current;
adc_digi_output_data_t * data;
esp_err_t ret;
bool read_ADC_1 = 0;
bool read_ADC_2 = 0;


static const char *TAG = "ADC";
static const char *TAG_CALI = "adc_cali";
// 校准ADC
static bool adc_calibration_init(adc_unit_t unit, adc_atten_t atten, adc_bitwidth_t bitwidth, adc_cali_handle_t *out_handle)
{
    if (!out_handle) {
        return false;
    }
    *out_handle = NULL;

    // Check which schemes are supported on this target
    adc_cali_scheme_ver_t scheme_mask = 0;
    esp_err_t err = adc_cali_check_scheme(&scheme_mask);
    if (err != ESP_OK) {
        ESP_LOGW(TAG_CALI, "adc_cali_check_scheme failed: %s", esp_err_to_name(err));
        return false;
    }

    if (!(scheme_mask & ADC_CALI_SCHEME_VER_LINE_FITTING)) {
        ESP_LOGW(TAG_CALI, "Line fitting scheme not supported on this target");
        return false;
    }

    // Create Line Fitting calibration scheme handle
    adc_cali_line_fitting_config_t cali_config = {
        .unit_id = unit,
        .atten = atten,
        .bitwidth = bitwidth,
        .default_vref = 0, // usually 0; used only if efuse calibration isn't available
    };

    err = adc_cali_create_scheme_line_fitting(&cali_config, out_handle);
    if (err == ESP_OK) {
        ESP_LOGI(TAG_CALI, "ADC calibration enabled (Line Fitting)");
        return true;
    }

    ESP_LOGW(TAG_CALI, "adc_cali_create_scheme_line_fitting failed: %s", esp_err_to_name(err));
    *out_handle = NULL;
    return false;
}

// 将ADC原始值转换为mV
static esp_err_t adc_raw_to_mv(adc_cali_handle_t cali_handle, int raw, int *out_mv)
{
    if (!cali_handle) {
        return ESP_ERR_INVALID_STATE;
    }
    return adc_cali_raw_to_voltage(cali_handle, raw, out_mv);
}

// 释放ADC校准
static void adc_calibration_deinit(adc_cali_handle_t cali_handle)
{
    if (cali_handle) {
        ESP_ERROR_CHECK(adc_cali_delete_scheme_line_fitting(cali_handle));
    }
}

// void current_sensor_setup()
// {
//     adc_oneshot_unit_init_cfg_t init_config1 =
//     {
//         .unit_id = ADC_UNIT_1,
//     };
//     ESP_ERROR_CHECK(adc_oneshot_new_unit(&init_config1, &adc1_handle));
//     adc_oneshot_chan_cfg_t config =
//     {
//         .atten = ADC_ATTEN,
//         .bitwidth = ADC_BIT_WIDTH,
//     };
//     ESP_ERROR_CHECK(adc_oneshot_config_channel(adc1_handle, ADC_IN_1, &config));
//     ESP_ERROR_CHECK(adc_oneshot_config_channel(adc1_handle, ADC_IN_2, &config));
    
//     adc_cali_handle_t adc1_cali_handle = NULL;
//     if (!adc_calibration_init(ADC_UNIT_1, ADC_ATTEN, ADC_BIT_WIDTH, &adc1_cali_handle))
//     {
//         ESP_LOGW("CAL_INIT", "CAL_INIT failed. ");
//         exit(EXIT_FAILURE);
//     }
// }

// current_readings read_current()
// {
//     ESP_ERROR_CHECK(adc_oneshot_read(adc1_handle, ADC_IN_1, &adc_raw));
//     current.Ia = adc_raw;

//     ESP_ERROR_CHECK(adc_oneshot_read(adc1_handle, ADC_IN_2, &adc_raw));
//     current.Ib = adc_raw;
//     return current;
// }



// 初始化ADC
void current_sensor_setup()
{
    adc_continuous_handle_cfg_t adc_config = {
        .max_store_buf_size = ADC_BUFFER_SIZE,
        .conv_frame_size = ADC_READ_LEN,
        .flags = {
            .flush_pool = true,
        }
    };
    ESP_ERROR_CHECK(adc_continuous_new_handle(&adc_config, &handle));

    adc_continuous_config_t dig_cfg = {
        .sample_freq_hz = ADC_SAMPLE_RATE,
        .conv_mode = ADC_CONV_MODE,
    };

    adc_digi_pattern_config_t adc_pattern[SOC_ADC_PATT_LEN_MAX] = {0};
    dig_cfg.pattern_num = channel_num;
    for (int i = 0; i < channel_num; i++) {
        adc_pattern[i].atten = ADC_ATTEN;
        adc_pattern[i].channel = channel[i];
        adc_pattern[i].unit = ADC_UNIT;
        adc_pattern[i].bit_width = ADC_BIT_WIDTH;

        // ESP_LOGI(TAG, "adc_pattern[%d].atten is :%"PRIx8, i, adc_pattern[i].atten);
        // ESP_LOGI(TAG, "adc_pattern[%d].channel is :%"PRIx8, i, adc_pattern[i].channel);
        // ESP_LOGI(TAG, "adc_pattern[%d].unit is :%"PRIx8, i, adc_pattern[i].unit);
    }
    dig_cfg.adc_pattern = adc_pattern;
    ESP_ERROR_CHECK(adc_continuous_config(handle, &dig_cfg));
    cali = NULL;
    cali_ok = adc_calibration_init(ADC_UNIT, ADC_ATTEN, ADC_BIT_WIDTH, &cali);

    ESP_ERROR_CHECK(adc_continuous_start(handle));
}




current_readings read_current()
{
    // flags to check if both adc is read
    read_ADC_1 = 0;
    read_ADC_2 = 0;

    // read from adc
    ret = adc_continuous_read(handle, result, ADC_READ_LEN, &ret_num, 0);

    // if success and number of bytes returned is correct, convert data
    if (ret == ESP_OK && ret_num == ADC_READ_LEN)
    {
        // convert result into struct adc_digi_output_data_t
        data = (adc_digi_output_data_t *) & result[0 * SOC_ADC_DIGI_RESULT_BYTES];

        // if data is from adc channel 1, add to Ia, else add to Ib
        if (data->type1.channel == ADC_IN_1)
        {
            int mv = -1;
            if (cali_ok && adc_raw_to_mv(cali, (int)data->type1.data, &mv) == ESP_OK)
            {
                current.Ia = 2 * ((int32_t)mv - REF_VOLTAGE);
                read_ADC_1 = 1;
            }
        }
        else
        {
            int mv = -1;
            if (cali_ok && adc_raw_to_mv(cali, (int)data->type1.data, &mv) == ESP_OK)
            {
                current.Ib = 2 * ((int32_t)mv - REF_VOLTAGE);
                read_ADC_2 = 1;
            }
        }

        // doing the same thing
        data = (adc_digi_output_data_t *) & result[1 * SOC_ADC_DIGI_RESULT_BYTES];
        if (data->type1.channel == ADC_IN_1)
        {
            int mv = -1;
            if (cali_ok && adc_raw_to_mv(cali, (int)data->type1.data, &mv) == ESP_OK)
            {
                current.Ia = 2 * ((int32_t)mv - REF_VOLTAGE);
                read_ADC_1 = 1;
            }
        }
        else
        {
            int mv = -1;
            if (cali_ok && adc_raw_to_mv(cali, (int)data->type1.data, &mv) == ESP_OK)
            {
                current.Ib = 2 * ((int32_t)mv - REF_VOLTAGE);
                read_ADC_2 = 1;
            }
        }

        // validate both channels are read
        if (read_ADC_1 && read_ADC_2)
        {
            return current;
        }
        
        ESP_LOGE("TASK", "failed to read both channel");
        exit(EXIT_FAILURE);
        // ESP_LOGI("ADC", "data is %d, %d, %d, %d, %d, %d, %d, %d", result[0], result[1], result[2], result[3], result[4], result[5], result[6], result[7]);
        // ESP_LOGI("TASK", "ret is %x, ret_num is %"PRIu32" bytes", ret, ret_num);

        // adc_continuous_data_t parsed_data[ret_num / SOC_ADC_DIGI_RESULT_BYTES];
        // uint32_t num_parsed_samples = 0;

        // esp_err_t parse_ret = adc_continuous_parse_data(handle, result, ret_num, parsed_data, &num_parsed_samples);
        // if (parse_ret == ESP_OK) {
        //     for (int i = 0; i < num_parsed_samples; i++)
        //     {
        //         if (parsed_data[i].valid)
        //         {
        //             // ESP_LOGI(TAG, "ADC%d, Channel: %d, Value: %"PRIu32,
        //             //             parsed_data[i].unit + 1,
        //             //             parsed_data[i].channel,
        //             //             parsed_data[i].raw_data);
        //             int mv = -1;
        //             if (cali_ok && adc_raw_to_mv(cali, (int)parsed_data[i].raw_data, &mv) == ESP_OK)
        //             {
        //                 // ESP_LOGI(TAG, "Ch%d raw=%"PRIu32" => %d mV", parsed_data[i].channel, parsed_data[i].raw_data, mv);
        //             }
        //             else
        //             {
        //                 // Fallback: print raw only
        //                 // ESP_LOGI(TAG, "Ch%d raw=%"PRIu32, parsed_data[i].channel, parsed_data[i].raw_data);
        //             }
        //         }
        //         else
        //         {
        //             ESP_LOGW(TAG, "Invalid data [ADC%d_Ch%d_%"PRIu32"]",
        //                         parsed_data[i].unit + 1,
        //                         parsed_data[i].channel,
        //                         parsed_data[i].raw_data);
        //         }
        //     }
        // } else {
        //     ESP_LOGE(TAG, "Data parsing failed: %s", esp_err_to_name(parse_ret));
        // }

        /**
         * Because printing is slow, so every time you call `ulTaskNotifyTake`, it will immediately return.
         * To avoid a task watchdog timeout, add a delay here. When you replace the way you process the data,
         * usually you don't need this delay (as this task will block for a while).
         */
    }
    // else if (ret == ESP_ERR_TIMEOUT)
    // {
    //     //We try to read `EXAMPLE_READ_LEN` until API returns timeout, which means there's no available data
    //     exit(EXIT_FAILURE);
    // }
    ESP_LOGE("TASK", "failed to read 2, return number is %lu", ret_num);
    exit(EXIT_FAILURE);
}