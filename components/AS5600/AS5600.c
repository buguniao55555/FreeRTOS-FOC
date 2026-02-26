#include "AS5600.h"
#include <stdio.h>
#include "driver/gpio.h"
#include "driver/i2c_master.h"
#include "esp_log.h"
#include "esp_timer.h"
#include "math.h"
#include "IQmathLib.h"

i2c_master_dev_handle_t dev_handle;

static const char * TAG = "AS5600";
static uint16_t sensor_angle = 0;
static uint16_t sensor_angle_prev = 0;
static int32_t sensor_overflow = 0;
static uint64_t sensor_time_prev = 0;
static int32_t sensor_counts_prev = 0;
static const uint8_t AS5600_REG_RAW_ANGLE_HI = 0x0C;
// static const uint8_t AS5600_REG_ANGLE_HI = 0x0E;

void AS5600_setup()
{
    i2c_master_bus_config_t i2c_mst_config = {
        .clk_source = I2C_CLK_SRC_DEFAULT,
        .i2c_port = I2C_NUM_0,
        .scl_io_num = SCL0,
        .sda_io_num = SDA0,
        .glitch_ignore_cnt = 7,
        .flags.enable_internal_pullup = false,
    };
    i2c_master_bus_handle_t bus_handle;
    ESP_ERROR_CHECK(i2c_new_master_bus(&i2c_mst_config, &bus_handle));

    i2c_device_config_t dev_cfg = {
        .dev_addr_length = I2C_ADDR_BIT_LEN_7,
        .device_address = 0x36,
        .scl_speed_hz = 100000,     // 100kHz
    };

    ESP_ERROR_CHECK(i2c_master_bus_add_device(bus_handle, &dev_cfg, &dev_handle));

    
    // read an initial value
    uint8_t reg = AS5600_REG_RAW_ANGLE_HI;
    uint8_t buffer[2];
    ESP_ERROR_CHECK(i2c_master_transmit_receive(dev_handle, &reg, 1, buffer, 2, -1));
    sensor_angle = ((uint16_t)buffer[0] << 8) | buffer[1];
    sensor_angle &= 0x0FFF;
    sensor_angle_prev = sensor_angle;
    sensor_overflow = 0;

    sensor_counts_prev = (int32_t)sensor_angle;
    sensor_time_prev = (uint64_t)esp_timer_get_time();
}



int32_t read_data()
{
    // RAW ANGLE register high byte is 0x0C
    uint8_t reg = AS5600_REG_RAW_ANGLE_HI;
    uint8_t buffer[2];
    ESP_ERROR_CHECK(i2c_master_transmit_receive(dev_handle, &reg, 1, buffer, 2, -1));

    // record previous sensor angle
    sensor_angle_prev = sensor_angle;

    // bit shifting High before Low bits
    sensor_angle = ((uint16_t)buffer[0] << 8) | buffer[1];

    // 只取12位有效位
    sensor_angle &= 0x0FFF;

    // check overflow
    int16_t diff = (int16_t)sensor_angle_prev - (int16_t)sensor_angle;
    if (diff > 2048)
    {
        ++ sensor_overflow;
    }
    else if (diff < -2048)
    {
        -- sensor_overflow;
    }


    // ESP_LOGI(TAG, "AS5600 raw angle = %u, accumulative angle = %d", sensor_angle, sensor_angle + 4096 * sensor_overflow);
    return (int32_t)sensor_angle + 4096 * sensor_overflow;
}

/**
 * @brief Park transform
 *
 * Transform I_alpha and I_beta to I_q and I_d. 
 *
 * @param[in]   sensor_counts   sensor count number in raw integer
 * 
 * @return Computed angle in _iq type and of format (degree / 360)
 */
_iq get_angle(int32_t sensor_counts)
{
    // 4096 counts per mechanical revolution
     
    return (_IQ((float)sensor_counts  * 0.0015339808f));
}

float get_angular_velocity(int32_t sensor_counts)
{
    const uint64_t now_us = (uint64_t)esp_timer_get_time();
    // 第一次读取
    if (sensor_time_prev == 0) {
        sensor_time_prev = now_us;
        sensor_counts_prev = sensor_counts;
        return 0.0f;
    }

    const uint64_t Ts = now_us - sensor_time_prev;
    sensor_time_prev = now_us;
    //防止除零
    if (Ts == 0) {
        sensor_counts_prev = sensor_counts;
        return 0.0f;
    }
    //计算变化量
    const int32_t dcounts = sensor_counts - sensor_counts_prev;
    sensor_counts_prev = sensor_counts;

    // rad/s = (dcounts/dt) * (2*pi/4096)
    const float dt_s = (float)Ts * 1e-6f;
    const float rad_per_count = 6.2831853071795864769f / 4096.0f;
    return ((float)dcounts / dt_s) * rad_per_count;
}