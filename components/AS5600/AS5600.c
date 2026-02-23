#include "AS5600.h"
#include <stdio.h>
#include "driver/gpio.h"
#include "driver/i2c_master.h"
#include "esp_log.h"

i2c_master_dev_handle_t dev_handle;

static const char * TAG = "AS5600";
static uint16_t sensor_angle = 0;
static uint16_t sensor_angle_prev = 0;
static int32_t sensor_overflow = 0;

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
        .scl_speed_hz = 400000,     // 400kHz
    };

    ESP_ERROR_CHECK(i2c_master_bus_add_device(bus_handle, &dev_cfg, &dev_handle));

    
    // read an initial value
    uint8_t reg = 0x0E;
    uint8_t buffer[2];
    ESP_ERROR_CHECK(i2c_master_transmit_receive(dev_handle, &reg, 1, buffer, 2, -1));
    sensor_angle = ((uint16_t)buffer[0] << 8) | buffer[1];
    sensor_angle &= 0x0FFF;
}



int32_t read_data()
{
    // angle register address is 0x0E
    uint8_t reg = 0x0E;
    uint8_t buffer[2];
    ESP_ERROR_CHECK(i2c_master_transmit_receive(dev_handle, &reg, 1, buffer, 2, -1));

    // record previous sensor angle
    sensor_angle_prev = sensor_angle;

    // bit shifting High before Low bits
    sensor_angle = ((uint16_t)buffer[0] << 8) | buffer[1];

    // add mask to clean the unused bits
    sensor_angle &= 0x0FFF;

    // check overflow
    int16_t diff = sensor_angle_prev - sensor_angle;
    if (diff > 2048)
    {
        ++ sensor_overflow;
    }
    else if (diff < -2048)
    {
        -- sensor_overflow;
    }


    // ESP_LOGI(TAG, "AS5600 raw angle = %u, accumulative angle = %d", sensor_angle, sensor_angle + 4096 * sensor_overflow);
    return sensor_angle + 4096 * sensor_overflow;
}