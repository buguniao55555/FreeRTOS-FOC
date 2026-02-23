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

void AS5600_setup();
int32_t read_data();
void read_current();
void current_sensor_setup();


void vTaskReadSensor()
{
    TickType_t xLastWakeTime;
    const TickType_t xFrequency = 100 / portTICK_PERIOD_MS;
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
    AS5600_setup();
    current_sensor_setup();


    static uint8_t ucParameterToPass;
    TaskHandle_t xHandle = NULL;
    xTaskCreate(vTaskReadSensor, "NAME", 4000, &ucParameterToPass, tskIDLE_PRIORITY, &xHandle);
    configASSERT( xHandle );
}
