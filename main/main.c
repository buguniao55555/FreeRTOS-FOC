#include <stdio.h>
#include "AS5600.h"
#include "current_sensor.h"

void AS5600_setup();
int32_t read_data();
void read_current();
void current_sensor_setup();

void app_main(void)
{
    printf("hello world!");
    AS5600_setup();
    current_sensor_setup();

    while(1)
    {
        read_data();
        read_current();
    }
}
