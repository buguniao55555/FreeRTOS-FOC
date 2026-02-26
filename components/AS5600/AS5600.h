#pragma once
#include "IQmathLib.h"
#include <stdint.h>

#define SDA0 GPIO_NUM_19
#define SCL0 GPIO_NUM_18
#define CS0 GPIO_NUM_22

#define SDA1 GPIO_NUM_23
#define SCL1 GPIO_NUM_5
#define CS1 GPIO_NUM_21

void AS5600_setup(void);
int32_t read_data(void);                 // 返回累积counts（带overflow展开）
_iq get_angle(int32_t sensor_counts);  // 度（deg）
float get_angular_velocity(int32_t sensor_counts); // rad/s

