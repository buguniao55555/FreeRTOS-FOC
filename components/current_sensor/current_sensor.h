#pragma once
#include <stdio.h>

#define ADC_IN_1 ADC_CHANNEL_0
#define ADC_IN_2 ADC_CHANNEL_3
#define ADC_UNIT ADC_UNIT_1
#define ADC_READ_LEN (SOC_ADC_DIGI_DATA_BYTES_PER_CONV * 2)
#define ADC_BUFFER_SIZE 256
#define ADC_CONV_MODE ADC_CONV_SINGLE_UNIT_1
#define ADC_SAMPLE_RATE 40 * 1000
#define ADC_ATTEN ADC_ATTEN_DB_12
#define ADC_BIT_WIDTH SOC_ADC_DIGI_MAX_BITWIDTH


typedef struct
{
    int32_t Ia;
    int32_t Ib;
} current_readings;