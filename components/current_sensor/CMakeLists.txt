idf_component_register(
    SRCS "current_sensor.c"
    INCLUDE_DIRS "."
    REQUIRES driver esp_adc
)