idf_component_register(
    SRCS "AS5600.c" "as5600.c"
    INCLUDE_DIRS "."
    REQUIRES driver esp_timer
)