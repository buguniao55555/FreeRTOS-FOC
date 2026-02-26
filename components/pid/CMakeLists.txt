idf_component_register(
    SRCS "pid.c" "pidIQ.c"
    INCLUDE_DIRS "."
    PRIV_REQUIRES esp_timer espressif__iqmath
)