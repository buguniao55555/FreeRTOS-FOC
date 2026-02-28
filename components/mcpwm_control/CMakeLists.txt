idf_component_register(
    SRCS "mcpwm.c"
    INCLUDE_DIRS "."
    PRIV_REQUIRES driver esp_common
)