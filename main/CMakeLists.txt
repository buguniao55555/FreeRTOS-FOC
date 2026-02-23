idf_component_register(SRCS "main.c"
                    INCLUDE_DIRS "."
                    REQUIRES AS5600 current_sensor transfer esp_timer)
