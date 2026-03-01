idf_component_register(
    SRCS "power_state_manager.c"
    INCLUDE_DIRS "include"
    REQUIRES driver esp_timer freertos
)
