idf_component_register(
    SRCS "mqtt_service.c"
    INCLUDE_DIRS "include"
    REQUIRES modem_at gnss_service storage_nvs call_service obd_service power_state_manager esp_adc driver esp_timer
)
