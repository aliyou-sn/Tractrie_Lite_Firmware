idf_component_register(
    SRCS "call_service.c"
    INCLUDE_DIRS "include"
    REQUIRES modem_at modem_urc esp_timer storage_nvs driver
)
