idf_component_register(SRCS "main.c"
                       INCLUDE_DIRS "."
                       REQUIRES modem_at gnss_service http_provisioning storage_nvs mqtt_service call_service obd_service
                       )
