idf_component_register(
    SRCS "http_provisioning.c"
    INCLUDE_DIRS "include"
    REQUIRES modem_at storage_nvs
)
