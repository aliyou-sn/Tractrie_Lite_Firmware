idf_component_register(
    SRCS "obd_service.c"
    INCLUDE_DIRS "include"
    REQUIRES bt nvs_flash call_service
)
