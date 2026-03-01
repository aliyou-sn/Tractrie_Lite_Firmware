idf_component_register(
    SRCS "modem_at.c"
    INCLUDE_DIRS "include"
    REQUIRES hal_uart modem_urc
)
