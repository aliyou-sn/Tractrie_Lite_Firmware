#pragma once

#include "driver/uart.h"
#include "esp_err.h"
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

typedef struct {
    uart_port_t uart_num;
    int tx_gpio;
    int rx_gpio;
    int rts_gpio;
    int cts_gpio;
    int baud_rate;
    int rx_buffer_size;
    int tx_buffer_size;
} hal_uart_config_t;

esp_err_t hal_uart_init(const hal_uart_config_t *cfg);
int hal_uart_write_bytes(uart_port_t uart_num, const uint8_t *data, int len);
int hal_uart_read_bytes(uart_port_t uart_num, uint8_t *buf, int len, int timeout_ms);

#ifdef __cplusplus
}
#endif
