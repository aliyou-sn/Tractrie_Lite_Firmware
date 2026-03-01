#pragma once

#include "driver/uart.h"
#include "esp_err.h"
#include <stdint.h>
#include <stdbool.h>

#ifdef __cplusplus
extern "C" {
#endif

typedef struct {
    uart_port_t uart_num;
    int tx_gpio;
    int rx_gpio;
    int baud_rate;
} modem_at_config_t;

esp_err_t modem_at_init(const modem_at_config_t *cfg);
void modem_at_set_log_rx(bool enable);
bool modem_at_lock(int timeout_ms);
void modem_at_unlock(void);
int modem_at_send(const char *cmd);
bool modem_at_wait_for(const char *token, int timeout_ms);
bool modem_at_cmd(const char *cmd, const char *expect, int timeout_ms);
bool modem_at_wait_for_line(const char *token, char *out, int out_len, int timeout_ms);
bool modem_at_read_line(char *out, int out_len, int timeout_ms);
bool modem_at_cmd_wait_line(const char *cmd, const char *token, char *out, int out_len, int timeout_ms);

#ifdef __cplusplus
}
#endif
