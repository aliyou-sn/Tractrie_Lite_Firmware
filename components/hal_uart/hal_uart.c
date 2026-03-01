#include "hal_uart.h"
#include "driver/uart.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"

static const char *TAG = "hal_uart";

esp_err_t hal_uart_init(const hal_uart_config_t *cfg)
{
    if (!cfg) {
        return ESP_ERR_INVALID_ARG;
    }

    uart_config_t uart_cfg = {
        .baud_rate = cfg->baud_rate,
        .data_bits = UART_DATA_8_BITS,
        .parity = UART_PARITY_DISABLE,
        .stop_bits = UART_STOP_BITS_1,
        .flow_ctrl = UART_HW_FLOWCTRL_DISABLE,
        .source_clk = UART_SCLK_DEFAULT,
    };

    ESP_ERROR_CHECK(uart_param_config(cfg->uart_num, &uart_cfg));
    ESP_ERROR_CHECK(uart_set_pin(cfg->uart_num, cfg->tx_gpio, cfg->rx_gpio, cfg->rts_gpio, cfg->cts_gpio));

    int rx_size = cfg->rx_buffer_size > 0 ? cfg->rx_buffer_size : 2048;
    int tx_size = cfg->tx_buffer_size > 0 ? cfg->tx_buffer_size : 2048;

    esp_err_t err = uart_driver_install(cfg->uart_num, rx_size, tx_size, 0, NULL, 0);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "uart_driver_install failed: %s", esp_err_to_name(err));
        return err;
    }

    ESP_LOGI(TAG, "UART%d init ok, baud=%d", cfg->uart_num, cfg->baud_rate);
    return ESP_OK;
}

int hal_uart_write_bytes(uart_port_t uart_num, const uint8_t *data, int len)
{
    if (!data || len <= 0) {
        return 0;
    }
    return uart_write_bytes(uart_num, (const char *)data, len);
}

int hal_uart_read_bytes(uart_port_t uart_num, uint8_t *buf, int len, int timeout_ms)
{
    if (!buf || len <= 0) {
        return 0;
    }
    TickType_t ticks = timeout_ms < 0 ? portMAX_DELAY : pdMS_TO_TICKS(timeout_ms);
    return uart_read_bytes(uart_num, buf, len, ticks);
}
