#include "modem_at.h"
#include "hal_uart.h"
#include "modem_urc.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"
#include "freertos/semphr.h"
#include <string.h>

#define MODEM_AT_MAX_LINE 256
#define MODEM_AT_RX_CHUNK 64
#define MODEM_AT_LINE_QUEUE_LEN 8

static const char *TAG = "modem_at";

typedef struct {
    char line[MODEM_AT_MAX_LINE];
} modem_line_t;

static QueueHandle_t s_line_q;
static uart_port_t s_uart_num = UART_NUM_1;
static bool s_log_rx = true;
static SemaphoreHandle_t s_cmd_mutex;

static void modem_at_rx_task(void *arg)
{
    uint8_t buf[MODEM_AT_RX_CHUNK];
    char line[MODEM_AT_MAX_LINE];
    size_t idx = 0;

    while (1) {
        int n = hal_uart_read_bytes(s_uart_num, buf, sizeof(buf), 1000);
        if (n <= 0) {
            continue;
        }
        for (int i = 0; i < n; i++) {
            uint8_t b = buf[i];
            if (b == '\r') {
                continue;
            }
            // SIMCOM CIPSEND prompt is usually a bare '>' (no newline).
            // Emit it as a synthetic line so command waiters can match it.
            if (b == '>') {
                modem_line_t prompt = {0};
                strncpy(prompt.line, ">", sizeof(prompt.line) - 1);
                xQueueSend(s_line_q, &prompt, 0);
                if (s_log_rx) {
                    ESP_LOGI(TAG, "RX: %s", prompt.line);
                }
                continue;
            }
            if (b == '\n') {
                if (idx == 0) {
                    continue;
                }
                line[idx] = '\0';

                modem_line_t msg = {0};
                strncpy(msg.line, line, sizeof(msg.line) - 1);
                xQueueSend(s_line_q, &msg, 0);

                if (s_log_rx) {
                    ESP_LOGI(TAG, "RX: %s", msg.line);
                }
                idx = 0;
                continue;
            }

            if (idx < sizeof(line) - 1) {
                line[idx++] = (char)b;
            } else {
                // overflow: reset line
                idx = 0;
            }
        }
    }
}

esp_err_t modem_at_init(const modem_at_config_t *cfg)
{
    if (!cfg) {
        return ESP_ERR_INVALID_ARG;
    }

    s_uart_num = cfg->uart_num;

    hal_uart_config_t uart_cfg = {
        .uart_num = cfg->uart_num,
        .tx_gpio = cfg->tx_gpio,
        .rx_gpio = cfg->rx_gpio,
        .rts_gpio = UART_PIN_NO_CHANGE,
        .cts_gpio = UART_PIN_NO_CHANGE,
        .baud_rate = cfg->baud_rate,
        .rx_buffer_size = 4096,
        .tx_buffer_size = 1024,
    };

    esp_err_t err = hal_uart_init(&uart_cfg);
    if (err != ESP_OK) {
        return err;
    }

    if (!s_line_q) {
        s_line_q = xQueueCreate(MODEM_AT_LINE_QUEUE_LEN, sizeof(modem_line_t));
    }
    if (!s_line_q) {
        return ESP_ERR_NO_MEM;
    }

    if (!s_cmd_mutex) {
        s_cmd_mutex = xSemaphoreCreateRecursiveMutex();
    }
    if (!s_cmd_mutex) {
        return ESP_ERR_NO_MEM;
    }

    xTaskCreate(modem_at_rx_task, "modem_at_rx", 4096, NULL, 10, NULL);
    ESP_LOGI(TAG, "modem_at init ok");
    return ESP_OK;
}

void modem_at_set_log_rx(bool enable)
{
    s_log_rx = enable;
}

bool modem_at_lock(int timeout_ms)
{
    if (!s_cmd_mutex) {
        return false;
    }
    TickType_t ticks = timeout_ms < 0 ? portMAX_DELAY : pdMS_TO_TICKS(timeout_ms);
    return xSemaphoreTakeRecursive(s_cmd_mutex, ticks) == pdTRUE;
}

void modem_at_unlock(void)
{
    if (s_cmd_mutex) {
        xSemaphoreGiveRecursive(s_cmd_mutex);
    }
}

int modem_at_send(const char *cmd)
{
    if (!cmd) {
        return 0;
    }
    int len = (int)strlen(cmd);
    ESP_LOGI(TAG, "TX: %s", cmd);
    return hal_uart_write_bytes(s_uart_num, (const uint8_t *)cmd, len);
}

bool modem_at_wait_for(const char *token, int timeout_ms)
{
    if (!token || !s_line_q) {
        return false;
    }

    TickType_t deadline = xTaskGetTickCount() + pdMS_TO_TICKS(timeout_ms);
    modem_line_t msg;

    while ((int)(deadline - xTaskGetTickCount()) > 0) {
        if (xQueueReceive(s_line_q, &msg, pdMS_TO_TICKS(50)) == pdTRUE) {
            if (strstr(msg.line, token) != NULL) {
                return true;
            }
            modem_urc_dispatch(msg.line);
        }
    }
    return false;
}

bool modem_at_cmd(const char *cmd, const char *expect, int timeout_ms)
{
    if (!cmd || !expect) {
        return false;
    }
    if (!modem_at_lock(timeout_ms)) {
        return false;
    }
    modem_at_send(cmd);
    bool ok = modem_at_wait_for(expect, timeout_ms);
    modem_at_unlock();
    return ok;
}

bool modem_at_wait_for_line(const char *token, char *out, int out_len, int timeout_ms)
{
    if (!token || !out || out_len <= 0 || !s_line_q) {
        return false;
    }

    TickType_t deadline = xTaskGetTickCount() + pdMS_TO_TICKS(timeout_ms);
    modem_line_t msg;

    while ((int)(deadline - xTaskGetTickCount()) > 0) {
        if (xQueueReceive(s_line_q, &msg, pdMS_TO_TICKS(50)) == pdTRUE) {
            if (strstr(msg.line, token) != NULL) {
                strncpy(out, msg.line, out_len - 1);
                out[out_len - 1] = '\0';
                return true;
            }
            modem_urc_dispatch(msg.line);
        }
    }
    return false;
}

bool modem_at_read_line(char *out, int out_len, int timeout_ms)
{
    if (!out || out_len <= 0 || !s_line_q) {
        return false;
    }

    TickType_t deadline = xTaskGetTickCount() + pdMS_TO_TICKS(timeout_ms);
    modem_line_t msg;

    while ((int)(deadline - xTaskGetTickCount()) > 0) {
        if (xQueueReceive(s_line_q, &msg, pdMS_TO_TICKS(50)) == pdTRUE) {
            if (msg.line[0] == '\0') {
                continue;
            }
            modem_urc_dispatch(msg.line);
            strncpy(out, msg.line, out_len - 1);
            out[out_len - 1] = '\0';
            return true;
        }
    }
    return false;
}

bool modem_at_cmd_wait_line(const char *cmd, const char *token, char *out, int out_len, int timeout_ms)
{
    if (!cmd || !token || !out || out_len <= 0) {
        return false;
    }
    if (!modem_at_lock(timeout_ms)) {
        return false;
    }
    modem_at_send(cmd);
    bool ok = modem_at_wait_for_line(token, out, out_len, timeout_ms);
    modem_at_unlock();
    return ok;
}
