#include "mqtt_service.h"
#include "modem_at.h"
#include "modem_urc.h"
#include "gnss_service.h"
#include "storage_nvs.h"
#include "call_service.h"
#include "esp_log.h"
#include "esp_timer.h"
#include "driver/i2c.h"
#include "driver/gpio.h"
#include "esp_adc/adc_oneshot.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"
#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include <time.h>
#include <ctype.h>

static const char *TAG = "mqtt_service";
static mqtt_service_config_t s_cfg;
static bool s_started;
static bool s_power_inited;
static adc_oneshot_unit_handle_t s_adc_handle;
static QueueHandle_t s_rpc_q;
static volatile bool s_whitelist_dirty;
static void mqtt_process_async_line(const char *line);

typedef enum {
    RPC_NONE = 0,
    RPC_START_CALL,
    RPC_END_CALL,
} rpc_type_t;

typedef struct {
    rpc_type_t type;
    char req_id[24];
    char number[40];
} rpc_cmd_t;

typedef struct {
    bool awaiting_topic;
    bool awaiting_payload;
    bool have_topic;
    bool have_payload;
    char topic[128];
    char payload[512];
} mqtt_rx_ctx_t;

static mqtt_rx_ctx_t s_rx_ctx;

#define MAX17048_I2C_ADDR         0x36
#define MAX17048_VCELL_REG        0x02
#define I2C_PORT_NUM              I2C_NUM_0
#define I2C_SDA_GPIO              15
#define I2C_SCL_GPIO              16
#define I2C_FREQ_HZ               100000
#define VBAT_ADC_CHANNEL          ADC_CHANNEL_0  // GPIO1 on ESP32-S3

static const char *call_state_str(call_service_state_t s)
{
    switch (s) {
    case CALL_STATE_IDLE: return "IDLE";
    case CALL_STATE_DIALING: return "DIALING";
    case CALL_STATE_RINGING: return "RINGING";
    case CALL_STATE_ACTIVE: return "ACTIVE";
    case CALL_STATE_ENDED: return "ENDED";
    case CALL_STATE_FAILED: return "FAILED";
    case CALL_STATE_REJECTED: return "REJECTED";
    default: return "IDLE";
    }
}

static bool extract_json_string(const char *json, const char *key, char *out, size_t out_len)
{
    if (!json || !key || !out || out_len < 2) return false;
    char needle[48];
    snprintf(needle, sizeof(needle), "\"%s\":\"", key);
    const char *p = strstr(json, needle);
    if (!p) return false;
    p += strlen(needle);
    const char *e = strchr(p, '"');
    if (!e) return false;
    size_t n = (size_t)(e - p);
    if (n >= out_len) n = out_len - 1;
    memcpy(out, p, n);
    out[n] = '\0';
    return true;
}

static bool extract_json_string_relaxed(const char *json, const char *key, char *out, size_t out_len)
{
    if (!json || !key || !out || out_len < 2) return false;
    char needle[64];
    snprintf(needle, sizeof(needle), "\"%s\"", key);
    const char *p = strstr(json, needle);
    if (!p) return false;
    p += strlen(needle);
    while (*p == ' ' || *p == '\t' || *p == '\r' || *p == '\n') p++;
    if (*p != ':') return false;
    p++;
    while (*p == ' ' || *p == '\t' || *p == '\r' || *p == '\n') p++;
    if (*p != '"') return false;
    p++;
    const char *e = strchr(p, '"');
    if (!e) return false;
    size_t n = (size_t)(e - p);
    if (n >= out_len) n = out_len - 1;
    memcpy(out, p, n);
    out[n] = '\0';
    return true;
}

static void normalize_number_local(const char *in, char *out, size_t out_len)
{
    if (!out || out_len == 0) return;
    out[0] = '\0';
    if (!in) return;

    size_t j = 0;
    for (size_t i = 0; in[i] && j < out_len - 1; i++) {
        char c = in[i];
        if (c == '+' && j == 0) {
            out[j++] = c;
            continue;
        }
        if (isdigit((unsigned char)c)) {
            out[j++] = c;
        }
    }
    out[j] = '\0';
}

static bool update_allowed_numbers_from_attributes(const char *payload)
{
    if (!payload || !payload[0]) return false;

    const char *p = strstr(payload, "\"allowed_numbers\"");
    if (!p) return false;

    p = strchr(p, '[');
    if (!p) return false;
    p++;

    esp_err_t err = storage_nvs_clear_whitelist();
    if (err != ESP_OK) {
        ESP_LOGW(TAG, "clear whitelist failed: %s", esp_err_to_name(err));
        return false;
    }

    size_t stored = 0;
    while (*p && *p != ']') {
        while (*p == ' ' || *p == '\t' || *p == '\r' || *p == '\n' || *p == ',') p++;
        if (*p == ']') break;
        if (*p != '"') {
            p++;
            continue;
        }

        p++;
        const char *e = strchr(p, '"');
        if (!e) break;

        size_t n = (size_t)(e - p);
        if (n > 0 && stored < STORAGE_NVS_MAX_WHITELIST) {
            char number[STORAGE_NVS_MAX_NUMBER_LEN] = {0};
            if (n >= sizeof(number)) n = sizeof(number) - 1;
            memcpy(number, p, n);
            number[n] = '\0';

            err = storage_nvs_set_whitelist_entry(stored, number);
            if (err == ESP_OK) {
                stored++;
            } else {
                ESP_LOGW(TAG, "set whitelist[%u] failed: %s", (unsigned)stored, esp_err_to_name(err));
            }
        }
        p = e + 1;
    }

    s_whitelist_dirty = true;
    ESP_LOGI(TAG, "Whitelist updated from attributes: %u", (unsigned)stored);
    return true;
}

static bool delete_whitelist_number(const char *number)
{
    if (!number || !number[0]) return false;

    char target[STORAGE_NVS_MAX_NUMBER_LEN] = {0};
    normalize_number_local(number, target, sizeof(target));
    if (!target[0]) return false;

    char keep[STORAGE_NVS_MAX_WHITELIST][STORAGE_NVS_MAX_NUMBER_LEN] = {{0}};
    size_t keep_count = 0;
    bool deleted = false;

    for (size_t i = 0; i < STORAGE_NVS_MAX_WHITELIST; i++) {
        char current[STORAGE_NVS_MAX_NUMBER_LEN] = {0};
        if (!storage_nvs_get_whitelist_entry(i, current, sizeof(current))) {
            continue;
        }
        char norm[STORAGE_NVS_MAX_NUMBER_LEN] = {0};
        normalize_number_local(current, norm, sizeof(norm));
        if ((strcmp(current, number) == 0) || (strcmp(norm, target) == 0)) {
            deleted = true;
            continue;
        }
        if (keep_count < STORAGE_NVS_MAX_WHITELIST) {
            strncpy(keep[keep_count], current, sizeof(keep[keep_count]) - 1);
            keep_count++;
        }
    }

    if (!deleted) {
        return false;
    }

    esp_err_t err = storage_nvs_clear_whitelist();
    if (err != ESP_OK) {
        ESP_LOGW(TAG, "clear whitelist failed on delete: %s", esp_err_to_name(err));
        return false;
    }

    for (size_t i = 0; i < keep_count; i++) {
        err = storage_nvs_set_whitelist_entry(i, keep[i]);
        if (err != ESP_OK) {
            ESP_LOGW(TAG, "restore whitelist[%u] failed: %s", (unsigned)i, esp_err_to_name(err));
            break;
        }
    }

    s_whitelist_dirty = true;
    ESP_LOGI(TAG, "Deleted number from whitelist: %s", target);
    return true;
}

static bool add_whitelist_number(const char *number)
{
    if (!number || !number[0]) return false;

    char target[STORAGE_NVS_MAX_NUMBER_LEN] = {0};
    normalize_number_local(number, target, sizeof(target));
    if (!target[0]) return false;

    // Fast path: if already present, nothing to do.
    for (size_t i = 0; i < STORAGE_NVS_MAX_WHITELIST; i++) {
        char current[STORAGE_NVS_MAX_NUMBER_LEN] = {0};
        if (!storage_nvs_get_whitelist_entry(i, current, sizeof(current))) {
            continue;
        }
        char norm[STORAGE_NVS_MAX_NUMBER_LEN] = {0};
        normalize_number_local(current, norm, sizeof(norm));
        if ((strcmp(current, number) == 0) || (strcmp(norm, target) == 0)) {
            ESP_LOGI(TAG, "Number already in whitelist: %s", target);
            return true;
        }
    }

    // Find first free slot.
    for (size_t i = 0; i < STORAGE_NVS_MAX_WHITELIST; i++) {
        char current[STORAGE_NVS_MAX_NUMBER_LEN] = {0};
        if (storage_nvs_get_whitelist_entry(i, current, sizeof(current))) {
            continue;
        }
        if (storage_nvs_set_whitelist_entry(i, target) == ESP_OK) {
            s_whitelist_dirty = true;
            ESP_LOGI(TAG, "Added number to whitelist[%u]: %s", (unsigned)i, target);
            return true;
        }
        ESP_LOGW(TAG, "Failed adding number to whitelist[%u]", (unsigned)i);
        return false;
    }

    ESP_LOGW(TAG, "Whitelist full, cannot add: %s", target);
    return false;
}

static void apply_call_policy_ops_from_attributes(const char *payload)
{
    if (!payload || !payload[0]) return;

    char add_number[STORAGE_NVS_MAX_NUMBER_LEN] = {0};
    if (extract_json_string_relaxed(payload, "add_number", add_number, sizeof(add_number))) {
        add_whitelist_number(add_number);
    }

    char remove_number[STORAGE_NVS_MAX_NUMBER_LEN] = {0};
    if (extract_json_string_relaxed(payload, "remove_number", remove_number, sizeof(remove_number))) {
        if (!delete_whitelist_number(remove_number)) {
            ESP_LOGW(TAG, "Remove request received but number not found: %s", remove_number);
        }
    }
}

static void mqtt_urc_line_handler(const char *line, void *user)
{
    (void)user;
    mqtt_process_async_line(line);
}

static void queue_rpc_from_message(const char *topic, const char *payload)
{
    if (!topic || !payload) return;
    const char *prefix = "v1/devices/me/rpc/request/";
    size_t pfx = strlen(prefix);
    if (strncmp(topic, prefix, pfx) != 0) return;

    rpc_cmd_t cmd = {0};
    strncpy(cmd.req_id, topic + pfx, sizeof(cmd.req_id) - 1);

    char method[32] = {0};
    if (!extract_json_string(payload, "method", method, sizeof(method))) {
        return;
    }
    if (strcmp(method, "START_CALL") == 0) {
        cmd.type = RPC_START_CALL;
        extract_json_string(payload, "number", cmd.number, sizeof(cmd.number));
    } else if (strcmp(method, "END_CALL") == 0) {
        cmd.type = RPC_END_CALL;
    } else {
        return;
    }
    if (s_rpc_q) {
        xQueueSend(s_rpc_q, &cmd, 0);
    }
}

static void mqtt_process_async_line(const char *line)
{
    if (!line || !line[0]) return;

    // MQTT RX state machine from SIMCOM URCs.
    if (strstr(line, "+CMQTTRXSTART:")) {
        memset(&s_rx_ctx, 0, sizeof(s_rx_ctx));
        return;
    }
    if (strstr(line, "+CMQTTRXTOPIC:")) {
        s_rx_ctx.awaiting_topic = true;
        return;
    }
    if (strstr(line, "+CMQTTRXPAYLOAD:")) {
        s_rx_ctx.awaiting_payload = true;
        return;
    }
    if (s_rx_ctx.awaiting_topic && line[0] != '+') {
        strncpy(s_rx_ctx.topic, line, sizeof(s_rx_ctx.topic) - 1);
        s_rx_ctx.have_topic = true;
        s_rx_ctx.awaiting_topic = false;
        return;
    }
    if (s_rx_ctx.awaiting_payload && line[0] != '+') {
        strncpy(s_rx_ctx.payload, line, sizeof(s_rx_ctx.payload) - 1);
        s_rx_ctx.have_payload = true;
        s_rx_ctx.awaiting_payload = false;
        return;
    }
    if (strstr(line, "+CMQTTRXEND:")) {
        if (s_rx_ctx.have_topic && s_rx_ctx.have_payload) {
            if (strcmp(s_rx_ctx.topic, "v1/devices/me/attributes") == 0) {
                update_allowed_numbers_from_attributes(s_rx_ctx.payload);
                apply_call_policy_ops_from_attributes(s_rx_ctx.payload);
            }
            queue_rpc_from_message(s_rx_ctx.topic, s_rx_ctx.payload);
        }
        memset(&s_rx_ctx, 0, sizeof(s_rx_ctx));
    }
}

static void normalize_broker_host(const char *in, char *out, size_t out_len)
{
    if (!in || !out || out_len == 0) return;
    const char *p = in;
    if (strncmp(p, "mqtt://", 7) == 0) p += 7;
    if (strncmp(p, "tcp://", 6) == 0) p += 6;
    strncpy(out, p, out_len - 1);
    out[out_len - 1] = '\0';
}

static bool cmd_expect_any_locked(const char *cmd, const char *a, const char *b, int timeout_ms)
{
    if (!cmd || (!a && !b)) return false;
    modem_at_send(cmd);
    TickType_t deadline = xTaskGetTickCount() + pdMS_TO_TICKS(timeout_ms);
    char line[160];
    while ((int)(deadline - xTaskGetTickCount()) > 0) {
        if (!modem_at_read_line(line, sizeof(line), 1000)) continue;
        mqtt_process_async_line(line);
        if (a && strstr(line, a)) return true;
        if (b && strstr(line, b)) return true;
        if (strstr(line, "ERROR")) return false;
    }
    return false;
}

static bool cmd_wait_urc_locked(const char *cmd, const char *urc_prefix, int timeout_ms, char *out, int out_len)
{
    if (!cmd || !urc_prefix) return false;
    modem_at_send(cmd);
    TickType_t deadline = xTaskGetTickCount() + pdMS_TO_TICKS(timeout_ms);
    char line[192];
    while ((int)(deadline - xTaskGetTickCount()) > 0) {
        if (!modem_at_read_line(line, sizeof(line), 1000)) continue;
        mqtt_process_async_line(line);
        if (strstr(line, urc_prefix)) {
            if (out && out_len > 0) {
                strncpy(out, line, out_len - 1);
                out[out_len - 1] = '\0';
            }
            return true;
        }
    }
    return false;
}

static bool wait_urc_locked(const char *urc_prefix, int timeout_ms, char *out, int out_len)
{
    if (!urc_prefix) return false;
    TickType_t deadline = xTaskGetTickCount() + pdMS_TO_TICKS(timeout_ms);
    char line[192];
    while ((int)(deadline - xTaskGetTickCount()) > 0) {
        if (!modem_at_read_line(line, sizeof(line), 1000)) continue;
        mqtt_process_async_line(line);
        if (strstr(line, urc_prefix)) {
            if (out && out_len > 0) {
                strncpy(out, line, out_len - 1);
                out[out_len - 1] = '\0';
            }
            return true;
        }
    }
    return false;
}

static bool net_is_open_locked(void)
{
    char line[96] = {0};
    if (!cmd_wait_urc_locked("AT+NETOPEN?\r\n", "+NETOPEN:", 5000, line, sizeof(line))) {
        return false;
    }
    return strstr(line, "+NETOPEN: 1") != NULL;
}

static bool wait_ok_or_error_locked(int timeout_ms)
{
    TickType_t deadline = xTaskGetTickCount() + pdMS_TO_TICKS(timeout_ms);
    char line[192];
    while ((int)(deadline - xTaskGetTickCount()) > 0) {
        if (!modem_at_read_line(line, sizeof(line), 1000)) continue;
        mqtt_process_async_line(line);
        if (strstr(line, "OK")) return true;
        if (strstr(line, "ERROR")) return false;
    }
    return false;
}

static void power_inputs_init_once(void)
{
    if (s_power_inited) return;

    i2c_config_t i2c_cfg = {
        .mode = I2C_MODE_MASTER,
        .sda_io_num = I2C_SDA_GPIO,
        .scl_io_num = I2C_SCL_GPIO,
        .sda_pullup_en = GPIO_PULLUP_ENABLE,
        .scl_pullup_en = GPIO_PULLUP_ENABLE,
        .master.clk_speed = I2C_FREQ_HZ,
    };
    if (i2c_param_config(I2C_PORT_NUM, &i2c_cfg) == ESP_OK) {
        i2c_driver_install(I2C_PORT_NUM, I2C_MODE_MASTER, 0, 0, 0);
    }

    adc_oneshot_unit_init_cfg_t unit_cfg = {
        .unit_id = ADC_UNIT_1,
    };
    if (adc_oneshot_new_unit(&unit_cfg, &s_adc_handle) == ESP_OK) {
        adc_oneshot_chan_cfg_t chan_cfg = {
            .atten = ADC_ATTEN_DB_12,
            .bitwidth = ADC_BITWIDTH_DEFAULT,
        };
        adc_oneshot_config_channel(s_adc_handle, VBAT_ADC_CHANNEL, &chan_cfg);
    }

    s_power_inited = true;
}

static bool read_max17048_cell_v(float *out_v)
{
    if (!out_v) return false;
    uint8_t reg = MAX17048_VCELL_REG;
    uint8_t data[2] = {0};

    esp_err_t err = i2c_master_write_read_device(
        I2C_PORT_NUM,
        MAX17048_I2C_ADDR,
        &reg, 1,
        data, sizeof(data),
        pdMS_TO_TICKS(100));
    if (err != ESP_OK) return false;

    // Same conversion style as your Arduino test code.
    uint16_t raw = ((uint16_t)data[0] << 8) | data[1];
    *out_v = ((float)raw / 65535.0f) * 5.0f;
    return true;
}

static bool read_vbat_divider_v(float *out_v)
{
    if (!out_v || !s_adc_handle) return false;
    int raw = 0;
    if (adc_oneshot_read(s_adc_handle, VBAT_ADC_CHANNEL, &raw) != ESP_OK) return false;

    // Approximate: ADC pin voltage in [0..3.3V], then undo 200k/100k divider => *3.
    float gpio_v = ((float)raw / 4095.0f) * 3.3f;
    *out_v = gpio_v * 3.0f;
    return true;
}

static void mqtt_cleanup_session_locked(void)
{
    // Per app note, release flow should be DISC -> REL -> STOP. Ignore failures.
    cmd_expect_any_locked("AT+CMQTTDISC=0,60\r\n", "OK", "+CMQTTDISC:", 4000);
    cmd_expect_any_locked("AT+CMQTTREL=0\r\n", "OK", "+CMQTTREL:", 3000);
    cmd_expect_any_locked("AT+CMQTTSTOP\r\n", "OK", "+CMQTTSTOP:", 5000);
}

static bool read_imei(char *out, size_t out_len)
{
    if (!out || out_len < 8) return false;
    modem_at_send("AT+GSN\r\n");
    TickType_t deadline = xTaskGetTickCount() + pdMS_TO_TICKS(5000);
    char line[128];
    while ((int)(deadline - xTaskGetTickCount()) > 0) {
        if (!modem_at_read_line(line, sizeof(line), 1000)) continue;
        if (strstr(line, "AT+GSN") || strstr(line, "OK")) continue;
        size_t n = strspn(line, "0123456789");
        if (n >= 14 && n < out_len) {
            memcpy(out, line, n);
            out[n] = '\0';
            return true;
        }
    }
    return false;
}

static bool mqtt_prepare_data_network(void)
{
    char cmd[160];
    char line[128];
    if (!modem_at_cmd("AT\r\n", "OK", 2000)) return false;
    if (!modem_at_cmd("AT+CPIN?\r\n", "READY", 5000)) return false;
    if (!modem_at_cmd("AT+CSQ\r\n", "OK", 3000)) return false;

    for (int i = 0; i < 10; i++) {
        if (modem_at_cmd_wait_line("AT+CGREG?\r\n", "+CGREG:", line, sizeof(line), 3000)) {
            if (strstr(line, "+CGREG: 0,1") || strstr(line, "+CGREG: 0,5")) break;
        }
        if (i == 9) return false;
        vTaskDelay(pdMS_TO_TICKS(1000));
    }

    if (!modem_at_cmd("AT+CGATT=1\r\n", "OK", 15000)) return false;
    snprintf(cmd, sizeof(cmd), "AT+CGDCONT=1,\"IP\",\"%s\"\r\n", s_cfg.apn);
    if (!modem_at_cmd(cmd, "OK", 5000)) return false;
    if (!modem_at_cmd("AT+CGACT=1,1\r\n", "OK", 15000)) return false;
    modem_at_cmd("AT+CGPADDR=1\r\n", "OK", 4000);

    // Some SIM7672 firmware variants behave better when socket PDN is selected
    // and NETOPEN is confirmed before CMQTTSTART.
    cmd_expect_any_locked("AT+CSOCKSETPN=1\r\n", "OK", "ERROR", 3000);

    if (net_is_open_locked()) {
        return true;
    }

    for (int i = 0; i < 3; i++) {
        // Accept "already opened" as success signal from some FW variants.
        bool open_cmd_ok = cmd_expect_any_locked("AT+NETOPEN\r\n", "OK", "+IP ERROR: Network is already opened", 10000);
        if (!open_cmd_ok) {
            ESP_LOGW(TAG, "NETOPEN command attempt %d did not return expected response", i + 1);
        }
        vTaskDelay(pdMS_TO_TICKS(700));
        if (net_is_open_locked()) {
            return true;
        }
    }
    ESP_LOGW(TAG, "NETOPEN never reached +NETOPEN: 1");
    return false;
}

static bool mqtt_connect(const char *token)
{
    char cmd[256];
    char urc[192];
    char host[96];
    bool use_ssl = (s_cfg.broker_port == 8883);

    normalize_broker_host(s_cfg.broker_host, host, sizeof(host));
    ESP_LOGI(TAG, "MQTT broker host=%s port=%d", host, s_cfg.broker_port);

    bool start_ok = false;
    // Always clean stale session before start.
    mqtt_cleanup_session_locked();

    // Start MQTT service (manual: CMQTTSTART must be executed first).
    for (int i = 0; i < 2; i++) {
        memset(urc, 0, sizeof(urc));
        if (cmd_wait_urc_locked("AT+CMQTTSTART\r\n", "+CMQTTSTART:", 12000, urc, sizeof(urc)) &&
            strstr(urc, "+CMQTTSTART: 0")) {
            start_ok = true;
            break;
        }
        // Retry once after cleanup
        mqtt_cleanup_session_locked();
        vTaskDelay(pdMS_TO_TICKS(1000));
    }
    if (!start_ok) return false;

    if (use_ssl) {
        // TLS without certificate verification (same approach as SIMCOM app-note mode).
        if (!cmd_expect_any_locked("AT+CSSLCFG=\"sslversion\",0,4\r\n", "OK", "ERROR", 5000)) return false;
        if (!cmd_expect_any_locked("AT+CSSLCFG=\"authmode\",0,0\r\n", "OK", "ERROR", 5000)) return false;
        cmd_expect_any_locked("AT+CSSLCFG=\"enableSNI\",0,1\r\n", "OK", "ERROR", 5000);
    }

    if (use_ssl) {
        if (!cmd_expect_any_locked("AT+CMQTTACCQ=0,\"tractrie_lite\",1\r\n", "OK", "+CMQTTACCQ:", 5000)) {
            return false;
        }
        if (!cmd_expect_any_locked("AT+CMQTTSSLCFG=0,0\r\n", "OK", "ERROR", 5000)) {
            return false;
        }
    } else if (!cmd_expect_any_locked("AT+CMQTTACCQ=0,\"tractrie_lite\"\r\n", "OK", "+CMQTTACCQ:", 5000)) {
        return false;
    }

    snprintf(cmd, sizeof(cmd),
             "AT+CMQTTCONNECT=0,\"tcp://%s:%d\",60,0,\"%s\"\r\n",
             host, s_cfg.broker_port, token);

    memset(urc, 0, sizeof(urc));
    if (!cmd_wait_urc_locked(cmd, "+CMQTTCONNECT:", 25000, urc, sizeof(urc))) {
        return false;
    }
    if (!strstr(urc, "+CMQTTCONNECT: 0,0")) {
        ESP_LOGW(TAG, "CMQTTCONNECT failed: %s", urc);
        return false;
    }

    // Required subscriptions for ThingsBoard RPC/attributes.
    const char *rpc_topic = "v1/devices/me/rpc/request/+";
    const char *attr_topic = "v1/devices/me/attributes";
    char sub_cmd[64];

    snprintf(sub_cmd, sizeof(sub_cmd), "AT+CMQTTSUB=0,%d,1\r\n", (int)strlen(rpc_topic));
    if (!modem_at_cmd(sub_cmd, ">", 5000)) return false;
    modem_at_send(rpc_topic);
    if (!wait_ok_or_error_locked(4000)) return false;
    if (!wait_urc_locked("+CMQTTSUB: 0,0", 8000, NULL, 0)) return false;

    snprintf(sub_cmd, sizeof(sub_cmd), "AT+CMQTTSUB=0,%d,1\r\n", (int)strlen(attr_topic));
    if (!modem_at_cmd(sub_cmd, ">", 5000)) return false;
    modem_at_send(attr_topic);
    if (!wait_ok_or_error_locked(4000)) return false;
    if (!wait_urc_locked("+CMQTTSUB: 0,0", 8000, NULL, 0)) return false;

    return true;
}

static bool mqtt_publish_topic_payload(const char *topic, const char *payload)
{
    char cmd[64];

    snprintf(cmd, sizeof(cmd), "AT+CMQTTTOPIC=0,%d\r\n", (int)strlen(topic));
    if (!modem_at_cmd(cmd, ">", 5000)) return false;
    modem_at_send(topic);
    if (!wait_ok_or_error_locked(7000)) return false;

    snprintf(cmd, sizeof(cmd), "AT+CMQTTPAYLOAD=0,%d\r\n", (int)strlen(payload));
    if (!modem_at_cmd(cmd, ">", 5000)) return false;
    modem_at_send(payload);
    if (!wait_ok_or_error_locked(7000)) return false;

    if (!cmd_expect_any_locked("AT+CMQTTPUB=0,1,60\r\n", "+CMQTTPUB: 0,0", "OK", 15000)) return false;
    return true;
}

static bool mqtt_publish_whitelist_telemetry(void)
{
    char payload[384];
    int used = snprintf(payload, sizeof(payload), "{\"call_policy\":{\"allowed_numbers\":[");
    if (used < 0 || used >= (int)sizeof(payload)) return false;

    bool first = true;
    for (size_t i = 0; i < STORAGE_NVS_MAX_WHITELIST; i++) {
        char number[STORAGE_NVS_MAX_NUMBER_LEN] = {0};
        if (!storage_nvs_get_whitelist_entry(i, number, sizeof(number))) {
            continue;
        }
        int n = snprintf(payload + used, sizeof(payload) - (size_t)used,
                         "%s\"%s\"", first ? "" : ",", number);
        if (n < 0 || n >= (int)(sizeof(payload) - (size_t)used)) {
            return false;
        }
        used += n;
        first = false;
    }

    int tail = snprintf(payload + used, sizeof(payload) - (size_t)used, "]}}");
    if (tail < 0 || tail >= (int)(sizeof(payload) - (size_t)used)) return false;

    return mqtt_publish_topic_payload("v1/devices/me/telemetry", payload);
}

static bool mqtt_publish_rpc_response(const char *req_id, const char *json)
{
    char topic[96];
    snprintf(topic, sizeof(topic), "v1/devices/me/rpc/response/%s", req_id);
    return mqtt_publish_topic_payload(topic, json);
}

static bool mqtt_publish_call_telemetry(void)
{
    call_service_state_t call_state = call_service_get_state();
    int call_duration_s = call_service_get_duration_s();
    char fail_reason[32] = {0};
    call_service_get_fail_reason(fail_reason, sizeof(fail_reason));

    char payload[192];
    snprintf(payload, sizeof(payload),
             "{\"call_state\":\"%s\",\"call_duration_s\":%d%s%s%s}",
             call_state_str(call_state),
             call_duration_s,
             fail_reason[0] ? ",\"call_fail_reason\":\"" : "",
             fail_reason[0] ? fail_reason : "",
             fail_reason[0] ? "\"" : "");
    return mqtt_publish_topic_payload("v1/devices/me/telemetry", payload);
}

static void mqtt_drain_async_locked(int ms)
{
    TickType_t deadline = xTaskGetTickCount() + pdMS_TO_TICKS(ms);
    char line[192];
    while ((int)(deadline - xTaskGetTickCount()) > 0) {
        if (!modem_at_read_line(line, sizeof(line), 50)) break;
        mqtt_process_async_line(line);
    }
}

static void mqtt_handle_rpc_cmd_locked(const rpc_cmd_t *cmd)
{
    if (!cmd) return;
    char response[160];

    if (cmd->type == RPC_START_CALL) {
        if (cmd->number[0] == '\0') {
            snprintf(response, sizeof(response), "{\"result\":\"ERROR\",\"reason\":\"missing number\"}");
            mqtt_publish_rpc_response(cmd->req_id, response);
            return;
        }
        if (call_service_dial(cmd->number)) {
            mqtt_publish_call_telemetry();
            snprintf(response, sizeof(response), "{\"result\":\"DIALING\"}");
        } else {
            mqtt_publish_call_telemetry();
            snprintf(response, sizeof(response), "{\"result\":\"ERROR\",\"reason\":\"dial failed\"}");
        }
        mqtt_publish_rpc_response(cmd->req_id, response);
        return;
    }

    if (cmd->type == RPC_END_CALL) {
        if (call_service_hangup()) {
            mqtt_publish_call_telemetry();
            snprintf(response, sizeof(response), "{\"result\":\"OK\"}");
        } else {
            snprintf(response, sizeof(response), "{\"result\":\"ERROR\",\"reason\":\"hangup failed\"}");
        }
        mqtt_publish_rpc_response(cmd->req_id, response);
    }
}

static void mqtt_task(void *arg)
{
    power_inputs_init_once();
    s_rpc_q = xQueueCreate(8, sizeof(rpc_cmd_t));

    char token[128] = {0};
    if (!storage_nvs_get_token(token, sizeof(token)) && s_cfg.fallback_token) {
        strncpy(token, s_cfg.fallback_token, sizeof(token) - 1);
    }
    if (!token[0]) {
        ESP_LOGE(TAG, "No MQTT token available");
        vTaskDelete(NULL);
        return;
    }

    char imei[24] = {0};
    if (!modem_at_lock(10000)) {
        vTaskDelete(NULL);
        return;
    }
    bool have_imei = read_imei(imei, sizeof(imei));
    modem_at_unlock();
    if (!have_imei) {
        strncpy(imei, "UNKNOWN", sizeof(imei) - 1);
    }

    while (1) {
        if (!modem_at_lock(30000)) {
            vTaskDelay(pdMS_TO_TICKS(1000));
            continue;
        }

        bool connected = mqtt_prepare_data_network() && mqtt_connect(token);
        modem_at_unlock();

        if (!connected) {
            ESP_LOGW(TAG, "MQTT connect failed, retrying");
            vTaskDelay(pdMS_TO_TICKS(5000));
            continue;
        }

        ESP_LOGI(TAG, "MQTT connected");
        s_whitelist_dirty = true;

        while (1) {
            gnss_fix_t fix = {0};
            gnss_service_get_latest(&fix);

            time_t now = time(NULL);
            int64_t ts_utc = (now > 0) ? (int64_t)now : 0;
            int rssi = -1;
            const char *operator_name = "UNKNOWN";
            float accel_rms = 0.0f;
            float tilt = 0.0f;
            float obd_speed = 0.0f;
            int rpm = 0;
            float throttle = 0.0f;
            float coolant_temp = 0.0f;
            float batt_v = 0.0f;
            const char *ignition_state = "UNKNOWN";
            call_service_state_t call_state = call_service_get_state();
            int voice_duration = call_service_get_duration_s();
            char fail_reason[32] = {0};
            call_service_get_fail_reason(fail_reason, sizeof(fail_reason));

            float batt_i2c = 0.0f;
            float batt_adc = 0.0f;
            bool have_i2c = read_max17048_cell_v(&batt_i2c);
            bool have_adc = read_vbat_divider_v(&batt_adc);
            if (have_adc) {
                batt_v = batt_adc;
            } else if (have_i2c) {
                batt_v = batt_i2c;
            }

            char payload[640];
            snprintf(payload, sizeof(payload),
                     "{"
                     "\"ts_utc\":%lld,"
                     "\"device_id\":\"%s\","
                     "\"node_type\":\"MOBILE_LITE\","
                     "\"video_state\":\"IDLE\","
                     "\"net\":{\"rssi\":%d,\"operator\":\"%s\"},"
                     "\"gps\":{\"lat\":%.6f,\"lon\":%.6f,\"speed\":%.2f},"
                     "\"imu_summary\":{\"accel_rms\":%.3f,\"tilt\":%.2f},"
                     "\"obd\":{\"speed\":%.2f,\"rpm\":%d,\"throttle\":%.2f,\"coolant_temp\":%.2f},"
                     "\"power\":{\"batt_v\":%.2f,\"ignition_state\":\"%s\"},"
                     "\"voice\":{\"call_state\":\"%s\",\"duration\":%d%s%s%s}"
                     "}",
                     (long long)ts_utc,
                     imei,
                     rssi,
                     operator_name,
                     fix.lat, fix.lon, fix.speed_kph,
                     accel_rms, tilt,
                     obd_speed, rpm, throttle, coolant_temp,
                     batt_v, ignition_state,
                     call_state_str(call_state), voice_duration,
                     fail_reason[0] ? ",\"fail_reason\":\"" : "",
                     fail_reason[0] ? fail_reason : "",
                     fail_reason[0] ? "\"" : "");

            if (!modem_at_lock(15000)) {
                break;
            }
            mqtt_drain_async_locked(150);

            rpc_cmd_t rpc = {0};
            while (xQueueReceive(s_rpc_q, &rpc, 0) == pdTRUE) {
                mqtt_handle_rpc_cmd_locked(&rpc);
            }

            if (s_whitelist_dirty) {
                if (!mqtt_publish_whitelist_telemetry()) {
                    modem_at_unlock();
                    ESP_LOGW(TAG, "Whitelist publish failed, reconnecting");
                    break;
                }
                s_whitelist_dirty = false;
            }

            bool ok = mqtt_publish_topic_payload("v1/devices/me/telemetry", payload);
            modem_at_unlock();

            if (!ok) {
                ESP_LOGW(TAG, "Publish failed, reconnecting");
                break;
            }

            ESP_LOGI(TAG, "Published: %s", payload);
            vTaskDelay(pdMS_TO_TICKS(s_cfg.publish_period_s * 1000));
        }

        if (modem_at_lock(5000)) {
            mqtt_cleanup_session_locked();
            modem_at_unlock();
        }
    }
}

esp_err_t mqtt_service_init(const mqtt_service_config_t *cfg)
{
    if (!cfg || !cfg->apn || !cfg->broker_host || cfg->broker_port <= 0) {
        return ESP_ERR_INVALID_ARG;
    }
    s_cfg = *cfg;
    if (s_cfg.publish_period_s <= 0) s_cfg.publish_period_s = 30;

    // Route MQTT RX URCs through one parser regardless of which task is reading AT lines.
    modem_urc_register("+CMQTTRXSTART:", mqtt_urc_line_handler, NULL);
    modem_urc_register("+CMQTTRXTOPIC:", mqtt_urc_line_handler, NULL);
    modem_urc_register("+CMQTTRXPAYLOAD:", mqtt_urc_line_handler, NULL);
    modem_urc_register("+CMQTTRXEND:", mqtt_urc_line_handler, NULL);
    modem_urc_register("v1/devices/me/", mqtt_urc_line_handler, NULL);
    modem_urc_register("{\"", mqtt_urc_line_handler, NULL);

    return ESP_OK;
}

void mqtt_service_start(void)
{
    if (s_started) return;
    s_started = true;
    xTaskCreate(mqtt_task, "mqtt_task", 8192, NULL, 7, NULL);
}
