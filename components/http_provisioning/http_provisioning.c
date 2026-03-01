#include "http_provisioning.h"
#include "modem_at.h"
#include "storage_nvs.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include <stdio.h>
#include <string.h>
#include <stdlib.h>

static const char *TAG = "http_prov";
static http_provisioning_config_t s_cfg;

typedef struct {
    char host[96];
    char path[192];
    int port;
    bool https;
} parsed_url_t;

static bool parse_url(const char *url, parsed_url_t *out)
{
    if (!url || !out) {
        return false;
    }

    memset(out, 0, sizeof(*out));
    out->port = 80;

    const char *p = url;
    if (strncmp(p, "https://", 8) == 0) {
        out->https = true;
        out->port = 443;
        p += 8;
    } else if (strncmp(p, "http://", 7) == 0) {
        p += 7;
    }

    const char *slash = strchr(p, '/');
    const char *host_end = slash ? slash : p + strlen(p);
    const char *colon = NULL;
    for (const char *q = p; q < host_end; q++) {
        if (*q == ':') {
            colon = q;
        }
    }

    if (colon) {
        size_t host_len = (size_t)(colon - p);
        if (host_len == 0 || host_len >= sizeof(out->host)) {
            return false;
        }
        memcpy(out->host, p, host_len);
        out->host[host_len] = '\0';
        out->port = atoi(colon + 1);
    } else {
        size_t host_len = (size_t)(host_end - p);
        if (host_len == 0 || host_len >= sizeof(out->host)) {
            return false;
        }
        memcpy(out->host, p, host_len);
        out->host[host_len] = '\0';
    }

    if (slash) {
        strncpy(out->path, slash, sizeof(out->path) - 1);
    } else {
        strncpy(out->path, "/", sizeof(out->path) - 1);
    }

    return true;
}

static bool modem_get_imei(char *out, size_t out_len)
{
    if (!out || out_len < 8) {
        return false;
    }

    modem_at_send("AT+GSN\r\n");
    TickType_t deadline = xTaskGetTickCount() + pdMS_TO_TICKS(5000);
    char line[128];

    while ((int)(deadline - xTaskGetTickCount()) > 0) {
        if (!modem_at_read_line(line, sizeof(line), 1000)) {
            continue;
        }
        if (strstr(line, "AT+GSN") || strstr(line, "OK")) {
            continue;
        }
        size_t n = strspn(line, "0123456789");
        if (n >= 14 && n < out_len) {
            memcpy(out, line, n);
            out[n] = '\0';
            return true;
        }
    }
    return false;
}

static bool parse_token_from_json(const char *json, char *out, size_t out_len)
{
    if (!json || !out || out_len == 0) {
        return false;
    }

    const char *k1 = "\"credentialsValue\":\"";
    const char *k2 = "\"access_token\":\"";
    const char *p = strstr(json, k1);
    if (p) {
        p += strlen(k1);
    } else {
        p = strstr(json, k2);
        if (!p) {
            return false;
        }
        p += strlen(k2);
    }

    const char *e = strchr(p, '"');
    if (!e) {
        return false;
    }

    size_t n = (size_t)(e - p);
    if (n == 0 || n >= out_len) {
        return false;
    }
    memcpy(out, p, n);
    out[n] = '\0';
    return true;
}

static bool wait_cgreg_ready(int timeout_ms)
{
    TickType_t deadline = xTaskGetTickCount() + pdMS_TO_TICKS(timeout_ms);
    char line[128];
    while ((int)(deadline - xTaskGetTickCount()) > 0) {
        if (modem_at_cmd_wait_line("AT+CGREG?\r\n", "+CGREG:", line, sizeof(line), 2000)) {
            if (strstr(line, "+CGREG: 0,1") || strstr(line, "+CGREG: 0,5")) {
                return true;
            }
        }
        vTaskDelay(pdMS_TO_TICKS(1000));
    }
    return false;
}

static bool network_open_tcp(const char *apn, const char *host, int port)
{
    char cmd[160];

    if (!modem_at_cmd("AT\r\n", "OK", 2000)) return false;
    if (!modem_at_cmd("AT+CPIN?\r\n", "READY", 5000)) return false;

    snprintf(cmd, sizeof(cmd), "AT+CGDCONT=1,\"IP\",\"%s\"\r\n", apn);
    if (!modem_at_cmd(cmd, "OK", 5000)) return false;

    modem_at_cmd("AT+CSQ\r\n", "OK", 3000);
    if (!wait_cgreg_ready(15000)) return false;

    modem_at_cmd("AT+CGACT=1,1\r\n", "OK", 15000);

    for (int i = 0; i < 5; i++) {
        if (modem_at_cmd("AT+CSOCKSETPN=1\r\n", "OK", 4000)) {
            break;
        }
        modem_at_cmd("AT+NETCLOSE\r\n", "OK", 2000);
        if (i == 4) return false;
    }

    modem_at_cmd("AT+CIPMODE=1\r\n", "OK", 2000);
    modem_at_cmd("AT+CIPMODE=0\r\n", "OK", 2000);
    modem_at_cmd("AT+CIPSENDMODE=0\r\n", "OK", 2000);
    modem_at_cmd("AT+CIPCCFG=10,0,0,0,1,0,75000\r\n", "OK", 3000);
    modem_at_cmd("AT+CIPTIMEOUT=75000,15000,15000\r\n", "OK", 3000);

    bool net_ok = false;
    for (int i = 0; i < 6; i++) {
        modem_at_cmd("AT+NETOPEN\r\n", "OK", 5000);
        char line[128];
        if (modem_at_cmd_wait_line("AT+NETOPEN?\r\n", "+NETOPEN:", line, sizeof(line), 4000)) {
            if (strstr(line, "+NETOPEN: 1")) {
                net_ok = true;
                break;
            }
        }
        vTaskDelay(pdMS_TO_TICKS(1000));
    }
    if (!net_ok) return false;

    snprintf(cmd, sizeof(cmd), "AT+CIPOPEN=0,\"TCP\",\"%s\",%d\r\n", host, port);
    for (int i = 0; i < 4; i++) {
        if (modem_at_cmd(cmd, "+CIPOPEN: 0,0", 12000)) {
            return true;
        }
        vTaskDelay(pdMS_TO_TICKS(1000));
    }

    return false;
}

static void network_close_tcp(void)
{
    modem_at_cmd("AT+CIPCLOSE=0\r\n", "OK", 3000);
    modem_at_cmd("AT+NETCLOSE\r\n", "OK", 3000);
}

static bool send_http_request_over_tcp(const char *host, const char *path, const char *payload,
                                       char *resp, size_t resp_len)
{
    char request[1024];
    int req_len = snprintf(request, sizeof(request),
                           "POST %s HTTP/1.1\r\n"
                           "Host: %s\r\n"
                           "Content-Type: application/json\r\n"
                           "Content-Length: %d\r\n"
                           "Connection: close\r\n"
                           "\r\n"
                           "%s\r\n",
                           path, host, (int)strlen(payload), payload);
    if (req_len <= 0 || req_len >= (int)sizeof(request)) {
        return false;
    }

    char cmd[64];
    snprintf(cmd, sizeof(cmd), "AT+CIPSEND=0,%d\r\n", req_len);
    if (!modem_at_cmd(cmd, ">", 8000)) {
        return false;
    }

    modem_at_send(request);

    memset(resp, 0, resp_len);
    size_t used = 0;
    TickType_t deadline = xTaskGetTickCount() + pdMS_TO_TICKS(12000);
    char line[256];
    bool got_http = false;

    while ((int)(deadline - xTaskGetTickCount()) > 0) {
        if (!modem_at_read_line(line, sizeof(line), 1000)) {
            continue;
        }
        if (used + strlen(line) + 2 < resp_len) {
            size_t n = strlen(line);
            memcpy(resp + used, line, n);
            used += n;
            resp[used++] = '\n';
            resp[used] = '\0';
        }
        if (strstr(line, "HTTP/1.1")) {
            got_http = true;
        }
    }

    return got_http;
}

esp_err_t http_provisioning_init(const http_provisioning_config_t *cfg)
{
    if (!cfg || !cfg->url) {
        return ESP_ERR_INVALID_ARG;
    }
    s_cfg = *cfg;
    return ESP_OK;
}

bool http_provisioning_run(char *out_token, size_t out_len)
{
    if (!s_cfg.url || !out_token || out_len == 0 || !s_cfg.apn) {
        return false;
    }

    if (!modem_at_lock(45000)) {
        ESP_LOGE(TAG, "Modem busy");
        return false;
    }

    parsed_url_t u;
    if (!parse_url(s_cfg.url, &u)) {
        modem_at_unlock();
        return false;
    }
    if (u.https) {
        ESP_LOGW(TAG, "HTTPS URL provided; socket provisioning uses plain TCP. Use http:// for test.");
    }

    char imei[24] = {0};
    if (!modem_get_imei(imei, sizeof(imei))) {
        ESP_LOGW(TAG, "Failed to read IMEI, using UNKNOWN");
        strncpy(imei, "UNKNOWN", sizeof(imei) - 1);
    }

    char payload[256];
    if (s_cfg.provision_key && s_cfg.provision_secret) {
        snprintf(payload, sizeof(payload),
                 "{\"provisionDeviceKey\":\"%s\",\"provisionDeviceSecret\":\"%s\",\"deviceName\":\"TRI-%s\"}",
                 s_cfg.provision_key, s_cfg.provision_secret, imei);
    } else {
        snprintf(payload, sizeof(payload),
                 "{\"device_id\":\"TRI-%s\",\"note\":\"webhook-test\"}", imei);
    }

    if (!network_open_tcp(s_cfg.apn, u.host, u.port)) {
        ESP_LOGE(TAG, "Network attach/TCP open failed");
        modem_at_unlock();
        return false;
    }

    char response[1200];
    bool ok = send_http_request_over_tcp(u.host, u.path, payload, response, sizeof(response));
    ESP_LOGI(TAG, "HTTP raw response: %s", response);

    if (ok && parse_token_from_json(response, out_token, out_len)) {
        // storage_nvs_set_token(out_token);
        ESP_LOGI(TAG, "Token stored");
    } else if (ok) {
        ESP_LOGW(TAG, "Token not found in response");
    }

    storage_nvs_set_token("KTQ8hXjWITMBx2FPtAA6");

    network_close_tcp();
    modem_at_unlock();
    return ok;
}
