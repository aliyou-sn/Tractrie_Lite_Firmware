#include "gnss_service.h"
#include "modem_at.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include <string.h>
#include <stdlib.h>

static const char *TAG = "gnss_service";

static gnss_service_config_t s_cfg = {
    .poll_period_ms = 2000,
};

static gnss_fix_t s_last_fix;
static bool s_started = false;

static bool parse_cgnssinfo(const char *line, gnss_fix_t *out_fix)
{
    if (!line || !out_fix) {
        return false;
    }

    // Expected SIMCOM format (typical):
    // +CGNSSINFO: <run>,<fix>,<utcdate>,<utctime>,<lat>,<lon>,<msl>,<speed>,<course>,<fixmode>,<res>,<hdop>,...
    char tmp[256];
    strncpy(tmp, line, sizeof(tmp) - 1);
    tmp[sizeof(tmp) - 1] = '\0';

    char *p = strchr(tmp, ':');
    if (!p) {
        return false;
    }
    p++;

    int field = 0;
    char *token = strtok(p, ",");

    int fix_status = 0;
    double lat = 0.0;
    double lon = 0.0;
    float speed = 0.0f;
    float hdop = 99.9f;

    while (token) {
        switch (field) {
        case 1:
            fix_status = atoi(token);
            break;
        case 4:
            lat = atof(token);
            break;
        case 5:
            lon = atof(token);
            break;
        case 7:
            speed = (float)atof(token);
            break;
        case 11:
            hdop = (float)atof(token);
            break;
        default:
            break;
        }

        field++;
        token = strtok(NULL, ",");
    }

    out_fix->valid = (fix_status == 1);
    out_fix->lat = lat;
    out_fix->lon = lon;
    out_fix->speed_kph = speed;
    out_fix->hdop = hdop;
    return true;
}

static void gnss_task(void *arg)
{
    char line[256];

    // Enable GNSS power if needed
    modem_at_cmd("AT+CGNSSPWR?\r\n", "+CGNSSPWR:", 2000);
    if (modem_at_cmd("AT+CGNSSPWR=1\r\n", "OK", 5000)) {
        ESP_LOGI(TAG, "GNSS power on");
    }

    while (1) {
        if (modem_at_cmd_wait_line("AT+CGNSSINFO\r\n", "+CGNSSINFO:", line, sizeof(line), 2000)) {
            gnss_fix_t fix = {0};
            if (parse_cgnssinfo(line, &fix)) {
                s_last_fix = fix;
                ESP_LOGI(TAG, "GNSS fix valid=%d lat=%.6f lon=%.6f speed=%.2f hdop=%.2f",
                         fix.valid, fix.lat, fix.lon, fix.speed_kph, fix.hdop);
            }
        }

        vTaskDelay(pdMS_TO_TICKS(s_cfg.poll_period_ms));
    }
}

esp_err_t gnss_service_init(const gnss_service_config_t *cfg)
{
    if (cfg) {
        s_cfg = *cfg;
        if (s_cfg.poll_period_ms <= 0) {
            s_cfg.poll_period_ms = 2000;
        }
    }
    memset(&s_last_fix, 0, sizeof(s_last_fix));
    return ESP_OK;
}

void gnss_service_start(void)
{
    if (s_started) {
        return;
    }
    s_started = true;
    xTaskCreate(gnss_task, "gnss_task", 4096, NULL, 9, NULL);
}

bool gnss_service_get_latest(gnss_fix_t *out_fix)
{
    if (!out_fix) {
        return false;
    }
    *out_fix = s_last_fix;
    return true;
}
