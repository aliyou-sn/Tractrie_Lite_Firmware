#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"
#include "modem_at.h"
#include "modem_urc.h"
#include "gnss_service.h"
#include "http_provisioning.h"
#include "storage_nvs.h"
#include "mqtt_service.h"
#include "call_service.h"
#include "obd_service.h"

static const char *TAG = "app_main";
static const char *CONTROL_CENTER_NUMBER = "+2349073283370";
// Set to 0 while validating GSM voice-only milestone (cloud independent).
#define ENABLE_MQTT 1

static void log_whitelist_numbers_on_boot(void)
{
    size_t count = storage_nvs_get_whitelist_count();
    ESP_LOGI(TAG, "Whitelist count on boot: %u", (unsigned)count);
    for (size_t i = 0; i < STORAGE_NVS_MAX_WHITELIST; i++) {
        char number[STORAGE_NVS_MAX_NUMBER_LEN] = {0};
        if (storage_nvs_get_whitelist_entry(i, number, sizeof(number))) {
            ESP_LOGI(TAG, "Whitelist[%u] = %s", (unsigned)i, number);
        }
    }
}

static void provisioning_task(void *arg)
{
    ESP_ERROR_CHECK(storage_nvs_init());

    char saved_token[128] = {0};
    if (storage_nvs_get_token(saved_token, sizeof(saved_token))) {
        ESP_LOGI(TAG, "Token already stored: %s", saved_token);
        vTaskDelete(NULL);
        return;
    }

    http_provisioning_config_t prov_cfg = {
        .url = "http://webhook.site/d5653af0-0b00-410e-be55-f8e706201f43",
        .provision_key = NULL,
        .provision_secret = NULL,
        // .apn = "internet.ng.airtel.com",
        .apn = "mtn web.gprs.mtnnigeria.net",
    };
    ESP_ERROR_CHECK(http_provisioning_init(&prov_cfg));

    char token[128] = {0};
    if (http_provisioning_run(token, sizeof(token))) {
        if (token[0]) {
            ESP_LOGI(TAG, "Provisioned token: %s", token);
        } else {
            ESP_LOGI(TAG, "Provisioning completed (no token in response)");
        }
    } else {
        ESP_LOGW(TAG, "Provisioning failed");
    }

    vTaskDelete(NULL);
}

void app_main(void)
{
    modem_urc_init();
    ESP_ERROR_CHECK(storage_nvs_init());
    log_whitelist_numbers_on_boot();

    modem_at_config_t cfg = {
        .uart_num = UART_NUM_1,
        .tx_gpio = 18,
        .rx_gpio = 17,
        .baud_rate = 115200,
    };

    ESP_ERROR_CHECK(modem_at_init(&cfg));

    while (1) {
        if (modem_at_cmd("AT\r\n", "OK", 1000)) {
            ESP_LOGI(TAG, "AT handshake OK");
            break;
        }
        ESP_LOGW(TAG, "No OK yet, retrying...");
        vTaskDelay(pdMS_TO_TICKS(1000));
    }

    xTaskCreate(provisioning_task, "prov_task", 8192, NULL, 8, NULL);

    gnss_service_config_t gnss_cfg = {
        .poll_period_ms = 2000,
    };
    ESP_ERROR_CHECK(gnss_service_init(&gnss_cfg));
    gnss_service_start();

    // ESP_ERROR_CHECK(storage_nvs_set_whitelist_entry(0, "+2349073283370"));

    call_service_config_t call_cfg = {
        .max_call_duration_s = 0,
        .sos_gpio_num = 4,
        .sos_active_low = true,
        .sos_number = CONTROL_CENTER_NUMBER,
        .sos_call_timeout_s = 120,
    };
    ESP_ERROR_CHECK(call_service_init(&call_cfg));
    call_service_start();

    obd_service_config_t obd_cfg = {
        .target_mac = NULL,   // Set dongle MAC "AA:BB:CC:DD:EE:FF" to lock to one adapter.
        .fast_interval_ms = 700,
        .medium_interval_ms = 1800,
    };
    ESP_ERROR_CHECK(obd_service_init(&obd_cfg));
    obd_service_start();

#if ENABLE_MQTT
    mqtt_service_config_t mqtt_cfg = {
        // .apn = "internet.ng.airtel.com",
        .apn = "mtn web.gprs.mtnnigeria.net",
        // .apn = "35.179.223.108",
        .broker_host = "54.159.242.170",
        // .broker_host = "mqtt.thingsboard.cloud",
        .broker_port = 1883,
        .fallback_token = "KTQ8hXjWITMBx2FPtAA6",
        .publish_period_s = 15,
    };
    ESP_ERROR_CHECK(mqtt_service_init(&mqtt_cfg));
    mqtt_service_start();
#else
    ESP_LOGW(TAG, "MQTT disabled (voice-only mode)");
#endif

    while (1) {
        vTaskDelay(pdMS_TO_TICKS(5000));
    }
}
