#pragma once

#include "esp_err.h"

#ifdef __cplusplus
extern "C" {
#endif

typedef struct {
    const char *apn;
    const char *broker_host;
    int broker_port;
    const char *fallback_token;
    int publish_period_s;
    int publish_period_on_s;
    int publish_period_off_s;
} mqtt_service_config_t;

esp_err_t mqtt_service_init(const mqtt_service_config_t *cfg);
void mqtt_service_start(void);

#ifdef __cplusplus
}
#endif
