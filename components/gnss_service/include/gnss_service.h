#pragma once

#include "esp_err.h"
#include <stdbool.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

typedef struct {
    bool valid;
    double lat;
    double lon;
    float speed_kph;
    float hdop;
} gnss_fix_t;

typedef struct {
    int poll_period_ms;
} gnss_service_config_t;

esp_err_t gnss_service_init(const gnss_service_config_t *cfg);
void gnss_service_start(void);
bool gnss_service_get_latest(gnss_fix_t *out_fix);

#ifdef __cplusplus
}
#endif
