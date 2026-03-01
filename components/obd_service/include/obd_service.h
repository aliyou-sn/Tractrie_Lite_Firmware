#pragma once

#include <stdbool.h>
#include <stdint.h>
#include "esp_err.h"

#ifdef __cplusplus
extern "C" {
#endif

typedef struct {
    // Optional dongle MAC in human format: "AA:BB:CC:DD:EE:FF"
    const char *target_mac;
    int fast_interval_ms;
    int medium_interval_ms;
} obd_service_config_t;

typedef struct {
    bool valid;
    bool connected;
    bool vin_valid;
    char vin[18]; // 17 chars + '\0'
    float speed_kph;
    int rpm;
    float throttle_pct;
    float coolant_temp_c;
    int64_t last_update_ms;
} obd_data_t;

esp_err_t obd_service_init(const obd_service_config_t *cfg);
void obd_service_start(void);
bool obd_service_get_latest(obd_data_t *out);
void obd_service_set_enabled(bool enabled);
bool obd_service_is_enabled(void);

#ifdef __cplusplus
}
#endif
