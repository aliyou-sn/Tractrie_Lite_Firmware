#pragma once

#include <stdbool.h>
#include <stdint.h>
#include "esp_err.h"

#ifdef __cplusplus
extern "C" {
#endif

typedef enum {
    POWER_STATE_DRIVE_ACTIVE = 0,
    POWER_STATE_PARK_AWAKE,
    POWER_STATE_PARK_SLEEP,
} power_state_t;

typedef struct {
    int ignition_gpio_num;
    bool ignition_active_high;
    int park_awake_timeout_s;
    int debounce_ms;
} power_state_manager_config_t;

esp_err_t power_state_manager_init(const power_state_manager_config_t *cfg);
void power_state_manager_start(void);
power_state_t power_state_manager_get_state(void);
bool power_state_manager_is_ignition_on(void);
int64_t power_state_manager_get_last_change_ms(void);
const char *power_state_manager_state_str(power_state_t s);
void power_state_manager_set_modem_csclk_target(int mode);
int power_state_manager_get_modem_csclk_target(void);
int power_state_manager_get_modem_csclk_applied(void);
void power_state_manager_mark_modem_csclk_applied(int mode);

#ifdef __cplusplus
}
#endif
