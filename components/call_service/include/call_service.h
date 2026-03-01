#pragma once

#include "esp_err.h"
#include <stdbool.h>

#ifdef __cplusplus
extern "C" {
#endif

typedef enum {
    CALL_STATE_IDLE = 0,
    CALL_STATE_RINGING,
    CALL_STATE_DIALING,
    CALL_STATE_ACTIVE,
    CALL_STATE_ENDED,
    CALL_STATE_FAILED,
    CALL_STATE_REJECTED,
} call_service_state_t;

typedef struct {
    int max_call_duration_s;
    int sos_gpio_num;               // e.g. 4, set <0 to disable
    bool sos_active_low;            // true: pressed=0 with pull-up
    const char *sos_number;         // E.164 preferred: +234...
    int sos_call_timeout_s;         // auto hangup timeout for SOS, 0=disabled
} call_service_config_t;

esp_err_t call_service_init(const call_service_config_t *cfg);
void call_service_start(void);

bool call_service_answer(void);
bool call_service_hangup(void);
bool call_service_dial(const char *number);

call_service_state_t call_service_get_state(void);
int call_service_get_duration_s(void);
void call_service_get_last_caller(char *out, int out_len);
void call_service_get_fail_reason(char *out, int out_len);

#ifdef __cplusplus
}
#endif
