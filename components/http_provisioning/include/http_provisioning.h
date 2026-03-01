#pragma once

#include "esp_err.h"
#include <stdbool.h>
#include <stddef.h>

#ifdef __cplusplus
extern "C" {
#endif

typedef struct {
    const char *url;
    const char *provision_key;
    const char *provision_secret;
    const char *apn;
} http_provisioning_config_t;

esp_err_t http_provisioning_init(const http_provisioning_config_t *cfg);
bool http_provisioning_run(char *out_token, size_t out_len);

#ifdef __cplusplus
}
#endif
