#pragma once

#include "esp_err.h"
#include <stddef.h>
#include <stdbool.h>

#define STORAGE_NVS_MAX_WHITELIST 8
#define STORAGE_NVS_MAX_NUMBER_LEN 24

#ifdef __cplusplus
extern "C" {
#endif

esp_err_t storage_nvs_init(void);
bool storage_nvs_get_token(char *out, size_t out_len);
esp_err_t storage_nvs_set_token(const char *token);
esp_err_t storage_nvs_erase_token(void);

esp_err_t storage_nvs_set_whitelist_entry(size_t index, const char *number);
bool storage_nvs_get_whitelist_entry(size_t index, char *out, size_t out_len);
esp_err_t storage_nvs_clear_whitelist(void);
size_t storage_nvs_get_whitelist_count(void);
bool storage_nvs_is_number_whitelisted(const char *number);

#ifdef __cplusplus
}
#endif
