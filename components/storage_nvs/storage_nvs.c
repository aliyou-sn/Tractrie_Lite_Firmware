#include "storage_nvs.h"
#include "nvs_flash.h"
#include "nvs.h"
#include "esp_log.h"
#include <string.h>
#include <stdio.h>

static const char *TAG = "storage_nvs";
static const char *NVS_NS = "tractrie";
static const char *NVS_KEY_TOKEN = "access_token";
static const char *NVS_KEY_WL_COUNT = "wl_count";

static void make_wl_key(size_t index, char *out, size_t out_len)
{
    snprintf(out, out_len, "wl%u", (unsigned)index);
}

esp_err_t storage_nvs_init(void)
{
    esp_err_t err = nvs_flash_init();
    if (err == ESP_ERR_NVS_NO_FREE_PAGES || err == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        ESP_ERROR_CHECK(nvs_flash_erase());
        err = nvs_flash_init();
    }
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "nvs_flash_init failed: %s", esp_err_to_name(err));
    }
    return err;
}

bool storage_nvs_get_token(char *out, size_t out_len)
{
    if (!out || out_len == 0) {
        return false;
    }

    nvs_handle_t nvs;
    esp_err_t err = nvs_open(NVS_NS, NVS_READONLY, &nvs);
    if (err != ESP_OK) {
        return false;
    }

    size_t len = out_len;
    err = nvs_get_str(nvs, NVS_KEY_TOKEN, out, &len);
    nvs_close(nvs);
    return (err == ESP_OK);
}

esp_err_t storage_nvs_set_token(const char *token)
{
    if (!token || !token[0]) {
        return ESP_ERR_INVALID_ARG;
    }

    nvs_handle_t nvs;
    esp_err_t err = nvs_open(NVS_NS, NVS_READWRITE, &nvs);
    if (err != ESP_OK) {
        return err;
    }

    err = nvs_set_str(nvs, NVS_KEY_TOKEN, token);
    if (err == ESP_OK) {
        err = nvs_commit(nvs);
    }
    nvs_close(nvs);
    return err;
}

esp_err_t storage_nvs_erase_token(void)
{
    nvs_handle_t nvs;
    esp_err_t err = nvs_open(NVS_NS, NVS_READWRITE, &nvs);
    if (err != ESP_OK) {
        return err;
    }

    err = nvs_erase_key(nvs, NVS_KEY_TOKEN);
    if (err == ESP_OK) {
        err = nvs_commit(nvs);
    }
    nvs_close(nvs);
    return err;
}

esp_err_t storage_nvs_set_whitelist_entry(size_t index, const char *number)
{
    if (index >= STORAGE_NVS_MAX_WHITELIST) {
        return ESP_ERR_INVALID_ARG;
    }

    nvs_handle_t nvs;
    esp_err_t err = nvs_open(NVS_NS, NVS_READWRITE, &nvs);
    if (err != ESP_OK) {
        return err;
    }

    char key[8] = {0};
    make_wl_key(index, key, sizeof(key));

    if (number && number[0]) {
        err = nvs_set_str(nvs, key, number);
    } else {
        err = nvs_erase_key(nvs, key);
        if (err == ESP_ERR_NVS_NOT_FOUND) {
            err = ESP_OK;
        }
    }

    size_t count = 0;
    for (size_t i = 0; i < STORAGE_NVS_MAX_WHITELIST; i++) {
        char k[8] = {0};
        make_wl_key(i, k, sizeof(k));
        size_t len = 0;
        if (nvs_get_str(nvs, k, NULL, &len) == ESP_OK && len > 1) {
            count++;
        }
    }
    if (err == ESP_OK) {
        err = nvs_set_u8(nvs, NVS_KEY_WL_COUNT, (uint8_t)count);
    }
    if (err == ESP_OK) {
        err = nvs_commit(nvs);
    }
    nvs_close(nvs);
    return err;
}

bool storage_nvs_get_whitelist_entry(size_t index, char *out, size_t out_len)
{
    if (index >= STORAGE_NVS_MAX_WHITELIST || !out || out_len == 0) {
        return false;
    }

    nvs_handle_t nvs;
    esp_err_t err = nvs_open(NVS_NS, NVS_READONLY, &nvs);
    if (err != ESP_OK) {
        return false;
    }

    char key[8] = {0};
    make_wl_key(index, key, sizeof(key));
    size_t len = out_len;
    err = nvs_get_str(nvs, key, out, &len);
    nvs_close(nvs);
    return (err == ESP_OK);
}

esp_err_t storage_nvs_clear_whitelist(void)
{
    nvs_handle_t nvs;
    esp_err_t err = nvs_open(NVS_NS, NVS_READWRITE, &nvs);
    if (err != ESP_OK) {
        return err;
    }

    for (size_t i = 0; i < STORAGE_NVS_MAX_WHITELIST; i++) {
        char key[8] = {0};
        make_wl_key(i, key, sizeof(key));
        esp_err_t e = nvs_erase_key(nvs, key);
        if (e != ESP_OK && e != ESP_ERR_NVS_NOT_FOUND) {
            err = e;
            break;
        }
    }

    if (err == ESP_OK) {
        err = nvs_set_u8(nvs, NVS_KEY_WL_COUNT, 0);
    }
    if (err == ESP_OK) {
        err = nvs_commit(nvs);
    }
    nvs_close(nvs);
    return err;
}

size_t storage_nvs_get_whitelist_count(void)
{
    nvs_handle_t nvs;
    esp_err_t err = nvs_open(NVS_NS, NVS_READONLY, &nvs);
    if (err != ESP_OK) {
        return 0;
    }

    uint8_t count = 0;
    err = nvs_get_u8(nvs, NVS_KEY_WL_COUNT, &count);
    nvs_close(nvs);
    if (err != ESP_OK) return 0;
    return (size_t)count;
}

bool storage_nvs_is_number_whitelisted(const char *number)
{
    if (!number || !number[0]) return false;

    nvs_handle_t nvs;
    esp_err_t err = nvs_open(NVS_NS, NVS_READONLY, &nvs);
    if (err != ESP_OK) {
        return false;
    }

    bool found = false;
    char val[STORAGE_NVS_MAX_NUMBER_LEN] = {0};
    for (size_t i = 0; i < STORAGE_NVS_MAX_WHITELIST; i++) {
        char key[8] = {0};
        make_wl_key(i, key, sizeof(key));
        size_t len = sizeof(val);
        if (nvs_get_str(nvs, key, val, &len) == ESP_OK) {
            if (strcmp(val, number) == 0) {
                found = true;
                break;
            }
        }
    }

    nvs_close(nvs);
    return found;
}
