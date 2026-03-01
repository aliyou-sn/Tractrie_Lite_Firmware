#include "call_service.h"
#include "modem_at.h"
#include "modem_urc.h"
#include "storage_nvs.h"
#include "esp_log.h"
#include "esp_timer.h"
#include "driver/gpio.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include <string.h>
#include <ctype.h>
#include <stdio.h>

static const char *TAG = "call_service";

static call_service_config_t s_cfg = {
    .max_call_duration_s = 0,
};
static bool s_started;
static call_service_state_t s_state = CALL_STATE_IDLE;
static int64_t s_call_started_ms;
static int s_call_duration_s;
static char s_last_caller[32];
static char s_fail_reason[24];
static bool s_incoming_pending;
static bool s_have_clip;
static int64_t s_ring_started_ms;
static bool s_sos_prev_pressed;
static int64_t s_sos_started_ms;

static void normalize_number(const char *in, char *out, size_t out_len)
{
    if (!out || out_len == 0) return;
    out[0] = '\0';
    if (!in) return;

    size_t j = 0;
    for (size_t i = 0; in[i] && j < out_len - 1; i++) {
        char c = in[i];
        if (c == '+' && j == 0) {
            out[j++] = c;
            continue;
        }
        if (isdigit((unsigned char)c)) {
            out[j++] = c;
        }
    }
    out[j] = '\0';

    // Basic Nigeria normalization: 0xxxxxxxxxx -> +234xxxxxxxxxx
    if (out[0] == '0' && strlen(out) == 11) {
        char tmp[32];
        snprintf(tmp, sizeof(tmp), "+234%s", out + 1);
        strncpy(out, tmp, out_len - 1);
        out[out_len - 1] = '\0';
    }
}

static void set_state(call_service_state_t new_state, const char *reason)
{
    if (new_state == CALL_STATE_ACTIVE && s_state != CALL_STATE_ACTIVE) {
        s_call_started_ms = esp_timer_get_time() / 1000;
        s_call_duration_s = 0;
    }
    if ((new_state == CALL_STATE_ENDED || new_state == CALL_STATE_FAILED || new_state == CALL_STATE_REJECTED) &&
        s_call_started_ms > 0) {
        int64_t now_ms = esp_timer_get_time() / 1000;
        s_call_duration_s = (int)((now_ms - s_call_started_ms) / 1000);
        s_call_started_ms = 0;
    }

    if (reason && reason[0]) {
        strncpy(s_fail_reason, reason, sizeof(s_fail_reason) - 1);
        s_fail_reason[sizeof(s_fail_reason) - 1] = '\0';
    } else if (new_state != CALL_STATE_FAILED) {
        s_fail_reason[0] = '\0';
    }

    if (new_state != s_state) {
        ESP_LOGI(TAG, "state: %d -> %d", (int)s_state, (int)new_state);
        s_state = new_state;
    }
}

static void parse_clip_number(const char *line)
{
    const char *q1 = strchr(line, '"');
    if (!q1) return;
    const char *q2 = strchr(q1 + 1, '"');
    if (!q2 || q2 <= q1 + 1) return;

    char raw[40] = {0};
    size_t n = (size_t)(q2 - (q1 + 1));
    if (n >= sizeof(raw)) n = sizeof(raw) - 1;
    memcpy(raw, q1 + 1, n);
    raw[n] = '\0';
    normalize_number(raw, s_last_caller, sizeof(s_last_caller));
    ESP_LOGI(TAG, "caller: %s", s_last_caller);
    s_have_clip = (s_last_caller[0] != '\0');
}

static void urc_ring(const char *line, void *user)
{
    (void)line;
    (void)user;
    if (s_state == CALL_STATE_IDLE || s_state == CALL_STATE_ENDED || s_state == CALL_STATE_FAILED || s_state == CALL_STATE_REJECTED) {
        set_state(CALL_STATE_RINGING, NULL);
        s_incoming_pending = true;
        s_have_clip = false;
        s_ring_started_ms = esp_timer_get_time() / 1000;
    }
}

static void urc_clip(const char *line, void *user)
{
    (void)user;
    parse_clip_number(line);
}

static void urc_end(const char *line, void *user)
{
    (void)user;
    if (strstr(line, "BUSY")) {
        set_state(CALL_STATE_FAILED, "BUSY");
    } else if (strstr(line, "NO ANSWER")) {
        set_state(CALL_STATE_FAILED, "NO_ANSWER");
    } else {
        set_state(CALL_STATE_ENDED, "NO_CARRIER");
    }
}

static void urc_voice_begin(const char *line, void *user)
{
    (void)line;
    (void)user;
    set_state(CALL_STATE_ACTIVE, NULL);
}

static void call_service_task(void *arg)
{
    (void)arg;

    // Voice URCs and caller ID setup.
    if (modem_at_lock(10000)) {
        modem_at_cmd("AT+CLIP=1\r\n", "OK", 3000);
        modem_at_cmd("AT+CRC=1\r\n", "OK", 3000);
        modem_at_cmd("AT+CMOD=0\r\n", "OK", 3000);
        modem_at_unlock();
    }

    if (s_cfg.sos_gpio_num >= 0) {
        gpio_config_t io = {
            .pin_bit_mask = (1ULL << s_cfg.sos_gpio_num),
            .mode = GPIO_MODE_INPUT,
            .pull_up_en = s_cfg.sos_active_low ? GPIO_PULLUP_ENABLE : GPIO_PULLUP_DISABLE,
            .pull_down_en = s_cfg.sos_active_low ? GPIO_PULLDOWN_DISABLE : GPIO_PULLDOWN_ENABLE,
            .intr_type = GPIO_INTR_DISABLE,
        };
        gpio_config(&io);
    }

    while (1) {
        // SOS button edge: press to dial when idle, press again to hangup during call.
        if (s_cfg.sos_gpio_num >= 0 && s_cfg.sos_number && s_cfg.sos_number[0]) {
            int level = gpio_get_level((gpio_num_t)s_cfg.sos_gpio_num);
            bool pressed = s_cfg.sos_active_low ? (level == 0) : (level != 0);

            if (pressed && !s_sos_prev_pressed) {
                if (s_state == CALL_STATE_IDLE || s_state == CALL_STATE_ENDED ||
                    s_state == CALL_STATE_FAILED || s_state == CALL_STATE_REJECTED) {
                    ESP_LOGI(TAG, "SOS pressed -> dialing %s", s_cfg.sos_number);
                    if (call_service_dial(s_cfg.sos_number)) {
                        s_sos_started_ms = esp_timer_get_time() / 1000;
                    }
                } else if (s_state == CALL_STATE_DIALING || s_state == CALL_STATE_RINGING || s_state == CALL_STATE_ACTIVE) {
                    ESP_LOGI(TAG, "SOS pressed during call -> hangup");
                    call_service_hangup();
                }
            }
            s_sos_prev_pressed = pressed;
        }

        // SOS auto-timeout hangup (separate from generic max duration).
        if (s_cfg.sos_call_timeout_s > 0 && s_sos_started_ms > 0 &&
            (s_state == CALL_STATE_DIALING || s_state == CALL_STATE_ACTIVE || s_state == CALL_STATE_RINGING)) {
            int64_t now_ms = esp_timer_get_time() / 1000;
            int elapsed = (int)((now_ms - s_sos_started_ms) / 1000);
            if (elapsed >= s_cfg.sos_call_timeout_s) {
                ESP_LOGW(TAG, "SOS timeout reached -> hanging up");
                call_service_hangup();
            }
        }

        if (s_state == CALL_STATE_RINGING && s_incoming_pending && s_have_clip) {
            bool allow = storage_nvs_is_number_whitelisted(s_last_caller);
            if (allow) {
                ESP_LOGI(TAG, "incoming caller allowed, answering");
                call_service_answer();
            } else {
                ESP_LOGW(TAG, "incoming caller blocked, rejecting");
                call_service_hangup();
                set_state(CALL_STATE_REJECTED, "NOT_WHITELISTED");
            }
            s_incoming_pending = false;
        }

        if (s_state == CALL_STATE_RINGING && s_incoming_pending && s_ring_started_ms > 0) {
            int64_t now_ms = esp_timer_get_time() / 1000;
            if ((now_ms - s_ring_started_ms) > 8000) {
                ESP_LOGW(TAG, "ring timeout without CLIP, rejecting");
                call_service_hangup();
                set_state(CALL_STATE_REJECTED, "NO_CLIP");
                s_incoming_pending = false;
            }
        }

        if (s_state == CALL_STATE_ACTIVE && s_cfg.max_call_duration_s > 0 && s_call_started_ms > 0) {
            int64_t now_ms = esp_timer_get_time() / 1000;
            int elapsed = (int)((now_ms - s_call_started_ms) / 1000);
            s_call_duration_s = elapsed;
            if (elapsed >= s_cfg.max_call_duration_s) {
                ESP_LOGW(TAG, "max call duration reached, hanging up");
                call_service_hangup();
                set_state(CALL_STATE_ENDED, "MAX_DURATION");
            }
        }

        // Keep ENDED/FAILED/REJECTED transient then back to IDLE.
        if (s_state == CALL_STATE_ENDED || s_state == CALL_STATE_FAILED || s_state == CALL_STATE_REJECTED) {
            vTaskDelay(pdMS_TO_TICKS(1500));
            set_state(CALL_STATE_IDLE, NULL);
            s_incoming_pending = false;
            s_have_clip = false;
            s_ring_started_ms = 0;
            s_sos_started_ms = 0;
        }
        vTaskDelay(pdMS_TO_TICKS(100));
    }
}

esp_err_t call_service_init(const call_service_config_t *cfg)
{
    if (cfg) {
        s_cfg = *cfg;
    }
    s_state = CALL_STATE_IDLE;
    s_last_caller[0] = '\0';
    s_fail_reason[0] = '\0';
    s_call_started_ms = 0;
    s_call_duration_s = 0;
    s_incoming_pending = false;
    s_have_clip = false;
    s_ring_started_ms = 0;
    s_sos_prev_pressed = false;
    s_sos_started_ms = 0;

    if (s_cfg.sos_gpio_num == 0 && s_cfg.sos_number == NULL && s_cfg.sos_call_timeout_s == 0 && s_cfg.max_call_duration_s == 0) {
        // Apply safe defaults when caller passes zeroed config.
        s_cfg.sos_gpio_num = -1;
        s_cfg.sos_active_low = true;
    }

    modem_urc_register("RING", urc_ring, NULL);
    modem_urc_register("+CRING:", urc_ring, NULL);
    modem_urc_register("+CLIP:", urc_clip, NULL);
    modem_urc_register("NO CARRIER", urc_end, NULL);
    modem_urc_register("BUSY", urc_end, NULL);
    modem_urc_register("NO ANSWER", urc_end, NULL);
    modem_urc_register("VOICE CALL: BEGIN", urc_voice_begin, NULL);
    modem_urc_register("VOICE CALL: END", urc_end, NULL);
    return ESP_OK;
}

void call_service_start(void)
{
    if (s_started) return;
    s_started = true;
    xTaskCreate(call_service_task, "call_service", 4096, NULL, 8, NULL);
}

bool call_service_answer(void)
{
    bool ok = modem_at_cmd("ATA\r\n", "OK", 8000);
    if (ok) {
        set_state(CALL_STATE_ACTIVE, NULL);
    }
    return ok;
}

bool call_service_hangup(void)
{
    bool ok = modem_at_cmd("ATH\r\n", "OK", 5000);
    if (ok) {
        set_state(CALL_STATE_ENDED, NULL);
    }
    return ok;
}

bool call_service_dial(const char *number)
{
    if (!number || !number[0]) return false;
    char norm[40] = {0};
    normalize_number(number, norm, sizeof(norm));
    if (!norm[0]) return false;

    char cmd[72];
    snprintf(cmd, sizeof(cmd), "ATD%s;\r\n", norm);
    bool ok = modem_at_cmd(cmd, "OK", 10000);
    if (ok) {
        set_state(CALL_STATE_DIALING, NULL);
    } else {
        set_state(CALL_STATE_FAILED, "DIAL_FAILED");
    }
    return ok;
}

call_service_state_t call_service_get_state(void)
{
    return s_state;
}

int call_service_get_duration_s(void)
{
    if (s_state == CALL_STATE_ACTIVE && s_call_started_ms > 0) {
        int64_t now_ms = esp_timer_get_time() / 1000;
        return (int)((now_ms - s_call_started_ms) / 1000);
    }
    return s_call_duration_s;
}

void call_service_get_last_caller(char *out, int out_len)
{
    if (!out || out_len <= 0) return;
    strncpy(out, s_last_caller, out_len - 1);
    out[out_len - 1] = '\0';
}

void call_service_get_fail_reason(char *out, int out_len)
{
    if (!out || out_len <= 0) return;
    strncpy(out, s_fail_reason, out_len - 1);
    out[out_len - 1] = '\0';
}
