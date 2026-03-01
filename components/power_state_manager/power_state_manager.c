#include "power_state_manager.h"
#include "driver/gpio.h"
#include "esp_log.h"
#include "esp_timer.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include <string.h>

static const char *TAG = "power_state";

static power_state_manager_config_t s_cfg;
static bool s_inited;
static bool s_started;
static volatile bool s_ign_on;
static volatile power_state_t s_state = POWER_STATE_PARK_AWAKE;
static volatile int64_t s_last_change_ms;
static int64_t s_ign_off_since_ms;
static volatile int s_modem_csclk_target = 0;
static volatile int s_modem_csclk_applied = -1;

const char *power_state_manager_state_str(power_state_t s)
{
    switch (s) {
    case POWER_STATE_DRIVE_ACTIVE: return "DRIVE_ACTIVE";
    case POWER_STATE_PARK_AWAKE: return "PARK_AWAKE";
    case POWER_STATE_PARK_SLEEP: return "PARK_SLEEP";
    default: return "PARK_AWAKE";
    }
}

static bool read_ignition_raw(void)
{
    int level = gpio_get_level((gpio_num_t)s_cfg.ignition_gpio_num);
    return s_cfg.ignition_active_high ? (level != 0) : (level == 0);
}

static void set_state(power_state_t next)
{
    if (s_state == next) return;
    s_state = next;
    s_last_change_ms = esp_timer_get_time() / 1000;
    ESP_LOGI(TAG, "State -> %s", power_state_manager_state_str(next));
}

static void power_state_task(void *arg)
{
    (void)arg;
    bool debounced = read_ignition_raw();
    bool candidate = debounced;
    int64_t candidate_since_ms = esp_timer_get_time() / 1000;
    s_ign_on = debounced;
    s_ign_off_since_ms = debounced ? 0 : candidate_since_ms;
    set_state(debounced ? POWER_STATE_DRIVE_ACTIVE : POWER_STATE_PARK_AWAKE);

    while (1) {
        int64_t now_ms = esp_timer_get_time() / 1000;
        bool raw = read_ignition_raw();

        if (raw != candidate) {
            candidate = raw;
            candidate_since_ms = now_ms;
        } else if (candidate != debounced && (now_ms - candidate_since_ms) >= s_cfg.debounce_ms) {
            debounced = candidate;
            s_ign_on = debounced;
            ESP_LOGI(TAG, "Ignition -> %s", debounced ? "ON" : "OFF");
            if (debounced) {
                s_ign_off_since_ms = 0;
                set_state(POWER_STATE_DRIVE_ACTIVE);
            } else {
                s_ign_off_since_ms = now_ms;
                set_state(POWER_STATE_PARK_AWAKE);
            }
        }

        if (!s_ign_on && s_state == POWER_STATE_PARK_AWAKE && s_ign_off_since_ms > 0) {
            if ((now_ms - s_ign_off_since_ms) >= ((int64_t)s_cfg.park_awake_timeout_s * 1000)) {
                set_state(POWER_STATE_PARK_SLEEP);
            }
        }

        if (s_ign_on && s_state != POWER_STATE_DRIVE_ACTIVE) {
            set_state(POWER_STATE_DRIVE_ACTIVE);
        }

        vTaskDelay(pdMS_TO_TICKS(200));
    }
}

esp_err_t power_state_manager_init(const power_state_manager_config_t *cfg)
{
    if (s_inited) return ESP_OK;
    if (!cfg || cfg->ignition_gpio_num < 0) return ESP_ERR_INVALID_ARG;

    memset(&s_cfg, 0, sizeof(s_cfg));
    s_cfg = *cfg;
    if (s_cfg.park_awake_timeout_s <= 0) s_cfg.park_awake_timeout_s = 120;
    if (s_cfg.debounce_ms <= 0) s_cfg.debounce_ms = 300;

    gpio_config_t io = {
        .pin_bit_mask = (1ULL << s_cfg.ignition_gpio_num),
        .mode = GPIO_MODE_INPUT,
        .pull_up_en = GPIO_PULLUP_DISABLE,
        .pull_down_en = GPIO_PULLDOWN_ENABLE,
        .intr_type = GPIO_INTR_DISABLE,
    };
    gpio_config(&io);

    s_last_change_ms = esp_timer_get_time() / 1000;
    s_inited = true;
    return ESP_OK;
}

void power_state_manager_start(void)
{
    if (!s_inited || s_started) return;
    s_started = true;
    xTaskCreate(power_state_task, "power_state_task", 3072, NULL, 8, NULL);
}

power_state_t power_state_manager_get_state(void)
{
    return s_state;
}

bool power_state_manager_is_ignition_on(void)
{
    return s_ign_on;
}

int64_t power_state_manager_get_last_change_ms(void)
{
    return s_last_change_ms;
}

void power_state_manager_set_modem_csclk_target(int mode)
{
    if (mode < 0 || mode > 2) return;
    s_modem_csclk_target = mode;
}

int power_state_manager_get_modem_csclk_target(void)
{
    return s_modem_csclk_target;
}

int power_state_manager_get_modem_csclk_applied(void)
{
    return s_modem_csclk_applied;
}

void power_state_manager_mark_modem_csclk_applied(int mode)
{
    if (mode < 0 || mode > 2) return;
    s_modem_csclk_applied = mode;
}
