#include "obd_service.h"
#include "call_service.h"
#include "esp_log.h"
#include "esp_timer.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include <ctype.h>
#include <stdio.h>
#include <string.h>
#include <stdlib.h>

#if CONFIG_BT_ENABLED && CONFIG_BT_NIMBLE_ENABLED
#include "host/ble_gap.h"
#include "host/ble_gatt.h"
#include "host/ble_hs.h"
#include "nimble/ble.h"
#include "nimble/nimble_port.h"
#include "nimble/nimble_port_freertos.h"
#include "services/gap/ble_svc_gap.h"
#include "services/gatt/ble_svc_gatt.h"
#endif

static const char *TAG = "obd_service";

static obd_service_config_t s_cfg;
static bool s_inited;
static bool s_started;
static volatile bool s_enabled = true;
static obd_data_t s_data;

#if CONFIG_BT_ENABLED && CONFIG_BT_NIMBLE_ENABLED

#define OBD_SERVICE_UUID16 0xFFF0
#define OBD_CHAR_UUID16    0xFFF1
#define CCCD_UUID16        0x2902
#define TX_GUARD_MS        180

static uint8_t s_own_addr_type;
static uint16_t s_conn_handle = BLE_HS_CONN_HANDLE_NONE;
static uint16_t s_chr_def_handle;
static uint16_t s_chr_val_handle;
static uint16_t s_svc_end_handle;
static uint16_t s_cccd_handle;
static uint8_t s_chr_props;
static bool s_service_found;
static bool s_obd_ready;
static bool s_obd_initialized;
static bool s_scan_active;
static bool s_target_mac_enabled;
static uint8_t s_target_mac[6];
static char s_rx_line[256];
static size_t s_rx_len;
static char s_vin_buf[18];
static size_t s_vin_len;
static int s_scan_backoff_ms = 1000;
static int64_t s_next_scan_at_ms;

static void start_scan(void);
static void schedule_scan_retry(void);
static void stop_ble_runtime(void);

static bool is_hex_char(char c)
{
    return (c >= '0' && c <= '9') || (c >= 'A' && c <= 'F') || (c >= 'a' && c <= 'f');
}

static int8_t hex_nibble(char c)
{
    if (c >= '0' && c <= '9') return (int8_t)(c - '0');
    c = (char)toupper((unsigned char)c);
    if (c >= 'A' && c <= 'F') return (int8_t)(10 + c - 'A');
    return -1;
}

static bool parse_mac_string(const char *s, uint8_t out[6])
{
    unsigned int b[6];
    if (!s || !out || s[0] == '\0') return false;
    if (sscanf(s, "%2x:%2x:%2x:%2x:%2x:%2x", &b[0], &b[1], &b[2], &b[3], &b[4], &b[5]) != 6) {
        return false;
    }
    // NimBLE keeps addr as little-endian bytes.
    out[0] = (uint8_t)b[5];
    out[1] = (uint8_t)b[4];
    out[2] = (uint8_t)b[3];
    out[3] = (uint8_t)b[2];
    out[4] = (uint8_t)b[1];
    out[5] = (uint8_t)b[0];
    return true;
}

static bool addr_matches_target(const ble_addr_t *addr)
{
    if (!s_target_mac_enabled) return true;
    return memcmp(addr->val, s_target_mac, sizeof(s_target_mac)) == 0;
}

static void mark_disconnected(void)
{
    s_data.connected = false;
    s_obd_ready = false;
    s_obd_initialized = false;
    s_conn_handle = BLE_HS_CONN_HANDLE_NONE;
    s_chr_def_handle = 0;
    s_chr_val_handle = 0;
    s_cccd_handle = 0;
    s_chr_props = 0;
    s_service_found = false;
    s_rx_len = 0;
    schedule_scan_retry();
}

static void stop_ble_runtime(void)
{
    if (s_scan_active) {
        ble_gap_disc_cancel();
        s_scan_active = false;
    }
    if (s_conn_handle != BLE_HS_CONN_HANDLE_NONE) {
        ble_gap_terminate(s_conn_handle, BLE_ERR_REM_USER_CONN_TERM);
    }
    mark_disconnected();
}

static void vin_reset(void)
{
    s_vin_len = 0;
    memset(s_vin_buf, 0, sizeof(s_vin_buf));
    s_data.vin_valid = false;
    s_data.vin[0] = '\0';
}

static void vin_consume_bytes(const uint8_t *data, size_t len)
{
    if (!data || len == 0 || s_data.vin_valid) return;

    for (size_t i = 0; i < len && s_vin_len < 17; i++) {
        char c = (char)data[i];
        if (isalnum((unsigned char)c)) {
            s_vin_buf[s_vin_len++] = c;
        }
    }

    if (s_vin_len >= 17) {
        s_vin_buf[17] = '\0';
        strncpy(s_data.vin, s_vin_buf, sizeof(s_data.vin) - 1);
        s_data.vin[sizeof(s_data.vin) - 1] = '\0';
        s_data.vin_valid = true;
        ESP_LOGI(TAG, "VIN: %s", s_data.vin);
    }
}

static void update_metric_speed(float speed)
{
    s_data.speed_kph = speed;
    s_data.valid = true;
    s_data.last_update_ms = esp_timer_get_time() / 1000;
}

static void update_metric_rpm(int rpm)
{
    s_data.rpm = rpm;
    s_data.valid = true;
    s_data.last_update_ms = esp_timer_get_time() / 1000;
}

static void update_metric_throttle(float throttle)
{
    s_data.throttle_pct = throttle;
    s_data.valid = true;
    s_data.last_update_ms = esp_timer_get_time() / 1000;
}

static void update_metric_coolant(float temp_c)
{
    s_data.coolant_temp_c = temp_c;
    s_data.valid = true;
    s_data.last_update_ms = esp_timer_get_time() / 1000;
}

static void parse_obd_line(const char *line)
{
    if (!line || !line[0]) return;

    char hex_only[128];
    size_t n = 0;
    for (size_t i = 0; line[i] != '\0' && n < sizeof(hex_only) - 1; i++) {
        if (is_hex_char(line[i])) {
            hex_only[n++] = (char)toupper((unsigned char)line[i]);
        }
    }
    hex_only[n] = '\0';

    if (n < 4 || (n % 2) != 0) return;

    uint8_t bytes[64];
    size_t count = 0;
    for (size_t i = 0; i + 1 < n && count < sizeof(bytes); i += 2) {
        int8_t hi = hex_nibble(hex_only[i]);
        int8_t lo = hex_nibble(hex_only[i + 1]);
        if (hi < 0 || lo < 0) return;
        bytes[count++] = (uint8_t)((hi << 4) | lo);
    }
    if (count < 2) return;

    // VIN response (Mode 09 PID 02): look for 49 02 then ASCII payload bytes.
    for (size_t i = 0; i + 1 < count; i++) {
        if (bytes[i] == 0x49 && bytes[i + 1] == 0x02) {
            if (i + 2 < count) {
                vin_consume_bytes(&bytes[i + 2], count - (i + 2));
            }
            return;
        }
    }

    if (bytes[0] != 0x41) return;

    switch (bytes[1]) {
    case 0x0D:  // speed
        if (count >= 3) update_metric_speed((float)bytes[2]);
        break;
    case 0x0C:  // rpm
        if (count >= 4) {
            int rpm = (int)(((bytes[2] << 8) | bytes[3]) / 4.0f);
            update_metric_rpm(rpm);
        }
        break;
    case 0x11:  // throttle
        if (count >= 3) {
            float throttle = ((float)bytes[2] * 100.0f) / 255.0f;
            update_metric_throttle(throttle);
        }
        break;
    case 0x05:  // coolant
        if (count >= 3) {
            float temp_c = (float)bytes[2] - 40.0f;
            update_metric_coolant(temp_c);
        }
        break;
    default:
        break;
    }
}

static void handle_obd_rx_bytes(const uint8_t *data, uint16_t len)
{
    for (uint16_t i = 0; i < len; i++) {
        char c = (char)data[i];
        if (c == '\r' || c == '\n' || c == '>') {
            if (s_rx_len > 0) {
                s_rx_line[s_rx_len] = '\0';
                parse_obd_line(s_rx_line);
                s_rx_len = 0;
            }
            continue;
        }
        if (s_rx_len < sizeof(s_rx_line) - 1) {
            s_rx_line[s_rx_len++] = c;
        } else {
            s_rx_len = 0;
        }
    }
}

static esp_err_t obd_write(const char *cmd)
{
    if (!s_obd_ready || s_conn_handle == BLE_HS_CONN_HANDLE_NONE || s_chr_val_handle == 0) {
        return ESP_ERR_INVALID_STATE;
    }
    int rc;
    if (s_chr_props & BLE_GATT_CHR_PROP_WRITE_NO_RSP) {
        rc = ble_gattc_write_no_rsp_flat(s_conn_handle, s_chr_val_handle, cmd, strlen(cmd));
    } else {
        rc = ble_gattc_write_flat(s_conn_handle, s_chr_val_handle, cmd, strlen(cmd), NULL, NULL);
    }
    if (rc != 0) {
        return ESP_FAIL;
    }
    return ESP_OK;
}

static void obd_write_blocking(const char *cmd, int wait_ms)
{
    if (obd_write(cmd) == ESP_OK) {
        vTaskDelay(pdMS_TO_TICKS(wait_ms));
    }
}

static int cccd_write_cb(uint16_t c_handle, const struct ble_gatt_error *error,
                         struct ble_gatt_attr *attr, void *arg)
{
    (void)c_handle;
    (void)attr;
    (void)arg;
    if (error->status == 0) {
        s_obd_ready = true;
        s_obd_initialized = false;
        s_data.connected = true;
        ESP_LOGI(TAG, "ELM notifications enabled");
    } else {
        ESP_LOGW(TAG, "CCCD write failed: %d", error->status);
    }
    return 0;
}

static int dsc_discovery_cb(uint16_t c_handle, const struct ble_gatt_error *error,
                            uint16_t chr_val_handle, const struct ble_gatt_dsc *dsc, void *arg)
{
    (void)c_handle;
    (void)chr_val_handle;
    (void)arg;

    if (error->status == 0 && dsc != NULL) {
        if (ble_uuid_u16(&dsc->uuid.u) == CCCD_UUID16) {
            s_cccd_handle = dsc->handle;
        }
        return 0;
    }

    if (error->status == BLE_HS_EDONE) {
        if (s_cccd_handle != 0) {
            const uint8_t enable_notify[2] = {0x01, 0x00};
            ble_gattc_write_flat(s_conn_handle, s_cccd_handle, enable_notify,
                                 sizeof(enable_notify), cccd_write_cb, NULL);
        }
    }
    return 0;
}

static int chr_discovery_cb(uint16_t c_handle, const struct ble_gatt_error *error,
                            const struct ble_gatt_chr *chr, void *arg)
{
    (void)c_handle;
    (void)arg;

    if (error->status == 0 && chr != NULL) {
        s_chr_def_handle = chr->def_handle;
        s_chr_val_handle = chr->val_handle;
        s_chr_props = chr->properties;
        return 0;
    }

    if (error->status == BLE_HS_EDONE && s_chr_val_handle != 0) {
        ble_gattc_disc_all_dscs(s_conn_handle, s_chr_def_handle, s_svc_end_handle, dsc_discovery_cb, NULL);
    }
    return 0;
}

static int svc_discovery_cb(uint16_t c_handle, const struct ble_gatt_error *error,
                            const struct ble_gatt_svc *svc, void *arg)
{
    (void)c_handle;
    (void)arg;

    if (error->status == 0 && svc != NULL) {
        s_service_found = true;
        s_svc_end_handle = svc->end_handle;
        const ble_uuid16_t chr_uuid = BLE_UUID16_INIT(OBD_CHAR_UUID16);
        ble_gattc_disc_chrs_by_uuid(s_conn_handle, svc->start_handle, svc->end_handle,
                                    &chr_uuid.u, chr_discovery_cb, NULL);
        return 0;
    }
    return 0;
}

static bool adv_has_obd_service(const struct ble_hs_adv_fields *fields)
{
    for (int i = 0; i < fields->num_uuids16; i++) {
        if (fields->uuids16[i].value == OBD_SERVICE_UUID16) return true;
    }
    return false;
}

static int gap_event_cb(struct ble_gap_event *event, void *arg)
{
    (void)arg;

    switch (event->type) {
    case BLE_GAP_EVENT_DISC: {
        struct ble_hs_adv_fields fields;
        int rc = ble_hs_adv_parse_fields(&fields, event->disc.data, event->disc.length_data);
        if (rc == 0 && adv_has_obd_service(&fields) && addr_matches_target(&event->disc.addr)) {
            ble_gap_disc_cancel();
            s_scan_active = false;
            rc = ble_gap_connect(s_own_addr_type, &event->disc.addr, 30000, NULL, gap_event_cb, NULL);
            if (rc != 0) {
                schedule_scan_retry();
            }
        }
        return 0;
    }
    case BLE_GAP_EVENT_CONNECT: {
        if (event->connect.status != 0) {
            schedule_scan_retry();
            return 0;
        }
        s_scan_backoff_ms = 1000;
        s_next_scan_at_ms = 0;
        s_conn_handle = event->connect.conn_handle;
        s_obd_ready = false;
        s_obd_initialized = false;
        s_service_found = false;
        s_data.connected = true;
        s_rx_len = 0;
        vin_reset();

        const ble_uuid16_t svc_uuid = BLE_UUID16_INIT(OBD_SERVICE_UUID16);
        ble_gattc_disc_svc_by_uuid(s_conn_handle, &svc_uuid.u, svc_discovery_cb, NULL);
        return 0;
    }
    case BLE_GAP_EVENT_NOTIFY_RX: {
        uint8_t data[256];
        uint16_t data_len = OS_MBUF_PKTLEN(event->notify_rx.om);
        if (data_len > sizeof(data)) data_len = sizeof(data);
        if (ble_hs_mbuf_to_flat(event->notify_rx.om, data, data_len, NULL) == 0) {
            handle_obd_rx_bytes(data, data_len);
        }
        return 0;
    }
    case BLE_GAP_EVENT_DISCONNECT:
        mark_disconnected();
        return 0;
    case BLE_GAP_EVENT_DISC_COMPLETE:
        s_scan_active = false;
        schedule_scan_retry();
        return 0;
    default:
        return 0;
    }
}

static void start_scan(void)
{
    if (!s_enabled) return;
    if (s_scan_active) return;
    if (s_conn_handle != BLE_HS_CONN_HANDLE_NONE) return;
    int64_t now_ms = esp_timer_get_time() / 1000;
    if (now_ms < s_next_scan_at_ms) return;
    struct ble_gap_disc_params params = {0};
    params.filter_duplicates = 1;
    params.passive = 0;
    params.itvl = 0x0010;
    params.window = 0x0010;
    if (ble_gap_disc(s_own_addr_type, BLE_HS_FOREVER, &params, gap_event_cb, NULL) == 0) {
        s_scan_active = true;
    }
}

static void schedule_scan_retry(void)
{
    int64_t now_ms = esp_timer_get_time() / 1000;
    s_next_scan_at_ms = now_ms + s_scan_backoff_ms;
    s_scan_backoff_ms *= 2;
    if (s_scan_backoff_ms > 15000) {
        s_scan_backoff_ms = 15000;
    }
}

static void on_sync(void)
{
    if (ble_hs_id_infer_auto(0, &s_own_addr_type) != 0) return;
    if (s_enabled) start_scan();
}

static void on_reset(int reason)
{
    ESP_LOGW(TAG, "BLE host reset: %d", reason);
}

static void nimble_host_task(void *param)
{
    (void)param;
    nimble_port_run();
    nimble_port_freertos_deinit();
}

static void obd_poll_task(void *arg)
{
    (void)arg;
    TickType_t last_fast = 0;
    TickType_t last_medium = 0;
    TickType_t last_tx = 0;
    TickType_t last_vin_req = 0;
    uint8_t medium_index = 0;

    while (1) {
        if (!s_enabled) {
            vTaskDelay(pdMS_TO_TICKS(300));
            continue;
        }
        if (!s_obd_ready) {
            vTaskDelay(pdMS_TO_TICKS(200));
            continue;
        }

        if (!s_obd_initialized) {
            ESP_LOGI(TAG, "Initializing ELM327");
            obd_write_blocking("ATZ\r", 1400);
            obd_write_blocking("ATE0\r", 300);
            obd_write_blocking("ATL0\r", 300);
            obd_write_blocking("ATS0\r", 300);
            obd_write_blocking("ATH0\r", 300);
            obd_write_blocking("ATSP0\r", 500);
            obd_write_blocking("0100\r", 600);
            obd_write_blocking("0902\r", 1200);  // VIN request
            s_obd_initialized = true;
            last_tx = xTaskGetTickCount();
            last_fast = last_tx;
            last_medium = last_tx;
            last_vin_req = last_tx;
            continue;
        }

        TickType_t now = xTaskGetTickCount();
        if ((now - last_tx) < pdMS_TO_TICKS(TX_GUARD_MS)) {
            vTaskDelay(pdMS_TO_TICKS(20));
            continue;
        }

        bool voice_busy = false;
        call_service_state_t cs = call_service_get_state();
        if (cs == CALL_STATE_DIALING || cs == CALL_STATE_RINGING || cs == CALL_STATE_ACTIVE) {
            voice_busy = true;
        }
        int fast_ms = s_cfg.fast_interval_ms;
        int medium_ms = s_cfg.medium_interval_ms;
        if (voice_busy) {
            if (fast_ms < 1500) fast_ms = 1500;
            if (medium_ms < 3000) medium_ms = 3000;
        }

        if ((now - last_fast) >= pdMS_TO_TICKS(fast_ms)) {
            static bool fast_toggle;
            last_fast = now;
            obd_write(fast_toggle ? "010D\r" : "010C\r");
            fast_toggle = !fast_toggle;
            last_tx = now;
        } else if ((now - last_medium) >= pdMS_TO_TICKS(medium_ms)) {
            last_medium = now;
            switch (medium_index) {
            case 0: obd_write("0111\r"); break;  // throttle
            default: obd_write("0105\r"); break; // coolant
            }
            medium_index = (uint8_t)((medium_index + 1) % 2);
            last_tx = now;
        } else if (!s_data.vin_valid && (now - last_vin_req) >= pdMS_TO_TICKS(30000)) {
            obd_write("0902\r");
            last_vin_req = now;
            last_tx = now;
        }
        vTaskDelay(pdMS_TO_TICKS(20));
    }
}

static void obd_link_task(void *arg)
{
    (void)arg;
    while (1) {
        if (!s_enabled) {
            if (s_conn_handle != BLE_HS_CONN_HANDLE_NONE || s_scan_active) {
                stop_ble_runtime();
            }
        } else if (s_conn_handle == BLE_HS_CONN_HANDLE_NONE) {
            start_scan();
        }
        vTaskDelay(pdMS_TO_TICKS(1000));
    }
}

#endif

esp_err_t obd_service_init(const obd_service_config_t *cfg)
{
    if (s_inited) return ESP_OK;
    memset(&s_data, 0, sizeof(s_data));
#if CONFIG_BT_ENABLED && CONFIG_BT_NIMBLE_ENABLED
    vin_reset();
#endif
    s_cfg = (obd_service_config_t) {
        .target_mac = NULL,
        .fast_interval_ms = 600,
        .medium_interval_ms = 1500,
    };
    if (cfg) {
        if (cfg->target_mac) s_cfg.target_mac = cfg->target_mac;
        if (cfg->fast_interval_ms > 0) s_cfg.fast_interval_ms = cfg->fast_interval_ms;
        if (cfg->medium_interval_ms > 0) s_cfg.medium_interval_ms = cfg->medium_interval_ms;
    }

#if CONFIG_BT_ENABLED && CONFIG_BT_NIMBLE_ENABLED
    if (s_cfg.target_mac && parse_mac_string(s_cfg.target_mac, s_target_mac)) {
        s_target_mac_enabled = true;
    } else {
        s_target_mac_enabled = false;
    }
#else
    ESP_LOGW(TAG, "NimBLE disabled in sdkconfig; OBD service will stay inactive");
#endif

    s_inited = true;
    return ESP_OK;
}

void obd_service_start(void)
{
    if (!s_inited || s_started) return;
    s_started = true;

#if CONFIG_BT_ENABLED && CONFIG_BT_NIMBLE_ENABLED
    int rc = nimble_port_init();
    if (rc != ESP_OK) {
        ESP_LOGE(TAG, "nimble_port_init failed: %d", rc);
        return;
    }
    ble_svc_gap_init();
    ble_svc_gatt_init();
    ble_hs_cfg.reset_cb = on_reset;
    ble_hs_cfg.sync_cb = on_sync;
    xTaskCreate(obd_poll_task, "obd_poll", 4096, NULL, 5, NULL);
    xTaskCreate(obd_link_task, "obd_link", 3072, NULL, 4, NULL);
    nimble_port_freertos_init(nimble_host_task);
#endif
}

bool obd_service_get_latest(obd_data_t *out)
{
    if (!out) return false;
    *out = s_data;
    return s_data.valid;
}

void obd_service_set_enabled(bool enabled)
{
    s_enabled = enabled;
#if CONFIG_BT_ENABLED && CONFIG_BT_NIMBLE_ENABLED
    if (!enabled) {
        stop_ble_runtime();
        ESP_LOGI(TAG, "OBD service disabled");
    } else {
        s_scan_backoff_ms = 1000;
        s_next_scan_at_ms = 0;
        ESP_LOGI(TAG, "OBD service enabled");
    }
#else
    (void)enabled;
#endif
}

bool obd_service_is_enabled(void)
{
    return s_enabled;
}
