#include "modem_urc.h"
#include "esp_log.h"
#include <string.h>

#define MODEM_URC_MAX_HANDLERS 16

static const char *TAG = "modem_urc";

typedef struct {
    const char *pattern;
    modem_urc_handler_t handler;
    void *user;
} modem_urc_entry_t;

static modem_urc_entry_t s_entries[MODEM_URC_MAX_HANDLERS];

void modem_urc_init(void)
{
    memset(s_entries, 0, sizeof(s_entries));
}

bool modem_urc_register(const char *pattern, modem_urc_handler_t handler, void *user)
{
    if (!pattern || !handler) {
        return false;
    }
    for (int i = 0; i < MODEM_URC_MAX_HANDLERS; i++) {
        if (s_entries[i].pattern == NULL) {
            s_entries[i].pattern = pattern;
            s_entries[i].handler = handler;
            s_entries[i].user = user;
            return true;
        }
    }
    return false;
}

void modem_urc_dispatch(const char *line)
{
    if (!line || !line[0]) {
        return;
    }

    for (int i = 0; i < MODEM_URC_MAX_HANDLERS; i++) {
        if (!s_entries[i].pattern) {
            continue;
        }
        if (strstr(line, s_entries[i].pattern) != NULL) {
            s_entries[i].handler(line, s_entries[i].user);
            return;
        }
    }

    ESP_LOGI(TAG, "URC: %s", line);
}
