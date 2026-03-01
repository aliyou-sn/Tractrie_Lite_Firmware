#pragma once

#include <stdbool.h>

#ifdef __cplusplus
extern "C" {
#endif

typedef void (*modem_urc_handler_t)(const char *line, void *user);

void modem_urc_init(void);
bool modem_urc_register(const char *pattern, modem_urc_handler_t handler, void *user);
void modem_urc_dispatch(const char *line);

#ifdef __cplusplus
}
#endif
