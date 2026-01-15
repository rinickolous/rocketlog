#pragma once

#include <stdbool.h>

#ifdef __cplusplus
extern "C" {
#endif

double rocketlog_monotonic_seconds(void);

double rocketlog_current_unix_time(void);

void rocketlog_time_set_unix(double unix_time);

bool rocketlog_time_is_set(void);

#ifdef __cplusplus
}
#endif
