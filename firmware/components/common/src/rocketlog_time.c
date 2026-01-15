#include "rocketlog_time.h"

#include <stdbool.h>

#include "esp_timer.h"

static double unix_time_at_sync = 0.0;
static double monotonic_time_at_sync = 0.0;
static bool time_is_set = false;

double rocketlog_monotonic_seconds(void) {
	return (double)esp_timer_get_time() / 1e6;
}

double rocketlog_current_unix_time(void) {
	if (!time_is_set) {
		return 0.0;
	}

	const double now = rocketlog_monotonic_seconds();
	return unix_time_at_sync + (now - monotonic_time_at_sync);
}

void rocketlog_time_set_unix(double unix_time) {
	unix_time_at_sync = unix_time;
	monotonic_time_at_sync = rocketlog_monotonic_seconds();
	time_is_set = true;
}

bool rocketlog_time_is_set(void) {
	return time_is_set;
}
