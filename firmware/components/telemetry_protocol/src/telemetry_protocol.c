#include "telemetry_protocol.h"

#include <stdio.h>

int telemetry_csv_encode(char *out, size_t out_len, const telemetry_sample_t *sample) {
	if (out == NULL || sample == NULL || out_len == 0) {
		return -1;
	}

	const int written =
		snprintf(out, out_len, "%.7f,%.2f,%.2f,%.3f,%.2f\n", sample->unix_time, (double)sample->altitude,
				 (double)sample->velocity, (double)sample->battery, (double)sample->temperature);

	if (written < 0 || (size_t)written >= out_len) {
		return -1;
	}

	return written;
}

int telemetry_csv_decode(telemetry_sample_t *out, const char *line) {
	if (out == NULL || line == NULL) {
		return -1;
	}

	double unix_time = 0.0;
	float altitude = 0.0f;
	float velocity = 0.0f;
	float battery = 0.0f;
	float temperature = 0.0f;

	// Allow optional trailing newline by letting sscanf stop early.
	const int matched = sscanf(line, "%lf,%f,%f,%f,%f", &unix_time, &altitude, &velocity, &battery, &temperature);
	if (matched != TELEMETRY_FIELDS_COUNT) {
		return -1;
	}

	out->unix_time = unix_time;
	out->altitude = altitude;
	out->velocity = velocity;
	out->battery = battery;
	out->temperature = temperature;

	return 0;
}
