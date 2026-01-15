#pragma once

#include <stddef.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

#define TELEMETRY_FIELDS_COUNT 5

typedef struct {
	double unix_time;
	float altitude;
	float velocity;
	float battery;
	float temperature;
} telemetry_sample_t;

/**
 * Encode a telemetry sample as the ASCII CSV line used on USB/UART.
 * Output is NUL-terminated and includes a trailing '\n'.
 */
int telemetry_csv_encode(char *out, size_t out_len, const telemetry_sample_t *sample);

/**
 * Parse a telemetry sample from the ASCII CSV line used on USB/UART.
 * Accepts optional trailing '\n'.
 */
int telemetry_csv_decode(telemetry_sample_t *out, const char *line);

#ifdef __cplusplus
}
#endif
