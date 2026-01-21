#include "telemetry_protocol.h"

#include <stdio.h>

int telemetry_csv_encode(char *out, size_t out_len, const telemetry_sample_t *sample) {
	if (out == NULL || sample == NULL || out_len == 0) {
		return -1;
	}

	// Include GPS fields in CSV: time,alt,vel,batt,temp,lat,lon,gps_alt,sats,gps_valid
	const int written = snprintf(out, out_len, "%.7f,%.2f,%.2f,%.3f,%.2f,%.6f,%.6f,%.1f,%d,%d\n", sample->unix_time,
								 (double)sample->altitude, (double)sample->velocity, (double)sample->battery,
								 (double)sample->temperature, sample->gps_latitude, sample->gps_longitude,
								 (double)sample->gps_altitude, (int)sample->gps_satellites, (int)sample->gps_valid);

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
	double gps_latitude = 0.0;
	double gps_longitude = 0.0;
	float gps_altitude = 0.0f;
	int gps_satellites = 0;
	int gps_valid = 0;

	// Try new 9-field format first: time,alt,vel,batt,temp,lat,lon,gps_alt,sats,gps_valid
	int matched = sscanf(line, "%lf,%f,%f,%f,%f,%lf,%lf,%f,%d,%d", &unix_time, &altitude, &velocity, &battery,
						 &temperature, &gps_latitude, &gps_longitude, &gps_altitude, &gps_satellites, &gps_valid);

	if (matched == 9) {
		// Old 5-field format for backward compatibility
		matched = sscanf(line, "%lf,%f,%f,%f,%f", &unix_time, &altitude, &velocity, &battery, &temperature);
		gps_latitude = 0.0;
		gps_longitude = 0.0;
		gps_altitude = 0.0f;
		gps_satellites = 0;
		gps_valid = 0;
	}

	if (matched < 5) {
		return -1;
	}

	out->unix_time = unix_time;
	out->altitude = altitude;
	out->velocity = velocity;
	out->battery = battery;
	out->temperature = temperature;
	out->gps_latitude = gps_latitude;
	out->gps_longitude = gps_longitude;
	out->gps_altitude = gps_altitude;
	out->gps_satellites = (uint8_t)gps_satellites;
	out->gps_valid = (bool)gps_valid;

	return 0;
}
