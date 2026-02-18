#pragma once

#include "esp_err.h"
#include <stdbool.h>
#include <stdint.h>

typedef struct {
	double latitude;		// Degrees, positive = North, negative = South
	double longitude;		// Degrees, positive = East, negative = West
	float altitude_m;		// Meters above sea level
	uint8_t num_satellites; // Number of satellites used
	uint8_t fix_quality;	// 0=invalid, 1=GPS, 2=DGPS
	float hdop;				// Horizontal dilution of precision
	bool valid;				// True if we have a valid fix
	double utc_time;		// UTC time in seconds since midnight
} gps_data_t;

// Initialize the GPS module (UART)
esp_err_t gps_init(void);

// Read and parse GPS data (non-blocking)
esp_err_t gps_read(gps_data_t *data);

// Check if GPS has a valid fix
bool gps_has_fix(void);

// Check if GPS module is detected and responsive
bool gps_is_detected(void);
