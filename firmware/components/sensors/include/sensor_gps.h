#pragma once

#include <stdint.h>
#include <stdbool.h>
#include "esp_err.h"

// GY-GPS6MV2 GPS module interface (UART-based)

typedef struct {
	double latitude;	 // Degrees, positive = North, negative = South
	double longitude;	 // Degrees, positive = East, negative = West
	float altitude;		 // Meters above sea level
	uint8_t satellites;	 // Number of satellites used
	uint8_t fix_quality; // 0=invalid, 1=GPS, 2=DGPS
	float hdop;			 // Horizontal dilution of precision
	bool valid;			 // True if we have a valid fix
	double utc_time;	 // UTC time in seconds since midnight
} gps_data_t;

// Initialize the GPS module (UART)
esp_err_t gps_init(void);

// Read and parse GPS data (non-blocking)
esp_err_t gps_read(gps_data_t *data);

// Check if GPS has a valid fix
bool gps_has_fix(void);

// Get raw NMEA sentences (for debugging)
esp_err_t gps_read_raw(char *buffer, size_t buffer_size, size_t *bytes_read);
