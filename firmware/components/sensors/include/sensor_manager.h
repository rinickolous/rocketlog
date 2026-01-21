#pragma once

#include <stdint.h>
#include <stdbool.h>
#include "esp_err.h"
#include "sensor_mpl3115a2.h"
#include "sensor_gps.h"

typedef struct {
	mpl3115a2_data_t barometer;
	gps_data_t gps;
	bool barometer_ready;
	bool gps_ready;
	uint32_t last_read_ms;
} sensor_data_t;

// Initialize all sensors
esp_err_t sensor_manager_init(void);

// Read all sensors (non-blocking)
esp_err_t sensor_manager_read_all(sensor_data_t *data);

// Check if all sensors are ready
bool sensor_manager_all_ready(void);

// Get sensor health status
bool sensor_manager_get_health(void);
