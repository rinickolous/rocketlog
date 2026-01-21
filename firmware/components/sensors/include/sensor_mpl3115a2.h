#pragma once

#include <stdint.h>
#include <stdbool.h>
#include "esp_err.h"

// MPL3115A2 barometric pressure sensor interface

typedef struct {
	float pressure_pa;
	float temperature_c;
	bool valid;
} mpl3115a2_data_t;

// Initialize the MPL3115A2 sensor
esp_err_t mpl3115a2_init(void);

// Read pressure and temperature data
esp_err_t mpl3115a2_read(mpl3115a2_data_t *data);

// Check if sensor is initialized and ready
bool mpl3115a2_is_ready(void);
