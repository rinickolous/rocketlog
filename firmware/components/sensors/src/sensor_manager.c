#include "sensor_manager.h"
#include "rocketlog_time.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

static const char *TAG = "sensor_manager";

static bool manager_initialized = false;

// Public API implementation
esp_err_t sensor_manager_init(void) {
	if (manager_initialized) {
		return ESP_OK;
	}

	ESP_LOGI(TAG, "Initializing sensor manager...");

	// Initialize MPL3115A2 barometer
	esp_err_t baro_err = mpl3115a2_init();
	if (baro_err == ESP_OK) {
		ESP_LOGI(TAG, "Barometer initialized successfully");
	} else {
		ESP_LOGE(TAG, "Barometer initialization failed: %s", esp_err_to_name(baro_err));
	}

	// Initialize GPS
	esp_err_t gps_err = gps_init();
	if (gps_err == ESP_OK) {
		ESP_LOGI(TAG, "GPS initialized successfully");
	} else {
		ESP_LOGE(TAG, "GPS initialization failed: %s", esp_err_to_name(gps_err));
	}

	manager_initialized = true;
	ESP_LOGI(TAG, "Sensor manager initialization complete");

	return (baro_err == ESP_OK || gps_err == ESP_OK) ? ESP_OK : ESP_FAIL;
}

esp_err_t sensor_manager_read_all(sensor_data_t *data) {
	if (!manager_initialized || data == NULL) {
		return ESP_ERR_INVALID_STATE;
	}

	// Get current timestamp
	data->last_read_ms = (uint32_t)(rocketlog_monotonic_seconds() * 1000.0);

	// Read barometer
	data->barometer_ready = false;
	if (mpl3115a2_is_ready()) {
		esp_err_t baro_err = mpl3115a2_read(&data->barometer);
		data->barometer_ready = (baro_err == ESP_OK && data->barometer.valid);

		if (data->barometer_ready) {
			ESP_LOGD(TAG, "Barometer: pressure=%.1f Pa, temp=%.1f°C", data->barometer.pressure_pa,
					 data->barometer.temperature_c);
		} else {
			ESP_LOGW(TAG, "Barometer read failed: %s", esp_err_to_name(baro_err));
		}
	}

	// Read GPS
	data->gps_ready = false;
	esp_err_t gps_err = gps_read(&data->gps);

	/* ESP_LOGW(TAG, "gps_err: 0x%x", gps_err); */
	/* ESP_LOGW(TAG, "GPS Data: lat=%.6f, lon=%.6f, alt=%.2f, sats=%d, fix=%d", data->gps.latitude, data->gps.longitude,
	 */
	/* 		 data->gps.altitude, data->gps.satellites, data->gps.fix_quality); */

	// Only consider GPS ready if module is detected AND we have valid data
	data->gps_ready = (gps_err == ESP_OK && data->gps.valid && gps_is_detected());

	if (data->gps_ready) {
		ESP_LOGD(TAG, "GPS: lat=%.6f, lon=%.6f, alt=%.1fm, sats=%d, hdop=%.1f", data->gps.latitude, data->gps.longitude,
				 data->gps.altitude, data->gps.satellites, data->gps.hdop);
	} else if (!gps_is_detected()) {
		ESP_LOGE(TAG, "GPS module not detected - check connections");
	} else {
		ESP_LOGD(TAG, "GPS module detected but no fix");
	}

	return ESP_OK;
}

bool sensor_manager_all_ready(void) {
	if (!manager_initialized) {
		return false;
	}

	return mpl3115a2_is_ready() && gps_has_fix();
}

bool sensor_manager_get_health(void) {
	if (!manager_initialized) {
		return false;
	}

	// Consider system healthy if at least one sensor is working
	bool baro_ok = mpl3115a2_is_ready();
	bool gps_ok = gps_is_detected(); // Check if GPS module is detected, not just if it has fix

	ESP_LOGD(TAG, "Sensor health - Barometer: %s, GPS detected: %s", baro_ok ? "OK" : "FAIL",
			 gps_ok ? "OK" : "NOT DETECTED");

	return (baro_ok || gps_ok);
}
