#include "sensor_manager.h"
#include "esp_err.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "rocketlog_time.h"

/* ---------------------------------------- */
/*  Config Knobs                            */
/* ---------------------------------------- */

/* TAG for logging */
static const char *TAG = "sensor_manager";

/* State variable to track if the sensor manager has already been initialized. This ensures that the initialization
 process is only performed once, even if sensor_manager_init() is called multiple times. */
static bool manager_initialized = false;

/* ---------------------------------------- */

/**
 * Read all sensors and update the provided data structure with the latest readings. This function first checks if the
 * component has been initialized and if the provided data pointer is valid. It then gets the current timestamp and
 * attempts to read the barometer and GPS data. For each sensor, it checks if the data is ready and valid before
 * updating the corresponding fields in the sensor_data_t structure. The function also includes logging to provide
 * feedback on the status of each sensor read operation, including any errors encountered. The barometer data is
 * considered ready if the MPL3115A2 sensor indicates that new data is available and the read operation is successful,
 * while the GPS data is considered ready if the read operation is successful and the data indicates a valid fix.
 *
 * @return ESP_OK on success, or an appropriate error code if the component is not initialized or if the data pointer is
 * invalid
 */
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
	} else {
		ESP_LOGW(TAG, "Barometer data not ready");
	}

	// Read GPS
	data->gps_ready = false;
	esp_err_t gps_err = gps_read(&data->gps);

	// Only consider GPS data ready if the read was successful and the data is valid (has a fix)
	data->gps_ready = (gps_err == ESP_OK && data->gps.valid);

	if (data->gps_ready) {
		ESP_LOGD(TAG, "GPS: lat=%.6f, lon=%.6f, alt=%.1f m, fix=%d, sats=%d, hdop=%.1f", data->gps.latitude,
				 data->gps.longitude, data->gps.altitude_m, data->gps.fix_quality, data->gps.num_satellites,
				 data->gps.hdop);
	} else {
		if (gps_err == ESP_OK) {
			ESP_LOGW(TAG, "GPS data not ready or no fix");
		} else {
			ESP_LOGE(TAG, "GPS read failed: %s", esp_err_to_name(gps_err));
		}
	}

	return ESP_OK;
}

/* ---------------------------------------- */

/**
 * Initialize the sensor manager by initializing each individual sensor. This function first checks if the manager has
 * already been initialized and returns success if so, allowing for multiple calls without issues. It then attempts to
 * initialize the MPL3115A2 barometer and the GPS module, logging the results of each initialization attempt. If either
 * sensor is successfully initialized, the function returns ESP_OK; otherwise, it returns ESP_FAIL to indicate that
 * initialization was unsuccessful.
 *
 * @return ESP_OK if at least one sensor was initialized successfully, or ESP_FAIL if both sensors failed to initialize
 */
esp_err_t sensor_manager_init(void) {
	// If the sensor manager is already initialized, simply return success. This allows multiple calls to
	// sensor_manager_init()
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
