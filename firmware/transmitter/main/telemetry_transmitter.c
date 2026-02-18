#include "FreeRTOSConfig.h"
#include "esp_log.h"

#include "freertos/idf_additions.h"
#include "freertos/projdefs.h"
#include "portmacro.h"
#include "sensor_gps.h"
#include "sensor_manager.h"

/* ---------------------------------------- */
/*  Config Knobs                            */
/* ---------------------------------------- */

// Sample rate of 1Hz is the maximum sample rate of the MPL3115A2 barometer.
static const TickType_t SAMPLE_PERIOD = pdMS_TO_TICKS(1000);

static const char *TAG = "transmitter";

#define RGB_LED_GPIO ROCKETLOG_RGB_LED_GPIO

/* static led_strip_handle_t led_strip; */

/* ---------------------------------------- */

static void telemetry_transmitter_task(void *arg) {
	(void)arg;

	sensor_data_t sensor_data = {0};

	while (1) {
		esp_err_t err = sensor_manager_read_all(&sensor_data);
		if (err != ESP_OK) {
			ESP_LOGE(TAG, "Failed to read sensors: %s", esp_err_to_name(err));
		} else {
			ESP_LOGI(TAG, "Sensor data read successfully: barometer_ready=%d gps_ready=%d", sensor_data.barometer_ready,
					 sensor_data.gps_ready);
		}

		if (sensor_data.barometer_ready) {
			ESP_LOGI(TAG, "Barometer: %.1f Pa, %.1f C", sensor_data.barometer.pressure_pa,
					 sensor_data.barometer.temperature_c);
		}

		if (sensor_data.gps_ready) {
			ESP_LOGI(TAG, "GPS: lat=%.6f, lon=%.6f, alt=%.1fm, sats=%d", sensor_data.gps.latitude,
					 sensor_data.gps.longitude, sensor_data.gps.altitude_m, sensor_data.gps.num_satellites);
		} else {
			ESP_LOGW(TAG, "GPS does not have a fix");
		}

		vTaskDelay(SAMPLE_PERIOD);
	}
}

/* ---------------------------------------- */

/**
 * Application entry point. Initializes the GPS sensor and logs the status. If the GPS initialization fails, it
 * logs an error.
 */
void app_main(void) {
	ESP_LOGI(TAG, "Telemetry Transmitter starting");

	ESP_LOGI(TAG, "Initializing all sensors through the sensor manager");
	esp_err_t err = sensor_manager_init();
	if (err == ESP_OK) {
		ESP_LOGI(TAG, "Sensors initialized successfully");
	} else {
		ESP_LOGE(TAG, "Sensor initialization failed: %s", esp_err_to_name(err));
	}

	ESP_LOGI(TAG, "Telemetry Transmitter setup complete");
	ESP_LOGI(TAG, "Entering main loop");
	xTaskCreate(telemetry_transmitter_task, "telemetry_transmitter_task", 4096, NULL, 5, NULL);
}
