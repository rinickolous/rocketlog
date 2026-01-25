#include "FreeRTOSConfig.h"
#include "esp_log.h"

#include "freertos/projdefs.h"
#include "led_strip_types.h"
#include "portmacro.h"
#include "sensor_gps.h"

/* ---------------------------------------- */
/*  Config Knobs                            */
/* ---------------------------------------- */

// Sample rate of 1Hz is the maximum sample rate of the MPL3115A2 barometer.
/* static const TickType_t SAMPLE_PERIOD = pdMS_TO_TICKS(1000); */

static const char *TAG = "transmitter";

/* #define RGB_LED_GPIO ROCKETLOG_RGB_LED_GPIO */

/* static led_strip_handle_t led_strip; */

/* ---------------------------------------- */

void app_main(void) {
	// Application entry point
	ESP_LOGI(TAG, "Telemetry Transmitter starting");

	esp_err_t err = gps_init();
	if (err != ESP_OK) {
		ESP_LOGE(TAG, "GPS initialization failed: %s", esp_err_to_name(err));
		return;
	}
}
