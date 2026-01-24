#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"
#include "driver/uart.h"
#include "esp_system.h"

static const char *TAG = "gps_test";

// GPS UART configuration - using confirmed working pins
#define GPS_UART_NUM UART_NUM_1
#define GPS_TX_GPIO 17
#define GPS_RX_GPIO 16
#define GPS_BAUD_RATE 9600
#define GPS_BUF_SIZE 1024
#define GPS_TASK_STACK_SIZE 4096
#define GPS_TASK_PRIORITY 5

static uart_port_t gps_uart = GPS_UART_NUM;
static bool gps_initialized = false;

// Minimal GPS data processing task
static void gps_task(void *arg) {
	(void)arg;
	uint8_t *data = malloc(GPS_BUF_SIZE);

	ESP_LOGI(TAG, "GPS test task started");
	ESP_LOGI(TAG, "Configuration: UART=%d, TX=%d, RX=%d, baud=%d", GPS_UART_NUM, GPS_TX_GPIO, GPS_RX_GPIO,
			 GPS_BAUD_RATE);

	while (1) {
		int len = uart_read_bytes(gps_uart, data, GPS_BUF_SIZE, pdMS_TO_TICKS(100));
		if (len > 0) {
			// Log raw data as both hex and ASCII for debugging
			ESP_LOGI(TAG, "RX %d bytes:", len);

			// Print hex dump (first 16 bytes max)
			for (int i = 0; i < len && i < 16; i++) {
				printf("%02x ", data[i]);
			}

			// Print ASCII representation (first 32 bytes max)
			printf(" | ");
			for (int i = 0; i < len && i < 32; i++) {
				printf("%c", (data[i] >= 32 && data[i] <= 126) ? data[i] : '.');
			}
			printf("\n");

			// Look for NMEA sentences
			bool found_nmea = false;
			for (int i = 0; i < len - 2; i++) {
				if (data[i] == '$' && data[i + 1] == 'G' && data[i + 2] == 'P') {
					found_nmea = true;
					break;
				}
			}

			if (found_nmea) {
				ESP_LOGI(TAG, "NMEA sentence detected!");
			}
		} else if (len == 0) {
			ESP_LOGD(TAG, "No data received");
		} else {
			ESP_LOGW(TAG, "UART read error: %d", len);
		}

		vTaskDelay(pdMS_TO_TICKS(1000));
	}

	free(data);
	vTaskDelete(NULL);
}

esp_err_t gps_init(void) {
	if (gps_initialized) {
		return ESP_OK;
	}

	ESP_LOGI(TAG, "Initializing GPS test...");
	vTaskDelay(pdMS_TO_TICKS(500));

	// Configure UART
	uart_config_t uart_cfg = {
		.baud_rate = GPS_BAUD_RATE,
		.data_bits = UART_DATA_8_BITS,
		.parity = UART_PARITY_DISABLE,
		.stop_bits = UART_STOP_BITS_1,
		.flow_ctrl = UART_HW_FLOWCTRL_DISABLE,
		.source_clk = UART_SCLK_DEFAULT,
	};

	ESP_LOGI(TAG, "Installing UART driver...");
	vTaskDelay(pdMS_TO_TICKS(500));

	esp_err_t err = uart_driver_install(gps_uart, GPS_BUF_SIZE, 0, 0, NULL, 0);
	ESP_LOGE(TAG, "err: %s", esp_err_to_name(err));
	if (err != ESP_OK) {
		ESP_LOGE(TAG, "Failed to install UART driver: %s", esp_err_to_name(err));
		return err;
	}

	ESP_LOGI(TAG, "Configuring UART parameters...");
	vTaskDelay(pdMS_TO_TICKS(500));

	err = uart_param_config(gps_uart, &uart_cfg);
	ESP_LOGE(TAG, "err: %s", esp_err_to_name(err));
	if (err != ESP_OK) {
		ESP_LOGE(TAG, "Failed to configure UART: %s", esp_err_to_name(err));
		uart_driver_delete(gps_uart);
		return err;
	}

	ESP_LOGI(TAG, "Setting UART pins (TX=%d, RX=%d)...", GPS_TX_GPIO, GPS_RX_GPIO);
	vTaskDelay(pdMS_TO_TICKS(500));

	err = uart_set_pin(gps_uart, GPS_TX_GPIO, GPS_RX_GPIO, UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE);
	if (err != ESP_OK) {
		ESP_LOGE(TAG, "Failed to set UART pins: %s", esp_err_to_name(err));
		uart_driver_delete(gps_uart);
		return err;
	}

	ESP_LOGI(TAG, "Creating GPS test task...");
	vTaskDelay(pdMS_TO_TICKS(500));

	// Create GPS test task
	BaseType_t task_err = xTaskCreate(gps_task, "gps_test_task", GPS_TASK_STACK_SIZE, NULL, GPS_TASK_PRIORITY, NULL);
	if (task_err != pdPASS) {
		ESP_LOGE(TAG, "Failed to create GPS test task");
		uart_driver_delete(gps_uart);
		return ESP_ERR_NO_MEM;
	}

	gps_initialized = true;
	ESP_LOGI(TAG, "GPS test initialized successfully");

	return ESP_OK;
}

void app_main(void) {
	ESP_LOGI(TAG, "=== GPS Test Firmware Starting ===");
	ESP_LOGI(TAG, "Just a sanity check before we can see what's crashing it.");

	// Initialize GPS with working configuration
	esp_err_t err = gps_init();
	if (err != ESP_OK) {
		ESP_LOGE(TAG, "GPS initialization failed: %s", esp_err_to_name(err));
		ESP_LOGE(TAG, "Restarting in 5 seconds...");
		vTaskDelay(pdMS_TO_TICKS(5000));
		ESP_LOGI(TAG, "Just kidding!");
		/* esp_restart(); */
	}

	ESP_LOGI(TAG, "GPS test ready - monitoring for data...");

	// Main task can exit - GPS task handles everything
	while (1) {
		vTaskDelay(pdMS_TO_TICKS(10000)); // 10 second intervals
		ESP_LOGI(TAG, "GPS test running - Free heap: %d bytes", esp_get_free_heap_size());
	}
}
