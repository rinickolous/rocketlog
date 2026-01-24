#include "sensor_gps.h"
#include "driver/uart.h"
#include "esp_err.h"
#include "esp_log.h"
#include "freertos/idf_additions.h"
#include "rocketlog_board.h"
#include <stdint.h>
#include <string.h>

/* ---------------------------------------- */
/*  Config Knobs                            */
/* ---------------------------------------- */

static const char *TAG = "sensor_gps";

#define GPS_UART_NUM UART_NUM_1
#define GPS_BAUD_RATE 9600
#define GPS_BUF_SIZE 1024
#define NMEA_MAX_SENTENCE 82
#define GPS_TASK_STACK_SIZE 4096
#define GPS_TASK_PRIORITY 5

/* NMEA sentence identifiers */
#define NMEA_GPGGA "$GPGGA"
#define NMEA_GPRMC "$GPRMC"

/* Static variables */
static uart_port_t gps_uart = GPS_UART_NUM;
static bool gps_initialized = false;
static gps_data_t last_gps_data = {0};
static char nmea_buffer[NMEA_MAX_SENTENCE];
static int nmea_index = 0;

/* ---------------------------------------- */

static uint8_t nmea_checksum(const char *sentence) {
	uint8_t checksum = 0;
	// Start after $ and before *
	for (int i = 1; sentence[i] != '*' && sentence[i] != '\0'; i++) {
		checksum ^= sentence[i];
	}
	return checksum;
}

/* ---------------------------------------- */

static bool validate_nmea_sentence(const char *sentence) {
	if (sentence == NULL) {
		ESP_LOGE(TAG, "NMEA Error: Sentence is NULL");
		return false;
	}

	int len = strlen(sentence);
	if (len < 6) {
		ESP_LOGE(TAG, "NMEA Error: Sentence has invalid length (< 6): %d", len);
		return false;
	}

	char *checksum_pos = strchr(sentence, '*');
	if (checksum_pos == NULL) {
		ESP_LOGE(TAG, "NMEA Error: No checksum delimiter found");
		return false;
	}

	// Parse checksum
	uint8_t received_checksum;
	if (sscanf(checksum_pos + 1, "%02hhx", &received_checksum) != 1) {
		ESP_LOGE(TAG, "NMEA Error: Checksum read failed");
		return false;
	}

	// Calculate checksum
	uint8_t calculated_checksum = nmea_checksum(sentence);
	if (received_checksum != calculated_checksum) {
		ESP_LOGE(TAG, "NMEA Error: Checksum failed. Expected: %d, but got: %d", calculated_checksum, received_checksum);
		return false;
	}
	return true;
}

/* ---------------------------------------- */

static void process_nmea_sentence(const char *sentence) {
	if (!validate_nmea_sentence(sentence)) {
		ESP_LOGE(TAG, "Invalid NMEA sentence: %s", sentence);
		return;
	}

	ESP_LOGD(TAG, "NMEA: %s", sentence);

	// Parse based on sentence type
	if (strncmp(sentence, NMEA_GPGGA, 6) == 0) {
	}
}

/* ---------------------------------------- */

static void gps_task(void *arg) {
	(void)arg;
	uint8_t *data = malloc(GPS_BUF_SIZE);

	ESP_LOGD(TAG, "Starting GPS Task");
	ESP_LOGD(TAG, "Config: UART=%d, TX=%d, RX=%d, baud=%d", GPS_UART_NUM, ROCKETLOG_GPS_TX_GPIO, ROCKETLOG_GPS_RX_GPIO,
			 GPS_BAUD_RATE);

	while (1) {
		int len = uart_read_bytes(gps_uart, data, GPS_BUF_SIZE, pdMS_TO_TICKS(100));

		if (len <= 0) {
			if (len == 0) {
				ESP_LOGE(TAG, "No data received.");
				continue;
			}

			ESP_LOGE(TAG, "GPS Read error: %s", len);
			continue;
		}

		ESP_LOGD(TAG, "GPS Raw: %.*s", len, data);
		for (int i = 0; i < len; i++) {
			char c = data[i];

			if (c == '$') {
				// Start of new sentence
				nmea_index = 0;
				nmea_buffer[nmea_index++] = c;
			} else if (c == '\r' || c == '\n') {
				// End of sentence
				if (nmea_index > 0) {
					nmea_buffer[nmea_index] = '\0';
					process_nmea_sentence(nmea_buffer);
					nmea_index = 0;
				}
			} else if (nmea_index < NMEA_MAX_SENTENCE - 1) {
				// Add character to current sentence
				nmea_buffer[nmea_index++] = c;
			}
		}
	}
}

/* ---------------------------------------- */

/**
 * Initialize the GPS module
 */
esp_err_t gps_init(void) {
	if (gps_initialized) {
		return ESP_OK;
	}

	ESP_LOGI(TAG, "Initializing GPS");

	uart_config_t uart_cfg = {
		.baud_rate = GPS_BAUD_RATE,
		.data_bits = UART_DATA_8_BITS,
		.parity = UART_PARITY_DISABLE,
		.stop_bits = UART_STOP_BITS_1,
		.flow_ctrl = UART_HW_FLOWCTRL_DISABLE,
		.source_clk = UART_SCLK_DEFAULT,
	};

	ESP_LOGD(TAG, "Config: baud_rate=%d, data_bits=%d, parity=%d, stop_bits=%d, flow_ctrl=%d, source_clk=%d",
			 uart_cfg.baud_rate, uart_cfg.data_bits, uart_cfg.flow_ctrl, uart_cfg.parity, uart_cfg.stop_bits,
			 uart_cfg.source_clk);

	ESP_LOGD(TAG, "Installing UART driver");
	esp_err_t err = uart_driver_install(gps_uart, GPS_BUF_SIZE, 0, 0, NULL, 0);
	if (err != ESP_OK) {
		ESP_LOGE(TAG, "Failed to install UART driver: %s (0x%x)", esp_err_to_name(err), err);
		return err;
	}

	ESP_LOGD(TAG, "Setting UART parameters");
	err = uart_param_config(gps_uart, &uart_cfg);
	if (err != ESP_OK) {
		ESP_LOGE(TAG, "Failed to set UART parameters: %s (0x%x)", esp_err_to_name(err), err);

		ESP_LOGD(TAG, "Deleting UART driver");
		esp_err_t delete_err = uart_driver_delete(gps_uart);
		if (delete_err != ESP_OK) {
			ESP_LOGE(TAG, "Failed to delete UART driver: %s (0x%x)", esp_err_to_name(delete_err), delete_err);
		}
		return err;
	}

	ESP_LOGD(TAG, "Setting UART pins (TX=%d, RX=%d)", ROCKETLOG_GPS_TX_GPIO, ROCKETLOG_GPS_RX_GPIO);
	err = uart_set_pin(gps_uart, ROCKETLOG_GPS_TX_GPIO, ROCKETLOG_GPS_RX_GPIO, UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE);
	if (err != ESP_OK) {
		ESP_LOGE(TAG, "Failed to set UART pins: %s (0x%x)", esp_err_to_name(err), err);

		ESP_LOGD(TAG, "Deleting UART driver");
		esp_err_t delete_err = uart_driver_delete(gps_uart);
		if (delete_err != ESP_OK) {
			ESP_LOGE(TAG, "Failed to delete UART driver: %s (0x%x)", esp_err_to_name(delete_err), delete_err);
		}
		return err;
	}

	ESP_LOGD(TAG, "Creating GPS Task");

	BaseType_t task_err = xTaskCreate(gps_task, "gps_task", GPS_TASK_STACK_SIZE, NULL, GPS_TASK_PRIORITY, NULL);
	if (task_err != pdPASS) {
		ESP_LOGE(TAG, "Failed to create GPS Task");

		ESP_LOGD(TAG, "Deleting UART driver");
		err = uart_driver_delete(gps_uart);
		if (err != ESP_OK) {
			ESP_LOGE(TAG, "Failed to delete UART driver: %s (0x%x)", esp_err_to_name(err), err);
		}
		// TODO: find out why this shoudl be NO_MEM
		return ESP_ERR_NO_MEM;
	}

	gps_initialized = true;
	ESP_LOGI(TAG, "GPS initialized successfully");

	return ESP_OK;
}
