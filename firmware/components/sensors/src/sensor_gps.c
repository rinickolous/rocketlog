#include "sensor_gps.h"
#include "rocketlog_board.h"

#include <string.h>
#include <stdlib.h>
#include "esp_log.h"
#include "driver/uart.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

static const char *TAG = "sensor_gps";

#define GPS_UART_NUM UART_NUM_1
#define GPS_BAUD_RATE 9600
#define GPS_BUF_SIZE 1024
#define NMEA_MAX_SENTENCE 82
#define GPS_TASK_STACK_SIZE 4096
#define GPS_TASK_PRIORITY 5

// NMEA sentence identifiers
#define NMEA_GPGGA "$GPGGA"
#define NMEA_GPRMC "$GPRMC"

// Static variables
static uart_port_t gps_uart = GPS_UART_NUM;
static bool gps_initialized = false;
static gps_data_t last_gps_data = {0};
static char nmea_buffer[NMEA_MAX_SENTENCE];
static int nmea_index = 0;

// GPS data parsing task
static void gps_task(void *arg);

// Helper functions
static uint8_t nmea_checksum(const char *sentence) {
	uint8_t checksum = 0;
	// Start after '$' and end before '*'
	for (int i = 1; sentence[i] != '*' && sentence[i] != '\0'; i++) {
		checksum ^= sentence[i];
	}
	return checksum;
}

static bool validate_nmea_sentence(const char *sentence) {
	if (sentence == NULL || strlen(sentence) < 6) {
		return false;
	}

	// Find checksum delimiter
	char *checksum_pos = strchr(sentence, '*');
	if (checksum_pos == NULL) {
		return false;
	}

	// Parse checksum from sentence
	uint8_t received_checksum;
	if (sscanf(checksum_pos + 1, "%02hhx", &received_checksum) != 1) {
		return false;
	}

	// Calculate and compare checksum
	uint8_t calculated_checksum = nmea_checksum(sentence);
	return (received_checksum == calculated_checksum);
}

static void parse_gga_sentence(const char *sentence, gps_data_t *data) {
	// $GPGGA,time,lat,NS,lon,EW,fix,satellites,hdop,altitude,M,height,M,,*checksum
	char lat_str[11], lon_str[11], ns_char, ew_char;
	int fix_quality, satellites;
	float hdop, altitude;

	if (sscanf(sentence, "$GPGGA,%*[^,],%10[^,],%c,%10[^,],%c,%d,%d,%f,%f", lat_str, &ns_char, lon_str, &ew_char,
			   &fix_quality, &satellites, &hdop, &altitude) >= 7) {

		// Parse latitude (DDMM.MMMMM -> decimal degrees)
		double lat_dd = atof(lat_str) / 100.0;
		int lat_deg = (int)lat_dd;
		double lat_min = (lat_dd - lat_deg) * 100.0;
		data->latitude = lat_deg + lat_min / 60.0;
		if (ns_char == 'S') {
			data->latitude = -data->latitude;
		}

		// Parse longitude (DDDMM.MMMMM -> decimal degrees)
		double lon_dd = atof(lon_str) / 100.0;
		int lon_deg = (int)lon_dd;
		double lon_min = (lon_dd - lon_deg) * 100.0;
		data->longitude = lon_deg + lon_min / 60.0;
		if (ew_char == 'W') {
			data->longitude = -data->longitude;
		}

		data->fix_quality = (uint8_t)fix_quality;
		data->satellites = (uint8_t)satellites;
		data->hdop = hdop;
		data->altitude = altitude;

		// Mark as valid if we have at least a 2D fix (fix_quality >= 1)
		data->valid = (fix_quality >= 1);
	}
}

static void parse_rmc_sentence(const char *sentence, gps_data_t *data) {
	// $GPRMC,time,status,lat,NS,lon,EW,速度,方向,日期,磁偏,磁偏方向,模式*校验和
	char lat_str[11], lon_str[11], status_char, ns_char, ew_char;
	float speed_knots, track_degrees;

	if (sscanf(sentence, "$GPRMC,%*[^,],%c,%10[^,],%c,%10[^,],%c,%f,%f", &status_char, lat_str, &ns_char, lon_str,
			   &ew_char, &speed_knots, &track_degrees) >= 4) {

		// Update valid status based on RMC status ('A' = active, 'V' = void)
		if (status_char == 'A') {
			// Parse lat/lon (same as GGA)
			double lat_dd = atof(lat_str) / 100.0;
			int lat_deg = (int)lat_dd;
			double lat_min = (lat_dd - lat_deg) * 100.0;
			data->latitude = lat_deg + lat_min / 60.0;
			if (ns_char == 'S') {
				data->latitude = -data->latitude;
			}

			double lon_dd = atof(lon_str) / 100.0;
			int lon_deg = (int)lon_dd;
			double lon_min = (lon_dd - lon_deg) * 100.0;
			data->longitude = lon_deg + lon_min / 60.0;
			if (ew_char == 'W') {
				data->longitude = -data->longitude;
			}
		} else {
			data->valid = false;
		}
	}
}

static void process_nmea_sentence(const char *sentence) {
	if (!validate_nmea_sentence(sentence)) {
		ESP_LOGD(TAG, "Invalid NMEA sentence: %s", sentence);
		return;
	}

	ESP_LOGD(TAG, "NMEA: %s", sentence);

	// Parse based on sentence type
	if (strncmp(sentence, NMEA_GPGGA, 6) == 0) {
		parse_gga_sentence(sentence, &last_gps_data);
	} else if (strncmp(sentence, NMEA_GPRMC, 6) == 0) {
		parse_rmc_sentence(sentence, &last_gps_data);
	}
}

static void gps_task(void *arg) {
	(void)arg;
	uint8_t *data = malloc(GPS_BUF_SIZE);

	ESP_LOGD(TAG, "Starting GPS Task");

	while (1) {
		/* ESP_LOGD(TAG, "Attempting UART read"); */
		int len = uart_read_bytes(gps_uart, data, GPS_BUF_SIZE, pdMS_TO_TICKS(100));
		if (len > 0) {
			ESP_LOGW(TAG, "successfully read UART!");
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

	free(data);
	vTaskDelete(NULL);
}

// Public API implementation
esp_err_t gps_init(void) {
	if (gps_initialized) {
		return ESP_OK;
	}

	uart_config_t uart_cfg = {
		.baud_rate = GPS_BAUD_RATE,
		.data_bits = UART_DATA_8_BITS,
		.parity = UART_PARITY_DISABLE,
		.stop_bits = UART_STOP_BITS_1,
		.flow_ctrl = UART_HW_FLOWCTRL_DISABLE,
		.source_clk = UART_SCLK_DEFAULT,
	};

	esp_err_t err = uart_driver_install(gps_uart, GPS_BUF_SIZE, 0, 0, NULL, 0);
	if (err != ESP_OK) {
		ESP_LOGE(TAG, "Failed to install UART driver: %s", esp_err_to_name(err));
		return err;
	}

	err = uart_param_config(gps_uart, &uart_cfg);
	if (err != ESP_OK) {
		ESP_LOGE(TAG, "Failed to configure UART: %s", esp_err_to_name(err));
		uart_driver_delete(gps_uart);
		return err;
	}

	err = uart_set_pin(gps_uart, ROCKETLOG_GPS_TX_GPIO, ROCKETLOG_GPS_RX_GPIO, UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE);
	if (err != ESP_OK) {
		ESP_LOGE(TAG, "Failed to set UART pins: %s", esp_err_to_name(err));
		uart_driver_delete(gps_uart);
		return err;
	}

	// Create GPS processing task
	BaseType_t task_err = xTaskCreate(gps_task, "gps_task", GPS_TASK_STACK_SIZE, NULL, GPS_TASK_PRIORITY, NULL);
	if (task_err != pdPASS) {
		ESP_LOGE(TAG, "Failed to create GPS task");
		uart_driver_delete(gps_uart);
		return ESP_ERR_NO_MEM;
	}

	gps_initialized = true;
	ESP_LOGI(TAG, "GPS initialized successfully (UART %d, pins TX=%d, RX=%d, baud=%d)", gps_uart, ROCKETLOG_GPS_TX_GPIO,
			 ROCKETLOG_GPS_RX_GPIO, GPS_BAUD_RATE);

	return ESP_OK;
}

esp_err_t gps_read(gps_data_t *data) {
	if (!gps_initialized || data == NULL) {
		return ESP_ERR_INVALID_STATE;
	}

	*data = last_gps_data;
	return ESP_OK;
}

bool gps_has_fix(void) {
	return gps_initialized && last_gps_data.valid;
}

esp_err_t gps_read_raw(char *buffer, size_t buffer_size, size_t *bytes_read) {
	if (!gps_initialized || buffer == NULL || bytes_read == NULL) {
		return ESP_ERR_INVALID_STATE;
	}

	int len = uart_read_bytes(gps_uart, (uint8_t *)buffer, buffer_size, pdMS_TO_TICKS(100));
	if (len < 0) {
		return ESP_FAIL;
	}

	*bytes_read = (size_t)len;
	return ESP_OK;
}
