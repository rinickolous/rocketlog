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
#define NMEA_GNGGA "$GNGGA"
#define NMEA_GNRMC "$GNRMC"
#define NMEA_GNTXT "$GNTXT"

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
static void parse_gga_sentence(const char *sentence, gps_data_t *data) {
	ESP_LOGD(TAG, "Parsing GGA sentence");

	// Structure: $GNGGA,time,lat,NS,lon,EW,fix,satellites,hdop,altitude,M,height,M,,*checksum
	char lat_str[11], lon_str[11], ns_char, ew_char;
	int fix_quality, satellites;
	float hdop, altitude;

	if (sscanf(sentence, "$GNGGA,%*[^,],%10[^,],%c,%10[^,],%c,%d,%d,%f,%f", lat_str, &ns_char, lon_str, &ew_char,
			   &fix_quality, &satellites, &hdop, &altitude) >= 7) {

		// Parse latitude (DDMM.MMMMM -> decimal degrees)
		double lat_dd = atof(lat_str) / 100.0;
		int lat_deg = (int)lat_dd;
		double lat_min = (lat_dd - lat_deg) * 100.0;
		data->latitude = lat_deg + lat_min / 60.0;
		if (ns_char == 'S') {
			data->latitude = -data->latitude;
		}

		// Parse longitude
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

		ESP_LOGI(TAG, "GGA Parsed: lat=%.6f, lon=%.6f, alt=%.1f, sats=%d, fix=%d", data->latitude, data->longitude,
				 data->altitude, data->satellites, data->fix_quality);
	}
}

/* ---------------------------------------- */

static void parse_rmc_sentence(const char *sentence, gps_data_t *data) {
	ESP_LOGD(TAG, "Parsing RMC sentence");

	// Structure: $GNRMC,time,status,lat,NS,lon,EW,speed,track,date,magvar,magvar_dir,mode*checksum
	char lat_str[11], lon_str[11], status_char, ns_char, ew_char;
	float speed_knots, track_degrees;

	if (sscanf(sentence, "$GNRMC,%*[^,],%c,%10[^,],%c,%10[^,],%c,%f,%f", &status_char, lat_str, &ns_char, lon_str,
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

		ESP_LOGI(TAG, "RMC Parsed: lat=%.6f, lon=%.6f, valid=%s", data->latitude, data->longitude,
				 data->valid ? "yes" : "no");
	}
}

/* ---------------------------------------- */

static void parse_txt_sentence(const char *sentence, gps_data_t *data) {
	ESP_LOGD(TAG, "Parsing TXT sentence");

	// Structure: $GNTXT,total,number,code,text*checksum
	int total, number, code;
	char text[61]; // Max 60 chars + null terminator

	if (sscanf(sentence, "$GNTXT,%d,%d,%d,%60[^*]", &total, &number, &code, text) == 4) {
		ESP_LOGI(TAG, "TXT Parsed: code=%d, text=%s", code, text);
		if (strncmp(text, "ANT_OPEN", 7) == 0) {
			ESP_LOGW(TAG, "GPS Warning: Antenna open circuit detected");
		}
	}
}

/* ---------------------------------------- */

static void process_nmea_sentence(const char *sentence) {
	if (!validate_nmea_sentence(sentence)) {
		ESP_LOGE(TAG, "Invalid NMEA sentence: %s", sentence);
		return;
	}

	ESP_LOGD(TAG, "NMEA: %s", sentence);

	// Parse based on sentence type
	if (strncmp(sentence, NMEA_GNGGA, 6) == 0) {
		parse_gga_sentence(sentence, &last_gps_data);
	} else if (strncmp(sentence, NMEA_GNRMC, 6) == 0) {
		parse_rmc_sentence(sentence, &last_gps_data);
	} else if (strncmp(sentence, NMEA_GNTXT, 6) == 0) {
		parse_txt_sentence(sentence, &last_gps_data);
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
		int len = uart_read_bytes(gps_uart, data, GPS_BUF_SIZE, pdMS_TO_TICKS(500));

		if (len <= 0) {
			if (len == 0) {
				ESP_LOGE(TAG, "No data received.");
				continue;
			}

			ESP_LOGE(TAG, "GPS Read error: %s", len);
			continue;
		}

		/* ESP_LOGD(TAG, "GPS Raw: %.*s", len, data); */
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

	// TODO: figure out why this is after while and what that does.
	free(data);
	vTaskDelete(NULL);
}

/* ---------------------------------------- */

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
