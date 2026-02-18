#include "sensor_gps.h"
#include "rocketlog_board.h"

#include "driver/uart.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "hal/uart_types.h"
#include <string.h>

/* ---------------------------------------- */
/*  Config Knobs                            */
/* ---------------------------------------- */

/* TAG for logging */
static const char *TAG = "sensor_gps";

/* UART configuration */
#define GPS_UART_NUM UART_NUM_1
#define GPS_BAUD_RATE 9600
#define GPS_BUF_SIZE 1024
#define NMEA_MAX_SENTENCE 82
#define GPS_TASK_STACK_SIZE 4096
#define GPS_TASK_PRIORITY 5

/* NMEA sentence identifiers */
#define NMEA_GPGGA "$GPGGA"
#define NMEA_GPRMC "$GPRMC"
#define NMEA_GPTXT "$GPTXT"
#define NMEA_GPGSA "$GPGSA"
#define NMEA_GPGSV "$GPGSV"

/* Static variables */
static uart_port_t gps_uart = GPS_UART_NUM; /* The UART port on the ESP32_S3 connected to the GPS module */
static bool gps_initialized = false;		/* State variable to track if GPS has alrady been initialized */
static gps_data_t last_gps_data = {0};		/* Latest parsed GPS data, updated by the GPS task and read by gps_read() */
static char nmea_buffer[NMEA_MAX_SENTENCE]; /* Buffer to accumulate incoming NMEA sentence characters until a full
											   sentence is received */
static int nmea_index = 0; /* Current index in the NMEA buffer, used to track how many characters have been received for
							  the current sentence */

/* ---------------------------------------- */

/**
 * Calculate the NMEA checksum for a given sentence (excluding the starting '$' and the '*' delimiter).
 * The checksum is computed by XORing all characters between '$' and '*'.
 *
 * @param sentence The NMEA sentence to calculate the checksum for.
 * @return The calculated checksum as an 8-bit unsigned integer.
 */
static uint8_t nmea_checksum(const char *sentence) {
	uint8_t checksum = 0;
	// Start after $ and before *
	for (int i = 1; sentence[i] != '*' && sentence[i] != '\0'; i++) {
		checksum ^= sentence[i];
	}
	return checksum;
}

/* ---------------------------------------- */

/**
 * Validate an NMEA sentence by checking its structure and verifying the checksum.
 * The function checks for:
 * - Non-null sentence pointer
 * - Minimum length (at least 6 characters to accommodate "$*XX")
 * - Presence of '*' character to separate data from checksum
 * - Correct checksum value matching the calculated checksum
 *
 * @param sentence The NMEA sentence string to validate
 * @return true if the sentence is valid, false otherwise
 */
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

/**
 * Parse a GGA NMEA sentence to extract GPS data such as latitude, longitude, altitude, fix quality, number of
 * satellites, and HDOP. The function uses sscanf to extract the relevant fields from the sentence and then converts the
 * latitude and longitude from the DDMM.MMMMM format to decimal degrees. It also updates the gps_data_t structure with
 * the parsed values and marks it as valid if a fix is present (fix_quality >= 1).
 *
 * @param sentence The GGA NMEA sentence string to parse
 * @param data Pointer to a gps_data_t structure where the parsed data will be stored
 */
static void parse_gga_sentence(const char *sentence, gps_data_t *data) {
	ESP_LOGD(TAG, "Parsing GGA sentence");

	// Structure: $GPGGA,time,lat,NS,lon,EW,fix,satellites,hdop,altitude,M,height,M,,*checksum
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

		// Parse longitude
		double lon_dd = atof(lon_str) / 100.0;
		int lon_deg = (int)lon_dd;
		double lon_min = (lon_dd - lon_deg) * 100.0;
		data->longitude = lon_deg + lon_min / 60.0;
		if (ew_char == 'W') {
			data->longitude = -data->longitude;
		}

		data->fix_quality = (uint8_t)fix_quality;
		data->num_satellites = (uint8_t)satellites;
		data->hdop = hdop;
		data->altitude_m = altitude;

		// Mark as valid if we have at least a 2D fix (fix_quality >= 1)
		data->valid = (fix_quality >= 1);

		ESP_LOGD(TAG, "GGA Parsed: lat=%.6f, lon=%.6f, alt=%.1f, sats=%d, fix=%d", data->latitude, data->longitude,
				 data->altitude_m, data->num_satellites, data->fix_quality);
	}
}

/* ---------------------------------------- */

/**
 * Parse an RMC NMEA sentence to extract GPS data such as latitude, longitude, speed, and track. The function uses
 * sscanf to extract the relevant fields from the sentence and then converts the latitude and longitude from the
 * DDMM.MMMMM format to decimal degrees. It also updates the gps_data_t structure with the parsed values and marks it as
 * valid if the status character indicates an active fix ('A').
 *
 * @param sentence The RMC NMEA sentence string to parse
 * @param data Pointer to a gps_data_t structure where the parsed data will be stored
 */
static void parse_rmc_sentence(const char *sentence, gps_data_t *data) {
	ESP_LOGD(TAG, "Parsing RMC sentence");

	// Structure: $GPRMC,time,status,lat,NS,lon,EW,speed,track,date,magvar,magvar_dir,mode*checksum
	char time_str[16]; // "hhmmss.sss"
	char status;	   // 'A' or 'V'

	ESP_LOGD(TAG, "RMC Sentence: %s", sentence);

	if (sscanf(sentence, "$%*2cRMC,%15[^,],%c", time_str, &status) == 2) {
		if (status == 'A') {
			ESP_LOGD(TAG, "RMC status is active, marking GPS data as valid");
			data->valid = true;

			char lat_str[16], lon_str[16]; // "llll.ll", "yyyyy.yy"
			char ns, ew;				   // Either 'N' or 'S' for latitude, and 'E' or 'W' for longitude
			float speed_knots, course_deg;
			char date_str[8]; // "ddmmyy"

			int n = sscanf(sentence, "$%*2cRMC,%15[^,],%c,%15[^,],%c,%15[^,],%c,%f,%f,%6[^,]", time_str, &status,
						   lat_str, &ns, lon_str, &ew, &speed_knots, &course_deg, date_str);

			if (n == 9) {
				// Parse lat/lon (same as GGA)
				double lat_dd = atof(lat_str) / 100.0;
				int lat_deg = (int)lat_dd;
				double lat_min = (lat_dd - lat_deg) * 100.0;
				data->latitude = lat_deg + lat_min / 60.0;
				if (ns == 'S') {
					data->latitude = -data->latitude;
				}

				double lon_dd = atof(lon_str) / 100.0;
				int lon_deg = (int)lon_dd;
				double lon_min = (lon_dd - lon_deg) * 100.0;
				data->longitude = lon_deg + lon_min / 60.0;
				if (ew == 'W') {
					data->longitude = -data->longitude;
				}
			}
		} else {
			ESP_LOGD(TAG, "RMC status is void, marking GPS data as invalid");
			data->valid = false;
		}

		ESP_LOGD(TAG, "RMC Parsed: lat=%.6f, lon=%.6f, valid=%s", data->latitude, data->longitude,
				 data->valid ? "yes" : "no");
	}
}

/* ---------------------------------------- */

/**
 * Parse a GSA NMEA sentence to extract information about the GPS fix mode and the type of fix (2D/3D). The function
 * uses sscanf to extract the mode character and fix type integer from the sentence. It then logs the parsed values for
 * debugging purposes.
 *
 * @param sentence The GSA NMEA sentence string to parse
 * @param data Pointer to a gps_data_t structure (not used in this function but included for consistency with other
 * parsers)
 */
static void parse_gsa_sentence(const char *sentence, gps_data_t *data) {
	ESP_LOGD(TAG, "Parsing GSA sentence");

	// Structure: $GPGSA,mode,fix_type,sat1,...,sat12,pdop,hdop,vdop*checksum
	char mode;
	int fix_type;

	if (sscanf(sentence, "$GPGSA,%c,%d", &mode, &fix_type) == 2) {
		ESP_LOGD(TAG, "GSA Parsed: mode=%c, fix_type=%d", mode, fix_type);
	}
}

/* ---------------------------------------- */

/**
 * Parse a GSV NMEA sentence to extract information about the satellites in view. The function uses sscanf to extract
 * the total number of messages, message number, and details about each satellite (ID, elevation, azimuth, SNR). It then
 * logs the parsed values for debugging purposes.
 *
 * @param sentence The GSV NMEA sentence string to parse
 * @param data Pointer to a gps_data_t structure (not used in this function but included for consistency with other
 * parsers)
 */
static void parse_gsv_sentence(const char *sentence, gps_data_t *data) {
	ESP_LOGD(TAG, "Parsing GSV sentence");

	// Structure: $GPGSV,total_msgs,msg_num,sat1_id,sat1_elev,sat1_azim,sat1_snr,...*checksum
	int total_msgs, msg_num;
	if (sscanf(sentence, "$GPGSV,%d,%d", &total_msgs, &msg_num) == 2) {
		ESP_LOGD(TAG, "GSV Parsed: total_msgs=%d, msg_num=%d", total_msgs, msg_num);
	}
}

/* ---------------------------------------- */

/**
 * Parse a TXT NMEA sentence to extract any warning or status messages from the GPS module. The function uses sscanf
 * to extract the total number of messages, message number, code, and text from the sentence. It then checks if the
 * text contains specific warnings (e.g., "ANT_OPEN") and logs them accordingly.
 *
 * @param sentence The TXT NMEA sentence string to parse
 * @param data Pointer to a gps_data_t structure (not used in this function but included for consistency with other
 * parsers)
 */
static void parse_txt_sentence(const char *sentence, gps_data_t *data) {
	ESP_LOGD(TAG, "Parsing TXT sentence");

	// Structure: $GPTXT,total,number,code,text*checksum
	int total, number, code;
	char text[61]; // Max 60 chars + null terminator

	if (sscanf(sentence, "$GPTXT,%d,%d,%d,%60[^*]", &total, &number, &code, text) == 4) {
		ESP_LOGD(TAG, "TXT Parsed: code=%d, text=%s", code, text);
		if (strncmp(text, "ANT_OPEN", 7) == 0) {
			ESP_LOGW(TAG, "GPS Warning: Antenna open circuit detected");
		}
	}
}

/* ---------------------------------------- */

/**
 * Process a complete NMEA sentence by validating it and then parsing it based on its type (GNGGA, GNRMC, GNTXT).
 */
static void process_nmea_sentence(const char *sentence) {
	if (!validate_nmea_sentence(sentence)) {
		ESP_LOGE(TAG, "Invalid NMEA sentence: %s", sentence);
		return;
	}

	// Parse based on sentence type
	if (strncmp(sentence, NMEA_GPGGA, strlen(NMEA_GPGGA)) == 0) {
		parse_gga_sentence(sentence, &last_gps_data);
	} else if (strncmp(sentence, NMEA_GPRMC, strlen(NMEA_GPRMC)) == 0) {
		parse_rmc_sentence(sentence, &last_gps_data);
	} else if (strncmp(sentence, NMEA_GPTXT, strlen(NMEA_GPTXT)) == 0) {
		parse_txt_sentence(sentence, &last_gps_data);
	} else if (strncmp(sentence, NMEA_GPGSA, strlen(NMEA_GPTXT)) == 0) {
		parse_gsa_sentence(sentence, &last_gps_data);
	} else if (strncmp(sentence, NMEA_GPGSV, strlen(NMEA_GPTXT)) == 0) {
		parse_gsv_sentence(sentence, &last_gps_data);
	} else {
		ESP_LOGW(TAG, "Unknown NMEA sentence: %s", sentence);
	}
}

/* ---------------------------------------- */

/**
 * Read the latest GPS data. This function copies the most recently parsed GPS data from the last_gps_data structure
 * into the provided gps_data_t structure. It first checks if the provided pointer is valid and then performs a
 * memory copy to transfer the data. The function returns ESP_OK on success or an appropriate error code if the
 * argument is invalid.
 *
 * @param data Pointer to a gps_data_t structure where the latest GPS data will be stored
 * @return ESP_OK on success, or ESP_ERR_INVALID_ARG if the provided pointer is NULL
 */
esp_err_t gps_read(gps_data_t *data) {
	if (data == NULL) {
		return ESP_ERR_INVALID_ARG;
	}

	// Copy the latest GPS data to the provided structure
	memcpy(data, &last_gps_data, sizeof(gps_data_t));
	return ESP_OK;
}

/* ---------------------------------------- */

/**
 * Continuously read raw NMEA sentences from the GPS module over UART, validate and parse them, and update the latest
 * GPS data structure. The task reads data in chunks from the UART buffer, processes it character by character to
 * accumulate and identify complete NMEA sentences, and then calls the appropriate parsing functions based on the
 * sentence type. The task includes error handling for UART read operations and logs any issues encountered during
 * reading or parsing. It runs indefinitely, with a short delay between read attempts to allow for new data to arrive
 * from the GPS module.
 */
static void gps_task(void *arg) {
	(void)arg;							  /* Unused parameter */
	uint8_t *data = malloc(GPS_BUF_SIZE); /* Buffer for raw UART data */

	ESP_LOGI(TAG, "Starting GPS Task");
	ESP_LOGD(TAG, "Config: UART=%d, TX=%d, RX=%d, baud=%d", GPS_UART_NUM, ROCKETLOG_GPS_TX_GPIO, ROCKETLOG_GPS_RX_GPIO,
			 GPS_BAUD_RATE);

	while (1) {
		// Read raw data from UART with a timeout of 500ms. This will return the number of bytes read, or 0 if no data
		int len = uart_read_bytes(gps_uart, data, GPS_BUF_SIZE, pdMS_TO_TICKS(500));

		// If len is 0, it means no data was received within the timeout.
		// If len is negative, it indicates an error. In either case,
		// we log the issue and continue to the next iteration to try reading again.
		if (len <= 0) {
			if (len == 0) {
				ESP_LOGE(TAG, "No data received.");
				continue;
			}

			ESP_LOGE(TAG, "GPS Read error: %s", len);
			continue;
		}

		// Process the received data character by character to accumulate NMEA sentences.
		// We look for the start of a sentence ('$'),
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

		// The GPS module typically sends data at 1Hz, so we can delay for a short time before trying to read again.
		vTaskDelay(pdMS_TO_TICKS(500));
	}

	free(data);
	vTaskDelete(NULL);
}

/* ---------------------------------------- */

/**
 * Initialize GPS module by configuring UART and starting GPS task.
 */
esp_err_t gps_init(void) {
	if (gps_initialized) {
		return ESP_OK;
	}

	ESP_LOGI(TAG, "Initializing GPS");

	// Set UART parameters for GPS
	uart_config_t uart_cfg = {
		.baud_rate = GPS_BAUD_RATE,
		.data_bits = UART_DATA_8_BITS,
		.parity = UART_PARITY_DISABLE,
		.stop_bits = UART_STOP_BITS_1,
		.flow_ctrl = UART_HW_FLOWCTRL_DISABLE,
		.source_clk = UART_SCLK_DEFAULT,
	};
	ESP_LOGD(TAG, "UART Config: baud_rate=%d, data_bits=%d, parity=%d, stop_bits=%d, flow_ctrl=%d, source_clk=%d",
			 uart_cfg.baud_rate, uart_cfg.data_bits, uart_cfg.parity, uart_cfg.stop_bits, uart_cfg.flow_ctrl,
			 uart_cfg.source_clk);

	// Install UART driver for GPS
	ESP_LOGD(TAG, "Installing UART driver for GPS on UART%d", gps_uart);
	esp_err_t err = uart_driver_install(gps_uart, GPS_BUF_SIZE, 0, 0, NULL, 0);
	if (err != ESP_OK) {
		ESP_LOGE(TAG, "Failed to install UART driver: %s", esp_err_to_name(err));
		return err;
	}

	// Configure UART parameters
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

	// Set UART pins for GPS (TX and RX)
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

	ESP_LOGI(TAG, "UART driver installed and configured successfully");

	// Create the GPS task to continuously read from the UART and process NMEA sentences. The task is pinned to the
	// same core as the main application to ensure it runs smoothly without contention. We also check if the task
	// creation failed and handle it by logging the error and cleaning up the UART driver before returning.
	ESP_LOGD(TAG, "Creating GPS task to read and parse NMEA sentences");
	BaseType_t task_err = xTaskCreate(gps_task, "gps_task", GPS_TASK_STACK_SIZE, NULL, GPS_TASK_PRIORITY, NULL);
	if (task_err != pdPASS) {
		ESP_LOGE(TAG, "Failed to create GPS task: %d", task_err);

		ESP_LOGD(TAG, "Deleting UART driver");
		esp_err_t delete_err = uart_driver_delete(gps_uart);
		if (delete_err != ESP_OK) {
			ESP_LOGE(TAG, "Failed to delete UART driver: %s (0x%x)", esp_err_to_name(delete_err), delete_err);
		}
		return ESP_FAIL;
	}

	gps_initialized = true;
	return ESP_OK;
}
