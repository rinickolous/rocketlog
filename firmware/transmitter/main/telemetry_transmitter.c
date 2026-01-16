#include <math.h>
#include <stdint.h>
#include <stdio.h>

#include <stdbool.h>
#include <string.h>

#include "rocketlog_board.h"
#include "rocketlog_time.h"
#include "telemetry_packet.h"
#include "telemetry_protocol.h"

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#include "driver/i2c_master.h"
#include "driver/rmt_tx.h"

#include "esp_err.h"
#include "esp_log.h"
#include "esp_timer.h"
#include "led_strip.h"
#include "led_strip_rmt.h"

#include "driver/usb_serial_jtag.h"

/* ---------------------------------------- */
/*  Config Knobs                            */
/* ---------------------------------------- */

/* Time keeping moved to components/common (rocketlog_time.*) */

static const TickType_t SAMPLE_PERIOD = pdMS_TO_TICKS(100);

#define RGB_LED_GPIO ROCKETLOG_RGB_LED_GPIO

static led_strip_handle_t led_strip;

// I2C

#define I2C_PORT I2C_NUM_0

static i2c_master_bus_handle_t i2c_bus;
static i2c_master_dev_handle_t mpl_dev;
#define I2C_SCL 11
#define I2C_SDA 12
#define I2C_FREQ 40000

// MPL3115A2 barometric pressure sensor (altimeter mode)
// Datasheet: registers 0x00.. etc. We keep this minimal: init + read altitude (m) and temp (C).
#define MPL3115A2_ADDR 0x60
#define MPL3115A2_REG_STATUS 0x00
#define MPL3115A2_REG_OUT_P_MSB 0x01
#define MPL3115A2_REG_OUT_T_MSB 0x04
#define MPL3115A2_REG_WHO_AM_I 0x0C
#define MPL3115A2_REG_CTRL_REG1 0x26
#define MPL3115A2_REG_PT_DATA_CFG 0x13

#define MPL3115A2_WHO_AM_I_VAL 0xC4

/* ---------------------------------------- */
/*  I2C Functions                           */
/* ---------------------------------------- */

static void i2c_probe_addr(i2c_master_bus_handle_t bus, uint8_t addr) {
	// Using the new I2C driver in ESP-IDF 5.5 can enable an internal "asynchronous" transaction
	// path that is not compatible with i2c_master_probe() and can be easily flooded.
	// For debugging wiring, do a single conservative transaction against the address of interest.
	// Read WHO_AM_I multiple times to check signal integrity.
	const uint8_t who_reg = MPL3115A2_REG_WHO_AM_I;

	i2c_master_dev_handle_t tmp = NULL;
	i2c_device_config_t dev_cfg = {
		.dev_addr_length = I2C_ADDR_BIT_LEN_7,
		.device_address = addr,
		.scl_speed_hz = I2C_FREQ,
	};

	esp_err_t err = i2c_master_bus_add_device(bus, &dev_cfg, &tmp);
	if (err != ESP_OK) {
		ESP_LOGE("rocketlog", "I2C add_device 0x%02X failed: %s (0x%x)", addr, esp_err_to_name(err), (unsigned)err);
		return;
	}

	uint8_t values[16] = {0};
	int ok = 0;
	for (int i = 0; i < (int)sizeof(values); i++) {
		err = i2c_master_transmit_receive(tmp, &who_reg, 1, &values[i], 1, 100);
		if (err == ESP_OK) {
			ok++;
		}
		vTaskDelay(pdMS_TO_TICKS(50));
	}

	ESP_LOGI("rocketlog", "I2C probe 0x%02X WHO_AM_I: ok=%d/%d", addr, ok, (int)sizeof(values));
	ESP_LOGI("rocketlog",
			 "I2C WHO_AM_I samples: %02X %02X %02X %02X %02X %02X %02X %02X %02X %02X %02X %02X %02X %02X %02X %02X",
			 values[0], values[1], values[2], values[3], values[4], values[5], values[6], values[7], values[8],
			 values[9], values[10], values[11], values[12], values[13], values[14], values[15]);

	i2c_master_bus_rm_device(tmp);
}

/* ---------------------------------------- */

static esp_err_t i2c_init(void) {
	if (i2c_bus != NULL) {
		return ESP_OK;
	}

	i2c_master_bus_config_t bus_cfg = {
		.i2c_port = I2C_PORT,
		.sda_io_num = I2C_SDA,
		.scl_io_num = I2C_SCL,
		.clk_source = I2C_CLK_SRC_DEFAULT,
		.glitch_ignore_cnt = 7,
		.intr_priority = 0,
		.trans_queue_depth = 64,
		.flags.enable_internal_pullup = 1,
		.flags.allow_pd = 0,
	};

	i2c_device_config_t dev_cfg = {
		.dev_addr_length = I2C_ADDR_BIT_LEN_7,
		.device_address = MPL3115A2_ADDR,
		.scl_speed_hz = I2C_FREQ,
		.scl_wait_us = 0,
		.flags.disable_ack_check = 0,
	};

	esp_err_t err = i2c_new_master_bus(&bus_cfg, &i2c_bus);
	if (err != ESP_OK) {
		ESP_LOGE("rocketlog", "i2c_new_master_bus failed: %s", esp_err_to_name(err));
		return err;
	}
	i2c_probe_addr(i2c_bus, MPL3115A2_ADDR);

	err = i2c_master_bus_add_device(i2c_bus, &dev_cfg, &mpl_dev);
	return err;
}

/* ---------------------------------------- */

static bool serial_ready(void) {
	// Guard against early log/frame sends before driver install.
	return usb_serial_jtag_is_connected();
}

static void serial_send_frame(const uint8_t *data, size_t len) {
	if (!serial_ready()) {
		return;
	}

	// COBS-frame the packet and terminate with 0x00.
	uint8_t enc[512];
	const size_t enc_len = rocketlog_cobs_encode(enc, sizeof(enc), data, len);
	if (enc_len == 0) {
		return;
	}

	usb_serial_jtag_write_bytes(enc, enc_len, portMAX_DELAY);
	const uint8_t delim = 0;
	usb_serial_jtag_write_bytes(&delim, 1, portMAX_DELAY);
	usb_serial_jtag_wait_tx_done(portMAX_DELAY);
}

static void serial_send_log(rocketlog_log_level_t level, const char *msg) {
	uint8_t pkt[512];
	int64_t t_us = 0;
	if (rocketlog_time_is_set()) {
		t_us = (int64_t)llround(rocketlog_current_unix_time() * 1000000.0);
	}
	const int pkt_len = rocketlog_log_packet_encode_v1(pkt, sizeof(pkt), level, t_us, msg);
	if (pkt_len <= 0) {
		return;
	}
	serial_send_frame(pkt, (size_t)pkt_len);
}

static esp_err_t i2c_write_reg(uint8_t reg, uint8_t value) {
	if (mpl_dev == NULL) {
		return ESP_ERR_INVALID_STATE;
	}

	uint8_t buf[2] = {reg, value};
	return i2c_master_transmit(mpl_dev, buf, sizeof(buf), 50);
}

static esp_err_t i2c_read_regs(uint8_t start_reg, uint8_t *out, size_t out_len) {
	if (mpl_dev == NULL) {
		ESP_LOGE("rocketlog", "i2c_read_regs: mpl_dev is NULL");
		return ESP_ERR_INVALID_STATE;
	}
	if (out == NULL || out_len == 0) {
		ESP_LOGE("rocketlog", "i2c_read_regs: invalid output buffer");
		return ESP_ERR_INVALID_ARG;
	}

	return i2c_master_transmit_receive(mpl_dev, &start_reg, 1, out, out_len, 50);
}

static esp_err_t mpl3115a2_init(void) {
	uint8_t who = 0;
	esp_err_t err = i2c_read_regs(MPL3115A2_REG_WHO_AM_I, &who, 1);
	ESP_LOGI("rocketlog", "MPL3115A2 WHO_AM_I read: err=%s (0x%x) who=0x%02x", esp_err_to_name(err), (unsigned)err,
			 who);
	if (err != ESP_OK) {
		return err;
	}

	if (who != MPL3115A2_WHO_AM_I_VAL) {
		ESP_LOGE("rocketlog", "MPL3115A2 WHO_AM_I mismatch: expected=0x%02X got=0x%02X", MPL3115A2_WHO_AM_I_VAL, who);
		return ESP_ERR_INVALID_RESPONSE;
	}

	// Enable data flags for pressure/altitude + temperature.
	// PT_DATA_CFG: set DREM | PDEFE | TDEFE
	err = i2c_write_reg(MPL3115A2_REG_PT_DATA_CFG, 0x07);
	if (err != ESP_OK) {
		return err;
	}

	// CTRL_REG1: ALT=1 (altimeter), OS=128 (0b111), SBYB=1 (active)
	// 0xB9 = 1011 1001
	err = i2c_write_reg(MPL3115A2_REG_CTRL_REG1, 0xB9);
	return err;
}

static esp_err_t mpl3115a2_read(float *altitude_m_out, float *temp_c_out) {
	if (altitude_m_out == NULL || temp_c_out == NULL) {
		ESP_LOGE("rocketlog", "Invalid initial values provided");
		return ESP_ERR_INVALID_ARG;
	}

	uint8_t status = 0;
	esp_err_t err = i2c_read_regs(MPL3115A2_REG_STATUS, &status, 1);
	if (err != ESP_OK) {
		ESP_LOGE("rocketlog", "i2c_read_regs failed err=%s (0x%x)", esp_err_to_name(err), (unsigned)err);
		return err;
	}
	ESP_LOGI("rocketlog", "stauts after read: %d", status);

	// STATUS: PDR (0x04) and TDR (0x02). In altimeter mode, pressure registers hold altitude.
	// If not ready yet, kick a one-shot measurement and retry a few times.
	for (int attempt = 0; attempt < 10 && ((status & 0x06) != 0x06); attempt++) {
		// CTRL_REG1: set OST (one-shot trigger) while keeping current mode bits.
		uint8_t ctrl = 0;
		(void)i2c_read_regs(MPL3115A2_REG_CTRL_REG1, &ctrl, 1);
		(void)i2c_write_reg(MPL3115A2_REG_CTRL_REG1, (uint8_t)(ctrl | 0x02));
		vTaskDelay(pdMS_TO_TICKS(50));
		err = i2c_read_regs(MPL3115A2_REG_STATUS, &status, 1);
		if (err != ESP_OK) {
			return err;
		}
	}

	if ((status & 0x06) != 0x06) {
		ESP_LOGE("rocketlog", "Timeout line 256");
		return ESP_ERR_TIMEOUT;
	}

	uint8_t buf[5] = {0};
	err = i2c_read_regs(MPL3115A2_REG_OUT_P_MSB, buf, sizeof(buf));
	if (err != ESP_OK) {
		ESP_LOGE("rocketlog", "Error line 263");
		return err;
	}

	// Altitude: 20-bit two's complement, Q16.4 (meters).
	int32_t alt_raw = ((int32_t)buf[0] << 16) | ((int32_t)buf[1] << 8) | (int32_t)buf[2];
	alt_raw >>= 4;
	if (alt_raw & (1 << 19)) {
		alt_raw -= (1 << 20);
	}
	*altitude_m_out = (float)alt_raw / 16.0f;

	// Temperature: 12-bit two's complement, Q8.4 (degC).
	int16_t temp_raw = (int16_t)(((uint16_t)buf[3] << 8) | (uint16_t)buf[4]);
	temp_raw >>= 4;
	if (temp_raw & (1 << 11)) {
		temp_raw -= (1 << 12);
	}
	*temp_c_out = (float)temp_raw / 16.0f;

	return ESP_OK;
}

static void i2c_task(void *arg) {
	(void)arg;

	// Simple periodic health logging so we know I2C is wired.
	// Wait until I2C has been initialized by telemetry_task.
	while (mpl_dev == NULL) {
		vTaskDelay(pdMS_TO_TICKS(100));
	}
	while (1) {
		float alt_m = 0.0f;
		float temp_c = 0.0f;
		const esp_err_t err = mpl3115a2_read(&alt_m, &temp_c);

		if (err != ESP_OK) {
			ESP_LOGE("rocketlog", "MPL3115A2 read failed: %s (0x%x)", esp_err_to_name(err), (unsigned)err);
		}
		vTaskDelay(pdMS_TO_TICKS(3000));
	}
}

/* ---------------------------------------- */
/*  LED Functions                           */
/* ---------------------------------------- */

static void status_led_init(void) {
	led_strip_config_t strip_config = {
		.strip_gpio_num = RGB_LED_GPIO,
		.max_leds = 1,
		.led_model = LED_MODEL_SK6812,
		.flags.invert_out = false,
	};

	led_strip_rmt_config_t rmt_config = {
		.clk_src = RMT_CLK_SRC_DEFAULT,
		.resolution_hz = 10 * 1000 * 1000, // 10 MHz
		.mem_block_symbols = 64,
	};

	ESP_ERROR_CHECK(led_strip_new_rmt_device(&strip_config, &rmt_config, &led_strip));
	led_strip_clear(led_strip);
	led_strip_refresh(led_strip);
}

/* ---------------------------------------- */

static void status_led_update(void) {
	// Transmitter stub mode: visually differentiate from receiver.
	// Dim pulse blue.
	double t = rocketlog_monotonic_seconds();
	uint8_t brightness = (uint8_t)((sin(t * 2.0) + 1.0) / 2.0 * 16.0);
	led_strip_set_pixel(led_strip, 0, 0, 0, brightness);
	led_strip_refresh(led_strip);
}

/* ---------------------------------------- */

static void telemetry_task(void *arg) {
	(void)arg;

	// Keep ESP logging enabled so idf.py monitor is readable.
	// (Binary-framed output is disabled by default below.)

	// Install USB Serial/JTAG driver for host commands.
	usb_serial_jtag_driver_config_t cfg = {
		.rx_buffer_size = 1024,
		.tx_buffer_size = 1024,
	};
	ESP_ERROR_CHECK(usb_serial_jtag_driver_install(&cfg));

	// stdout is not used for framing; keep it quiet anyway.
	setvbuf(stdout, NULL, _IONBF, 0);

	ESP_LOGI("rocketlog", "Transmitter started");
	ESP_LOGI("rocketlog", "USB Serial/JTAG ready");
	const esp_err_t i2c_err = i2c_init();
	if (i2c_err != ESP_OK) {
		ESP_LOGE("rocketlog", "I2C init failed: %s (0x%x)", esp_err_to_name(i2c_err), (unsigned)i2c_err);
	} else {
		ESP_LOGI("rocketlog", "I2C init OK");
	}

	const esp_err_t mpl_err = (i2c_err == ESP_OK) ? mpl3115a2_init() : ESP_FAIL;
	if (mpl_err == ESP_OK) {
		ESP_LOGI("rocketlog", "MPL3115A2 init OK");
	} else {
		ESP_LOGE("rocketlog", "MPL3115A2 init error: %s (0x%x)", esp_err_to_name(mpl_err), (unsigned)mpl_err);
	}

	const double t0 = rocketlog_monotonic_seconds();

	while (1) {
		const double t_unix = rocketlog_current_unix_time();
		if (!rocketlog_time_is_set()) {
			// Skip output until time is set

			vTaskDelay(SAMPLE_PERIOD);
			continue;
		}
		const double t = t_unix - t0;

		float alt_m = 0.0f;
		float temp_c = 0.0f;
		const bool mpl_ok = (mpl3115a2_read(&alt_m, &temp_c) == ESP_OK);

		// Velocity and battery are placeholders until IMU/ADC are wired.
		double vel = 0.0;
		double batt = 12.6 - 0.002 * t;

		if (!mpl_ok) {
			ESP_LOGW("rocketlog", "MPL3115A2 read failed");
		}

		// Altitude from MPL3115A2 is meters. Protocol uses "altitude" as meters.
		double alt = mpl_ok ? (double)alt_m : 0.0;
		double temp = mpl_ok ? (double)temp_c : 0.0;

		telemetry_sample_t sample = {
			.unix_time = t_unix,
			.altitude = (float)alt,
			.velocity = (float)vel,
			.battery = (float)batt,
			.temperature = (float)temp,
		};
		// Temporarily disable binary-framed telemetry output so idf.py monitor
		// remains readable while debugging sensor wiring.
		(void)sample;

		vTaskDelay(SAMPLE_PERIOD);
	}
}

/* ---------------------------------------- */

static int serial_read_frame(uint8_t *out, size_t out_len) {
	// Non-blocking frame receive from USB Serial/JTAG.
	// Returns decoded length, or -1 if no complete frame yet.
	static uint8_t enc[512];
	static size_t enc_len = 0;

	while (1) {
		uint8_t ch = 0;
		const int rx = usb_serial_jtag_read_bytes(&ch, 1, 0);
		if (rx <= 0) {
			return -1;
		}
		if (ch == 0) {
			break;
		}
		if (enc_len < sizeof(enc)) {
			enc[enc_len++] = ch;
		} else {
			enc_len = 0;
			serial_send_log(ROCKETLOG_LOG_WARN, "RX frame too large; dropping");
		}
	}

	if (enc_len == 0) {
		return -1;
	}
	const size_t dec_len = rocketlog_cobs_decode(out, out_len, enc, enc_len);
	enc_len = 0;
	if (dec_len == 0) {
		return -1;
	}
	return (int)dec_len;
}

static void time_sync_task(void *arg) {
	(void)arg;

	uint8_t pkt[512];
	while (1) {
		status_led_update();

		// IMPORTANT: Do not block here. Use non-blocking USB Serial/JTAG reads.
		const int pkt_len = serial_read_frame(pkt, sizeof(pkt));
		if (pkt_len <= 0) {
			vTaskDelay(pdMS_TO_TICKS(10));
			continue;
		}

		uint8_t msg_type = 0;
		const uint8_t *payload = NULL;
		size_t payload_len = 0;
		if (rocketlog_packet_decode_v1(&msg_type, &payload, &payload_len, pkt, (size_t)pkt_len) != 0) {
			serial_send_log(ROCKETLOG_LOG_WARN, "Dropped invalid packet (decode/crc)");
			continue;
		}

		if (msg_type == ROCKETLOG_MSG_TIME_SYNC) {
			serial_send_log(ROCKETLOG_LOG_INFO, "TIME_SYNC received");
			if (payload_len != 12) {
				serial_send_log(ROCKETLOG_LOG_WARN, "TIME_SYNC bad payload length");
				continue;
			}
			int64_t unix_time_us = 0;
			uint32_t seq = 0;
			memcpy(&unix_time_us, &payload[0], sizeof(unix_time_us));
			memcpy(&seq, &payload[8], sizeof(seq));

			rocketlog_time_set_unix((double)unix_time_us / 1000000.0);

			uint8_t ack_pkt[64];
			const int ack_len =
				rocketlog_ack_packet_encode_v1(ack_pkt, sizeof(ack_pkt), ROCKETLOG_MSG_TIME_SYNC, seq, 0, unix_time_us);
			if (ack_len > 0) {
				serial_send_frame(ack_pkt, (size_t)ack_len);
			} else {
				serial_send_log(ROCKETLOG_LOG_ERROR, "Failed to encode ACK");
			}
			serial_send_log(ROCKETLOG_LOG_INFO, "Time sync applied");

			// Test: send one log of each level after successful time sync.
			serial_send_log(ROCKETLOG_LOG_DEBUG, "Test log: DEBUG");
			serial_send_log(ROCKETLOG_LOG_INFO, "Test log: INFO");
			serial_send_log(ROCKETLOG_LOG_WARN, "Test log: WARN");
			serial_send_log(ROCKETLOG_LOG_ERROR, "Test log: ERROR");
		}
	}
}

/* ---------------------------------------- */

void app_main(void) {
	status_led_init();

	// IMPORTANT: Do not initialize ESP-IDF driver objects here.
	// app_main runs in the context of the main task, but some drivers allocate
	// internal FreeRTOS objects and are happier if initialized from an already-running
	// application task. We'll init I2C + MPL inside telemetry_task.

	status_led_update();

	xTaskCreate(time_sync_task, "time_sync_task", 4096, NULL, 6, NULL);
	xTaskCreate(telemetry_task, "telemetry_task", 4096, NULL, 5, NULL);
	xTaskCreate(i2c_task, "i2c_task", 4096, NULL, 5, NULL);
}
