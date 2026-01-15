#include <math.h>
#include <stdint.h>
#include <stdio.h>

#include <stdbool.h>
#include <string.h>

#include "rocketlog_time.h"
#include "telemetry_packet.h"
#include "telemetry_protocol.h"

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#include "driver/i2c.h"
#include "driver/rmt_tx.h"
#include "esp_log.h"
#include "esp_timer.h"
#include "led_strip.h"
#include "led_strip_rmt.h"

#include "driver/usb_serial_jtag.h"

/* ---------------------------------------- */
/*  Config Knobs                            */
/* ---------------------------------------- */

/* Time keeping moved to components/common (rocketlog_time.*) */

static volatile int64_t last_host_seen_us = 0;		// updated on any host command
static const int64_t HOST_TIMEOUT_US = 5 * 1000000; // 5 seconds

static const TickType_t SAMPLE_PERIOD = pdMS_TO_TICKS(100);

#define RGB_LED_GPIO 48

static led_strip_handle_t led_strip;

// I2C

#define I2C_PORT I2C_NUM_0
#define I2C_SCL 10
#define I2C_SDA 11
#define I2C_FREQ 100000

/* ---------------------------------------- */
/*  Host Connect                            */
/* ---------------------------------------- */

static bool host_alive(void) {
	int64_t now = esp_timer_get_time();
	int64_t age = now - last_host_seen_us;
	return (last_host_seen_us != 0) && (age <= HOST_TIMEOUT_US);
}

/* ---------------------------------------- */
/*  I2C Functions                           */
/* ---------------------------------------- */

static void i2c_init(void) {
	i2c_config_t conf = {
		.mode = I2C_MODE_MASTER,
		.sda_io_num = I2C_SDA,
		.scl_io_num = I2C_SCL,
		.sda_pullup_en = GPIO_PULLUP_ENABLE,
		.scl_pullup_en = GPIO_PULLUP_ENABLE,
		.master.clk_speed = I2C_FREQ,
	};
	ESP_ERROR_CHECK(i2c_param_config(I2C_PORT, &conf));
	ESP_ERROR_CHECK(i2c_driver_install(I2C_PORT, conf.mode, 0, 0, 0));
}

/* ---------------------------------------- */

static void serial_send_frame(const uint8_t *data, size_t len) {
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

static void i2c_task(void *arg) {
	(void)arg;

	while (1) {
		int found = 0;

		for (uint8_t addr = 1; addr < 0x7F; addr++) {
			i2c_cmd_handle_t cmd = i2c_cmd_link_create();
			i2c_master_start(cmd);
			i2c_master_write_byte(cmd, (addr << 1) | I2C_MASTER_WRITE, true);
			i2c_master_stop(cmd);

			esp_err_t err = i2c_master_cmd_begin(I2C_PORT, cmd, pdMS_TO_TICKS(50));

			i2c_cmd_link_delete(cmd);

			if (err == ESP_OK) {
				found++;
			}
		}

		if (!found) {
			serial_send_log(ROCKETLOG_LOG_WARN, "No I2C devices found");
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
	if (rocketlog_time_is_set() && host_alive()) {
		// Solid green
		led_strip_set_pixel(led_strip, 0, 0, 64, 0);
		led_strip_refresh(led_strip);
	} else {
		// Slow pulse red
		double t = rocketlog_monotonic_seconds();

		uint8_t brightness = (uint8_t)((sin(t * 2.0) + 1.0) / 2.0 * 64.0);
		led_strip_set_pixel(led_strip, 0, brightness, 0, 0);
		/* led_strip_set_pixel(led_strip, 0, 64, 0, 0); // red */
		led_strip_refresh(led_strip);
	}
}

/* ---------------------------------------- */

static void telemetry_task(void *arg) {
	(void)arg;

	// Disable ESP logging to keep stdout as a binary-framed stream.
	esp_log_level_set("*", ESP_LOG_NONE);

	// Install USB Serial/JTAG driver for host commands.
	usb_serial_jtag_driver_config_t cfg = {
		.rx_buffer_size = 1024,
		.tx_buffer_size = 1024,
	};
	ESP_ERROR_CHECK(usb_serial_jtag_driver_install(&cfg));

	// stdout is not used for framing; keep it quiet anyway.
	setvbuf(stdout, NULL, _IONBF, 0);

	serial_send_log(ROCKETLOG_LOG_INFO, "Receiver started (binary framed)");

	const double t0 = rocketlog_monotonic_seconds();

	while (1) {
		const double t_unix = rocketlog_current_unix_time();
		if (!rocketlog_time_is_set()) {
			// Skip output until time is set

			vTaskDelay(SAMPLE_PERIOD);
			continue;
		}
		const double t = t_unix - t0;

		// Fake ascent / descent curve (mirrors your Python)
		double alt = fmax(0.0, 5.0 * t - 0.02 * t * t) * 10.0;
		double vel = (5.0 - 0.04 * t) * 10.0;

		// Battery droop
		double batt = 12.6 - 0.002 * t;

		// Temperature oscillation
		double temp = 22.0 + 2.0 * sin(t / 5.0);

		telemetry_sample_t sample = {
			.unix_time = t_unix,
			.altitude = (float)alt,
			.velocity = (float)vel,
			.battery = (float)batt,
			.temperature = (float)temp,
		};
		uint8_t pkt[ROCKETLOG_TELEMETRY_PACKET_V1_LEN];
		const int pkt_len = rocketlog_telemetry_packet_encode_v1(pkt, sizeof(pkt), &sample);
		if (pkt_len > 0) {
			serial_send_frame(pkt, (size_t)pkt_len);
		}

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
		last_host_seen_us = esp_timer_get_time();
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
		last_host_seen_us = esp_timer_get_time();

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
			last_host_seen_us = esp_timer_get_time();

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
	i2c_init();
	status_led_update(); // starts RED

	xTaskCreate(time_sync_task, "time_sync_task", 4096, NULL, 6, NULL);
	xTaskCreate(telemetry_task, "telemetry_task", 4096, NULL, 5, NULL);
	xTaskCreate(i2c_task, "i2c_task", 4096, NULL, 5, NULL);
}
