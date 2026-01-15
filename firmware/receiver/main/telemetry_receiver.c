#include <math.h>
#include <stdint.h>
#include <stdio.h>

#include <stdbool.h>

#include "rocketlog_time.h"
#include "telemetry_protocol.h"

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#include "driver/i2c.h"
#include "driver/rmt_tx.h"
#include "esp_log.h"
#include "esp_timer.h"
#include "led_strip.h"
#include "led_strip_rmt.h"

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
				printf("Found device at 0x%02X\n", addr);
				found++;
			}
		}

		if (!found) {
			printf("No I2C devices found\n");
		}

		printf("Scan done\n\n");
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

	// Disable ESP logging to keep the serial stream "pure CSV"
	esp_log_level_set("*", ESP_LOG_NONE);

	// Unbuffer stdout so lines appear immediately
	setvbuf(stdout, NULL, _IONBF, 0);

	const double t0 = rocketlog_monotonic_seconds();

	while (1) {
		const double t_unix = rocketlog_current_unix_time();
		if (!rocketlog_time_is_set()) {
			// Skip output until time is set
			/* printf("Error: Time is not set! Waiting...\n"); */
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
		char line[96];
		if (telemetry_csv_encode(line, sizeof(line), &sample) > 0) {
			fputs(line, stdout);
		}

		vTaskDelay(SAMPLE_PERIOD);
	}
}

/* ---------------------------------------- */

static void time_sync_task(void *arg) {
	(void)arg;
	char buf[128];

	while (1) {
		status_led_update();
		if (fgets(buf, sizeof(buf), stdin) == NULL) {
			vTaskDelay(pdMS_TO_TICKS(50));

			continue;
		}
		last_host_seen_us = esp_timer_get_time();

		double unix_time;
		if (sscanf(buf, "SET_TIME,%lf", &unix_time) == 1) {
			rocketlog_time_set_unix(unix_time);
			last_host_seen_us = esp_timer_get_time();
			printf("# TIME_SET %.6f\n", unix_time);
		}
	}
}

/* ---------------------------------------- */

void app_main(void) {
	status_led_init();
	/* i2c_init(); */
	status_led_update(); // starts RED

	xTaskCreate(time_sync_task, "time_sync_task", 4096, NULL, 6, NULL);
	xTaskCreate(telemetry_task, "telemetry_task", 4096, NULL, 5, NULL);
	/* xTaskCreate(i2c_task, "i2c_task", 4096, NULL, 5, NULL); */
}
