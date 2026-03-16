#include <math.h>
#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>
#include <string.h>

#include "rocketlog_board.h"
#include "rocketlog_time.h"
#include "telemetry_packet.h"
#include "telemetry_protocol.h"

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#include "esp_log.h"
#include "esp_timer.h"
#include "led_strip.h"
#include "led_strip_rmt.h"

#include "driver/usb_serial_jtag.h"

/* ---------------------------------------- */
/*  Launch site parameters                  */
/* ---------------------------------------- */

// Approx. coordinates of a generic launch site (can be adjusted).
// Latitude and longitude in decimal degrees.
#define LAUNCH_LAT 43.6150	// deg N
#define LAUNCH_LON -79.3900 // deg W
#define LAUNCH_ALT 76.0f	// metres ASL (site elevation)

/* ---------------------------------------- */
/*  Flight profile constants                */
/* ---------------------------------------- */

// All durations in seconds from liftoff (t=0).
// Phases: pre-launch pad hold → boost → coast → apogee → drogue → main → landing

#define T_LAUNCH 0.0	   // liftoff
#define T_BURNOUT 2.8	   // motor burnout
#define T_APOGEE 11.5	   // peak altitude
#define T_DROGUE 12.0	   // drogue deploys (0.5 s after apogee)
#define T_MAIN_DEPLOY 52.0 // main chute deploys at ~300 m AGL
#define T_LANDING 80.0	   // touchdown

#define APOGEE_ALT_M 2000.0	  // target peak altitude metres AGL
#define MAIN_DEPLOY_ALT 300.0 // altitude at main chute deployment

// GPS cold-start: fix acquired this many seconds after launch start.
#define GPS_FIX_DELAY_S 4.0

// Horizontal drift: constant slow wind drift during flight.
// Expressed as metres per second north / east.
#define DRIFT_NORTH_MPS 2.5f
#define DRIFT_EAST_MPS 1.2f

/* ---------------------------------------- */
/*  Sample period                           */
/* ---------------------------------------- */

// Physics/LED loop ticks at 10 Hz for smooth state transitions.
static const TickType_t TICK_PERIOD = pdMS_TO_TICKS(100);

// Telemetry transmit intervals.
#define TX_INTERVAL_PAD_MS 5000	   // 1 packet per 5 s during pad hold
#define TX_INTERVAL_FLIGHT_MS 1000 // 1 packet per 1 s during flight

/* ---------------------------------------- */
/*  LED                                     */
/* ---------------------------------------- */

#define RGB_LED_GPIO ROCKETLOG_RGB_LED_GPIO

static led_strip_handle_t led_strip;

static void status_led_init(void) {
	led_strip_config_t strip_config = {
		.strip_gpio_num = RGB_LED_GPIO,
		.max_leds = 1,
		.led_model = LED_MODEL_SK6812,
		.flags.invert_out = false,
	};
	led_strip_rmt_config_t rmt_config = {
		.clk_src = RMT_CLK_SRC_DEFAULT,
		.resolution_hz = 10 * 1000 * 1000,
		.mem_block_symbols = 64,
	};
	ESP_ERROR_CHECK(led_strip_new_rmt_device(&strip_config, &rmt_config, &led_strip));
	led_strip_clear(led_strip);
	led_strip_refresh(led_strip);
}

/* ---------------------------------------- */
/*  Serial framing (identical to receiver)  */
/* ---------------------------------------- */

static void serial_send_frame(const uint8_t *data, size_t len) {
	// COBS-encode the packet and terminate with 0x00.
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

/* ---------------------------------------- */
/*  Time-sync task (identical to receiver)  */
/* ---------------------------------------- */

static int serial_read_frame(uint8_t *out, size_t out_len) {
	// Non-blocking COBS frame receive from USB Serial/JTAG.
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
			}
			serial_send_log(ROCKETLOG_LOG_INFO, "Time sync applied");
		}
	}
}

/* ---------------------------------------- */
/*  Flight model helpers                    */
/* ---------------------------------------- */

// Smooth step: maps t in [0,1] → [0,1] with zero derivative at both ends.
static inline double smoothstep(double t) {
	if (t <= 0.0)
		return 0.0;
	if (t >= 1.0)
		return 1.0;
	return t * t * (3.0 - 2.0 * t);
}

// Clamp a double to [lo, hi].
static inline double clampd(double v, double lo, double hi) {
	return v < lo ? lo : (v > hi ? hi : v);
}

// Return altitude (metres AGL) and velocity (m/s) for a given flight time t (s).
// t = 0 is liftoff.
static void flight_model(double t, double *alt_out, double *vel_out) {
	double alt = 0.0;
	double vel = 0.0;

	if (t < T_LAUNCH) {
		// Pre-launch: sitting on the pad.
		alt = 0.0;
		vel = 0.0;

	} else if (t < T_BURNOUT) {
		// Boost phase: motor burn — approximately constant high thrust.
		// Use a smooth curve that reaches ~60% of apogee altitude at burnout.
		double frac = (t - T_LAUNCH) / (T_BURNOUT - T_LAUNCH);
		// Altitude at burnout: ~600 m for 2000 m apogee.
		double alt_burnout = APOGEE_ALT_M * 0.30;
		// Velocity at burnout peaks around 250 m/s (Mach ~0.75).
		double vel_burnout = 250.0;
		alt = alt_burnout * smoothstep(frac);
		vel = vel_burnout * smoothstep(frac);

	} else if (t < T_APOGEE) {
		// Coast phase: motor off, drag and gravity decelerating.
		double frac = (t - T_BURNOUT) / (T_APOGEE - T_BURNOUT);
		double alt_burnout = APOGEE_ALT_M * 0.30;
		double vel_burnout = 250.0;
		// Altitude rises from alt_burnout to APOGEE_ALT_M.
		alt = alt_burnout + (APOGEE_ALT_M - alt_burnout) * smoothstep(frac);
		// Velocity drops from vel_burnout to 0 at apogee.
		vel = vel_burnout * (1.0 - smoothstep(frac));

	} else if (t < T_DROGUE) {
		// Brief moment at/just past apogee.
		alt = APOGEE_ALT_M;
		vel = 0.0;

	} else if (t < T_MAIN_DEPLOY) {
		// Drogue descent: ~25 m/s sink rate.
		double dt = t - T_DROGUE;
		double drogue_rate = 25.0; // m/s descent
		alt = APOGEE_ALT_M - drogue_rate * dt;
		vel = -drogue_rate;
		// Clamp to main deploy altitude.
		if (alt < MAIN_DEPLOY_ALT) {
			alt = MAIN_DEPLOY_ALT;
		}

	} else if (t < T_LANDING) {
		// Main chute descent: ~5 m/s sink rate from MAIN_DEPLOY_ALT.
		double dt = t - T_MAIN_DEPLOY;
		double main_rate = MAIN_DEPLOY_ALT / (T_LANDING - T_MAIN_DEPLOY);
		alt = MAIN_DEPLOY_ALT - main_rate * dt;
		vel = -main_rate;
		if (alt < 0.0) {
			alt = 0.0;
			vel = 0.0;
		}

	} else {
		// Landed.
		alt = 0.0;
		vel = 0.0;
	}

	*alt_out = clampd(alt, 0.0, APOGEE_ALT_M + 10.0);
	*vel_out = vel;
}

/* ---------------------------------------- */
/*  Telemetry task                          */
/* ---------------------------------------- */

static void telemetry_task(void *arg) {
	(void)arg;

	serial_send_log(ROCKETLOG_LOG_DEBUG, "Log level check: DEBUG");
	serial_send_log(ROCKETLOG_LOG_INFO, "Sim started (binary framed, v2 telemetry)");
	serial_send_log(ROCKETLOG_LOG_WARN, "Log level check: WARN");
	serial_send_log(ROCKETLOG_LOG_ERROR, "Log level check: ERROR");
	serial_send_log(ROCKETLOG_LOG_INFO, "Waiting for TIME_SYNC...");

	// Wait for time sync before starting flight simulation.
	while (!rocketlog_time_is_set()) {
		vTaskDelay(TICK_PERIOD);
	}

	serial_send_log(ROCKETLOG_LOG_INFO, "Time synced — starting flight simulation");
	serial_send_log(ROCKETLOG_LOG_INFO, "PRE-LAUNCH: holding on pad for 3 s");

	// Record the Unix time at which the sim starts (t_sim=0 is sim start,
	// liftoff happens at T_LAUNCH seconds into the sim).
	const double t_sim_start = rocketlog_current_unix_time();

	// Pre-launch delay (3 s) before liftoff.
	const double t_liftoff_offset = 3.0;

	bool announced_liftoff = false;
	bool announced_burnout = false;
	bool announced_apogee = false;
	bool announced_drogue = false;
	bool announced_main = false;
	bool announced_landing = false;

	// Total horizontal displacement from launch site (metres).
	double disp_north_m = 0.0;
	double disp_east_m = 0.0;

	// GPS satellite count ramp.
	uint8_t gps_sats = 0;
	bool gps_fix = false;

	// Telemetry TX rate control.
	// next_tx_ms tracks the tick-count deadline for the next transmission.
	TickType_t next_tx = xTaskGetTickCount();

	while (1) {
		const double t_unix = rocketlog_current_unix_time();
		// t = seconds since liftoff (negative during pad hold).
		const double t = (t_unix - t_sim_start) - t_liftoff_offset;

		// ---- Altitude and velocity ----------------------------------------
		double alt_agl = 0.0;
		double vel_mps = 0.0;

		if (t >= T_LAUNCH) {
			flight_model(t, &alt_agl, &vel_mps);
		}

		// ---- Battery: droop from 12.6 V to 12.0 V over full flight --------
		double elapsed = t_unix - t_sim_start;
		double batt = 12.6 - 0.006 * clampd(elapsed, 0.0, 100.0);

		// ---- Temperature: ISA lapse rate (~6.5 °C / 1000 m) ---------------
		double temp = 22.0 - 6.5 * (alt_agl / 1000.0);
		// Add small oscillation for realism.
		temp += 0.5 * sin(elapsed / 8.0);

		// ---- GPS simulation -----------------------------------------------
		// Satellite count ramps up during pad hold (cold start acquisition).
		double t_pad = elapsed; // seconds since sim start (before liftoff)
		if (t_pad < GPS_FIX_DELAY_S) {
			gps_sats = (uint8_t)(9.0 * (t_pad / GPS_FIX_DELAY_S));
			gps_fix = false;
		} else {
			// Full fix acquired; vary sats slightly during flight.
			gps_sats = (uint8_t)(9 - (uint8_t)(2.0 * fabs(sin(elapsed / 30.0))));
			gps_fix = true;
		}

		// Horizontal drift accumulates while airborne.
		// dt is one tick period (100 ms) in seconds.
		const double dt_s = (double)pdTICKS_TO_MS(TICK_PERIOD) / 1000.0;
		if (t >= T_LAUNCH && alt_agl > 0.0) {
			disp_north_m += DRIFT_NORTH_MPS * dt_s;
			disp_east_m += DRIFT_EAST_MPS * dt_s;
		}

		// Convert displacement to lat/lon offsets.
		double lat = LAUNCH_LAT + disp_north_m / 111000.0;
		double lon = LAUNCH_LON + disp_east_m / (111000.0 * cos(LAUNCH_LAT * M_PI / 180.0));
		double gps_alt = LAUNCH_ALT + alt_agl + 1.5 * sin(elapsed / 3.0); // ±1.5 m noise

		// ---- Phase announcements ------------------------------------------
		if (!announced_liftoff && t >= T_LAUNCH) {
			serial_send_log(ROCKETLOG_LOG_INFO, "LIFTOFF");
			announced_liftoff = true;
		}
		if (!announced_burnout && t >= T_BURNOUT) {
			serial_send_log(ROCKETLOG_LOG_INFO, "BURNOUT — motor off, coasting");
			announced_burnout = true;
		}
		if (!announced_apogee && t >= T_APOGEE) {
			serial_send_log(ROCKETLOG_LOG_INFO, "APOGEE");
			announced_apogee = true;
		}
		if (!announced_drogue && t >= T_DROGUE) {
			serial_send_log(ROCKETLOG_LOG_INFO, "DROGUE deployed");
			announced_drogue = true;
		}
		if (!announced_main && t >= T_MAIN_DEPLOY) {
			serial_send_log(ROCKETLOG_LOG_INFO, "MAIN chute deployed");
			announced_main = true;
		}
		if (!announced_landing && t >= T_LANDING) {
			serial_send_log(ROCKETLOG_LOG_INFO, "LANDING — flight complete");
			announced_landing = true;
		}

		// ---- LED: blue during flight, solid green after landing -----------
		if (t >= T_LANDING) {
			led_strip_set_pixel(led_strip, 0, 0, 64, 0); // green
		} else if (t >= T_LAUNCH) {
			uint8_t b = (uint8_t)((sin(elapsed * 4.0) + 1.0) / 2.0 * 48.0 + 16.0);
			led_strip_set_pixel(led_strip, 0, 0, 0, b); // pulsing blue
		} else {
			// Pad hold: slow amber pulse.
			uint8_t r = (uint8_t)((sin(elapsed * 1.5) + 1.0) / 2.0 * 40.0 + 8.0);
			led_strip_set_pixel(led_strip, 0, r, r / 4, 0);
		}
		led_strip_refresh(led_strip);

		// ---- Transmit telemetry at rate-limited intervals -----------------
		// Pad hold: 1 packet per 5 s. In flight: 1 packet per 1 s.
		const TickType_t now_ticks = xTaskGetTickCount();
		if ((TickType_t)(now_ticks - next_tx) < (TickType_t)(UINT32_MAX / 2)) {
			const uint32_t tx_interval_ms = (t >= T_LAUNCH) ? TX_INTERVAL_FLIGHT_MS : TX_INTERVAL_PAD_MS;
			next_tx = now_ticks + pdMS_TO_TICKS(tx_interval_ms);

			telemetry_sample_t sample = {
				.unix_time = t_unix,
				.altitude = (float)alt_agl,
				.velocity = (float)vel_mps,
				.battery = (float)batt,
				.temperature = (float)temp,
				.gps_latitude = lat,
				.gps_longitude = lon,
				.gps_altitude = (float)gps_alt,
				.gps_satellites = gps_sats,
				.gps_valid = gps_fix,
			};

			uint8_t pkt[ROCKETLOG_TELEMETRY_PACKET_V2_LEN];
			const int pkt_len = rocketlog_telemetry_packet_encode_v2(pkt, sizeof(pkt), &sample);
			if (pkt_len > 0) {
				serial_send_frame(pkt, (size_t)pkt_len);
			}
		}

		vTaskDelay(TICK_PERIOD);
	}
}

/* ---------------------------------------- */
/*  Entry point                             */
/* ---------------------------------------- */

void app_main(void) {
	// Install USB serial driver before any task starts — both tasks use it.
	esp_log_level_set("*", ESP_LOG_NONE);
	usb_serial_jtag_driver_config_t usb_cfg = {
		.rx_buffer_size = 1024,
		.tx_buffer_size = 1024,
	};
	ESP_ERROR_CHECK(usb_serial_jtag_driver_install(&usb_cfg));
	setvbuf(stdout, NULL, _IONBF, 0);

	status_led_init();

	// LED starts amber (pad hold).
	led_strip_set_pixel(led_strip, 0, 32, 8, 0);
	led_strip_refresh(led_strip);

	xTaskCreate(time_sync_task, "time_sync_task", 4096, NULL, 6, NULL);
	xTaskCreate(telemetry_task, "telemetry_task", 4096, NULL, 5, NULL);
}
