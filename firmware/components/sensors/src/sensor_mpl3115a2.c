#include "sensor_mpl3115a2.h"
#include "rocketlog_board.h"

#include <string.h>
#include "esp_log.h"
#include "driver/i2c_master.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

static const char *TAG = "mpl3115a2";

// MPL3115A2 barometric pressure sensor (barometer mode)
#define MPL3115A2_ADDR 0x60
#define MPL3115A2_REG_STATUS 0x00
#define MPL3115A2_REG_OUT_P_MSB 0x01
#define MPL3115A2_REG_OUT_T_MSB 0x04
#define MPL3115A2_REG_WHO_AM_I 0x0C
#define MPL3115A2_REG_CTRL_REG1 0x26
#define MPL3115A2_REG_PT_DATA_CFG 0x13

#define MPL3115A2_WHO_AM_I_VAL 0xC4

#define I2C_PORT I2C_NUM_0
#define I2C_FREQ 40000

// Static variables
static i2c_master_bus_handle_t i2c_bus = NULL;
static i2c_master_dev_handle_t mpl_dev = NULL;
static bool initialized = false;

// Helper functions
static esp_err_t i2c_init(void) {
	if (i2c_bus != NULL) {
		return ESP_OK;
	}

	i2c_master_bus_config_t bus_cfg = {
		.i2c_port = I2C_PORT,
		.sda_io_num = ROCKETLOG_I2C_SDA_GPIO,
		.scl_io_num = ROCKETLOG_I2C_SCL_GPIO,
		.clk_source = I2C_CLK_SRC_DEFAULT,
		.glitch_ignore_cnt = 7,
		.intr_priority = 0,
		.trans_queue_depth = 0,
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
		ESP_LOGE(TAG, "i2c_new_master_bus failed: %s", esp_err_to_name(err));
		return err;
	}

	if (i2c_master_probe(i2c_bus, MPL3115A2_ADDR, -1) != ESP_OK) {
		ESP_LOGE(TAG, "MPL3115A2 not found on I2C bus");
		return ESP_ERR_NOT_FOUND;
	}

	err = i2c_master_bus_add_device(i2c_bus, &dev_cfg, &mpl_dev);
	if (err != ESP_OK) {
		ESP_LOGE(TAG, "i2c_master_bus_add_device failed: %s", esp_err_to_name(err));
	}

	return err;
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
		ESP_LOGE(TAG, "i2c_read_regs: mpl_dev is NULL");
		return ESP_ERR_INVALID_STATE;
	}
	if (out == NULL || out_len == 0) {
		ESP_LOGE(TAG, "i2c_read_regs: invalid output buffer");
		return ESP_ERR_INVALID_ARG;
	}

	return i2c_master_transmit_receive(mpl_dev, &start_reg, 1, out, out_len, 50);
}

// Public API implementation
esp_err_t mpl3115a2_init(void) {
	if (initialized) {
		return ESP_OK;
	}

	esp_err_t err = i2c_init();
	if (err != ESP_OK) {
		return err;
	}

	uint8_t who = 0;
	err = i2c_read_regs(MPL3115A2_REG_WHO_AM_I, &who, 1);
	ESP_LOGI(TAG, "MPL3115A2 WHO_AM_I read: err=%s (0x%x) who=0x%02x", esp_err_to_name(err), (unsigned)err, who);

	if (err != ESP_OK) {
		return err;
	}

	if (who != MPL3115A2_WHO_AM_I_VAL) {
		ESP_LOGE(TAG, "MPL3115A2 WHO_AM_I mismatch: expected=0x%02X got=0x%02X", MPL3115A2_WHO_AM_I_VAL, who);
		return ESP_ERR_INVALID_RESPONSE;
	}

	// Enable data flags for pressure/altitude + temperature.
	// PT_DATA_CFG: set DREM | PDEFE | TDEFE
	err = i2c_write_reg(MPL3115A2_REG_PT_DATA_CFG, 0x07);
	if (err != ESP_OK) {
		return err;
	}

	// CTRL_REG1: ALT=0 (barometer), OS=128 (0b111), SBYB=0 (standby)
	// 0x19 = 0001 1001
	err = i2c_write_reg(MPL3115A2_REG_CTRL_REG1, 0x19);
	if (err != ESP_OK) {
		ESP_LOGE(TAG, "Failed to write CTRL_REG1: %s", esp_err_to_name(err));
		return err;
	}

	initialized = true;
	ESP_LOGI(TAG, "MPL3115A2 initialized successfully");
	return ESP_OK;
}

esp_err_t mpl3115a2_read(mpl3115a2_data_t *data) {
	if (!initialized || data == NULL) {
		return ESP_ERR_INVALID_STATE;
	}

	uint8_t status = 0;
	esp_err_t err = i2c_read_regs(MPL3115A2_REG_STATUS, &status, 1);
	if (err != ESP_OK) {
		ESP_LOGE(TAG, "i2c_read_regs failed err=%s (0x%x)", esp_err_to_name(err), (unsigned)err);
		return err;
	}

	// STATUS: PDR (bit2) and TDR (bit1). In barometer mode, pressure registers hold pressure.
	// If not ready yet, kick a one-shot measurement and retry a few times.
	// NOTE: currently causes garbled data output, so just timeout instead.
	/* for (int attempt = 0; attempt < 10 && ((status & 0x06) != 0x06); attempt++) { */
	/* 	// CTRL_REG1: set OST (one-shot trigger) while keeping current mode bits. */
	/* 	uint8_t ctrl = 0; */
	/* 	(void)i2c_read_regs(MPL3115A2_REG_CTRL_REG1, &ctrl, 1); */
	/* 	(void)i2c_write_reg(MPL3115A2_REG_CTRL_REG1, (uint8_t)(ctrl | 0x40)); */
	/* 	vTaskDelay(pdMS_TO_TICKS(1000)); */
	/* 	err = i2c_read_regs(MPL3115A2_REG_STATUS, &status, 1); */
	/* 	if (err != ESP_OK) { */
	/* 		return err; */
	/* 	} */
	/* } */

	if ((status & 0x06) != 0x06) {
		ESP_LOGW(TAG, "MPL3115A2 timeout waiting for data ready");
		return ESP_ERR_TIMEOUT;
	}

	uint8_t buf[5] = {0};
	err = i2c_read_regs(MPL3115A2_REG_OUT_P_MSB, buf, sizeof(buf));
	if (err != ESP_OK) {
		ESP_LOGE(TAG, "Error reading data registers: %s (0x%x)", esp_err_to_name(err), (unsigned)err);
		return err;
	}

	ESP_LOGD(TAG, "MPL3115A2 raw: %02x %02x %02x %02x %02x", buf[0], buf[1], buf[2], buf[3], buf[4]);

	// Pressure: 20-bit unsigned (Pascals).
	uint32_t press_raw = ((uint32_t)buf[0] << 16) | ((uint32_t)buf[1] << 8) | buf[2];
	press_raw >>= 4;
	data->pressure_pa = (float)press_raw / 4.0f;

	// Temperature: 12-bit two's complement, Q8.4 (degC).
	int16_t temp_raw = (int16_t)(((uint16_t)buf[3] << 8) | (uint16_t)buf[4]);
	temp_raw >>= 4;
	if (temp_raw & (1 << 11)) {
		temp_raw -= (1 << 12);
	}
	data->temperature_c = (float)temp_raw / 16.0f;

	data->valid = true;
	return ESP_OK;
}

bool mpl3115a2_is_ready(void) {
	return initialized && (mpl_dev != NULL);
}
