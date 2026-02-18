#include "sensor_mpl3115a2.h"
#include "rocketlog_board.h"

#include "driver/i2c_master.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include <string.h>

/* ---------------------------------------- */
/*  Config Knobs                            */
/* ---------------------------------------- */

/* TAG for logging */
static const char *TAG = "mpl3115a2";

/* MPL3115A2 I2C address and register definitions based on the datasheet:
 - 7-bit I2C address is 0x60 (0xC0 write, 0xC1 read)
 - Key registers:
   - STATUS (0x00): data ready flags
   - OUT_P_MSB (0x01): pressure/altitude MSB
   - OUT_T_MSB (0x04): temperature MSB
   - WHO_AM_I (0x0C): should return 0xC4
   - CTRL_REG1 (0x26): control register for mode and oversampling
   - PT_DATA_CFG (0x13): data configuration for pressure/temperature events
*/
#define MPL3115A2_ADDR 0x60
#define MPL3115A2_REG_STATUS 0x00
#define MPL3115A2_REG_OUT_P_MSB 0x01
#define MPL3115A2_REG_OUT_T_MSB 0x04
#define MPL3115A2_REG_WHO_AM_I 0x0C
#define MPL3115A2_REG_CTRL_REG1 0x26
#define MPL3115A2_REG_PT_DATA_CFG 0x13

/* MPL3115A2 WHO_AM_I expected value */
#define MPL3115A2_WHO_AM_I_VAL 0xC4

/* Physical I2C pins and configuration */
#define I2C_PORT I2C_NUM_0
#define I2C_FREQ 40000

/* Static variables */
static i2c_master_bus_handle_t i2c_bus = NULL; /* The I2C bus handle used for communication with the MPL3115A2 sensor.
												  Initialized in i2c_init() and used for all I2C transactions. */
static i2c_master_dev_handle_t mpl_dev = NULL; /* The I2C device handle for the MPL3115A2 sensor, created after probing
												  the device on the bus. Used for register read/write operations. */
static bool mpl_initialized = false; /* State variable to track if the I2C bus has been initialized. Ensures that
										initialization is only performed once. */

/* ---------------------------------------- */

/**
 * Read multiple registers from the MPL3115A2 sensor starting from a specified register address. This function performs
 * an I2C read operation by first sending the starting register address and then reading the specified number of bytes
 * into the provided output buffer. It includes error handling to check for null pointers and invalid buffer lengths,
 * as well as logging to provide feedback on any issues encountered during the read operation.
 *
 * @param start_reg The starting register address to read from
 * @param out Pointer to the output buffer where the read data will be stored
 * @param out_len The number of bytes to read into the output buffer
 * @return ESP_OK on success, or an appropriate error code if the read operation fails
 */
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

/* ---------------------------------------- */

/**
 * Write a single byte value to a specified register on the MPL3115A2 sensor. This function performs an I2C write
 * operation by sending the register address followed by the value to be written. It includes error handling to check
 * if the device handle is valid before attempting the write operation, and returns an appropriate error code if the
 * device is not initialized.
 *
 * @param reg The register address to write to
 * @param value The byte value to write to the specified register
 * @return ESP_OK on success, or an appropriate error code if the write operation fails
 */
static esp_err_t i2c_write_reg(uint8_t reg, uint8_t value) {
	if (mpl_dev == NULL) {
		return ESP_ERR_INVALID_STATE;
	}

	uint8_t buf[2] = {reg, value};
	return i2c_master_transmit(mpl_dev, buf, sizeof(buf), 50);
}
/* ---------------------------------------- */

/**
 * Initialize the I2C bus and probe for the MPL3115A2 sensor. This function sets up the I2C master bus with the
 * specified configuration, including GPIO pins for SDA and SCL, clock source, glitch filter, and internal pull-ups.
 * It then probes the bus to check if the MPL3115A2 sensor is present at the expected address. If found, it adds the
 * device to the bus and creates a handle for it. The function also includes error handling and logging to provide
 * feedback on the initialization process. If the bus is already initialized, it simply returns success to allow
 * multiple calls without issues.
 *
 * @return ESP_OK on successful initialization, or an appropriate error code if initialization fails.
 */
esp_err_t i2c_init(void) {

	/* If the I2C bus is already initialized, simply return success. This allows multiple calls to i2c_init() without
	   causing issues or redundant initialization. */
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

	ESP_LOGD(TAG,
			 "Initializing I2C bus with config: port=%d, SDA=%d, SCL=%d, clk_source=%d, glitch_ignore_cnt=%d, "
			 "intr_priority=%d",
			 bus_cfg.i2c_port, bus_cfg.sda_io_num, bus_cfg.scl_io_num, bus_cfg.clk_source, bus_cfg.glitch_ignore_cnt,
			 bus_cfg.intr_priority);
	esp_err_t err = i2c_new_master_bus(&bus_cfg, &i2c_bus);
	if (err != ESP_OK) {
		ESP_LOGE(TAG, "i2c_new_master_bus failed: %s", esp_err_to_name(err));
		return err;
	}

	if (i2c_master_probe(i2c_bus, MPL3115A2_ADDR, -1) != ESP_OK) {
		ESP_LOGE(TAG, "MPL3115A2 not found on I2C bus");
		return ESP_ERR_NOT_FOUND;
	}

	ESP_LOGD(TAG, "Adding MPL3115A2 device to I2C bus with address 0x%02X", MPL3115A2_ADDR);
	err = i2c_master_bus_add_device(i2c_bus, &dev_cfg, &mpl_dev);
	if (err != ESP_OK) {
		ESP_LOGE(TAG, "i2c_master_bus_add_device failed: %s", esp_err_to_name(err));
		return err;
	}

	return ESP_OK;
}

/* ---------------------------------------- */

/**
 * Check if the MPL3115A2 sensor has new pressure and temperature data ready to be read. This function first verifies
 * that the sensor is initialized, then reads the STATUS register to check the PDR (Pressure Data Ready) and TDR
 * (Temperature Data Ready) bits. It returns true if both pressure and temperature data are ready, and false otherwise.
 * The function also includes error handling to log any issues encountered during the I2C read operation.
 *
 * @return true if both pressure and temperature data are ready, false otherwise
 */
bool mpl3115a2_is_ready(void) {
	if (!mpl_initialized) {
		return false;
	}

	uint8_t status = 0;
	esp_err_t err = i2c_read_regs(MPL3115A2_REG_STATUS, &status, 1);
	if (err != ESP_OK) {
		ESP_LOGE(TAG, "i2c_read_regs failed err=%s (0x%x)", esp_err_to_name(err), (unsigned)err);
		return false;
	}

	bool pressure_ready = (status & 0x04) != 0; // PDR bit
	bool temp_ready = (status & 0x02) != 0;		// TDR bit

	return pressure_ready && temp_ready;
}

/* ---------------------------------------- */

/**
 * Read pressure and temperature data from the MPL3115A2 sensor. This function first checks if the sensor is initialized
 * and if the provided data pointer is valid. It then reads the STATUS register to check if new pressure and temperature
 * data are available. If the data is ready, it reads the pressure and temperature registers, converts the raw values
 * to physical units (Pascals for pressure and Celsius for temperature), and updates the provided data structure with
 * the results. The function also includes error handling to return appropriate error codes if any step of the process
 * fails or if the data is not ready.
 *
 * @param data Pointer to a mpl3115a2_data_t structure where the read pressure and temperature data will be stored
 * @return ESP_OK on success, or an appropriate error code if reading fails or if data is not ready
 */
esp_err_t mpl3115a2_read(mpl3115a2_data_t *data) {
	ESP_LOGD(TAG, "Reading MPL3115A2 sensor data");
	if (!mpl_initialized || data == NULL) {
		return ESP_ERR_INVALID_STATE;
	}

	uint8_t status = 0;
	ESP_LOGD(TAG, "Checking MPL3115A2 STATUS register for data readiness");
	esp_err_t err = i2c_read_regs(MPL3115A2_REG_STATUS, &status, 1);
	ESP_LOGD(TAG, "MPL3115A2 STATUS register read: err=%s (0x%x) status=0x%02x", esp_err_to_name(err), (unsigned)err,
			 status);
	if (err != ESP_OK) {
		ESP_LOGE(TAG, "i2c_read_regs failed err=%s (0x%x)", esp_err_to_name(err), (unsigned)err);
		return err;
	}

	bool pressure_ready = (status & 0x04) != 0; // PDR bit
	bool temp_ready = (status & 0x02) != 0;		// TDR bit

	ESP_LOGD(TAG, "Data readiness: pressure_ready=%d, temp_ready=%d", pressure_ready, temp_ready);

	if (!pressure_ready || !temp_ready) {
		return ESP_ERR_TIMEOUT; // Data not ready
	}

	ESP_LOGD(TAG, "Reading pressure and temperature data from MPL3115A2");

	uint8_t buf[5];
	err = i2c_read_regs(MPL3115A2_REG_OUT_P_MSB, buf, sizeof(buf));
	ESP_LOGD(TAG, "MPL3115A2 data read: err=%s (0x%x) buf=[%02x %02x %02x %02x %02x]", esp_err_to_name(err),
			 (unsigned)err, buf[0], buf[1], buf[2], buf[3], buf[4]);
	if (err != ESP_OK) {
		ESP_LOGE(TAG, "Failed to read pressure/temp data: %s (0x%x)", esp_err_to_name(err), (unsigned)err);
		return err;
	}

	int32_t raw_pressure = ((int32_t)buf[0] << 16) | ((int32_t)buf[1] << 8) | buf[2];
	raw_pressure >>= 4;						 // 20-bit value
	data->pressure_pa = raw_pressure / 4.0f; // Convert to Pascals

	ESP_LOGD(TAG, "Raw pressure: %d, Pressure (Pa): %.1f", raw_pressure, data->pressure_pa);

	int16_t raw_temp = ((int16_t)buf[3] << 8) | buf[4];
	raw_temp >>= 4;							// 12-bit value
	data->temperature_c = raw_temp / 16.0f; // Convert to Celsius

	ESP_LOGD(TAG, "Raw temperature: %d, Temperature (°C): %.1f", raw_temp, data->temperature_c);

	data->valid = true;
	return ESP_OK;
}

/* ---------------------------------------- */

esp_err_t mpl3115a2_init(void) {
	if (mpl_initialized) {
		return ESP_OK;
	}

	esp_err_t err = i2c_init();
	if (err != ESP_OK) {
		return err;
	}

	uint8_t who = 0;
	err = i2c_read_regs(MPL3115A2_REG_WHO_AM_I, &who, 1);
	ESP_LOGD(TAG, "MPL3115A2 WHO_AM_I read: err=%s (0x%x) who=0x%02x", esp_err_to_name(err), (unsigned)err, who);

	if (err != ESP_OK) {
		return err;
	}

	if (who != MPL3115A2_WHO_AM_I_VAL) {
		ESP_LOGE(TAG, "MPL3115A2 WHO_AM_I mismatch: expected=0x%02X got=0x%02X", MPL3115A2_WHO_AM_I_VAL, who);
		return ESP_ERR_INVALID_RESPONSE;
	}

	// Eable data dlags for pressure/altitude + temperature.
	// PT_DATA_CFG: set DREM | PDEFE | TDEFE
	err = i2c_write_reg(MPL3115A2_REG_PT_DATA_CFG, 0x07);
	if (err != ESP_OK) {
		ESP_LOGE(TAG, "Failed to write PT_DATA_CFG: %s (0x%x)", esp_err_to_name(err), (unsigned)err);
		return err;
	}

	// CTRL_REG1: ALT=0 (barometer), OS=128 (0b111), SBYB=0 (standby)
	// 0x19 = 0b00011001: OS=128, SBYB=0, ALT=0
	err = i2c_write_reg(MPL3115A2_REG_CTRL_REG1, 0x19);
	if (err != ESP_OK) {
		ESP_LOGE(TAG, "Failed to write CTRL_REG1: %s (0x%x)", esp_err_to_name(err), (unsigned)err);
		return err;
	}

	ESP_LOGI(TAG, "MPL3115A2 initialization complete, putting sensor in active mode");
	mpl_initialized = true;
	return ESP_OK;
}
