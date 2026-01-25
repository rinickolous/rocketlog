#pragma once

// Board-level pin mappings shared across RocketLog firmwares.

// Status RGB LED (SK6812) on ESP32-S3 dev board.
#define ROCKETLOG_RGB_LED_GPIO 48

// GPS module UART pins (GY-GPS6MV2)
#define ROCKETLOG_GPS_TX_GPIO 17 // Corresponds to RX of GPS module
#define ROCKETLOG_GPS_RX_GPIO 16 // Corresponds to TX of GPS module

// I2C pins for sensors
#define ROCKETLOG_I2C_SCL_GPIO 11
#define ROCKETLOG_I2C_SDA_GPIO 12
