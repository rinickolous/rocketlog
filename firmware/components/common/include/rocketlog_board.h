#pragma once

// Board-level pin mappings shared across RocketLog firmwares.

// Status RGB LED (SK6812) on ESP32-S3 dev board.
#define ROCKETLOG_RGB_LED_GPIO 48

// GPS module UART pins (GY-GPS6MV2)
#define ROCKETLOG_GPS_TX_GPIO 44
#define ROCKETLOG_GPS_RX_GPIO 43

// I2C pins for sensors
#define ROCKETLOG_I2C_SCL_GPIO 11
#define ROCKETLOG_I2C_SDA_GPIO 12
