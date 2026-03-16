#pragma once

/*
 * RocketLog event code system
 *
 * Over LoRa, verbose text log messages are too large. Instead, firmware emits
 * compact EVENT packets: a 1-byte code identifies the event; an optional 4-byte
 * signed integer parameter carries a numeric value (altitude, error id, etc.).
 *
 * Packet type: ROCKETLOG_MSG_EVENT (5)
 * Payload:     [code : u8][param : i32]   (5 bytes, little-endian)
 *
 * Code groups (high nibble)
 * -------------------------
 *   0x0x  System / lifecycle
 *   0x1x  Flight phase transitions
 *   0x2x  Sensor / hardware status
 *   0x3x  Comms / radio
 *   0x4x  Errors / faults
 *
 * Parameter semantics are code-specific. Where no parameter is meaningful
 * the value 0 should be sent. Signed so negative values (e.g. temperatures,
 * error codes) can be represented without casting.
 */

#ifdef __cplusplus
extern "C" {
#endif

/* ---------------------------------------- */
/*  0x0x  System / lifecycle                */
/* ---------------------------------------- */

/* Firmware boot complete. param = firmware build version (0 if not set). */
#define EVT_SYS_BOOT 0x00

/* Time sync applied. param = UTC unix time (seconds, truncated to i32). */
#define EVT_SYS_TIME_SYNC 0x01

/* Recording started. param = 0. */
#define EVT_SYS_REC_START 0x02

/* Recording stopped. param = 0. */
#define EVT_SYS_REC_STOP 0x03

/* Watchdog reset triggered. param = reset reason code. */
#define EVT_SYS_WATCHDOG 0x04

/* Low battery warning. param = battery_mv (millivolts). */
#define EVT_SYS_LOW_BATT 0x05

/* ---------------------------------------- */
/*  0x1x  Flight phase transitions          */
/* ---------------------------------------- */

/* Pre-launch hold. param = pad hold duration remaining (seconds). */
#define EVT_FLIGHT_PAD_HOLD 0x10

/* Liftoff detected. param = altitude_cm at detection. */
#define EVT_FLIGHT_LIFTOFF 0x11

/* Motor burnout. param = altitude_cm at burnout. */
#define EVT_FLIGHT_BURNOUT 0x12

/* Apogee reached. param = peak altitude_cm. */
#define EVT_FLIGHT_APOGEE 0x13

/* Drogue chute deployed. param = altitude_cm at deployment. */
#define EVT_FLIGHT_DROGUE 0x14

/* Main chute deployed. param = altitude_cm at deployment. */
#define EVT_FLIGHT_MAIN 0x15

/* Landing detected. param = final altitude_cm (should be ~0). */
#define EVT_FLIGHT_LANDING 0x16

/* ---------------------------------------- */
/*  0x2x  Sensor / hardware status          */
/* ---------------------------------------- */

/* GPS fix acquired. param = satellite count. */
#define EVT_SENSOR_GPS_FIX 0x20

/* GPS fix lost. param = satellite count at loss. */
#define EVT_SENSOR_GPS_LOST 0x21

/* Barometer initialised OK. param = 0. */
#define EVT_SENSOR_BARO_OK 0x22

/* Barometer read error. param = error code (driver-specific). */
#define EVT_SENSOR_BARO_ERR 0x23

/* IMU (accelerometer/gyro) ready. param = 0. */
#define EVT_SENSOR_IMU_OK 0x24

/* IMU read error. param = error code. */
#define EVT_SENSOR_IMU_ERR 0x25

/* ---------------------------------------- */
/*  0x3x  Comms / radio                     */
/* ---------------------------------------- */

/* LoRa radio initialised. param = frequency_khz. */
#define EVT_COMMS_LORA_INIT 0x30

/* LoRa TX complete. param = packet sequence number. */
#define EVT_COMMS_LORA_TX 0x31

/* LoRa RX packet received. param = RSSI (dBm, negative). */
#define EVT_COMMS_LORA_RX 0x32

/* LoRa TX error / timeout. param = error code. */
#define EVT_COMMS_LORA_ERR 0x33

/* USB serial connected (ground-station link). param = 0. */
#define EVT_COMMS_USB_CONN 0x34

/* ---------------------------------------- */
/*  0x4x  Errors / faults                   */
/* ---------------------------------------- */

/* Unhandled exception / panic. param = fault address (truncated to i32). */
#define EVT_ERR_PANIC 0x40

/* CRC check failed on a received packet. param = message type that failed. */
#define EVT_ERR_CRC 0x41

/* Packet decode error. param = message type that failed. */
#define EVT_ERR_DECODE 0x42

/* Memory allocation failure. param = requested size (bytes). */
#define EVT_ERR_ALLOC 0x43

/* Task stack overflow. param = task identifier (app-specific). */
#define EVT_ERR_STACK 0x44

/* Sensor timeout — no data within expected interval. param = sensor id. */
#define EVT_ERR_SENSOR_TIMEOUT 0x45

#ifdef __cplusplus
}
#endif
