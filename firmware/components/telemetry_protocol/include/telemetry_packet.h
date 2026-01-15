#pragma once

#include <stddef.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

#include "telemetry_protocol.h"

// Keep packet framing constants colocated with encode/decode.

// Binary packet framing for radio payloads (e.g. LoRa).
//
// Layout (little-endian, packed):
// - magic[2]        : 0x52 0x4C ("RL")
// - version         : 1
// - msg_type        : 1 (telemetry)
// - payload_len     : bytes of payload (currently fixed)
// - unix_time_us    : int64 (UTC unix epoch microseconds)
// - altitude_cm     : int32
// - velocity_cms    : int32
// - battery_mv      : uint16
// - temperature_cC  : int16 (centi-degC)
// - crc32           : CRC-32 of all prior bytes
//
// This format intentionally avoids floats for deterministic encoding.

#define ROCKETLOG_PACKET_MAGIC0 0x52
#define ROCKETLOG_PACKET_MAGIC1 0x4C
#define ROCKETLOG_PACKET_VERSION 1

typedef enum {
	ROCKETLOG_MSG_TELEMETRY = 1,
	ROCKETLOG_MSG_LOG = 2,
	ROCKETLOG_MSG_TIME_SYNC = 3,
	ROCKETLOG_MSG_ACK = 4,
} rocketlog_msg_type_t;

typedef enum {
	ROCKETLOG_LOG_DEBUG = 0,
	ROCKETLOG_LOG_INFO = 1,
	ROCKETLOG_LOG_WARN = 2,
	ROCKETLOG_LOG_ERROR = 3,
} rocketlog_log_level_t;

// Total bytes for a v1 telemetry packet.
#define ROCKETLOG_TELEMETRY_PACKET_V1_LEN 29

int rocketlog_telemetry_packet_encode_v1(uint8_t *out, size_t out_len, const telemetry_sample_t *sample);

int rocketlog_telemetry_packet_decode_v1(telemetry_sample_t *out, const uint8_t *packet, size_t packet_len);

// Variable-length packet helpers (CRC32-protected, little-endian, packed)

// COBS framing helpers for serial streams. Encoded frames contain no 0x00.
// Use 0x00 as the frame delimiter.
size_t rocketlog_cobs_encode(uint8_t *out, size_t out_len, const uint8_t *in, size_t in_len);
size_t rocketlog_cobs_decode(uint8_t *out, size_t out_len, const uint8_t *in, size_t in_len);

// Basic v1 packet (header + payload + crc32).
// Max packet length is enforced by caller.
int rocketlog_packet_encode_v1(uint8_t *out, size_t out_len, uint8_t msg_type, const uint8_t *payload,
							   size_t payload_len);
int rocketlog_packet_decode_v1(uint8_t *msg_type_out, const uint8_t **payload_out, size_t *payload_len_out,
							   const uint8_t *packet, size_t packet_len);

// Convenience payload builders/parsers.
// LOG payload: [level:u8][unix_time_us:i64][msg_utf8...]
int rocketlog_log_packet_encode_v1(uint8_t *out, size_t out_len, rocketlog_log_level_t level, int64_t unix_time_us,
								   const char *msg_utf8);

// TIME_SYNC payload: [unix_time_us:i64][seq:u32]
int rocketlog_time_sync_packet_encode_v1(uint8_t *out, size_t out_len, int64_t unix_time_us, uint32_t seq);

// ACK payload: [ack_type:u8][seq:u32][status:u8][applied_unix_time_us:i64]
int rocketlog_ack_packet_encode_v1(uint8_t *out, size_t out_len, uint8_t ack_type, uint32_t seq, uint8_t status,
								   int64_t applied_unix_time_us);

#ifdef __cplusplus
}
#endif
