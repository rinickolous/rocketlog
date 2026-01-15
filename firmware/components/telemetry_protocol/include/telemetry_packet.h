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
} rocketlog_msg_type_t;

// Total bytes for a v1 telemetry packet.
#define ROCKETLOG_TELEMETRY_PACKET_V1_LEN 29

int rocketlog_telemetry_packet_encode_v1(uint8_t *out, size_t out_len, const telemetry_sample_t *sample);

int rocketlog_telemetry_packet_decode_v1(telemetry_sample_t *out, const uint8_t *packet, size_t packet_len);

#ifdef __cplusplus
}
#endif
