#include "telemetry_packet.h"

#include <math.h>
#include <stdint.h>
#include <string.h>

#include "esp_crc.h"

#pragma pack(push, 1)
typedef struct {
	uint8_t magic[2];
	uint8_t version;
	uint8_t msg_type;
	uint8_t payload_len;

	int64_t unix_time_us;
	int32_t altitude_cm;
	int32_t velocity_cms;
	uint16_t battery_mv;
	int16_t temperature_cC;

	uint32_t crc32;
} rocketlog_telemetry_packet_v1_t;
#pragma pack(pop)

_Static_assert(sizeof(rocketlog_telemetry_packet_v1_t) == ROCKETLOG_TELEMETRY_PACKET_V1_LEN,
			   "rocketlog_telemetry_packet_v1_t size mismatch");

static uint32_t rocketlog_crc32(const uint8_t *data, size_t len) {
	// ESP-IDF provides a hardware-accelerated CRC implementation.
	return esp_crc32_le(0, data, len);
}

int rocketlog_telemetry_packet_encode_v1(uint8_t *out, size_t out_len, const telemetry_sample_t *sample) {
	if (out == NULL || sample == NULL) {
		return -1;
	}
	if (out_len < ROCKETLOG_TELEMETRY_PACKET_V1_LEN) {
		return -1;
	}

	rocketlog_telemetry_packet_v1_t pkt = {
		.magic = {ROCKETLOG_PACKET_MAGIC0, ROCKETLOG_PACKET_MAGIC1},
		.version = ROCKETLOG_PACKET_VERSION,
		.msg_type = ROCKETLOG_MSG_TELEMETRY,
		.payload_len = (uint8_t)(ROCKETLOG_TELEMETRY_PACKET_V1_LEN - 2 - 1 - 1 - 1 - sizeof(uint32_t)),
	};

	// Scale values into fixed-point integers.
	const double unix_time_us_f = sample->unix_time * 1000000.0;
	pkt.unix_time_us = (int64_t)llround(unix_time_us_f);

	pkt.altitude_cm = (int32_t)llround((double)sample->altitude * 100.0);
	pkt.velocity_cms = (int32_t)llround((double)sample->velocity * 100.0);

	const double batt_mv_f = (double)sample->battery * 1000.0;
	if (batt_mv_f < 0.0) {
		pkt.battery_mv = 0;
	} else if (batt_mv_f > 65535.0) {
		pkt.battery_mv = 65535;
	} else {
		pkt.battery_mv = (uint16_t)llround(batt_mv_f);
	}

	pkt.temperature_cC = (int16_t)llround((double)sample->temperature * 100.0);

	pkt.crc32 = rocketlog_crc32((const uint8_t *)&pkt, sizeof(pkt) - sizeof(pkt.crc32));

	memcpy(out, &pkt, sizeof(pkt));
	return (int)sizeof(pkt);
}

int rocketlog_telemetry_packet_decode_v1(telemetry_sample_t *out, const uint8_t *packet, size_t packet_len) {
	if (out == NULL || packet == NULL) {
		return -1;
	}
	if (packet_len != ROCKETLOG_TELEMETRY_PACKET_V1_LEN) {
		return -1;
	}

	rocketlog_telemetry_packet_v1_t pkt;
	memcpy(&pkt, packet, sizeof(pkt));

	if (pkt.magic[0] != ROCKETLOG_PACKET_MAGIC0 || pkt.magic[1] != ROCKETLOG_PACKET_MAGIC1) {
		return -1;
	}
	if (pkt.version != ROCKETLOG_PACKET_VERSION) {
		return -1;
	}
	if (pkt.msg_type != ROCKETLOG_MSG_TELEMETRY) {
		return -1;
	}

	const uint32_t expected_crc = rocketlog_crc32((const uint8_t *)&pkt, sizeof(pkt) - sizeof(pkt.crc32));
	if (expected_crc != pkt.crc32) {
		return -1;
	}

	out->unix_time = (double)pkt.unix_time_us / 1000000.0;
	out->altitude = (float)((double)pkt.altitude_cm / 100.0);
	out->velocity = (float)((double)pkt.velocity_cms / 100.0);
	out->battery = (float)((double)pkt.battery_mv / 1000.0);
	out->temperature = (float)((double)pkt.temperature_cC / 100.0);

	return 0;
}

/* ---------------------------------------- */
/*  v2 packet — adds GPS fields             */
/* ---------------------------------------- */

#pragma pack(push, 1)
typedef struct {
	uint8_t magic[2];
	uint8_t version;
	uint8_t msg_type;
	uint8_t payload_len;

	int64_t unix_time_us;
	int32_t altitude_cm;
	int32_t velocity_cms;
	uint16_t battery_mv;
	int16_t temperature_cC;

	int32_t latitude_1e7;
	int32_t longitude_1e7;
	int32_t gps_alt_cm;
	uint8_t gps_sats;
	uint8_t gps_fix;

	uint32_t crc32;
} rocketlog_telemetry_packet_v2_t;
#pragma pack(pop)

_Static_assert(sizeof(rocketlog_telemetry_packet_v2_t) == ROCKETLOG_TELEMETRY_PACKET_V2_LEN,
			   "rocketlog_telemetry_packet_v2_t size mismatch");

int rocketlog_telemetry_packet_encode_v2(uint8_t *out, size_t out_len, const telemetry_sample_t *sample) {
	if (out == NULL || sample == NULL) {
		return -1;
	}
	if (out_len < ROCKETLOG_TELEMETRY_PACKET_V2_LEN) {
		return -1;
	}

	rocketlog_telemetry_packet_v2_t pkt = {
		.magic = {ROCKETLOG_PACKET_MAGIC0, ROCKETLOG_PACKET_MAGIC1},
		.version = ROCKETLOG_PACKET_VERSION,
		.msg_type = ROCKETLOG_MSG_TELEMETRY,
		.payload_len = ROCKETLOG_TELEMETRY_PACKET_V2_PAYLOAD_LEN,
	};

	const double unix_time_us_f = sample->unix_time * 1000000.0;
	pkt.unix_time_us = (int64_t)llround(unix_time_us_f);

	pkt.altitude_cm = (int32_t)llround((double)sample->altitude * 100.0);
	pkt.velocity_cms = (int32_t)llround((double)sample->velocity * 100.0);

	const double batt_mv_f = (double)sample->battery * 1000.0;
	if (batt_mv_f < 0.0) {
		pkt.battery_mv = 0;
	} else if (batt_mv_f > 65535.0) {
		pkt.battery_mv = 65535;
	} else {
		pkt.battery_mv = (uint16_t)llround(batt_mv_f);
	}

	pkt.temperature_cC = (int16_t)llround((double)sample->temperature * 100.0);

	pkt.latitude_1e7 = (int32_t)llround(sample->gps_latitude * 1e7);
	pkt.longitude_1e7 = (int32_t)llround(sample->gps_longitude * 1e7);
	pkt.gps_alt_cm = (int32_t)llround((double)sample->gps_altitude * 100.0);
	pkt.gps_sats = sample->gps_satellites;
	pkt.gps_fix = sample->gps_valid ? 1 : 0;

	pkt.crc32 = rocketlog_crc32((const uint8_t *)&pkt, sizeof(pkt) - sizeof(pkt.crc32));

	memcpy(out, &pkt, sizeof(pkt));
	return (int)sizeof(pkt);
}

int rocketlog_telemetry_packet_decode_v2(telemetry_sample_t *out, const uint8_t *packet, size_t packet_len) {
	if (out == NULL || packet == NULL) {
		return -1;
	}
	if (packet_len != ROCKETLOG_TELEMETRY_PACKET_V2_LEN) {
		return -1;
	}

	rocketlog_telemetry_packet_v2_t pkt;
	memcpy(&pkt, packet, sizeof(pkt));

	if (pkt.magic[0] != ROCKETLOG_PACKET_MAGIC0 || pkt.magic[1] != ROCKETLOG_PACKET_MAGIC1) {
		return -1;
	}
	if (pkt.version != ROCKETLOG_PACKET_VERSION) {
		return -1;
	}
	if (pkt.msg_type != ROCKETLOG_MSG_TELEMETRY) {
		return -1;
	}
	if (pkt.payload_len != ROCKETLOG_TELEMETRY_PACKET_V2_PAYLOAD_LEN) {
		return -1;
	}

	const uint32_t expected_crc = rocketlog_crc32((const uint8_t *)&pkt, sizeof(pkt) - sizeof(pkt.crc32));
	if (expected_crc != pkt.crc32) {
		return -1;
	}

	out->unix_time = (double)pkt.unix_time_us / 1000000.0;
	out->altitude = (float)((double)pkt.altitude_cm / 100.0);
	out->velocity = (float)((double)pkt.velocity_cms / 100.0);
	out->battery = (float)((double)pkt.battery_mv / 1000.0);
	out->temperature = (float)((double)pkt.temperature_cC / 100.0);

	out->gps_latitude = (double)pkt.latitude_1e7 / 1e7;
	out->gps_longitude = (double)pkt.longitude_1e7 / 1e7;
	out->gps_altitude = (float)((double)pkt.gps_alt_cm / 100.0);
	out->gps_satellites = pkt.gps_sats;
	out->gps_valid = (pkt.gps_fix != 0);

	return 0;
}
