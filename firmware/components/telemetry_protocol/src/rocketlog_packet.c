#include "telemetry_packet.h"

#include <stdint.h>
#include <string.h>

#include "esp_crc.h"

#pragma pack(push, 1)
typedef struct {
	uint8_t magic[2];
	uint8_t version;
	uint8_t msg_type;
	uint8_t payload_len;
} rocketlog_packet_header_v1_t;
#pragma pack(pop)

_Static_assert(sizeof(rocketlog_packet_header_v1_t) == 5, "rocketlog_packet_header_v1_t size mismatch");

static uint32_t rocketlog_crc32(const uint8_t *data, size_t len) {
	return esp_crc32_le(0, data, len);
}

int rocketlog_packet_encode_v1(uint8_t *out, size_t out_len, uint8_t msg_type, const uint8_t *payload,
							   size_t payload_len) {
	if (out == NULL) {
		return -1;
	}
	if (payload_len > 255) {
		return -1;
	}

	const size_t total_len = sizeof(rocketlog_packet_header_v1_t) + payload_len + sizeof(uint32_t);
	if (out_len < total_len) {
		return -1;
	}

	rocketlog_packet_header_v1_t hdr = {
		.magic = {ROCKETLOG_PACKET_MAGIC0, ROCKETLOG_PACKET_MAGIC1},
		.version = ROCKETLOG_PACKET_VERSION,
		.msg_type = msg_type,
		.payload_len = (uint8_t)payload_len,
	};

	memcpy(out, &hdr, sizeof(hdr));
	if (payload_len > 0) {
		if (payload == NULL) {
			return -1;
		}
		memcpy(out + sizeof(hdr), payload, payload_len);
	}

	uint32_t crc = rocketlog_crc32(out, sizeof(hdr) + payload_len);
	memcpy(out + sizeof(hdr) + payload_len, &crc, sizeof(crc));
	return (int)total_len;
}

int rocketlog_packet_decode_v1(uint8_t *msg_type_out, const uint8_t **payload_out, size_t *payload_len_out,
							   const uint8_t *packet, size_t packet_len) {
	if (msg_type_out == NULL || payload_out == NULL || payload_len_out == NULL) {
		return -1;
	}
	if (packet == NULL) {
		return -1;
	}
	if (packet_len < sizeof(rocketlog_packet_header_v1_t) + sizeof(uint32_t)) {
		return -1;
	}

	rocketlog_packet_header_v1_t hdr;
	memcpy(&hdr, packet, sizeof(hdr));
	if (hdr.magic[0] != ROCKETLOG_PACKET_MAGIC0 || hdr.magic[1] != ROCKETLOG_PACKET_MAGIC1) {
		return -1;
	}
	if (hdr.version != ROCKETLOG_PACKET_VERSION) {
		return -1;
	}

	const size_t expected_len = sizeof(hdr) + (size_t)hdr.payload_len + sizeof(uint32_t);
	if (expected_len != packet_len) {
		return -1;
	}

	uint32_t got_crc;
	memcpy(&got_crc, packet + sizeof(hdr) + (size_t)hdr.payload_len, sizeof(got_crc));
	const uint32_t expected_crc = rocketlog_crc32(packet, sizeof(hdr) + (size_t)hdr.payload_len);
	if (got_crc != expected_crc) {
		return -1;
	}

	*msg_type_out = hdr.msg_type;
	*payload_out = packet + sizeof(hdr);
	*payload_len_out = (size_t)hdr.payload_len;
	return 0;
}

int rocketlog_log_packet_encode_v1(uint8_t *out, size_t out_len, rocketlog_log_level_t level, int64_t unix_time_us,
								   const char *msg_utf8) {
	if (out == NULL || msg_utf8 == NULL) {
		return -1;
	}

	// payload = level + unix_time_us + msg bytes (no NUL required)
	const size_t msg_len = strlen(msg_utf8);
	if (msg_len > 240) {
		// avoid huge packets on serial; truncate at caller if desired
		return -1;
	}

	uint8_t payload[1 + 8 + 240];
	payload[0] = (uint8_t)level;
	memcpy(&payload[1], &unix_time_us, sizeof(unix_time_us));
	memcpy(&payload[1 + sizeof(unix_time_us)], msg_utf8, msg_len);

	return rocketlog_packet_encode_v1(out, out_len, ROCKETLOG_MSG_LOG, payload, 1 + sizeof(unix_time_us) + msg_len);
}

int rocketlog_time_sync_packet_encode_v1(uint8_t *out, size_t out_len, int64_t unix_time_us, uint32_t seq) {
	if (out == NULL) {
		return -1;
	}
	uint8_t payload[8 + 4];
	memcpy(&payload[0], &unix_time_us, sizeof(unix_time_us));
	memcpy(&payload[8], &seq, sizeof(seq));
	return rocketlog_packet_encode_v1(out, out_len, ROCKETLOG_MSG_TIME_SYNC, payload, sizeof(payload));
}

int rocketlog_ack_packet_encode_v1(uint8_t *out, size_t out_len, uint8_t ack_type, uint32_t seq, uint8_t status,
								   int64_t applied_unix_time_us) {
	if (out == NULL) {
		return -1;
	}
	uint8_t payload[1 + 4 + 1 + 8];
	payload[0] = ack_type;
	memcpy(&payload[1], &seq, sizeof(seq));
	payload[1 + 4] = status;
	memcpy(&payload[1 + 4 + 1], &applied_unix_time_us, sizeof(applied_unix_time_us));
	return rocketlog_packet_encode_v1(out, out_len, ROCKETLOG_MSG_ACK, payload, sizeof(payload));
}
