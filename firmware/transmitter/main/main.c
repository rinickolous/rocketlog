#include <stdio.h>

#include "telemetry_packet.h"
#include "telemetry_protocol.h"

/* ---------------------------------------- */
/*  Application                             */
/* ---------------------------------------- */

void app_main(void) {
	telemetry_sample_t sample = {
		.unix_time = 0.0,
		.altitude = 0.0f,
		.velocity = 0.0f,
		.battery = 0.0f,
		.temperature = 0.0f,
	};

	char line[96];
	if (telemetry_csv_encode(line, sizeof(line), &sample) > 0) {
		fputs(line, stdout);
	}

	// Demonstrate shared binary radio packet format.
	uint8_t packet[ROCKETLOG_TELEMETRY_PACKET_V1_LEN];
	const int pkt_len = rocketlog_telemetry_packet_encode_v1(packet, sizeof(packet), &sample);
	if (pkt_len > 0) {
		printf("pkt_v1_len=%d\n", pkt_len);
	}
}
