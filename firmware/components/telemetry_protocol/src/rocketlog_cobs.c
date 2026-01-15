#include "telemetry_packet.h"

#include <string.h>

// Minimal COBS implementation.
// Returns encoded length, or 0 on error.
size_t rocketlog_cobs_encode(uint8_t *out, size_t out_len, const uint8_t *in, size_t in_len) {
	if (out == NULL || in == NULL) {
		return 0;
	}
	if (out_len == 0) {
		return 0;
	}

	// COBS worst-case overhead is + (in_len / 254) + 1.
	// We don't pre-check precisely here; return 0 if we run out.
	size_t read_index = 0;
	size_t write_index = 1;
	size_t code_index = 0;
	uint8_t code = 1;

	while (read_index < in_len) {
		if (in[read_index] == 0) {
			if (code_index >= out_len) {
				return 0;
			}
			out[code_index] = code;
			code_index = write_index;
			write_index++;
			if (write_index > out_len) {
				return 0;
			}
			code = 1;
			read_index++;
			continue;
		}

		out[write_index] = in[read_index];
		read_index++;
		write_index++;
		if (write_index > out_len) {
			return 0;
		}
		code++;

		if (code == 0xFF) {
			if (code_index >= out_len) {
				return 0;
			}
			out[code_index] = code;
			code_index = write_index;
			write_index++;
			if (write_index > out_len) {
				return 0;
			}
			code = 1;
		}
	}

	if (code_index >= out_len) {
		return 0;
	}
	out[code_index] = code;
	return write_index;
}

// Returns decoded length, or 0 on error.
size_t rocketlog_cobs_decode(uint8_t *out, size_t out_len, const uint8_t *in, size_t in_len) {
	if (out == NULL || in == NULL) {
		return 0;
	}
	if (in_len == 0) {
		return 0;
	}

	size_t read_index = 0;
	size_t write_index = 0;
	while (read_index < in_len) {
		const uint8_t code = in[read_index];
		if (code == 0) {
			return 0;
		}
		read_index++;

		const size_t copy_len = (size_t)code - 1;
		if (read_index + copy_len > in_len) {
			return 0;
		}
		if (write_index + copy_len > out_len) {
			return 0;
		}
		if (copy_len > 0) {
			memcpy(&out[write_index], &in[read_index], copy_len);
			read_index += copy_len;
			write_index += copy_len;
		}

		if (code != 0xFF && read_index < in_len) {
			if (write_index >= out_len) {
				return 0;
			}
			out[write_index] = 0;
			write_index++;
		}
	}

	return write_index;
}
