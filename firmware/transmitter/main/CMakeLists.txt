idf_component_register(
		SRCS "telemetry_transmitter.c"
		INCLUDE_DIRS "."
		PRIV_REQUIRES log driver led_strip common telemetry_protocol
)
