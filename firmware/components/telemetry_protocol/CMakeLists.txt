idf_component_register(
    SRCS
        "src/telemetry_protocol.c"
        "src/telemetry_packet.c"
        "src/rocketlog_packet.c"
        "src/rocketlog_cobs.c"
    INCLUDE_DIRS "include"
    REQUIRES esp_hw_support
)
