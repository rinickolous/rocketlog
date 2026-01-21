idf_component_register(
    SRCS
        "src/sensor_mpl3115a2.c"
        "src/sensor_gps.c"
        "src/sensor_manager.c"
    INCLUDE_DIRS "include"
    REQUIRES driver common
)