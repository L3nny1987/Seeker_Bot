idf_component_register(
    SRCS "ld20_lidar.c" "ld20_scan_builder.c"
    INCLUDE_DIRS "include"
    REQUIRES seeker_config esp_driver_uart esp_driver_gpio esp_timer
)
