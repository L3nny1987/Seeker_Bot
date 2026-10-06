idf_component_register(
    SRCS "track_drive.c"
    INCLUDE_DIRS "include"
    REQUIRES seeker_config esp_driver_gpio esp_driver_ledc
)
