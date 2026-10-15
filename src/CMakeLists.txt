# ESP-IDF first reads components in a script-only requirements pass.
# Generate assets only in the real configure pass.
if(NOT CMAKE_BUILD_EARLY_EXPANSION)
# Keep compression in the build: no runtime heap/compression cost on ESP32.
set(DASHBOARD_HEADER "${CMAKE_CURRENT_BINARY_DIR}/generated/dashboard_asset.h")
set_property(DIRECTORY APPEND PROPERTY CMAKE_CONFIGURE_DEPENDS
    "${CMAKE_CURRENT_SOURCE_DIR}/web/index.html"
    "${CMAKE_CURRENT_SOURCE_DIR}/../tools/embed_dashboard.py")
execute_process(
    COMMAND "${PYTHON}" "${CMAKE_CURRENT_SOURCE_DIR}/../tools/embed_dashboard.py"
        "${CMAKE_CURRENT_SOURCE_DIR}/web/index.html" "${DASHBOARD_HEADER}"
    RESULT_VARIABLE DASHBOARD_RESULT
)
if(NOT DASHBOARD_RESULT EQUAL 0)
    message(FATAL_ERROR "Dashboard asset generation failed")
endif()
endif()

idf_component_register(
    SRCS
        "main.c"
        "diag.c"
        "navigation.c"
        "route_runner.c"
        "slam.c"
        "imu.c"
        "mapper.c"
        "robot_manager.c"
        "network.c"
        "web_server.c"
    INCLUDE_DIRS
        "."
        "../include"
    REQUIRES
        seeker_config
        track_drive
        ld20_lidar
        esp_timer
        esp_driver_i2c
        esp_wifi
        esp_netif
        esp_event
        esp_http_server
        esp_system
        nvs_flash
        app_update
        espressif__cjson
)

if(NOT CMAKE_BUILD_EARLY_EXPANSION)
    target_include_directories(${COMPONENT_LIB} PRIVATE "${CMAKE_CURRENT_BINARY_DIR}/generated")
endif()
