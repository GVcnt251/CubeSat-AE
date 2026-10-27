idf_component_register(
    SRCS app_main.c imu.c camera.c wheel.c can_mcp2518fd.c comms.c battery.c
    INCLUDE_DIRS "."
    REQUIRES cube_core driver esp_timer esp_wifi esp_event esp_netif nvs_flash lwip esp_adc)
