idf_component_register(
    SRCS
        "main.cpp"
        "wifi.c"
        "ranging.cpp"
        "led.c"
    INCLUDE_DIRS "."
    REQUIRES DW1000Ng esp_wifi esp_netif esp_event nvs_flash lwip esp_driver_rmt esp_hw_support
)
