idf_component_register(
    SRCS "main.c"
    INCLUDE_DIRS "."
    REQUIRES
        nvs_flash
        esp_wifi
        esp_netif
        lwip
        esp_driver_uart
        esp_event
        mavlink
)

target_compile_options(__idf_main PRIVATE
    -Wno-address-of-packed-member
)