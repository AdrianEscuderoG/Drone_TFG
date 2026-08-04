idf_component_register(
    SRCS "main.c"
    INCLUDE_DIRS "."
    REQUIRES
        nvs_flash
        esp_wifi
        esp_event
        esp_netif
        esp_driver_uart
        esp_driver_spi
        esp_driver_gpio
        lwip
        mavlink
)

target_compile_options(__idf_main PRIVATE
    -Wno-address-of-packed-member
)