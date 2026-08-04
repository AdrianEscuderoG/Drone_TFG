# main/CMakeLists.txt
idf_component_register(
    SRCS "main.c"
    INCLUDE_DIRS "."
    REQUIRES
        nvs_flash
        esp_driver_spi
        esp_driver_gpio
        esp_timer
        freertos
        esp32-camera
)