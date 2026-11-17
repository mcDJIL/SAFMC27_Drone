idf_component_register(
    SRCS "src/easy_gpio.c"
         "src/easy_gpio_irq.c"
         "src/easy_gpio_analog.c"
    INCLUDE_DIRS "include"
    REQUIRES esp_driver_gpio
    PRIV_REQUIRES esp_driver_ledc esp_adc esp_timer esp_rom freertos log
)