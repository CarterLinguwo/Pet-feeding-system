idf_component_register(
    SRCS bme280.c
    INCLUDE_DIRS .
    REQUIRES log esp_driver_i2c
)