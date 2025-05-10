idf_component_register(
    SRCS "main.c"
    INCLUDE_DIRS "." "../include"
    REQUIRES driver
    PRIV_REQUIRES i2c_bus bme280 ec11 esp_netif esp_http_server spiffs wifi_station ultrasonic
    
)
idf_build_set_property(COMPILE_OPTIONS "-DLV_CONF_INCLUDE_SIMPLE=1" APPEND)
idf_build_set_property(COMPILE_OPTIONS "-I../include" APPEND)