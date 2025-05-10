# Component: BME280

The BME280 is as combined digital humidity, pressure and temperature sensor based on proven sensing principles. The sensor module is housed in an extremely compact metal-lid LGA package with a footprint of only 2.5 × 2.5 mm² with a height of 0.93 mm. Its small dimensions and its low power consumption allow the implementation in battery driven devices such as handsets, GPS modules or watches.

## Add component to your project

Add the following to your top-level CMakeLists.txt:

`set(EXTRA_COMPONENT_DIRS ${CMAKE_CURRENT_SOURCE_DIR}/components)`

Then add `bme280` as a required idf_component

## Example of BME280 usage

Pin assignment:

* master:
  * GPIO2 is assigned as the clock signal of i2c master port
  * GPIO1 is assigned as the data signal of i2c master port
* Connection:
  * connect SDA of sensor with GPIO1
  * connect SCL of sensor with GPIO2


```c
static i2c_master_bus_handle_t i2c_bus = NULL;
static bme280_handle_t bme280 = NULL;

//Step1: Init I2C bus
i2c_master_bus_config_t bus_config = {
    .clk_source = I2C_CLK_SRC_DEFAULT,
    .i2c_port = I2C_HOST,
    .sda_io_num = I2C_MASTER_SDA_IO,
    .scl_io_num = I2C_MASTER_SCL_IO,
    .glitch_ignore_cnt = 7,
    .flags.enable_internal_pullup = true,
};
i2c_new_master_bus(&bus_config, &i2c_bus);

//Step2: Init bme280
bme280_create(i2c_bus, &bme280, BME280_I2C_ADDRESS_DEFAULT);
bme280_default_init(bme280);

//Step3: Read temperature, humidity and pressure
float temperature = 0.0, humidity = 0.0, pressure = 0.0;
bme280_read_temperature(bme280, &temperature);
bme280_read_humidity(bme280, &humidity);
bme280_read_pressure(bme280, &pressure);
```