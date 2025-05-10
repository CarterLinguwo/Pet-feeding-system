#include <stdio.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"
#include "bme280.h"
#include "esp_timer.h"
#include "esp_lcd_panel_io.h"
#include "esp_lcd_panel_ops.h"
#include "driver/i2c_master.h"
#include "lvgl.h"
#include "esp_lvgl_port.h"
#include "lv_conf.h"
#include "esp_lcd_panel_vendor.h"
#include "iot_button.h"
#include "button_gpio.h"
#include "knob.h"
#include "wifi_station.h"
#include "esp_netif_sntp.h"
#include "esp_sntp.h"
#include "esp_spiffs.h"
#include "esp_http_server.h"
#include <stdint.h>
#include <sys/stat.h>
#include <time.h>
#include <sys/time.h>
#include "driver/pulse_cnt.h"
#include "bdc_motor.h"
#include "pid_ctrl.h"
#include "esp_adc/adc_oneshot.h"
#include "led_indicator.h"
#include "ultrasonic.h"

static const char *TAG = "Final Project";

static void update_reading(float pres, float temp, float hum);

/* I2C bus */
#define I2C_HOST                       0                                // I2C controller number
#define I2C_MASTER_SDA_IO              1                                // GPIO number for I2C master data
#define I2C_MASTER_SCL_IO              2                                // GPIO number for I2C master clock
#define I2C_MASTER_FREQ_HZ             400000                           // I2C master clock frequency
static i2c_master_bus_handle_t i2c_bus = NULL;                          // I2C bus master

/* OLED Display */
#define SSD1306_I2C_ADDR               0x3C                             // Default I2C address of display
#define SSD1306_PIN_NUM_RST            -1                               // No reset pin
// The pixel number in horizontal and vertical
#define LCD_H_RES                     128
#define LCD_V_RES                     64
// Bit number used to represent command and parameter
#define LCD_CMD_BITS                  8
#define LCD_PARAM_BITS                8

/* Motor */
#define BDC_MCPWM_TIMER_RESOLUTION_HZ 10000000                          // 10MHz, 1 tick = 0.1us
#define BDC_MCPWM_FREQ_HZ             25000                             // 25KHz PWM
#define BDC_MCPWM_DUTY_TICK_MAX       (BDC_MCPWM_TIMER_RESOLUTION_HZ / BDC_MCPWM_FREQ_HZ) // maximum value we can set for the duty cycle, in ticks
#define BDC_MCPWM_GPIO_A              7
#define BDC_MCPWM_GPIO_B              15
#define BDC_ENCODER_GPIO_A            36
#define BDC_ENCODER_GPIO_B            35
#define BDC_ENCODER_PCNT_HIGH_LIMIT   1000
#define BDC_ENCODER_PCNT_LOW_LIMIT    -1000
#define BDC_PID_LOOP_PERIOD_MS        10                                // calculate the motor speed every 10ms

/* Potentiometer */
#define EXAMPLE_ADC1_CHAN3            ADC_CHANNEL_3
#define EXAMPLE_ADC_ATTEN             ADC_ATTEN_DB_12
#define Portion_MIN                     1                               // set the min speed in the pulses counted by the encoder
#define Portion_MAX                     10                              // set the max speed in the pulses counted by the encoder

/* Ultrasonic */
#define MAX_DISTANCE_CM 10
#define TRIGGER_GPIO 11
#define ECHO_GPIO 10

/* LED */
#define GPIO_WS2812                 GPIO_NUM_48                        // built-in addressable LED
#define STRIPS_NUM_WS2812           1                                  // only one LED
#define LED_STRIP_RMT_RES_HZ        (10 * 1000 * 1000)           

/* Feeding Schedule */
static const uint8_t    feeding_hours[] = { 5, 6, 7, 8 };
static const size_t     feeding_count = sizeof(feeding_hours) / sizeof(feeding_hours[0]);

/* GLOBAL VARIABLES */
float pressure;                                                         // Pressure from BME280
float temperature;                                                      // Temperature from BME280
float humidity;                                                         // Humidity from BME280
static int portion_size;
static time_t next_feed_time = 0;
static adc_oneshot_unit_handle_t adc1_handle = NULL;
static led_indicator_handle_t led_handle = NULL;                        // indicator LED handle
static knob_handle_t knob_handle = NULL;                                // knob handle

// I2C Initialize
void i2c_init() 
{
    i2c_master_bus_config_t bus_config = {
        .clk_source = I2C_CLK_SRC_DEFAULT,
        .i2c_port = I2C_HOST,
        .sda_io_num = I2C_MASTER_SDA_IO,
        .scl_io_num = I2C_MASTER_SCL_IO,
        .glitch_ignore_cnt = 7,
        .flags.enable_internal_pullup = true,
    };
    ESP_ERROR_CHECK(i2c_new_master_bus(&bus_config, &i2c_bus));
    ESP_LOGI(TAG, "New i2c master bus created");
}


// Map function
long Map(long x, long in_min, long in_max, long out_min, long out_max)
{
    return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
}

// ADC reading by potentiometer
void adc_init() {
    //-------------ADC1 Init---------------//
    adc_oneshot_unit_init_cfg_t init_config1 = {
        .unit_id = ADC_UNIT_1,
    };
    ESP_ERROR_CHECK(adc_oneshot_new_unit(&init_config1, &adc1_handle));

    //-------------ADC1 Config---------------//
    adc_oneshot_chan_cfg_t config = {
        .atten = EXAMPLE_ADC_ATTEN,
        .bitwidth = ADC_BITWIDTH_DEFAULT,
    };
    ESP_ERROR_CHECK(adc_oneshot_config_channel(adc1_handle, EXAMPLE_ADC1_CHAN3, &config));
}

// Portion from 1-10 by potentiometer
static void portion_task(void *pvParameters)
{
    int val = 0;
    while (1) {
        ESP_ERROR_CHECK(adc_oneshot_read(adc1_handle, EXAMPLE_ADC1_CHAN3, &val));
        portion_size = Map(val, 0, 4095, Portion_MIN, Portion_MAX) * 3;
        vTaskDelay(100 / portTICK_PERIOD_MS);
    }
}

      
// LED Indicator modes
enum {
    BLINK_GREEN = 0,
    BLINK_WHITE,
    BLINK_MAX,
} indicator_t;
static const blink_step_t green[] = {
    {LED_BLINK_RGB, SET_RGB(0, 255, 0), 0},                             // Pure green
    {LED_BLINK_BRIGHTNESS, 15, 0},
    {LED_BLINK_HOLD, LED_STATE_ON, 1000},                              
    {LED_BLINK_STOP, 0, 0},
};
static const blink_step_t white[] = {
    {LED_BLINK_RGB, SET_RGB(255, 255, 255), 0},                         // White color
    {LED_BLINK_BRIGHTNESS, 15, 0},
    {LED_BLINK_HOLD, LED_STATE_ON, 1000},                               
    {LED_BLINK_STOP, 0, 0},
};
blink_step_t const *led_mode[] = {
    [BLINK_GREEN] = green,
    [BLINK_WHITE] = white,
    [BLINK_MAX] = NULL,
};

// LED Indicator initialize
static void indicator_init(void)
{
    led_strip_config_t strip_config = {
        .strip_gpio_num = GPIO_WS2812,                                  // GPIO for LED strip data line
        .max_leds = STRIPS_NUM_WS2812,                                  // number of LEDs in strip
        .led_pixel_format = LED_PIXEL_FORMAT_GRB,                       // pixel format of LED strip
        .led_model = LED_MODEL_WS2812,                                  // LED strip model
        .flags.invert_out = false,                                      // whether to invert the output signal
    };

    // LED strip backend configuration: RMT
    led_strip_rmt_config_t rmt_config = {
        .clk_src = RMT_CLK_SRC_DEFAULT,                                 // different clock source can lead to different power consumption
        .resolution_hz = LED_STRIP_RMT_RES_HZ,                          // RMT counter clock frequency
        .flags.with_dma = false,                                        // DMA feature is available on ESP target like ESP32-S3
    };

    led_indicator_strips_config_t strips_config = {
        .led_strip_cfg = strip_config,
        .led_strip_driver = LED_STRIP_RMT,
        .led_strip_rmt_cfg = rmt_config,
    };

    const led_indicator_config_t config = {
        .mode = LED_STRIPS_MODE,
        .led_indicator_strips_config = &strips_config,
        .blink_lists = led_mode,
        .blink_list_num = BLINK_MAX,
    };

    led_handle = led_indicator_create(&config);
    if (led_handle == NULL) {
        ESP_LOGE(TAG, "Failed to create LED indicator");
    }
}



// Ultrasonic sensor reading
void ultrasonic_test(void *pvParameters)
{
    ultrasonic_sensor_t sensor = {
        .trigger_pin = TRIGGER_GPIO,
        .echo_pin = ECHO_GPIO
    };

    ultrasonic_init(&sensor);

    while (1){
        float distance;
        esp_err_t res = ultrasonic_measure(&sensor, MAX_DISTANCE_CM, &distance);
        if (res != ESP_OK)
        {
            printf("Error %d: ", res);
            switch (res)
            {
                case ESP_ERR_ULTRASONIC_PING:
                    printf("Cannot ping (device is in invalid state)\n");
                    break;
                case ESP_ERR_ULTRASONIC_PING_TIMEOUT:
                    printf("Ping timeout (no device found)\n");
                    break;
                case ESP_ERR_ULTRASONIC_ECHO_TIMEOUT:
                    printf("Echo timeout (i.e. distance too big)\n");
                    break;
                default:
                    printf("%s\n", esp_err_to_name(res));
            }
        }
        else
            printf("Distance: %0.04f cm\n", distance*100);

        vTaskDelay(pdMS_TO_TICKS(500));
    }
}



/*WIFI Station & SNTP*/
#define EXAMPLE_UPDATE_INTERVAL  CONFIG_UPDATE_INTERVAL
#define EXAMPLE_SYNC_TIME  CONFIG_SYNC_TIME

#ifdef CONFIG_SYNC_TIME
#define EXAMPLE_RESYNC_INTERVAL  CONFIG_SNTP_RESYNC_INTERVAL
#define EXAMPLE_SNTP_NUM_SERVERS CONFIG_SNTP_NUM_SERVERS
#define EXAMPLE_SNTP_SERVER1     CONFIG_SNTP_TIME_SERVER1
#define EXAMPLE_SNTP_SERVER2     CONFIG_SNTP_TIME_SERVER2
#define EXAMPLE_SNTP_SERVER3     CONFIG_SNTP_TIME_SERVER3
#endif

#ifndef INET6_ADDRSTRLEN
#define INET6_ADDRSTRLEN 48
#endif

#define INDEX_HTML_PATH "/spiffs/index.html"
static char index_html[16384];

static httpd_handle_t server = NULL;
static int ws_fd = -1;

#ifdef CONFIG_SYNC_TIME
static bool sntp_initialized = false;

void time_sync_notification_cb(struct timeval *tv)
{
    ESP_LOGI(TAG, "Time synchronized to server");
    sntp_initialized = true;
}

static void print_servers(void)
{
    ESP_LOGI(TAG, "List of configured NTP servers:");

    for (uint8_t i = 0; i < EXAMPLE_SNTP_NUM_SERVERS; ++i){
        if (esp_sntp_getservername(i)){
            ESP_LOGI(TAG, "server %d: %s", i, esp_sntp_getservername(i));
        } else {
            // we have either IPv4 or IPv6 address, let's print it
            char buff[INET6_ADDRSTRLEN];
            ip_addr_t const *ip = esp_sntp_getserver(i);
            if (ipaddr_ntoa_r(ip, buff, INET6_ADDRSTRLEN) != NULL)
                ESP_LOGI(TAG, "server %d: %s", i, buff);
        }
    }
}

// Obtain current time from STNP server
static void obtain_time(void)
{
    ESP_LOGI(TAG, "Initializing and starting SNTP");
#if EXAMPLE_SNTP_NUM_SERVERS == 2
    esp_sntp_config_t config = ESP_NETIF_SNTP_DEFAULT_CONFIG_MULTIPLE(2,
                               ESP_SNTP_SERVER_LIST(EXAMPLE_SNTP_SERVER1, EXAMPLE_SNTP_SERVER2));
#elif EXAMPLE_SNTP_NUM_SERVERS == 3
    esp_sntp_config_t config = ESP_NETIF_SNTP_DEFAULT_CONFIG_MULTIPLE(3,
                               ESP_SNTP_SERVER_LIST(EXAMPLE_SNTP_SERVER1, EXAMPLE_SNTP_SERVER2, 
                                EXAMPLE_SNTP_SERVER3));
#else
    /*
     * This is the basic default config with one server and starting the service
     */
    esp_sntp_config_t config = ESP_NETIF_SNTP_DEFAULT_CONFIG(EXAMPLE_SNTP_SERVER1);
#endif
    config.sync_cb = time_sync_notification_cb;     // Note: This is only needed if we want
#ifdef CONFIG_SNTP_TIME_SYNC_METHOD_SMOOTH
    config.smooth_sync = true;
#endif
    esp_netif_sntp_init(&config);
    print_servers();

    int retry = 0;
    const int retry_count = 20;
    while (esp_netif_sntp_sync_wait(2000 / portTICK_PERIOD_MS) == ESP_ERR_TIMEOUT && ++retry < retry_count) {
        ESP_LOGI(TAG, "Waiting for system time to be set... (%d/%d)", retry, retry_count);
    }

    esp_netif_sntp_deinit(); 
}
#endif

static void station_task(void *pvParameters)
{
    esp_netif_t *sta = (esp_netif_t *) pvParameters;
#ifdef CONFIG_SYNC_TIME
    time_t now = 0;
    struct tm timeinfo = { 0 };
    time_t last_resync = 0;
    char strftime_buf[64];
#endif
    esp_netif_ip_info_t ip;
    memset(&ip, 0, sizeof(esp_netif_ip_info_t));

    while (1) {
        vTaskDelay(EXAMPLE_UPDATE_INTERVAL * 1000 / portTICK_PERIOD_MS);

        if (esp_netif_get_ip_info(sta, &ip) == 0) {
            ESP_LOGI(TAG, "~~~~~~~~~~~");
            ESP_LOGI(TAG, "IP:"IPSTR, IP2STR(&ip.ip));
            ESP_LOGI(TAG, "MASK:"IPSTR, IP2STR(&ip.netmask));
            ESP_LOGI(TAG, "GW:"IPSTR, IP2STR(&ip.gw));
#ifdef CONFIG_SYNC_TIME
            time(&now);
            localtime_r(&now, &timeinfo);
            // Is time set? If not, tm_year will be (1970 - 1900).
            if (timeinfo.tm_year < (2016 - 1900)) {
                ESP_LOGI(TAG, "Time is not set yet. Getting time over NTP.");
                obtain_time();
                // update 'now' variable with current time
                time(&now);
                last_resync = now;
            }

            // Resync time if after resync interval has passed
            if (difftime(now, last_resync) >= EXAMPLE_RESYNC_INTERVAL) {
                ESP_LOGI(TAG, "Resyncing time. Getting time over NTP.");
                obtain_time();
                // update 'now' variable with current time
                time(&now);
                last_resync = now;

                if (sntp_get_sync_mode() == SNTP_SYNC_MODE_SMOOTH) {
                    struct timeval outdelta;
                    while (sntp_get_sync_status() == SNTP_SYNC_STATUS_IN_PROGRESS) {
                        adjtime(NULL, &outdelta);
                        ESP_LOGI(TAG, "Waiting for time adjustment ... outdelta = %jd sec: %li ms: %li us",
                                    (intmax_t)outdelta.tv_sec,
                                    outdelta.tv_usec/1000,
                                    outdelta.tv_usec%1000);
                        vTaskDelay(1000 / portTICK_PERIOD_MS);
                    }
                }
            }
            // Set timezone to Eastern Standard Time and print local time
            setenv("TZ", "EST5EDT,M3.2.0/2,M11.1.0", 1);
            tzset();
            localtime_r(&now, &timeinfo);
            strftime(strftime_buf, sizeof(strftime_buf), "%c", &timeinfo);
            ESP_LOGI(TAG, "Current date/time in London, ON: %s", strftime_buf);
#endif
            ESP_LOGI(TAG, "~~~~~~~~~~~");
        }
        else {
            ESP_LOGI(TAG, "Not connected");
        }
    }
}

static void init_html(void)
{
    esp_vfs_spiffs_conf_t conf = {
        .base_path = "/spiffs",
        .partition_label = NULL,
        .max_files = 5,
        .format_if_mount_failed = true};

    ESP_ERROR_CHECK(esp_vfs_spiffs_register(&conf));

    memset((void *)index_html, 0, sizeof(index_html));
    struct stat st;
    if (stat(INDEX_HTML_PATH, &st))
    {
        ESP_LOGE(TAG, "index.html not found");
        return;
    }

    FILE *fp = fopen(INDEX_HTML_PATH, "r");
    if (fread(index_html, st.st_size, 1, fp) != st.st_size)
    {
        ESP_LOGE(TAG, "fread failed");
    }
    fclose(fp);
}



/*Rotary Encoder*/
#define GPIO_KNOB_BUTTON            GPIO_NUM_16                             // EC11 button
#define GPIO_EC11_A                 GPIO_NUM_18                             // EC11 encoder channel A
#define GPIO_EC11_B                 GPIO_NUM_17                             // EC11 encdoer channel B
#define KNOB_HIGH_LIMIT             100                                     // upper limit for knob count
#define KNOB_LOW_LIMIT              -100                                    // lower limit for knob count
static int knob_count = 1;                                                  // knob encoder count

typedef enum {
    BUTTON_KNOB = 0,
    BUTTON_NUM,
} button_t;
static button_handle_t btn_handle[BUTTON_NUM];                              // buttons handle

static const button_gpio_config_t button_config[BUTTON_NUM] = {
    {
        .gpio_num = GPIO_KNOB_BUTTON,                                       // EC11 button
        .active_level = 0,                                                  // active low
    }
};


// Initialized BME280 and take repeated readings of temperature, humidity, and pressure
void bme280_task(void *pvParameters) 
{
    static bme280_handle_t bme280 = NULL;
    esp_err_t res;

    if ((res = bme280_create(i2c_bus, &bme280, BME280_I2C_ADDRESS_DEFAULT)) != ESP_OK) {
        ESP_LOGI(TAG, "Could not add BME280 to I2C bus: %d (%s)", res, esp_err_to_name(res));
    }
    if ((res = bme280_default_init(bme280)) != ESP_OK) {
        ESP_LOGI(TAG, "Could not initialize BME280: %d (%s)", res, esp_err_to_name(res));
    }

    bme280_dev_t *sens = (bme280_dev_t *) bme280;

    bool bme280p = sens->chip_id == BME280_DEFAULT_CHIPID;
    ESP_LOGI(TAG, "BMP280: found %s", bme280p ? "BME280" : "BMP280");

    while (1) {
        vTaskDelay(pdMS_TO_TICKS(200));
        if ((res = bme280_read_temperature(bme280, &temperature)) != ESP_OK) {
            ESP_LOGI(TAG, "Could not read temp: %d (%s)", res, esp_err_to_name(res));
        }
        if ((res = bme280_read_pressure(bme280, &pressure)) != ESP_OK) {
            ESP_LOGI(TAG, "Could not read pressure: %d (%s)", res, esp_err_to_name(res));
        }
        if ((res = bme280_read_humidity(bme280, &humidity)) != ESP_OK) {
            ESP_LOGI(TAG, "Could not read humidity: %d (%s)", res, esp_err_to_name(res));
        }
        update_reading(pressure, temperature, humidity);

    }
}


// Compute the next feeding time
static time_t next_feeding_time(void)
{
    //Grab current SNTP-synced time
    time_t now = 0;
    struct tm timeinfo = { 0 };
    time(&now);
    localtime_r(&now, &timeinfo);

    time_t feed_ts[feeding_count];
    for (size_t i = 0; i < feeding_count; i++) {
        struct tm time_schedule = timeinfo;             // start with year/month/day of today
        time_schedule.tm_hour = feeding_hours[i];       // set feed hour
        time_schedule.tm_min  = 32;                     // set feed minutes
        time_schedule.tm_sec  = 0;                      // set feed seconds

        time_t ts = mktime(&time_schedule);
        if (ts <= now) {                                // if the time is already past, +1 day
            ts += 24 * 3600;
        }
        feed_ts[i] = ts;
    }

    time_t best_ts = INT64_MAX;
    for (size_t i = 0; i < feeding_count; i++) {
        if (feed_ts[i] > now && feed_ts[i] < best_ts) {
            best_ts = feed_ts[i];
        }
    }
    return best_ts;
}

// Send synced data to web page
static void send_async(void *arg)
{
    if (ws_fd < 0)
    {
        return;
    }

#ifdef CONFIG_SYNC_TIME    
    time_t now = 0;
    struct tm timeinfo = { 0 };
    time(&now);
    localtime_r(&now, &timeinfo);
#endif

    char buff[512];
    memset(buff, 0, sizeof(buff));
    // format JSON data
    time_t best_ts = next_feeding_time();
    struct tm next_feed;
    localtime_r(&best_ts, &next_feed);

#ifdef CONFIG_SYNC_TIME
    sprintf(buff, "{\"pres\": %.2f, \"temp\": %.2f, \"hum\": %.2f, \"next_feed\":\"%04d-%02d-%02d %02d:%02d\", \"portion\": %d, \"time\": \"%02d:%02d:%02d\"}",
            pressure, temperature, humidity, next_feed.tm_year + 1900, next_feed.tm_mon + 1, next_feed.tm_mday, next_feed.tm_hour, next_feed.tm_min, portion_size / 3,
            timeinfo.tm_hour, timeinfo.tm_min, timeinfo.tm_sec);
#else
//    sprintf(buff, "{\"pres\": %.2f, \"temp\": %.2f, \"hum\": %.2f, \"time\": \"00:00:00\"}",
    sprintf(buff, "{\"state\": \"%s\", \"pres\": %.2f, \"temp\": %.2f, \"hum\": %.2f}",
            pressure, temperature, humidity);
#endif

    httpd_ws_frame_t ws_pkt;
    memset(&ws_pkt, 0, sizeof(httpd_ws_frame_t));
    ws_pkt.payload = (uint8_t *)buff;
    ws_pkt.len = strlen(buff);
    ws_pkt.type = HTTPD_WS_TYPE_TEXT;

    httpd_ws_send_frame_async(server, ws_fd, &ws_pkt);
}

static esp_err_t handle_http_get(httpd_req_t *req)
{
    if (index_html[0] == 0)
    {
        httpd_resp_set_status(req, HTTPD_500);
        return httpd_resp_send(req, "no index.html", HTTPD_RESP_USE_STRLEN);
    }
    return httpd_resp_send(req, index_html, HTTPD_RESP_USE_STRLEN);
}

static esp_err_t handle_ws_req(httpd_req_t *req)
{

    httpd_ws_frame_t ws_pkt;
    uint8_t buff[16];
    memset(&ws_pkt, 0, sizeof(httpd_ws_frame_t));
    ws_pkt.payload = buff;
    ws_pkt.type = HTTPD_WS_TYPE_BINARY;

    httpd_ws_recv_frame(req, &ws_pkt, sizeof(buff));
    return ESP_OK;
}

static esp_err_t handle_socket_opened(httpd_handle_t hd, int sockfd)
{
    ws_fd = sockfd;
    return ESP_OK;
}

static void handle_socket_closed(httpd_handle_t hd, int sockfd)
{
    if (sockfd == ws_fd)
    {
        ws_fd = -1;
    }
}

//start server
static void start_server(void)
{
    httpd_config_t config = HTTPD_DEFAULT_CONFIG();
    config.open_fn = handle_socket_opened;
    config.close_fn = handle_socket_closed;

    if (httpd_start(&server, &config) == ESP_OK)
    {
        httpd_uri_t uri_get = {
            .uri = "/",
            .method = HTTP_GET,
            .handler = handle_http_get,
            .user_ctx = NULL};
        httpd_register_uri_handler(server, &uri_get);

        httpd_uri_t ws = {
            .uri = "/ws",
            .method = HTTP_GET,
            .handler = handle_ws_req,
            .user_ctx = NULL,
            .is_websocket = true};
        httpd_register_uri_handler(server, &ws);
    }
}

// update readings
static void update_reading(float pres, float temp, float hum)
{
    pressure    = pres;
    temperature = temp;
    humidity    = hum;

    if (server != NULL && ws_fd >= 0) {
        httpd_queue_work(server, send_async, NULL);
    }
}

// LVGL display and demo screens
static lv_disp_t *disp = NULL;                                         // display handle
static lv_obj_t *main_screen = NULL;                                   // main screen
static lv_obj_t *main_label = NULL;                                    // main label
static QueueHandle_t q_page_num;                                       // page queue
static uint8_t g_page_num = 0;                                         // selected page

// LVGL text styles
static lv_style_t banner_style;                                         // Top banner, inverted
static lv_style_t data_style;                                           // Normal text

// Display pages
typedef enum {
    PAGE_MENU1 = 0,
    PAGE_MENU2,
    PAGE_MENU3,
} page_t;

// Configure SSD1306 to use LVGL
static void display_init()
{
    // Configure SSD 1306 OLED display
    ESP_LOGI(TAG, "Install panel IO");
    esp_lcd_panel_io_handle_t io_handle = NULL;
    esp_lcd_panel_io_i2c_config_t io_config = {
        .dev_addr = SSD1306_I2C_ADDR,
        .scl_speed_hz = I2C_MASTER_FREQ_HZ,
        .control_phase_bytes = 1,                                    // According to SSD1306 datasheet
        .lcd_cmd_bits = LCD_CMD_BITS,                                // According to SSD1306 datasheet
        .lcd_param_bits = LCD_PARAM_BITS,                            // According to SSD1306 datasheet
        .dc_bit_offset = 6,                                          // According to SSD1306 datasheet
    };
    ESP_ERROR_CHECK(esp_lcd_new_panel_io_i2c(i2c_bus, &io_config, &io_handle));
    esp_lcd_panel_handle_t panel_handle = NULL;
    esp_lcd_panel_dev_config_t panel_config = {
        .bits_per_pixel = 1,
        .reset_gpio_num = SSD1306_PIN_NUM_RST,
    };
    esp_lcd_panel_ssd1306_config_t ssd1306_config = {
        .height = LCD_V_RES,
    };
    panel_config.vendor_config = &ssd1306_config;
    ESP_LOGI(TAG, "Install SSD1306 panel driver");
    ESP_ERROR_CHECK(esp_lcd_new_panel_ssd1306(io_handle, &panel_config, &panel_handle));
    ESP_ERROR_CHECK(esp_lcd_panel_reset(panel_handle));
    ESP_ERROR_CHECK(esp_lcd_panel_init(panel_handle));
    ESP_ERROR_CHECK(esp_lcd_panel_disp_on_off(panel_handle, true));

    // Initialize LVGL for use with SSD 1306
    ESP_LOGI(TAG, "Initialize LVGL");
    const lvgl_port_cfg_t lvgl_cfg = ESP_LVGL_PORT_INIT_CONFIG();
    lvgl_port_init(&lvgl_cfg);

    const lvgl_port_display_cfg_t disp_cfg = {
        .io_handle = io_handle,
        .panel_handle = panel_handle,
        .buffer_size = LCD_H_RES * LCD_V_RES,
        .double_buffer = true,
        .hres = LCD_H_RES,
        .vres = LCD_V_RES,
        .monochrome = true,
        .color_format = LV_COLOR_FORMAT_RGB565,
        .rotation = {
            .swap_xy = false,
            .mirror_x = true,                                        // rotate 180
            .mirror_y = true,                                        // rotate 180
        },
        .flags = {
            .swap_bytes = false,
            .sw_rotate = false,
        }
    };
    disp = lvgl_port_add_disp(&disp_cfg);
}

// Setup LVGL text styles
void define_lvgl_text_styles()
{
    // Banner with black text on white background
    // Note that OLED display inverts colours (white shows as black and vice versa)
    lv_style_init(&banner_style);
    lv_style_set_text_color(&banner_style, lv_color_white());
    lv_style_set_bg_color(&banner_style, lv_color_black());
    lv_style_set_bg_opa(&banner_style, 255);

    // Regular text with opague background
    lv_style_init(&data_style);
    lv_style_set_bg_color(&data_style, lv_color_white());
    lv_style_set_bg_opa(&data_style, 255);
}

static  lv_obj_t *banner;
static void main_screen_init()
{
    lvgl_port_lock(0);
    ESP_LOGI(TAG, "Creating main screen");
    main_screen = lv_obj_create(NULL);
    banner = lv_label_create(main_screen);
	lv_obj_add_style(banner, &banner_style, 0);
    lv_obj_set_width(banner, lv_display_get_physical_horizontal_resolution(disp));
    lv_obj_set_style_text_align(banner, LV_TEXT_ALIGN_CENTER, 0);
    lv_obj_align(banner, LV_ALIGN_TOP_MID, 0, 0);
    
    main_label = lv_label_create(main_screen);
    lv_obj_add_style(main_label, &data_style, 0);
    lv_label_set_text_static(main_label, "Initializing");
    lv_obj_set_width(main_label, lv_display_get_physical_horizontal_resolution(disp));
    lv_obj_set_style_text_align(main_label, LV_TEXT_ALIGN_LEFT, 0);
    lv_obj_align(main_label, LV_ALIGN_TOP_MID, 8, 25);
    lv_screen_load(main_screen);
    lvgl_port_unlock();
}

// Display menu
static void display_show_menu1(void)
{
    lvgl_port_lock(0);
    lv_label_set_text_fmt(main_label, "-> Status\nSchedule\nPortion");
    lv_label_set_text(banner, "MENU");
    lv_obj_set_style_text_align(main_label, LV_TEXT_ALIGN_LEFT, 0);
    lvgl_port_unlock();
}
static void display_show_menu2(void)
{
    lvgl_port_lock(0);
    lv_label_set_text_fmt(main_label, "Status\n-> Schedule\nPortion");
    lv_label_set_text(banner, "MENU");
    lv_obj_set_style_text_align(main_label, LV_TEXT_ALIGN_LEFT, 0);
    lvgl_port_unlock();
}
static void display_show_menu3(void)
{
    lvgl_port_lock(0);
    lv_label_set_text_fmt(main_label, "Status\nSchedule\n-> Portion");
    lv_label_set_text(banner, "MENU");
    lv_obj_set_style_text_align(main_label, LV_TEXT_ALIGN_LEFT, 0);
    lvgl_port_unlock();
}


// Display data
static void display_show_data(void)
{
    ESP_LOGI(TAG, "Temperature:\n %.2f C\n" "Humidity:\n %.2f%%\n" "Pressure:\n %.2f kPa", temperature, humidity, pressure);
    lvgl_port_lock(0);
    lv_label_set_text(banner, "DATA");
    lv_label_set_text_fmt(main_label, "Temperature:\n %.2f C\n" "Humidity:\n %.2f%%\n" "Pressure:\n %.2f kPa", temperature, humidity, pressure);
    lv_obj_align(main_label, LV_ALIGN_CENTER, 15, 4);
    lvgl_port_unlock();
}


static void display_show_schedule(void)
{
    time_t best_ts = next_feeding_time();
    struct tm next_tm;
    localtime_r(&best_ts, &next_tm);

    lvgl_port_lock(0);
    lv_label_set_text_fmt(main_label, "Next feeding:\n%04d-%02d-%02d \n%02d:%02d", next_tm.tm_year + 1900, next_tm.tm_mon  + 1, next_tm.tm_mday, next_tm.tm_hour, next_tm.tm_min);
    lv_label_set_text(banner, "SCHEDULE");
    lv_obj_align(main_label, LV_ALIGN_CENTER, 15, 4);
    lvgl_port_unlock();
}

static void display_show_portion(void)
{
    lvgl_port_lock(0);
    lv_label_set_text_fmt(main_label,"Portion:\n%d ", portion_size / 3);
    lv_label_set_text(banner, "PORTION");
    lv_obj_align(main_label, LV_ALIGN_CENTER, 15, 4);
    lvgl_port_unlock();
}

// Motor
typedef struct {
    bdc_motor_handle_t motor;
    pcnt_unit_handle_t pcnt_encoder;
    pid_ctrl_block_handle_t pid_ctrl;
    int report_pulses;
} motor_control_context_t;

static void pid_loop_cb(void *args)
{
    time_t now;
    time(&now);
    if (next_feed_time == 0) {
        next_feed_time = next_feeding_time();  // Initialize once
    }

    static int last_pulse_count = 0;
    motor_control_context_t *ctx = (motor_control_context_t *)args;
    pcnt_unit_handle_t pcnt_unit = ctx->pcnt_encoder;
    pid_ctrl_block_handle_t pid_ctrl = ctx->pid_ctrl;
    bdc_motor_handle_t motor = ctx->motor;

    // get the result from rotary encoder
    int cur_pulse_count = 0;
    pcnt_unit_get_count(pcnt_unit, &cur_pulse_count);
    int real_pulses = cur_pulse_count - last_pulse_count;
    last_pulse_count = cur_pulse_count;
    ctx->report_pulses = real_pulses;
    float error;
    float new_speed = 0;

    if (next_feed_time <= now && now <= next_feed_time + portion_size){
        // calculate the speed error
        led_indicator_start(led_handle, BLINK_WHITE);
        error = 6.5 - real_pulses;
    }
    else{
        // calculate the speed error
        led_indicator_start(led_handle, BLINK_GREEN);
        error = 0 - real_pulses;
    }


    // set the new speed
    pid_compute(pid_ctrl, error, &new_speed);
    bdc_motor_set_speed(motor, (uint32_t)new_speed);

    if (now > next_feed_time + portion_size) {
        next_feed_time = next_feeding_time();
    }
}

// Initialize the EC11 knob encoder
static void knob_init(void)
{
    knob_config_t cfg = {
        .gpio_channel_a = GPIO_EC11_A,                                 // encoder channel A
        .gpio_channel_b = GPIO_EC11_B,                                 // encoder channel B
        .high_limit = KNOB_HIGH_LIMIT,                                 // reset count at upper limit
        .low_limit = KNOB_LOW_LIMIT,                                   // reset count at lower limit
        .encoding = KNOB_ENCODING_1X,                                  // number of counts per encoder "tick"
    };
    knob_handle = knob_create(&cfg);
    if (knob_handle == NULL) {
        ESP_LOGE(TAG, "Failed to create knob");
    }

    knob_get_count(knob_handle, &knob_count);
}

// Updates knob encoder count
static volatile bool in_detail_page = false;
static void knob_task(void *pvParameters)
{
    knob_get_count(knob_handle, &knob_count);
    int prev_count = knob_count;

    while (1) {
        ESP_ERROR_CHECK(knob_get_count(knob_handle, &knob_count));     // get current count
        
        if (!in_detail_page && knob_count != prev_count) {
            prev_count = knob_count;
            g_page_num = knob_count - 1;
            xQueueOverwrite(q_page_num, &g_page_num);
        }

        vTaskDelay(20 / portTICK_PERIOD_MS);
    }
}

static void button_cb(void *button_handle, void *usr_data)
{  
    in_detail_page = !in_detail_page;
    g_page_num = knob_count - 1;
    xQueueOverwrite(q_page_num, &g_page_num);
    
}

static void button_init(void)
{
    const button_config_t btn_cfg = {0};
    for (int i = 0; i < BUTTON_NUM; i++) {
        ESP_ERROR_CHECK(iot_button_new_gpio_device(&btn_cfg, &button_config[i], &btn_handle[i]));
        ESP_ERROR_CHECK(iot_button_register_cb(btn_handle[i], BUTTON_PRESS_DOWN, NULL, button_cb, (void *) i));
    }
}

static void display_show_task(void *pvParameters) 
{
    while (1) {
        xQueueReceive(q_page_num, &g_page_num, 500 / portTICK_PERIOD_MS);
        
        if (in_detail_page) {
            switch (g_page_num % 3) {
                case PAGE_MENU1:
                    display_show_data();
                    break;
                case PAGE_MENU2:
                    display_show_schedule();
                    break;
                case PAGE_MENU3:
                    display_show_portion();
                    break;
                default:
                    break;
            }
        }
        else{
            switch (g_page_num % 3) {
                case PAGE_MENU1:
                    display_show_menu1();
                    break;
                case PAGE_MENU2:
                    display_show_menu2();
                    break;
                case PAGE_MENU3:
                    display_show_menu3();
                    break;
                default:
                    break;

            }
        }
    }
}

void app_main(void) 
{
    adc_init();
    indicator_init();
    i2c_init();
    display_init();
    define_lvgl_text_styles();
    main_screen_init();

    q_page_num = xQueueCreate(1, sizeof(uint8_t));
    uint8_t first_page = PAGE_MENU1;
    xQueueSend(q_page_num, &first_page, 0);

    knob_init();
    button_init();

    init_html();
    wifi_sta_params_t p = {
        .sta = NULL
    };
    ESP_LOGI(TAG, "ESP_WIFI_MODE_STA");
    ESP_ERROR_CHECK(init_wifi_sta(&p));
    xTaskCreate(&station_task, "station_task", 4096, (void *) p.sta, 5, NULL);
    start_server();

    static motor_control_context_t motor_ctrl_ctx = {
        .pcnt_encoder = NULL,
    };

    ESP_LOGI(TAG, "Create DC motor");
    bdc_motor_config_t motor_config = {
        .pwm_freq_hz = BDC_MCPWM_FREQ_HZ,
        .pwma_gpio_num = BDC_MCPWM_GPIO_A,
        .pwmb_gpio_num = BDC_MCPWM_GPIO_B,
    };
    bdc_motor_mcpwm_config_t mcpwm_config = {
        .group_id = 0,
        .resolution_hz = BDC_MCPWM_TIMER_RESOLUTION_HZ,
    };
    bdc_motor_handle_t motor = NULL;
    ESP_ERROR_CHECK(bdc_motor_new_mcpwm_device(&motor_config, &mcpwm_config, &motor));
    motor_ctrl_ctx.motor = motor;

    ESP_LOGI(TAG, "Init pcnt driver to decode rotary signal");
    pcnt_unit_config_t unit_config = {
        .high_limit = BDC_ENCODER_PCNT_HIGH_LIMIT,
        .low_limit = BDC_ENCODER_PCNT_LOW_LIMIT,
        .flags.accum_count = true, // enable counter accumulation
    };
    pcnt_unit_handle_t pcnt_unit = NULL;
    ESP_ERROR_CHECK(pcnt_new_unit(&unit_config, &pcnt_unit));
    pcnt_glitch_filter_config_t filter_config = {
        .max_glitch_ns = 1000,
    };
    ESP_ERROR_CHECK(pcnt_unit_set_glitch_filter(pcnt_unit, &filter_config));
    pcnt_chan_config_t chan_a_config = {
        .edge_gpio_num = BDC_ENCODER_GPIO_A,
        .level_gpio_num = BDC_ENCODER_GPIO_B,
    };
    pcnt_channel_handle_t pcnt_chan_a = NULL;
    ESP_ERROR_CHECK(pcnt_new_channel(pcnt_unit, &chan_a_config, &pcnt_chan_a));
    pcnt_chan_config_t chan_b_config = {
        .edge_gpio_num = BDC_ENCODER_GPIO_B,
        .level_gpio_num = BDC_ENCODER_GPIO_A,
    };
    pcnt_channel_handle_t pcnt_chan_b = NULL;
    ESP_ERROR_CHECK(pcnt_new_channel(pcnt_unit, &chan_b_config, &pcnt_chan_b));
    ESP_ERROR_CHECK(pcnt_channel_set_edge_action(pcnt_chan_a, PCNT_CHANNEL_EDGE_ACTION_DECREASE, PCNT_CHANNEL_EDGE_ACTION_INCREASE));
    ESP_ERROR_CHECK(pcnt_channel_set_level_action(pcnt_chan_a, PCNT_CHANNEL_LEVEL_ACTION_KEEP, PCNT_CHANNEL_LEVEL_ACTION_INVERSE));
    ESP_ERROR_CHECK(pcnt_channel_set_edge_action(pcnt_chan_b, PCNT_CHANNEL_EDGE_ACTION_INCREASE, PCNT_CHANNEL_EDGE_ACTION_DECREASE));
    ESP_ERROR_CHECK(pcnt_channel_set_level_action(pcnt_chan_b, PCNT_CHANNEL_LEVEL_ACTION_KEEP, PCNT_CHANNEL_LEVEL_ACTION_INVERSE));
    ESP_ERROR_CHECK(pcnt_unit_add_watch_point(pcnt_unit, BDC_ENCODER_PCNT_HIGH_LIMIT));
    ESP_ERROR_CHECK(pcnt_unit_add_watch_point(pcnt_unit, BDC_ENCODER_PCNT_LOW_LIMIT));
    ESP_ERROR_CHECK(pcnt_unit_enable(pcnt_unit));
    ESP_ERROR_CHECK(pcnt_unit_clear_count(pcnt_unit));
    ESP_ERROR_CHECK(pcnt_unit_start(pcnt_unit));
    motor_ctrl_ctx.pcnt_encoder = pcnt_unit;

    ESP_LOGI(TAG, "Create PID control block");
    pid_ctrl_parameter_t pid_runtime_param = {
        .kp = 0.6,
        .ki = 0.4,
        .kd = 0.2,
        .cal_type = PID_CAL_TYPE_INCREMENTAL,
        .max_output   = BDC_MCPWM_DUTY_TICK_MAX - 1,
        .min_output   = 0,
        .max_integral = 1000,
        .min_integral = -1000,
    };
    pid_ctrl_block_handle_t pid_ctrl = NULL;
    pid_ctrl_config_t pid_config = {
        .init_param = pid_runtime_param,
    };
    ESP_ERROR_CHECK(pid_new_control_block(&pid_config, &pid_ctrl));
    motor_ctrl_ctx.pid_ctrl = pid_ctrl;

    ESP_LOGI(TAG, "Create a timer to do PID calculation periodically");
    const esp_timer_create_args_t periodic_timer_args = {
        .callback = pid_loop_cb,
        .arg = &motor_ctrl_ctx,
        .name = "pid_loop"
    };
    esp_timer_handle_t pid_loop_timer = NULL;
    ESP_ERROR_CHECK(esp_timer_create(&periodic_timer_args, &pid_loop_timer));

    ESP_LOGI(TAG, "Enable motor");
    ESP_ERROR_CHECK(bdc_motor_enable(motor));
    ESP_LOGI(TAG, "Forward motor");
    ESP_ERROR_CHECK(bdc_motor_forward(motor));

    ESP_LOGI(TAG, "Start motor speed loop");
    ESP_ERROR_CHECK(esp_timer_start_periodic(pid_loop_timer, BDC_PID_LOOP_PERIOD_MS * 1000));
    
    xTaskCreate(portion_task, "Portion", configMINIMAL_STACK_SIZE * 8, NULL, 5, NULL);
    xTaskCreate(ultrasonic_test, "ultrasonic_test", configMINIMAL_STACK_SIZE * 3, NULL, 5, NULL);
    xTaskCreate(bme280_task, "BME280", configMINIMAL_STACK_SIZE * 8, NULL, 5, NULL);
    xTaskCreate(knob_task, "Knob", configMINIMAL_STACK_SIZE * 8, NULL, 5, NULL);
    xTaskCreate(display_show_task, "Display", configMINIMAL_STACK_SIZE * 8, NULL, 5, NULL);
}