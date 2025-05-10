
/*
 * SPDX-FileCopyrightText: 2025 Michael D. Naish
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include "esp_log.h"
#include "i2c_bus.h"

#define I2C_PORT         CONFIG_I2C_PORT
#define I2C_MASTER_SDA   CONFIG_I2C_MASTER_SDA
#define I2C_MASTER_SCL   CONFIG_I2C_MASTER_SCL

static const char *TAG = "i2c_bus";

bool initialized = false;

esp_err_t i2c_bus_create(i2c_master_bus_handle_t *handle)
{
    esp_err_t res;

    if (!initialized) {
        i2c_master_bus_config_t bus_config = {
            .clk_source = I2C_CLK_SRC_DEFAULT,
            .i2c_port = I2C_PORT,
            .sda_io_num = I2C_MASTER_SDA,
            .scl_io_num = I2C_MASTER_SCL,
            .glitch_ignore_cnt = 7,
            .flags.enable_internal_pullup = true,
        };
        if ((res = i2c_new_master_bus(&bus_config, handle)) != ESP_OK) {
            ESP_LOGI(TAG, "Could not initialize I2C master bus: %d (%s)", res, esp_err_to_name(res));
            return ESP_FAIL;
        }
        ESP_LOGI(TAG, "New I2C master bus created");
        initialized = true;
        return ESP_OK;
    }
    else { // already initialized
        if ((res = i2c_master_get_bus_handle(0, handle)) != ESP_OK) {
            ESP_LOGI(TAG, "Could not get I2C master bus handle: %d (%s)", res, esp_err_to_name(res));
            return ESP_FAIL;
        }
        ESP_LOGI(TAG, "Got I2C master bus handle");
        return ESP_OK;
    }
}

bool i2c_bus_is_initialized()
{
    return initialized;
}