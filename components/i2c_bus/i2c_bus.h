/*
 * SPDX-FileCopyrightText: 2025 Michael D. Naish
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#pragma once 

#include "driver/i2c_master.h"
 
#ifdef __cplusplus
extern "C"
{
#endif

/**
 * @brief   Initiaize I2C master bus
 *
 * @param  I2C master bus handle
 * @return
 *     - ESP_OK Success
 *     - ESP_FAIL Fail
 */
esp_err_t i2c_bus_create(i2c_master_bus_handle_t *handle);

/**
 * @brief   Check whether I2C master bus has been initialized
 *
  * @return
 *     - true Initialized
 *     - false Uninitialized
 */
bool i2c_bus_is_initialized();

#ifdef __cplusplus
}
#endif