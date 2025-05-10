/*
 * SPDX-FileCopyrightText: 2024 Michael D. Naish
 *
 * SPDX-License-Identifier: CC-BY-4.0
 */

#ifndef _KNOB_H_
#define _KNOB_H_

#include "esp_err.h"
#include "driver/pulse_cnt.h"
#include "driver/gpio.h"

typedef void *knob_handle_t;

/**
 * @brief Knob quadrature encoding method
 * 
 */
typedef enum {
    KNOB_ENCODING_1X,           // increments by 1 per knob detent
    KNOB_ENCODING_2X,           // increments by 2 per knob detent
    KNOB_ENCODING_4X,           // increments by 4 per knob detent
} knob_encoding_t;

/**
 * @brief Configuration of knob
 *
 */
typedef struct {
    int gpio_channel_a;         // GPIO pin for encoder Channel A
    int gpio_channel_b;         // GPIO pin for encoder Channel B
    int high_limit;             // upper limit before count resets
    int low_limit;              // lower limit before count resets
    knob_encoding_t encoding;   // quadrature encoding method
} knob_config_t;

/**
 * @brief Knob device
 *
 */
typedef struct {
    pcnt_unit_handle_t unit;    // handle to PNCT peripherial used to count encoder ticks
} knob_dev_t;

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief Create a knob
 *
 * @param[in] config pointer of knob configuration
 *
 * @return A handle to the created knob, or NULL in case of error.
 */
knob_handle_t knob_create(const knob_config_t *config);

/**
 * @brief Get knob count value
 *
 * @param[in] knob_handle Unit handle
 * @param[out] count Returned count value
 * @return
 *      - ESP_OK: Get knob count successfully
 *      - ESP_ERR_INVALID_ARG: Get knob count failed because of invalid argument
 *      - ESP_FAIL: Get knob count failed because of other error
 */
esp_err_t knob_get_count(knob_handle_t knob_handle, int *count);

/**
 * @brief Reset knob count value to 0
 *
 * @param[in] knob_handle Unit handle
 * @return
 *      - ESP_OK: Knob count reset successfully
 *      - ESP_ERR_INVALID_ARG: Knob count reset failed because of invalid argument
 *      - ESP_FAIL: Knob count reset failed because of other error
 */

esp_err_t knob_reset_count(knob_handle_t knob_handle);

#ifdef __cplusplus
}
#endif

#endif /* _KNOB_H_ */