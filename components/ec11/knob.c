/*
 * SPDX-FileCopyrightText: 2024 Michael D. Naish
 *
 * SPDX-License-Identifier: CC-BY-4.0
 */

#include "esp_log.h"
#include "esp_check.h"
#include "knob.h"

static const char *TAG = "knob";

#define MAX_GLITCH_NS                 CONFIG_MAX_GLITCH_NS

#define KNOB_CHECK(a, str, ret_val)                               \
    if (!(a)) {                                                   \
        ESP_LOGE(TAG, "%s(%d): %s", __FUNCTION__, __LINE__, str); \
        return (ret_val);                                         \
    }

knob_handle_t knob_create(const knob_config_t *config) {
    KNOB_CHECK(config, "Invalid knob config", NULL);
    knob_dev_t *knob = (knob_dev_t *) calloc(1, sizeof(knob_dev_t));
    KNOB_CHECK(NULL != knob, "Knob memory alloc failed", NULL);

    ESP_LOGI(TAG, "Install PCNT unit");
    pcnt_unit_config_t unit_config = {
        .high_limit = config->high_limit,
        .low_limit = config->low_limit,
    };
    //pcnt_unit_handle_t unit = NULL;
    ESP_ERROR_CHECK(pcnt_new_unit(&unit_config, &(knob->unit)));

    ESP_LOGI(TAG, "Set glitch filter");
    pcnt_glitch_filter_config_t filter_config = {
        .max_glitch_ns = MAX_GLITCH_NS,
    };
    ESP_ERROR_CHECK(pcnt_unit_set_glitch_filter(knob->unit, &filter_config));

    ESP_LOGI(TAG, "Install PNCT channels");
    pcnt_chan_config_t chan_a_config = {
        .edge_gpio_num = config->gpio_channel_a,
        .level_gpio_num = config->gpio_channel_b,
    };
    pcnt_channel_handle_t pcnt_chan_a = NULL;
    ESP_ERROR_CHECK(pcnt_new_channel(knob->unit, &chan_a_config, &pcnt_chan_a));
    pcnt_chan_config_t chan_b_config = {
        .edge_gpio_num = config->gpio_channel_b,
        .level_gpio_num = config->gpio_channel_a,
    };
    pcnt_channel_handle_t pcnt_chan_b = NULL;
    ESP_ERROR_CHECK(pcnt_new_channel(knob->unit, &chan_b_config, &pcnt_chan_b));

    switch (config->encoding) {
        case 0:  // 1X
            ESP_ERROR_CHECK(pcnt_channel_set_edge_action(pcnt_chan_a, PCNT_CHANNEL_EDGE_ACTION_DECREASE, PCNT_CHANNEL_EDGE_ACTION_HOLD));
            ESP_ERROR_CHECK(pcnt_channel_set_level_action(pcnt_chan_a, PCNT_CHANNEL_LEVEL_ACTION_HOLD, PCNT_CHANNEL_LEVEL_ACTION_INVERSE));
            ESP_ERROR_CHECK(pcnt_channel_set_edge_action(pcnt_chan_b, PCNT_CHANNEL_EDGE_ACTION_INCREASE, PCNT_CHANNEL_EDGE_ACTION_HOLD));
            ESP_ERROR_CHECK(pcnt_channel_set_level_action(pcnt_chan_b, PCNT_CHANNEL_LEVEL_ACTION_HOLD, PCNT_CHANNEL_LEVEL_ACTION_INVERSE));
            break;

        case 1:  // 2x
            ESP_ERROR_CHECK(pcnt_channel_set_edge_action(pcnt_chan_a, PCNT_CHANNEL_EDGE_ACTION_DECREASE, PCNT_CHANNEL_EDGE_ACTION_INCREASE));
            ESP_ERROR_CHECK(pcnt_channel_set_level_action(pcnt_chan_a, PCNT_CHANNEL_LEVEL_ACTION_HOLD, PCNT_CHANNEL_LEVEL_ACTION_INVERSE));
            ESP_ERROR_CHECK(pcnt_channel_set_edge_action(pcnt_chan_b, PCNT_CHANNEL_EDGE_ACTION_INCREASE, PCNT_CHANNEL_EDGE_ACTION_DECREASE));
            ESP_ERROR_CHECK(pcnt_channel_set_level_action(pcnt_chan_b, PCNT_CHANNEL_LEVEL_ACTION_HOLD, PCNT_CHANNEL_LEVEL_ACTION_INVERSE));
            break;

        case 2:  // 4x
            ESP_ERROR_CHECK(pcnt_channel_set_edge_action(pcnt_chan_a, PCNT_CHANNEL_EDGE_ACTION_DECREASE, PCNT_CHANNEL_EDGE_ACTION_INCREASE));
            ESP_ERROR_CHECK(pcnt_channel_set_level_action(pcnt_chan_a, PCNT_CHANNEL_LEVEL_ACTION_KEEP, PCNT_CHANNEL_LEVEL_ACTION_INVERSE));
            ESP_ERROR_CHECK(pcnt_channel_set_edge_action(pcnt_chan_b, PCNT_CHANNEL_EDGE_ACTION_INCREASE, PCNT_CHANNEL_EDGE_ACTION_DECREASE));
            ESP_ERROR_CHECK(pcnt_channel_set_level_action(pcnt_chan_b, PCNT_CHANNEL_LEVEL_ACTION_KEEP, PCNT_CHANNEL_LEVEL_ACTION_INVERSE));
            break;
        
        default:  // 1x
            ESP_ERROR_CHECK(pcnt_channel_set_edge_action(pcnt_chan_a, PCNT_CHANNEL_EDGE_ACTION_DECREASE, PCNT_CHANNEL_EDGE_ACTION_HOLD));
            ESP_ERROR_CHECK(pcnt_channel_set_level_action(pcnt_chan_a, PCNT_CHANNEL_LEVEL_ACTION_HOLD, PCNT_CHANNEL_LEVEL_ACTION_INVERSE));
            ESP_ERROR_CHECK(pcnt_channel_set_edge_action(pcnt_chan_b, PCNT_CHANNEL_EDGE_ACTION_INCREASE, PCNT_CHANNEL_EDGE_ACTION_HOLD));
            ESP_ERROR_CHECK(pcnt_channel_set_level_action(pcnt_chan_b, PCNT_CHANNEL_LEVEL_ACTION_HOLD, PCNT_CHANNEL_LEVEL_ACTION_INVERSE));
            break;
    }
    ESP_LOGI(TAG, "enable pcnt unit");
    ESP_ERROR_CHECK(pcnt_unit_enable(knob->unit));
    ESP_LOGI(TAG, "clear pcnt unit");
    ESP_ERROR_CHECK(pcnt_unit_clear_count(knob->unit));
    ESP_LOGI(TAG, "start pcnt unit");
    ESP_ERROR_CHECK(pcnt_unit_start(knob->unit));

    KNOB_CHECK(NULL != knob, "knob create failed", NULL);

    return (knob_handle_t)knob;
}

esp_err_t knob_get_count(knob_handle_t knob_handle, int *count) {
    knob_dev_t *knob = (knob_dev_t *) knob_handle;

    ESP_RETURN_ON_ERROR(pcnt_unit_get_count(knob->unit, count), TAG, "unable to get count");
    return ESP_OK;
}


esp_err_t knob_reset_count(knob_handle_t knob_handle) {
    knob_dev_t *knob = (knob_dev_t *) knob_handle;

    ESP_RETURN_ON_ERROR(pcnt_unit_clear_count(knob->unit), TAG, "unable to reset count");
    return ESP_OK;
}
