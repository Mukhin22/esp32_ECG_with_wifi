/*
 * gpio.c
 *
 *  Created on: May 5, 2021
 *      Author: strngr
 */
#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"
#include "gpio_ctrl.h"
#include "esp_log.h"

static const char *TAG = "gpio_control";

esp_err_t gpio_pin_configure(gpio_num_t       pin_num,
                             gpio_mode_t      mode,
                             gpio_pull_mode_t pull_mode,
                             gpio_lvl_t       lvl)
{
    esp_err_t ret = ESP_OK;
    if (mode < GPIO_MODE_DISABLE || mode > GPIO_MODE_OUTPUT) {
        ret = ESP_ERR_INVALID_ARG;
        ESP_LOGE(TAG, "Wrong mode to configure pin");
        goto exit;
    }
    gpio_config_t io_conf;
    //disable interrupt
    io_conf.intr_type = GPIO_INTR_DISABLE;
    //set as output mode. Supported only out, in, disable.

    io_conf.mode = mode;
    //bit mask of the pins that you want to set,e.g.GPIO18/19
    io_conf.pin_bit_mask = (1ULL << pin_num);
    if (mode == GPIO_MODE_INPUT) {
        ESP_LOGI(TAG, "GPIO mode is input configuring pull mode...");
        switch (pull_mode) {
        case GPIO_PULLUP_ONLY:
            io_conf.pull_down_en = 0;
            io_conf.pull_up_en   = 1;
            break;
        case GPIO_PULLDOWN_ONLY:
            io_conf.pull_down_en = 1;
            io_conf.pull_up_en   = 0;
            break;
        case GPIO_PULLUP_PULLDOWN:
            io_conf.pull_down_en = 1;
            io_conf.pull_up_en   = 0;
            break;
        case GPIO_FLOATING:
            io_conf.pull_down_en = 0;
            io_conf.pull_up_en   = 0;
            break;
        default:
            io_conf.pull_down_en = 0;
            io_conf.pull_up_en   = 0;
            break;
        }
        ESP_ERROR_CHECK(gpio_config(&io_conf));
        goto exit;
    }
    if (mode == GPIO_MODE_OUTPUT) {
        if (lvl < LOW_LVL || lvl > HIGH_LVL) {
            ret = ESP_ERR_INVALID_ARG;
            ESP_LOGE(TAG, "Wrong level to set on pin");
            goto exit;
        }
        ESP_ERROR_CHECK(gpio_config(&io_conf));
        ESP_LOGI(TAG, "GPIO mode is output setting the level...");
        ESP_LOGI(TAG, "setting pin level %s", lvl ? "high" : "low");
        ESP_ERROR_CHECK(gpio_set_level(pin_num, (uint32_t)lvl));
        goto exit;
    }
    io_conf.pull_down_en = 0;
    io_conf.pull_up_en   = 0;
    ESP_ERROR_CHECK(gpio_config(&io_conf));
exit:
    return ret;
}
