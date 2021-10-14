/*
 * gpio.h
 *
 *  Created on: May 5, 2021
 *      Author: strngr
 */

#ifndef COMPONENTS_GPIO_CTRL_GPIO_CTRL_H_
#define COMPONENTS_GPIO_CTRL_GPIO_CTRL_H_

#define HIGH_LVL 1
#define LOW_LVL  0

#include "driver/gpio.h"

typedef enum { low_lvl = LOW_LVL, high_lvl = HIGH_LVL } gpio_lvl_t;

esp_err_t gpio_pin_configure(gpio_num_t       pin_num,
                             gpio_mode_t      mode,
                             gpio_pull_mode_t pull_mode,
                             gpio_lvl_t       lvl);

#endif /* COMPONENTS_GPIO_CTRL_GPIO_CTRL_H_ */
