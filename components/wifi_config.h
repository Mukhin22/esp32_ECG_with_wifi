/*
 * wifi_config.h
 *
 *  Created on: Mar 25, 2021
 *      Author: strngr
 */

#ifndef COMPONENTS_WIFI_CONFIG_H_
#define COMPONENTS_WIFI_CONFIG_H_

#include <stdint.h>
#include "sdkconfig.h"

#define ESP32_SSID      "esp32_ssid"
#define ESP32_PASS      "esp32_pass"
#define ESP32_CHAN      1
#define ESP32_MAX_CON   4
#define ESP32_MAX_RETRY 5

#define APSTA_STA_SSID CONFIG_STA_MODE_SSID
#define APSTA_STA_PASS CONFIG_STA_MODE_PASS

#define APSTA_AP_SSID CONFIG_AP_MODE_SSID
#define APSTA_AP_PASS CONFIG_AP_MODE_PASS

#define WIFI_CONNECTED_BIT    BIT0
#define WIFI_FAIL_BIT         BIT1
#define WIFI_DISCONNECTED_BIT BIT2

#define ESP32_MDNS_HOSTNAME "esp32_HOST"
#define ESP32_MDNS_INSTANCE "esp32_MDNS_INSTANCE"

#define TARGET_HOST          "www.google.com"
#define PING_TASK_STACK_SIZE 8192

#endif /* COMPONENTS_WIFI_CONFIG_H_ */
