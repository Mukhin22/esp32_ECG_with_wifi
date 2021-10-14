/*
 * file_utils.h
 *
 *  Created on: May 11, 2021
 *      Author: strngr
 */

#ifndef MAIN_FILE_UTILS_H_
#define MAIN_FILE_UTILS_H_

#include "esp_err.h"
/**
 * @brief      Configuration data from JSON file stored on HTTP server
 */
typedef struct json_cfg_data {
    char *pass;     /*!< password to SSID stored on server */
    char *ssid;     /*!< SSID stored on server */
    char *JSON_str; /*!< JSON string */
} json_cfg_data_t;

/**
 * @brief Write json file with wifi configuration to SD card
 * @param wifi_config - name of file to create
 * @return ESP_OK success or error code on failure
 */
esp_err_t write_SD_wifi_json(const json_cfg_data_t wifi_config);

/**
 * @brief Read wifi.json file on the disk and prints its content to logs
 * @return ESP_OK success or error code on failure
 */
esp_err_t read_SD_wifi_json(void);

#endif /* MAIN_FILE_UTILS_H_ */
