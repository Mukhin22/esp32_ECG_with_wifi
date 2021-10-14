/*
 * HTTP_client.h
 *
 *  Created on: Apr 19, 2021
 *      Author: strngr
 */

#ifndef COMPONENTS_HTTP_CLIENT_HTTP_CLIENT_H_
#define COMPONENTS_HTTP_CLIENT_HTTP_CLIENT_H_
#include "esp_err.h"
#include "../../main/file_utils.h"

#define PTR_IS_NULL(_ptr) (_ptr == NULL)

#define JSON_URL "http://192.168.4.2:8080/db.json"

/**
 * @brief API to create and process HTTP client no the device
 * @param url - URL adress used to check for JSON location
 * @param wifi_cfg - handle for wifi configuration data to store from JSON
 * @return ESP_OK success or error code on failure
 */
esp_err_t http_client(const char *url, json_cfg_data_t *wifi_cfg);

/**
 * @brief API used to print the wifi configuration got from JSON
 * @param wifi_cfg - handle for wifi configuration data to store from JSON
 * @return ESP_OK success or error code on failure
 */
esp_err_t print_wifi_cfg(const json_cfg_data_t *wifi_cfg);

/**
 * @brief API used to init dafault values to wifi_cfg. Also allocate memory if it's needed
 * @param wifi_cfg - handle for wifi configuration data to store from JSON
 * @return ESP_OK success or error code on failure
 */
esp_err_t set_wifi_cfg_default(json_cfg_data_t *wifi_cfg);

/**
 * @brief Free memory used to stire wifi configuration JSON data
 * @param wifi_cfg - handle for wifi configuration data to store from JSON
 * @return ESP_OK success or error code on failure
 */
esp_err_t deinit_wifi_cfg(json_cfg_data_t *wifi_cfg);

/**
 * @brief Allocate  wifi configuration JSON data
 * @return json_cfg_data - handle for wifi configuration data to store from JSON
 */
json_cfg_data_t *create_json_cfg(void);

//esp_err_t init_wifi_cfg(json_cfg_data &wifi_cfg);
//
//esp_err_t delete_wifi_cfg(json_cfg_data &wifi_cfg);

#endif /* COMPONENTS_HTTP_CLIENT_HTTP_CLIENT_H_ */
