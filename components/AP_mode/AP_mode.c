/*
 * AP_mode.c
 *
 *  Created on: Mar 26, 2021
 *      Author: strngr
 */
/*  WiFi softAP Example

   This example code is in the Public Domain (or CC0 licensed, at your option.)

   Unless required by applicable law or agreed to in writing, this
   software is distributed on an "AS IS" BASIS, WITHOUT WARRANTIES OR
   CONDITIONS OF ANY KIND, either express or implied.
*/

#include <string.h>
#include "../wifi_config.h"
#include "AP_mode.h"

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/event_groups.h"
#include "esp_system.h"
#include "esp_wifi.h"
#include "esp_event.h"
#include "esp_log.h"
#include "nvs_flash.h"
#define AP_TAG "AP_module"

#include "lwip/err.h"
#include "lwip/sys.h"
/* The examples use WiFi configuration that you can set via project configuration menu.

   If you'd rather not, just change the below entries to strings with
   the config you want - ie #define EXAMPLE_WIFI_SSID "mywifissid"
*/

static void wifi_event_handler(void *           arg,
                               esp_event_base_t event_base,
                               int32_t          event_id,
                               void *           event_data)
{
    if (event_id == WIFI_EVENT_AP_STACONNECTED) {
        wifi_event_ap_staconnected_t *event =
                (wifi_event_ap_staconnected_t *)event_data;
        ESP_LOGI(AP_TAG,
                 "station " MACSTR " join, AID=%d",
                 MAC2STR(event->mac),
                 event->aid);
    } else if (event_id == WIFI_EVENT_AP_STADISCONNECTED) {
        wifi_event_ap_stadisconnected_t *event =
                (wifi_event_ap_stadisconnected_t *)event_data;
        ESP_LOGI(AP_TAG,
                 "station " MACSTR " leave, AID=%d",
                 MAC2STR(event->mac),
                 event->aid);
    }
}

void wifi_init_softap(void)
{
    ESP_ERROR_CHECK(esp_netif_init());
    ESP_ERROR_CHECK(
            esp_event_loop_create_default()); // this line ESP_ERR_INVALID_STATE s_default_loop is true
    esp_netif_create_default_wifi_ap();

    wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();
    ESP_ERROR_CHECK(esp_wifi_init(&cfg));

    ESP_ERROR_CHECK(esp_event_handler_instance_register(
            WIFI_EVENT, ESP_EVENT_ANY_ID, &wifi_event_handler, NULL, NULL));
    wifi_config_t __wifi_config = { .ap = { .ssid           = APSTA_AP_SSID,
                                            .password       = APSTA_AP_PASS,
                                            .ssid_len       = 0,
                                            .max_connection = 4,
                                            .authmode = WIFI_AUTH_WPA_PSK } };

    if (strlen(APSTA_AP_PASS) == 0) {
        __wifi_config.ap.authmode = WIFI_AUTH_OPEN;
    }

    ESP_ERROR_CHECK(esp_wifi_set_mode(WIFI_MODE_AP));
    ESP_ERROR_CHECK(esp_wifi_set_config(ESP_IF_WIFI_AP, &__wifi_config));
    ESP_ERROR_CHECK(esp_wifi_start());

    ESP_LOGI(AP_TAG,
             "wifi_init_softap finished. SSID:%s password:%s channel:%d",
             APSTA_AP_SSID,
             APSTA_AP_PASS,
             MED_BRACELET_CHAN);
}

void wifi_change_to_softap(void)
{
    //    esp_wifi_stop();
    //    esp_wifi_deinit();
    //ESP_ERROR_CHECK(esp_event_loop_create_default()); // this line ESP_ERR_INVALID_STATE s_default_loop is true
    esp_netif_create_default_wifi_ap();
    ESP_LOGI(AP_TAG, "ESP wifi ap created");
    vTaskDelay(100 / portTICK_PERIOD_MS);
    wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();
    ESP_ERROR_CHECK(esp_wifi_init(&cfg));
    ESP_LOGI(AP_TAG, "esp_wifi_init done");
    ESP_ERROR_CHECK(esp_event_handler_instance_register(
            WIFI_EVENT, ESP_EVENT_ANY_ID, &wifi_event_handler, NULL, NULL));
    ESP_LOGI(AP_TAG, "esp_event_handler_instance_register done");
    static wifi_config_t __wifi_config = {
        .ap = { .ssid           = APSTA_AP_SSID,
                .ssid_len       = strlen(APSTA_AP_SSID),
                .channel        = MED_BRACELET_CHAN,
                .password       = APSTA_AP_PASS,
                .max_connection = MED_BRACELET_MAX_CON,
                .authmode       = WIFI_AUTH_WPA_WPA2_PSK },
    };
    if (strlen(APSTA_AP_PASS) == 0) {
        __wifi_config.ap.authmode = WIFI_AUTH_OPEN;
    }

    ESP_ERROR_CHECK(esp_wifi_set_mode(WIFI_MODE_AP));
    ESP_LOGI(AP_TAG, "esp_wifi_set_mode done");
    ESP_ERROR_CHECK(esp_wifi_set_config(ESP_IF_WIFI_AP, &__wifi_config));
    ESP_LOGI(AP_TAG, "esp_wifi_set_config done");
    ESP_ERROR_CHECK(esp_wifi_start());
    ESP_LOGI(AP_TAG, "esp_wifi_start done");

    ESP_LOGI(AP_TAG,
             "wifi_init_softap finished. SSID:%s password:%s channel:%d",
             APSTA_AP_SSID,
             APSTA_AP_PASS,
             MED_BRACELET_CHAN);
}
