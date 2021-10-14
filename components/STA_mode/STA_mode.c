/*
 * STA_mode.c
 *
 *  Created on: Mar 25, 2021
 *      Author: strngr
 */

#define LOG_LOCAL_LEVEL ESP_LOG_DEBUG

#include <string.h>
#include "../wifi_config.h"
#include "../AP_mode/AP_mode.h"
#include "../STA_mode/STA_mode.h"

#include "esp_event_loop.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/event_groups.h"
#include "esp_system.h"
#include "esp_wifi.h"
#include "esp_event.h"
#include "esp_log.h"
#include "nvs_flash.h"

#include "lwip/err.h"
#include "lwip/sys.h"
#include "lwip/inet.h"
#include "lwip/ip4_addr.h"
#include "lwip/dns.h"
#include "ping/ping_sock.h"

static EventGroupHandle_t s_wifi_event_group;

#define STA_TAG "STA_module"

static int s_retry_num = 0;

static void __sta_event_handler(void *           arg,
                                esp_event_base_t event_base,
                                int32_t          event_id,
                                void *           event_data)
{
    if (event_base == WIFI_EVENT && event_id == WIFI_EVENT_STA_START) {
        esp_wifi_connect();
    } else if (event_base == WIFI_EVENT &&
               event_id == WIFI_EVENT_STA_DISCONNECTED) {
        if (s_retry_num < MED_BRACELET_MAX_RETRY) {
            esp_wifi_connect();
            s_retry_num++;
            ESP_LOGI(STA_TAG, "retry to connect to the AP");
        } else {
            xEventGroupSetBits(s_wifi_event_group, WIFI_FAIL_BIT);
            ESP_LOGI(
                    STA_TAG,
                    "connect to the SSID: %s with PASS: %s failed setting the AP mode...",
                    APSTA_STA_SSID,
                    APSTA_STA_PASS);
            /* Stop and deinit STA Mode*/
            ESP_ERROR_CHECK(esp_wifi_stop());
            ESP_ERROR_CHECK(esp_wifi_deinit());

            /* Change WIFI to softAP mode*/
            ESP_LOGI(STA_TAG, "ESP_WIFI_MODE_AP start");
            wifi_change_to_softap();
        }
    } else if (event_base == IP_EVENT && event_id == IP_EVENT_STA_GOT_IP) {
        ip_event_got_ip_t *event_got_ip = (ip_event_got_ip_t *)event_data;
        ESP_LOGI(STA_TAG, "got ip:" IPSTR, IP2STR(&event_got_ip->ip_info.ip));
        s_retry_num = 0;
        xEventGroupSetBits(s_wifi_event_group, WIFI_CONNECTED_BIT);
    }
}

esp_err_t wifi_init_sta(void)
{
    esp_err_t ret_value = ESP_OK;
    s_wifi_event_group  = xEventGroupCreate();
    ESP_LOGD(STA_TAG, "xEventGroupCreate done");
    ESP_ERROR_CHECK(esp_netif_init());
    ESP_LOGD(STA_TAG, "esp_netif_init done");
    esp_event_loop_delete_default();
    ESP_LOGD(STA_TAG, "esp_event_loop_delete_default done");
    ESP_ERROR_CHECK(esp_event_loop_create_default());
    ESP_LOGD(STA_TAG, "esp_event_loop_create_default done");
    esp_netif_create_default_wifi_sta();
    ESP_LOGD(STA_TAG, "esp_netif_create_default_wifi_sta done");

    wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();
    ESP_ERROR_CHECK(esp_wifi_init(&cfg));
    ESP_LOGD(STA_TAG, "esp_wifi_init done");

    esp_event_handler_instance_t instance_any_id;
    esp_event_handler_instance_t instance_got_ip;

    ESP_ERROR_CHECK(esp_event_handler_instance_register(WIFI_EVENT,
                                                        ESP_EVENT_ANY_ID,
                                                        &__sta_event_handler,
                                                        NULL,
                                                        &instance_any_id));
    ESP_ERROR_CHECK(esp_event_handler_instance_register(IP_EVENT,
                                                        IP_EVENT_STA_GOT_IP,
                                                        &__sta_event_handler,
                                                        NULL,
                                                        &instance_got_ip));

    wifi_config_t wifi_config = {
        .sta = {
            .threshold.authmode = WIFI_AUTH_WPA2_PSK,
            .ssid = APSTA_STA_SSID,
            .password = APSTA_STA_PASS,
            .pmf_cfg = {
                .capable = true,
                .required = false
            },
        },
    };
    ESP_ERROR_CHECK(esp_wifi_set_mode(WIFI_MODE_STA));
    ESP_LOGD(STA_TAG, "esp_wifi_set_mode done");
    ESP_ERROR_CHECK(esp_wifi_set_config(ESP_IF_WIFI_STA, &wifi_config));
    ESP_LOGD(STA_TAG, "esp_wifi_set_config done");
    ESP_ERROR_CHECK(esp_wifi_start());
    ESP_LOGD(STA_TAG, "esp_wifi_start done");

    ESP_LOGI(STA_TAG, "wifi_init_sta finished.");

    EventBits_t bits = xEventGroupWaitBits(s_wifi_event_group,
                                           WIFI_CONNECTED_BIT | WIFI_FAIL_BIT,
                                           pdFALSE,
                                           pdFALSE,
                                           portMAX_DELAY);

    if (bits & WIFI_CONNECTED_BIT) {
        ESP_LOGI(STA_TAG,
                 "connected to ap SSID:%s password:%s",
                 APSTA_STA_SSID,
                 APSTA_STA_PASS);

    } else if (bits & WIFI_FAIL_BIT) {
        ESP_LOGI(STA_TAG,
                 "Failed to connect to SSID:%s, password:%s",
                 APSTA_STA_SSID,
                 APSTA_STA_PASS);
        ret_value = ESP_FAIL;
    } else {
        ESP_LOGE(STA_TAG, "UNEXPECTED EVENT");
        ret_value = ESP_ERR_INVALID_STATE;
    }
    return ret_value;
}
