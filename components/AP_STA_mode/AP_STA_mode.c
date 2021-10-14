/*
 * AP_STA_mode.c
 *
 *  Created on: Apr 7, 2021
 *      Author: strngr
 */

#define LOG_LOCAL_LEVEL ESP_LOG_DEBUG

#include <string.h>
#include "../wifi_config.h"
#include "../AP_mode/AP_mode.h"

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
#include "../HTTP_client/HTTP_client.h"
#include "../STA_mode/STA_mode.h"

#define AP_STA_TAG "app_sta"

static EventGroupHandle_t s_wifi_event_group;
static int                s_retry_num = 0;

static esp_err_t ap_sta_event_handler(void *ctx, system_event_t *event)
{
    switch (event->event_id) {
    case WIFI_EVENT_WIFI_READY:
        ESP_LOGI(AP_STA_TAG, "WIFI_EVENT_WIFI_READY");
        break;
    case WIFI_EVENT_SCAN_DONE:
        ESP_LOGI(AP_STA_TAG, "WIFI_EVENT_SCAN_DONE");
        break;
    case WIFI_EVENT_STA_STOP:
        ESP_LOGI(AP_STA_TAG, "WIFI_EVENT_STA_STOP");
        break;
    case SYSTEM_EVENT_STA_START:
        ESP_LOGI(AP_STA_TAG, "SYSTEM_EVENT_STA_START");
        ESP_LOGI(AP_STA_TAG, "Connecting to AP...");
        esp_wifi_connect();
        break;
    case WIFI_EVENT_STA_AUTHMODE_CHANGE:
        ESP_LOGI(AP_STA_TAG, "WIFI_EVENT_STA_AUTHMODE_CHANGE");
        break;
    case SYSTEM_EVENT_STA_GOT_IP:
        ESP_LOGI(AP_STA_TAG, "SYSTEM_EVENT_STA_GOT_IP");
        s_retry_num = 0;
        xEventGroupSetBits(s_wifi_event_group, WIFI_CONNECTED_BIT);
        ESP_LOGI(AP_STA_TAG, "Connected.");
        break;
    case SYSTEM_EVENT_AP_STAIPASSIGNED:
        ESP_LOGI(AP_STA_TAG, "SYSTEM_EVENT_AP_STAIPASSIGNED event...");
        break;
    case WIFI_EVENT_STA_CONNECTED:
        ESP_LOGI(AP_STA_TAG, "WIFI_EVENT_STA_CONNECTED");
        break;
    case SYSTEM_EVENT_STA_DISCONNECTED:
        ESP_LOGI(AP_STA_TAG,
                 "SYSTEM_EVENT_STA_DISCONNECTED, try to connect again...");
        if (s_retry_num < MED_BRACELET_MAX_RETRY) {
            esp_wifi_connect();
            s_retry_num++;
            ESP_LOGI(AP_STA_TAG, "retry to connect to the AP");
        } else {
            xEventGroupSetBits(s_wifi_event_group, WIFI_FAIL_BIT);
            ESP_LOGI(
                    AP_STA_TAG,
                    "connect to the SSID: %s with PASS: %s failed setting the AP mode...",
                    APSTA_STA_SSID,
                    APSTA_STA_PASS);
            /* Stop and deinit STA Mode*/
            ESP_ERROR_CHECK(esp_wifi_stop());
            ESP_ERROR_CHECK(esp_wifi_deinit());
        }
        break;
    case WIFI_EVENT_AP_STACONNECTED:
        ESP_LOGI(AP_STA_TAG, "WIFI_EVENT_AP_STACONNECTED");
        break;
    case WIFI_EVENT_AP_START:
        ESP_LOGI(AP_STA_TAG, "WIFI_EVENT_AP_START");
        break;
    case WIFI_EVENT_AP_STOP:
        ESP_LOGI(AP_STA_TAG, "WIFI_EVENT_AP_STOP");
        break;
    case WIFI_EVENT_AP_PROBEREQRECVED:
        ESP_LOGI(AP_STA_TAG, "WIFI_EVENT_AP_PROBEREQRECVED");
        break;
    default:
        ESP_LOGD(AP_STA_TAG, "UNKNOWN EVENT");
        break;
    }
    return ESP_OK;
}

esp_err_t wifi_init_ap_sta()
{
    esp_err_t ret_value = ESP_OK;
    s_wifi_event_group  = xEventGroupCreate();
    tcpip_adapter_init();
    ESP_ERROR_CHECK(esp_event_loop_init(ap_sta_event_handler, NULL));
    wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();
    ESP_ERROR_CHECK(esp_wifi_init(&cfg));

    esp_event_handler_instance_t instance_any_id;
    esp_event_handler_instance_t instance_got_ip;

    ESP_ERROR_CHECK(esp_event_handler_instance_register(WIFI_EVENT,
                                                        ESP_EVENT_ANY_ID,
                                                        &ap_sta_event_handler,
                                                        NULL,
                                                        &instance_any_id));
    ESP_ERROR_CHECK(esp_event_handler_instance_register(IP_EVENT,
                                                        IP_EVENT_STA_GOT_IP,
                                                        &ap_sta_event_handler,
                                                        NULL,
                                                        &instance_got_ip));

    ESP_ERROR_CHECK(esp_wifi_set_storage(WIFI_STORAGE_RAM));
    ESP_ERROR_CHECK(esp_wifi_set_mode(WIFI_MODE_APSTA));
    wifi_config_t sta_config = {
            .sta = {
                .ssid = APSTA_STA_SSID,
                .password = APSTA_STA_PASS,
                /* Setting a password implies station will connect to all security modes including WEP/WPA.
                 * However these modes are deprecated and not advisable to be used. Incase your Access point
                 * doesn't support WPA2, these mode can be enabled by commenting below line */
                .threshold.authmode = WIFI_AUTH_WPA2_PSK,

                .pmf_cfg = {
                    .capable = true,
                    .required = false
                },
            },
        };
    wifi_config_t ap_config = { .ap = { .ssid           = APSTA_AP_SSID,
                                        .password       = APSTA_AP_PASS,
                                        .ssid_len       = 0,
                                        .max_connection = 4,
                                        .authmode       = WIFI_AUTH_WPA_PSK } };
    ESP_ERROR_CHECK(esp_wifi_set_config(WIFI_IF_STA, &sta_config));
    ESP_ERROR_CHECK(esp_wifi_set_config(WIFI_IF_AP, &ap_config));
    ESP_ERROR_CHECK(esp_wifi_start());
    /* Waiting until either the connection is established (WIFI_CONNECTED_BIT) or connection failed for the maximum
     * number of re-tries (WIFI_FAIL_BIT). The bits are set by event_handler() (see above) */
    EventBits_t bits = xEventGroupWaitBits(s_wifi_event_group,
                                           WIFI_CONNECTED_BIT | WIFI_FAIL_BIT,
                                           pdFALSE,
                                           pdFALSE,
                                           portMAX_DELAY);
    /* xEventGroupWaitBits() returns the bits before the call returned, hence we can test which event actually
     * happened. */
    if (bits & WIFI_CONNECTED_BIT) {
        ESP_LOGI(AP_STA_TAG,
                 "connected to ap SSID:%s password:%s",
                 APSTA_STA_SSID,
                 APSTA_STA_PASS);

    } else if (bits & WIFI_FAIL_BIT) {
        ESP_LOGI(AP_STA_TAG,
                 "Failed to connect to SSID:%s, password:%s",
                 APSTA_STA_SSID,
                 APSTA_STA_PASS);
        ret_value = ESP_FAIL;
    } else {
        ESP_LOGE(AP_STA_TAG, "UNEXPECTED EVENT");
        ret_value = ESP_ERR_INVALID_STATE;
    }

    return ret_value;
}
