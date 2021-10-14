/*
 * HTTP_client.с
 *
 *  Created on: Apr 19, 2021
 *      Author: strngr
 */

#define LOG_LOCAL_LEVEL ESP_LOG_DEBUG

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/event_groups.h"
#include "freertos/ringbuf.h"
#include "esp_system.h"
#include "esp_wifi.h"
#include "esp_event.h"
#include "esp_log.h"
#include "nvs_flash.h"

#include "lwip/err.h"
#include "lwip/sys.h"

#include "HTTP_client.h"
#include "esp_http_client.h"
#include "esp_tls.h"
#include "cJSON.h"

#define JSON_SSID_KEY  "SSID"
#define JSON_PASS_KEY  "pass"
#define CREDENTIAL_LEN 32
#define HTTP_READ_ERR  -1

static const char *TAG = "HTTP_client";

/* Ring buffer to store  JSON configuration data*/
static RingbufHandle_t xRingbuffer;

static esp_err_t _http_event_handler(esp_http_client_event_t *evt)
{
    switch (evt->event_id) {
    case HTTP_EVENT_ERROR:
        ESP_LOGI(TAG, "HTTP_EVENT_ERROR");
        break;
    case HTTP_EVENT_ON_CONNECTED:
        ESP_LOGI(TAG, "HTTP_EVENT_ON_CONNECTED");
        break;
    case HTTP_EVENT_HEADER_SENT:
        ESP_LOGI(TAG, "HTTP_EVENT_HEADER_SENT");
        break;
    case HTTP_EVENT_ON_HEADER:
        ESP_LOGI(TAG,
                 "HTTP_EVENT_ON_HEADER, key=%s, value=%s",
                 evt->header_key,
                 evt->header_value);
        break;
    case HTTP_EVENT_ON_DATA:
        ESP_LOGI(TAG, "HTTP_EVENT_ON_DATA, len=%d", evt->data_len);
        if (!esp_http_client_is_chunked_response(evt->client)) {
            //char buffer[512];
            char *buffer = malloc(evt->data_len + 1);
            if (esp_http_client_read(evt->client, buffer, evt->data_len) ==
                HTTP_READ_ERR) {
                ESP_LOGE(TAG, "ERROR reading file");
            }
            buffer[evt->data_len] = 0;
            ESP_LOGI(TAG, "buffer=%s", buffer);
            ESP_LOGD(TAG, "data len = %d", evt->data_len);
            UBaseType_t res = xRingbufferSend(
                    xRingbuffer, buffer, evt->data_len, pdMS_TO_TICKS(1000));
            if (res != pdTRUE) {
                ESP_LOGE(TAG, "Failed to xRingbufferSend");
            }
            free(buffer);
        }
        break;
    case HTTP_EVENT_ON_FINISH:
        ESP_LOGI(TAG, "HTTP_EVENT_ON_FINISH");
        break;
    case HTTP_EVENT_DISCONNECTED:
        ESP_LOGI(TAG, "HTTP_EVENT_DISCONNECTED");
        break;
    }
    return ESP_OK;
}

static char *JSON_Types(int type)
{
    if (type == cJSON_Invalid)
        return ("cJSON_Invalid");
    if (type == cJSON_False)
        return ("cJSON_False");
    if (type == cJSON_True)
        return ("cJSON_True");
    if (type == cJSON_NULL)
        return ("cJSON_NULL");
    if (type == cJSON_Number)
        return ("cJSON_Number");
    if (type == cJSON_String)
        return ("cJSON_String");
    if (type == cJSON_Array)
        return ("cJSON_Array");
    if (type == cJSON_Object)
        return ("cJSON_Object");
    if (type == cJSON_Raw)
        return ("cJSON_Raw");
    return NULL;
}

static void JSON_Parse(const cJSON *const root, json_cfg_data_t *wifi_cfg)
{
    if (PTR_IS_NULL(wifi_cfg)) {
        ESP_LOGE(TAG, "Wrong pointer JSON_Parse");
    }
    cJSON *current_element = NULL;
    cJSON_ArrayForEach(current_element, root)
    {
        ESP_LOGI(TAG, "type=%s", JSON_Types(current_element->type));
        ESP_LOGI(TAG, "current_element->string=%p", current_element->string);
        if (current_element->string) {
            const char *string = current_element->string;
            ESP_LOGI(TAG, "[%s]", string);
            if (strstr(string, JSON_SSID_KEY)) {
                memcpy(wifi_cfg->ssid,
                       current_element->valuestring,
                       strlen(current_element->valuestring) + 1);
                ESP_LOGI(TAG, "ssid found %s", current_element->valuestring);
            }
            if (strstr(string, JSON_PASS_KEY)) {
                memcpy(wifi_cfg->pass,
                       current_element->valuestring,
                       strlen(current_element->valuestring) + 1);
                ESP_LOGI(TAG, "pass found %s", current_element->valuestring);
            }
        }
        if (cJSON_IsInvalid(current_element)) {
            ESP_LOGI(TAG, "Invalid");
        } else if (cJSON_IsFalse(current_element)) {
            ESP_LOGI(TAG, "False");
        } else if (cJSON_IsTrue(current_element)) {
            ESP_LOGI(TAG, "True");
        } else if (cJSON_IsNull(current_element)) {
            ESP_LOGI(TAG, "Null");
        } else if (cJSON_IsNumber(current_element)) {
            int    valueint    = current_element->valueint;
            double valuedouble = current_element->valuedouble;
            ESP_LOGI(TAG, "int=%d double=%f", valueint, valuedouble);
        } else if (cJSON_IsString(current_element)) {
            ESP_LOGI(TAG, "JSON is string:");
            const char *valuestring = current_element->valuestring;
            ESP_LOGI(TAG, "%s", valuestring);
        } else if (cJSON_IsArray(current_element)) {
            ESP_LOGI(TAG, "Array");
            JSON_Parse(current_element, wifi_cfg);
        } else if (cJSON_IsObject(current_element)) {
            ESP_LOGI(TAG, "Object");
            JSON_Parse(current_element, wifi_cfg);
        } else if (cJSON_IsRaw(current_element)) {
            ESP_LOGI(TAG, "Raw(Not support)");
        }
    }
}

esp_err_t http_client(const char *url, json_cfg_data_t *wifi_cfg)
{
    esp_err_t err = ESP_OK;
    if (PTR_IS_NULL(wifi_cfg) || PTR_IS_NULL(url)) {
        err = ESP_ERR_INVALID_ARG;
        ESP_LOGE(TAG, "NULL PTR FOUND");
        goto exit;
    }

    //Create Ring Buffer with no Split
    xRingbuffer = xRingbufferCreate(1024, RINGBUF_TYPE_NOSPLIT);
    if (PTR_IS_NULL(xRingbuffer)) {
        err = ESP_FAIL;
        ESP_LOGE(TAG, "FAILED TO CREATE RINGBUF");
        goto exit;
    }

    esp_http_client_config_t config = {
        .url           = JSON_URL,
        .event_handler = _http_event_handler,
    };

    esp_http_client_handle_t client = esp_http_client_init(&config);
    if (PTR_IS_NULL(client)) {
        err = ESP_FAIL;
        ESP_LOGE(TAG, "FAILED TO INIT HTTP CLIENT");
        goto exit;
    }
    // GET
    err = esp_http_client_perform(client);
    if (err == ESP_OK) {
        ESP_LOGI(TAG,
                 "HTTP GET Status = %d, content_length = %d",
                 esp_http_client_get_status_code(client),
                 esp_http_client_get_content_length(client));
        //Receive an item from no-split ring buffer
        int   bufferSize = esp_http_client_get_content_length(client);
        char *buffer     = malloc(bufferSize + 1);
        if (PTR_IS_NULL(buffer)) {
            ESP_LOGE(TAG, "FAILED TO ALLOCATE MEMORY");
            err = ESP_FAIL;
            goto exit;
        }
        size_t item_size;
        int    index = 0;
        while (1) {
            char *item = (char *)xRingbufferReceive(
                    xRingbuffer, &item_size, pdMS_TO_TICKS(1000));
            if (item != NULL) {
                ESP_LOGD(TAG, "GOT ITEM FROM RING BUFFER");
                for (int i = 0; i < item_size; i++) {
                    printf("%c", item[i]);
                    buffer[index] = item[i];
                    index++;
                    buffer[index] = 0;
                }
                printf("\n");
                //Return Item
                vRingbufferReturnItem(xRingbuffer, (void *)item);
            } else {
                ESP_LOGE(TAG, "End of receive item");
                break;
            }
        }
        ESP_LOGI(TAG, "buffer=\n%s", buffer);

        ESP_LOGI(TAG, "Deserialize.....");
        cJSON *root        = cJSON_Parse(buffer);
        wifi_cfg->JSON_str = (char *)cJSON_Print(root);
        JSON_Parse(root, wifi_cfg);
        cJSON_Delete(root);
        free(buffer);
        buffer = NULL;

    } else {
        ESP_LOGE(TAG, "HTTP GET request failed: %s", esp_err_to_name(err));
    }
    esp_http_client_cleanup(client);
exit:
    return err;
}

esp_err_t print_wifi_cfg(const json_cfg_data_t *wifi_cfg)
{
    esp_err_t err = ESP_OK;
    if (!wifi_cfg->pass || !wifi_cfg->ssid) {
        ESP_LOGE(TAG, "NULL PTR detected while printing");
        err = ESP_ERR_INVALID_ARG;
        goto exit;
    } else {
        ESP_LOGI(TAG, "SSID:%s ,Pass:%s .", wifi_cfg->ssid, wifi_cfg->pass);
    }
exit:
    return err;
}

esp_err_t set_wifi_cfg_default(json_cfg_data_t *wifi_cfg)
{
    esp_err_t err = ESP_OK;
    if (!wifi_cfg) {
        err = ESP_ERR_INVALID_ARG;
        ESP_LOGE(TAG, "NULL PTR FOUND!");
        goto exit;
    }
    ESP_LOGD(TAG, "Setting cfg to default");
    if (!wifi_cfg->ssid) {
        wifi_cfg->ssid = (char *)malloc(sizeof(char) * CREDENTIAL_LEN);
        if (!wifi_cfg->ssid) {
            ESP_LOGE(TAG, "FAILED TO ALLOCATE MEM FOR SSID");
            err = ESP_FAIL;
            goto exit;
        } else {
            ESP_LOGD(TAG, "Allocated ssid");
        }
    } else {
        wifi_cfg->ssid = '\0';
    }
    if (!wifi_cfg->pass) {
        wifi_cfg->pass = (char *)malloc(sizeof(char) * CREDENTIAL_LEN);
        if (!wifi_cfg->pass) {
            ESP_LOGE(TAG, "FAILED TO ALLOCATE MEM FOR PASS");
            err = ESP_FAIL;
            goto exit;
        } else {
            ESP_LOGD(TAG, "Allocated pass");
        }
    } else {
        wifi_cfg->pass = '\0';
    }
    if (!wifi_cfg->JSON_str) {
        wifi_cfg->JSON_str = (char *)malloc(sizeof(char) * CREDENTIAL_LEN);
        if (!wifi_cfg->JSON_str) {
            ESP_LOGE(TAG, "FAILED TO ALLOCATE MEM FOR JSON_str");
            err = ESP_FAIL;
            goto exit;
        } else {
            ESP_LOGD(TAG, "Allocated string");
        }
    } else {
        wifi_cfg->JSON_str = '\0';
    }
exit:
    return err;
}

esp_err_t deinit_wifi_cfg(json_cfg_data_t *wifi_cfg)
{
    esp_err_t err = ESP_OK;
    if (!wifi_cfg) {
        err = ESP_ERR_INVALID_ARG;
        ESP_LOGE(TAG, "NULL PTR FOUND!");
        goto exit;
    }
    ESP_LOGI(TAG, "Free the wifi_cfg handle now");
    free(wifi_cfg->pass);
    free(wifi_cfg->ssid);
    wifi_cfg->pass = NULL;
    wifi_cfg->ssid = NULL;
    ESP_LOGI(TAG, "Handle deleted");
    free(wifi_cfg);
    wifi_cfg = NULL;
exit:
    return err;
}

json_cfg_data_t *create_json_cfg(void)
{
    json_cfg_data_t *wifi_cfg =
            (json_cfg_data_t *)malloc(sizeof(json_cfg_data_t));
    if (wifi_cfg) {
        memset(wifi_cfg, '\0', sizeof(json_cfg_data_t));
    } else {
        ESP_LOGE(TAG, "FAILED TO ALLOCATE MEM FOR JSON_CFG_DATA");
    }
    return wifi_cfg;
}
