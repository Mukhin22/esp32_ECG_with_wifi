/*
 * file_utils.c
 *
 *  Created on: May 11, 2021
 *      Author: strngr
 */
#define LOG_LOCAL_LEVEL ESP_LOG_DEBUG

#include <stdio.h>
#include <string.h>
#include "esp_log.h"
#include "../components/SD_card/sd_card.h"
#include "file_utils.h"

static const char *TAG = "file_utils";

esp_err_t write_SD_wifi_json(const json_cfg_data_t wifi_config)
{
    esp_err_t ret = ESP_OK;
    ESP_LOGD(TAG, "Opening file to set wifi_config");
    FILE *f = fopen(MOUNT_POINT "/wifi.json", "w");
    if (f == NULL) {
        ESP_LOGE(TAG, "Failed to open file for writing");
        ret = ESP_FAIL;
        goto exit;
    }
    fprintf(f,
            "{\n\tPass: \"%s\",\n\tSSID: \"%s\"\n}",
            wifi_config.pass,
            wifi_config.ssid);
    fclose(f);
    ESP_LOGD(TAG, "File written");
exit:
    return ret;
}

esp_err_t read_SD_wifi_json(void)
{
    esp_err_t ret = ESP_OK;
    FILE *    f   = fopen(MOUNT_POINT "/wifi.json", "rb");

    if (f == NULL) {
        ESP_LOGE(TAG, "Failed to open file for reading");
        ret = ESP_FAIL;
        goto exit;
    }

    fseek(f, 0, SEEK_END);
    long fsize = ftell(f);
    fseek(f, 0, SEEK_SET);

    char *string = malloc(fsize + 1);
    if (!string) {
        ESP_LOGE(TAG, "Failed to allocate memory to read from file");
        ret = ESP_FAIL;
        fclose(f);
        goto exit;
    }
    if (!fread(string, 1, fsize, f)) {
        ESP_LOGE(TAG, "fread error");
        ret = ESP_FAIL;
        free(string);
        fclose(f);
        goto exit;
    }
    fclose(f);

    string[fsize] = 0;
    ESP_LOGI(TAG, "Read from file:\n%s", string);
    free(string);
    string = NULL;
exit:
    return ret;
}
