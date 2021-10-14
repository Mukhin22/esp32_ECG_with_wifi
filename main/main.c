/* WiFi station Example

   This example code is in the Public Domain (or CC0 licensed, at your option.)

   Unless required by applicable law or agreed to in writing, this
   software is distributed on an "AS IS" BASIS, WITHOUT WARRANTIES OR
   CONDITIONS OF ANY KIND, either express or implied.
*/
#include <string.h>
#include <stdbool.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/event_groups.h"
#include "freertos/queue.h"
#include "esp_system.h"
#include "esp_wifi.h"
#include "esp_event.h"
#include "esp_log.h"
#include "nvs_flash.h"
#include "sys/stat.h"
#include "main.h"
#include "driver/gpio.h"

#include "lwip/err.h"
#include "lwip/sys.h"

#include "../components/wifi_config.h"
#include "../components/HTTP_client/HTTP_client.h"
#include "../components/AP_mode/AP_mode.h"
#include "../components/STA_mode/STA_mode.h"
#include "../components/ping/ping.h"
#include "../components/AP_STA_mode/AP_STA_mode.h"
#include "../components/SD_card/sd_card.h"
#include "../components/console_comp/console_comp.h"
#include "../components/gpio_ctrl/gpio_ctrl.h"
#include "../components/i2c/i2c_med.h"
#include "../components/MAX86150/max86xxx_core.h"
#include "../components/MAX86150/max86xxx_ecg.h"

int i          = 10;
int integer    = 20;
int asdmasdasd = 2200;

#define MAIN_TAG "MAIN"

void app_main(void)
{
    /* Initialize NVS */
    esp_err_t ret = nvs_flash_init();
    if (ret == ESP_ERR_NVS_NO_FREE_PAGES ||
        ret == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        ESP_ERROR_CHECK(nvs_flash_erase());
        ret = nvs_flash_init();
    }
    ESP_ERROR_CHECK(ret);

#ifdef CONFIG_SD_CARD_ADDED
    sd_card_cfg_t sd_card_cfg = SD_CARD_CFG_DEFAULT();
    ESP_ERROR_CHECK(init_SD_card(sd_card_cfg));
    json_cfg_data_t wifi_cfg = {
        .ssid = APSTA_STA_SSID,
        .pass = APSTA_STA_PASS,
    };
    ESP_ERROR_CHECK(write_SD_wifi_json(wifi_cfg));
    ESP_ERROR_CHECK(read_SD_wifi_json());
    ESP_ERROR_CHECK(unmount_SD_card());
#endif

#ifdef CONFIG_USE_ECG
    ESP_ERROR_CHECK(i2c_master_init());
    ESP_LOGI(MAIN_TAG, "max86xxx_init() returned %d", max86xxx_init());
#endif

    ESP_LOGI(MAIN_TAG, "ESP_WIFI_MODE_AP_STA start");
    if (wifi_init_ap_sta() == ESP_OK) {
        ESP_LOGI(MAIN_TAG, "Connection success");
        xTaskCreate(console_task, "console", 1024 * 10, NULL, 2, NULL);
    } else {
        ESP_LOGE(MAIN_TAG, "Connection failed");
        while (1) {
            vTaskDelay(1);
        }
    }
}
