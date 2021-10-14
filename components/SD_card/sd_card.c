
#define LOG_LOCAL_LEVEL ESP_LOG_DEBUG

#include "esp_err.h"
#include "esp_log.h"
#include "esp_vfs_fat.h"
#include "driver/sdspi_host.h"
#include "driver/spi_common.h"
#include "sdmmc_cmd.h"
#include "sd_card.h"

static const char *  TAG = "SD_card";
static sdmmc_card_t *card;

/* SD card used pins definition */
#define SD_CMD_PIN 15
#define SD_D0_PIN  2
#define SD_D1_PIN  4
#define SD_D2_PIN  12
#define SD_D3_PIN  13

esp_err_t init_SD_card(sd_card_cfg_t card_cfg)
{
    esp_err_t ret = ESP_OK;

    esp_vfs_fat_sdmmc_mount_config_t mount_config = {
        .format_if_mount_failed =
                true, // Must be true to work with no file system SD cards.
        .max_files            = card_cfg.max_files_opened,
        .allocation_unit_size = card_cfg.alloc_unit_size
    };

    ESP_LOGI(TAG, "Initializing SD card");

    ESP_LOGI(TAG, "Using SDMMC peripheral");
    sdmmc_host_t host = SDMMC_HOST_DEFAULT();

    host.max_freq_khz =
            card_cfg.max_host_freq_khz; // uncomment to change max host freq to highspeed

    sdmmc_slot_config_t slot_config = SDMMC_SLOT_CONFIG_DEFAULT();

    /* To use 1-line SD mode, uncomment the following line */
    // slot_config.width = 1;

    gpio_set_pull_mode(SD_CMD_PIN,
                       GPIO_PULLUP_ONLY); // CMD, needed in 4- and 1- line modes
    gpio_set_pull_mode(SD_D0_PIN,
                       GPIO_PULLUP_ONLY); // D0, needed in 4- and 1-line modes
    gpio_set_pull_mode(SD_D1_PIN,
                       GPIO_PULLUP_ONLY); // D1, needed in 4-line mode only
    gpio_set_pull_mode(SD_D2_PIN,
                       GPIO_PULLUP_ONLY); // D2, needed in 4-line mode only
    gpio_set_pull_mode(SD_D3_PIN,
                       GPIO_PULLUP_ONLY); // D3, needed in 4- and 1-line modes

    ret = esp_vfs_fat_sdmmc_mount(
            card_cfg.mount_point, &host, &slot_config, &mount_config, &card);

    if (ret != ESP_OK) {
        if (ret == ESP_FAIL) {
            ESP_LOGE(TAG, "Failed to mount filesystem. ");
        } else {
            ESP_LOGE(TAG,
                     "Failed to initialize the card (%s). "
                     "Make sure SD card lines have pull-up resistors in place.",
                     esp_err_to_name(ret));
        }
        goto exit;
    }
    /* Card has been initialized, print its properties */
    sdmmc_card_print_info(stdout, card);
exit:
    return ret;
}

esp_err_t unmount_SD_card(void)
{
    esp_err_t ret = esp_vfs_fat_sdcard_unmount(MOUNT_POINT, card);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "FAILED TO UNMOUNT SD CARD");
    } else {
        ESP_LOGD(TAG, "unmount_SD_card done");
    }
    return ret;
}
