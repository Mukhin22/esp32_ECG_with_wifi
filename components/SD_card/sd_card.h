/*
 * sd_card.h
 *
 *  Created on: Apr 26, 2021
 *      Author: strngr
 */

#ifndef COMPONENTS_SD_CARD_SD_CARD_H_
#define COMPONENTS_SD_CARD_SD_CARD_H_

#include "esp_err.h"
#include "../HTTP_client/HTTP_client.h"
#include "driver/sdmmc_host.h"

#define MOUNT_POINT "/med_bracelet"
#define SD_MAX_FILES \
    5 // maximum number of files which can be open at the same time
#define SD_ALLOCATION_UNIT_SIZE 16 * 1024

/**
 * @brief Default sd_card_cfg_t structure initializer for SD card peripheral
 */
#define SD_CARD_CFG_DEFAULT()                         \
    {                                                 \
        .mount_point       = MOUNT_POINT,             \
        .alloc_unit_size   = SD_ALLOCATION_UNIT_SIZE, \
        .max_files_opened  = SD_MAX_FILES,            \
        .max_host_freq_khz = SDMMC_FREQ_HIGHSPEED,    \
    }

typedef struct {
    char * mount_point;
    size_t alloc_unit_size;
    int    max_files_opened;
    int    max_host_freq_khz;
} sd_card_cfg_t;

/**
 * @brief SD card initialization function
 * @return ESP_OK success or error code on failure
 */
esp_err_t init_SD_card(sd_card_cfg_t card_cfg);

/**
 * @brief SD card deinitialization function
 * @return ESP_OK success or error code on failure
 */
esp_err_t unmount_SD_card(void);

#endif /* COMPONENTS_SD_CARD_SD_CARD_H_ */
