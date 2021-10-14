/*
 * STA_mode.h
 *
 *  Created on: Mar 25, 2021
 *      Author: strngr
 */

#ifndef COMPONENTS_STA_MODE_H_
#define COMPONENTS_STA_MODE_H_

#include "../HTTP_client/HTTP_client.h"

/**
 * @brief  Init the WIFI AP in STA mode
 * @note This function should be called only in case when no previous WIFI configuration was done
 */
esp_err_t wifi_init_sta(void);

#endif /* COMPONENTS_STA_MODE_STA_MODE_H_ */
