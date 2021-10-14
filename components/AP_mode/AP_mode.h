/*
 * AP_mode.h
 *
 *  Created on: Mar 29, 2021
 *      Author: strngr
 */

#ifndef COMPONENTS_AP_MODE_H_
#define COMPONENTS_AP_MODE_H_
#include "../HTTP_client/HTTP_client.h"
/**
 * @brief  Init the WIFI AP in SoftAP mode
 * @note This function should be called only in case when no previous WIFI configuration was done
 */
void wifi_init_softap(void);
/**
 * @brief  Change the WIFI AP to SoftAP mode (in case it was in STA mode before)
 * @note Before calling this function use esp_wifi_stop() and esp_wifi_deinit() funcs
 */
void wifi_change_to_softap(void);

#endif /* COMPONENTS_AP_MODE_AP_MODE_H_ */
