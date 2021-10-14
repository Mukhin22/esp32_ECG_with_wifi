/*
 * ping.h
 *
 *  Created on: Apr 5, 2021
 *      Author: strngr
 */

#ifndef COMPONENTS_PING_PING_H_
#define COMPONENTS_PING_PING_H_

#ifndef __PING_H__
#define __PING_H__

#include "ping/ping_sock.h"
#include "mdns.h"

void initialise_mdns(void);
esp_err_t
initialize_ping(uint32_t interval_ms, uint32_t task_prio, char *target_host);

#endif /* __PING_H__ */

#endif /* COMPONENTS_PING_PING_H_ */
