/*
 * max86xxx_algo.h
 *
 *  Created on: May 11, 2021
 *      Author: strngr
 */

#ifndef COMPONENTS_MAX86150_MAX86XXX_ALGO_H_
#define COMPONENTS_MAX86150_MAX86XXX_ALGO_H_
#include "max86908_map.h"
#include "max86xxx_common.h"
#include "max86xxx_core.h"

/* this prototype must be defined to use func*/
int max86xxx_set_ecg_sr(u16 sample_rate);

/* this prototype must be defined to use func*/
void max86xxx_algo_init(void);

/* this prototype must be defined to use func*/
int max86xxx_ecg_algo_init(ecg_cfg_t *ecg_cfg);

/* this prototype must be defined to use func*/
int max86xxx_algo_set_ecg_notch(u16 param);

/* this prototype must be defined to use func*/
int max86xxx_algo_set_ecg_cutoff(u16 param);

/* this prototype must be defined to use func*/
int max86xxx_algo_set_ecg_adapt(bool param);

int max86xxx_algo_execute_once(max86xxx_sns_report_t *sns_report);

#endif /* COMPONENTS_MAX86150_MAX86XXX_ALGO_H_ */
