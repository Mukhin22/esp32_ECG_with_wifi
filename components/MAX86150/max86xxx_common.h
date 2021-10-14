/*******************************************************************************
* Author: Ismail Kose, Ismail.Kose@maximintegrated.com
* Copyright (C) 2016 Maxim Integrated Products, Inc., All Rights Reserved.
*
* Permission is hereby granted, free of charge, to any person obtaining a
* copy of this software and associated documentation files (the "Software"),
* to deal in the Software without restriction, including without limitation
* the rights to use, copy, modify, merge, publish, distribute, sublicense,
* and/or sell copies of the Software, and to permit persons to whom the
* Software is furnished to do so, subject to the following conditions:
*
* The above copyright notice and this permission notice shall be included
* in all copies or substantial portions of the Software.
*
* THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS
* OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF
* MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT.
* IN NO EVENT SHALL MAXIM INTEGRATED BE LIABLE FOR ANY CLAIM, DAMAGES
* OR OTHER LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE,
* ARISING FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR
* OTHER DEALINGS IN THE SOFTWARE.
*
* Except as contained in this notice, the name of Maxim Integrated
* Products, Inc. shall not be used except as stated in the Maxim Integrated
* Products, Inc. Branding Policy.
*
* The mere transfer of this software does not imply any licenses
* of trade secrets, proprietary technology, copyrights, patents,
* trademarks, maskwork rights, or any other form of intellectual
* property whatsoever. Maxim Integrated Products, Inc. retains all
* ownership rights.
*******************************************************************************
*/

#ifndef MAX86XXX_COMMON_H
#define MAX86XXX_COMMON_H
#include <stdint.h>
#include <string.h>
/* ESP_IDF sdk configuration file include*/
#include "sdkconfig.h"
#include "../i2c/i2c_med.h"

#ifdef CONFIG_IDF_TARGET_ESP32
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/event_groups.h"
#include "freertos/queue.h"
#include "esp_system.h"
#include "esp_err.h"
#define UNSUSED_ATTR

/// @brief Number of data lines to use.
typedef enum {
    SPIM_WIDTH_1 = 0,
    SPIM_WIDTH_2 = 1,
    SPIM_WIDTH_4 = 2
} spim_width_t;

/// @brief SPIM Transaction request.
typedef struct spim_req spim_req_t;
struct spim_req {
    uint8_t ssel;  ///< Slave select number.
    uint8_t deass; ///< De-assert slave select at the end of the transaction.
    const uint8_t *tx_data;   ///< TX buffer.
    uint8_t *      rx_data;   ///< RX buffer.
    spim_width_t   width;     ///< Number of data lines to use
    unsigned       len;       ///< Number of bytes to send.
    unsigned       read_num;  ///< Number of bytes read.
    unsigned       write_num; ///< Number of bytes written.

    /**
     * @brief   Callback for asynchronous request.
     * @param   spim_req_t*  Pointer to the transaction request.
     * @param   int         Error code.
     */
    void (*callback)(spim_req_t *, int);
};
typedef uint8_t  u8;
typedef uint16_t u16;
typedef uint32_t u32;
typedef int8_t   s8;
typedef int8_t   s16;
typedef int32_t  s32;

typedef void *UNSUSED_ATTR max86xxx_sns_report_t;
typedef void UNSUSED_ATTR
        max_spim_regs_t; // SPI registers not used - needs to be redefined to platform
typedef void UNSUSED_ATTR
        mxc_spim_regs_t; // SPI registers not used - needs to be redefined to platform

#ifndef EINVAL
#define EINVAL ESP_ERR_INVALID_ARG
#endif

#define SENSOR_MAX86908
#elif
#include "platform.h"
#include "queue.h"

#endif /* CONFIG_IDF_TARGET_ESP32 */

#define SENSOR_MAX86908
struct regmap {
    u8 val;
    u8 addr;
};

#define LED_DRIVE_CURRENT_FULL_SCALE \
    (MAX_LED_DRIVE_CURRENT - MIN_LED_DRIVE_CURRENT)

#define AGC_DEFAULT_LED_OUT_RANGE_IR_RED 30
#define AGC_DEFAULT_LED_OUT_RANGE_GREEN  70

#define AGC_DEFAULT_CORRECTION_COEFF    50
#define AGC_DEFAULT_SENSITIVITY_PERCENT 10
#define AGC_DEFAULT_MIN_NUM_PERCENT     50

#define ILLEGAL_OUTPUT_POINTER            1
#define ILLEGAL_DIODE_OUTPUT_MIN_MAX_PAIR 2
#define ILLEGAL_LED_SETTING_MIN_MAX_PAIR  3
#define CONSTRAINT_VIOLATION              4

#define MIN_SAMPLES_NEEDED_FOR_AVG 100
#define MASK_LED_RGE_BITS          3
#if defined(SENSOR_MAX30110) && defined(DUAL_LED)
#define PROX_THRESHOLD_1 100000
#define PROX_THRESHOLD_2 400000
#elif defined(SENSOR_MAX86908) || defined(SENSOR_MAX30110)
#define PROX_THRESHOLD_1 10000
#define PROX_THRESHOLD_2 40000
#else
#error "Sensor type is not defined"
#endif
#define PROX_DEBOUNCE_SPS 2
#define DAQ_DEBOUNCE_SPS  20

#define DEFAULT_DAQ_LED_CURRENT_1  40000
#define DEFAULT_DAQ_LED_CURRENT_2  40000
#define DEFAULT_PROX_LED_CURRENT_1 10000
#define DEFAULT_PROX_LED_CURRENT_2 0

typedef struct _ecg_cfg_t {
    uint16_t fs;
    uint16_t notch_freq;
    uint16_t cutoff_freq;
    char     adaptive_filter_on;
} ecg_cfg_t;

enum LED_CTRL_SM {
    LED_PROX = 1,
    LED_DATA_ACQ,
};

enum {
    LED_1 = 0,
    LED_2,
    LED_3,
    NUM_OF_LED,
};

union led_range {
    struct {
        uint8_t led1 : 2;
        uint8_t led2 : 2;
        uint8_t led3 : 2;
        uint8_t : 2;
    };
    uint8_t val;
};

struct max86xxx_dev;

struct led_control {
    u32             diode_sum[NUM_OF_LED];
    u32             state;
    u32             prox_sum;
    u32             prox_sample_cnt;
    s32             led_current[NUM_OF_LED];
    u32             default_current[NUM_OF_LED];
    s32             agc_led_out_percent;
    s32             agc_corr_coeff;
    s32             agc_min_num_samples;
    s32             agc_sensitivity_percent;
    s32             change_by_percent_of_range[NUM_OF_LED];
    s32             change_by_percent_of_current_setting[NUM_OF_LED];
    s32             change_led_by_absolute_count[NUM_OF_LED];
    int             agc_is_enabled;
    union led_range led_range_settings;
    uint8_t         led_ranges;
};

extern struct device_attribute dev_attr_device_id;

int  update_led_current(union led_range *led_range_settings,
                        int              led_new_val,
                        int              led_num);
void ppg_auto_gain_ctrl(struct max86xxx_dev *dev,
                        struct led_control * led_ctrl,
                        u32                  sample_cnt,
                        int                  diode_data,
                        int                  led_num);
int  led_prox_init(struct max86xxx_dev *dev,
                   struct led_control * led_ctrl,
                   char                 lpm);
int  led_daq_init(struct max86xxx_dev *dev,
                  struct led_control * led_ctrl,
                  char                 lpm);
int  led_control_sm(struct max86xxx_dev *dev,
                    struct led_control * led_ctrl,
                    int                  diode_data,
                    char                 lpm);
void led_control_init(struct led_control *led_ctrl);
void led_control_reset(struct led_control *led_ctrl);

int max86xxx_get_ecg_imp_sample_rate(int *sample_rate);
int max86xxx_get_ppg_flicker_sample_rate(int *sample_rate);
#endif
