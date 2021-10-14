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

#ifndef _MAX86XXX_CORE_H_
#define _MAX86XXX_CORE_H_
#include "sdkconfig.h"
#ifdef CONFIG_IDF_TARGET_ESP32
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/event_groups.h"
#include "freertos/queue.h"
#include "esp_system.h"
#include "max86xxx_common.h"
#else
#include "platform.h"
#include "queue.h"
#endif
//#define MAX86XXX_PPG_ENABLED
//#define MAX86XXX_ECG_ENABLED
#define MAX86XXX_ECPPG_ENABLED
/* #define MAX86XXX_BIOZ_ENABLED */
/* #define MAX86XXX_GSR_ENABLED */
//#define MAX86XXX_FLICKER_ENABLED
//#define MAX86XXX_BLUE_ENABLED
//#define MAX86XXX_EUV_ENABLED
/* #define MAX86XXX_TP_ENABLED */

#ifdef MAX86XXX_PPG_ENABLED
#include "max86xxx_ppg.h"
#endif
#ifdef MAX86XXX_ECPPG_ENABLED
#include "max86xxx_ecppg.h"
#endif
#ifdef MAX86XXX_FLICKER_ENABLED
#include "max86xxx_flicker.h"
#endif
#ifdef MAX86XXX_ECG_ENABLED
#include "max86xxx_ecg.h"
#endif
#if (defined(MAX86XXX_BIOZ_ENABLED) || defined(MAX86XXX_GSR_ENABLED))
#include "max86xxx_imp.h"
#endif
#ifdef MAX86XXX_BLUE_ENABLED
#include "max86xxx_blue.h"
#endif
#ifdef MAX86XXX_EUV_ENABLED
#include "max86xxx_uv.h"
#endif
#ifdef MAX86XXX_TP_ENABLED
#include "max86xxx_tp.h"
#endif

/* #define USE_MAX86903_REGMAP */
//#define USE_MAX86908_REGMAP
/* #define USE_MAX30110_REGMAP */

#ifdef USE_MAX86903_REGMAP
#include "max86903_map.h"
#elif defined(SENSOR_MAX86908)
#include "max86908_map.h"
#define EXPECTED_PART_ID   MAX86908_EXPECTED_PART_ID
#define WHOAMI_REG_PART_ID MAX86XXX_REG_WHOAMI_REG_PART_ID
#elif defined(SENSOR_MAX30110)
#include "max30110_map.h"
#define EXPECTED_PART_ID   MAX30110_EXPECTED_PART_ID
#define WHOAMI_REG_PART_ID MAX30110_REG_WHOAMI_PART_ID
#else
#endif

#define HYBRID_PACKAGE

#define get_sensor_ptr(pdev, id) ((*pdev->sensors)[id])
#define priv_of(sd, id)          ((*sd->sensors)[id]->priv)

#define PWR_ON  true
#define PWR_OFF false

#define MAX86XXX_I2C_RETRY_DELAY 10
#define MAX86XXX_I2C_MAX_RETRIES 5

#define MAX86XXX_FIFO_SIZE 32
#define MAX_EOL_RESULT     132
#define MAX_LIB_VER        20

#define MAX_FIFO_SLOT_NUM    4
#define NUM_BYTES_PER_SAMPLE 3
#define MAX_SENSOR_NAME_LEN  20

#define MAX86XXX_MAX_FIFO_BUF_LEN \
    (NUM_BYTES_PER_SAMPLE * MAX86XXX_FIFO_SIZE * MAX_FIFO_SLOT_NUM)

#define FIFO_DATA_18BIT_MASK 0x3FFFF

#define MAX86XXX_COMMON_FIFO_SZ (24 * 50)
/* sizeof ecg_data_t and ppg_data_t are 8 bytes, sizeof ecppg_data_t is 12 bytes.
 * Least common multiple value is 24 of three data type sizes.
 * Therefore, 24 * 50 is common multiple value for each data type.
 *
 * If any of these data types data size is changed, the common buffer size need to be checked.
 * It provides 100 slots for ecppg and 150 slots for ecg and ppg.
 **/

enum max86xxx_fifo_modes {
    MAX86XXX_NONE_MODE = -1,
#ifdef MAX86XXX_PPG_ENABLED
    MAX86XXX_PPG_MODE,
#endif
#ifdef MAX86XXX_ECG_ENABLED
    MAX86XXX_ECG_MODE,
#endif
#ifdef MAX86XXX_ECPPG_ENABLED
    MAX86XXX_ECPPG_MODE,
#endif
#ifdef MAX86XXX_BIOZ_ENABLED
    MAX86XXX_BIOZ_MODE,
#endif
#ifdef MAX86XXX_GSR_ENABLED
    MAX86XXX_GSR_MODE,
#endif
#ifdef MAX86XXX_FLICKER_ENABLED
    MAX86XXX_FLICKER_MODE,
#endif
#ifdef MAX86XXX_BLUE_ENABLED
    MAX86XXX_BLUE_MODE,
#endif
#ifdef MAX86XXX_EUV_ENABLED
    MAX86XXX_EUV_MODE,
#endif
#ifdef MAX86XXX_TP_ENABLED
    MAX86XXX_TP_MODE,
#endif
    MAX86XXX_NUM_OF_SENSORS,
};

struct max86xxx_input_report_desc {
    unsigned int type;
    unsigned int code;
    int          min;
    int          max;
    int          fuzz;
    int          flat;
};

struct max86xxx_dev;

struct max86xxx_sensor {
    void *                             priv; /* Private data */
    const char *                       name;
    char *                             lib_ver;
    struct input_dev *                 input_dev;
    struct device *                    sensor_dev;
    struct max86xxx_dev *              dev;
    struct attribute_group *           input_attrs;
    struct device_attribute **         sensor_attrs;
    struct max86xxx_input_report_desc *report_desc;
    const struct regmap *              reg_settings;
    int                                regmap_len;

    char (*is_sensor_active)(struct max86xxx_dev *, u16, int);
    int (*init)(struct max86xxx_dev *);
    int (*enable)(struct max86xxx_sensor *, int enable);
    int (*pause)(struct max86xxx_sensor *, int pause);
    int (*remove)(struct max86xxx_sensor *);
    int (*reset)(struct max86xxx_sensor *);
    int (*update)(struct max86xxx_sensor *);
    void (*report)(struct max86xxx_sensor *, int *);
    int (*eol_test)(struct max86xxx_sensor *);
    int (*eol_test_init)(struct max86xxx_sensor *);
};

struct max86xxx_dev {
    struct max86xxx_sensor *(*sensors)[];
    int            num_sensors;
    struct device *dev;
    int            regulator_state;
    int            pause_mode;
    int            curr_state;
    int            die_temp;
    int            int_gpio;
    int            vdd_oor_cnt;
    int            irq;
    uint8_t        q_buffer[MAX86XXX_COMMON_FIFO_SZ];
    uint8_t        part_id;
    uint8_t        rev_id;
};

int  max86xxx_sensor_enable(struct max86xxx_dev *sd, int sensor_id, int state);
int  max86xxx_sensor_pause(struct max86xxx_dev *sd, int sensor_id, int pause);
int  max86xxx_lib_ver_store(struct max86xxx_sensor *sensor, const char *buf);
int  max86xxx_i2c_block_write(const struct regmap reg_block[], int size);
int  max86xxx_reset(struct max86xxx_dev *sd);
int  max86xxx_poweroff(struct max86xxx_dev *sd);
int  max86xxx_write_reg(u8 reg_addr, u8 reg_data);
int  max86xxx_read_reg(u8 *buffer, int length);
int  max86xxx_regulator_onoff(struct max86xxx_dev *sd, char enable);
int  max86xxx_enable_die_temp(struct max86xxx_dev *sd);
int  max86xxx_read_die_temp(struct max86xxx_dev *sd);
int  max86xxx_get_fifo_settings(struct max86xxx_dev *sd, u16 *ch);
char max86xxx_is_ch_active(u16 fd_settings, u8 mode);
struct max86xxx_sensor *max86906_get_sensor_by_name(struct max86xxx_dev *sd,
                                                    const char *         name);
void *                  get_sensor_drvdata(struct device *dev, int sensor_id);
int                     max86xxx_get_sensor_mode(struct max86xxx_dev *sd,
                                                 u16                  fd_settings,
                                                 int                  num_channel);
int  max86xxx_get_num_samples_in_fifo(struct max86xxx_dev *sd);
void max86xxx_preprocess_data(int *samples, u8 fifo_channelmed);
int  update_bits(u8 reg_addr, u8 mask, u8 val);
#ifdef MAX86XXX_EUV_ENABLED
int  max86xxx_read_uv_data(struct max86xxx_dev *sd, int *samples);
void max86xxx_uv_report(struct max86xxx_sensor *sensor, int *samples);
void max86xxx_process_uv_data(struct max86xxx_dev *sd,
                              u8 *                 fifo_data,
                              int *                samples);
#endif

void *max86xxx_get_device_data(void);
int   max86xxx_init(void);
int   max86xxx_dump_regs(uint8_t *buf, int num_reg);

int spi_read_reg(mxc_spim_regs_t *_spim,
                 uint8_t          ssel,
                 uint8_t *        buffer,
                 int              length);
int max86xxx_spi_read_reg(mxc_spim_regs_t *_spim,
                          uint8_t          ssel,
                          uint8_t *        buffer,
                          int              length);
int spi_write_reg(mxc_spim_regs_t *_spim,
                  uint8_t          ssel,
                  uint8_t          reg_addr,
                  uint8_t          buf);
int max86xxx_spi_write_reg(mxc_spim_regs_t *_spim,
                           uint8_t          ssel,
                           uint8_t          reg_addr,
                           uint8_t          buf);
int spi_read_word(mxc_spim_regs_t *_spim,
                  uint8_t          ssel,
                  const uint8_t *  reg_addr,
                  uint8_t *        buffer);
#endif
