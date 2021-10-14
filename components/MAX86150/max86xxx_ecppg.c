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
#include "max86xxx_core.h"
#include "max86xxx_algo.h"
#define LOG_LOCAL_LEVEL ESP_LOG_DEBUG
#include "esp_log.h"
static const char *TAG = "max86150_ecppg";

#if defined(MAX86XXX_ECPPG_ENABLED)

#ifdef USE_MAX86903_REGMAP
static const struct regmap ecppg_init_cfg[] = {
    /* FIFO Enable */
    { MAX86903_REG_TM, MAX86903_TM_ENTER1 },
    { MAX86903_REG_TM, MAX86903_TM_ENTER2 },
    { MAX86903_REG_ECG_ETI_TEST1, MAX86903_ECG_ETI_IN_BIAS_OPT },
    { MAX86903_REG_ECG_ETI_TEST2,
      MAX86903_ECG_ETI_DC_RESTORE_EN | MAX86903_ECG_ETI_F_CHOP_9K6 },
    { MAX86903_REG_ECG_ETI_TEST3, 0x00 },
    { MAX86903_REG_TM, MAX86903_TM_EXIT },
    { MAX86903_REG_FIFO_DATA_CTRL_1, (RED_PA_CHANNEL << 4) | IR_PA_CHANNEL },
    { MAX86903_REG_FIFO_DATA_CTRL_2, (IMP_I_CHANNEL << 4) | ECG_CHANNEL },
    { MAX86903_REG_ECG_ETI_CFG1,
      MAX86903_ECG_CIC_3RD_ORDER | MAX86903_ECG_ADC_OSR_2 },
    { MAX86903_REG_ECG_ETI_CFG2,
      MAX86903_ECG_PGA_GAIN_8 | MAX86903_ECG_IA_GAIN_20 },
    { MAX86903_REG_ECG_ETI_CFG3, 0x00 },
    { MAX86903_REG_PPG_CFG1,
      MAX86903_PPG_ADC_RGE_16384 | MAX86903_PPG_SR_400HZ |
              MAX86903_PPG_LED_PW_400US },
    { MAX86903_REG_PPG_CFG2, 0x00 },
    { MAX86903_REG_FIFO_CONFIG,
      MAX86903_FIFO_ROLLS_ON_FIFO | MAX86903_FIFO_A_FULL_F },
    { MAX86903_REG_INT_ENABLE1, A_FULL_MASK },
};

static const struct regmap ecppg_fifo_en_cfg[] = {
    { MAX86903_REG_SYSTEM_CTRL, MAX86903_SYSTEM_FIFO_EN },
};

static const struct regmap ecppg_fifo_dis_cfg[] = {
    { MAX86903_REG_SYSTEM_CTRL, 0x00 },
};
#elif defined(SENSOR_MAX86908)
static const struct regmap ecppg_init_cfg[] = {
    { MAX86908_REG_TM, MAX86908_TM_ENTER1 },
    { MAX86908_REG_TM, MAX86908_TM_ENTER2 },
    { MAX86908_REG_ECG_ETI_TEST1, MAX86908_BIAS_SWITCH_CAP_100MOHM },
    { MAX86908_REG_ECG_ETI_TEST2,
      MAX86908_ECG_ETI_DC_RESTORE_EN | MAX86908_ECG_ETI_DB_DISABLE },
    { MAX86908_REG_ECG_ETI_TEST3, MAX86908_ETI_CHOP_EN_MASK },
    { MAX86908_REG_TM, MAX86908_TM_EXIT },

    { MAX86908_REG_FIFO_DATA_CTRL_1, RED_PA_CHANNEL << 4 | IR_PA_CHANNEL },
    { MAX86908_REG_FIFO_DATA_CTRL_2, IMP_I_CHANNEL << 4 | ECG_CHANNEL },
    { MAX86908_REG_ECG_ETI_CFG_1, MAX86908_ECG_ADC_OSR_3 },
    { MAX86908_REG_ECG_ETI_CFG_2, MAX86908_ECG_F_CHOP },
    { MAX86908_REG_ECG_ETI_CFG_3,
      MAX86908_IMP_PGA_GAIN_8 | MAX86908_ECG_PGA_GAIN_8 |
              MAX86908_ECG_IA_GAIN_10 },
    { MAX86908_REG_ECG_ETI_CFG_4, 0x04 },

    { MAX86908_REG_PPG_CFG_1,
      MAX86908_PPG_ADC_RGE_32768 | MAX86908_PPG_SR_400HZ |
              MAX86908_PPG_LED_PW_400US },

    { MAX86908_REG_PPG_CFG_2, 0x00 },
    { MAX86908_REG_FIFO_CFG,
      MAX86908_FIFO_ROLLS_ON_FULL | (MAX86908_FIFO_A_FULL_MASK - 5) },
    { MAX86908_REG_INT_ENABLE_1, A_FULL_MASK },
    { MAX86908_REG_SYSTEM_CTRL, MAX86908_SYSTEM_FIFO_EN },
};

static const struct regmap ecppg_fifo_en_cfg[] = {
    { MAX86908_REG_SYSTEM_CTRL, MAX86908_SYSTEM_FIFO_EN },
};

static const struct regmap ecppg_fifo_dis_cfg[] = {
    { MAX86908_REG_SYSTEM_CTRL, 0x00 },
};
#elif defined(SENSOR_MAX30110)
static const struct regmap ecppg_init_cfg[] = {
    { MAX30110_REG_TM, MAX30110_TM_ENTER1 },
    { MAX30110_REG_TM, MAX30110_TM_ENTER2 },
    { MAX30110_ECG_ETI_TEST_MODE, 0x00 },
    { MAX30110_REG_ECG_ETI_TEST1, MAX30110_BIAS_DIR_2_5MOHM },
    { MAX30110_REG_ECG_ETI_TEST2, MAX30110_ECG_ETI_DC_RESTORE_EN },
    { MAX30110_REG_ECG_ETI_TEST3,
      MAX30110_ETI_CHOP_EN_MASK | MAX30110_BIOZ_BIAS_DIS },
    { MAX30110_REG_ECG_ETI_TEST4, 0x00 },
    { MAX30110_REG_TM, MAX30110_TM_EXIT },

    { MAX30110_REG_FIFO_DATA_CTRL_1, RED_PA_CHANNEL << 4 | IR_PA_CHANNEL },
    { MAX30110_REG_FIFO_DATA_CTRL_2, IMP_I_CHANNEL << 4 | ECG_CHANNEL },

    { MAX30110_REG_ECG_ETI_CFG_1,
      MAX30110_BIOZ_PHASE_90 | MAX30110_ECG_ADC_OSR_3 },
    { MAX30110_REG_ECG_ETI_CFG_2, 0x00 },
    { MAX30110_REG_ECG_ETI_CFG_3,
      MAX30110_ECG_PGA_GAIN_4 | MAX30110_PGA_ETI_GAIN_4 |
              MAX30110_LEAD_BIAS_100_MOHM },
    { MAX30110_REG_ECG_ETI_CFG_4, MAX30110_ETI_AC_CUR_50NA },

    { MAX30110_REG_PPG_CFG_1,
      MAX30110_PPG_ADC_RGE_32768 | MAX30110_PPG_SR_400HZ |
              MAX30110_PPG_LED_PW_400US },

    { MAX30110_REG_PPG_CFG_2, 0x00 },
    { MAX30110_REG_FIFO_CFG,
      MAX30110_FIFO_ROLLS_ON_FULL | MAX30110_FIFO_A_FULL_MASK },
    { MAX30110_REG_INT_ENABLE_1, A_FULL_MASK },
    { MAX30110_REG_SYSTEM_CTRL, MAX30110_SYSTEM_FIFO_EN },
};

static const struct regmap ecppg_fifo_en_cfg[] = {
    { MAX30110_REG_SYSTEM_CTRL, MAX30110_SYSTEM_FIFO_EN },
};

static const struct regmap ecppg_fifo_dis_cfg[] = {
    { MAX30110_REG_SYSTEM_CTRL, 0x00 },
};
#else
#warning "MAX86XXX register map is not defined"
#endif

char max86xxx_is_ecppg_active(struct max86xxx_dev *sd,
                              u16                  fd_settings,
                              int                  num_ch)
{
    return (num_ch == 4) && max86xxx_is_ch_active(fd_settings, IR_PA_CHANNEL) &&
           max86xxx_is_ch_active(fd_settings, RED_PA_CHANNEL) &&
           max86xxx_is_ch_active(fd_settings, ECG_CHANNEL) &&
           max86xxx_is_ch_active(fd_settings, IMP_I_CHANNEL);
}

int max86xxx_get_ecppg_data(struct max86xxx_dev *dev, ecppg_data_t *ecppg_data)
{
    int                         ret;
    struct max86xxx_sensor *    sensor;
    struct max86xxx_ecppg_data *ecppg_dev;

    sensor    = get_sensor_ptr(dev, MAX86XXX_ECPPG_MODE);
    ecppg_dev = sensor->priv;
    ret       = xQueueReceive(&ecppg_dev->queue, ecppg_data, (TickType_t)0);
    return ret;
}

void max86xxx_ecppg_report(struct max86xxx_sensor *sensor, int *samples)
{
    struct max86xxx_ecppg_data *ecppg_dev =
            priv_of(sensor->dev, MAX86XXX_ECPPG_MODE);
    int          ret;
    ecppg_data_t ecppg_data;

    ecppg_data.red = samples[RED_PA_CHANNEL];
    ecppg_data.ir  = samples[IR_PA_CHANNEL];
    ecppg_data.ecg = samples[ECG_CHANNEL];

    ret = xQueueSend(&ecppg_dev->queue, &ecppg_data, (TickType_t)0);
    if (ret < 0)
        printf("%s:%d failed. ret: %d\n", __func__, __LINE__, ret);

    if (ecppg_dev->led_ctrl.agc_is_enabled) {
        ecppg_dev->sample_cnt++;
        ret = led_control_sm(ecppg_dev->dev,
                             &ecppg_dev->led_ctrl,
                             samples[IR_PA_CHANNEL],
                             false);
        if (ret == LED_DATA_ACQ) {
            ppg_auto_gain_ctrl(ecppg_dev->dev,
                               &ecppg_dev->led_ctrl,
                               ecppg_dev->sample_cnt,
                               samples[IR_PA_CHANNEL],
                               LED_1);
            ppg_auto_gain_ctrl(ecppg_dev->dev,
                               &ecppg_dev->led_ctrl,
                               ecppg_dev->sample_cnt,
                               samples[RED_PA_CHANNEL],
                               LED_2);
        }
    }
}

int max86xxx_ecppg_agc_enable(struct max86xxx_dev *sd, int agc_enable)
{
    int                         ret = -1;
    struct max86xxx_ecppg_data *ecppg_dev;
    struct max86xxx_sensor *    sensor;

    if (!sd)
        return ret;

    sensor    = get_sensor_ptr(sd, MAX86XXX_ECPPG_MODE);
    ecppg_dev = sensor->priv;
    ecppg_dev->led_ctrl.agc_is_enabled = agc_enable ? 1 : 0;

    if (ecppg_dev->led_ctrl.agc_is_enabled)
        ret = led_prox_init(ecppg_dev->dev, &ecppg_dev->led_ctrl, true);
    else
        ret = led_daq_init(ecppg_dev->dev, &ecppg_dev->led_ctrl, true);

    return ret;
}

int max86xxx_ecppg_enable(struct max86xxx_sensor *sensor, int enable)
{
    int ret = 0;
    ESP_LOGD(TAG, "Getting max86xxx_ecppg_data ecppg_dev");
    struct max86xxx_ecppg_data *ecppg_dev =
            priv_of(sensor->dev, MAX86XXX_ECPPG_MODE);

#ifdef CONFIG_IDF_TARGET_ESP32
    ESP_LOGD(TAG, "Reseting queue");
    xQueueReset(ecppg_dev->queue);
#else
    queue_reset(&ecppg_dev->queue);
#endif
    if (ecppg_dev && enable) {
        if (ecppg_dev->led_ctrl.agc_is_enabled) {
            ESP_LOGD(TAG, "led_prox_init start");
            ret = led_prox_init(ecppg_dev->dev, &ecppg_dev->led_ctrl, false);
        } else {
            ESP_LOGD(TAG, "led_daq_init start");
            ret = led_daq_init(ecppg_dev->dev, &ecppg_dev->led_ctrl, true);
        }
    }
    return ret;
}

int max86xxx_ecppg_pause(struct max86xxx_sensor *sensor, int pause)
{
    if (pause)
        return max86xxx_i2c_block_write(ecppg_fifo_dis_cfg,
                                        ARRAY_SIZE(ecppg_fifo_dis_cfg));
    else
        return max86xxx_i2c_block_write(ecppg_fifo_en_cfg,
                                        ARRAY_SIZE(ecppg_fifo_en_cfg));
}

int max86xxx_ecppg_reset(struct max86xxx_sensor *sensor)
{
    ESP_LOGD(TAG, "max86xxx_ecppg_reset start");
    struct max86xxx_ecppg_data *ecppg_dev = sensor->priv;

    if (ecppg_dev) {
        ecppg_dev->sample_cnt = 0;
//		led_control_reset(&ecppg_dev->led_ctrl);
#ifdef CONFIG_IDF_TARGET_ESP32
        ESP_LOGD(TAG, "xQueueReset start");
        xQueueReset(ecppg_dev->queue);
        ESP_LOGD(TAG, "xQueueReset done");

#else
        queue_reset(&ecppg_dev->queue);
#endif
    }

    return 0;
}

int max86xxx_ecppg_init(struct max86xxx_dev *sd)
{
    ESP_LOGI(TAG, "Starting max86xxx_ecppg_init");
    int                         err = 0;
    struct max86xxx_ecppg_data *ecppg_dev;
    struct max86xxx_sensor *sensor = get_sensor_ptr(sd, MAX86XXX_ECPPG_MODE);
    ESP_LOGD(TAG, "get_sensor_ptr done");

    ecppg_dev = malloc(sizeof(struct max86xxx_ecppg_data));
    if (ecppg_dev == NULL) {
        printf("%s:%d - No more memory\n", __func__, __LINE__);
        return -1;
    }
    ESP_LOGD(TAG, "max86xxx_ecppg_data memory allocated");

    memset(ecppg_dev, 0, sizeof(struct max86xxx_ecppg_data));
    ecppg_dev->dev = sd;
    sensor->priv   = ecppg_dev;
#ifdef CONFIG_IDF_TARGET_ESP32
    ESP_LOGD(TAG, "xQueueCreate for ecppg_dev");
    ecppg_dev->queue =
            xQueueCreate(MAX86XXX_COMMON_FIFO_SZ, sizeof(ecppg_data_t));
    if (ecppg_dev->queue == NULL) {
        err = ESP_FAIL;
        printf("%s:%d - failed. err: %d\n", __func__, __LINE__, err);
        return err;
    }
#else
    err = queue_init(&ecppg_dev->queue,
                     &sd->q_buffer,
                     sizeof(ecppg_data_t),
                     MAX86XXX_COMMON_FIFO_SZ);
    if (err < 0)
        printf("%s:%d - failed. err: %d\n", __func__, __LINE__, err);

    max86xxx_algo_init();

    led_control_init(&ecppg_dev->led_ctrl);
#endif
    max86xxx_ecppg_reset(sensor);
    return err;
}

int max86xxx_ecppg_remove(struct max86xxx_sensor *sensor)
{
    free(sensor->priv);
    return 0;
}

struct max86xxx_sensor ecppg_sensor = {
    .name             = "ecppg",
    .init             = max86xxx_ecppg_init,
    .remove           = max86xxx_ecppg_remove,
    .reset            = max86xxx_ecppg_reset,
    .is_sensor_active = max86xxx_is_ecppg_active,
    .enable           = max86xxx_ecppg_enable,
    .pause            = max86xxx_ecppg_pause,
    .report           = max86xxx_ecppg_report,
    .reg_settings     = ecppg_init_cfg,
    .regmap_len       = ARRAY_SIZE(ecppg_init_cfg),
};
#endif //MAX86XXX_ECPPG_ENABLED
