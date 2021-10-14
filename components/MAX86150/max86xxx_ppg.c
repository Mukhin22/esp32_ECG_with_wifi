/*
 * Copyright (c) 2014 Maxim Integrated Products, Inc.
 *
 * Author: Ismail H. Kose <Ismail.Kose@maximintegrated.com>,
 *
 * This software is licensed under the terms of the GNU General Public
 * License, as published by the Free Software Foundation, and
 * may be copied, distributed, and modified under those terms.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 51 Franklin St, Fifth Floor, Boston, MA
 * 02110-1301 USA
 */

#include "max86xxx_core.h"
#include "max86xxx_ppg.h"

#if defined(MAX86XXX_PPG_ENABLED)

#ifdef USE_MAX86903_REGMAP
static const struct regmap ppg_init_cfg[] = {
    { MAX86903_REG_SYSTEM_CTRL, MAX86903_SYSTEM_FIFO_EN },
    { MAX86903_REG_PPG_CFG2, 0x00 },
    { MAX86903_REG_FIFO_DATA_CTRL_1, (RED_PA_CHANNEL << 4) | IR_PA_CHANNEL },
    { MAX86903_REG_FIFO_CONFIG,
      MAX86903_FIFO_ROLLS_ON_FIFO | MAX86903_FIFO_A_FULL_F },
};
#elif defined(SENSOR_MAX86908)
static const struct regmap ppg_init_cfg[] = {
    { MAX86908_REG_SYSTEM_CTRL, MAX86908_SYSTEM_FIFO_EN },
    { MAX86908_REG_FIFO_DATA_CTRL_1, RED_PA_CHANNEL << 4 | IR_PA_CHANNEL },
    { MAX86908_REG_PPG_CFG_2, 0x00 },
    { MAX86908_REG_FIFO_CFG,
      MAX86908_FIFO_ROLLS_ON_FULL | MAX86908_FIFO_A_FULL_MASK },
};
#elif defined(SENSOR_MAX30110)
static const struct regmap ppg_init_cfg[] = {
    { MAX30110_REG_SYSTEM_CTRL, MAX30110_SYSTEM_FIFO_EN },
#if defined DUAL_LED
    { MAX30110_REG_FIFO_DATA_CTRL_1, DUAL_LED_CHANNEL },
#else
    { MAX30110_REG_FIFO_DATA_CTRL_1, RED_PA_CHANNEL << 4 | IR_PA_CHANNEL },
#endif
    { MAX30110_REG_FIFO_DATA_CTRL_2, 0x00 },
    { MAX30110_REG_PPG_CFG_2, 0x00 },
    { MAX30110_REG_FIFO_CFG,
      MAX30110_FIFO_ROLLS_ON_FULL | MAX30110_FIFO_A_FULL_MASK },
};
#else
#warning "MAX86XXX register map is not defined"
#endif

char max86xxx_is_ppg_active(struct max86xxx_dev *sd,
                            u16                  fd_settings,
                            int                  num_ch)
{
#if defined(SENSOR_MAX30110) && defined(DUAL_LED)
    return (num_ch == 1) &&
           max86xxx_is_ch_active(fd_settings, DUAL_LED_CHANNEL);
#elif defined(SENSOR_MAX86908) || defined(SENSOR_MAX30110)
    return (num_ch == 2) && max86xxx_is_ch_active(fd_settings, IR_PA_CHANNEL) &&
           max86xxx_is_ch_active(fd_settings, RED_PA_CHANNEL);
#endif
}

static int max86xxx_set_hr_range(struct max86xxx_dev *sd, int range_index)
{
    struct max86xxx_ppg_data *ppg_dev = priv_of(sd, MAX86XXX_PPG_MODE);
    int                       ret;
    static const u8           range_list[] = {
        MAX86XXX_PPG_ADC_RGE_4096,
        MAX86XXX_PPG_ADC_RGE_8192,
        MAX86XXX_PPG_ADC_RGE_16384,
        MAX86XXX_PPG_ADC_RGE_32768,
    };

    if (range_index > 3 || range_index < 0)
        return -EINVAL;

    printf("%s - ppg_range = %d\n", __func__, ppg_dev->hr_range);
    ret = update_bits(MAX86XXX_REG_PPG_CFG_1,
                      MAX86XXX_MASK_PPG_ADC_RGE,
                      range_list[range_index]);
    if (ret < 0)
        return ret;

    return 0;
}

int max86xxx_ppg_eol_test_control(struct max86xxx_sensor *sensor)
{
    return 0;
}

int max86xxx_get_ppg_data(struct max86xxx_dev *dev, ppg_data_t *ppg_data)
{
    int                       ret;
    struct max86xxx_sensor *  sensor;
    struct max86xxx_ppg_data *ppg_dev;

    sensor  = get_sensor_ptr(dev, MAX86XXX_PPG_MODE);
    ppg_dev = sensor->priv;
    ret     = dequeue(&ppg_dev->queue, ppg_data);
    return ret;
}

void ppg_data_data_report(struct max86xxx_sensor *sensor, int *samples)
{
    struct max86xxx_ppg_data *ppg_dev = sensor->priv;
    int                       ret;
    ppg_data_t                ppg_data;

    ppg_dev->avg_cnt++;
#if defined(SENSOR_MAX30110) && defined(DUAL_LED)
    ppg_dev->green_sum += samples[DUAL_LED_CHANNEL];
#elif defined(SENSOR_MAX86908) || defined(SENSOR_MAX30110)
    ppg_dev->ir_sum += samples[IR_PA_CHANNEL];
    ppg_dev->red_sum += samples[RED_PA_CHANNEL];
#else
#warning "Sensor type is not defined"
#endif

#if defined(SENSOR_MAX30110) && defined(DUAL_LED)
    ppg_data.green = ppg_dev->green_sum;
#elif defined(SENSOR_MAX86908) || defined(SENSOR_MAX30110)
    ppg_data.ir      = ppg_dev->ir_sum;
    ppg_data.red     = ppg_dev->red_sum;
#else
#warning "Sensor type is not defined"
#endif
    ret = enqueue(&ppg_dev->queue, &ppg_data);
    if (ret != 0) {
        if (ret == -2)
            printf("%s:%d FIFO full. ret: %d\n", __func__, __LINE__, ret);
        else
            printf("%s:%d failed. ret: %d\n", __func__, __LINE__, ret);
    }

#if defined(SENSOR_MAX30110) && defined(DUAL_LED)
    ppg_dev->green_sum = 0;
#elif defined(SENSOR_MAX86908) || defined(SENSOR_MAX30110)
    ppg_dev->ir_sum  = 0;
    ppg_dev->red_sum = 0;
#else
#warning "Sensor type is not defined"
#endif
}

void max86xxx_ppg_report_handler(struct max86xxx_sensor *sensor, int *samples)
{
    struct max86xxx_ppg_data *ppg_dev  = sensor->priv;
    struct led_control *      led_ctrl = &ppg_dev->led_ctrl;
    int                       ret;

    ppg_dev->sample_cnt++;
    if (ppg_dev->eol_test_is_enabled) {
        ret = max86xxx_ppg_eol_test_control(sensor);
        if (ret < 0)
            printf("%s - EOL failed, ret: %d\n", __func__, ret);
    } else {
#if defined(SENSOR_MAX30110) && defined(DUAL_LED)
        if (led_ctrl->agc_is_enabled) {
            ret = led_control_sm(
                    ppg_dev->dev, led_ctrl, samples[DUAL_LED_CHANNEL], true);
            if (ret == LED_DATA_ACQ) {
                ppg_auto_gain_ctrl(ppg_dev->dev,
                                   led_ctrl,
                                   ppg_dev->sample_cnt,
                                   samples[DUAL_LED_CHANNEL],
                                   LED_1);
                ppg_auto_gain_ctrl(ppg_dev->dev,
                                   led_ctrl,
                                   ppg_dev->sample_cnt,
                                   samples[DUAL_LED_CHANNEL],
                                   LED_2);
            }
            ppg_data_data_report(sensor, samples);
        } else
            ppg_data_data_report(sensor, samples);
    }

#elif defined(SENSOR_MAX86908) || \
        (defined(SENSOR_MAX30110) && !defined(DUAL_LED))
        if (led_ctrl->agc_is_enabled) {
            ret = led_control_sm(
                    ppg_dev->dev, led_ctrl, samples[IR_PA_CHANNEL], true);
            if (ret == LED_DATA_ACQ) {
                ppg_auto_gain_ctrl(ppg_dev->dev,
                                   led_ctrl,
                                   ppg_dev->sample_cnt,
                                   samples[IR_PA_CHANNEL],
                                   LED_1);
                ppg_auto_gain_ctrl(ppg_dev->dev,
                                   led_ctrl,
                                   ppg_dev->sample_cnt,
                                   samples[RED_PA_CHANNEL],
                                   LED_2);
            }
            ppg_data_data_report(sensor, samples);
        } else
            ppg_data_data_report(sensor, samples);
    }
#else
#warning "Sensor type is not defined"
#endif
}

int max86xxx_ppg_reset(struct max86xxx_sensor *sensor)
{
    struct max86xxx_ppg_data *ppg_dev = sensor->priv;
    ppg_dev->sample_cnt               = 0;
#if defined(SENSOR_MAX30110) && defined(DUAL_LED)
    ppg_dev->green_sum = 0;
#elif defined(SENSOR_MAX86908) || defined(SENSOR_MAX30110)
    ppg_dev->ir_sum  = 0;
    ppg_dev->red_sum = 0;
#else
#warning "Sensor type is not defined"
#endif
    ppg_dev->avg_cnt         = 0;
    ppg_dev->hr_range        = 0;
    ppg_dev->look_mode_ir    = 0;
    ppg_dev->look_mode_red   = 0;
    ppg_dev->eol_test_status = 0;
    queue_reset(&ppg_dev->queue);
    led_control_reset(&ppg_dev->led_ctrl);
    return 0;
}

int max86xxx_ppg_agc_enable(struct max86xxx_dev *sd, int agc_enable)
{
    int                       ret = -1;
    struct max86xxx_ppg_data *ppg_dev;
    struct max86xxx_sensor *  sensor;

    if (!sd)
        return ret;

    sensor                           = get_sensor_ptr(sd, MAX86XXX_PPG_MODE);
    ppg_dev                          = sensor->priv;
    ppg_dev->led_ctrl.agc_is_enabled = agc_enable ? 1 : 0;

    if (ppg_dev->led_ctrl.agc_is_enabled)
        ret = led_prox_init(ppg_dev->dev, &ppg_dev->led_ctrl, true);
    else
        ret = led_daq_init(ppg_dev->dev, &ppg_dev->led_ctrl, true);

    return ret;
}

int max86xxx_ppg_enable(struct max86xxx_sensor *sensor, int enable)
{
    int                       ret     = 0;
    struct max86xxx_ppg_data *ppg_dev = sensor->priv;

    if (enable) {
        //ppg_dev->led_ctrl.agc_is_enabled = 1;
        if (ppg_dev->led_ctrl.agc_is_enabled)
            ret = led_prox_init(ppg_dev->dev, &ppg_dev->led_ctrl, true);
        else
            ret = led_daq_init(ppg_dev->dev, &ppg_dev->led_ctrl, true);
    }

    return ret;
}

int max86xxx_ppg_init(struct max86xxx_dev *sd)
{
    int                       err = 0;
    struct max86xxx_ppg_data *ppg_dev;
    struct max86xxx_sensor *  sensor = get_sensor_ptr(sd, MAX86XXX_PPG_MODE);

    ppg_dev = malloc(sizeof(struct max86xxx_ppg_data));
    if (ppg_dev == NULL) {
        printf("%s:%d - No more memory\n", __func__, __LINE__);
        return -1;
    }

    memset(ppg_dev, 0, sizeof(struct max86xxx_ppg_data));
    ppg_dev->dev = sd;
    sensor->priv = ppg_dev;
    err          = queue_init(&ppg_dev->queue,
                     &sd->q_buffer,
                     sizeof(ppg_data_t),
                     MAX86XXX_COMMON_FIFO_SZ);
    if (err < 0)
        printf("%s:%d failed.\n", __func__, __LINE__);
    led_control_init(&ppg_dev->led_ctrl);
    max86xxx_ppg_reset(sensor);
    return err;
}

int max86xxx_ppg_remove(struct max86xxx_sensor *sensor)
{
    free(sensor->priv);
    return 0;
}

struct max86xxx_sensor ppg_sensor = {
    .name             = "ppg",
    .init             = max86xxx_ppg_init,
    .enable           = max86xxx_ppg_enable,
    .remove           = max86xxx_ppg_remove,
    .reset            = max86xxx_ppg_reset,
    .is_sensor_active = max86xxx_is_ppg_active,
    .report           = max86xxx_ppg_report_handler,
    .reg_settings     = ppg_init_cfg,
    .regmap_len       = ARRAY_SIZE(ppg_init_cfg),
};
#endif //MAX86XXX_PPG_ENABLED
