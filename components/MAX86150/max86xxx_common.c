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

#ifdef USE_MAX86903_REGMAP
static const struct regmap low_pm_settings[] = {
    /* 400 Hz, ADC_RGE= 0, PD_MUX=0, LED_PW=400us -- 0x0F */
    { MAX86903_REG_PPG_CFG1,
      MAX86903_PPG_ADC_RGE_16384 | MAX86903_PPG_SR_50HZ |
              MAX86903_PPG_LED_PW_400US },
    { MAX86903_REG_INT_ENABLE1, A_FULL_MASK },
};

static const struct regmap non_lpm_settings[] = {
    /* 400 Hz, ADC_RGE= 0, PD_MUX=0, LED_PW=400us -- 0x0F */
    { MAX86903_REG_PPG_CFG1,
      MAX86903_PPG_ADC_RGE_16384 | MAX86903_PPG_SR_400HZ |
              MAX86903_PPG_LED_PW_400US },
    { MAX86903_REG_INT_ENABLE1, A_FULL_MASK },
};

#elif defined(SENSOR_MAX86908)
static const struct regmap low_pm_settings[] = {
    { MAX86908_REG_PPG_CFG_1,
      MAX86908_PPG_ADC_RGE_32768 | MAX86908_PPG_SR_10HZ |
              MAX86908_PPG_LED_PW_400US },
    { MAX86908_REG_INT_ENABLE_1, PPG_RDY_MASK },
    { MAX86908_REG_LED1_PA, MAX86908_LED1_LOW_VAL },
    { MAX86908_REG_LED2_PA, MAX86908_LED2_LOW_VAL },

};

static const struct regmap non_lpm_settings[] = {
    { MAX86908_REG_PPG_CFG_1,
      MAX86908_PPG_ADC_RGE_32768 | MAX86908_PPG_SR_100HZ |
              MAX86908_PPG_LED_PW_400US },
    { MAX86908_REG_INT_ENABLE_1, A_FULL_MASK },
    { MAX86908_REG_LED1_PA, MAX86908_LED1_VAL },
    { MAX86908_REG_LED2_PA, MAX86908_LED2_VAL },
};
#elif defined(SENSOR_MAX30110)

#if defined DUAL_LED
static const struct regmap low_pm_settings[] = {
    { MAX30110_REG_PPG_CFG_1,
      MAX30110_PPG_ADC_RGE_8192 //MAX30110_PPG_ADC_RGE_32768 //Ozan
              | MAX30110_PPG_SR_20HZ | MAX30110_PPG_LED_PW_400US },
    { MAX30110_REG_INT_ENABLE_1, PPG_RDY_MASK },
    { MAX30110_REG_LED1_PA, MAX30110_LED1_LOW_VAL },
    { MAX30110_REG_LED2_PA, MAX30110_LED2_LOW_VAL },
};

static const struct regmap non_lpm_settings[] = {
    { MAX30110_REG_PPG_CFG_1,
      MAX30110_PPG_ADC_RGE_8192 //MAX30110_PPG_ADC_RGE_32768 //Ozan
              | MAX30110_PPG_SR_400HZ | MAX30110_PPG_LED_PW_400US },
    { MAX30110_REG_INT_ENABLE_1, A_FULL_MASK },
    { MAX30110_REG_LED1_PA, MAX30110_LED1_VAL },
    { MAX30110_REG_LED2_PA, MAX30110_LED2_VAL },
};
#elif defined(SENSOR_MAX86908) || defined(SENSOR_MAX30110)
static const struct regmap low_pm_settings[] = {
    { MAX30110_REG_PPG_CFG_1,
      MAX30110_PPG_ADC_RGE_32768 | MAX30110_PPG_SR_20HZ |
              MAX30110_PPG_LED_PW_400US },
    { MAX30110_REG_INT_ENABLE_1, PPG_RDY_MASK },
    { MAX30110_REG_LED1_PA, MAX30110_LED1_LOW_VAL },
    { MAX30110_REG_LED2_PA, MAX30110_LED2_LOW_VAL },
};

static const struct regmap non_lpm_settings[] = {
    { MAX30110_REG_PPG_CFG_1,
      MAX30110_PPG_ADC_RGE_32768 | MAX30110_PPG_SR_400HZ |
              MAX30110_PPG_LED_PW_400US },
    { MAX30110_REG_INT_ENABLE_1, A_FULL_MASK },
    { MAX30110_REG_LED1_PA, MAX30110_LED1_VAL },
    { MAX30110_REG_LED2_PA, MAX30110_LED2_VAL },
};
#else
#warning "MAX86XXX register map is not defined"
#endif

#else
#warning "MAX86XXX register map is not defined"
#endif

int update_led_range(int              new_range,
                     int              led_num,
                     union led_range *led_range_settings)
{
    int old_range;

    //printf("update_led_range led%d->%d\r\n", led_num, new_range);

    switch (led_num) {
    case LED_1:
        old_range                = led_range_settings->led1;
        led_range_settings->led1 = new_range;
        break;
    case LED_2:
        old_range                = led_range_settings->led2;
        led_range_settings->led2 = new_range;
        break;
    case LED_3:
        old_range                = led_range_settings->led3;
        led_range_settings->led3 = new_range;
        break;
    default:
        return -EINVAL;
    }

    if (old_range == new_range)
        return 0;

    return max86xxx_write_reg(MAX86XXX_REG_LED_RGE, led_range_settings->val);
}

int update_led_current(union led_range *led_range_settings,
                       int              led_new_val,
                       int              led_num)
{
    //printf("update_led_current led%d->%d\r\n", led_num, led_new_val);
    int ret                  = 0;
    u8  led_current_reg_addr = MAX86XXX_REG_LED1_PA + led_num;
    int led_range;
    u8  led_current_reg_val;
    int led_range_index = led_new_val / 50000;
    const int
            led_range_steps[] = {
                LED_RANGE_STEP_200uA, LED_RANGE_STEP_400uA,
                LED_RANGE_STEP_600uA, LED_RANGE_STEP_800uA,
                LED_RANGE_STEP_800uA, /* For led current greater than 200 */
            };

    if (led_new_val < MIN_LED_DRIVE_CURRENT ||
        led_new_val > MAX_LED_DRIVE_CURRENT) {
        printf("Invalid led value: %d\n", led_new_val);
        return -EINVAL;
    }

    if (led_num < LED_1 || led_num > LED_3) {
        printf("Invalid led number: %d\n", led_num);
        return -EINVAL;
    }

    led_current_reg_val = led_new_val / led_range_steps[led_range_index];
    ret = max86xxx_write_reg(led_current_reg_addr, led_current_reg_val);
    if (ret < 0)
        return ret;

    led_range = led_range_index;
    if (led_range > 3)
        led_range = 3;
    ret = update_led_range(led_range, led_num, led_range_settings);
    if (ret < 0)
        return ret;
    return ret;
}

s32 agc_adj_calculator(s32 *change_by_percent_of_range,
                       s32 *change_by_percent_of_current_setting,
                       s32 *change_led_by_absolute_count,
                       s32 *set_led_to_absolute_count,
                       s32  target_percent_of_range,
                       s32  correction_coefficient,
                       s32  allowed_error_in_percentage,
                       s32  current_average,
                       s32  number_of_samples_averaged,
                       s32  led_drive_current_value)
{
    s32 current_percent_of_range = 0;
    s32 delta                    = 0;
    s32 desired_delta            = 0;
    s32 current_power_percent    = 0;

    if (change_by_percent_of_range == 0 ||
        change_by_percent_of_current_setting == 0 ||
        change_led_by_absolute_count == 0 || set_led_to_absolute_count == 0)
        return ILLEGAL_OUTPUT_POINTER;

    if (target_percent_of_range > 90 || target_percent_of_range < 10)
        return CONSTRAINT_VIOLATION;

    if (correction_coefficient > 100 || correction_coefficient < 0)
        return CONSTRAINT_VIOLATION;

    if (allowed_error_in_percentage > 100 || allowed_error_in_percentage < 0)
        return CONSTRAINT_VIOLATION;

#if ((MAX_PPG_DIODE_VAL - MIN_PPG_DIODE_VAL) <= 0 || \
     (MAX_PPG_DIODE_VAL < 0) || (MIN_PPG_DIODE_VAL < 0))
#error "Illegal diode Min/Max Pair"
#endif

#if ((MAX_LED_DRIVE_CURRENT - MIN_LED_DRIVE_CURRENT) <= 0 || \
     (MAX_LED_DRIVE_CURRENT < 0) || (MIN_LED_DRIVE_CURRENT < 0))
#error "Illegal LED Min/Max current Pair"
#endif

    if (led_drive_current_value > MAX_LED_DRIVE_CURRENT ||
        led_drive_current_value < MIN_LED_DRIVE_CURRENT)
        return CONSTRAINT_VIOLATION;

    if (current_average < MIN_PPG_DIODE_VAL ||
        current_average > MAX_PPG_DIODE_VAL)
        return CONSTRAINT_VIOLATION;

    current_percent_of_range = 100 * (current_average - MIN_PPG_DIODE_VAL) /
                               (MAX_PPG_DIODE_VAL - MIN_PPG_DIODE_VAL);

    delta = current_percent_of_range - target_percent_of_range;
    delta = delta * correction_coefficient / 100;

    if (delta > -allowed_error_in_percentage &&
        delta < allowed_error_in_percentage) {
        *change_by_percent_of_range           = 0;
        *change_by_percent_of_current_setting = 0;
        *change_led_by_absolute_count         = 0;
        *set_led_to_absolute_count            = led_drive_current_value;
        return 0;
    }

    current_power_percent = 100 *
                            (led_drive_current_value - MIN_LED_DRIVE_CURRENT) /
                            (MAX_LED_DRIVE_CURRENT - MIN_LED_DRIVE_CURRENT);
    if (delta < 0)
        desired_delta = -delta * (100 - current_power_percent) /
                        (100 - current_percent_of_range);

    if (delta > 0)
        desired_delta =
                -delta * (current_power_percent) / (current_percent_of_range);

    *change_by_percent_of_range = desired_delta;

    *change_led_by_absolute_count =
            (desired_delta * LED_DRIVE_CURRENT_FULL_SCALE / 100);
    *change_by_percent_of_current_setting =
            (*change_led_by_absolute_count * 100) / (led_drive_current_value);
    *set_led_to_absolute_count =
            led_drive_current_value + *change_led_by_absolute_count;
    return 0;
}

void ppg_auto_gain_ctrl(struct max86xxx_dev *dev,
                        struct led_control * led_ctrl,
                        u32                  sample_cnt,
                        int                  diode_data,
                        int                  led_num)
{
    int ret;
    int diode_avg;

    if (led_num < LED_1 || led_num > LED_3)
        return;

    led_ctrl->diode_sum[led_num] += diode_data;
    if (sample_cnt % led_ctrl->agc_min_num_samples == 0) {
        diode_avg =
                led_ctrl->diode_sum[led_num] / led_ctrl->agc_min_num_samples;
        led_ctrl->diode_sum[led_num] = 0;
    } else
        return;

    ret = agc_adj_calculator(
            &led_ctrl->change_by_percent_of_range[led_num],
            &led_ctrl->change_by_percent_of_current_setting[led_num],
            &led_ctrl->change_led_by_absolute_count[led_num],
            &led_ctrl->led_current[led_num],
            led_ctrl->agc_led_out_percent,
            led_ctrl->agc_corr_coeff,
            led_ctrl->agc_sensitivity_percent,
            diode_avg,
            led_ctrl->agc_min_num_samples,
            led_ctrl->led_current[led_num]);
    if (ret)
        return;

    if (led_ctrl->change_led_by_absolute_count[led_num] == 0)
        return;

    ret = update_led_current(&led_ctrl->led_range_settings,
                             led_ctrl->led_current[led_num],
                             led_num);
    if (ret < 0)
        printf("%s failed\n", __func__);
    return;
}

int led_prox_init(struct max86xxx_dev *dev,
                  struct led_control * led_ctrl,
                  char                 lpm)
{
    int ret;
    led_ctrl->led_current[LED_1] = DEFAULT_PROX_LED_CURRENT_1;
    ret                          = update_led_current(
            &led_ctrl->led_range_settings, led_ctrl->led_current[LED_1], LED_1);

    led_ctrl->led_current[LED_2] = DEFAULT_PROX_LED_CURRENT_2;
    ret |= update_led_current(
            &led_ctrl->led_range_settings, led_ctrl->led_current[LED_2], LED_2);
    if (lpm)
        ret |= max86xxx_i2c_block_write(low_pm_settings,
                                        ARRAY_SIZE(low_pm_settings));

    return ret;
}

int led_daq_init(struct max86xxx_dev *dev,
                 struct led_control * led_ctrl,
                 char                 lpm)
{
    int ret;

    led_ctrl->led_current[LED_1] = DEFAULT_DAQ_LED_CURRENT_1;
    ret                          = update_led_current(
            &led_ctrl->led_range_settings, led_ctrl->led_current[LED_1], LED_1);

    led_ctrl->led_current[LED_2] = DEFAULT_DAQ_LED_CURRENT_2;
    ret |= update_led_current(
            &led_ctrl->led_range_settings, led_ctrl->led_current[LED_2], LED_2);

    if (lpm)
        ret |= max86xxx_i2c_block_write(non_lpm_settings,
                                        ARRAY_SIZE(non_lpm_settings));

    return ret;
}

int led_control_sm(struct max86xxx_dev *dev,
                   struct led_control * led_ctrl,
                   int                  diode_data,
                   char                 lpm)
{
    int ret = led_ctrl->state;
    int avg = 0;

    led_ctrl->prox_sample_cnt++;
    led_ctrl->prox_sum += diode_data;

    switch (led_ctrl->state) {
    case LED_PROX:
        if (led_ctrl->prox_sample_cnt % PROX_DEBOUNCE_SPS != 0)
            break;

        avg = led_ctrl->prox_sum / PROX_DEBOUNCE_SPS;
        if (avg >= PROX_THRESHOLD_1) {
            //printf("begin acq mode\r\n");
            led_ctrl->state           = LED_DATA_ACQ;
            ret                       = led_daq_init(dev, led_ctrl, lpm);
            led_ctrl->prox_sample_cnt = 0;
        }
        led_ctrl->prox_sum = 0;
        break;

    case LED_DATA_ACQ:
        if (led_ctrl->prox_sample_cnt % DAQ_DEBOUNCE_SPS != 0)
            break;

        avg = led_ctrl->prox_sum / DAQ_DEBOUNCE_SPS;
        if (avg <= PROX_THRESHOLD_2) {
            //printf("begin prox mode\r\n");
            led_ctrl->state           = LED_PROX;
            ret                       = led_prox_init(dev, led_ctrl, lpm);
            led_ctrl->prox_sample_cnt = 0;
        }
        led_ctrl->prox_sum = 0;
        break;

    default:
        led_ctrl->state           = LED_PROX;
        led_ctrl->prox_sum        = 0;
        led_ctrl->prox_sample_cnt = 0;
        return -EINVAL;
    }

    return ret;
}

void led_control_reset(struct led_control *led_ctrl)
{
    led_ctrl->led_current[LED_1] = led_ctrl->default_current[LED_1];
    led_ctrl->led_current[LED_2] = led_ctrl->default_current[LED_2];
    led_ctrl->led_current[LED_3] = led_ctrl->default_current[LED_3];

    memset(led_ctrl->change_by_percent_of_range,
           0,
           sizeof(led_ctrl->change_by_percent_of_range));
    memset(led_ctrl->change_by_percent_of_current_setting,
           0,
           sizeof(led_ctrl->change_by_percent_of_range));
    memset(led_ctrl->change_led_by_absolute_count,
           0,
           sizeof(led_ctrl->change_by_percent_of_range));
    memset(led_ctrl->diode_sum, 0, sizeof(led_ctrl->diode_sum));

    led_ctrl->prox_sum        = 0;
    led_ctrl->prox_sample_cnt = 0;
    led_ctrl->state           = LED_PROX;
}

void led_control_init(struct led_control *led_ctrl)
{
    memset(led_ctrl, 0, sizeof(struct led_control));

    led_ctrl->default_current[LED_1] = MAX86XXX_DEFAULT_CURRENT1_VAL;
    led_ctrl->default_current[LED_2] = MAX86XXX_DEFAULT_CURRENT2_VAL;
    led_ctrl->default_current[LED_3] = MAX86XXX_DEFAULT_CURRENT3_VAL;
#ifdef SENSOR_MAX86908
    led_ctrl->agc_led_out_percent = AGC_DEFAULT_LED_OUT_RANGE_IR_RED;
#elif defined(SENSOR_MAX30110)
    led_ctrl->agc_led_out_percent = AGC_DEFAULT_LED_OUT_RANGE_GREEN;
#else
#warning "MAX86XXX register map is not defined"
#endif
    led_ctrl->agc_corr_coeff          = AGC_DEFAULT_CORRECTION_COEFF;
    led_ctrl->agc_min_num_samples     = AGC_DEFAULT_MIN_NUM_PERCENT;
    led_ctrl->agc_sensitivity_percent = AGC_DEFAULT_SENSITIVITY_PERCENT;
}

int max86xxx_get_ecg_imp_sample_rate(int *sample_rate)
{
    int ret;
    u8  reg;

    const u16 sr_array[] = { 1600, 800, 400, 200, 3200, 1600, 800, 400 };

    reg = MAX86XXX_REG_ECG_ETI_CFG1;
    ret = max86xxx_read_reg(&reg, 1);
    if (ret < 0)
        return ret;

    reg &= 7;
    *sample_rate = sr_array[reg];

    return ret;
}

int max86xxx_get_ppg_flicker_sample_rate(int *sample_rate)
{
    int       ret = 0;
    u8        reg;
    const u16 sr_array[] = { 10, 20, 50, 100, 200, 400, 800, 1000, 1600, 3200 };

    reg = MAX86XXX_REG_PPG_CFG_1;
    ret = max86xxx_read_reg(&reg, 1);
    if (ret != 0) {
        printf("Unable to read ppg data with I2C. %s:%d - ret: %d\n",
               __func__,
               __LINE__,
               ret);
        return ret;
    }

    *sample_rate = sr_array[(reg >> MAX86XXX_IR_RED_SR_OFFSET) &
                            MAX86XXX_IR_RED_SR_MASK];
    return ret;
}

static void max86xxx_get_device_id(struct max86xxx_dev *sd,
                                   u8 *                 part_id,
                                   u8 *                 rev_id,
                                   unsigned long long * device_id)
{
    u8  i2cbuf[4];
    int err       = 0;
    int reg_state = sd->regulator_state;

    if (reg_state == PWR_OFF)
        err |= max86xxx_regulator_onoff(sd, PWR_ON);

    if (err)
        goto device_id_fail;

    *part_id = MAX86XXX_REG_WHOAMI_REG_PART_ID;
    err |= max86xxx_read_reg(part_id, 1);

    *rev_id = MAX86XXX_REG_WHOAMI_REG_REV;
    err |= max86xxx_read_reg(rev_id, 1);

    err |= max86xxx_write_reg(MAX86XXX_REG_TM, MAX86XXX_TM_ENTER1);
    err |= max86xxx_write_reg(MAX86XXX_REG_TM, MAX86XXX_TM_ENTER2);

    i2cbuf[0] = 0xA7;
    err |= max86xxx_read_reg(i2cbuf, 4);
    err |= max86xxx_write_reg(MAX86XXX_REG_TM, 0);

    *device_id = i2cbuf[0];
    *device_id <<= 8;
    *device_id |= i2cbuf[1];
    *device_id <<= 8;
    *device_id |= i2cbuf[2];
    *device_id <<= 8;
    *device_id |= i2cbuf[3];

    if (reg_state == PWR_OFF)
        err |= max86xxx_regulator_onoff(sd, PWR_OFF);

    if (err)
        goto device_id_fail;

    return;

device_id_fail:
    *part_id   = 0;
    *rev_id    = 0;
    *device_id = 0;
}
