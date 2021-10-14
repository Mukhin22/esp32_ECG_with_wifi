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
#include <string.h>
#include "max86xxx_core.h"
#define MAX86XXX_ECG_ENABLED
#include "max86xxx_ecg.h"
#include "max86xxx_algo.h"
#if defined(MAX86XXX_ECG_ENABLED)

#ifdef USE_MAX86903_REGMAP
static const struct regmap ecg_init_cfg[] = {
    { MAX86903_REG_SYSTEM_CTRL, MAX86903_SYSTEM_FIFO_EN },
    { MAX86903_REG_ECG_ETI_CFG1,
      MAX86903_ECG_CIC_3RD_ORDER | MAX86903_ECG_ADC_OSR_2 },
    { MAX86903_REG_ECG_ETI_CFG2,
      MAX86903_IMP_PGA_GAIN_4 | MAX86903_ECG_PGA_GAIN_8 |
              MAX86903_ECG_IA_GAIN_50 },
    { MAX86903_REG_ECG_ETI_CFG3, MAX86903_ETI_AC_CUR_7 },
    { MAX86903_REG_FIFO_DATA_CTRL_1, (IMP_I_CHANNEL << 4) | ECG_CHANNEL },
    { MAX86903_REG_FIFO_DATA_CTRL_2, 0x00 },
    { MAX86903_REG_FIFO_CONFIG,
      MAX86903_FIFO_ROLLS_ON_FIFO | MAX86903_FIFO_A_FULL_F },
    { MAX86903_REG_INT_ENABLE1, A_FULL_MASK },
    { MAX86903_REG_TM, MAX86903_TM_ENTER1 },
    { MAX86903_REG_TM, MAX86903_TM_ENTER2 },
    { MAX86903_REG_ECG_ETI_TEST1,
      MAX86903_ECG_ETI_HPF | MAX86903_ECG_ETI_IN_BIAS_OPT },
    { MAX86903_REG_ECG_ETI_TEST2,
      MAX86903_ECG_ETI_DC_RESTORE_EN | MAX86903_ECG_ETI_F_CHOP_9K6 },
    { MAX86903_REG_ECG_ETI_TEST3, 0x00 },
    { MAX86903_REG_TM, MAX86903_TM_EXIT },
};
#elif defined(SENSOR_MAX86908)
static const struct regmap ecg_init_cfg[] = {
    { MAX86908_REG_SYSTEM_CTRL, MAX86908_SYSTEM_FIFO_EN },
    { MAX86908_REG_ECG_ETI_CFG_1,
      MAX86908_ECG_CIC_ORDER | MAX86908_ECG_ADC_OSR_2 },
    { MAX86908_REG_ECG_ETI_CFG_2, MAX86908_ECG_F_CHOP },
    { MAX86908_REG_ECG_ETI_CFG_3,
      MAX86908_IMP_PGA_GAIN_1 | MAX86908_ECG_PGA_GAIN_8 |
              MAX86908_ECG_IA_GAIN_20 },
    { MAX86908_REG_ECG_ETI_CFG_4, MAX86908_ETI_AC_CUR_MASK },
    { MAX86908_REG_FIFO_DATA_CTRL_1, (IMP_I_CHANNEL << 4) | ECG_CHANNEL },
    { MAX86908_REG_FIFO_DATA_CTRL_2, 0x00 },
    { MAX86908_REG_INT_ENABLE_1, A_FULL_MASK },
    { MAX86908_REG_TM, MAX86908_TM_ENTER1 },
    { MAX86908_REG_TM, MAX86908_TM_ENTER2 },
    { MAX86908_REG_ECG_ETI_TEST1, MAX86908_ECG_ETI_IN_BIAS_OPT },
    { MAX86908_REG_ECG_ETI_TEST2, MAX86908_ECG_ETI_DC_RESTORE_EN },
    { MAX86908_REG_ECG_ETI_TEST3, MAX86908_ETI_CHOP_EN_MASK },
    { MAX86908_REG_TM, MAX86908_TM_EXIT },
};
#elif defined(SENSOR_MAX30110)
static const struct regmap ecg_init_cfg[] = {
    { MAX30110_REG_SYSTEM_CTRL, MAX30110_SYSTEM_FIFO_EN },
    { MAX30110_REG_TM, MAX30110_TM_ENTER1 },
    { MAX30110_REG_TM, MAX30110_TM_ENTER2 },
    { MAX30110_ECG_ETI_TEST_MODE, 0x00 },
    { MAX30110_REG_ECG_ETI_TEST1, MAX30110_BIAS_DIR_2_5MOHM },
    { MAX30110_REG_ECG_ETI_TEST2, MAX30110_ECG_ETI_DC_RESTORE_EN },
    { MAX30110_REG_ECG_ETI_TEST3,
      MAX30110_ETI_CHOP_EN_MASK | MAX30110_BIOZ_BIAS_DIS },
    { MAX30110_REG_ECG_ETI_TEST4, 0x00 },
    { MAX30110_REG_TM, MAX30110_TM_EXIT },

    { MAX30110_REG_FIFO_DATA_CTRL_1, (IMP_I_CHANNEL << 4) | ECG_CHANNEL },
    { MAX30110_REG_FIFO_DATA_CTRL_2, 0x00 },

    { MAX30110_REG_ECG_ETI_CFG_1,
      MAX30110_BIOZ_PHASE_90 | MAX30110_ECG_ADC_OSR_3 },
    { MAX30110_REG_ECG_ETI_CFG_2, 0x00 },
    { MAX30110_REG_ECG_ETI_CFG_3,
      MAX30110_ECG_PGA_GAIN_4 | MAX30110_PGA_ETI_GAIN_4 |
              MAX30110_LEAD_BIAS_100_MOHM },
    { MAX30110_REG_ECG_ETI_CFG_4, MAX30110_ETI_AC_CUR_50NA },
};
#else
#warning "MAX86XXX register map is not defined"
#endif

char max86xxx_is_ecg_active(struct max86xxx_dev *sd,
                            u16                  fd_settings,
                            int                  num_ch)
{
    return (num_ch == 2) && max86xxx_is_ch_active(fd_settings, ECG_CHANNEL) &&
           max86xxx_is_ch_active(fd_settings, IMP_I_CHANNEL);
}

static int __max86xxx_set_ecg_sr(struct max86xxx_dev *sd, u16 sample_rate)
{
    int ret;
    u8  reg = 0;

    switch (sample_rate) {
    case 200:
        reg = MAX86XXX_ECG_CIC_ORDER | MAX86XXX_ECG_ADC_OSR_3;
        break;
    case 400:
        reg = MAX86XXX_ECG_CIC_ORDER | MAX86XXX_ECG_ADC_OSR_2;
        break;
    case 800:
        reg = MAX86XXX_ECG_CIC_ORDER | MAX86XXX_ECG_ADC_OSR_1;
        break;

    default:
        printf("Wrong sample rate: %d\n", sample_rate);
    }

    ret = max86xxx_write_reg(MAX86XXX_REG_ECG_ETI_CFG1, reg);
    if (ret != 0) {
        printf("%s - error initializing MAX86903_REG_LED1_PA!\n", __func__);
        return 1;
    }
    return 0;
}

static int max86xxx_set_ecg_pga_gain(struct max86xxx_dev *sd, u8 pga_gain)
{
    int ret = 0;
    u8  pga_reg;

    switch (pga_gain) {
    case 1:
        pga_reg = MAX86XXX_ECG_PGA_GAIN_1;
        break;
    case 2:
        pga_reg = MAX86XXX_ECG_PGA_GAIN_2;
        break;
    case 4:
        pga_reg = MAX86XXX_ECG_PGA_GAIN_4;
        break;
    case 8:
        pga_reg = MAX86XXX_ECG_PGA_GAIN_8;
        break;
    default:
        pga_reg = MAX86XXX_ECG_PGA_GAIN_1;
        return -EINVAL;
    }

#if defined(SENSOR_MAX86908) || defined(SENSOR_MAX30110)
    ret = update_bits(
            MAX86XXX_REG_ECG_ETI_CFG_3, MAX86XXX_MASK_ECG_PGA_GAIN, pga_reg);
#elif defined(USE_MAX86903_REGMAP)
    ret = update_bits(
            MAX86XXX_REG_ECG_ETI_CFG_2, MAX86XXX_MASK_ECG_PGA_GAIN, pga_reg);
#endif

    if (!ret) {
        printf("Unable to read pga gain. ret: %d\n", ret);
        goto err_i2c_comm;
    }

    return ret;
err_i2c_comm:
    printf("%s failed. ret: %d\n", __func__, ret);
    return ret;
}

static int max86xxx_set_ecg_ia_gain(struct max86xxx_dev *sd, u8 ia_gain)
{
#if defined(SENSOR_MAX86908) || defined(USE_MAX86903_REGMAP)
    int ret;
    u8  ia_reg;

    switch (ia_gain) {
    case 5:
        ia_reg = MAX86XXX_ECG_IA_GAIN_5;
        break;
    case 10:
        ia_reg = MAX86XXX_ECG_IA_GAIN_10;
        break;
    case 20:
        ia_reg = MAX86XXX_ECG_IA_GAIN_20;
        break;
    case 50:
        ia_reg = MAX86XXX_ECG_IA_GAIN_50;
        break;
    default:
        ia_reg = 0;
        return -EINVAL;
    }

#if defined(SENSOR_MAX86908)
    ret = update_bits(
            MAX86XXX_REG_ECG_ETI_CFG_3, MAX86XXX_MASK_ECG_IA_GAIN, ia_reg);
#elif defined(USE_MAX86903_REGMAP)
    ret = update_bits(
            MAX86XXX_REG_ECG_ETI_CFG_2, MAX86XXX_MASK_ECG_IA_GAIN, ia_reg);
#endif
    if (!ret) {
        printf("%s - failed. ret: %d\n", __func__, ret);
        return ret;
    }

    return 0;
#else
    return -1;
#endif
}

#if defined(SENSOR_MAX86908) || defined(USE_MAX86903_REGMAP)
static int max86xxx_ecg_set_hpf(struct max86xxx_dev *sd, u8 hpf)
{
    int ret;
    u8  val;

    ret = max86xxx_write_reg(MAX86XXX_REG_TM, MAX86XXX_TM_ENTER1);
    if (ret < 0)
        goto err_i2c_comm;
    ret = max86xxx_write_reg(MAX86XXX_REG_TM, MAX86XXX_TM_ENTER2);
    if (ret < 0)
        goto err_i2c_comm;

    if (hpf)
        val = MAX86XXX_ECG_ETI_HPF;
    else
        val = 0;

    ret = update_bits(
            MAX86XXX_REG_ECG_ETI_TEST1, MAX86XXX_MASK_ECG_ETI_HPF, val);
    if (ret < 0)
        goto err_i2c_comm;

    ret = max86xxx_write_reg(MAX86XXX_REG_TM, MAX86XXX_TM_EXIT);
    if (ret < 0)
        goto err_i2c_comm;
    return 0;
err_i2c_comm:
    printf("I2c comm failed. ret:%d, line: %d\n", ret, __LINE__);
    return ret;
}
#endif

static int max86xxx_ecg_get_hpf(struct max86xxx_dev *sd)
{
#if defined(SENSOR_MAX86908) || defined(USE_MAX86903_REGMAP)
    int ret;
    u8  reg;

    ret = max86xxx_write_reg(MAX86XXX_REG_TM, MAX86XXX_TM_ENTER1);
    if (ret < 0)
        goto err_i2c_comm;
    ret = max86xxx_write_reg(MAX86XXX_REG_TM, MAX86XXX_TM_ENTER2);
    if (ret < 0)
        goto err_i2c_comm;

    reg = MAX86XXX_REG_ECG_ETI_TEST1;
    ret = max86xxx_read_reg(&reg, 1);
    if (ret < 0)
        goto err_i2c_comm;

    ret = max86xxx_write_reg(MAX86XXX_REG_TM, MAX86XXX_TM_EXIT);
    if (ret < 0)
        goto err_i2c_comm;
    return !!(reg & MAX86XXX_MASK_ECG_ETI_HPF);

err_i2c_comm:
    printf("%s failed\n", __func__);
    return ret;
#else
    return -EPERM;
#endif
}

int max86xxx_get_ecg_data(struct max86xxx_dev *dev, ecg_data_t *ecg_data)
{
    struct max86xxx_ecg_data *ecg_dev;
    struct max86xxx_sensor *  sensor =
            get_sensor_ptr(dev, MAX86XXX_ECPPG_MODE /* MAX86XXX_ECG_MODE */);

    ecg_dev = sensor->priv;
    return xQueueReceive(&ecg_dev->queue, ecg_data, (TickType_t)0);
}

void max86xxx_ecg_report(struct max86xxx_sensor *sensor, int *samples)
{
    struct max86xxx_ecg_data *ecg_dev = sensor->priv;
    BaseType_t                ret;
    ecg_data_t                ecg_data;

    ecg_data.data = samples[ECG_CHANNEL];
    ecg_data.cnt  = ecg_dev->sample_cnt++;

    ret = xQueueSend(&ecg_dev->queue, &ecg_data, (TickType_t)0);
    if (ret == pdFALSE)
        printf("%s:%d failed. ret: %d\n", __func__, __LINE__, ret);
}

int max86xxx_ecg_reset(struct max86xxx_sensor *sensor)
{
    struct max86xxx_ecg_data *ecg_dev = sensor->priv;
    int                       ret;

    if (ecg_dev) {
        ecg_dev->sample_cnt = 0;
        xQueueReset(&ecg_dev->queue);
#ifdef CONFIG_IDF_TARGET_ESP32
        ret = max86xxx_ecg_algo_init(&ecg_dev->ecg_cfg);
#endif
        if (ret < 0)
            printf("%s:%d failed - ret: %d\n", __func__, __LINE__, ret);
        return ret;
    }

    return -1;
}

int max86xxx_ecg_init(struct max86xxx_dev *sd)
{
    int                       err = 0;
    struct max86xxx_ecg_data *ecg_dev;
    struct max86xxx_sensor *  sensor =
            get_sensor_ptr(sd, MAX86XXX_ECPPG_MODE /* MAX86XXX_ECG_MODE */);

    ecg_dev = malloc(sizeof(struct max86xxx_ecg_data));
    if (ecg_dev == NULL) {
        printf("%s:%d - No more memory\n", __func__, __LINE__);
        return -1;
    }

    memset(ecg_dev, 0, sizeof(struct max86xxx_ecg_data));
    sensor->priv = ecg_dev;
#ifdef CONFIG_IDF_TARGET_ESP32
    ecg_dev->queue = xQueueCreate(MAX86XXX_COMMON_FIFO_SZ, sizeof(ecg_data_t));
    if (ecg_dev->queue == NULL) {
        err = ESP_FAIL;
        printf("%s:%d - failed. err: %d\n", __func__, __LINE__, err);
        return err;
    }
#else
    err = queue_init(&ecg_dev->queue,
                     &sd->q_buffer,
                     sizeof(ecg_data_t),
                     MAX86XXX_COMMON_FIFO_SZ);
    if (err < 0) {
        printf("%s:%d - failed. err: %d\n", __func__, __LINE__, err);
        return err;
    }
#endif

    return max86xxx_ecg_reset(sensor);
}

int max86xxx_ecg_remove(struct max86xxx_sensor *sensor)
{
    free(sensor->priv);
    return 0;
}

struct max86xxx_sensor ecg_sensor = {
    .name             = "ecg",
    .init             = max86xxx_ecg_init,
    .remove           = max86xxx_ecg_remove,
    .is_sensor_active = max86xxx_is_ecg_active,
    .report           = max86xxx_ecg_report,
    .reg_settings     = ecg_init_cfg,
    .regmap_len       = ARRAY_SIZE(ecg_init_cfg),
};
#endif // MAX86XXX_ECG_ENABLED
