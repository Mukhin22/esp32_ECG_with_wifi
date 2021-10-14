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

#ifdef CONFIG_IDF_TARGET_ESP32
#define LOG_LOCAL_LEVEL ESP_LOG_DEBUG
#define INT_PIN_NUM     17 /* This pin has to be defined*/
#define SPI_READ_CMD
#include "esp_log.h"
#include "esp_err.h"
static const char *TAG = "max86150_core";
#include "../i2c/i2c_med.h"
i2c_port_t i2cm_id_global = I2C_MASTER_NUM;
#else
#include "platform_event.h"
#endif

//extern mxc_spim_regs_t *spim_id_global;
//extern uint8_t spim_slave_select_global;
uint8_t i2c_slave_addr_global = 0xBC; //0b1011110;

//static struct platform_events_t irq_events;

static struct max86xxx_sensor *max86xxx_sensors[] = {
#ifdef MAX86XXX_PPG_ENABLED
    [MAX86XXX_PPG_MODE] = &ppg_sensor,
#endif
#ifdef MAX86XXX_ECG_ENABLED
    [MAX86XXX_ECG_MODE] = &ecg_sensor,
#endif
#ifdef MAX86XXX_ECPPG_ENABLED
    [MAX86XXX_ECPPG_MODE] = &ecppg_sensor,
#endif
#ifdef MAX86XXX_BIOZ_ENABLED
    [MAX86XXX_BIOZ_MODE] = &bioz_sensor,
#endif
#ifdef MAX86XXX_GSR_ENABLED
    [MAX86XXX_GSR_MODE] = &gsr_sensor,
#endif
#ifdef MAX86XXX_FLICKER_ENABLED
    [MAX86XXX_FLICKER_MODE] = &flicker_sensor,
#endif
#ifdef MAX86XXX_BLUE_ENABLED
    [MAX86XXX_BLUE_MODE] = &blue_sensor,
#endif
#ifdef MAX86XXX_EUV_ENABLED
    [MAX86XXX_EUV_MODE] = &uv_sensor,
#endif
#ifdef MAX86XXX_TP_ENABLED
    [MAX86XXX_TP_MODE] = &tp_sensor,
#endif
};

static void *max86xxx_device_mem(void *dev_ptr)
{
    static void *dev_data_ptr;

    if (dev_ptr)
        dev_data_ptr = dev_ptr;
    return dev_data_ptr;
}

void *max86xxx_get_device_data(void)
{
    return max86xxx_device_mem(NULL);
}

static void *max86xxx_set_device_data(void *dev_ptr)
{
    return max86xxx_device_mem(dev_ptr);
}

int max86xxx_write_reg(u8 reg_addr, u8 reg_data)
{
    //#if defined(MAX32620_EvKit_V1) || (MAX32625_EvKit_V1)
    //	uint8_t i2c_buf[2] = {reg_addr, reg_data};
    //	return I2CM_Write(MXC_I2CM1, (MAX86XXX_SLAVE_ADDR >> 1), NULL, 0, i2c_buf, sizeof(i2c_buf)) != sizeof(i2c_buf);
    //#elif defined(Pegasus_V1)
    //	#ifdef SENSOR_MAX30110
    //		return (max86xxx_spi_write_reg(spim_id_global, spim_slave_select_global, reg_addr, reg_data)) != 3;
#if defined(SENSOR_MAX86908)
    uint8_t cmd[2] = { reg_addr, reg_data };
    return I2CM_Write(i2cm_id_global,
                      (i2c_slave_addr_global >> 1),
                      NULL,
                      0,
                      cmd,
                      sizeof(cmd));
#else
#warning "Board:  Sensor type is not defined"
    return -1;
#endif
}

#ifdef USE_SPI
int spi_smbus_read_word_data(u8 reg_addr)
{
    uint16_t reg_val;
    int      ret;

    ret = spi_read_word(spim_id_global,
                        spim_slave_select_global,
                        &reg_addr,
                        (uint8_t *)&reg_val) != 2;
    return ret == 0 ? reg_val : ret;
}
#endif

int i2c_smbus_read_word_data(u8 reg_addr)
{
    uint16_t reg_val;
    int      ret;

#if defined(MAX32620_EvKit_V1) || defined(MAX32625_EvKit_V1)
    ret = I2CM_Read(MXC_I2CM1,
                    (MAX86XXX_SLAVE_ADDR >> 1),
                    &reg_addr,
                    1,
                    (uint8_t *)&reg_val,
                    2) != 2;
    return ret == 0 ? reg_val : ret;
#elif defined(SENSOR_MAX86908)
    ret = I2CM_Read(i2cm_id_global,
                    (i2c_slave_addr_global >> 1),
                    &reg_addr,
                    1,
                    (uint8_t *)&reg_val,
                    2) != 2;
    return ret == 0 ? reg_val : ret;
#else
#warning "MAX86XXX Board type is not defined"
    return -1;
#endif
}

int max86xxx_read_reg(u8 *buffer, int length)
{
    int ret;

#if defined SENSOR_MAX86908 //OB92
    unsigned char reg_Addr = buffer[0];
    ret                    = I2CM_Read(i2cm_id_global,
                    (i2c_slave_addr_global >> 1),
                    &reg_Addr,
                    1,
                    buffer,
                    length);
    return ret;
#else
#warning "Sensor type is not defined"
    return -1;
#endif
}

#ifdef USE_SPI
int spi_read_reg(mxc_spim_regs_t *_spim,
                 uint8_t          ssel,
                 uint8_t *        buffer,
                 int              length)
{
    u8         tx_data[2] = { buffer[0], SPI_READ_CMD };
    spim_req_t req;

    // Send the command
    req.ssel    = ssel;
    req.len     = 2;
    req.tx_data = tx_data;
    req.rx_data = NULL;
    req.deass   = 0;
    req.width   = SPIM_WIDTH_1;
    if (SPIM_Trans(_spim, &req) != 2) {
        return E_COMM_ERR;
    }

    // Read the data
    req.len     = length;
    req.tx_data = NULL;
    req.rx_data = buffer;
    req.deass   = 1;
    req.width   = SPIM_WIDTH_1;
    return SPIM_Trans(_spim, &req) != length;
}
int spi_read_word(mxc_spim_regs_t *_spim,
                  uint8_t          ssel,
                  const uint8_t *  reg_addr,
                  uint8_t *        buffer)
{
    u8         tx_data[2] = { reg_addr[0], SPI_READ_CMD };
    spim_req_t req;
    // Send the command
    req.ssel    = ssel;
    req.len     = 2;
    req.tx_data = tx_data;
    req.rx_data = NULL;
    req.deass   = 0;
    req.width   = SPIM_WIDTH_1;
    if (SPIM_Trans(_spim, &req) != 2) {
        return E_COMM_ERR;
    }

    // Read the data
    req.len     = 2;
    req.tx_data = NULL;
    req.rx_data = buffer;
    req.deass   = 1;
    req.width   = SPIM_WIDTH_1;
    return SPIM_Trans(_spim, &req);
}
int max86xxx_spi_read_reg(mxc_spim_regs_t *_spim,
                          uint8_t          ssel,
                          uint8_t *        buffer,
                          int              length)
{
    int ret_val;
    //	    gpio_cfg_t *slave_select_pin;
    //		if(ssel == 0)
    //			slave_select_pin = &P3_0_OS53cs1;
    //		else if(ssel == 1)
    //			slave_select_pin = &P3_1_OS53cs2;

    //	GPIO_OutClr(slave_select_pin);
    ret_val = spi_read_reg(_spim, ssel, buffer, length);
    //	GPIO_OutSet(slave_select_pin);
    return ret_val;
}

int spi_write_reg(mxc_spim_regs_t *_spim,
                  uint8_t          ssel,
                  uint8_t          reg_addr,
                  uint8_t          buf)
{
    spim_req_t req;
    u8         tx_data[3] = { reg_addr, SPI_WRITE_CMD, buf };

    int ret_val;

    // Send the command
    req.ssel    = ssel;
    req.len     = 3;
    req.tx_data = tx_data;
    req.rx_data = NULL;
    req.deass   = 1;
    req.width   = SPIM_WIDTH_1;
    //returns Bytes transacted if everything is successful, error if unsuccessful.
    ret_val = SPIM_Trans(_spim, &req);
    if (ret_val != (req.len)) {
        printf(" E_COMM_ERR %d\n", ret_val);
        return E_COMM_ERR;
    }
    return ret_val;
}

int max86xxx_spi_write_reg(mxc_spim_regs_t *_spim,
                           uint8_t          ssel,
                           uint8_t          reg_addr,
                           uint8_t          buf)
{
    int ret_val;
    //	if(ssel == 0)
    //		slave_select_pin = &P3_0_OS53cs1;
    //	else if(ssel == 1)
    //		slave_select_pin = &P3_1_OS53cs2;

    //	GPIO_OutClr(slave_select_pin);
    ret_val = spi_write_reg(_spim, ssel, reg_addr, buf);
    //	GPIO_OutSet(slave_select_pin);
    return ret_val;
}
#endif

int max86xxx_i2c_block_write(const struct regmap reg_block[], int size)
{
    int i;
    int ret = 0;

    for (i = 0; i < size; i++) {
        ret = max86xxx_write_reg(reg_block[i].addr, reg_block[i].val);
        if (ret < 0)
            return ret;
    }

    return ret;
}

int update_bits(u8 reg_addr, u8 mask, u8 val)
{
    int ret;
    u8  tmp, orig = reg_addr;

    ret = max86xxx_read_reg(&orig, 1);
    if (ret < 0)
        return ret;

    tmp = orig & ~mask;
    tmp |= val & mask;

    if (tmp != orig)
        ret = max86xxx_write_reg(reg_addr, tmp);
    return ret;
}

int max86xxx_regulator_onoff(struct max86xxx_dev *sd, char enable)
{
    sd->regulator_state = enable;

    return 0;
}

int max86xxx_reset(struct max86xxx_dev *sd)
{
    int ret = 0;
    int i;

    struct max86xxx_sensor *sensor;

    if (!sd->regulator_state) {
        ret = max86xxx_regulator_onoff(sd, PWR_ON);
        if (ret < 0) {
            printf("Unable to turn on the regulator. %s:%d, ret: %d\n",
                   __func__,
                   __LINE__,
                   ret);
        }
    }

    ret            = max86xxx_write_reg(MAX86XXX_REG_SYSTEM_CTRL,
                             MAX86XXX_MASK_SYSTEM_RESET);
    sd->curr_state = MAX86XXX_NONE_MODE;
    sd->die_temp   = 0;
    //	for (i = 0; i < sd->num_sensors; i++) {
    ESP_LOGI(TAG, "reseting sensor in %s", __func__);
    sensor = get_sensor_ptr(sd, 0 /*i*/);
    if (sensor->reset)
        sensor->reset(sensor);
    //	}

    return ret;
}

int max86xxx_poweroff(struct max86xxx_dev *sd)
{
    int ret = 0;

    ret = max86xxx_write_reg(MAX86XXX_REG_SYSTEM_CTRL,
                             MAX86XXX_MASK_SYSTEM_SHDN);
    if (sd->regulator_state) {
        ret |= max86xxx_regulator_onoff(sd, PWR_OFF);
        if (ret < 0) {
            printf("Unable to turn off the regulator. %s:%d, ret: %d\n",
                   __func__,
                   __LINE__,
                   ret);
        }
    }

    return ret;
}

int max86xxx_sensor_enable(struct max86xxx_dev *sd, int sensor_id, int state)
{
    int                     num_channel;
    char                    sensor_status;
    u16                     fd_settings;
    int                     ret = 0;
    struct max86xxx_sensor *sensor;
    printf("Trying to get sensor id");
    if (sensor_id <= MAX86XXX_NONE_MODE ||
        sensor_id >= MAX86XXX_NUM_OF_SENSORS) {
        printf("sensor_id: %d is not defined as mode\n", sensor_id);
        return -EINVAL;
    }

    sensor = get_sensor_ptr(sd, sensor_id);
    printf("%s - %10.s mode:%d\n", __func__, sensor->name, state);

    if (!sd->regulator_state) {
        ESP_LOGD(TAG, "regulator state is %d", sd->regulator_state);
        ret |= max86xxx_regulator_onoff(sd, PWR_ON);
        if (ret < 0) {
            printf("Unable to turn on the regulator. %s, ret: %d\n",
                   __func__,
                   ret);
            goto fail;
        }
    }

    num_channel = max86xxx_get_fifo_settings(sd, &fd_settings);
    if (num_channel < 0)
        goto fail;

    sensor_status = sensor->is_sensor_active(sd, fd_settings, num_channel);

    if (state) {
        if (sensor_status) {
            printf("The mode (%d) is already enabled.\n", sensor_id);
            return ret;
        }

        ret = max86xxx_reset(sd);
        if (ret)
            goto fail;

        if (sensor->reg_settings) {
            ESP_LOGI(TAG, "writing reg settings");
            ret = max86xxx_i2c_block_write(sensor->reg_settings,
                                           sensor->regmap_len);
            if (ret)
                goto fail;
        }

        if (sensor->enable) {
            ESP_LOGI(TAG, "Enabling sensor");
            sensor->enable(sensor, state);
        }
        if (sensor->pause) {
            ESP_LOGI(TAG, "Setting to pause");
            sensor->pause(sensor, sd->pause_mode);
        }

        printf("%10.s was configured.\n", sensor->name);
    } else {
        if (sensor_status) {
            ret = max86xxx_reset(sd);
            if (ret)
                goto fail;
            if (sensor->enable) {
                ESP_LOGI(TAG, "enabling sensor");
                ret = sensor->enable(sensor, state);
                if (ret)
                    goto fail;
            }
            ret = max86xxx_poweroff(sd);
            if (ret)
                goto fail;
        } else
            printf("%10.s sensor is already disabled.\n", sensor->name);
    }

    return ret;
fail:
    printf("%s failed. sensor_id: %d, new state: %d, ret: %d\n",
           __func__,
           sensor_id,
           state,
           ret);
    return ret;
}

int max86xxx_sensor_pause(struct max86xxx_dev *sd, int sensor_id, int pause)
{
    int                     ret = 0;
    struct max86xxx_sensor *sensor;

    if (sensor_id <= MAX86XXX_NONE_MODE ||
        sensor_id >= MAX86XXX_NUM_OF_SENSORS) {
        printf("sensor_id: %d is not defined as mode\n", sensor_id);
        return -EINVAL;
    }

    sensor = get_sensor_ptr(sd, sensor_id);
    if (sensor->pause) {
        ret = sensor->pause(sensor, pause);
        if (!ret) {
            sd->pause_mode = pause;
        }
    }

    return ret;
}

int max86xxx_enable_die_temp(struct max86xxx_dev *sd)
{
    int ret = 0;

#if defined(USE_MAX86903_REGMAP) || defined(SENSOR_MAX86908)
    ret = max86xxx_write_reg(MAX86XXX_REG_DIE_TEMP_CFG, 0x01);
    if (ret < 0)
        printf("I2C Communication error: %s:%d\n", __func__, __LINE__);
#endif
    return ret;
}

int max86xxx_read_die_temp(struct max86xxx_dev *sd)
{
    int ret = 0;
#if defined(USE_MAX86903_REGMAP) || defined(SENSOR_MAX86908)
    u8  buf[2];
    int die_temp;

    buf[0] = MAX86XXX_REG_DIE_TEMP_INT;
    ret    = max86xxx_read_reg(buf, 2);
    if (ret < 0) {
        printf("Unable to read die_temp. %s:%d - ret: %d\n",
               __func__,
               __LINE__,
               ret);
        return ret;
    }

    die_temp     = (((int)buf[0] << 4) | ((int)buf[1] & 0x0F));
    die_temp     = die_temp;
    sd->die_temp = die_temp;

    printf("Die temp: %d - %d, %d\n", die_temp, buf[0], buf[1]);
    ret = max86xxx_enable_die_temp(sd);
#else
    sd->die_temp = 0;
#endif
    return ret;
}

static int max86xxx_startup_init(struct max86xxx_dev *sd)
{
    ESP_LOGI(TAG, "max86xxx_startup_init start");
    int ret = 0;

    ret = max86xxx_reset(sd);
    ESP_LOGD(TAG, "max86xxx_reset returned %d", ret);
    if (ret < 0)
        goto fail;

    ret = max86xxx_poweroff(sd);
    if (ret < 0)
        goto fail;

    printf("%s done\n", __func__);
    return ret;
fail:
    printf("%s failed. ret: %d\n", __func__, ret);
    return ret;
}

int max86xxx_get_fifo_settings(struct max86xxx_dev *sd, u16 *ch)
{
    int num_item = 0;
    int i;
    u16 tmp;
    int ret;

    /*
	 * TODO: Somehow when the data corrupted in the bus,
		and i2c functions don't return error messages.
	*/
#ifdef SENSOR_MAX30110
    ret = spi_smbus_read_word_data(MAX86XXX_REG_FIFO_DATA_CTRL_1);
#elif defined(SENSOR_MAX86908)
    ret          = i2c_smbus_read_word_data(MAX86XXX_REG_FIFO_DATA_CTRL_1);
#else
#warning "Sensor type is not defined"
#endif
    if (ret < 0)
        return ret;

    *ch = tmp = ret;
    for (i = 0; i < 4; i++) {
        if (tmp & 0x000F)
            num_item++;
        tmp >>= 4;
    }

    return num_item;
}

int max86xxx_get_num_samples_in_fifo(struct max86xxx_dev *sd)
{
    int fifo_rd_ptr;
    int fifo_wr_ptr;
    int fifo_ovf_cnt;
    u8  fifo_data[3];
    int num_samples = 0;
    int ret;

    fifo_data[0] = MAX86XXX_REG_FIFO_WRITE_PTR;
    ret          = max86xxx_read_reg(fifo_data, 3);
    if (ret < 0) {
        printf("%s failed. ret: %d\n", __func__, ret);
        return ret;
    }

    fifo_wr_ptr  = fifo_data[0] & 0x1F;
    fifo_ovf_cnt = fifo_data[1] & 0x1F;
    fifo_rd_ptr  = fifo_data[2] & 0x1F;

    if (fifo_ovf_cnt || fifo_rd_ptr == fifo_wr_ptr) {
        printf("# FIFO is Full. OVF: %d RD:%d WR:%d\n",
               fifo_ovf_cnt,
               fifo_rd_ptr,
               fifo_wr_ptr);
        num_samples = MAX86XXX_FIFO_SIZE;
    } else {
        if (fifo_wr_ptr > fifo_rd_ptr)
            num_samples = fifo_wr_ptr - fifo_rd_ptr;
        else if (fifo_wr_ptr < fifo_rd_ptr)
            num_samples = MAX86XXX_FIFO_SIZE + fifo_wr_ptr - fifo_rd_ptr;
    }

    return num_samples;
}
void max86xxx_preprocess_data(int *samples, u8 fifo_channel)
{
    if (fifo_channel >= IR_PA_CHANNEL && fifo_channel <= PILOT_LED4_PA_CHANNEL)
        *samples &= PPG_DATA_BIT_MASK;
    else if (fifo_channel == ECG_CHANNEL || fifo_channel == IMP_I_CHANNEL ||
             fifo_channel == IMP_Q_CHANNEL) {
        *samples &= FIFO_DATA_18BIT_MASK;
        /* ECG, IMP are two's complement and can be negative */
        if (*samples & (1 << 17))
            *samples -= (1 << 18);
    } else
        printf("Wrong mode: %.1X - %s:%d\n", fifo_channel, __func__, __LINE__);
}

char max86xxx_is_ch_active(u16 fd_settings, u8 mode)
{
    int i;

    for (i = 0; i < 4; i++) {
        if (((fd_settings >> (i * 4)) & 0x000F) == mode)
            return true;
    }
    return false;
}

int max86xxx_get_sensor_mode(struct max86xxx_dev *sd,
                             u16                  fd_settings,
                             int                  num_channel)
{
    int                     i;
    struct max86xxx_sensor *sensor;

    for (i = 0; i < sd->num_sensors; i++) {
        sensor = get_sensor_ptr(sd, i);
        if (sensor->is_sensor_active &&
            sensor->is_sensor_active(sd, fd_settings, num_channel))
            return i;
    }

    return MAX86XXX_NONE_MODE;
}

void max86xxx_fifo_irq_handler(struct max86xxx_dev *sd)
{
    int ret = 0;
    int num_samples;
    int num_bytes;
    int num_channel = 0;
    u16 fd_settings = 0;
    int tmp;
    int i;
    int j;
    int offset1;
    int offset2;
    int index;
    u16 tmp_fd;
    int samples[16] = {
        0,
    };
    u8                      fifo_buf[MAX86XXX_MAX_FIFO_BUF_LEN];
    struct max86xxx_sensor *sensor;
    int                     fifo_mode = MAX86XXX_NONE_MODE;

    num_samples = max86xxx_get_num_samples_in_fifo(sd);
    if (num_samples <= 0 || num_samples > MAX86XXX_FIFO_SIZE) {
        ret = num_samples;
        goto fail;
    }
    num_channel = max86xxx_get_fifo_settings(sd, &fd_settings);
    if (num_channel < 0) {
        ret = num_channel;
        goto fail;
    }

    num_bytes   = num_channel * num_samples * NUM_BYTES_PER_SAMPLE;
    fifo_buf[0] = MAX86XXX_REG_FIFO_DATA;
    ret         = max86xxx_read_reg(fifo_buf, num_bytes);
    if (ret < 0)
        goto fail;

    fifo_mode = max86xxx_get_sensor_mode(sd, fd_settings, num_channel);
    if (fifo_mode < MAX86XXX_NONE_MODE)
        goto fail;

    sensor = get_sensor_ptr(sd, fifo_mode);
    for (i = 0; i < num_samples; i++) {
        offset1 = i * NUM_BYTES_PER_SAMPLE * num_channel;
        offset2 = 0;

        for (j = 0; j < MAX_FIFO_SLOT_NUM; j++) {
            tmp_fd = (fd_settings >> (4 * j)) & 0x000F;
            if (tmp_fd) {
                index = offset1 + offset2;
                tmp   = ((int)fifo_buf[index + 0] << 16) |
                      ((int)fifo_buf[index + 1] << 8) |
                      ((int)fifo_buf[index + 2]);

                samples[tmp_fd] = tmp;
#if defined(SENSOR_MAX30110) && defined(DUAL_LED)
                max86xxx_preprocess_data(&samples[tmp_fd], 1);
#elif defined(SENSOR_MAX86908) || defined(SENSOR_MAX30110)
                max86xxx_preprocess_data(&samples[tmp_fd], tmp_fd);
#else
#warning "Sensor type is not defined"
#endif
                offset2 += NUM_BYTES_PER_SAMPLE;
            }
        }

        if (fifo_mode != MAX86XXX_NONE_MODE && fifo_mode < sd->num_sensors)
            sensor->report(sensor, samples);
        else
            printf("Undefined mode fifo_mode: %d, fd: %.4X, num_modes: %d\n",
                   fifo_mode,
                   fd_settings,
                   sd->num_sensors);
    }

    if (sensor->update)
        sensor->update(sensor);

    return;
fail:
    printf("%s failed. ret: %d, fifo_mode: %d\n", __func__, ret, fifo_mode);
    return;
}

void max86xxx_irq_handler(void *cbdata)
{
    printf("max86xxx_irq_handler runned");
    struct max86xxx_dev *sd = max86xxx_get_device_data();
    int                  ret;
    union int_status     status;

    //start_event(&irq_events);

    status.val[0] = MAX86XXX_REG_INT_STATUS1;
    ret           = max86xxx_read_reg(status.val, 2);
    if (ret < 0) {
        printf("I2C Communication error. err: %d. %s:%d\n",
               ret,
               __func__,
               __LINE__);
        //stop_event(&irq_events);
    }

    if (status.a_full || status.ppg_rdy || status.ecg_imp_rdy ||
        status.prox_int) {
        max86xxx_fifo_irq_handler(sd);
    }

#if 0
	if (status.pwr_rdy)
		printf("Pwr_rdy interrupt was triggered.\n");
#endif

    if (status.die_temp_rdy)
        max86xxx_read_die_temp(sd);

    if (status.vdd_oor) {
        sd->vdd_oor_cnt++;
        printf("VDD Out of range cnt: %d\n", sd->vdd_oor_cnt);
    }
}

int max86xxx_init(void)
{
    int                     ret       = 0;
    uint8_t                 buffer[2] = { 0 };
    struct max86xxx_dev *   sd;
    struct max86xxx_sensor *sensor;
    int                     i;

    buffer[0] = MAX86XXX_REG_WHOAMI_REG_REV;
    ret       = max86xxx_read_reg(buffer, 2);
    if (ret != 0) {
        printf("%d - communication failed\n", ret);
        return -1;
    }

    if (buffer[1] != EXPECTED_PART_ID) {
        printf("Part ID Error. Part_id: %.2X, rev_id: %.2X, ret: %d\n",
               buffer[1],
               buffer[0],
               ret);
        return -1;
    }
    printf("Max86xxx detected\n");
    sd = malloc(sizeof(struct max86xxx_dev));
    if (sd == NULL) {
        printf("%s:%d - Out of memory\n", __func__, __LINE__);
        return -1;
    }

    memset(sd, 0, sizeof(struct max86xxx_dev));
    max86xxx_set_device_data(sd);
    sd->sensors     = &max86xxx_sensors;
    sd->num_sensors = ARRAY_SIZE(max86xxx_sensors);

    sd->rev_id  = buffer[0];
    sd->part_id = buffer[1];
    printf("Part_id: %.2X, rev_id: %.2X\n", sd->part_id, sd->rev_id);

    //	for (i = 0; i < sd->num_sensors; i++) {
    ESP_LOGI(TAG, "sensor_num is %d", sd->num_sensors);
    sensor = get_sensor_ptr(sd, 0);
    //		ESP_LOGI(TAG,"Initializing %.20s, i: %d\n", sensor->name, i);

    if (sensor->init) {
        ret = sensor->init(sd);
        if (ret != 0)
            goto fail;
    }
    sensor->dev = sd;
    printf("%s is succeed\n", sensor->name);
    //	}

    ret = max86xxx_startup_init(sd);
    if (ret != 0) {
        printf("max86xxx_startup_init FAILED\n");
        return -1;
    }

#ifdef CONFIG_IDF_TARGET_ESP32
    ESP_LOGI(TAG, "Installing the ISR service");
    ESP_ERROR_CHECK(gpio_install_isr_service(0));
    gpio_config_t io_conf;
    io_conf.intr_type    = GPIO_PIN_INTR_ANYEDGE;
    io_conf.mode         = GPIO_MODE_INPUT;
    io_conf.pull_down_en = false;
    io_conf.pull_up_en   = true;
    io_conf.pin_bit_mask = (1ULL << INT_PIN_NUM);
    ESP_LOGD(TAG, "Running GPIO config");
    ESP_ERROR_CHECK(gpio_config(&io_conf));

    /* Attach interrupt handler*/
    int pin_num = INT_PIN_NUM;
    ESP_LOGD(TAG, "gpio_isr_handler_add for max86150 interrupt start");

    ESP_ERROR_CHECK(gpio_isr_handler_add(
            INT_PIN_NUM, (gpio_isr_t)max86xxx_irq_handler, (void *)&pin_num));
    ESP_LOGD(TAG, "gpio_isr_handler_add for max86150 interrupt done");

#else
    register_gpio_irq_handler((void *)max86xxx_irq_handler, &maxdev_int, 0);

    init_events(&irq_events, 1000000, "irq");
#endif

#if defined(MAX86XXX_PPG_ENABLED)
    ret = max86xxx_sensor_enable(sd, MAX86XXX_PPG_MODE, 1);
    if (ret < 0)
        return ret;
#elif defined(MAX86XXX_ECG_ENABLED)
    ret = max86xxx_sensor_enable(sd, MAX86XXX_ECG_MODE, 1);
    if (ret < 0)
        return ret;
#elif defined(MAX86XXX_ECPPG_ENABLED)
    ESP_LOGI(TAG, "Starting sensor enable");
    ret = max86xxx_sensor_enable(sd, MAX86XXX_ECPPG_MODE, 1);
    if (ret < 0)
        return ret;
#endif
    return ret;
fail:
    printf("Init failed %s:%d\n", __func__, __LINE__);
    return -1;
}

int max86xxx_dump_regs(uint8_t *buf, int num_reg)
{
    int     i;
    uint8_t tmp[2];

    for (i = 0; i < num_reg; i++) {
        tmp[0] = i;
        max86xxx_read_reg(tmp, 1);
        printf("address = [0x%.2x] val= 0x%.2x\n", i, tmp[0]);

        buf[i] = tmp[0];
    }
    return 0;
}
