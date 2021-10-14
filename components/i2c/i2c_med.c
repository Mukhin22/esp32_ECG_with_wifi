/*
   For other examples please check:
   https://github.com/espressif/esp-idf/tree/master/examples

   See README.md file to get detailed usage of this example.

   This example code is in the Public Domain (or CC0 licensed, at your option.)

   Unless required by applicable law or agreed to in writing, this
   software is distributed on an "AS IS" BASIS, WITHOUT WARRANTIES OR
   CONDITIONS OF ANY KIND, either express or implied.
*/

#define LOG_LOCAL_LEVEL ESP_LOG_DEBUG

#include <stdio.h>
#include "esp_log.h"
#include "sdkconfig.h"
#include "i2c_med.h"

static const char *TAG = "i2c_module";

#define I2C_MASTER_SCL_IO         19    /*!< gpio number for I2C master clock */
#define I2C_MASTER_SDA_IO         18    /*!< gpio number for I2C master data  */
#define I2C_MASTER_FREQ_HZ        50000 /*!< I2C master clock frequency */
#define I2C_MASTER_TX_BUF_DISABLE 0     /*!< I2C master doesn't need buffer */
#define I2C_MASTER_RX_BUF_DISABLE 0     /*!< I2C master doesn't need buffer */
#define WRITE_BIT                 I2C_MASTER_WRITE /*!< I2C master write */
#define READ_BIT                  I2C_MASTER_READ  /*!< I2C master read */
#define ACK_CHECK_EN              0x1 /*!< I2C master will check ack from slave*/
#define ACK_CHECK_DIS             0x0 /*!< I2C master will not check ack from slave */
#define ACK_VAL                   0x0 /*!< I2C ack value */
#define NACK_VAL                  0x1 /*!< I2C nack value */

int func_FUNC(int i, int y)
{
    return 0;
}

int I2CM_Read(i2c_port_t i2c_num,
              u8         slave_adr,
              const u8 * cmd_data,
              u32        cmd_len,
              u8 *       data,
              u32        len)
{
    int err = ESP_OK;

    if (!data || !len) {
        ESP_LOGE(TAG,
                 "Invalid arguments. Data adress is %p, data len is %d ",
                 data,
                 len);
        return ESP_ERR_INVALID_ARG;
    }

    i2c_cmd_handle_t cmd = i2c_cmd_link_create();
    ESP_LOGD(TAG, "Slave address is %x", slave_adr);
    ESP_LOGD(TAG, "cmd data  is %x len is %x", cmd_data[0], cmd_len);
    //if there is something to write before reading(for example - slave adress)
    if (cmd_len) {
        i2c_master_start(cmd);
        i2c_master_write_byte(cmd, (slave_adr << 1) | WRITE_BIT, ACK_CHECK_EN);
        i2c_master_write_byte(cmd, cmd_data[0], ACK_CHECK_EN);
    }

    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, (slave_adr << 1) | READ_BIT, ACK_CHECK_EN);

    if (len > 1) {
        i2c_master_read(cmd, data, len - 1, ACK_VAL);
    }

    i2c_master_read_byte(cmd, data + len - 1, NACK_VAL);
    i2c_master_stop(cmd);
    err = i2c_master_cmd_begin(i2c_num, cmd, 1000 / portTICK_RATE_MS);
    i2c_cmd_link_delete(cmd);
    if (err) {
        ESP_LOGE(TAG, "Failed to read I2C master");
    }

    return err;
}

int I2CM_Write(i2c_port_t i2c_num,
               u8         slave_adr,
               const u8 * cmd_data,
               u32        cmd_len,
               u8 *       data,
               u32        len)
{
    int err = ESP_OK;

    if (!data || !len) {
        ESP_LOGE(TAG,
                 "Invalid arguments. Data adress is %p, data len is %d ",
                 data,
                 len);
        return ESP_ERR_INVALID_ARG;
    }
    i2c_cmd_handle_t cmd = i2c_cmd_link_create();
    //if there is something to write before writing(for example - register adress)
    if (cmd_len) {
        i2c_master_start(cmd);
        i2c_master_write_byte(cmd, (slave_adr << 1) | WRITE_BIT, ACK_CHECK_EN);
        i2c_master_write_byte(cmd, cmd_data[0], ACK_CHECK_EN);
    } else {
        i2c_master_write_byte(cmd, (slave_adr << 1) | WRITE_BIT, ACK_CHECK_EN);
    }

    i2c_master_start(cmd);
    i2c_master_write(cmd, data, len, ACK_CHECK_EN);
    err = i2c_master_cmd_begin(i2c_num, cmd, 1000 / portTICK_RATE_MS);
    i2c_cmd_link_delete(cmd);
    if (err) {
        ESP_LOGE(TAG, "Failed to write I2C master");
    }
    return err;
}

esp_err_t i2c_master_init(void)
{
    int          i2c_master_port = I2C_MASTER_NUM;
    i2c_config_t conf;

    conf.mode             = I2C_MODE_MASTER;
    conf.sda_io_num       = I2C_MASTER_SDA_IO;
    conf.sda_pullup_en    = GPIO_PULLUP_ENABLE;
    conf.scl_io_num       = I2C_MASTER_SCL_IO;
    conf.scl_pullup_en    = GPIO_PULLUP_ENABLE;
    conf.master.clk_speed = I2C_MASTER_FREQ_HZ;
    i2c_param_config(i2c_master_port, &conf);

    return i2c_driver_install(i2c_master_port,
                              conf.mode,
                              I2C_MASTER_RX_BUF_DISABLE,
                              I2C_MASTER_TX_BUF_DISABLE,
                              0);
}
