/*
 * i2c_med.h
 *
 *  Created on: May 6, 2021
 *      Author: strngr
 */

#ifndef COMPONENTS_I2C_I2C_MED_H_
#define COMPONENTS_I2C_I2C_MED_H_

#include "driver/i2c.h"
#include "../MAX86150/max86xxx_common.h"
#define I2C_MASTER_NUM 1 /*!< I2C port number for master dev */

typedef uint8_t  u8;
typedef uint16_t u16;
typedef uint32_t u32;
typedef int8_t   s8;
typedef int8_t   s16;
typedef int32_t  s32;

/**
 * @brief   Read I2CM data. Will block until transaction is complete.
 * @param   i2c_num     Port of I2C to send.
 * @param   slave_adr   I2C address of the slave.
 * @param   cmd_data    Data to write before reading.
 * @param   cmd_len     Number of bytes to write before reading.
 * @param   data        Where to store read data.
 * @param   len         Number of bytes to read.
 * @details Command is an optional feature where the master will write the cmd_data
 *          before reading from the slave. If command is undesired, leave the pointer
 *          NULL and cmd_len 0. If there is a command, the master will send a
            repeated start before reading. Will block until transaction has completed.
 * @returns Bytes transacted if everything is successful, error if unsuccessful.
 */
int I2CM_Read(i2c_port_t i2c_num,
              u8         slave_adr,
              const u8 * cmd_data,
              u32        cmd_len,
              u8 *       data,
              u32        len);

/**
 * @brief   Write I2CM data. Will block until transaction is complete.
 * @param   i2cm        Pointer to I2CM regs.
 * @param   addr        I2C address of the slave.
 * @param   cmd_data    Data to write before writing data.
 * @param   cmd_len     Number of bytes to write before writing data.
 * @param   data        Data to be written.
 * @param   len         Number of bytes to Write.
 * @details Command is an optional feature where the master will write the cmd_data
 *          before writing to the slave. If command is undesired, leave the pointer
 *          NULL and cmd_len 0. If there is a command, the master will send a
            repeated start before writing again. Will block until transaction has completed.
 * @returns Bytes transacted if everything is successful, error if unsuccessful.
 */
int I2CM_Write(i2c_port_t i2c_num,
               u8         slave_adr,
               const u8 * cmd_data,
               u32        cmd_len,
               u8 *       data,
               u32        len);

/**
 * @brief i2c master initialization
 * @returns ESP_OK in case of successes.
 */
esp_err_t i2c_master_init(void);

#endif /* COMPONENTS_I2C_I2C_MED_H_ */
