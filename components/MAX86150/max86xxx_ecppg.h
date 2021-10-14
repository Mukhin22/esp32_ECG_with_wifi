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

#ifndef MAX86XXX_ECPPG_H
#define MAX86XXX_ECPPG_H

#define ARRAY_SIZE(X) sizeof(X)
typedef struct _ecppg_data_t {
    int ir;
    int red;
    int ecg;
} ecppg_data_t;

struct max86xxx_ecppg_data {
    u32                  sample_cnt;
    struct max86xxx_dev *dev;
    struct led_control   led_ctrl;
    //	struct queue_t queue;
    QueueHandle_t queue;
    ecg_cfg_t     ecg_cfg;
};

typedef struct {
    uint8_t  start_byte : 8;
    uint32_t sample_count : 24;
    uint32_t ir : 20;
    uint32_t red : 20;
    int32_t  ecg : 19;
    int32_t  ecg_filt : 19;

    uint8_t : 0;
    uint8_t crc8 : 8;
} __attribute__((packed)) data_packet_mode0;

int max86xxx_get_ecppg_data(struct max86xxx_dev *dev, ecppg_data_t *ecppg_data);

int max86xxx_ecppg_agc_enable(struct max86xxx_dev *sd, int agc_enable);

extern struct max86xxx_sensor ecppg_sensor;
#endif
