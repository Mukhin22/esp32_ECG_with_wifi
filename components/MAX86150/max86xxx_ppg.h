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
#ifndef MAX86XXX_PPG_H
#define MAX86XXX_PPG_H

typedef struct {
#if defined DUAL_LED
    uint32_t green;
#else
    uint32_t       ir;
    uint32_t       red;
#endif
} ppg_data_t;

struct max86xxx_ppg_data {
    struct max86xxx_dev *dev;
    struct led_control   led_ctrl;
#ifdef CONFIG_IDF_TARGET_ESP32
    QueueHandle_t queue;
#else
    struct queue_t queue;
#endif
    u8  hr_range;
    u8  look_mode_ir;
    u8  look_mode_red;
    u32 sample_cnt;
#if defined DUAL_LED
    u32 green_sum;
#else
    u32            ir_sum;
    u32            red_sum;
#endif
    u32   avg_cnt;
    int   eol_test_is_enabled;
    char *eol_test_result;
    u8    eol_test_status;
};

extern struct max86xxx_sensor ppg_sensor;
int max86xxx_ppg_agc_enable(struct max86xxx_dev *sd, int agc_enable);
int max86xxx_get_ppg_data(struct max86xxx_dev *dev, ppg_data_t *ppg_data);
#endif
