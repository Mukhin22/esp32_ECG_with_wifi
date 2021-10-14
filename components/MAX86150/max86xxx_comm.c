/***************************************************************************
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
****************************************************************************
*/

#ifdef USE_COMM

#include <stdio.h>
#include <ctype.h>

#include "sdkconfig.h"

#ifdef CONFIG_IDF_TARGET_ESP32
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/event_groups.h"
#include "freertos/queue.h"
#include "driver/uart.h"
#include "esp_system.h"
#include "max86xxx_common.h"
#include "string.h"
#else
#include "platform.h"
#include "platform_event.h"
#include "queue.h"
#endif
#include "max86xxx_core.h"
#include "max86xxx_algo.h" // usage of this header is not defined
#include "max86xxx_comm.h"
#include "max86xxx_ecppg.h"
#include "comm_helper.h"
#include "crc-8.h"

#ifdef CONFIG_IDF_TARGET_ESP32
#define UART_Write(uart_num, data, len) uart_write_bytes(uart_num, data, len)
#endif
volatile uint8_t max86xxx_data_report_mode = stop;
volatile uint8_t console_interface_exists  = false;
volatile uint8_t ble_interface_exists      = false;

static void *max86xxx_comm_data(void *io_ptr)
{
    static void *data_ptr;
    if (io_ptr)
        data_ptr = io_ptr;
    return data_ptr;
}

void *max86xxx_get_comm_data(void)
{
    return max86xxx_comm_data(NULL);
}

static void *max86xxx_set_comm_data(void *io_ptr)
{
    return max86xxx_comm_data(io_ptr);
}

int max86xxx_comm_init(void)
{
    max86xxx_comm_t *output;
    int              ret = 0;

    output = malloc(sizeof(max86xxx_comm_t));
    if (output == NULL) {
        printf("%s:%d Out of memory\n", __func__, __LINE__);
        return -1;
    }

    max86xxx_set_comm_data(output);
    output->user_cmd = stop;
    output->last_cmd = stop;
    memset(output->cmd_str, 0, MAX86XXX_STR_BUF_SZ);
    output->cmd_idx     = 0;
    output->silent_mode = false;
    return ret;
}

static int max86xxx_get_reg(char *ptr_ch, uint8_t *reg_addr, uint8_t *value)
{
    int     ret = EXIT_FAILURE;
    uint8_t num_found;
    int     which_reg;

    while (*ptr_ch) {
        if (isxdigit((int)*ptr_ch)) {
            num_found = (uint8_t)sscanf(ptr_ch, "%x", &which_reg);
            if (num_found == 1) {
                *reg_addr = which_reg;
                *value    = *reg_addr;
                ret       = max86xxx_read_reg(value, 1);
            }
            break;
        } else {
            ptr_ch++;
        }
    }
    return ret;
}

static int max86xxx_get_param(const char *ptr_ch, const char *cmd, int *val)
{
    int num_found;
    int cmd_len = 0;

    while (*cmd++) {
        cmd_len++;
    }

    num_found = sscanf(ptr_ch + cmd_len, "%d", val);
    if (num_found == 1) {
        return 0;
    }

    return EXIT_FAILURE;
}

static int max86xxx_set_reg(char *ptr_ch)
{
    int     ret = EXIT_FAILURE;
    uint8_t num_found;
    int     reg_addr, value;
    char *  ptr_char;

    ptr_char = ptr_ch;
    while (*ptr_char) {
        if (isxdigit((int)*ptr_char)) {
            num_found = (uint8_t)sscanf(ptr_char, "%x %x", &reg_addr, &value);
            if (num_found == 2) {
                ret = max86xxx_write_reg(reg_addr, value);
            }
            break;
        } else {
            ptr_char++;
        }
    }
    return ret;
}

static void max86xxx_parse_command()
{
    max86xxx_comm_t *    output = max86xxx_get_comm_data();
    struct max86xxx_dev *sd     = max86xxx_get_device_data();
    unsigned long        dev_id = 0;
    char                 cbuf[512];
    int                  cbuf_idx = 0;

    const char *cmd_tbl[MAX86XXX_NUM_CMDS] = {
        "stop",
        "get_format ecppg 0",
        "read ecppg 0", /* raw */
        "get_device_info",
        "get_reg ecppg",
        "set_reg ecppg",
        "dump_reg ecppg",
        "silent_mode 0",
        "silent_mode 1",
        "pause 0",
        "pause 1",
        "set_cfg ecppg agc 0",
        "set_cfg ecppg agc 1",
        "set_cfg ecppg sr",
        "set_cfg ecppg notch",
        "set_cfg ecppg cutoff",
        "set_cfg ecppg adapt",
    };
    char *   ptr_ch;
    int      i;
    int      ret = EXIT_SUCCESS;
    uint16_t reg_addr;
    uint8_t  val;
    int      param         = -1;
    bool     recognizedCmd = false;

    for (i = 0; i < MAX86XXX_NUM_CMDS; i++) {
        ptr_ch = strstr(&output->cmd_str[0], cmd_tbl[i]);
        if (*ptr_ch) {
            output->last_cmd = output->user_cmd;
            output->user_cmd = (cmd_state_t)i;
            recognizedCmd    = true;
            switch (output->user_cmd) {
            case stop:
                max86xxx_data_report_mode = stop;
                ret                       = max86xxx_reset(sd);
                if (ret < 0) {
                    printf("max86xxx_reset failed. ret: %d\n", ret);
                }

                cbuf_idx += snprintf(cbuf,
                                     sizeof(cbuf) - cbuf_idx - 1,
                                     "\r\n%s err=%d\r\n",
                                     ptr_ch,
                                     ret);
                printf(cbuf);
                comm_report_data(cbuf, cbuf_idx);
                break;

            case get_format_mode0:
#ifdef ASCII_COMM
                cbuf_idx += snprintf(
                        cbuf,
                        sizeof(cbuf) - cbuf_idx - 1,
                        "\r\n%s format=smpleCnt,irCnt,redCnt,ecg,ecg_filt err=0\r\n",
                        ptr_ch);
                printf(cbuf);
                comm_report_data(cbuf, cbuf_idx);
#else
                cbuf_idx += snprintf(
                        cbuf,
                        sizeof(cbuf) - cbuf_idx - 1,
                        "\r\n%s enc=bin cs=1 format={smpleCnt,24},{irCnt,20},{redCnt,20},{ecg,19},{ecg_filt,19} err=0\r\n",
                        ptr_ch);
                printf(cbuf);
                comm_report_data(cbuf, cbuf_idx);
#endif
                break;
            case ecppg_mode0:
                max86xxx_data_report_mode = ecppg_mode0;
                ret = max86xxx_sensor_enable(sd, MAX86XXX_ECPPG_MODE, 1);
                if (ret < 0) {
                    printf("max86xxx_sensor_enable failed. ret: %d\r\n", ret);
                }

                cbuf_idx += snprintf(cbuf,
                                     sizeof(cbuf) - cbuf_idx - 1,
                                     "\r\n%s err=%d\r\n",
                                     ptr_ch,
                                     ret);
                printf(cbuf);
                comm_report_data(cbuf, cbuf_idx);
                break;
            case get_device_info:
                cbuf_idx += snprintf(cbuf + cbuf_idx,
                                     sizeof(cbuf) - cbuf_idx - 1,
                                     "\r\n%s platform=%s firmware_ver=%s ",
                                     ptr_ch,
                                     MAX86XXX_PLATFORM,
                                     FIRMWARE_VERSION);

                cbuf_idx += snprintf(cbuf + cbuf_idx,
                                     sizeof(cbuf) - cbuf_idx - 1,
                                     "sensors=%s driver_ver_%s=%s ",
                                     MAX86XXX_NAME,
                                     MAX86XXX_NAME,
                                     MAX86XXX_DRIVER_VERSION);

                cbuf_idx += snprintf(cbuf + cbuf_idx,
                                     sizeof(cbuf) - cbuf_idx - 1,
                                     "part_name_%s=%s part_id_%s=%.2X ",
                                     MAX86XXX_NAME,
                                     MAX86XXX_PART_NAME,
                                     MAX86XXX_NAME,
                                     sd->part_id);

                cbuf_idx += snprintf(
                        cbuf + cbuf_idx,
                        sizeof(cbuf) - cbuf_idx - 1,
                        "part_rev_id_%s=%.2X part_device_id_%s=%lu err=0\r\n",
                        MAX86XXX_NAME,
                        sd->rev_id,
                        MAX86XXX_NAME,
                        dev_id);

                printf(cbuf);
                comm_report_data(cbuf, cbuf_idx);

                break;
            case get_reg:
                reg_addr = 0;
                val      = 0;
                ret      = max86xxx_get_reg(ptr_ch + strlen(cmd_tbl[i]),
                                       (uint8_t *)&reg_addr,
                                       &val);

                cbuf_idx += snprintf(cbuf,
                                     sizeof(cbuf) - cbuf_idx - 1,
                                     "\r\n%s reg_val=%02X err=%d\r\n",
                                     ptr_ch,
                                     val,
                                     ret);
                printf(cbuf);
                comm_report_data(cbuf, cbuf_idx);
                break;
            case set_reg:
                ret = max86xxx_set_reg(ptr_ch + strlen(cmd_tbl[i]));
                ret |= max86xxx_get_reg(ptr_ch + strlen(cmd_tbl[i]),
                                        (uint8_t *)&reg_addr,
                                        &val);
                cbuf_idx += snprintf(cbuf,
                                     sizeof(cbuf) - cbuf_idx - 1,
                                     "\r\n%s err=0\r\n",
                                     ptr_ch);
                printf(cbuf);
                comm_report_data(cbuf, cbuf_idx);
                break;
            case dump_regs:
                cbuf_idx = 0;
                cbuf_idx += snprintf(cbuf + cbuf_idx,
                                     sizeof(cbuf) - cbuf_idx - 1,
                                     "\r\n%s reg_val=",
                                     ptr_ch);
                ret = 0;
                for (reg_addr = 0; reg_addr <= 0x25; reg_addr++) {
                    val = reg_addr;
                    ret |= max86xxx_read_reg(&val, 1);
                    cbuf_idx += snprintf(cbuf + cbuf_idx,
                                         sizeof(cbuf) - cbuf_idx - 1,
                                         "{%02X,%02X},",
                                         reg_addr,
                                         val);
                }
                for (reg_addr = 0x3C; reg_addr <= 0x3F; reg_addr++) {
                    val = reg_addr;
                    ret |= max86xxx_read_reg(&val, 1);
                    cbuf_idx += snprintf(cbuf + cbuf_idx,
                                         sizeof(cbuf) - cbuf_idx - 1,
                                         "{%02X,%02X},",
                                         reg_addr,
                                         val);
                }
                for (reg_addr = 0x42; reg_addr <= 0x44; reg_addr++) {
                    val = reg_addr;
                    ret |= max86xxx_read_reg(&val, 1);
                    cbuf_idx += snprintf(cbuf + cbuf_idx,
                                         sizeof(cbuf) - cbuf_idx - 1,
                                         "{%02X,%02X},",
                                         reg_addr,
                                         val);
                }
                for (reg_addr = 0xFE; reg_addr <= 0xFF; reg_addr++) {
                    val = reg_addr;
                    ret |= max86xxx_read_reg(&val, 1);
                    cbuf_idx += snprintf(cbuf + cbuf_idx,
                                         sizeof(cbuf) - cbuf_idx - 1,
                                         "{%02X,%02X},",
                                         reg_addr,
                                         val);
                }
                cbuf_idx += snprintf(cbuf + cbuf_idx,
                                     sizeof(cbuf) - cbuf_idx - 1,
                                     " err=%d\r\n",
                                     ret);
                printf(cbuf);
                comm_report_data(cbuf, cbuf_idx);
                break;
            case echo_mode:
                output->silent_mode = false;
                cbuf_idx += snprintf(cbuf,
                                     sizeof(cbuf) - cbuf_idx - 1,
                                     "\r\n%s err=0\r\n",
                                     ptr_ch);
                printf(cbuf);
                comm_report_data(cbuf, cbuf_idx);
                break;
            case silent_mode:
                output->silent_mode = true;
                cbuf_idx += snprintf(cbuf,
                                     sizeof(cbuf) - cbuf_idx - 1,
                                     "\r\n%s err=0\r\n",
                                     ptr_ch);
                printf(cbuf);
                comm_report_data(cbuf, cbuf_idx);
                break;
            case continue_mode:
                ret = max86xxx_sensor_pause(sd, MAX86XXX_ECPPG_MODE, 0);
                cbuf_idx += snprintf(cbuf,
                                     sizeof(cbuf) - cbuf_idx - 1,
                                     "\r\n%s err=%d\r\n",
                                     ptr_ch,
                                     ret);
                printf(cbuf);
                comm_report_data(cbuf, cbuf_idx);
                break;
            case pause_mode:
                ret = max86xxx_sensor_pause(sd, MAX86XXX_ECPPG_MODE, 1);
                cbuf_idx += snprintf(cbuf,
                                     sizeof(cbuf) - cbuf_idx - 1,
                                     "\r\n%s err=%d\r\n",
                                     ptr_ch,
                                     ret);
                printf(cbuf);
                comm_report_data(cbuf, cbuf_idx);
                break;
            case ecppg_agc_disable:
                ret = max86xxx_ecppg_agc_enable(sd, 0);
                cbuf_idx += snprintf(cbuf,
                                     sizeof(cbuf) - cbuf_idx - 1,
                                     "\r\n%s err=%d\r\n",
                                     ptr_ch,
                                     ret == 0 ? 0 : -255);
                printf(cbuf);
                comm_report_data(cbuf, cbuf_idx);
                break;
            case ecppg_agc_enable:
                ret = max86xxx_ecppg_agc_enable(sd, 1);
                cbuf_idx += snprintf(cbuf,
                                     sizeof(cbuf) - cbuf_idx - 1,
                                     "\r\n%s err=%d\r\n",
                                     ptr_ch,
                                     ret == 0 ? 0 : -255);
                printf(cbuf);
                comm_report_data(cbuf, cbuf_idx);
                break;
            case ecppg_ecg_sr:
                ret = max86xxx_get_param(ptr_ch, cmd_tbl[i], &param);
                ret |= max86xxx_set_ecg_sr(param);
                cbuf_idx += snprintf(cbuf,
                                     sizeof(cbuf) - cbuf_idx - 1,
                                     "\r\n%s err=%d\r\n",
                                     ptr_ch,
                                     ret == 0 ? 0 : -254);
                printf(cbuf);
                comm_report_data(cbuf, cbuf_idx);
                break;
            case ecppg_ecg_notch:
                ret = max86xxx_get_param(ptr_ch, cmd_tbl[i], &param);
                ret |= max86xxx_algo_set_ecg_notch(param);
                cbuf_idx += snprintf(cbuf,
                                     sizeof(cbuf) - cbuf_idx - 1,
                                     "\r\n%s err=%d\r\n",
                                     ptr_ch,
                                     ret == 0 ? 0 : -254);
                printf(cbuf);
                comm_report_data(cbuf, cbuf_idx);
                break;
            case ecppg_ecg_cutoff:
                ret = max86xxx_get_param(ptr_ch, cmd_tbl[i], &param);
                ret |= max86xxx_algo_set_ecg_cutoff(param);
                cbuf_idx += snprintf(cbuf,
                                     sizeof(cbuf) - cbuf_idx - 1,
                                     "\r\n%s err=%d\r\n",
                                     ptr_ch,
                                     ret == 0 ? 0 : -254);
                printf(cbuf);
                comm_report_data(cbuf, cbuf_idx);
                break;
            case ecppg_ecg_adapt:
                ret = max86xxx_get_param(ptr_ch, cmd_tbl[i], &param);
                max86xxx_algo_set_ecg_adapt((bool)param);
                cbuf_idx += snprintf(cbuf,
                                     sizeof(cbuf) - cbuf_idx - 1,
                                     "\r\n%s err=0\r\n",
                                     ptr_ch);
                printf(cbuf);
                comm_report_data(cbuf, cbuf_idx);
                break;

            default:
                break;
            }
        }
    }

    //Continue collecting data if we were interrupted by another command
    if (output->last_cmd == ecppg_mode0 && output->user_cmd != stop) {
        output->user_cmd = output->last_cmd;
    }

    if (!recognizedCmd) {
        cbuf_idx += snprintf(cbuf,
                             sizeof(cbuf) - cbuf_idx - 1,
                             "\r\n%s err=-255\r\n",
                             &output->cmd_str[0]);
        printf(cbuf);
        comm_report_data(cbuf, cbuf_idx);
    }
}

#ifdef CONFIG_BTSTACK_ENABLE
int btstack_data_transmit_handler(uint8_t *buf, int buf_sz)
{
    int                   tx_len = 0;
    struct max86xxx_dev * sd     = max86xxx_get_device_data();
    max86xxx_sns_report_t data_report;
    data_packet_mode0 *   dp_mode0;

    int len = comm_get_fragmented_data(buf, buf_sz);
    if (len > 0)
        return len;

    if (max86xxx_data_report_mode == stop)
        return -1;

    if (sd->pause_mode)
        return -1;

    while (tx_len < (buf_sz - sizeof(data_packet_mode0))) {
        if (max86xxx_algo_execute_once(&data_report) < 0) {
            return tx_len;
        }

        switch (max86xxx_data_report_mode) {
        case ecppg_mode0:
#ifdef ASCII_COMM
            tx_len += snprintf(buf,
                               sizeof(buf_sz),
                               "%d,%d,%d,%d,%.1f\r\n",
                               data_report.sample_cnt,
                               (int)data_report.ir,
                               (int)data_report.red,
                               (int)data_report.ecg_raw,
                               (float)data_report.ecg_filtered);
#else
            if (sizeof(data_packet_mode0) > buf_sz)
                return -1;

            dp_mode0 = (data_packet_mode0 *)(buf + tx_len);
            memset(dp_mode0, 0, sizeof(data_packet_mode0));
            dp_mode0->start_byte   = 0xAA;
            dp_mode0->sample_count = data_report.sample_cnt;
            dp_mode0->ir           = data_report.ir;
            dp_mode0->red          = data_report.red;
            dp_mode0->ecg          = data_report.ecg_raw;
            dp_mode0->ecg_filt     = data_report.ecg_filtered;

            dp_mode0->crc8 =
                    crc8((uint8_t *)dp_mode0, sizeof(data_packet_mode0) - 1);
            tx_len += sizeof(data_packet_mode0);
#endif
            break;

        default:
            return -1;
        }
    }

    if (console_interface_exists) {
        enter_critical_section();
        UART_Write(MXC_UART_GET_UART(CONSOLE_UART), buf, tx_len);
        exit_critical_section();
    }

    return tx_len;
}

#else

void max86xxx_data_report_execute(void)
{
    struct max86xxx_dev *sd = max86xxx_get_device_data();
    max86xxx_sns_report_t data_report;
    data_packet_mode0 *dp_mode0;
    char console_tx_buf[256];
    int16_t console_tx_len = 0;

    if (max86xxx_data_report_mode == stop ||
        max86xxx_algo_execute_once(&data_report) < 0)
        return;

    switch (max86xxx_data_report_mode) {
    case ecppg_mode0:
#ifdef ASCII_COMM
        console_tx_len = snprintf(console_tx_buf,
                                  sizeof(console_tx_buf),
                                  "%d,%d,%d,%d,%.1f\r\n",
                                  data_report.sample_cnt,
                                  (int)data_report.ir,
                                  (int)data_report.red,
                                  (int)data_report.ecg_raw,
                                  (float)data_report.ecg_filtered);
#else

        dp_mode0 = (data_packet_mode0 *)(&console_tx_buf[0]);
        memset(dp_mode0, 0, sizeof(data_packet_mode0));
        dp_mode0->start_byte   = 0xAA;
        dp_mode0->sample_count = data_report.sample_cnt;
        dp_mode0->ir           = data_report.ir;
        dp_mode0->red          = data_report.red;
        dp_mode0->ecg          = data_report.ecg_raw;
        dp_mode0->ecg_filt     = data_report.ecg_filtered;

        dp_mode0->crc8 =
                crc8((uint8_t *)dp_mode0, sizeof(data_packet_mode0) - 1);
        console_tx_len                 = sizeof(data_packet_mode0);
        console_tx_buf[console_tx_len] = '\0';
#endif
        break;

    default:
        return;
    }

    if (!sd->pause_mode && console_interface_exists) {
        enter_critical_section();
        UART_Write(MXC_UART_GET_UART(CONSOLE_UART),
                   (uint8_t *)console_tx_buf,
                   console_tx_len);
        exit_critical_section();
    }

    //pulseox_frontend(&data_report, max86xxx_data_report_mode);
}
#endif

void max86xxx_build_command(char ch)
{
    max86xxx_comm_t *output = max86xxx_get_comm_data();

    if (!output->silent_mode) /* BUG: POTENTIAL BUG, what uart port to echo, not only console */
        UART_Write(MXC_UART_GET_UART(CONSOLE_UART), (uint8_t *)&ch, 1);

    if (ch == 0x00) {
        printf("Ignored char 0x00\n");
        return;
    }

    if ((ch == '\n') || (ch == '\r')) {
        max86xxx_parse_command();

        //Clear cmd_str
        while (output->cmd_idx >
               0) /* BUG: POTENTIAL BUG for multiple port access */
            output->cmd_str[--output->cmd_idx] = '\0';

    } else if ((ch == 0x08 || ch == 0x7F) && output->cmd_idx > 0) {
        //Backspace character
        if (output->cmd_idx > 0)
            output->cmd_str[--output->cmd_idx] = '\0';
    } else {
        /* BUG: POTENTIAL BUG for multiple port access */
        if (output->cmd_idx < MAX86XXX_STR_BUF_SZ)
            output->cmd_str[output->cmd_idx++] = ch;
    }
}

#endif USE_COMM
