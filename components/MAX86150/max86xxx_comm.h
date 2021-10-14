/*******************************************************************************
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
* OR OTHER LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHECONFIG_IDF_TARGET_ESP32RWISE,
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

#ifndef _MAX86XXX_COMM_H_
#define _MAX86XXX_COMM_H_

#ifdef USE_COMM

#include "sdkconfig.h" // esp platform only
#ifdef CONFIG_IDF_TARGET_ESP32
#define BOARD_NAME       "med_brac_board"
#define FIRMWARE_VERSION "1.0"
#endif

#define MAX86XXX_PART_NAME                 "max86150"
#define MAX86XXX_NAME                      "ecppg"
#define MAX86XXX_IO_ERR_A_GT_B_OR_A_NONPOS 1
#define MAX86XXX_IO_ERR_A_ABOVE_AB         2
#define MAX86XXX_IO_ERR_INPUT              3
#define MAX86XXX_DRIVER_VERSION            "2.3.02"

#if defined(BOARD_NAME)
#define MAX86XXX_PLATFORM BOARD_NAME
#else
#error "BOARD_NAME undefined"
#endif

#define MAX86XXX_STR_BUF_SZ 80

typedef enum _cmd_state_t {
    stop = 0,
    get_format_mode0,
    ecppg_mode0,
    get_device_info,
    get_reg,
    set_reg,
    dump_regs,
    echo_mode,
    silent_mode,
    continue_mode,
    pause_mode,
    ecppg_agc_disable,
    ecppg_agc_enable,
    ecppg_ecg_sr,
    ecppg_ecg_notch,
    ecppg_ecg_cutoff,
    ecppg_ecg_adapt,
    MAX86XXX_NUM_CMDS,
} cmd_state_t;

typedef struct max86xxx_comm {
    cmd_state_t user_cmd;
    cmd_state_t last_cmd;
    int         cmd_idx;
    char        cmd_str[MAX86XXX_STR_BUF_SZ];
    bool        silent_mode;
} max86xxx_comm_t;

extern volatile uint8_t console_interface_exists;

typedef enum {
    INTERFACE_PC_GUI_UART = 0,
    INTERFACE_NORDIC_BLE_UART
} enum_interface_t;

void max86xxx_process_execute(void);
int  max86xxx_comm_init(void);
void max86xxx_build_command(char ch);
void max86xxx_data_report_execute(void);

#endif /* USE_COMM */
#endif /* _MAX86XXX_COMM_H_ */
