/*
 * max86xxx_algo.c
 *
 *  Created on: May 11, 2021
 *      Author: strngr
 */
#include "max86xxx_algo.h"

void max86xxx_algo_init(void)
{
    printf("algo_init function must be implemented\n");
}

int max86xxx_set_ecg_sr(u16 sample_rate)
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
        printf("%s - error writing reg!\n", __func__);
        return 1;
    }
    return 0;
}
