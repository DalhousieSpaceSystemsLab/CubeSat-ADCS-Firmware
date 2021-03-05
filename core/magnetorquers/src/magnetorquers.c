/**
 * @file magnetorquers.c
 * @author Carl Mattatall (cmattatall2@gmail.com)
 * @brief Source module for interface to ADCS magnetorquers
 * @version 0.1
 * @date 2020-12-09
 *
 * @copyright Copyright (c) 2020
 *
 */
#include <stdlib.h>
#include <limits.h>
#include <string.h>
#include <stdio.h>

#if defined(TARGET_MCU)
#include <msp430.h>
#else
#endif /* #if defined(TARGET_MCU) */

#include "magnetorquers.h"
#include "mqtr_timer_pwm_api.h"
#include "targets.h"

#define MQTR_PWM_DEFAULT ((pwm_t)(PWM_DEFAULT))

typedef enum
{
    MQTR_DIR_pos = ROT_DIR_clock,
    MQTR_DIR_neg = ROT_DIR_anticlock,
} MQTR_DIR_t;


static int mqtr_voltage_mv[] = {
    [MQTR_x] = 0,
    [MQTR_y] = 0,
    [MQTR_z] = 0,
};


void mqtr_init(void)
{
    mqtr_pwm_init();
    mqtr_pwm_set_coil_voltage_mv(MQTR_x, 500);
    mqtr_pwm_set_coil_voltage_mv(MQTR_y, 1500);
    mqtr_pwm_set_coil_voltage_mv(MQTR_y, 2500);
}


void mqtr_set_config(MQTR_t mqtr, int volts_mv)
{
    switch (mqtr)
    {
        case MQTR_x:
        case MQTR_y:
        case MQTR_z:
        {
            mqtr_voltage_mv[mqtr] = volts_mv;
        }
        break;
        default:
        {
            /* do nothing */
        }
        break;
    }
}


void mqtr_config_apply(void)
{
#if defined(TARGET_MCU)
    mqtr_pwm_set_coil_voltage_mv(MQTR_x, mqtr_voltage_mv[MQTR_x]);
    mqtr_pwm_set_coil_voltage_mv(MQTR_y, mqtr_voltage_mv[MQTR_y]);
    mqtr_pwm_set_coil_voltage_mv(MQTR_z, mqtr_voltage_mv[MQTR_z]);
#else
    printf("called %s\n", __func__);
#endif /* #if defined(TARGET_MCU) */
}


int mqtr_config_to_str(char *buf, int buflen)
{
    CONFIG_ASSERT(NULL != buf);
    int required_len =
        snprintf(buf, buflen, "[ %d, %d, %d ]", mqtr_voltage_mv[MQTR_x],
                 mqtr_voltage_mv[MQTR_y], mqtr_voltage_mv[MQTR_z]);
    return (required_len < buflen) ? 0 : 1;
}
