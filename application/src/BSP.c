/**
 * @file BSP.c
 * @author Carl Mattatall (cmattatall2@gmail.com)
 * @brief BOARD SUPPORT PACKAGE FILE
 * @version 0.1
 * @date 2020-12-19
 *
 * @copyright Copyright (c) 2020
 *
 * @note
 * @todo
 */
#if defined(TARGET_MCU)
#include <msp430.h>
#endif /* #if defined(TARGET_MCU) */

#include <stdlib.h>

#include "BSP.h"
#include "callback_api.h"


void BSP_init(void)
{
#if defined(TARGET_MCU)
    P1DIR = 0xFF; /* Configure onboard red LED pin for output direction */
#else

#endif /* #if defined(TARGET_MCU) */
}

void BSP_toggle_red_led(__unused callback_args args)
{
#if defined(TARGET_MCU)
    P1OUT ^= 0x01;
#else

#endif /* #if defined(TARGET_MCU) */
}

void BSP_set_red_led(__unused callback_args args)
{
#if defined(TARGET_MCU)
    P1OUT = 0x01;

#else

#endif /* #if defined(TARGET_MCU) */
}

void BSP_clr_red_led(__unused callback_args args)
{
#if defined(TARGET_MCU)
    P1OUT = 0x00;
#else

#endif /* #if defined(TARGET_MCU) */
}
