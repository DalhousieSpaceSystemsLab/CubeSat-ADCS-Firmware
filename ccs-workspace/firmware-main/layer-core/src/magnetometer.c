/**
 * @file sun_sensors.c
 * @author Carl Mattatall (cmattatall2@gmail.com)
 * @brief Source module to implement the sun sensor interface
 * @version 0.2
 * @date 2021-01-01
 *
 * @copyright Copyright (c) 2021 Carl Mattatall
 *
 *
 * @note PINOUT:
 *  P2.7 == SPI CHIP SELECT
 *
 * @todo IMPLEMENT RESET FUNCTION
 */

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <stdbool.h>

#include "attributes.h"
#include "config_assert.h"

#include "magnetometer.h"
#include "magnetorquers.h"

#if defined(TARGET_MCU)
#include <msp430.h>

#include "spi.h"
#include "ads7841e.h"
#endif /* #if defined(TARGET_MCU) */


/** @todo DON'T FORGET THIS */
#warning FIGURE OUT WHICH CHANNELS CORRESPOND TO WHICH FACES
#warning FIGURE OUT WHICH CHANNELS CORRESPOND TO WHICH FACES
#warning FIGURE OUT WHICH CHANNELS CORRESPOND TO WHICH FACES
#define MAGTOM_ADS7841_X_FACE_CHANNEL (ADS7841_CHANNEL_SGL_1)
#define MAGTOM_ADS7841_Y_FACE_CHANNEL (ADS7841_CHANNEL_SGL_2)
#define MAGTOM_ADS7841_Z_FACE_CHANNEL (ADS7841_CHANNEL_SGL_3)


//typedef struct MAGTOM_measurement MAGTOM_measurement_t;

static void MAGTOM_enable_ADS7841(void);
static void MAGTOM_disable_ADS7841(void);
static void MAGTOM_init_phy(void);

MAGTOM_measurement_t MAGTOM_get_measurement(void);


static bool phy_initialized = false;


void MAGTOM_init(void)
{
    MAGTOM_init_phy();
    phy_initialized = true;
}


void MAGTOM_reset(void)
{
#if defined(TARGET_MCU)
    /* Issue reset command to IMU */

/** @todo RESET THE MAGNETOMETER */
#warning NOT IMPLEMENTED YET
#else
    printf("Called %s\n", __func__);
#endif /* #if defined(TARGET_MCU) */
}


int MAGTOM_measurement_to_string(char *buf, int buflen)
{
    CONFIG_ASSERT(NULL != buf);
    MAGTOM_measurement_t meas            = MAGTOM_get_measurement();
    int                  required_length = 0;
    required_length = snprintf(buf, buflen, "[ %.4f, %.4f, %.4f ]", meas.x_BMAG,
                               meas.y_BMAG, meas.z_BMAG);
    return (required_length < buflen) ? 0 : 1;
}


static void MAGTOM_enable_ADS7841(void)
{
#if defined(TARGET_MCU)

    P2OUT &= ~BIT7;

#else
    printf("Called %s\n", __func__);
#endif /* #if defined(TARGET_MCU) */
}


static void MAGTOM_disable_ADS7841(void)
{
#if defined(TARGET_MCU)

    P2OUT |= BIT7;

#else
    printf("Called %s\n", __func__);
#endif /* #if defined(TARGET_MCU) */
}


MAGTOM_measurement_t MAGTOM_get_measurement(void)
{
    MAGTOM_measurement_t data = {0};

    /* Disable magnetorquers before measurement */
    int MQTR_x_mv = MQTR_get_coil_voltage_mv(MQTR_x);
    int MQTR_y_mv = MQTR_get_coil_voltage_mv(MQTR_y);
    int MQTR_z_mv = MQTR_get_coil_voltage_mv(MQTR_z);

    /** @todo This is a hacky delay to allow coils to de-energize.
     * In future firmware revs this can be replaced by a timer expiration
     * + callback */
    volatile uint16_t blocking_delay = 0;
    while (++blocking_delay < UINT16_MAX)
        ;

    /* Prevent caller API error (normally they have to call init first) */
    if (!phy_initialized)
    {
        MAGTOM_init_phy();
    }

#if defined(TARGET_MCU)

    MAGTOM_reset();

    ADS7841_driver_init(MAGTOM_enable_ADS7841, MAGTOM_disable_ADS7841,
                        ADS7841_PWRMODE_stayOn, ADS7841_BITRES_12);

    /*
     * Measure the conversion value (singled ended mode) from channel 2 on the
     * ADS7841 chip that is selected using functions MAGTOM_enable_ADS7841 and
     * MAGTOM_disable_ADS7841
     */
    data.x_BMAG = ADS7841_measure_channel(MAGTOM_ADS7841_X_FACE_CHANNEL);
    data.y_BMAG = ADS7841_measure_channel(MAGTOM_ADS7841_Y_FACE_CHANNEL);
    data.z_BMAG = ADS7841_measure_channel(MAGTOM_ADS7841_Z_FACE_CHANNEL);
    /*2021-09-14 Update:the output result from ADS7841_measure_channel is
     *mV = (output/4096)*3.3, need some convertion to gauss (uT)
     */

    float a,b,c;
    a = (data.x_BMAG / 4096)*3.3;
    b = (data.y_BMAG / 4096)*3.3;
    c = (data.z_BMAG / 4096)*3.3;
    //printf("z voltage:%f; y voltage:%f; x voltage:%f\n", a,b,c);


    data.x_BMAG = ((data.x_BMAG / 4096)*3.3 - 1.65) / 244.6 / 3.3 / 0.0033 *100;
    data.y_BMAG = ((data.y_BMAG / 4096)*3.3 - 1.65) / 244.6 / 3.3 / 0.0033 *100;
    data.z_BMAG = ((data.z_BMAG / 4096)*3.3 - 1.65) / 244.6 / 3.3 / 0.0033 *100;


    ADS7841_driver_deinit();

    /* Re-enable magneqtorquers */
    MQTR_set_coil_voltage_mv(MQTR_x, MQTR_x_mv);
    MQTR_set_coil_voltage_mv(MQTR_y, MQTR_y_mv);
    MQTR_set_coil_voltage_mv(MQTR_z, MQTR_z_mv);

#else
    printf("Called %s\n", __func__);
#endif /* #if defined(TARGET_MCU) */
    return data;
}


static void MAGTOM_init_phy(void)
{
#if defined(TARGET_MCU)

    /* De-select ADS7841 for the magnetometer */
    P2DIR |= BIT7;
    P2OUT |= BIT7;

#else
    printf("Called %s\n", __func__);
#endif /* #if defined(TARGET_MCU) */
}
