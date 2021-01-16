/**
 * @file sun_sensors.c
 * @author Carl Mattatall (cmattatall2@gmail.com)
 * @brief Source module to implement the sun sensor interface
 * @version 0.1
 * @date 2021-01-01
 *
 * @copyright Copyright (c) 2021 Carl Mattatall
 *
 * @todo IMPLEMENT THE INTERFACE
 */

#include <stdlib.h>
#include <string.h>

#include "config_assert.h"
#include "sun_sensors.h"

#if defined(TARGET_MCU)
#include "spi.h"
#include "ads7841e.h"
#else
#endif /* #if defined(TARGET_MCU) */


static void               SUNSEN_enable_ADS7841(void);
static void               SUNSEN_disable_ADS7841(void);
static SUNSEN_intensity_t SUNSEN_get_sun_vector(void);


SUNSEN_measurement_t SUNSEN_measure_intensities(void)
{
    SUNSEN_measurement_t measurement;
    memset((void *)&measurement, 0, sizeof(measurement));
    unsigned int i;
    for (i = 0; i < NUM_SUN_SENSORS; i++)
    {
        measurement.vals[i] = SUNSEN_get_sun_vector();
    }
    return measurement;
}


static SUNSEN_intensity_t SUNSEN_get_sun_vector(void)
{
    ADS7841_driver_init(SUNSEN_enable_ADS7841, SUNSEN_disable_ADS7841,
                        ADS7841_PWRMODE_always_on, ADS7841_CONVMODE_12);
    SUNSEN_intensity_t bearing;
    bearing.first  = ADS7841_measure_channel(ADS7841_CHANNEL_1);
    bearing.second = ADS7841_measure_channel(ADS7841_CHANNEL_2);
    bearing.third  = ADS7841_measure_channel(ADS7841_CHANNEL_3);
    ADS7841_driver_deinit();
    return bearing;
}


static void SUNSEN_enable_ADS7841(void)
{
#if defined(TARGET_MCU)

    /** @todo SET SPI CS MUX TO DRIVE CS PIN OF ADS7841 ON SUN SENSOR LOW */

#warning NOT IMPLEMENTED YET

#else


#endif /* #if defined(TARGET_MCU) */
}


static void SUNSEN_disable_ADS7841(void)
{
#if defined(TARGET_MCU)
#else

#warning NOT IMPLEMENTED YET
    /** @todo SET SPI CS MUX TO DRIVE CS PIN OF ADS7841 ON SUN SENSOR HIGH */

#endif /* #if defined(TARGET_MCU) */
}
