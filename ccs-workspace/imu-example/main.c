/**
 * @file main.c
 *
 * @purpose IMU api example for bmx160 imu
 *
 * @author thomas christison (christisonthomas@gmail.com)
 * @brief
 * @version 0.1
 * @date 2021-06-11
 *
 *
 * IMU Datasheet
 * https://www.bosch-sensortec.com/media/boschsensortec/downloads/datasheets/bst-bmx160-ds0001.pdf
 *
 * Rev B will be using default I2C address of 0x68 with ADDR select pin pulled to ground.
 *
 * BMX160 IMU is a sensor which internally contains the BMI160 IMU which has a vendor provided driver so that will be used
 *  - To use the vendor driver, three functions are required to be written for the microcontroller: write, read, and delay
 *
 * Additionally we should configure the IMU for the sensor configuration we desire.
 *  - In this case we only require the gyroscope data and can ignore the magnetometer and accelerometer configuration data.
 *  - The additional sensors can be integrated into the ADCS in the future if desired by extenting our code base and system to
 *    use the measurements.
 *
 */

#define TARGET_MCU

/* Comment out following line if using example on Rev B Motherboard target*/
#define MSP_LAUNCH_PAD /* target microcontroller - MSPlaunchpad used for sensor testing and characterization*/

#include <msp430.h>

#include "imu.h"
#include "indicator_led.h"


int main()
{
    int8_t rslt;

    imu_sensor_data_t gyro_readings = {0};              /* */
    imu_sensor_offsets_t imu_offsets = {0};             /* */
    imu_power_mode_status_t imu_power_status = {0};     /* */
    //imu_fast_off_comp_t imu_foc = {0};                  /* */

    WDTCTL = WDTPW + WDTHOLD;   /* Stop watchdog timer */

    led_init();                 /* Turn on indicator LED */

    IMU_init();                 /* Initialize IMU */

    rslt = IMU_self_test();            /* self test gyro sensor */

    rslt = IMU_get_offsets(&imu_offsets);               /* Offsets */

    //rslt = IMU_set_offsets(&imu_foc, &imu_offsets);   /* Offsets */

    rslt = IMU_get_power_mode(&imu_power_status);       /* Check IMU power mode config */

    rslt = IMU_get_gyro(&gyro_readings);   /* Get sensor data */

    IMU_reset();

    return rslt;
}

