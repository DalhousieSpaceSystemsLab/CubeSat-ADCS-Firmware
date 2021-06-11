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

#include <msp430.h>

#include "bmi160.h"
#include "i2c.h"

/* Comment out following line if using example on Rev B Motherboard target*/
#define MSP_LAUNCH_PAD /* target microcontroller - MSPlaunchpad used for sensor testing and characterization*/

int main()
{
    WDTCTL = WDTPW + WDTHOLD; // Stop watchdog timer

#if defined(MSP_LAUNCH_PAD)

    /* MSP430F5529 Launch Pad Indicator LED (P1.0)*/
    P6DIR |= BIT0;
    P6OUT |= BIT0;

#elif /* Using ADCS Motherboard as target*/

    /* Motherboard Indicator LED (P6.0)*/
    P6DIR |= BIT0;
    P6OUT |= BIT0;

#endif /* #if defined(MSP430LP) */

    /* Initialize IMU */
    /*
     * Soft reset - restarts device
     * get power mode
     *
     *
     */
    IMU_init_i2c();



    /* read sensor data
     *
     */


    /* self test
     *
     */

    /* Offsets
     *
     */

    while (1)
    {
        /* read gyro data*/
    }
}
