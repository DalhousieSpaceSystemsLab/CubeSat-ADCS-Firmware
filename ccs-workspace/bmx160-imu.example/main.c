/**
 * @file main.c
 * @author thomas christison (christisonthomas@gmail.com)
 * @brief
 *
 *
 * @version 0.1
 * @date 2021-05-23
 *
 * @copyright Copyright (c) 2021 Thomas Christison
 *
 * https://www.bosch-sensortec.com/media/boschsensortec/downloads/datasheets/bst-bmx160-ds0001.pdf
 * use B0 I2C interface
 *
 * There is no dedicated BMX160 driver
 * The BMX160 is a BMI160 IMU with a BMM150 magnetometer connected to its aux port
 * - can reuse the BMI160 and BMM150 drivers
 *      - only need write, read, and delay functions for API
 * - need to change chip id in the bmi160_defs.h file to 0xD8
 * - example code https://github.com/BoschSensortec/BMI160_driver/wiki/How-to-use-an-auxiliary-sensor-or-magnetometer-with-the-BMI160.
 */

#include <msp430.h> 

int main(void)
{
    /*@todo
     * - Update device id
     * - update firmware main code imu.c to use bmx160 code instead of bno055 in Rev B board
     * - Rev B board uses B0 for IMU with I2C
     * - Rev A used B0 for spi with multiple devices.
     */

	WDTCTL = WDTPW | WDTHOLD;	// stop watchdog timer
	
	return 0;
}
