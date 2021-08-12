/**
 * @file imu.c
 * @author Carl Mattatall (cmattatall2@gmail.com), Thomas Christison (christisonthomas@gmail.com)
 * @brief Source module for the inertial measurement unit API
 * @version 0.1
 * @date 2021-01-02
 *
 * @copyright Copyright (c) 2021 Carl Mattatall, Thomas Christison
 *
 * @note this entire source module is ugly as sin...
 *
 * @todo for now we can just use CC to hide direct register
 * writes and have native build succeed, but in future an I2C API really
 * should be written so core application is portable to other devices.
 *
 *  Datasheet for BNO055: https://cdn-shop.adafruit.com/datasheets/BST_BNO055_DS000_12.pdf
 *
 *  Datasheet for BMX160: https://www.bosch-sensortec.com/media/boschsensortec/downloads/datasheets/bst-bmx160-ds0001.pdf
 *  For BMX160 example with abstracted host mcu see: https://github.com/BoschSensortec/BMI160_driver/blob/master/examples/read_sensor_data/read_sensor_data.c
 */

/*********************************************************************/

#include <stdio.h>
#include <string.h>
#include <stdint.h>

/** @note THIS AND ALL REGISTER LEVEL MANIPS SHOULD BE REFACTORED AWAY INTO
 * DRIVER LEVEL API CALLS IN THE FUTURE */
#include <msp430.h>

#include "targets.h"
#include "imu.h"
#include "i2c_b1.h"

#if defined(BNO055)
#include "bno055.h"
#endif /* #if defined(BNO055) */

#if defined(BMX160)
#include "bmi160.h"
#endif /* #if defined(BMX160) */

/**************** Static Function Declarations ***********************/

static void IMU_init_i2c(void);

/***************** Global Variables **********************************/

imu_dev_t imu_dev;
extern uint8_t ReceiveBuffer[]; /* defined in i2c_b1.c*/
/***************** Function Definitions ******************************/

/*
 * @brief format from interface document : {"imu" : [ +5, +2, -3] }
 */
int IMU_measurements_to_string(char *buf, unsigned int buflen, const imu_sensor_data_t gyro_readings)
{
    CONFIG_ASSERT(buf != NULL);

    /* May need to calculate gyro measurements based on reading and gyro dps range*/
    //imu_dev.gyro_cfg.bw


#warning /** @todo IMPLEMENT string conversion */

    return 0;
}

/*
 * @brief
 */
int8_t IMU_init(void)
{
    int8_t rslt;

    IMU_init_i2c();

    /* link read/write/delay function to appropriate IMU function call prototypes */
    imu_dev.read        = IMU_I2C_bus_read;         /* Read Function Pointer  */
    imu_dev.write       = IMU_I2C_bus_write;        /* Write Function Pointer */
    imu_dev.delay_ms    = delay_ms;                 /* Delay Function Pointer */

#if defined(BNO055)

    /* Configure BNO055 Sensor Settings*/

#endif /* #if defined(BNO055) */

#if defined(BMX160)

    /* Restart the device - All register values are overwritten with default parameters*/
    //bmi160_soft_reset(&imu_dev);

    /* Set correct I2C address */
    imu_dev.id      = BMX160_I2C_ADDR;              /* Set I2C device address */
    imu_dev.intf    = BMX160_I2C_INTF;              /* Set 0 for I2C interface */

    rslt = bmi160_init(&imu_dev);

    if(rslt != 0)
    {
        /* Error */
        return rslt;
    }

    else /* Initialization successful*/
    {
#warning Ensure gyro settings are properly selected
    /* Set IMU sensor & power configuration */

    /* Select the Output data rate, range of accelerometer sensor */
    //imu_dev.accel_cfg.odr     = BMI160_ACCEL_ODR_1600HZ;
    //imu_dev.accel_cfg.range   = BMI160_ACCEL_RANGE_16G;
    //imu_dev.accel_cfg.bw      = BMI160_ACCEL_BW_NORMAL_AVG4;

    /* Select the power mode of accelerometer sensor */
    //imu_dev.accel_cfg.power   = BMI160_ACCEL_NORMAL_MODE;

    /* Select the Output data rate, range of Gyroscope sensor */
    /* @todo SET DESIRED CONFIGURATION */

    imu_dev.gyro_cfg.odr        = BMI160_GYRO_ODR_25HZ;    /* Highest output datarate is 3200Hz
                                                              * lowest is 25Hz - See Table 15 in datasheet
                                                              */
    /*
     * Selection of ODR affects bandwidth of the sensor.
     * Frequency response is often represented as “bandwidth” in specification tables
     * for IMUs and gyroscopes. As a performance parameter, it represents the frequency
     * at which the output magnitude drops to about 70% (–3 dB) of the actual magnitude
     * of motion that the sensor is experiencing. In some cases, bandwidth may also be
     * defined by the frequency at which the output response lags the actual motion by
     * 90 degrees (for a 2-pole system).
     *
     * See table 15 in BMX160 Datasheet
     */

    imu_dev.gyro_cfg.range      = BMI160_GYRO_RANGE_125_DPS;    /* Lowest full range for highest resolution measurements*/
    imu_dev.gyro_cfg.bw         = BMI160_GYRO_BW_NORMAL_MODE;   /* No oversampling */


    /* Select the power mode of Gyroscope sensor */
    imu_dev.gyro_cfg.power      = BMI160_GYRO_NORMAL_MODE;

    /* Set the sensor configuration */
    rslt = bmi160_set_sens_conf(&imu_dev);

    }

#endif /* #if defined(BMX160) */

    return rslt;

}

/*
 * @brief Initialize appropriate I2C interface
 */
static void IMU_init_i2c(void)
{
    i2c_b1_init();
}


/*
 * @brief
 *    int8_t bmi160_get_regs(uint8_t reg_addr,      uint8_t *data,      uint16_t len,   const struct bmi160_dev *dev)
 *           bmi160_get_regs(BMI160_GYRO_DATA_ADDR, data_array,         6,              dev);
 *
 *                 dev->read(dev->id,               reg_addr,           data,           len);
 */
int8_t IMU_I2C_bus_read(uint8_t dev_addr, uint8_t reg_addr, uint8_t *reg_data, uint8_t cnt)
{
#if defined(BNO055)
    /* Write i2c bus read function for BNO055 */
#endif /* #if defined(BNO055) */

#if defined(BMX160)

    uint8_t  stringpos, rslt;

    //I2C_Mode rslt;/* = IDLE_MODE;*/

    /*
     * See page 99 - 100 in BMX160 datasheet
     *
     * First the master writes two bytes to I2C bus to tell the slave what data it wants to read:
     *
     * [Slave Address with R/W bit = 0][ Register Address]
     *
     */

    rslt = I2CB1_Master_WriteReg(dev_addr, reg_addr, 0, 0);

    if (rslt == BMI160_OK)
    {
    /*
     * Next the master writes another byte to the I2C bus
     * [Slave Address with R/W bit = 1]
     * Now the slave takes over the bus and pushes cnt number of bytes to the bus for the master to read
     */
    rslt = I2CB1_Master_ReadReg(dev_addr, reg_addr, cnt); // Received bytes in ReceiveBuffer

        /* Copy ReceiveBuffer into reg_data pointer */
        for (stringpos = 0; stringpos < cnt; stringpos++)
        {
            *(reg_data + stringpos) = ReceiveBuffer[stringpos];
        }
    }

    return rslt;

#endif /* #if defined(BMX160) */

}

/*
 * @brief
 *
 * @note
 *      for BMX160 invoked in bmi160.c's function:
 *          int8_t bmi160_set_regs(uint8_t reg_addr, uint8_t *data, uint16_t len, const struct bmi160_dev *dev)
 *      using format:
 *          dev->write(dev->id, reg_addr, data, len);
 */
int8_t IMU_I2C_bus_write(uint8_t dev_addr, uint8_t reg_addr, uint8_t *reg_data, uint8_t cnt)
{
#if defined(BNO055)
#endif /* #if defined(BNO055) */

#if defined(BMX160)

    /* See page 99 in BMX160 Datasheet
     *
     * Master writes three bytes to I2C bus to tell the slave which register to write, followed by the byte to write
     *
     * [Slave Address with R/W bit = 0][Register Address][Data byte to write]
     *
     */
    return (int8_t)I2CB1_Master_WriteReg(dev_addr, reg_addr, reg_data, cnt);

#endif /* #if defined(BMX160) */
}

/*
 * @brief delay ms number of milliseconds on msp430
 *
 * example source: https://www.embeddedrelated.com/showcode/314.php
 *
 * clock configuration info:
 * http://www.simplyembedded.org/tutorials/msp430-configuration/
 * also see datasheet/family user guide
 *
 *      BCSCTL1 = CALBC1_1MHZ;  // Basic clock system control 1
 *      DCOCTL = CALDCO_1MHZ;   // DCOCLK: internal digitally controller oscillator
 *
 *      On power up or after a reset, the device is configured such that
 *      MCLK (the master clock used by the CPU) is sourced from DCOCLK
 *      which has a frequency of approximately 1.1MHz.
 *
 */
void delay_ms(uint32_t ms)
{
    /** @todo */
    while (ms)
    {
        __delay_cycles(1000); /* 1000 for 1MHz and 16000 for 16MHz */
        ms--;
    }
}


/*
 * @brief
 */
int8_t IMU_get_gyro(imu_sensor_data_t *gyro_data)
{

#if defined(BMX160)

    imu_sensor_data_t acc_data_dummy;

    /*     * API call reads sensor data, stores it in
     * the imu_sensor_data_t structure pointer passed by the user.
     * The user can ask for accel data ,gyro data or both sensor
     * data using (BMI160_ACCEL_SEL | BMI160_GYRO_SEL) enum
     */
    return bmi160_get_sensor_data(BMI160_GYRO_SEL, &acc_data_dummy, gyro_data, &imu_dev);

#endif /* #if defined(BMX160) */
}

/*
 * @brief
 */
int8_t IMU_self_test(void)
{

#if defined(BMX160)

    return bmi160_perform_self_test(BMI160_GYRO_ONLY, &imu_dev);

#endif /* #if defined(BMX160) */
}

/*
 * @brief
 */
int8_t IMU_get_offsets(imu_sensor_offsets_t *offset)
{

#if defined(BMX160)

    /* See IMU datasheet for offset calibration information */
    return bmi160_get_offsets(offset, &imu_dev);

#endif /* #if defined(BMX160) */
}

/*
 * @brief
 */
int8_t IMU_set_offsets(const imu_fast_off_comp_t *foc_conf, const imu_sensor_offsets_t *offset)
{

#if defined(BMX160)

    /* See IMU datasheet for offset calibration information */
    return bmi160_set_offsets(foc_conf, offset, &imu_dev);

#endif /* #if defined(BMX160) */
}

/*
 * @brief
 */
int8_t IMU_get_power_mode(imu_power_mode_status_t *powermode)
{

#if defined(BMX160)

    return bmi160_get_power_mode(powermode, &imu_dev);

#endif /* #if defined(BMX160) */
}

/*
 * @brief
 */
void IMU_reset(void)
{

#if defined(BMX160)

    bmi160_soft_reset(&imu_dev); /* restarts device */
    IMU_init();                  /* reconfigure IMU */

#endif /* #if defined(BMX160) */
}






