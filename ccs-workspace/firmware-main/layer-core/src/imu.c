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
#include "i2c.h"

#if defined(BNO055)
#include "bno055.h"
#endif /* #if defined(BNO055) */

#if defined(BMX160)
#include "bmi160.h"
#endif /* #if defined(BMX160) */

/**************** Static Function Declarations ***********************/
/*
int8_t IMU_I2C_bus_read(u8 dev_addr, u8 reg_addr, u8 *reg_data, u8 cnt);
int8_t IMU_I2C_bus_write(u8 dev_addr, u8 reg_addr, u8 *reg_data, u8 cnt);
*/
static void IMU_init_i2c(void);
/***************** Global Variables **********************************/

imu_dev_t imu_dev;

/***************** Function Definitions ******************************/

/*
 * @brief format from interface document : {"imu" : [ +5, +2, -3] }
 */
int IMU_measurements_to_string(char *buf, unsigned int buflen)
{
    CONFIG_ASSERT(buf != NULL);

    /** @todo IMPLEMENT string conversion */
#warning IMPLEMENT IMU_init_i2c IN TERMS OF THE I2C API DRIVER FUNCTIONS

    return 0;
}

/*
 * @brief
 */
void IMU_init(void)
{

    IMU_init_i2c();

    /* link read/write/delay function to appropriate IMU function call prototypes */
    imu_dev.read        = IMU_I2C_bus_read;         /* Read Function Pointer  */
    imu_dev.write       = IMU_I2C_bus_write;        /* Write Function Pointer */
    imu_dev.delay_ms    = delay_ms;                 /* Delay Function Pointer */

#if defined(BNO055)

    /* Configure BNO055 Sensor Settings*/

#endif /* #if defined(BNO055) */

#if defined(BMX160)

    int8_t rslt;

    /* Restart the device - All register values are overwritten with default parameters*/
    //bmi160_soft_reset(&imu_dev);

    /* Set correct I2C address */s
    imu_dev.id      = BMX160_I2C_ADDR;              /* Set I2C device address */
    imu_dev.intf    = BMX160_I2C_INTF;              /* Set 0 for I2C interface */

    rslt = bmi160_init(&imu_dev);

    /*if (rslt == BMX160_OK)
    {
        printf("BMX160 initialization success !\n");
        printf("Chip ID 0x%X\n", imu_dev.chip_id);
    }
    else
    {
        printf("BMIX60 initialization failure !\n");
        exit();
    }*/

    /* Set IMU sensor & power configuration */

    /* Select the Output data rate, range of accelerometer sensor */
    //imu_dev.accel_cfg.odr     = BMI160_ACCEL_ODR_1600HZ;
    //imu_dev.accel_cfg.range   = BMI160_ACCEL_RANGE_16G;
    //imu_dev.accel_cfg.bw      = BMI160_ACCEL_BW_NORMAL_AVG4;

    /* Select the power mode of accelerometer sensor */
    //imu_dev.accel_cfg.power   = BMI160_ACCEL_NORMAL_MODE;

    /* Select the Output data rate, range of Gyroscope sensor */
    imu_dev.gyro_cfg.odr        = BMI160_GYRO_ODR_3200HZ;
    imu_dev.gyro_cfg.range      = BMI160_GYRO_RANGE_2000_DPS;
    imu_dev.gyro_cfg.bw         = BMI160_GYRO_BW_NORMAL_MODE;

    /* Select the power mode of Gyroscope sensor */
    imu_dev.gyro_cfg.power      = BMI160_GYRO_NORMAL_MODE;

    /* Set the sensor configuration */
    rslt = bmi160_set_sens_conf(&imu_dev);

#endif /* #if defined(BMX160) */

#if !defined(BNO055) && !defined(BMX160)
    printf("Called %s\n", __func__);                /* @todo Is this still necessary? */
#endif /* !defined(BNO055) && !defined(BMX160) */

}

/*
 * @brief Initialize appropriate I2C interface
 */
static void IMU_init_i2c(void)
{

#if defined(BNO055)
    /* We use I2C1 (using UCB1) for BNO055 IMU in Rev A ONLY */
    I2C1_init();
#endif /* #if defined(BNO055) */

#if defined(BMX160)
    /* IN REV B We use I2C0 (using UCB0) with replacement BMX160 due to chip shortage */
    I2C0_init();
#endif /* #if defined(BMX160) */

}









/*
 * @brief
 * int8_t bmi160_get_regs(uint8_t reg_addr, uint8_t *data, uint16_t len, const struct bmi160_dev *dev)
 *
 */
int8_t IMU_I2C_bus_read(u8 dev_addr, u8 reg_addr, u8 *reg_data, u8 cnt)
{
#if defined(BNO055)
    /** @todo */
#warning THIS NEEDS TO BE IMLPEMENTED

    s32 BNO055_iERROR = BNO055_INIT_VALUE;
#if 0
    u8  array[I2C_BUFFER_LEN] = {BNO055_INIT_VALUE};
    u8  stringpos             = BNO055_INIT_VALUE;


    array[BNO055_INIT_VALUE] = reg_addr;

    /* Please take the below API as your reference
     * for read the data using I2C communication
     * add your I2C read API here.
     * "BNO055_iERROR = I2C_WRITE_READ_STRING(DEV_ADDR,
     * ARRAY, ARRAY, 1, CNT)"
     * BNO055_iERROR is an return value of SPI write API
     * Please select your valid return value
     * In the driver BNO055_SUCCESS defined as 0
     * and FAILURE defined as -1
     */
    for (stringpos = BNO055_INIT_VALUE; stringpos < cnt; stringpos++)
    {
        *(reg_data + stringpos) = array[stringpos];
    }
#endif
    return (s8)BNO055_iERROR;
#endif /* #if defined(BNO055) */

#if defined(BMX160)
    int8_t rslt = BMI160_OK;

    // copied from studying example at:
    // https://github.com/DFRobot/DFRobot_BMX160/blob/master/DFRobot_BMX160.cpp
    uint8_t outGoingBytes[] = {BMX160_MAG_DATA_ADDR, 0};
    I2C0_write_bytes(dev_addr, outGoingBytes, sizeof(outGoingBytes));
    I2C0_read_bytes(reg_data, cnt);

    return 0;

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
int8_t IMU_I2C_bus_write(u8 dev_addr, u8 reg_addr, u8 *reg_data, u8 cnt)
{
#if defined(BNO055)
    s32 BNO055_iERROR = BNO055_INIT_VALUE;


    char txbuf[50]; /* just hard coding this for now */
    txbuf[0] = reg_addr;

    unsigned int bcnt = sizeof(reg_addr) + cnt;
    if (bcnt > sizeof(txbuf))
    {
        bcnt = sizeof(txbuf);
    }
    strncpy(&txbuf[1], (char *)reg_data, bcnt);

    int write_status = I2C1_write_bytes((uint8_t)dev_addr, txbuf, bcnt);
    if (write_status != 0 && write_status != -1)
    {
        BNO055_iERROR = BNO055_SUCCESS;
    }

    /* SEE SECTION 4.6 OF BNO055 DATASHEET - I2C Protocol*/
#if 0
    u8  array[I2C_BUFFER_LEN];
    u8  stringpos = BNO055_INIT_VALUE;

    array[BNO055_INIT_VALUE] = reg_addr;
    for (stringpos = BNO055_INIT_VALUE; stringpos < cnt; stringpos++)
    {
        array[stringpos + BNO055_I2C_BUS_WRITE_ARRAY_INDEX] =
            *(reg_data + stringpos);
    }
#endif /* #if 0 */
    return (s8)BNO055_iERROR;
#endif /* #if defined(BNO055) */


#if defined(BMX160)
    int8_t rslt = BMX160_OK;

    char txbuf[50]; /* just hard coding this for now */
    txbuf[0] = reg_addr;

    unsigned int bcnt = sizeof(reg_addr) + cnt;
    if (bcnt > sizeof(txbuf))
    {
        bcnt = sizeof(txbuf);
    }
    strncpy(&txbuf[1], (char *)reg_data, bcnt);

    int write_status = I2C0_write_bytes((uint8_t)dev_addr, txbuf, bcnt);
    if (write_status != 0 && write_status != -1)
    {
        rslt = IMU_WRITE_SUCCESS;
    }

    /*  */
#if 0
    u8  array[I2C_BUFFER_LEN];
    u8  stringpos = BMX160_INIT_VALUE;

    array[BMX160_INIT_VALUE] = reg_addr;
    for (stringpos = BMX160_INIT_VALUE; stringpos < cnt; stringpos++)
    {
        array[stringpos + BMX160_I2C_BUS_WRITE_ARRAY_INDEX] =
            *(reg_data + stringpos);
    }
#endif /* #if 0 */
    return rslt;
#endif /* #if defined(BMX160) */
}

/*
 * @brief delay ms number of milliseconds on msp430
 *
 * example source: https://www.embeddedrelated.com/showcode/314.php
 */
void delay_ms(uint32_t ms)
{
    /** @todo */
#warning THIS NEEDS TO BE IMLPEMENTED

}


/*
 * @brief
 */
int8_t IMU_get_gyro(imu_sensor_data_t *gyro_data)
{

#if defined(BMX160)

    //int8_t rslt = BMX160_OK;

    imu_sensor_data_t acc_data_dummy;

    /*
     * API call reads sensor data, stores it in
     * the imu_sensor_data_t structure pointer passed by the user.
     * The user can ask for accel data ,gyro data or both sensor
     * data using (BMI160_ACCEL_SEL | BMI160_GYRO_SEL) enum
     */
    return bmi160_get_sensor_data(BMI160_GYRO_SEL, &acc_data_dummy, &gyro_data, &imu_dev);

#endif /* #if defined(BMX160) */
}


/*


IMU_self_test();

IMU_set_offsets();

IMU_get_offsets()

IMU_set_power_mode();

IMU_get_power_mode();

    //int8_t bmi160_get_sens_conf(struct bmi160_dev *dev);
    //int8_t bmi160_set_sens_conf(struct bmi160_dev *dev);
*/
