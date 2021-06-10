/**
 * @file imu.c
 * @author Carl Mattatall (cmattatall2@gmail.com)
 * @brief Source module for the inertial measurement unit API
 * @version 0.1
 * @date 2021-01-02
 *
 * @copyright Copyright (c) 2021 Carl Mattatall
 *
 * @note this entire source module is ugly as sin...
 *
 * @todo for now we can just use CC to hide direct register
 * writes and have native build succeed, but in future an I2C API really
 * should be written so core application is portable to other devices.
 */
#include <stdio.h>
#include <string.h>
#include <stdint.h>

#include "targets.h"
#include "imu.h"

/** @note THIS AND ALL REGISTER LEVEL MANIPS SHOULD BE REFACTORED AWAY INTO
 * DRIVER LEVEL API CALLS IN THE FUTURE */
#include <msp430.h>

#include "i2c.h"

#define BMX160

#if defined(BNO055)

#include "bno055.h"

#endif /* #if defined(BNO055) */


#if defined(BMX160)

#include "bmi160.h"

#endif /* #if defined(BMX160) */


#define I2C_BUFFER_LEN 8
#define I2C0 5
#define BNO055_I2C_BUS_WRITE_ARRAY_INDEX ((u8)1)
#define BMX160_I2C_BUS_WRITE_ARRAY_INDEX ((u8)1)

/*
 * This is why stdint.h is a thing...
 * I shouldn't have to wrap your customized platform-specific typedefs for fixed
 * width integer types fffffss...
 * #TEXASINSTRUMENTSISGARBAGE
 */


#if defined(BNO055)

static struct bno055_t bno055;

static s8 BNO055_I2C_bus_read(u8 dev_addr, u8 reg_addr, u8 *reg_data, u8 cnt);
static s8 BNO055_I2C_bus_write(u8 dev_addr, u8 reg_addr, u8 *reg_data, u8 cnt);
static void BNO055_delay_msek(u32 msek);
static void IMU_init_i2c(void);

#endif /* #if defined(BNO055) */

#if defined(BMX160)

static struct bno055_t bno055;

static s8 BMX160_I2C_bus_read(u8 dev_addr, u8 reg_addr, u8 *reg_data, u8 cnt);
static s8 BMX160_I2C_bus_write(u8 dev_addr, u8 reg_addr, u8 *reg_data, u8 cnt);
static void BMX160_delay_msek(u32 msek);
static void IMU_init_i2c(void);

#endif /* #if defined(BMX160) */


/* format from interface document : {"imu" : [ +5, +2, -3] } */
int IMU_measurements_to_string(char *buf, unsigned int buflen)
{
    CONFIG_ASSERT(buf != NULL);

    /** @todo IMLEMENT */
#warning IMPLEMENT IMU_init_i2c IN TERMS OF THE I2C API DRIVER FUNCTIONS

    return 0;
}


void IMU_init(void)
{
#if defined(BNO055)

    IMU_init_i2c();

    bno055.bus_read   = BNO055_I2C_bus_read;
    bno055.bus_write  = BNO055_I2C_bus_write;
    bno055.delay_msec = BNO055_delay_msek;
    bno055.bus_read   = bno055_init(&bno055);

#elif defined(BMX160)

    IMU_init_i2c();

    bno055.bus_read   = BMX160_I2C_bus_read;
    bno055.bus_write  = BMX160_I2C_bus_write;
    bno055.delay_msec = BMX160_delay_msek;
    bno055.bus_read   = bmx160_init(&bmx160);

#else

    printf("Called %s\n", __func__);

#endif /* #if defined(BNO055) */
}


static void IMU_init_i2c(void)
{
#if defined(BNO055)

    /* We use I2C1 (using UCB1) for BNO055 IMU in Rev A ONLY */
    I2C1_init();

#endif /* #if defined(BNO055) */

#if defined(BMX160)

    /*
     * IN REV B We use I2C0 (using UCB0) with replacement BMX160 IMU
     * due to chip shortage
     */
    I2C0_init();

#endif /* #if defined(BMX160) */
}


#if defined(BNO055)


static s8 BNO055_I2C_bus_read(u8 dev_addr, u8 reg_addr, u8 *reg_data, u8 cnt)
{

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
}


s8 BNO055_I2C_bus_write(u8 dev_addr, u8 reg_addr, u8 *reg_data, u8 cnt)
{
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

    /* SEE SECTION 4.6 OF DATASHEET */
#if 0
    u8  array[I2C_BUFFER_LEN];
    u8  stringpos = BNO055_INIT_VALUE;

    array[BNO055_INIT_VALUE] = reg_addr;
    for (stringpos = BNO055_INIT_VALUE; stringpos < cnt; stringpos++)
    {
        array[stringpos + BNO055_I2C_BUS_WRITE_ARRAY_INDEX] =
            *(reg_data + stringpos);
    }
#endif
    return (s8)BNO055_iERROR;
}


static void BNO055_delay_msek(u32 msek)
{
/** @todo */
#warning THIS NEEDS TO BE IMLPEMENTED
    /*Here you can write your own delay routine*/
}


#endif /* #if defined(BNO055) */

#if defined(BMX160)


static s8 BMX160_I2C_bus_read(u8 dev_addr, u8 reg_addr, u8 *reg_data, u8 cnt)
{

    s32 BMX160_iERROR = BMI160_OK;

    // copied from studying example at:
    // https://github.com/DFRobot/DFRobot_BMX160/blob/master/DFRobot_BMX160.cpp
    uint8_t outGoingBytes[] = {BMX160_MAG_DATA_ADDR, 0};
    I2C0_write_bytes(dev_addr, outGoingBytes, sizeof(outGoingBytes));
    I2C0_read_bytes(reg_data, cnt);

    return 0;

}

s8 BMX160_I2C_bus_write(u8 dev_addr, u8 reg_addr, u8 *reg_data, u8 cnt)
{
    s32 BMX160_iERROR = BMX160_INIT_VALUE;


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
        BMX160_iERROR = BMX160_SUCCESS;
    }

    /* SEE SECTION 4.6 OF DATASHEET */
#if 0
    u8  array[I2C_BUFFER_LEN];
    u8  stringpos = BMX160_INIT_VALUE;

    array[BMX160_INIT_VALUE] = reg_addr;
    for (stringpos = BMX160_INIT_VALUE; stringpos < cnt; stringpos++)
    {
        array[stringpos + BMX160_I2C_BUS_WRITE_ARRAY_INDEX] =
            *(reg_data + stringpos);
    }
#endif
    return (s8)BMX160_iERROR;
}


static void BMX160_delay_msek(u32 msek)
{
/** @todo */
#warning THIS NEEDS TO BE IMLPEMENTED
    /*Here you can write your own delay routine*/
}


#endif /* #if defined(BMX160) */
