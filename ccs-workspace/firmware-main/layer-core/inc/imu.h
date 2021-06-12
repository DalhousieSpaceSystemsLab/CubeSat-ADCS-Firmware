#ifndef __IMU_H__
#define __IMU_H__
#ifdef __cplusplus
/* clang-format off */
extern "C"
{
/* clang-format on */
#endif /* Start C linkage */

#include "bmi160.h"
#include "bno055.h"

/****************************** Define Macros ********************************/

#define I2C_BUFFER_LEN 8                            /* @todo shouldn't this go in i2c.h?? */
#define I2C0 5                                      /* @todo shouldn't this go in i2c.h?? */
#define BNO055_I2C_BUS_WRITE_ARRAY_INDEX ((uint8_t)1)    /* @todo is this needed? */
#define BMX160_I2C_BUS_WRITE_ARRAY_INDEX ((uint8_t)1)    /* @todo is this needed? */

/* Select IMU */

//#define BNO055    /* This IMU is no longer used due to chip shortage */
#define BMX160      /* This IMU IS being used in Rev B */

#define BMX160_I2C_ADDR BMI160_I2C_ADDR
#define BMX160_I2C_INTF BMI160_I2C_INTF
#define BMX160_OK       BMI160_OK

/************************** Type Definitions *********************************/

#if defined(BNO055)
typedef struct bno055_t imu_dev_t;
#elif defined(BMX160)
typedef struct bmi160_dev imu_dev_t;
typedef struct bmi160_sensor_data imu_sensor_data_t;
#endif

/************** Global Function Declarations *********************************/

int IMU_measurements_to_string(char *buf, unsigned int buflen);
void IMU_init(void);

/* Functions linked to device level structs for use in vendor drivers */
int8_t IMU_I2C_bus_read(uint8_t dev_addr, uint8_t reg_addr, uint8_t *reg_data, uint8_t cnt);
int8_t IMU_I2C_bus_write(uint8_t dev_addr, uint8_t reg_addr, uint8_t *reg_data, uint8_t cnt);
void delay_ms(uint32_t ms);         /* @todo Makes sense to move this function to separate core layer file since it may be useful elsewhere too*/

int8_t IMU_get_gyro(imu_sensor_data_t *gyro_data);
#ifdef __cplusplus
/* clang-format off */
}
/* clang-format on */
#endif /* End C linkage */
#endif /* __IMU_H__ */
