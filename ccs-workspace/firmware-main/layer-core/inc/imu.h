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
#include "bmm150.h"

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
//typedef struct bmm150_mag_data imu_sensor_data_t2;
typedef struct bmi160_offsets imu_sensor_offsets_t;
typedef struct bmi160_pmu_status imu_power_mode_status_t;
typedef struct bmi160_foc_conf imu_fast_off_comp_t;
#endif

/************** Global Function Declarations *********************************/

int IMU_measurements_to_string(char *buf, unsigned int buflen, const imu_sensor_data_t gyro_readings);
int8_t IMU_init(void);

/* Functions linked to device level structs for use in vendor drivers */
int8_t IMU_I2C_bus_read(uint8_t dev_addr, uint8_t reg_addr, uint8_t *reg_data, uint8_t cnt);
int8_t IMU_I2C_bus_write(uint8_t dev_addr, uint8_t reg_addr, uint8_t *reg_data, uint8_t cnt);
void delay_ms(uint32_t ms);         /* @todo Makes sense to move this function to separate core layer file since it may be useful elsewhere too*/

/* IMU API*/
int8_t IMU_get_gyro(imu_sensor_data_t *gyro_data);
//int8_t IMU_get_magno(imu_sensor_data_t *magno_data);
int8_t IMU_self_test(void);
int8_t IMU_get_offsets(imu_sensor_offsets_t *offset);
int8_t IMU_set_offsets(const imu_fast_off_comp_t *foc_conf, const imu_sensor_offsets_t *offset);
int8_t IMU_get_power_mode(imu_power_mode_status_t *powermode);

/* for aux */
int8_t user_aux_read(uint8_t id, uint8_t reg_addr, uint8_t *aux_data, uint16_t len);
int8_t user_aux_write(uint8_t id, uint8_t reg_addr, uint8_t *aux_data, uint16_t len);

void IMU_reset(void);

#ifdef __cplusplus
/* clang-format off */
}
/* clang-format on */
#endif /* End C linkage */
#endif /* __IMU_H__ */
