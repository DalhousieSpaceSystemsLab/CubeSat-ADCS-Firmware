/*
 * test.h
 *
 *  Created on: 2022Äê1ÔÂ18ÈÕ
 *      Author: Zacka
 */

#ifndef LAYER_CORE_INC_TEST_H_
#define LAYER_CORE_INC_TEST_H_
#include "bmi160.h"
#include "bno055.h"
#include "bmm150.h"
typedef struct bmi160_sensor_data imu_sensor_data_t;
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

int8_t IMU_init(void);
int IMU_measurements_to_string(char *buf, unsigned int buflen, const imu_sensor_data_t gyro_readings);
/* Functions linked to device level structs for use in vendor drivers */
int8_t IMU_I2C_bus_read(uint8_t dev_addr, uint8_t reg_addr, uint8_t *reg_data, uint8_t cnt);
int8_t IMU_I2C_bus_write(uint8_t dev_addr, uint8_t reg_addr, uint8_t *reg_data, uint8_t cnt);
void delay_ms(uint32_t ms);
int8_t bmm150_aux_read(uint8_t id, uint8_t reg_addr, uint8_t *aux_data, uint16_t len);
int8_t bmm150_aux_write(uint8_t id, uint8_t reg_addr, uint8_t *aux_data, uint16_t len);


#endif /* LAYER_CORE_INC_TEST_H_ */
