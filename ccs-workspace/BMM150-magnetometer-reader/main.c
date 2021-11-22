/**
 * @file main.c
 *
 * @purpose IMU logger to be read by computer through USB port
 *
 * @author thomas christison (christisonthomas@gmail.com)
 * @modified by jasper grant (jasper.grant@dal.ca)
 * @brief
 * @version 0.1
 * @date 2021-07-19
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

#include "imu.h"
#include "indicator_led.h"
#include <stdio.h>
#include <stdint.h>
#include <string.h>
#include <uart.h>

#define UART_BUFLEN 200u

#define UART_RX_MSG_DELIM '!'

volatile uint8_t  uart_rxbuf[UART_BUFLEN];
static volatile int uart_rx_delim_received;
static uint8_t uart_txbuf[UART_BUFLEN];
volatile uint8_t *uart_rx_inptr;
volatile uint8_t *uart_rx_outptr;

uint8_t caller_rxbuf[200];

#define uart_printf(fmt, ...)                                                  \
    do                                                                         \
    {                                                                          \
        uint16_t cnt;                                                          \
        cnt = snprintf(uart_txbuf, sizeof(uart_txbuf), (fmt), ##__VA_ARGS__);  \
        if (cnt > sizeof(uart_txbuf))                                          \
        {                                                                      \
            cnt = sizeof(uart_txbuf);                                          \
        }                                                                      \
        uart_transmit(uart_txbuf, cnt);                                        \
    } while (0)


int main()



{
    int8_t rslt;

    imu_sensor_data_t magno_readings = {0};


    WDTCTL = WDTPW + WDTHOLD;

    led_init();

    rslt = IMU_init();

    uart_init();
    bmi160_aux_init(&imu_dev); //This function needs proper arguments
        __bis_SR_register(GIE);
    while (rslt == 0 )
        {
            rslt = IMU_get_magno(&magno_readings);


        //uart_printf("%d\n", magnetometer);


    }

    return rslt;
}
