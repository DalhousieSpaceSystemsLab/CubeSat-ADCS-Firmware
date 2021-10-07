/**
 * @file main.c
 *
 * @purpose magnetometer logger to be read by computer through UART-> USB port
 *
 * @author Zhang Yiming (Yimimg.Zhang@dal.ca)
 * @brief
 * @version 0.2
 * @date 2021-08-25
 *
 *
 * Magnetometer Datasheet
 * https://aerospace.honeywell.com/content/dam/aerobt/en/documents/learn/products/sensors/datasheet/HMC_1051-1052-1053_Data_Sheet.pdf
 *
 * HMC 1053 is a 3 Axis Magnetic Sensors, it will be used to measure the 3 axis magnetic field value around LORIS satellite in space
 *
 */

#include <msp430.h>

#include "magnetometer.h"
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
    //int8_t rslt = 0;
    MAGTOM_measurement_t magtom_readings = {0}; //has error

    WDTCTL = WDTPW + WDTHOLD; /*watchdog*/

    MAGTOM_init(); /* Magnetometer Initilization*/

    uart_init();
    __bis_SR_register(GIE); /* ??? */
    led_init(); /* Turn on indicator LED */

    FILE *out;
    out = fopen ("magtomout.txt","w");
    puts("Hello, world!\n");
    uart_printf("Hello, world!\n");
    int i = 0;
    for (i = 0; i < 1000; i++)
    {
        magtom_readings = MAGTOM_get_measurement();
        //uart_printf("%f %f %f\n", magtom_readings.x_BMAG, magtom_readings.y_BMAG, magtom_readings.z_BMAG);
        //printf("Z:%f Y:%f X:%f\n", magtom_readings.x_BMAG, magtom_readings.y_BMAG, magtom_readings.z_BMAG);
        //fprintf(out,"%f %f %f\n\n", magtom_readings.x_BMAG, magtom_readings.y_BMAG, magtom_readings.z_BMAG);

    }
    fclose(out);
    return 0;
}
