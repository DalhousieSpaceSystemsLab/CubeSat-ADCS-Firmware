/**
 * @file main.c
 *
 * @purpose IMU logger to be read by computer through USB port
 *
 * @author thomas christison (christisonthomas@gmail.com)
 * @modified by jasper grant (jasper.grant@dal.ca)
 * @modified by tanaka akiyama (tanaka.akiyama@dal.ca)
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
/*
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


typedef enum
{
    MQTR_x,
    MQTR_y,
    MQTR_z,
} MQTR_t;

int main()



{
    int8_t rslt;


    WDTCTL = WDTPW + WDTHOLD; //Disable the Watchdog timer

    led_init();

    rslt = IMU_init();

    MQTR_init();
    MQTR_set_coil_voltage_mv(MQTR_x, 3000);
    MQTR_set_coil_voltage_mv(MQTR_y, 3000);
    MQTR_set_coil_voltage_mv(MQTR_z, 3000);

    uart_init();
        __bis_SR_register(GIE);
        while (rslt == 0 )
        {

            uart_printf("%d %d %d\n", MQTR_get_coil_voltage_mv(MQTR_x), MQTR_get_coil_voltage_mv(MQTR_y),MQTR_get_coil_voltage_mv(MQTR_z));

        }

    return rslt;
}
*/



#include <msp430F5529.h>
#include <stdint.h>
#define PWM_PERIOD (1000.0f)
#define MAX_VOLTAGE_MV (3300.0f)

#define MQTR_X_F_REG TA0CCR1 // pin 1.2 (TA0.1)
#define MQTR_X_R_REG TA0CCR2 // pin 1.3 (TA0.2)
#define MQTR_Y_F_REG TA1CCR2 // pin 2.1 (TA1.2)
#define MQTR_Y_R_REG TA1CCR1 // pin 2.0 (TA1.1)
#define MQTR_Z_F_REG TA0CCR3 // pin 1.4 (TA0.3)
#define MQTR_Z_R_REG TA0CCR4 // pin 1.5 (TA0.4)

typedef enum
{
    MQTR_x,
    MQTR_y,
    MQTR_z,
} MQTR_t;

static void MQTR_set_PWM_pins();
static void MQTR_init_PWM_timers ();
static void MQTR_set_coil_voltage_mv(MQTR_t mqtr, int16_t voltage_mv);


int main(void)
{

    WDTCTL = WDTPW + WDTHOLD; // Stop watchdog timer

    /* CONFIGURE_PINS */
    MQTR_set_PWM_pins();

    /* CONFIGURE_TIMERS */
    MQTR_init_PWM_timers();

    MQTR_set_coil_voltage_mv(MQTR_y, 500);
    MQTR_set_coil_voltage_mv(MQTR_z, -500);
    MQTR_set_coil_voltage_mv(MQTR_x, 1000);


    _BIS_SR(LPM0_bits); // GO to low power mode

}

static void MQTR_set_PWM_pins (){
    /* Configure X coil F pwm pin */
    P1DIR |= BIT2; /* P1.2 in output direction */
    P1SEL |= BIT2; /* P1.2 will be used for its peripheral function */

    /* Configure X coil R pwm pin */
    P1DIR |= BIT3; /* P1.3 in output direction */
    P1SEL |= BIT3; /* P1.3 will be used for its peripheral function */

    /* Configure Y coil R pwm pin */
    P2DIR |= BIT0; /* P2.0 in output direction */
    P2SEL |= BIT0; /* P2.0 will be used for its peripheral function */

    /* Configure Y coil R pwm pin */
    P2DIR |= BIT1; /* P2.1 in output direction */
    P2SEL |= BIT1; /* P2.1 will be used for its peripheral function */

    /* Configure Z coil F pwm pin */
    P1DIR |= BIT4; /* P1.4 in output direction */
    P1SEL |= BIT4; /* P1.4 will be used for its peripheral function */

    /* Configure Z coil R pwm pin */
    P1DIR |= BIT5; /* P1.5 in output direction */
    P1SEL |= BIT5; /* P1.5 will be used for its peripheral function */
}

static void MQTR_init_PWM_timers (){

    /* Init TimerA0 */
    TA0CCR0 = PWM_PERIOD; // Set the period in the Timer A0 Capture/Compare 0 register
    TA0CTL = TASSEL_2 + MC_1; // TASSEL_2 selects SMLCK as clock source, and MC_1 tells it to count up to value in TA0CCR0.

    /* Init Timer A1 */
    TA1CCR0 = PWM_PERIOD; // Set the period in the Timer A1 Capture/Compare 0 register
    TA1CTL = TASSEL_2 + MC_1; // TASSEL_2 selects SMLCK as clock source, and MC_1 tells it to count up to value in TA0CCR1.

    // Pin 1.2 (TA0.1) , X coil F pin
    TA0CCTL1 = OUTMOD_7; // CCR1 reset/set
    MQTR_X_F_REG = 0; // CCR1 PWM duty cycle

    // Pin 1.3 (TA0.2), X coil R pin
    TA0CCTL2 = OUTMOD_7;
    MQTR_X_R_REG = 0;

    // Pin 2.0 (TA1.1), Y coil R pin
    TA1CCTL1 = OUTMOD_7;
    MQTR_Y_R_REG = 0;

    // Pin 2.1 (TA1.2)
    TA1CCTL2 = OUTMOD_7;
    MQTR_Y_F_REG = 0;

    //Pin 1.4 (TA0.3)
    TA0CCTL3 = OUTMOD_7;
    MQTR_Z_F_REG = 0;

    //Pin1.5 (TA0.4)
    TA0CCTL4 = OUTMOD_7;
    MQTR_Z_R_REG4 = 0;
}

static void MQTR_set_coil_voltage_mv(MQTR_t mqtr, int16_t voltage_mv){

    float duty_cycle_val = voltage_mv / MAX_VOLTAGE_MV * PWM_PERIOD;
    int reverse = 0;

    if (voltage_mv < 0){
        reverse = 1;
    } else {
        reverse = 0;
    }

    switch (mqtr)
    {
        case MQTR_x:
        {
            if (reverse) {
                MQTR_X_R_REG = (uint16_t)duty_cycle_val;
            } else {
                MQTR_X_F_REG = (uint16_t)duty_cycle_val;
            }
        }
        break;

        case MQTR_y:
        {
            if (reverse) {
                MQTR_Y_R_REG = (uint16_t)duty_cycle_val;
            } else {
                MQTR_Y_F_REG = (uint16_t)duty_cycle_val;
            }
        }
        break;

        case MQTR_z:
        {
            if (reverse) {
                MQTR_Z_R_REG = (uint16_t)duty_cycle_val;
            } else {
                MQTR_Z_F_REG = (uint16_t)duty_cycle_val;
            }
        }
        break;
    }
}
