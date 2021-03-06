#include <stdint.h>
#include <stdlib.h>
#include <string.h>
#define DEBUG 1 //new add
#if defined(TARGET_MCU)
#include "watchdog.h"
#include "mcu.h"
#include "timer_a.h"
#include "magnetorquers.h"
#include "magnetometer.h"
#include "reaction_wheels.h"
#include "imu.h"
#include "indicator_led.h"
#else
#include <errno.h>
#endif /* #if defined(TARGET_MCU) */

#include "obc_interface.h"
#include "jsons.h"
#include <uart.h>

#define UART_RX_MSG_DELIM '!'
#define UART_BUFLEN 200u

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




static void pulldown_unused_floating_pins(void);


static uint8_t msg[128];

int main(void)
{

#if defined(TARGET_MCU)
#if defined(DEBUG)
    watchdog_stop();
#else
    watchdog_start();
#endif /* #if defined(DEBUG) */

    OBC_IF_config(OBC_IF_PHY_CFG_UART);
    //IMU_init();
    //MAGTOM_init();
    //RW_init();
    //MQTR_init();
    pulldown_unused_floating_pins();
    enable_interrupts();

#else
    OBC_IF_config(OBC_IF_PHY_CFG_EMULATED);
#endif /* #if defined(TARGET_MCU) */


    while (1)
    {
        puts("loop start\n");
        if (OBC_IF_dataRxFlag_read() == OBC_IF_DATA_RX_FLAG_SET)
        {
            puts("T1\n");
            /* get command json string from OBC interface */
            OCB_IF_get_command_string(msg, sizeof(msg));

            /* Parse command json string */
            JSON_PARSE_t status = json_parse(msg);
            switch (status)
            {
                case JSON_PARSE_format_err:
                {
                    OBC_IF_printf(
                        "{\"error\" : \"json format\",    \"received\":\"%s\"}",
                        msg);
                }
                break;
                case JSON_PARSE_unsupported:
                {
                    OBC_IF_printf("{\"error\" : \"json unsupported\",    "
                                  "\"received\":\"%s\"}",
                                  msg);
                }
                break;
                case JSON_PARSE_ok:
                {
                    /* Do nothing */
                }
                break;
                default:
                {
                }
                break;
            }
            OBC_IF_dataRxFlag_write(OBC_IF_DATA_RX_FLAG_CLR);
        }
        puts("loop close1\n");
#if defined(TARGET_MCU) && !defined(DEBUG)
        watchdog_kick();
#endif /* #if defined(TARGET_MCU) && !defined(DEBUG)*/
    }
}


static void pulldown_unused_floating_pins(void)
{
#warning IMPLEMENT THIS
    /** @todo ON FINAL BOARD MAKE SURE ALL FLOATING PINS ARE PULLED DOWN
     * INTERNALLY TO PREVENT CHARGE BUILDUP IN ORBIT */
}
