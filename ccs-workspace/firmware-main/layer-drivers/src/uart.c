/**
 * @file uart.c
 * @author Carl Mattatall (cmattatall2@gmail.com)
 * @brief UART peripheral file
 * @version 0.1
 * @date 2020-12-09
 *
 * @copyright Copyright (c) 2020
 *
 */

#if !defined(TARGET_MCU)
#error DRIVER COMPILATION SHOULD ONLY OCCUR ON CROSSCOMPILED TARGETS
#endif /* !defined(TARGET_MCU) */

#include <stdbool.h>
#include <stdlib.h>
#include <stdint.h>
#include <string.h> /* memcpy */

#include "attributes.h"

#include "targets.h"

#include <msp430.h>

#include "uart.h"

static receive_func uart_rx_cb;


#define UART_BUFLEN 200u

#define UART_RX_MSG_DELIM '!'

extern volatile uint8_t  uart_rxbuf[UART_BUFLEN];
extern volatile uint8_t *uart_rx_inptr;
extern volatile uint8_t *uart_rx_outptr;

extern volatile int uart_rx_delim_received;

void uart_init(void)
{
    uart_rx_inptr  = uart_rxbuf;
    uart_rx_outptr = uart_rxbuf;

    P3SEL |= BIT3; /* P3.3 = UCA0TX */
    P3SEL |= BIT4; /* P3.4 = UCA0RX */
    UCA0CTL1 |= UCSWRST; /* **Put state machine in reset** */

    UCA0CTL1 |= UCSSEL__SMCLK; /* SMCLK */

    UCA0BR0 = 104; /* 9600 baud 8N1 See table 36-5 */
    UCA0BR1 = 0;

    UCA0MCTL &= ~UCOS16;
    UCA0MCTL &= ~(UCBRS0 | UCBRS1 | UCBRS2);
    UCA0MCTL |= UCBRS_1;
    UCA0MCTL &= ~(UCBRF0 | UCBRF1 | UCBRF2 | UCBRF3);
    UCA0MCTL |= UCBRF0;

    /* over sampling */
    UCA0CTL1 &= ~UCSWRST; /* Initialize USCI state machine */

    UCA0IE |= UCRXIE; /* Enable USCI_A0 RX interrupt */
}

void uart_deinit(void)
{
    /** @todo
     *  - RESET REGISTERS TO DEFAULT VAULES
     *  - RESET CONFIGURATION OF USCI_A0 to defaults
     *  - DISABLE CLOCK MUX to peripheral (optional - improves power savings)
     */

    if (uart_rx_cb != NULL)
    {
        uart_rx_cb = NULL;
    }

    /* Disable interrupts */
    UCA0IE = 0;

    /* Clear any pending interrupt flags */
    UCA0IFG = 0;

    log_trace("deinitialized uart\n");
}


int uart_transmit(uint8_t *msg, uint_least16_t msglen)
{
    if (!(UCA0IE & UCTXIE) && !(UCA0STAT & UCBUSY))
        {
            unsigned int i = 0;
            do
            {
if (UCA0IFG & UCTXIFG)
                {
                    UCA0TXBUF = msg[i++];
                }
            } while (i < msglen);
        }
        return 0;
}

int uart_receive_bytes(uint8_t *caller_buf, uint16_t caller_buflen)
{
    int retval      = 0;
    int i           = 0;
    int delim_found = 0;
    do
    {
        caller_buf[i] = *uart_rx_outptr;
        if (caller_buf[i] == '\0')
        {
            delim_found = 1;
        }

        uart_rx_outptr++;
        if (uart_rx_outptr > uart_rxbuf + sizeof(uart_rxbuf))
        {
            uart_rx_outptr = uart_rxbuf;
        }
    } while (++i < caller_buflen && !delim_found);

    if (!delim_found)
    {
        retval = -1;
    }
    else
    {
        retval = i;
    }

    return retval;
}

//******************************************************************************
//***********************UART Interrupt Service Routine************************
//******************************************************************************
#if defined(__TI_COMPILER_VERSION__) || defined(__IAR_SYSTEMS_ICC__)
#pragma vector=USCI_A0_VECTOR
__interrupt void USCI_A0_ISR(void)
#elif defined(__GNUC__)
void __attribute__ ((interrupt(USCI_A0_VECTOR))) USCI_A0_ISR (void)
#else
#error Compiler not supported!
#endif
{
    /* See table 39-19 */
    switch (UCA0IV)
    {
        case 0x02: /* Receive buffer full */
        {
            *uart_rx_inptr = UCA0RXBUF;

            if (*uart_rx_inptr == UART_RX_MSG_DELIM)
            {
                *uart_rx_inptr         = '\0';
                uart_rx_delim_received = 1;
                printf("%d\n", uart_rx_delim_received);
            }

            if (++uart_rx_inptr > uart_rxbuf + sizeof(uart_rxbuf))
            {
                uart_rx_inptr = uart_rxbuf;
            }
        }
        break;
        case 0x06: /* Start bit received */
        {
        }
        break;
        case 0x08: /* Transmit complete (last bit was sent to PHY)
                    */
        {
        }
        break;
        default: /* not sure what happened here */
        {
        }
        break;
    }
}
