/**
 * @file i2c0.c
 * @author Carl Mattatall (cmattatall2@gmail.com), Thomas Christison (christisonthomas@gmail.com)
 * @brief Source module to implement an i2c driver using UCB0 for msp430f5529
 * @brief
 * @version 0.1
 * @date 2021-03-25
 *
 * @copyright Copyright (c) 2021 Carl Mattatall, Thomas Christison
 *
 * Section 38: https://www.ti.com/lit/ug/slau208q/slau208q.pdf?ts=1623263272335&ref_url=https%253A%252F%252Fwww.ti.com%252Fproduct%252FMSP430BT5190
 *
 */

#if !defined(TARGET_MCU)
#error DRIVER COMPILATION SHOULD ONLY OCCUR ON CROSSCOMPILED TARGETS
#endif /* !defined(TARGET_MCU) */

#include <string.h>
#include <stdlib.h>

#include <msp430.h>

#include "i2c.h"

#define USCI_B_I2C_SET_DATA_RATE_100KBPS                                 100000

static volatile uint8_t  I2C0_i2c_txbuf[32];
static volatile uint8_t *I2C0_i2c_txbuf_ptr;
static volatile uint16_t I2C0_i2c_txcnt;

static volatile uint8_t  I2C0_i2c_rxbuf[32];
static volatile uint8_t *I2C0_i2c_rxbuf_inptr;
static volatile uint8_t *I2C0_i2c_rxbuf_outptr;

static void I2C0_PHY_init(void);

/*
 * @brief initialize I2C master on USCIB0
 *
 * The recommended USCI initialization/reconfiguration process is:
 * 1.  Set UCSWRST(BIS.B#UCSWRST,&UCxCTL1).
 * 2.  Initialize all USCI registers with UCSWRST= 1.
 * 3.  Configureports.
 * 4.  Clear UCSWRST through software (BIC.B #UCSWRST,&UCxCTL1).
 * 5.  Enable interrupts (optional).
 */
void I2C0_init(void)
{
    uint16_t preScalarValue;

    /*
     * set peripheral function - multiplex gpio pins to i2c mode
     * P3.0 - UCB1 SDA
     * P3.1 - UCB0 SCL
     */
    I2C0_PHY_init();

    /* Disable the USCI module and clears the other bits of control register */
    UCB0CTL1 |= UCSWRST;

    /*
     * Configure as I2C master mode.
     * UCMST = Master mode
     * UCMODE_3 = I2C mode
     * UCSYNC = Synchronous mode
     */
    UCB0CTL0 = UCMST + UCMODE_3 + UCSYNC;

    /* Initialize I2C clock source */
    UCB0CTL1 = UCSSEL__SMCLK + USCWRST; /* sub-main clock*/

    /* Set data rate.
     * Compute the clock divider that achieves the fastest speed less than or
     * equal to the desired speed.  The numerator is biased to favor a larger
     * clock divider so that the resulting clock is always less than or equal
     * to the desired clock, never greater.
     */

/*#warning NEED Current SMCLK frequency in Hz - Not implemented */
    //preScalarValue = (unsigned short)(CURR_SMCLK_HZ / USCI_B_I2C_SET_DATA_RATE_100KBPS);
    //UCB0BRW = preScalarValue; /* Bit rate control word */
    UCB0BR0 = 10;           // fSCL = SMCLK/160 = ~100kHz
    UCB0BR1 = 0;

    /* Init master */
    UCB0CTL1 &= ~UCSWRST;   // Clear SW reset, resume operation
    UCB0IE |= UCNACKIE;     /* Enable Interrupts */

}

/*
 * After initialization master transmitter mode is initiated by:
 * 1. writing the desired slave address to the UCB0I2CSA register
 * 2. selecting the size of the slave address with the UCSLA10 bit
 * 3. Setting UCTR for trasmitter mode
 * 4. Setting UCTXSTT to generate a START condition
 *
 * The USCI module checks if the bus is available, generates the START condition, and
 * transmits the slave address. The UCTXIFG bit is set when the START condition is generated and
 * the first data to be transmitted can be written into UCB0TXBUF. As soon as the slave
 * acknowledges the address, the UCTXSTT bit is cleared.
 */
int I2C0_write_bytes(uint8_t dev_addr, uint8_t *bytes, uint16_t byte_count)
{
    int retval = 0;

    /* Set slave address*/
    UCB0I2CSA = dev_addr;

    /* Set UCSLA10 bit for 7-bit slave address*/
    UCB0CTL0 |= UCSLA10;

    /* Set UCTR for transmitter mode */
    UCB0CTL1 |= UCTR;

    /* Generate START condition */
    UCB0CTL1 |= UCTXSTT;


#if 0
    if (bytes != NULL)
    {
        I2C0_i2c_txbuf_ptr = I2C0_i2c_txbuf; // reset buffer first

        /* First byte in transmit buffer is device address */
        I2C0_i2c_txbuf_ptr = I2C0_i2c_txbuf;
        *I2C0_i2c_txbuf_ptr = dev_addr;
        I2C0_i2c_txbuf_ptr++;
        /* Rest of data comes next */
        uint16_t bcnt = sizeof(dev_addr) + byte_count;
        if (bcnt > sizeof(I2C0_i2c_txbuf))
        {
            retval = -1;
        }
        else
        {
            strncpy((char *)I2C0_i2c_txbuf_ptr, (char *)bytes, bcnt);
            I2C0_i2c_txcnt = bcnt;
            /** @todo IMPLEMENT THE STUFF THAT ACTUALLY TRANSMITS, CALLER DATA
             * IS JUST LOADED RIGHT NOW*/
            UCB0TXBUF = *I2C0_i2c_txbuf;
            while (!UCB0CTL1 |= UCTXSTP)
                ; // wait for stop condition
        }
    }
    else
    {
        retval = 0;
    }


#endif /* #if 0 */

    return retval;
}

/*
 * After initialization master receiver mode is initiated by:
 * 1. writing the desired slave address to the UCB0I2CSA register
 * 2. selecting the size of the slave address with the UCSLA10 bit
 * 3. Clearing UCTR for receiver mode
 * 4. Setting UCTXSTT to generate a START condition
 *
 * The USCI module checks if the bus is available, generates the START condition, and
 * transmits the slave address. As soon as the slave acknowledges the address, the UCTXSTT
 * bit is cleared.
 *
 *
 */
int I2C0_read_bytes(uint8_t dev_addr, uint8_t *caller_buf, uint16_t caller_buflen)
{
    if (caller_buf == NULL)
    {
        return 0;
    }
    else
    {
        unsigned int bcnt = 0;
        while (I2C0_i2c_rxbuf_outptr != I2C0_i2c_rxbuf_inptr)
        {
            caller_buf[bcnt] = *I2C0_i2c_rxbuf_outptr;
            I2C0_i2c_rxbuf_outptr++;
            if (I2C0_i2c_rxbuf_outptr > I2C0_i2c_rxbuf + sizeof(I2C0_i2c_rxbuf))
            {
                I2C0_i2c_rxbuf_outptr = I2C0_i2c_rxbuf_outptr;
            }
            ++bcnt;

            /* Data overrun condition */
            if (bcnt > sizeof(I2C0_i2c_rxbuf))
            {
                bcnt = -1;
                break;
            }

            if(bcnt == caller_buflen)
            {
                break;
            }
        }
        return bcnt;
    }
}


static void I2C0_PHY_init(void)
{
    P3SEL |= BIT0; /* p3.0 == SDA */
    P3SEL |= BIT1; /* p3.1 == SCL */
}


__interrupt_vec(USCI_B0_VECTOR) void USCI_I2C_ISR(void)
{
    switch (__even_in_range(UCB0IV, 12))
    {
        case 0:
            break; // Vector  0: No interrupts
        case 2:
            break; // Vector  2: ALIFG
        case 4:
            break; // Vector  4: NACKIFG
        case 6:
            break; // Vector  6: STTIFG
        case 8:
            break; // Vector  8: STPIFG
        case 10:
        {
            /** @todo IMPLEMENT THIS STUFF */
#warning RECEIVE ISR HANDLING NOT IMPLEMENTED FOR I2C0
        }
        break;   // Vector 10: RXIFG
        case 12: // Vector 12: TXIFG
        {
            if (I2C0_i2c_txcnt)
            {
                UCB0TXBUF = *I2C0_i2c_txbuf_ptr;
                I2C0_i2c_txbuf_ptr++; /* advance data ptr */
                I2C0_i2c_txcnt--;     // Decrement TX byte counter
            }
            else
            {
                UCB0CTL1 |= UCTXSTP; // I2C stop condition
                UCB0IFG &= ~UCTXIFG; // Clear USCI_B0 TX int flag
                __no_operation();    /* NOP to fix silicon errata with ISR */
            }
        }
        break;
        default:
            break;
    }
}
