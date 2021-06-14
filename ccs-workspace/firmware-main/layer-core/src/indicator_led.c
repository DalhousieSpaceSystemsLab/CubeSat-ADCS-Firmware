
#include <msp430.h>

/*
 * @brief turns on indicator led to indicate MSP430 active
 */
void led_init(void)
{
#if defined(MSP_LAUNCH_PAD)

    P1DIR |= BIT0;  /* MSP430F5529 Launch Pad Indicator LED (P1.0)*/
    P1OUT |= BIT0;

#else /* Using ADCS Motherboard as target*/

    P6DIR |= BIT0;  /* Motherboard Indicator LED (P6.0)*/
    P6OUT |= BIT0;

#endif /* #if defined(MSP_LAUNCH_PAD) */
}
