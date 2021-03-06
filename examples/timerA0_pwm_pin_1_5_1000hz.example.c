/**
 * @file timerA_pwm.example.c
 * @author Carl Mattatall (cmattatall2@gmail.com)
 * @brief Source module example to demo generation of pwm signal using timerA
 * @version 0.1
 * @date 2021-03-01
 *
 * @copyright Copyright (c) 2021 Carl Mattatall
 *
 *  1000HZ!!!!!
 * PWM SIGNAL GENERATED ON PIN1.5 (which is output pin for TA0CCR4 COMPARE EVT)
 */
#include <stdlib.h>
#include <msp430.h>
#include <stdint.h>

#define TA0IV_IF1 (0X02)
#define TA0IV_IF2 (0X04)
#define TA0IV_IF3 (0X06)
#define TA0IV_IF4 (0X08)
#define TA0IV_IF5 (0X0A)
#define TA0IV_IF6 (0X0C)
#define TA0IV_OVF (0X0E)

/* clang-format off */
#define OUTMOD_MSK     (OUTMOD0 | OUTMOD1 | OUTMOD2)
#define OUTMOD_OUTBIT  (OUTMOD_0)
#define OUTMOD_SET     (OUTMOD_1)
#define OUTMOD_TOG_RST (OUTMOD_2)
#define OUTMOD_SET_RST (OUTMOD_3)
#define OUTMOD_TOGGLE  (OUTMOD_4)
#define OUTMOD_RESET   (OUTMOD_5)
#define OUTMOD_TOG_SET (OUTMOD_6)
#define OUTMOD_RST_SET (OUTMOD_7)
/* clang-format on */


static uint16_t TIMERA0_duty_cycle(unsigned int percent)
{
    return (uint16_t)(TA0CCR0 * ((float)((percent % 100u) / 100.0f)));
}


static void TIMERA0_pin_1_5_pwm_phy_init(void)
{
    P1DIR ^= BIT5; /* P1.5 in output direction */
    P1SEL |= BIT5; /* P1.5 will be used for its peripheral function */
}

int main(void)
{
    /* Stop WDT */
    WDTCTL = WDTPW + WDTHOLD;

    TIMERA0_pin_1_5_pwm_phy_init();

    TA0CTL &= ~(MC0 | MC1); /* Stop timer */

    TA0CCR0 = 1000;
    
    TA0CCR4 = TIMERA0_duty_cycle(25);
    TA0CCTL4 |= OUTMOD_TOG_SET;

    /* Set Timer A0 clock source to smclk */
    TA0CTL &= ~(TASSEL0 | TASSEL1);
    TA0CTL |= TASSEL__SMCLK;

    /* Set count mode */
    TA0CTL &= ~(MC0 | MC1);
    TA0CTL |= MC__UP;

    /** @note VERY IMPORTANT. IF YOU DO _BIS_SR(GIE) it will not work! */
    _BIS_SR(GIE + LPM0_bits);

    while (1)
    {
    }
}
