#ifndef __UART_H__
#define __UART_H__
#ifdef __cplusplus
/* clang-format off */
extern "C"
{
/* clang-format on */
#endif /* Start C linkage */

#if defined(TARGET_MCU)

#include "injection_api.h"

/**
 * @brief Initialize the UART with interface UCA0
 * @param rx the receive function to execute upon execution of the UART rx ISR
 */
void uart_init(void);
void uart_deinit(void);
int uart_transmit(uint8_t *msg, uint_least16_t msglen);
int uart_receive_bytes(uint8_t *caller_buf, uint16_t caller_buflen);

#else

#error USE OF MEMORY MAPPED PERIPHERALS ON A VIRTUAL MEMORY NATIVE MACHINE IS NOT PERMITTED

#endif /* #if defined(TARGET_MCU) */

#ifdef __cplusplus
/* clang-format off */
}
/* clang-format on */
#endif /* End C linkage */
#endif /* __UART_H__ */
