/**
 * @file uart_interface.c
 * @author Carl Mattatall (cmattatall2@gmail.com)
 * @brief Abstraction layer between PHY peripheral interface and
 * application level protocol
 * @version 0.1
 * @date 2020-12-09
 *
 * @copyright Copyright (c) 2020 DSS - LORIS project
 *
 */
#include <stdio.h> /* snprintf */
#include <string.h>
#include <stdint.h>
#include <limits.h>
#include <stdbool.h>
#include <stdarg.h>
#include "targets.h"
#include "attributes.h"
#include "obc_interface.h"
#include "bufferlib.h"
#include "injection_api.h"



#if !defined(TARGET_MCU)
#include "obc_emulator.h"
#include <pthread.h>
static pthread_mutex_t OBC_IF_rxflag_lock;
#else
#include "uart.h"
#include "spi.h"
#endif /* !defined(TARGET_MCU) */

#define OBC_INTERFACE_BUFFER_SIZE 500

typedef struct
{
    rx_injector_func init;
    deinit_func      deinit;
    transmit_func    tx;
} OBC_IF_fops;

__attribute__((weak)) size_t strnlen (const char *s, size_t maxlen)
{
  size_t i;
  for (i = 0; i < maxlen; ++i)
    if (s[i] == '\0')
      break;
  return i;
}

/**
 * @brief Receive callback to be injected from OBC IF into LL driver
 *
 * @param byte byte to receive from driver
 */
static void OBC_IF_receive_byte_internal(uint8_t byte);

static int OBC_IF_config_internal(rx_injector_func init, deinit_func deinit,
                                  transmit_func tx);

static OBC_IF_fops   ops           = {NULL};
static volatile bool OBC_IF_rxflag = false;
static buffer_handle obc_buf_handle;
static uint8_t       obcTxBuf[OBC_INTERFACE_BUFFER_SIZE];

int OBC_IF_config(OBC_IF_PHY_CFG_t cfg_mode)
{
    int retval = 1;
    switch (cfg_mode)
    {
        case OBC_IF_PHY_CFG_UART:
        {
#if defined(TARGET_MCU)
            retval =
                OBC_IF_config_internal(uart_init, uart_deinit, uart_transmit);
#else
            CONFIG_ASSERT(0); /* best we can do is hang */

#endif /* #if defined(TARGET_MCU) */
        }
        break;
        case OBC_IF_PHY_CFG_EMULATED:
        {
#if defined(TARGET_MCU)
            CONFIG_ASSERT(0); /* best we can do is hang */
#else
            retval = OBC_IF_config_internal(NULL, NULL, OBC_EMU_tx);
            OBC_EMU_start();
#endif /* #if defined(TARGET_MCU) */
        }
        break;
        default:
        {
        }
        break;
    }
    return retval;
}


void OBC_IF_clear_config(void)
{
    ops.init = NULL;
    ops.tx   = NULL;
    if (ops.deinit != NULL)
    {
        ops.deinit();
        ops.deinit = NULL;
    }

    obc_buf_handle.delete(obc_buf_handle.this);

#if !defined(TARGET_MCU)
    pthread_mutex_destroy(&OBC_IF_rxflag_lock);
#endif /* !defined(TARGET_MCU) */
}


#if !defined(TARGET_MCU)
void OBC_IF_receive_byte(uint8_t byte)
{
    OBC_IF_receive_byte_internal(byte);
}
#endif /* #if defined(TARGET_MCU) */

int OCB_IF_get_command_string(uint8_t *buf, uint_least16_t buflen)
{
    CONFIG_ASSERT(buf != NULL);
    int           status      = 0;
    bool          found_delim = false;
    uint_fast16_t i           = 0;
    do
    {
        int tmp = obc_buf_handle.read_next(obc_buf_handle.this);
        if (tmp != BUFFERLIB_READ_FAILURE)
        {
            buf[i] = (char)tmp;
            if (buf[i] == OBC_MSG_DELIM)
            {
                buf[i]      = '\0';
                found_delim = true;
            }
        }
        else
        {
            break;
        }
    } while (!found_delim && ++i < buflen);

    if (!found_delim)
    {
        status = 1;
    }
    return status;
}


__attribute__((weak)) int OBC_IF_tx(uint8_t *buf, uint_least16_t buflen)
{
    CONFIG_ASSERT(ops.tx != NULL);
    return ops.tx(buf, buflen);
}


bool OBC_IF_dataRxFlag_read(void)
{
    bool flag_state;

#if !defined(TARGET_MCU)
    pthread_mutex_lock(&OBC_IF_rxflag_lock);
#endif /* #if defined(TARGET_MCU) */

    flag_state = OBC_IF_rxflag;

#if !defined(TARGET_MCU)
    pthread_mutex_unlock(&OBC_IF_rxflag_lock);
#endif /* #if defined(TARGET_MCU) */

    return flag_state;
}

void OBC_IF_dataRxFlag_write(bool data_state)
{
#if !defined(TARGET_MCU)
    pthread_mutex_lock(&OBC_IF_rxflag_lock);
#endif /* #if defined(TARGET_MCU) */

    OBC_IF_rxflag = data_state;

#if !defined(TARGET_MCU)
    pthread_mutex_unlock(&OBC_IF_rxflag_lock);
#endif /* #if defined(TARGET_MCU) */
}


/** @todo THIS FUNCTION IS SO GODDAMN UGLY BUT AT LEAST ITS WORKING - Carl */
int OBC_IF_printf(const char *restrict fmt, ...)
{
    CONFIG_ASSERT(fmt != NULL);
    int     bytes_transmitted = 0;
    va_list args;
    va_start(args, fmt);

#if defined(TARGET_MCU)
    char *fmt_str = (char *)fmt;
#else
    /** @note why the fuck do I even have to add this. I shouldn't have to add
     * it. In fact, according to POSIX spec for termios I shouldn't even have
     * to WORRY about it...
     *
     * The entire reason this is here is so that the god damn newline gets
     * appended to the format string because aparently, POSIX shells can't
     * fucking detect EOL signals consistently...
     *
     * I've spend HOURS finding the problem and then fixing it.
     *
     * IF YOU TOUCH THIS PART I WILL ROLL OVER IN MY GRAVE...
     *
     * - Carl
     */
    char fmt_str[250] = {'\0'};
    CONFIG_ASSERT(strnlen(fmt, sizeof(fmt_str)) <= sizeof(fmt_str));
    strcpy(fmt_str, fmt);
    strcat(fmt_str, "\n");
#endif /* #if defined(TARGET_MCU) */

    memset(obcTxBuf, 0, sizeof(obcTxBuf));
    vsnprintf((char *)obcTxBuf, sizeof(obcTxBuf), fmt_str, args);
    size_t msg_len = strnlen((char *)obcTxBuf, sizeof(obcTxBuf));

    bytes_transmitted = OBC_IF_tx(obcTxBuf, msg_len);
    va_end(args);
    return bytes_transmitted;
}


static void OBC_IF_receive_byte_internal(uint8_t byte)
{
    obc_buf_handle.write_next(obc_buf_handle.this, byte);
    if (byte == OBC_MSG_DELIM)
    {
        OBC_IF_dataRxFlag_write(OBC_IF_DATA_RX_FLAG_SET);
    }
}


static int OBC_IF_config_internal(rx_injector_func init, deinit_func deinit,
                                  transmit_func tx)
{
    int status = 0;
    ops.init   = init;
    ops.deinit = deinit;
    ops.tx     = tx;

    obc_buf_handle = bufferlib_ringbuf_new(OBC_INTERFACE_BUFFER_SIZE);

#if !defined(TARGET_MCU)
    pthread_mutex_init(&OBC_IF_rxflag_lock, NULL);
#endif /* !defined(TARGET_MCU) */

    if (ops.init != NULL)
    {
        /* Inject LL comms protocol interface with OBC interface byte rx */
        ops.init(OBC_IF_receive_byte_internal);
    }

    if (ops.tx == NULL)
    {
        status = 1;
    }

    return status;
}
