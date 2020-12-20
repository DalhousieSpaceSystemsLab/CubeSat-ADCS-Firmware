#include <stdint.h>
#include <stdlib.h>

#if defined(TARGET_MCU)
#include "spi.h"
#include "uart.h"
#include "watchdog.h"
#include "mcu.h"
#include "bsp.h"
#include "timer_a0.h"
#else
#include "obc_emulator.h"
#endif /* #if defined(TARGET_MCU) */

#include "obc_interface.h"
#include "jsons.h"

#include "injection_api.h"

/* NOTE THESE 2 HEADERS ARE JUST TEMPORARY STUFF  */

static uint8_t json_buffer[500];


int main(void)
{
#if defined(TARGET_MCU)
    watchdog_stop();

    OBC_IF_config(uart_init, uart_deinit, uart_transmit);

    BSP_init();

    TIMERA0_heartbeat_init();

    callback_handle_t heartbeat_cb = new_callback(BSP_toggle_red_led, NULL);
    TIMERA0_register_callback(heartbeat_cb);

    /* This should be the very last thing that occurs */
    enable_interrupts();
#else
    OBC_IF_config(NULL, NULL, OBC_EMU_tx);
    OBC_EMU_start();
#endif /* #if defined(TARGET_MCU) */

    while (1)
    {

        if (OBC_IF_dataRxFlag_read() == OBC_IF_DATA_RX_FLAG_SET)
        {

#if 0
            /* get command json string from OBC interface */
            OCB_IF_get_command_string(json_buffer, sizeof(json_buffer));
            /* Parse command json string */
            int parse_status = json_parse(json_buffer, sizeof(json_buffer));
            if (JSON_PARSE_ERROR(parse_status))
            {
                uint8_t error_message[] = "{\"error\" : \"json format\"}\n";
                OBC_IF_tx(error_message, sizeof(error_message));
            }
            else if (JSON_PARSE_UNK(parse_status))
            {
                uint8_t message[] = "{\"ADCS\" : \"unknown command\"}\n";
                OBC_IF_tx(message, sizeof(message));
            }
            else
            {
                /* successful, don't do anything */
            }
#endif

            OBC_IF_dataRxFlag_write(OBC_IF_DATA_RX_FLAG_CLR);
        }
    }
}