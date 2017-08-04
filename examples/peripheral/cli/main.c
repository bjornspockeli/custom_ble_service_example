/**
 * Copyright (c) 2016 - 2017, Nordic Semiconductor ASA
 * 
 * All rights reserved.
 * 
 * Redistribution and use in source and binary forms, with or without modification,
 * are permitted provided that the following conditions are met:
 * 
 * 1. Redistributions of source code must retain the above copyright notice, this
 *    list of conditions and the following disclaimer.
 * 
 * 2. Redistributions in binary form, except as embedded into a Nordic
 *    Semiconductor ASA integrated circuit in a product or a software update for
 *    such product, must reproduce the above copyright notice, this list of
 *    conditions and the following disclaimer in the documentation and/or other
 *    materials provided with the distribution.
 * 
 * 3. Neither the name of Nordic Semiconductor ASA nor the names of its
 *    contributors may be used to endorse or promote products derived from this
 *    software without specific prior written permission.
 * 
 * 4. This software, with or without modification, must only be used with a
 *    Nordic Semiconductor ASA integrated circuit.
 * 
 * 5. Any software provided in binary form under this license must not be reverse
 *    engineered, decompiled, modified and/or disassembled.
 * 
 * THIS SOFTWARE IS PROVIDED BY NORDIC SEMICONDUCTOR ASA "AS IS" AND ANY EXPRESS
 * OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES
 * OF MERCHANTABILITY, NONINFRINGEMENT, AND FITNESS FOR A PARTICULAR PURPOSE ARE
 * DISCLAIMED. IN NO EVENT SHALL NORDIC SEMICONDUCTOR ASA OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE
 * GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION)
 * HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT
 * OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 * 
 */
#include <stdio.h>
#include <stdbool.h>
#include <stddef.h>
#include <ctype.h>


#include "nrf.h"
#include "nrf_drv_clock.h"
#include "nrf_gpio.h"
#include "nrf_delay.h"

#include "nrf_drv_power.h"
#include "app_timer.h"

#include "app_error.h"
#include "app_util.h"

#include "nrf_cli.h"
#include "nrf_cli_uart.h"
#include "nrf_cli_rtt.h"
#include "nrf_cli_types.h"

#ifdef NRF52840_XXAA
#include "nrf_cli_cdc_acm.h"

#include "nrf_drv_usbd.h"
#include "app_usbd_core.h"
#include "app_usbd.h"
#include "app_usbd_string_desc.h"
#include "app_usbd_cdc_acm.h"
#endif

#include "boards.h"

#include "nrf_log.h"
#include "nrf_log_ctrl.h"
#include "nrf_log_default_backends.h"

/**@file
 * @defgroup CLI_example main.c
 *
 * @{
 *
 */
#define LED_USB_RESUME     (BSP_BOARD_LED_0)
#define LED_CDC_ACM_OPEN   (BSP_BOARD_LED_1)
#define LED_CDC_ACM_RX     (BSP_BOARD_LED_2)
#define LED_CDC_ACM_TX     (BSP_BOARD_LED_3)

#define CLI_EXAMPLE_MAX_CMD_CNT (20u)
#define CLI_EXAMPLE_MAX_CMD_LEN (33u)
/* buffer holding dynamicly created user commands */
static char m_dynamic_cmd_buffer[CLI_EXAMPLE_MAX_CMD_CNT][CLI_EXAMPLE_MAX_CMD_LEN];
/* commands counter */
static uint8_t m_dynamic_cmd_cnt;


#ifdef BOARD_PCA10056
/**
 * @brief Enable power USB detection
 *
 * Configure if example supports USB port connection
 */
#ifndef USBD_POWER_DETECTION
#define USBD_POWER_DETECTION true
#endif

/**
 * @brief  USB connection status
 * */
static bool m_usb_connected = false;

static void power_usb_event_handler(nrf_drv_power_usb_evt_t event)
{
    switch(event)
    {
        case NRF_DRV_POWER_USB_EVT_DETECTED:

            if (!nrf_drv_usbd_is_enabled())
            {
                app_usbd_enable();
            }
            break;
        case NRF_DRV_POWER_USB_EVT_REMOVED:
            m_usb_connected = false;
            break;
        case NRF_DRV_POWER_USB_EVT_READY:
            m_usb_connected = true;
            break;
        default:
            ASSERT(false);
    }
}

static void usbd_user_ev_handler(app_usbd_event_type_t event)
{
    switch (event)
    {
        case APP_USBD_EVT_STOPPED:
            app_usbd_disable();
            break;
        default:
            break;
    }
}

static void usb_start(void)
{
    if (USBD_POWER_DETECTION)
    {
        static const nrf_drv_power_usbevt_config_t config =
        {
            .handler = power_usb_event_handler
        };
        ret_code_t ret;

        ret = nrf_drv_power_usbevt_init(&config);
        APP_ERROR_CHECK(ret);
    }
    else
    {
        app_usbd_enable();
        app_usbd_start();
        m_usb_connected = true;
    }
}

static bool usb_connection_handle(bool last_usb_conn_status)
{
    if (last_usb_conn_status != m_usb_connected)
    {
        last_usb_conn_status = m_usb_connected;
        m_usb_connected ? app_usbd_start() : app_usbd_stop();
    }

    return last_usb_conn_status;
}
#endif

static uint32_t m_counter;
static bool m_counter_active = false;


/**
 * @brief Command line interface instance
 * */
#define CLI_EXAMPLE_LOG_QUEUE_SIZE  (4)
#ifdef BOARD_PCA10056
NRF_CLI_CDC_ACM_DEF(m_cli_cdc_acm_transport);
NRF_CLI_DEF(m_cli_cdc_acm,
            "usb_cli:~$ ",
            &m_cli_cdc_acm_transport.transport,
            '\r',
            CLI_EXAMPLE_LOG_QUEUE_SIZE);
#endif
NRF_CLI_UART_DEF(m_cli_uart_transport, 0, 64, 16);
NRF_CLI_DEF(m_cli_uart,
            "uart_cli:~$ ",
            &m_cli_uart_transport.transport,
            '\r',
            CLI_EXAMPLE_LOG_QUEUE_SIZE);
NRF_CLI_RTT_DEF(m_cli_rtt_transport);
NRF_CLI_DEF(m_cli_rtt,
            "rtt_cli:~$ ",
            &m_cli_rtt_transport.transport,
            '\n',
            CLI_EXAMPLE_LOG_QUEUE_SIZE);

static void timer_handle(void * p_context)
{
    if (m_counter_active)
    {
        m_counter++;
        NRF_LOG_RAW_INFO("counter = %d\r\n", m_counter);
    }
}

static void cli_start(void)
{
    ret_code_t ret;
#ifdef BOARD_PCA10056
    ret = nrf_cli_start(&m_cli_cdc_acm);
    APP_ERROR_CHECK(ret);
#endif
    ret = nrf_cli_start(&m_cli_uart);
    APP_ERROR_CHECK(ret);
    ret = nrf_cli_start(&m_cli_rtt);
    APP_ERROR_CHECK(ret);
}

int main(void)
{
    ret_code_t ret;

    APP_ERROR_CHECK(NRF_LOG_INIT(app_timer_cnt_get));

    ret = nrf_drv_clock_init();
    APP_ERROR_CHECK(ret);
    nrf_drv_clock_lfclk_request(NULL);

    ret = nrf_drv_power_init(NULL);
    APP_ERROR_CHECK(ret);

    bsp_board_leds_init();
    bsp_board_buttons_init();

    ret = app_timer_init();
    APP_ERROR_CHECK(ret);

    APP_TIMER_DEF(timer_0);
    ret = app_timer_create(&timer_0, APP_TIMER_MODE_REPEATED, timer_handle);
    APP_ERROR_CHECK(ret);

    ret = app_timer_start(timer_0, APP_TIMER_TICKS(1000), NULL);
    APP_ERROR_CHECK(ret);

#ifdef BOARD_PCA10056
    ret = nrf_cli_init(&m_cli_cdc_acm, NULL, true, true, NRF_LOG_SEVERITY_INFO);
    APP_ERROR_CHECK(ret);
#endif

    nrf_drv_uart_config_t uart_config = NRF_DRV_UART_DEFAULT_CONFIG;
    uart_config.pseltxd = TX_PIN_NUMBER;
    uart_config.pselrxd = RX_PIN_NUMBER;
    uart_config.hwfc    = NRF_UART_HWFC_DISABLED;
    ret = nrf_cli_init(&m_cli_uart, &uart_config, true, true, NRF_LOG_SEVERITY_INFO);
    APP_ERROR_CHECK(ret);

    ret = nrf_cli_init(&m_cli_rtt, NULL, true, true, NRF_LOG_SEVERITY_INFO);
    APP_ERROR_CHECK(ret);

#ifdef BOARD_PCA10056
    static const app_usbd_config_t usbd_config = {
        .ev_handler = app_usbd_event_execute,
        .ev_state_proc = usbd_user_ev_handler
    };
    ret = app_usbd_init(&usbd_config);
    APP_ERROR_CHECK(ret);

    app_usbd_class_inst_t const * class_cdc_acm =
            app_usbd_cdc_acm_class_inst_get(&nrf_cli_cdc_acm);
    ret = app_usbd_class_append(class_cdc_acm);
    APP_ERROR_CHECK(ret);

    bool last_usb_conn_status = false;
    bool last_port_status = false;
    usb_start();
#endif

    cli_start();

    NRF_LOG_RAW_INFO("Command Line Interface example.\r\n");
    NRF_LOG_RAW_INFO("Please press <Tab> to see all available commands.\r\n");
    while (true)
    {
        (void)NRF_LOG_PROCESS();

        nrf_cli_process(&m_cli_rtt);
        nrf_cli_process(&m_cli_uart);
#ifdef BOARD_PCA10056
#if APP_USBD_EVENT_QUEUE_ENABLE
        while (app_usbd_event_queue_process())
        {
            /* Nothing to do */
        }
#endif
        last_usb_conn_status = usb_connection_handle(last_usb_conn_status);
        if (last_port_status != nrf_cli_cdc_acm_port_is_open())
        {
            last_port_status = nrf_cli_cdc_acm_port_is_open();
        }
        nrf_cli_process(&m_cli_cdc_acm);
#endif
    }
}

/* Command handlers */
static void cmd_print_param(nrf_cli_t const * p_cli, size_t argc, char **argv)
{
    for (size_t i = 1; i < argc; i++)
    {
        nrf_cli_fprintf(p_cli, NRF_CLI_NORMAL, "argv[%d] = %s\r\n", i, argv[i]);
    }
}

static void cmd_print_all(nrf_cli_t const * p_cli, size_t argc, char **argv)
{
    for (size_t i = 1; i < argc; i++)
    {
        nrf_cli_fprintf(p_cli, NRF_CLI_NORMAL, "%s ", argv[i]);
    }
    nrf_cli_fprintf(p_cli, NRF_CLI_NORMAL, "\r\n");
}

static void cmd_print(nrf_cli_t const * p_cli, size_t argc, char **argv)
{
    ASSERT(p_cli);
    ASSERT(p_cli->p_ctx && p_cli->p_iface && p_cli->p_name);

    if ((argc == 1) || nrf_cli_help_requested(p_cli))
    {
        nrf_cli_help_print(p_cli, NULL, 0);
        return;
    }

    if (argc != 2)
    {
        nrf_cli_fprintf(p_cli, NRF_CLI_ERROR, "%s: bad parameter count\r\n", argv[0]);
        return;
    }

    nrf_cli_fprintf(p_cli, NRF_CLI_ERROR, "%s: unknown parameter: %s\r\n", argv[0], argv[1]);
}

static void cmd_python(nrf_cli_t const * p_cli, size_t argc, char **argv)
{
    nrf_cli_fprintf(p_cli, NRF_CLI_ERROR, "Nice joke ;)\r\n");
}

static void cmd_dynamic(nrf_cli_t const * p_cli, size_t argc, char **argv)
{
    if ((argc == 1) || nrf_cli_help_requested(p_cli))
    {
        nrf_cli_help_print(p_cli, NULL, 0);
        return;
    }

    if (argc > 2)
    {
        nrf_cli_fprintf(p_cli, NRF_CLI_ERROR, "%s: bad parameter count\r\n", argv[0]);
    }
    else
    {
        nrf_cli_fprintf(p_cli, NRF_CLI_ERROR, "%s: please specify subcommand\r\n", argv[0]);
    }
}

/* function required by qsort */
static int string_cmp(const void * p_a, const void * p_b)
{
    ASSERT(p_a);
    ASSERT(p_b);
    return strcmp((const char *)p_a, (const char *)p_b);
}

static void cmd_dynamic_add(nrf_cli_t const * p_cli, size_t argc, char **argv)
{
    if (nrf_cli_help_requested(p_cli))
    {
        nrf_cli_help_print(p_cli, NULL, 0);
        return;
    }

    if (argc != 2)
    {
        nrf_cli_fprintf(p_cli, NRF_CLI_ERROR, "%s: bad parameter count\r\n", argv[0]);
        return;
    }

    if (m_dynamic_cmd_cnt >= CLI_EXAMPLE_MAX_CMD_CNT)
    {
        nrf_cli_fprintf(p_cli, NRF_CLI_ERROR, "command limit reached\r\n");
        return;
    }

    uint8_t idx;
    nrf_cli_cmd_len_t cmd_len = strlen(argv[1]);

    if (cmd_len >= CLI_EXAMPLE_MAX_CMD_LEN)
    {
        nrf_cli_fprintf(p_cli, NRF_CLI_ERROR, "too long command\r\n");
        return;
    }

    for (idx = 0; idx < cmd_len; idx++)
    {
        if (!isalnum((int)(argv[1][idx])))
        {
            nrf_cli_fprintf(p_cli,
                            NRF_CLI_ERROR,
                            "bad command name - please use only alphanumerical characters\r\n");
            return;
        }
    }

    for (idx = 0; idx < CLI_EXAMPLE_MAX_CMD_CNT; idx++)
    {
        if (!strcmp(m_dynamic_cmd_buffer[idx], argv[1]))
        {
            nrf_cli_fprintf(p_cli, NRF_CLI_ERROR, "duplicated command\r\n");
            return;
        }
    }

    sprintf(m_dynamic_cmd_buffer[m_dynamic_cmd_cnt++], "%s", argv[1]);

    qsort(m_dynamic_cmd_buffer,
          m_dynamic_cmd_cnt,
          sizeof (m_dynamic_cmd_buffer[0]),
          string_cmp);

    nrf_cli_fprintf(p_cli, NRF_CLI_NORMAL, "command added successfully\r\n");
}

static void cmd_dynamic_show(nrf_cli_t const * p_cli, size_t argc, char **argv)
{
    if (nrf_cli_help_requested(p_cli))
    {
        nrf_cli_help_print(p_cli, NULL, 0);
        return;
    }

    if (argc != 1)
    {
        nrf_cli_fprintf(p_cli, NRF_CLI_ERROR, "%s: bad parameter count\r\n", argv[0]);
        return;
    }

    if (m_dynamic_cmd_cnt == 0)
    {
        nrf_cli_fprintf(p_cli, NRF_CLI_WARNING, "Please add some commands first.\r\n");
        return;
    }
    nrf_cli_fprintf(p_cli, NRF_CLI_NORMAL, "Dynamic command list:\r\n");
    for (uint8_t i = 0; i < m_dynamic_cmd_cnt; i++)
    {
        nrf_cli_fprintf(p_cli, NRF_CLI_NORMAL, "[%3d] %s\r\n", i, m_dynamic_cmd_buffer[i]);
    }
}

static void cmd_dynamic_execute(nrf_cli_t const * p_cli, size_t argc, char **argv)
{
    if (nrf_cli_help_requested(p_cli))
    {
        nrf_cli_help_print(p_cli, NULL, 0);
        return;
    }

    if (argc != 2)
    {
        nrf_cli_fprintf(p_cli, NRF_CLI_ERROR, "%s: bad parameter count\r\n", argv[0]);
        return;
    }

    for (uint8_t idx = 0; idx <  m_dynamic_cmd_cnt; idx++)
    {
        if (!strcmp(m_dynamic_cmd_buffer[idx], argv[1]))
        {
            nrf_cli_fprintf(p_cli, NRF_CLI_NORMAL, "dynamic command: %s\r\n", argv[1]);
            return;
        }
    }
    nrf_cli_fprintf(p_cli, NRF_CLI_ERROR, "%s: uknown parameter: %s\r\n", argv[0], argv[1]);
}

static void cmd_dynamic_remove(nrf_cli_t const * p_cli, size_t argc, char **argv)
{
    if ((argc == 1) || nrf_cli_help_requested(p_cli))
    {
        nrf_cli_help_print(p_cli, NULL, 0);
        return;
    }

    if (argc != 2)
    {
        nrf_cli_fprintf(p_cli, NRF_CLI_ERROR, "%s: bad parameter count\r\n", argv[0]);
        return;
    }

    for (uint8_t idx = 0; idx <  m_dynamic_cmd_cnt; idx++)
    {
        if (!strcmp(m_dynamic_cmd_buffer[idx], argv[1]))
        {
            if (idx == CLI_EXAMPLE_MAX_CMD_CNT - 1)
            {
                m_dynamic_cmd_buffer[idx][0] = '\0';
            }
            else
            {
                memmove(m_dynamic_cmd_buffer[idx],
                        m_dynamic_cmd_buffer[idx + 1],
                        sizeof(m_dynamic_cmd_buffer[idx]) * (m_dynamic_cmd_cnt - idx));
            }
            --m_dynamic_cmd_cnt;
            nrf_cli_fprintf(p_cli, NRF_CLI_NORMAL, "command removed successfully\r\n");
            return;
        }
    }
    nrf_cli_fprintf(p_cli, NRF_CLI_ERROR, "did not find command: %s\r\n", argv[1]);
}

static void cmd_counter_start(nrf_cli_t const * p_cli, size_t argc, char **argv)
{
    if (argc != 1)
    {
        nrf_cli_fprintf(p_cli, NRF_CLI_ERROR, "%s: bad parameter count\r\n", argv[0]);
        return;
    }

    m_counter_active = true;
}

static void cmd_counter_stop(nrf_cli_t const * p_cli, size_t argc, char **argv)
{
    if (argc != 1)
    {
        nrf_cli_fprintf(p_cli, NRF_CLI_ERROR, "%s: bad parameter count\r\n", argv[0]);
        return;
    }

    m_counter_active = false;
}

static void cmd_counter_reset(nrf_cli_t const * p_cli, size_t argc, char **argv)
{
    if (argc != 1)
    {
        nrf_cli_fprintf(p_cli, NRF_CLI_ERROR, "%s: bad parameter count\r\n", argv[0]);
        return;
    }

    m_counter = 0;
}

static void cmd_counter(nrf_cli_t const * p_cli, size_t argc, char **argv)
{
    ASSERT(p_cli);
    ASSERT(p_cli->p_ctx && p_cli->p_iface && p_cli->p_name);

    /* Extra defined dummy option */
    static const nrf_cli_getopt_option_t opt[] = {
        NRF_CLI_OPT(
            "--test",
            "-t",
            "dummy option help string"
        )
    };

    if ((argc == 1) || nrf_cli_help_requested(p_cli))
    {
        nrf_cli_help_print(p_cli, opt, ARRAY_SIZE(opt));
        return;
    }

    if (argc != 2)
    {
        nrf_cli_fprintf(p_cli, NRF_CLI_ERROR, "%s: bad parameter count\r\n", argv[0]);
        return;
    }

    if (!strcmp(argv[1], "-t") || !strcmp(argv[1], "--test"))
    {
        nrf_cli_fprintf(p_cli, NRF_CLI_NORMAL, "Dummy test option.\r\n");
        return;
    }

    /* subcommands have their own handlers and they are not processed here */
    nrf_cli_fprintf(p_cli, NRF_CLI_ERROR, "%s: unknown parameter: %s\r\n", argv[0], argv[1]);
}

static void cmd_nordic(nrf_cli_t const * p_cli, size_t argc, char **argv)
{
    if (nrf_cli_help_requested(p_cli))
    {
        nrf_cli_help_print(p_cli, NULL, 0);
        return;
    }

    nrf_cli_fprintf(p_cli, NRF_CLI_OPTION,
                    "\r\n"
                    "            .co:.                   'xo,          \r\n"
                    "         .,collllc,.             'ckOOo::,..      \r\n"
                    "      .:ooooollllllll:'.     .;dOOOOOOo:::;;;'.   \r\n"
                    "   'okxddoooollllllllllll;'ckOOOOOOOOOo:::;;;,,,' \r\n"
                    "   OOOkxdoooolllllllllllllllldxOOOOOOOo:::;;;,,,'.\r\n"
                    "   OOOOOOkdoolllllllllllllllllllldxOOOo:::;;;,,,'.\r\n"
                    "   OOOOOOOOOkxollllllllllllllllllcccldl:::;;;,,,'.\r\n"
                    "   OOOOOOOOOOOOOxdollllllllllllllccccc::::;;;,,,'.\r\n"
                    "   OOOOOOOOOOOOOOOOkxdlllllllllllccccc::::;;;,,,'.\r\n"
                    "   kOOOOOOOOOOOOOOOOOOOkdolllllllccccc::::;;;,,,'.\r\n"
                    "   kOOOOOOOOOOOOOOOOOOOOOOOxdllllccccc::::;;;,,,'.\r\n"
                    "   kOOOOOOOOOOOOOOOOOOOOOOOOOOkxolcccc::::;;;,,,'.\r\n"
                    "   kOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOkdlc::::;;;,,,'.\r\n"
                    "   xOOOOOOOOOOOxdkOOOOOOOOOOOOOOOOOOOOxoc:;;;,,,'.\r\n"
                    "   xOOOOOOOOOOOdc::ldkOOOOOOOOOOOOOOOOOOOkdc;,,,''\r\n"
                    "   xOOOOOOOOOOOdc::;;,;cdkOOOOOOOOOOOOOOOOOOOxl;''\r\n"
                    "   .lkOOOOOOOOOdc::;;,,''..;oOOOOOOOOOOOOOOOOOOOx'\r\n"
                    "      .;oOOOOOOdc::;;,.       .:xOOOOOOOOOOOOd;.  \r\n"
                    "          .:xOOdc:,.              'ckOOOOkl'      \r\n"
                    "             .od'                    'xk,         \r\n"
                    "\r\n");

    nrf_cli_fprintf(p_cli,NRF_CLI_NORMAL,
                    "                Nordic Semiconductor              \r\n\r\n");
}

/**
 * @brief Command set array
 * */
NRF_CLI_CMD_REGISTER(nordic, NULL, "Print Nordic Semiconductor logo.", cmd_nordic);

NRF_CLI_CREATE_STATIC_SUBCMD_SET(m_sub_print)
{
    NRF_CLI_CMD(all,   NULL, "Print all entered parameters.", cmd_print_all),
    NRF_CLI_CMD(param, NULL, "Print each parameter in new line.", cmd_print_param),
    NRF_CLI_SUBCMD_SET_END
};
NRF_CLI_CMD_REGISTER(print, &m_sub_print, "print", cmd_print);

NRF_CLI_CMD_REGISTER(python, NULL, "python", cmd_python);

NRF_CLI_CREATE_STATIC_SUBCMD_SET(m_sub_counter)
{
    NRF_CLI_CMD(reset,  NULL, "Reset seconds counter.",  cmd_counter_reset),
    NRF_CLI_CMD(start,  NULL, "Start seconds counter.",  cmd_counter_start),
    NRF_CLI_CMD(stop,   NULL, "Stop seconds counter.",   cmd_counter_stop),
    NRF_CLI_SUBCMD_SET_END
};
NRF_CLI_CMD_REGISTER(counter,
                     &m_sub_counter,
                     "Display seconds on terminal screen",
                     cmd_counter);

/* dynamic command creation */
static void dynamic_cmd_get(size_t idx, nrf_cli_static_entry_t * p_static)
{
    ASSERT(p_static);

    if (idx < m_dynamic_cmd_cnt)
    {
        /* m_dynamic_cmd_buffer must be sorted alphabetically to ensure correct CLI completion */
        p_static->p_syntax = m_dynamic_cmd_buffer[idx];
        p_static->handler  = NULL;
        p_static->p_subcmd = NULL;
        p_static->p_help = "Show dynamic command name.";
    }
    else
    {
        /* if there are no more dynamic commands available p_syntax must be set to NULL */
        p_static->p_syntax = NULL;
    }
}

NRF_CLI_CREATE_DYNAMIC_CMD(m_sub_dynamic_set, dynamic_cmd_get);
NRF_CLI_CREATE_STATIC_SUBCMD_SET(m_sub_dynamic)
{
    NRF_CLI_CMD(add, NULL,
        "Add a new dynamic command.\nExample usage: [ dynamic add test ] will add "
        "a dynamic command 'test'.\nIn this example, command name length is limited to 32 chars. "
        "You can add up to 20 commands. Commands are automatically sorted to ensure correct "
        "CLI completion.",
        cmd_dynamic_add),
    NRF_CLI_CMD(execute, &m_sub_dynamic_set, "Execute a command.", cmd_dynamic_execute),
    NRF_CLI_CMD(remove, &m_sub_dynamic_set, "Remove a command.", cmd_dynamic_remove),
    NRF_CLI_CMD(show, NULL, "Show all added dynamic commands.", cmd_dynamic_show),
    NRF_CLI_SUBCMD_SET_END
};
NRF_CLI_CMD_REGISTER(dynamic,
                     &m_sub_dynamic,
                     "Demonstrate dynamic command usage.",
                     cmd_dynamic);

/** @} */
