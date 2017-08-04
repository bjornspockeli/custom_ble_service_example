/*
 * Copyright (c) 2017, Infineon Technologies AG
 *
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 * 1.  Redistributions of source code must retain the above copyright notice,
 *     this list of conditions and the following disclaimer.
 *
 * 2.  Redistributions in binary form must reproduce the above copyright
 *     notice, this list of conditions and the following disclaimer in the
 *     documentation and/or other materials provided with the distribution.
 *
 * 3.  Neither the name of the copyright holder nor the names of its contributors
 *     may be used to endorse or promote products derived from this software
 *     without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 *
 */
/*lint -e???? -save */
/**
 * @file
 * @defgroup ifx_i2c_hal_nordic Infineon I2C Protocol Stack: HAL for Nordic nRF
 * @{
 * @ingroup ifx_i2c
 * @brief Infineon I2C Protocol Stack: Hardware Abstraction Layer Implementation for Nordic nRF
 */

#ifdef NRF52

#include "ifx_i2c_hal.h"
#include "app_twi.h"
#include "app_util_platform.h" // constant APP_IRQ_PRIORITY_HIGH
#include "nrf_delay.h"


/** @brief PIN for I2C SCL to Infineon OPTIGA Trust E device */
#define OPTIGA_PIN_I2C_SCL   (27)
/** @brief PIN for I2C SDA to Infineon OPTIGA Trust E device */
#define OPTIGA_PIN_I2C_SDA   (26)
/** @brief PIN for reset line to Infineon OPTIGA Trust E device */
#define OPTIGA_PIN_RST       (25)


/** @brief Nordic nRF GPIO Pin Configuration */
#define PIN_CONF (   (GPIO_PIN_CNF_SENSE_Disabled   << GPIO_PIN_CNF_SENSE_Pos) \
                   | (GPIO_PIN_CNF_DRIVE_S0S1       << GPIO_PIN_CNF_DRIVE_Pos) \
                   | (GPIO_PIN_CNF_PULL_Disabled    << GPIO_PIN_CNF_PULL_Pos)  \
                   | (GPIO_PIN_CNF_INPUT_Disconnect << GPIO_PIN_CNF_INPUT_Pos) \
                   | (GPIO_PIN_CNF_DIR_Output       << GPIO_PIN_CNF_DIR_Pos) )

/** @brief Delay in us until timer tasks are complete */
#define MAX_RTC_TASKS_DELAY 47

/** @brief I2C driver instance */
#define TWI_INSTANCE_ID             0
/** @brief Maximal number of pending I2C transactions */
#define MAX_PENDING_TRANSACTIONS    5
/** @brief Definition of I2C driver instance */
APP_TWI_DEF(m_app_twi, MAX_PENDING_TRANSACTIONS, TWI_INSTANCE_ID);

static volatile uint16_t m_i2c_length;

// Event handlers
static volatile IFX_I2C_EventHandler m_upper_layer_event_handler = 0;
static volatile IFX_Timer_Callback   m_timer_callback            = 0;

/**
 * @brief APP TWI event handler (remaps events from platform specific values to generic ones)
 */
static void app_twi_callback(ret_code_t result, void * p_user_data)
{
    if (result == NRF_SUCCESS)
    {
        m_upper_layer_event_handler((uint32_t)p_user_data);
    }
    else
    {
        m_upper_layer_event_handler(IFX_I2C_HAL_ERROR);
    }
}

void ifx_timer_setup(uint16_t time_us, IFX_Timer_Callback callback_function)
{
    // Assumes LFCLK is already running (is the case after starting the SoftDevice)
    m_timer_callback = callback_function;

    NRF_RTC2->EVTENSET = RTC_EVTEN_COMPARE0_Msk;
    NRF_RTC2->INTENSET = RTC_INTENSET_COMPARE0_Msk;
    NRF_RTC2->CC[0] = (time_us/30) + 1; // Approximation using 30.517 us ticks

    NVIC_EnableIRQ(RTC2_IRQn);
    NRF_RTC2->TASKS_START = 1;
}

/**
 * @brief I2C Data Link Layer Timer Interrupt Handler
 */
void RTC2_IRQHandler(void)
{
    NRF_RTC2->EVENTS_COMPARE[0] = 0;

    NRF_RTC2->TASKS_STOP = 1;
    nrf_delay_us(MAX_RTC_TASKS_DELAY);
    NRF_RTC2->TASKS_CLEAR = 1;
    nrf_delay_us(MAX_RTC_TASKS_DELAY);

    if (m_timer_callback)
    {
        m_timer_callback();
    }
}

uint16_t ifx_i2c_init(uint8_t reinit, IFX_I2C_EventHandler handler)
{
    nrf_drv_twi_config_t const config = {
           .scl                = OPTIGA_PIN_I2C_SCL,
           .sda                = OPTIGA_PIN_I2C_SDA,
           .frequency          = NRF_TWI_FREQ_400K,
           .interrupt_priority = APP_IRQ_PRIORITY_LOWEST,
           .clear_bus_init     = false
        };

    m_upper_layer_event_handler = handler;

    // Configure RESET pin
    NRF_GPIO->PIN_CNF[OPTIGA_PIN_RST] = PIN_CONF;

    // Timer (RTC2) Setup
    NRF_RTC2->PRESCALER = 0; // 32 kHz
    NVIC_SetPriority(RTC2_IRQn, APP_IRQ_PRIORITY_LOW);

    // Initialize I2C driver
    if (app_twi_init(&m_app_twi, &config) != NRF_SUCCESS)
    {
        return IFX_I2C_STACK_ERROR;
    }

    // Perform hardware-triggered warm reset via RST pin
    NRF_GPIO->OUTSET = 1 << OPTIGA_PIN_RST;
    nrf_delay_ms(10);
    NRF_GPIO->OUTCLR = 1 << OPTIGA_PIN_RST;
    nrf_delay_ms(1);
    NRF_GPIO->OUTSET = 1 << OPTIGA_PIN_RST;
    nrf_delay_ms(12);

    return IFX_I2C_STACK_SUCCESS;
}

static app_twi_transfer_t    m_transfer;
static app_twi_transaction_t m_transaction;

void ifx_i2c_transmit(uint8_t* p_data, uint16_t length)
{
    m_i2c_length = length;

    m_transfer.p_data    = p_data;
    m_transfer.length    = length;
    m_transfer.operation = APP_TWI_WRITE_OP(IFX_I2C_BASE_ADDR);
    m_transfer.flags     = 0;

    m_transaction.callback            = app_twi_callback;
    m_transaction.number_of_transfers = 1;
    m_transaction.p_required_twi_cfg  = NULL;
    m_transaction.p_transfers         = &m_transfer;
    m_transaction.p_user_data         = (void*) IFX_I2C_HAL_TX_SUCCESS;

    if (app_twi_schedule(&m_app_twi, &m_transaction) != NRF_SUCCESS)
    {
        app_twi_callback(NRF_ERROR_BUSY, 0);
    }
}

void ifx_i2c_receive(uint8_t* p_data, uint16_t length)
{
    m_i2c_length = length;

    m_transfer.p_data    = p_data;
    m_transfer.length    = length;
    m_transfer.operation = APP_TWI_READ_OP(IFX_I2C_BASE_ADDR);
    m_transfer.flags     = 0;

    m_transaction.callback            = app_twi_callback;
    m_transaction.number_of_transfers = 1;
    m_transaction.p_required_twi_cfg  = 0;
    m_transaction.p_transfers         = &m_transfer;
    m_transaction.p_user_data         = (void*) IFX_I2C_HAL_RX_SUCCESS;

    if (app_twi_schedule(&m_app_twi, &m_transaction) != NRF_SUCCESS)
    {
        app_twi_callback(NRF_ERROR_BUSY, 0);
    }
}

#if IFX_I2C_LOG_PL == 1 || IFX_I2C_LOG_DL == 1 || IFX_I2C_LOG_TL == 1
#include "nrf_log.h"
#include "nrf_log_ctrl.h"
#include <stdarg.h>
void ifx_debug_log(uint8_t log_id, char * format_msg, ...)
{
    uint16_t i;
    uint16_t arg_count = 0;
    uint16_t string_len = strlen(format_msg);
    uint32_t arg1, arg2;
    va_list p_args;

    for (i = 0; i < string_len; i++)
    {
        if (format_msg[i] == '%')
        {
            arg_count++;
        }
    }

    va_start(p_args, format_msg);

    if (arg_count == 0)
    {
        nrf_log_frontend_std_0(NRF_LOG_LEVEL_INFO, format_msg);
    }
    else if (arg_count == 1)
    {
        arg1 = va_arg(p_args, uint32_t);
        nrf_log_frontend_std_1(NRF_LOG_LEVEL_INFO, format_msg, arg1);
    }
    else if (arg_count == 2)
    {
        arg1 = va_arg(p_args, uint32_t);
        arg2 = va_arg(p_args, uint32_t);
        nrf_log_frontend_std_2(NRF_LOG_LEVEL_INFO, format_msg, arg1, arg2);
    }
    else
    {
        nrf_log_frontend_std_0(NRF_LOG_LEVEL_INFO, "IFX Logging error: to many arguments.");
    }

    va_end(p_args);

    NRF_LOG_FLUSH();
}
#endif

#endif // ifdef NRF52

/** @} */
/* lint -restore */
