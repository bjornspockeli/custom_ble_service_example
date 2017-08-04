/**
 * This software is subject to the ANT+ Shared Source License
 * www.thisisant.com/swlicenses
 * Copyright (c) Dynastream Innovations, Inc. 2014
 * All rights reserved.
 * 
 * Redistribution and use in source and binary forms, with or
 * without modification, are permitted provided that the following
 * conditions are met:
 * 1) Redistributions of source code must retain the above
 *    copyright notice, this list of conditions and the following
 *    disclaimer.
 * 
 * 2) Redistributions in binary form must reproduce the above
 *    copyright notice, this list of conditions and the following
 *    disclaimer in the documentation and/or other materials
 *    provided with the distribution.
 * 
 * 3) Neither the name of Dynastream nor the names of its
 *    contributors may be used to endorse or promote products
 *    derived from this software without specific prior
 *    written permission.
 * 
 * The following actions are prohibited:
 * 1) Redistribution of source code containing the ANT+ Network
 *    Key. The ANT+ Network Key is available to ANT+ Adopters.
 *    Please refer to http://thisisant.com to become an ANT+
 *    Adopter and access the key.
 * 
 * 2) Reverse engineering, decompilation, and/or disassembly of
 *    software provided in binary form under this license.
 * 
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND
 * CONTRIBUTORS "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES,
 * INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF
 * MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE HEREBY
 * DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR
 * CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL,
 * SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES(INCLUDING,
 * BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; DAMAGE TO ANY DEVICE, LOSS OF USE, DATA, OR
 * PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON
 * ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
 * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 * ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED
 * OF THE POSSIBILITY OF SUCH DAMAGE. SOME STATES DO NOT ALLOW
 * THE EXCLUSION OF INCIDENTAL OR CONSEQUENTIAL DAMAGES, SO THE
 * ABOVE LIMITATIONS MAY NOT APPLY TO YOU.
 * 
 */

#include <stdbool.h>
#include <stdint.h>
#include "app_error.h"
#include "nrf.h"
#include "ant_interface.h"
#include "ant_parameters.h"
#include "nrf_soc.h"
#include "nrf_sdm.h"
#include "nrf_delay.h"
#include "nrf_gpio.h"
#include "ant_error.h"
#include "ant_boot_settings_api.h"
#include <string.h>
#include "ant_stack_config.h"
#include "app_util_platform.h"
#include "boards.h"


// Channel configuration.
#define ANT_CHANNEL_NUMBER              0x00                 /**< ANT Channel Number*/
#define ANT_RF_FREQUENCY                0x32u                /**< Channel RF Frequency = (2400 + 50)MHz */
#define ANT_CHANNEL_PERIOD              8192u                /**< Channel period 4 Hz. */
#define ANT_EXT_ASSIGN                  0x00                 /**< ANT Ext Assign. */
#define ANT_NETWORK_KEY                {0xE8, 0xE4, 0x21, 0x3B, 0x55, 0x7A, 0x67, 0xC1}      /**< ANT public network key. */
#define ANT_NETWORK_NUMBER              0x00                 /**< Network Number */
#define ANT_TRANSMIT_POWER              0u                   /**< ANT Custom Transmit Power (Invalid/Not Used). */

// Channel ID configuration.
#define ANT_DEV_TYPE                    0x20u                /**< Device type = 32. */
#define ANT_TRANS_TYPE                  0x05u                /**< Transmission type. */
#define ANT_DEV_NUM                     (NRF_FICR->DEVICEID[0])         /**< Device number. */

// Test broadcast data
#define BROADCAST_PAYLOAD               {0x01, 0x00, 0x00, 0x00, 0x00, 0xC0, 0xFF, 0xEE}
#define BROADCAST_DATA_BUFFER_SIZE      8

// Version string
#define VERSION_STRING              "BFM1.00B01"

// Message definitions
#define COMMAND_ID                      0x02u
#define COMMAND_RESTART_BOOTLOADER      0x01u

static const uint8_t m_version_string[] = VERSION_STRING; // Version string

/**@brief Function for handling an error.
 *
 * @param[in] id    Fault identifier. See @ref NRF_FAULT_IDS.
 * @param[in] pc    The program counter of the instruction that triggered the fault, or 0 if
 *                  unavailable.
 * @param[in] info  Optional additional information regarding the fault. Refer to each fault
 *                  identifier for details.
 */
void app_error_fault_handler(uint32_t id, uint32_t pc, uint32_t info)
{
    for (;;)
    {
        // No implementation needed.
    }
}


/**@brief Reset the device, and start bootloader
*/
static void restart_in_bootloader(void)
{
    uint32_t err_code;
    static ant_boot_settings_t ant_boot_settings;

    err_code = ant_boot_settings_clear(&ant_boot_settings); // Clears and set FFs to the memory block
    APP_ERROR_CHECK(err_code);
    memcpy((void*) ant_boot_settings.app_version, m_version_string, sizeof(m_version_string));
    ant_boot_settings.app_size = 2000;                      // Estimated current application size used to try to preserve itself
    err_code = ant_boot_settings_save(&ant_boot_settings);
    APP_ERROR_CHECK(err_code);
    ant_boot_settings_validate(1);                          // Sets flag to indicate restarting in bootloader mode. Must be done last before the reset!!!
    NVIC_SystemReset();
}

/**@brief Function for setting up the ANT module to be ready for TX broadcast.
 */
static void ant_channel_tx_broadcast_setup(void)
{
    uint32_t err_code;
    uint8_t  m_network_key[] = ANT_NETWORK_KEY;
    uint8_t m_broadcast_data[] = BROADCAST_PAYLOAD;

    // Set Network Key.
    err_code = sd_ant_network_address_set(ANT_NETWORK_NUMBER, (uint8_t*)m_network_key);
    APP_ERROR_CHECK(err_code);

    // Assign Channel
    err_code = sd_ant_channel_assign(ANT_CHANNEL_NUMBER,
                                     CHANNEL_TYPE_MASTER,
                                     ANT_NETWORK_NUMBER,
                                     ANT_EXT_ASSIGN);

    APP_ERROR_CHECK(err_code);

    // Set Channel ID.
    err_code = sd_ant_channel_id_set(ANT_CHANNEL_NUMBER,
                                     ANT_DEV_NUM,
                                     ANT_DEV_TYPE,
                                     ANT_TRANS_TYPE);
    APP_ERROR_CHECK(err_code);

    // Set Channel Period
    err_code = sd_ant_channel_period_set(ANT_CHANNEL_NUMBER, ANT_CHANNEL_PERIOD);
    APP_ERROR_CHECK(err_code);

    // Set RF Frequency
    err_code = sd_ant_channel_radio_freq_set(ANT_CHANNEL_NUMBER, ANT_RF_FREQUENCY);
    APP_ERROR_CHECK(err_code);

    //Set Tx Power
    err_code = sd_ant_channel_radio_tx_power_set(ANT_CHANNEL_NUMBER, RADIO_TX_POWER_LVL_3, ANT_TRANSMIT_POWER);
    APP_ERROR_CHECK(err_code);

    // Setup broadcast payload
    err_code = sd_ant_broadcast_message_tx(ANT_CHANNEL_NUMBER,
                                          BROADCAST_DATA_BUFFER_SIZE,
                                          m_broadcast_data);

    if (err_code != NRF_ANT_ERROR_CHANNEL_IN_WRONG_STATE)
    {
        APP_ERROR_CHECK(err_code);
    }

    // Open channel
    err_code = sd_ant_channel_open(ANT_CHANNEL_NUMBER);
    APP_ERROR_CHECK(err_code);
}


/**@brief Function for handling ANT TX channel events.
 *
 * @param[in] event The received ANT event to handle.
 * @param[in] p_ant_message The ANT message structure
 */
static void channel_event_handle(uint32_t event, ANT_MESSAGE* p_ant_message)
{
    uint8_t page_num;
    uint8_t command;

    switch (event)
    {
        case EVENT_RX:
            switch (p_ant_message->ANT_MESSAGE_ucMesgID)
            {
                case MESG_BROADCAST_DATA_ID:
                case MESG_ACKNOWLEDGED_DATA_ID:
                    page_num = p_ant_message->ANT_MESSAGE_aucPayload[0];
                    command = p_ant_message->ANT_MESSAGE_aucPayload[7];
                    if (page_num == COMMAND_ID && command == COMMAND_RESTART_BOOTLOADER)
                    {
                        restart_in_bootloader();
                    }
                    break;
            }
            break;
        default:
            break;
    }
}


/**@brief Function for stack interrupt handling.
 *
 * Implemented to clear the pending flag when receiving
 * an interrupt from the stack.
 */
void SD_EVT_IRQHandler(void)
{
   uint32_t ulEvent;

   while (sd_evt_get(&ulEvent) != NRF_ERROR_NOT_FOUND) // read out SOC events
   {
       ant_boot_settings_event(ulEvent);
   }
}

/**@brief Function for handling SoftDevice asserts.
 *
 */
void softdevice_assert_callback(uint32_t id, uint32_t pc, uint32_t info)
{
    for (;;)
    {
        // No implementation needed.
    }
}


/**@brief Function for application main entry. Does not return.
 */
int main(void)
{
    // ANT event message buffer.
    static ANT_MESSAGE ant_message_buffer;

    // Enable SoftDevice.
    uint32_t err_code;

    nrf_clock_lf_cfg_t clock_lf_cfg = NRF_CLOCK_LFCLKSRC;

    err_code = sd_softdevice_enable(&clock_lf_cfg,
                                    softdevice_assert_callback,
                                    ANT_LICENSE_KEY);
    APP_ERROR_CHECK(err_code);

    // Set application IRQ to lowest priority.
    err_code = sd_nvic_SetPriority(SD_EVT_IRQn, APP_IRQ_PRIORITY_LOWEST);
    APP_ERROR_CHECK(err_code);

    // Enable application IRQ (triggered from protocol).
    err_code = sd_nvic_EnableIRQ(SD_EVT_IRQn);
    APP_ERROR_CHECK(err_code);

    // Configure ant stack regards used channels.
    err_code = ant_stack_static_config();
    APP_ERROR_CHECK(err_code);

    // Setup channel
    ant_channel_tx_broadcast_setup();

    uint8_t event;
    uint8_t ant_channel;

    // Main loop.
    for (;;)
    {
        // Put CPU in sleep if possible.
        err_code = sd_app_evt_wait();
        APP_ERROR_CHECK(err_code);

        // Extract and process all pending ANT events as long as there are any left.
        do
        {
            // Fetch the event.
            err_code = sd_ant_event_get(&ant_channel, &event, ant_message_buffer.aucMessage);
            if (err_code == NRF_SUCCESS)
            {
                // Handle event
                channel_event_handle(event, &ant_message_buffer);
            }
        }
        while (err_code == NRF_SUCCESS);
    }
}

