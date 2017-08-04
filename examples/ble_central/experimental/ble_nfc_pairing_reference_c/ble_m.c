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
#include "ble_m.h"
#include "nordic_common.h"
#include "nrf_sdh.h"
#include "nrf_sdh_ble.h"
#include "nrf_sdh_soc.h"
#include "peer_manager.h"
#include "nfc_pair_m.h"
#include "fds.h"
#include "nrf_fstorage.h"
#include "nfc_ble_oob_advdata_parser.h"
#include "nrf_ble_gatt.h"

#define NRF_LOG_MODULE_NAME BLE_M
#include "nrf_log.h"
NRF_LOG_MODULE_REGISTER();

#define MIN_CONNECTION_INTERVAL     MSEC_TO_UNITS(7.5, UNIT_1_25_MS)            /**< Determines minimum connection interval in milliseconds. */
#define MAX_CONNECTION_INTERVAL     MSEC_TO_UNITS(30, UNIT_1_25_MS)             /**< Determines maximum connection interval in milliseconds. */
#define SLAVE_LATENCY               0                                           /**< Determines slave latency in terms of connection events. */
#define SUPERVISION_TIMEOUT         MSEC_TO_UNITS(4000, UNIT_10_MS)             /**< Determines supervision time-out in units of 10 milliseconds. */

#define SCAN_INTERVAL               0x00A0                                      /**< Determines scan interval in units of 0.625 millisecond. */
#define SCAN_WINDOW                 0x0050                                      /**< Determines scan window in units of 0.625 millisecond. */
#define SCAN_TIMEOUT                0x0000                                      /**< Timout when scanning. 0x0000 disables timeout. */

#define APP_BLE_OBSERVER_PRIO       1                                           /**< Application's BLE observer priority. You shouldn't need to modify this value. */
#define APP_SOC_OBSERVER_PRIO       1                                           /**< Applications' SoC observer priority. You shoulnd't need to modify this value. */
#define APP_BLE_CONN_CFG_TAG        1                                           /**< A tag identifying the SoftDevice BLE configuration. */


NRF_BLE_GATT_DEF(m_gatt);                                                       /**< GATT module instance. */

static bool     m_is_connected              = false;                            /**< Flag to keep track of BLE connections with peripheral devices */
static uint16_t m_conn_handle               = BLE_CONN_HANDLE_INVALID;          /**< Current connection handle. */
static bool     m_memory_access_in_progress = false;                            /**< Flag to keep track of ongoing operations on persistent memory. */

/**@brief Connection parameters requested for connection. */
static ble_gap_conn_params_t const m_connection_param =
{
    .min_conn_interval = (uint16_t)MIN_CONNECTION_INTERVAL,
    .max_conn_interval = (uint16_t)MAX_CONNECTION_INTERVAL,
    .slave_latency     = (uint16_t)SLAVE_LATENCY,
    .conn_sup_timeout  = (uint16_t)SUPERVISION_TIMEOUT
};

/**@brief Parameters used when scanning. */
ble_gap_scan_params_t const m_scan_params =
{
    .active   = 1,
    .interval = SCAN_INTERVAL,
    .window   = SCAN_WINDOW,
    .timeout  = SCAN_TIMEOUT,
    #if (NRF_SD_BLE_API_VERSION == 2)
        .selective   = 0,
        .p_whitelist = NULL,
    #endif
    #if (NRF_SD_BLE_API_VERSION == 3)
        .use_whitelist = 0,
    #endif
};


void ble_disconnect(void)
{
    ret_code_t err_code;

    if (m_is_connected)
    {
        err_code = sd_ble_gap_disconnect(m_conn_handle, BLE_HCI_REMOTE_USER_TERMINATED_CONNECTION);
        APP_ERROR_CHECK(err_code);
    }
}


bool ble_is_connected(void)
{
    return m_is_connected;
}


uint16_t ble_get_conn_handle(void)
{
    return m_conn_handle;
}


void scan_start(void)
{
    ret_code_t err_code;

    // If there is any pending write to flash, defer scanning until it completes.
    if (nrf_fstorage_is_busy(NULL))
    {
        m_memory_access_in_progress = true;
        return;
    }

    (void) sd_ble_gap_scan_stop();

    err_code = sd_ble_gap_scan_start(&m_scan_params);
    APP_ERROR_CHECK(err_code);
}


/**@brief Function for handling the advertising report BLE event.
 *
 * @param[in] p_ble_evt  Bluetooth stack event.
 */
static void on_adv_report(ble_evt_t const * const p_ble_evt)
{
    ret_code_t  err_code;
    uint8_t   * p_adv_data;
    uint8_t     data_len;
    uint8_t   * dev_name_ptr;
    char        dev_name[32];

    // For readibility.
    ble_gap_evt_t const * const  p_gap_evt  = &p_ble_evt->evt.gap_evt;
    ble_gap_addr_t const * const peer_addr  = &p_gap_evt->params.adv_report.peer_addr;

    // Initialize advertisement report for parsing.
    p_adv_data = (uint8_t *)p_gap_evt->params.adv_report.data;
    data_len   = p_gap_evt->params.adv_report.dlen;

    // Search for advertising names.
    err_code = nfc_ble_oob_advdata_parser_field_find(BLE_GAP_AD_TYPE_COMPLETE_LOCAL_NAME,
                                                     p_adv_data,
                                                     &data_len,
                                                     &dev_name_ptr);
    if (err_code != NRF_SUCCESS)
    {
        // Look for the short local name if it was not found as complete.
        err_code = nfc_ble_oob_advdata_parser_field_find(BLE_GAP_AD_TYPE_SHORT_LOCAL_NAME,
                                                         p_adv_data,
                                                         &data_len,
                                                         &dev_name_ptr);
        if (err_code != NRF_SUCCESS)
        {
            // If we can't parse the data, then exit.
            return;
        }
    }

    memcpy(dev_name, dev_name_ptr, data_len);
    dev_name[data_len] = 0;

    NRF_LOG_DEBUG("Found advertising device: %s", nrf_log_push((char *)dev_name));

    // Check if device address is the same as address taken from the NFC tag.
    if (nfc_oob_pairing_tag_match(peer_addr))
    {
        // If address is correct, stop scanning and initiate connection with peripheral device.
        err_code = sd_ble_gap_scan_stop();
        APP_ERROR_CHECK(err_code);

        err_code = sd_ble_gap_connect(peer_addr, &m_scan_params,
                                      &m_connection_param, APP_BLE_CONN_CFG_TAG);
        APP_ERROR_CHECK(err_code);
    }
}


/**@brief Function for handling BLE events.
 *
 * @param[in]   p_ble_evt   Bluetooth stack event.
 * @param[in]   p_context   Unused.
 */
static void ble_evt_handler(ble_evt_t const * p_ble_evt, void * p_context)
{
    ret_code_t err_code;
    ble_gap_evt_t const * p_gap_evt = &p_ble_evt->evt.gap_evt;

    switch (p_ble_evt->header.evt_id)
    {
        // Upon connection, initiate secure bonding.
        case BLE_GAP_EVT_CONNECTED:
            NRF_LOG_INFO("Connected.");
            err_code = pm_conn_secure(p_ble_evt->evt.gap_evt.conn_handle, false);
            APP_ERROR_CHECK(err_code);
            m_is_connected = true;
            m_conn_handle  = p_ble_evt->evt.gap_evt.conn_handle;
            break;

        // Upon disconnection, reset the connection handle of the peer which disconnected
        // and invalidate data taken from the NFC tag.
        case BLE_GAP_EVT_DISCONNECTED:
            NRF_LOG_INFO("Disconnected.");
            m_conn_handle  = BLE_CONN_HANDLE_INVALID;
            m_is_connected = false;
            nfc_oob_pairing_tag_invalidate();
            break;

        case BLE_GAP_EVT_ADV_REPORT:
            on_adv_report(p_ble_evt);
            break;

        case BLE_GAP_EVT_TIMEOUT:
            // We have not specified a timeout for scanning, so only connection attemps can timeout.
            if (p_gap_evt->params.timeout.src == BLE_GAP_TIMEOUT_SRC_CONN)
            {
                NRF_LOG_INFO("[APP]: Connection Request timed out.");
            }
            break;

        case BLE_GAP_EVT_CONN_PARAM_UPDATE_REQUEST:
            // Accept parameters requested by peer.
            err_code = sd_ble_gap_conn_param_update(p_gap_evt->conn_handle,
                                        &p_gap_evt->params.conn_param_update_request.conn_params);
            APP_ERROR_CHECK(err_code);
            break;

#if defined(S132)
        case BLE_GAP_EVT_PHY_UPDATE_REQUEST:
        {
            NRF_LOG_DEBUG("PHY update request.");
            ble_gap_phys_t const phys =
            {
                .rx_phys = BLE_GAP_PHY_AUTO,
                .tx_phys = BLE_GAP_PHY_AUTO,
            };
            err_code = sd_ble_gap_phy_update(p_ble_evt->evt.gap_evt.conn_handle, &phys);
            APP_ERROR_CHECK(err_code);
        } break;
#endif

        case BLE_GATTC_EVT_TIMEOUT:
            // Disconnect on GATT Client timeout event.
            NRF_LOG_DEBUG("GATT Client Timeout.");
            err_code = sd_ble_gap_disconnect(p_ble_evt->evt.gattc_evt.conn_handle,
                                             BLE_HCI_REMOTE_USER_TERMINATED_CONNECTION);
            APP_ERROR_CHECK(err_code);
            break;

        case BLE_GATTS_EVT_TIMEOUT:
            // Disconnect on GATT Server timeout event.
            NRF_LOG_DEBUG("GATT Server Timeout.");
            err_code = sd_ble_gap_disconnect(p_ble_evt->evt.gatts_evt.conn_handle,
                                             BLE_HCI_REMOTE_USER_TERMINATED_CONNECTION);
            APP_ERROR_CHECK(err_code);
            break;

        case BLE_GAP_EVT_AUTH_STATUS:
            NRF_LOG_INFO("BLE_GAP_EVT_AUTH_STATUS");
            if (p_ble_evt->evt.gap_evt.params.auth_status.auth_status == BLE_GAP_SEC_STATUS_SUCCESS)
            {
                NRF_LOG_INFO("Authorization succeeded!");
            }
            else
            {
                NRF_LOG_INFO("Authorization failed with code: %u!", p_ble_evt->evt.gap_evt.params.auth_status.auth_status);
            }

            break;

        case BLE_GAP_EVT_CONN_SEC_UPDATE:
            NRF_LOG_INFO("BLE_GAP_EVT_CONN_SEC_UPDATE");
            NRF_LOG_INFO("Security mode: %u", p_ble_evt->evt.gap_evt.params.conn_sec_update.conn_sec.sec_mode.lv);
            break;

        default:
            // No implementation needed.
            break;
    }
}


/**@brief Function for handling the Application's system events.
 *
 * @param[in]   sys_evt   system event.
 */
static void soc_evt_handler(uint32_t sys_evt, void * p_context)
{
    switch (sys_evt)
    {
        case NRF_EVT_FLASH_OPERATION_SUCCESS:
            /* fall through */
        case NRF_EVT_FLASH_OPERATION_ERROR:

            if (m_memory_access_in_progress)
            {
                m_memory_access_in_progress = false;
                scan_start();
            }
            break;

        default:
            // No implementation needed.
            break;
    }
}


void gatt_init(void)
{
    ret_code_t err_code = nrf_ble_gatt_init(&m_gatt, NULL);
    APP_ERROR_CHECK(err_code);
}


/**@brief Function for initializing the BLE stack.
 *
 * @details Initializes the SoftDevice and the BLE event interrupt.
 */
void ble_stack_init(void)
{
    ret_code_t err_code;

    err_code = nrf_sdh_enable_request();
    APP_ERROR_CHECK(err_code);

    // Configure the BLE stack using the default settings.
    // Fetch the start address of the application RAM.
    uint32_t ram_start = 0;
    err_code = nrf_sdh_ble_default_cfg_set(APP_BLE_CONN_CFG_TAG, &ram_start);
    APP_ERROR_CHECK(err_code);

    // Enable BLE stack.
    err_code = nrf_sdh_ble_enable(&ram_start);
    APP_ERROR_CHECK(err_code);

    // Register handlers for BLE and SoC events.
    NRF_SDH_BLE_OBSERVER(m_ble_observer, APP_BLE_OBSERVER_PRIO, ble_evt_handler, NULL);
    NRF_SDH_SOC_OBSERVER(m_soc_observer, APP_SOC_OBSERVER_PRIO, soc_evt_handler, NULL);
}
