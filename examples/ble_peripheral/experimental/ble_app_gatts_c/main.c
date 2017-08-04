/**
 * Copyright (c) 2017 - 2017, Nordic Semiconductor ASA
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

/** @brief GATT Service client example application main file.

    @details This file contains the main source code for a sample client application that uses the
             GATT Service. This client can be used to receive Service Changed indications. This is
             needed if your application interacts with GATT servers that modify, remove or add
             services. A typical course of action if a Service Changed indication is received is to
             rediscover the database on the peer device.

             For more information about the GATT Service, see "Defined Generic Attribute Profile Service"
             in Bluetooth Specification Version 5.0 Vol 3, Part G Section 7.
*/

#include "app_pm.h"
#include "app_adv.h"
#include <stdint.h>
#include <string.h>
#include "nordic_common.h"
#include "nrf.h"
#include "app_error.h"
#include "ble.h"
#include "ble_srv_common.h"
#include "ble_advdata.h"
#include "ble_advertising.h"
#include "nrf_sdh.h"
#include "nrf_sdh_soc.h"
#include "nrf_sdh_ble.h"
#include "app_timer.h"
#include "bsp.h"
#include "bsp_btn_ble.h"
#include "nrf_ble_gatts_c.h"
#include "boards.h"
#include "ble_gap.h"
#include "nrf_ble_gatt.h"
#include "peer_manager.h"
#include "fds.h"
#include "ble_conn_state.h"

#include "nrf_log.h"
#include "nrf_log_ctrl.h"


#define APP_BLE_OBSERVER_PRIO       1                                       /**< Application's BLE observer priority. You shouldn't need to modify this value. */
#define APP_BLE_CONN_CFG_TAG        1                                       /**< A tag identifying the SoftDevice BLE configuration. */
#define FEATURE_NOT_SUPPORTED       BLE_GATT_STATUS_ATTERR_APP_BEGIN + 2    /**< Reply when unsupported features are requested. */
#define DEAD_BEEF                   0xDEADBEEF                              /**< Value used as error code on stack dump, can be used to identify stack location on stack unwind. */

NRF_BLE_GATT_DEF(m_gatt);                                                   /**< GATT module instance. */
NRF_BLE_GATTS_C_DEF(m_gatts_c);                                             /**< GATT Service client instance. Handles Service Changed indications from the peer. */
BLE_DB_DISCOVERY_DEF(m_ble_db_discovery);                                   /**< DB discovery module instance. */

static bool m_erase_bonds;                                                  /**< Bool to determine if bonds should be erased before advertising starts. Based on button push upon startup. */


/**@brief Function for assert macro callback.

   @details This function will be called in case of an assert in the SoftDevice.

   @warning This handler is an example only and does not fit a final product. You need to analyze
            how your product is supposed to react in case of Assert.
   @warning On assert from the SoftDevice, the system can only recover on reset.

   @param[in] line_num    Line number of the failing ASSERT call.
   @param[in] p_file_name File name of the failing ASSERT call.
 */
void assert_nrf_callback(uint16_t line_num, const uint8_t * p_file_name)
{
    app_error_handler(DEAD_BEEF, line_num, p_file_name);
}


/**@brief Function for the LEDs initialization.

   @details Initializes all LEDs used by the application.
 */
static void leds_init(void)
{
    bsp_board_leds_init();
}


/**@brief Function for the Timer initialization.

   @details Initializes the timer module.
 */
static void timers_init(void)
{
    ret_code_t err_code;

    err_code = app_timer_init();
    APP_ERROR_CHECK(err_code);
}


/**@brief Function for the GAP initialization.

   @details This function sets up all the necessary GAP (Generic Access Profile) parameters of the
            device including the device name, appearance, and the preferred connection parameters.
 */
static void gap_params_init(void)
{
    ret_code_t              err_code;
    ble_gap_conn_params_t   gap_conn_params;
    ble_gap_conn_sec_mode_t sec_mode;

    BLE_GAP_CONN_SEC_MODE_SET_OPEN(&sec_mode);

    err_code = sd_ble_gap_device_name_set(&sec_mode,
                                          (const uint8_t *)DEVICE_NAME,
                                          strlen(DEVICE_NAME));
    APP_ERROR_CHECK(err_code);

    memset(&gap_conn_params, 0, sizeof(gap_conn_params));

    gap_conn_params.min_conn_interval = MSEC_TO_UNITS(MIN_CONN_INTERVAL_MSEC, UNIT_1_25_MS);
    gap_conn_params.max_conn_interval = MSEC_TO_UNITS(MAX_CONN_INTERVAL_MSEC, UNIT_1_25_MS);
    gap_conn_params.slave_latency     = SLAVE_LATENCY;
    gap_conn_params.conn_sup_timeout  = MSEC_TO_UNITS(CONN_SUP_TIMEOUT_MSEC, UNIT_10_MS);

    err_code = sd_ble_gap_ppcp_set(&gap_conn_params);
    APP_ERROR_CHECK(err_code);
}


/**@brief Function for initializing buttons and leds.

   @param[out] p_erase_bonds Boolean that will be set to true if the device is
                             woken up by the BSP_EVENT_CLEAR_BONDING_DATA event.
*/
static void buttons_leds_init(bool * p_erase_bonds)
{
    ret_code_t err_code;
    bsp_event_t startup_event;

    err_code = bsp_init(BSP_INIT_LED | BSP_INIT_BUTTONS, NULL);
    APP_ERROR_CHECK(err_code);

    err_code = bsp_btn_ble_init(NULL, &startup_event);
    APP_ERROR_CHECK(err_code);

    *p_erase_bonds = (startup_event == BSP_EVENT_CLEAR_BONDING_DATA);
}


/**@brief Function for handling events from the GATT Servive client module.

   @param[in] p_evt GATT Service event.
*/
static void gatts_evt_handler(nrf_ble_gatts_c_evt_t * p_evt)
{
    ret_code_t err_code;

    switch (p_evt->evt_type)
    {
      case NRF_BLE_GATTS_C_EVT_DISCOVERY_COMPLETE:
            NRF_LOG_INFO("GATT Service and Service Changed characteristic found on server.\r\n");

            err_code = nrf_ble_gatts_c_handles_assign(&m_gatts_c, p_evt->conn_handle, &p_evt->params.gatts_handles);
            APP_ERROR_CHECK(err_code);

            err_code = nrf_ble_gatts_c_enable_indication(&m_gatts_c, true);
            APP_ERROR_CHECK(err_code);
        break;

      case NRF_BLE_GATTS_C_EVT_DISCOVERY_FAILED:
            NRF_LOG_INFO("GATT Service or Service Changed characteristic not found on server.\r\n");
        break;

      case NRF_BLE_GATTS_C_EVT_DISCONN_COMPLETE:
            NRF_LOG_INFO("GATTS Service disconnected.\r\n");
          break;

      case NRF_BLE_GATTS_C_EVT_SRV_CHANGED:
          NRF_LOG_INFO("Service Changed on peer device.\r\n");
          NRF_LOG_INFO("Handle range start: %04x", p_evt->params.handle_range.start_handle);
          NRF_LOG_INFO("Handle range end: %04x", p_evt->params.handle_range.end_handle);
        break;

      default:
        break;
    }
}


/**@brief Function for handling database discovery events.

   @details This function is callback function to handle events from the database discovery module.
            Depending on the UUIDs that are discovered, this function should forward the events
            to their respective services.

   @param[in] p_event  Pointer to the database discovery event.
 */
static void db_disc_handler(ble_db_discovery_evt_t * p_evt)
{
    nrf_ble_gatts_c_on_db_disc_evt(&m_gatts_c, p_evt);
}


/**@brief Function for handling connection events.

   @param[in] p_ble_evt Bluetooth stack event.
*/
static void on_connect(ble_evt_t const * p_ble_evt)
{
    ret_code_t err_code;
    NRF_LOG_INFO("Connected.\r\n");
    err_code = nrf_ble_gatts_c_handles_assign(&m_gatts_c, p_ble_evt->evt.gap_evt.conn_handle, NULL);
    APP_ERROR_CHECK(err_code);

    err_code = ble_db_discovery_start(&m_ble_db_discovery, p_ble_evt->evt.gap_evt.conn_handle);
    APP_ERROR_CHECK(err_code);

    err_code = bsp_indication_set(BSP_INDICATE_CONNECTED);
    APP_ERROR_CHECK(err_code);
}


/**@brief Function for replying to autorized read/write events.

   @param[in] p_ble_evt Bluetooth stack event.
*/
static void on_rw_authorize_req(ble_evt_t const * p_ble_evt)
{
    ret_code_t                            err_code;
    ble_gatts_evt_rw_authorize_request_t  req;
    ble_gatts_rw_authorize_reply_params_t auth_reply;

    req = p_ble_evt->evt.gatts_evt.params.authorize_request;

    if (req.type != BLE_GATTS_AUTHORIZE_TYPE_INVALID)
    {
        if ((req.request.write.op == BLE_GATTS_OP_PREP_WRITE_REQ)     ||
            (req.request.write.op == BLE_GATTS_OP_EXEC_WRITE_REQ_NOW) ||
            (req.request.write.op == BLE_GATTS_OP_EXEC_WRITE_REQ_CANCEL))
        {
            if (req.type == BLE_GATTS_AUTHORIZE_TYPE_WRITE)
            {
                auth_reply.type = BLE_GATTS_AUTHORIZE_TYPE_WRITE;
            }
            else
            {
                auth_reply.type = BLE_GATTS_AUTHORIZE_TYPE_READ;
            }
            auth_reply.params.write.gatt_status = FEATURE_NOT_SUPPORTED;
            err_code = sd_ble_gatts_rw_authorize_reply(p_ble_evt->evt.gatts_evt.conn_handle,
                                                       &auth_reply);
            APP_ERROR_CHECK(err_code);
        }
    }
}

/**@brief Function for handling timeout BLE stack events.

   @param[in] p_ble_evt Bluetooth stack event.
*/
static void on_timeout(ble_evt_t const * p_ble_evt)
{
    ret_code_t err_code;

    switch (p_ble_evt->header.evt_id)
    {
        case BLE_GATTC_EVT_TIMEOUT:
            NRF_LOG_DEBUG("GATT Client Timeout.\r\n");
            err_code = sd_ble_gap_disconnect(p_ble_evt->evt.gattc_evt.conn_handle,
                                             BLE_HCI_REMOTE_USER_TERMINATED_CONNECTION);
            APP_ERROR_CHECK(err_code);
            break;

        case BLE_GATTS_EVT_TIMEOUT:
            NRF_LOG_DEBUG("GATT Server Timeout.\r\n");
            err_code = sd_ble_gap_disconnect(p_ble_evt->evt.gatts_evt.conn_handle,
                                             BLE_HCI_REMOTE_USER_TERMINATED_CONNECTION);
            APP_ERROR_CHECK(err_code);
            break;

        default:
            // No implementation needed.
            break;
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

    switch (p_ble_evt->header.evt_id)
    {
        case BLE_GAP_EVT_CONNECTED:
            on_connect(p_ble_evt);
            break;

        case BLE_GAP_EVT_DISCONNECTED:
            NRF_LOG_INFO("Disconnected. \r\n");
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

        case BLE_GATTS_EVT_SYS_ATTR_MISSING:
            // No system attributes have been stored.
            err_code = sd_ble_gatts_sys_attr_set(p_ble_evt->evt.gatts_evt.conn_handle, NULL, 0, 0);
            APP_ERROR_CHECK(err_code);
            break;

        case BLE_GATTC_EVT_TIMEOUT:
            //fall-through.
        case BLE_GATTS_EVT_TIMEOUT:
            on_timeout(p_ble_evt);
            break;

        case BLE_EVT_USER_MEM_REQUEST:
            err_code = sd_ble_user_mem_reply(p_ble_evt->evt.gattc_evt.conn_handle, NULL);
            APP_ERROR_CHECK(err_code);
            break;

        case BLE_GATTS_EVT_RW_AUTHORIZE_REQUEST:
            on_rw_authorize_req(p_ble_evt);
            break;

        default:
            // No implementation needed.
            break;
    }
}


/**@brief Function for initializing the BLE stack.

 @details Initializes the SoftDevice and the BLE event interrupt.
*/
static void ble_stack_init(void)
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

    // Register handler for BLE events.
    NRF_SDH_BLE_OBSERVER(m_ble_observer, APP_BLE_OBSERVER_PRIO, ble_evt_handler, NULL);
}


/** @brief Database discovery initialization.
*/
static void db_discovery_init(void)
{
    ret_code_t err_code = ble_db_discovery_init(db_disc_handler);
    APP_ERROR_CHECK(err_code);
}


/**@brief Function for initializing the nrf log module.
 */
static void log_init(void)
{
    ret_code_t err_code = NRF_LOG_INIT(NULL);
    APP_ERROR_CHECK(err_code);
}


/**@brief Function for handling the idle state (main loop). If there is no pending log operation,
          then sleep until next the next event occurs.
 */
static void idle_state_handle(void)
{
    if (NRF_LOG_PROCESS() == false)
    {
        uint32_t err_code = sd_app_evt_wait();
        APP_ERROR_CHECK(err_code);
    }
}


/**@brief Function for initializing the GATT module.
 */
static void gatt_init(void)
{
    ret_code_t err_code = nrf_ble_gatt_init(&m_gatt, NULL);
    APP_ERROR_CHECK(err_code);
}


/**@brief Function for initializing services that will be used by the application.
 */
static void services_init(void)
{
    ret_code_t             err_code;
    nrf_ble_gatts_c_init_t init;

    init.evt_handler = gatts_evt_handler;

    err_code = nrf_ble_gatts_c_init(&m_gatts_c, &init);
    APP_ERROR_CHECK(err_code);
}


/**@brief Function for initializing all the modules used in this example application.
 */
static void modules_init(void)
{
    log_init();
    leds_init();
    timers_init();
    buttons_leds_init(&m_erase_bonds);
    ble_stack_init();
    gap_params_init();
    gatt_init();
    db_discovery_init();
    services_init();
    advertising_init();
    peer_manager_init();
}


/**@brief Function for application main entry.
 */
int main(void)
{
    // Initialize.
    modules_init();

    // Start execution.
    NRF_LOG_INFO("GATT Service client started.\r\n");
    advertising_start(m_erase_bonds);

    // Enter main loop.
    for (;;)
    {
        idle_state_handle();
    }
}


/**
 * @}
 */
