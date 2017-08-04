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

/**
 * @brief BLE Object Transfer Service client application main file.
 *
 * This file contains the source code for a sample Object Transfer Service client application.
 */

#include <stdint.h>
#include <stdio.h>
#include <string.h>
#include "nordic_common.h"
#include "nrf_sdm.h"
#include "ble.h"
#include "ble_hci.h"
#include "ble_db_discovery.h"
#include "app_util.h"
#include "app_error.h"
#include "boards.h"
#include "nrf_gpio.h"
#include "nrf_ble_ots_c.h"
#include "nrf_ble_ots_c_oacp.h"
#include "nrf_ble_ots_c_l2cap.h"
#include "app_util.h"
#include "app_timer.h"
#include "bsp.h"
#include "bsp_btn_ble.h"
#include "peer_manager.h"
#include "fds.h"
#include "nrf_fstorage.h"
#include "ble_conn_state.h"
#include "nrf_ble_gatt.h"
#include "nrf_sdh.h"
#include "nrf_sdh_ble.h"
#include "nrf_sdh_soc.h"

#include "nrf_log.h"
#include "nrf_log_ctrl.h"
#include "nrf_log_default_backends.h"


#define APP_BLE_OBSERVER_PRIO     1                                     /**< Application's BLE observer priority. You shoulnd't need to modify this value. */
#define APP_SOC_OBSERVER_PRIO     1                                     /**< Applications' SoC observer priority. You shoulnd't need to modify this value. */

#define SEC_PARAM_BOND            1                                     /**< Perform bonding. */
#define SEC_PARAM_MITM            0                                     /**< Man In The Middle protection not required. */
#define SEC_PARAM_LESC            0                                     /**< LE Secure Connections not enabled. */
#define SEC_PARAM_KEYPRESS        0                                     /**< Keypress notifications not enabled. */
#define SEC_PARAM_IO_CAPABILITIES BLE_GAP_IO_CAPS_NONE                  /**< No I/O capabilities. */
#define SEC_PARAM_OOB             0                                     /**< Out Of Band data not available. */
#define SEC_PARAM_MIN_KEY_SIZE    7                                     /**< Minimum encryption key size. */
#define SEC_PARAM_MAX_KEY_SIZE    16                                    /**< Maximum encryption key size. */

#define SCAN_INTERVAL             0x00A0                                /**< Determines scan interval in units of 0.625 millisecond. */
#define SCAN_WINDOW               0x0050                                /**< Determines scan window in units of 0.625 millisecond. */

#define MIN_CONNECTION_INTERVAL   MSEC_TO_UNITS(7.5, UNIT_1_25_MS)      /**< Determines minimum connection interval in millisecond. */
#define MAX_CONNECTION_INTERVAL   MSEC_TO_UNITS(30, UNIT_1_25_MS)       /**< Determines maximum connection interval in millisecond. */
#define SLAVE_LATENCY             0                                     /**< Determines slave latency in counts of connection events. */
#define SUPERVISION_TIMEOUT       MSEC_TO_UNITS(4000, UNIT_10_MS)       /**< Determines supervision time-out in units of 10 millisecond. */

#define TARGET_UUID               BLE_UUID_OTS_SERVICE                  /**< Target device name that application is looking for. */
#define UUID16_SIZE               2                                     /**< Size of 16 bit UUID */

#define APP_CONN_CFG_TAG          1

#define BLE_OTS_PSM               0x0025                                 /**< PSM used for L2CAP CoC for Object Transfer. This value is assigned by Bluetooth SIG for Object Transfer Specication.*/
#define OBJECT_SIZE               100                                    /**< Size of the local object. */

#define L2CAP_RX_MPS              60                                     /**< Size of L2CAP Rx MPS (must be at least BLE_L2CAP_MPS_MIN).*/
#define L2CAP_TX_MPS              40                                     /**< Size of L2CAP Tx MPS (must be at least BLE_L2CAP_MPS_MIN).*/
#define L2CAP_RX_MTU              30                                     /**< Rx L2CAP MTU size (must be at least BLE_L2CAP_MTU_MIN).*/

/**@breif Macro to unpack 16bit unsigned UUID from octet stream. */
#define UUID16_EXTRACT(DST, SRC) \
    do                           \
    {                            \
        (*(DST))   = (SRC)[1];   \
        (*(DST)) <<= 8;          \
        (*(DST))  |= (SRC)[0];   \
    } while (0)


/**@brief Variable length data encapsulation in terms of length and pointer to data */
typedef struct
{
    uint8_t * p_data;   /**< Pointer to data. */
    uint16_t  data_len; /**< Length of data. */
} data_t;


NRF_BLE_GATT_DEF(m_gatt);                           /**< GATT module instance. */
BLE_DB_DISCOVERY_DEF(m_db_disc);                    /**< DB Discovery module instance. */
NRF_BLE_OTS_C_DEF(m_ots_c);                         /**< Object transfer service client instance. */

//An empty buffer for the data of an object, we can receive the object on the peer side into this buffer.
static uint8_t m_object_rx[OBJECT_SIZE] = {0};

static uint16_t              m_conn_handle;                 /**< Current connection handle. */
static bool                  m_whitelist_disabled;          /**< True if whitelist has been temporarily disabled. */
static bool                  m_memory_access_in_progress;   /**< Flag to keep track of ongoing operations on persistent memory. */
static ble_gap_scan_params_t m_scan_param;                  /**< Scan parameters requested for scanning and connection. */

static ble_data_t m_sdu_buf;
static uint8_t    m_object[OBJECT_SIZE] =
{
    0x61, 0x62, 0x63, 0x64, 0x65, 0x66, 0x67, 0x68, 0x69, 0x6a, 0x6b, 0x6c, 0x6d, 0x6e, 0x6f,
    0x70, 0x71, 0x72, 0x73, 0x74, 0x75, 0x76, 0x77, 0x78, 0x79, 0x7a, 0x31, 0x32, 0x33, 0x34
};

/**@brief String literals for the OACP result codes. Used for logging output. */
static const char * lit_request[] =
{
    "RFU",
    "Create",
    "Delete",
    "Calculate Checksum",
    "Execute",
    "Read",
    "Write",
    "Abort",
};

/**@brief String literals for the OACP result codes. Used for logging output. */
static const char * lit_result[] =
{
    "RFU",
    "Success",
    "Op Code Not Supported",
    "Invalid Parameter",
    "Insufficient Resources",
    "Invalid Object",
    "Channel Unavailable",
    "Unsuported Type",
    "Procedure Not Permitted",
    "Object Locked",
    "Operation Failed"
};

/**
 * @brief Connection parameters requested for connection.
 */
static ble_gap_conn_params_t const m_connection_param =
{
    (uint16_t)MIN_CONNECTION_INTERVAL, // Minimum connection.
    (uint16_t)MAX_CONNECTION_INTERVAL, // Maximum connection.
    (uint16_t)SLAVE_LATENCY,           // Slave latency.
    (uint16_t)SUPERVISION_TIMEOUT      // Supervision time-out.
};

static void scan_start(void);


/**@brief Function for asserts in the SoftDevice.
 *
 * @details This function will be called in case of an assert in the SoftDevice.
 *
 * @warning This handler is an example only and does not fit a final product. You need to analyze
 *          how your product is supposed to react in case of Assert.
 * @warning On assert from the SoftDevice, the system can only recover on reset.
 *
 * @param[in] line_num     Line number of the failing ASSERT call.
 * @param[in] p_file_name  File name of the failing ASSERT call.
 */
void assert_nrf_callback(uint16_t line_num, const uint8_t * p_file_name)
{
    app_error_handler(0xDEADBEEF, line_num, p_file_name);
}


/**@brief Function for handling database discovery events.
 *
 * @details This function is callback function to handle events from the database discovery module.
 *          Depending on the UUIDs that are discovered, this function should forward the events
 *          to their respective services.
 *
 * @param[in] p_event  Pointer to the database discovery event.
 *
 */
static void db_disc_handler(ble_db_discovery_evt_t * p_evt)
{
    nrf_ble_ots_c_on_db_disc_evt(&m_ots_c, p_evt);
}


/**@brief Function for handling Peer Manager events.
 *
 * @param[in] p_evt  Peer Manager event.
 */
static void pm_evt_handler(pm_evt_t const * p_evt)
{
    ret_code_t err_code;

    switch (p_evt->evt_id)
    {
        case PM_EVT_BONDED_PEER_CONNECTED:
        {
            NRF_LOG_INFO("Connected to a previously bonded device.");
        } break;

        case PM_EVT_CONN_SEC_SUCCEEDED:
        {
            NRF_LOG_INFO("Connection secured: role: %d, conn_handle: 0x%x, procedure: %d.",
                         ble_conn_state_role(p_evt->conn_handle),
                         p_evt->conn_handle,
                         p_evt->params.conn_sec_succeeded.procedure);
        } break;

        case PM_EVT_CONN_SEC_FAILED:
        {
            /* Often, when securing fails, it shouldn't be restarted, for security reasons.
             * Other times, it can be restarted directly.
             * Sometimes it can be restarted, but only after changing some Security Parameters.
             * Sometimes, it cannot be restarted until the link is disconnected and reconnected.
             * Sometimes it is impossible, to secure the link, or the peer device does not support it.
             * How to handle this error is highly application dependent. */
        } break;

        case PM_EVT_CONN_SEC_CONFIG_REQ:
        {
            // Reject pairing request from an already bonded peer.
            pm_conn_sec_config_t conn_sec_config = {.allow_repairing = false};
            pm_conn_sec_config_reply(p_evt->conn_handle, &conn_sec_config);
        } break;

        case PM_EVT_STORAGE_FULL:
        {
            // Run garbage collection on the flash.
            err_code = fds_gc();
            if (err_code == FDS_ERR_BUSY || err_code == FDS_ERR_NO_SPACE_IN_QUEUES)
            {
                // Retry.
            }
            else
            {
                APP_ERROR_CHECK(err_code);
            }
        } break;

        case PM_EVT_PEERS_DELETE_SUCCEEDED:
        {
            scan_start();
        } break;

        case PM_EVT_LOCAL_DB_CACHE_APPLY_FAILED:
        {
            // The local database has likely changed, send service changed indications.
            pm_local_database_has_changed();
        } break;

        case PM_EVT_PEER_DATA_UPDATE_FAILED:
        {
            // Assert.
            APP_ERROR_CHECK(p_evt->params.peer_data_update_failed.error);
        } break;

        case PM_EVT_PEER_DELETE_FAILED:
        {
            // Assert.
            APP_ERROR_CHECK(p_evt->params.peer_delete_failed.error);
        } break;

        case PM_EVT_PEERS_DELETE_FAILED:
        {
            // Assert.
            APP_ERROR_CHECK(p_evt->params.peers_delete_failed_evt.error);
        } break;

        case PM_EVT_ERROR_UNEXPECTED:
        {
            // Assert.
            APP_ERROR_CHECK(p_evt->params.error_unexpected.error);
        } break;

        case PM_EVT_CONN_SEC_START:
        case PM_EVT_PEER_DATA_UPDATE_SUCCEEDED:
        case PM_EVT_PEER_DELETE_SUCCEEDED:
        case PM_EVT_LOCAL_DB_CACHE_APPLIED:
        case PM_EVT_SERVICE_CHANGED_IND_SENT:
        case PM_EVT_SERVICE_CHANGED_IND_CONFIRMED:
        default:
            break;
    }
}


/**
 * @brief Parses advertisement data, providing length and location of the field in case
 *        matching data is found.
 *
 * @param[in]  Type of data to be looked for in advertisement data.
 * @param[in]  Advertisement report length and pointer to report.
 * @param[out] If data type requested is found in the data report, type data length and
 *             pointer to data will be populated here.
 *
 * @retval NRF_SUCCESS if the data type is found in the report.
 * @retval NRF_ERROR_NOT_FOUND if the data type could not be found.
 */
static uint32_t adv_report_parse(uint8_t type, data_t * p_advdata, data_t * p_typedata)
{
    uint32_t  index = 0;
    uint8_t * p_data;

    p_data = p_advdata->p_data;

    while (index < p_advdata->data_len)
    {
        uint8_t field_length = p_data[index];
        uint8_t field_type   = p_data[index + 1];

        if (field_type == type)
        {
            p_typedata->p_data   = &p_data[index + 2];
            p_typedata->data_len = field_length - 1;
            return NRF_SUCCESS;
        }
        index += field_length + 1;
    }
    return NRF_ERROR_NOT_FOUND;
}


/**@brief Function for putting the chip into sleep mode.
 *
 * @note This function will not return.
 */
static void sleep_mode_enter(void)
{
    ret_code_t err_code;

    err_code = bsp_indication_set(BSP_INDICATE_IDLE);
    APP_ERROR_CHECK(err_code);

    // Prepare wakeup buttons.
    err_code = bsp_btn_ble_sleep_mode_prepare();
    APP_ERROR_CHECK(err_code);

    // Go to system-off mode (this function will not return; wakeup will cause a reset).
    err_code = sd_power_system_off();
    APP_ERROR_CHECK(err_code);
}


/**@brief Function for handling BLE events.
 *
 * @param[in]   p_ble_evt   Bluetooth stack event.
 * @param[in]   p_context   Unused.
 */
static void ble_evt_handler(ble_evt_t const * p_ble_evt, void * p_context)
{
    ret_code_t            err_code;
    ble_gap_evt_t const * p_gap_evt = &p_ble_evt->evt.gap_evt;

    switch (p_ble_evt->header.evt_id)
    {
        case BLE_GAP_EVT_CONNECTED:
        {
            NRF_LOG_INFO("Peer Connected. ");
            // Discover peer's services.
            memset(&m_db_disc, 0x00, sizeof(ble_db_discovery_t));
            err_code = ble_db_discovery_start(&m_db_disc,
                                              p_ble_evt->evt.gap_evt.conn_handle);
            APP_ERROR_CHECK(err_code);

            err_code = bsp_indication_set(BSP_INDICATE_CONNECTED);
            APP_ERROR_CHECK(err_code);

        } break;

        case BLE_GAP_EVT_ADV_REPORT:
        {
            data_t adv_data;
            data_t type_data;

            // Initialize advertisement report for parsing.
            adv_data.p_data   = (uint8_t *)p_gap_evt->params.adv_report.data;
            adv_data.data_len = p_gap_evt->params.adv_report.dlen;

            err_code = adv_report_parse(BLE_GAP_AD_TYPE_16BIT_SERVICE_UUID_MORE_AVAILABLE,
                                        &adv_data,
                                        &type_data);

            if (err_code != NRF_SUCCESS)
            {
                // Compare short local name in case complete name does not match.
                err_code = adv_report_parse(BLE_GAP_AD_TYPE_16BIT_SERVICE_UUID_COMPLETE,
                                            &adv_data,
                                            &type_data);
            }

            // Verify if short or complete name matches target.
            if (err_code == NRF_SUCCESS)
            {
                uint16_t extracted_uuid;

                // UUIDs found, look for matching UUID
                for (uint32_t u_index = 0; u_index < (type_data.data_len / UUID16_SIZE); u_index++)
                {
                    UUID16_EXTRACT(&extracted_uuid, &type_data.p_data[u_index * UUID16_SIZE]);

                    if (extracted_uuid == TARGET_UUID)
                    {
                        // Stop scanning.
                        (void) sd_ble_gap_scan_stop();

                        err_code = bsp_indication_set(BSP_INDICATE_IDLE);
                        APP_ERROR_CHECK(err_code);

                        // Initiate connection.
                        #if (NRF_SD_BLE_API_VERSION <= 2)
                            m_scan_param.selective = 0;
                        #endif
                        #if (NRF_SD_BLE_API_VERSION >= 3)
                            m_scan_param.use_whitelist = 0;
                        #endif

                        err_code = sd_ble_gap_connect(&p_gap_evt->params.adv_report.peer_addr,
                                                      &m_scan_param,
                                                      &m_connection_param,
                                                      APP_CONN_CFG_TAG);

                        m_whitelist_disabled = false;

                        if (err_code != NRF_SUCCESS)
                        {
                            NRF_LOG_DEBUG("Connection Request Failed, reason 0x%x", err_code);
                        }
                        break;
                    }
                }
            }
        } break; // BLE_GAP_EVT_ADV_REPORT

        case BLE_GAP_EVT_TIMEOUT:
        {
            if (p_gap_evt->params.timeout.src == BLE_GAP_TIMEOUT_SRC_SCAN)
            {
                NRF_LOG_INFO("Scan timed out.");
                scan_start();
            }
            else if (p_gap_evt->params.timeout.src == BLE_GAP_TIMEOUT_SRC_CONN)
            {
                NRF_LOG_DEBUG("Connection Request timed out.");
            }
        } break;

        case BLE_GAP_EVT_CONN_PARAM_UPDATE_REQUEST:
        {
            // Accepting parameters requested by peer.
            err_code = sd_ble_gap_conn_param_update(p_gap_evt->conn_handle,
                                                    &p_gap_evt->params.conn_param_update_request.conn_params);
            APP_ERROR_CHECK(err_code);
        } break;

        case BLE_GAP_EVT_DISCONNECTED:
        {
            NRF_LOG_INFO("Peer Disconnected.");
            if (ble_conn_state_n_centrals() < NRF_SDH_BLE_CENTRAL_LINK_COUNT)
            {
                scan_start();
            }
        } break;

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
        {
            // Disconnect on GATT Client timeout event.
            NRF_LOG_DEBUG("GATT Client Timeout.");
            err_code = sd_ble_gap_disconnect(p_ble_evt->evt.gattc_evt.conn_handle,
                                             BLE_HCI_REMOTE_USER_TERMINATED_CONNECTION);
            APP_ERROR_CHECK(err_code);
        } break;

        case BLE_GATTS_EVT_TIMEOUT:
        {
            // Disconnect on GATT Server timeout event.
            NRF_LOG_DEBUG("GATT Server Timeout.");
            err_code = sd_ble_gap_disconnect(p_ble_evt->evt.gatts_evt.conn_handle,
                                             BLE_HCI_REMOTE_USER_TERMINATED_CONNECTION);
            APP_ERROR_CHECK(err_code);
        } break;

        default:
            break;
    }
}


/**@brief Function for handling the Application's system events.
 *
 * @param[in] sys_evt   system event.
 * @param[in] p_context Unused.
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


/**@brief Function for initializing the BLE stack.
 *
 * @details Initializes the SoftDevice and the BLE event interrupt.
 */
static void ble_stack_init(void)
{
    ret_code_t err_code;

    err_code = nrf_sdh_enable_request();
    APP_ERROR_CHECK(err_code);

    uint32_t ram_start = 0;
    err_code = nrf_sdh_ble_default_cfg_set(APP_CONN_CFG_TAG, &ram_start);
    APP_ERROR_CHECK(err_code);

    // Overwrite some of the default configurations for the BLE stack.
    ble_cfg_t ble_cfg;

    // Set l2cap channel configuration
    ble_cfg.conn_cfg.conn_cfg_tag                        = APP_CONN_CFG_TAG;
    ble_cfg.conn_cfg.params.l2cap_conn_cfg.rx_mps        = L2CAP_RX_MPS;
    ble_cfg.conn_cfg.params.l2cap_conn_cfg.rx_queue_size = 1;
    ble_cfg.conn_cfg.params.l2cap_conn_cfg.tx_mps        = L2CAP_TX_MPS;
    ble_cfg.conn_cfg.params.l2cap_conn_cfg.tx_queue_size = 1;
    ble_cfg.conn_cfg.params.l2cap_conn_cfg.ch_count      = 1;

    err_code = sd_ble_cfg_set(BLE_CONN_CFG_L2CAP, &ble_cfg, ram_start);
    APP_ERROR_CHECK(err_code);

    // Enable BLE stack.
    err_code = nrf_sdh_ble_enable(&ram_start);
    APP_ERROR_CHECK(err_code);

    // Register handlers for BLE and SoC events.
    NRF_SDH_BLE_OBSERVER(m_ble_observer, APP_BLE_OBSERVER_PRIO, ble_evt_handler, NULL);
    NRF_SDH_SOC_OBSERVER(m_soc_observer, APP_SOC_OBSERVER_PRIO, soc_evt_handler, NULL);
}


/**@brief Function for disabling the use of whitelist for scanning.
 */
static void whitelist_disable(void)
{
    if (!m_whitelist_disabled)
    {
        NRF_LOG_INFO("Whitelist temporarily disabled.");
        m_whitelist_disabled = true;
        (void) sd_ble_gap_scan_stop();
        scan_start();
    }
}


/**@brief Function for handling events from the BSP module.
 *
 * @param[in]   event   Event generated by button press.
 */
void bsp_event_handler(bsp_event_t event)
{
    switch (event)
    {
        case BSP_EVENT_KEY_0:
        {
            ret_code_t err_code;
            err_code = nrf_ble_ots_c_oacp_write_object(&m_ots_c, 0, OBJECT_SIZE, 0);
            APP_ERROR_CHECK(err_code);
            NRF_LOG_INFO("Write object command sent to server Obejct Action Control Point.");
        }
            break;
        case BSP_EVENT_KEY_1:
        {
            ret_code_t err_code;
            NRF_LOG_INFO("Read the size of the selected object.");
            err_code = nrf_ble_ots_c_obj_size_read(&m_ots_c);
            APP_ERROR_CHECK(err_code);
        }break;

        case BSP_EVENT_SLEEP:
            sleep_mode_enter();
            break;

        case BSP_EVENT_DISCONNECT:
        {
            ret_code_t err_code;
            err_code = sd_ble_gap_disconnect(m_conn_handle,
                                             BLE_HCI_REMOTE_USER_TERMINATED_CONNECTION);
            if (err_code != NRF_ERROR_INVALID_STATE)
            {
                APP_ERROR_CHECK(err_code);
            }
        }break;

        case BSP_EVENT_WHITELIST_OFF:
            whitelist_disable();
            break;

        default:
            break;
    }
}


static void print_feature(nrf_ble_ots_c_feature_t const * const p_feature)
{
    if (p_feature->oacp_create == 1)
    {
        NRF_LOG_INFO("\tCreate OP Code Supported");
    }
    if (p_feature->oacp_delete == 1)
    {
        NRF_LOG_INFO("\tDelete OP Code Supported");
    }
    if (p_feature->oacp_crc == 1)
    {
        NRF_LOG_INFO("\tCalculate Checksum OP Code Supported.");
    }
    if (p_feature->oacp_execute == 1)
    {
        NRF_LOG_INFO("\tExecute OP Code Supported.");
    }
    if (p_feature->oacp_read == 1)
    {
        NRF_LOG_INFO("\tRead OP Code Supported.");
    }
    if (p_feature->oacp_write == 1)
    {
        NRF_LOG_INFO("\tWrite OP Code Supported.");
    }
    if (p_feature->oacp_append == 1)
    {
        NRF_LOG_INFO("\tAppend Additional Data to Objects Supported.");
    }
    if (p_feature->oacp_truncate == 1)
    {
        NRF_LOG_INFO("\tTruncation of Objects Supported.");
    }
    if (p_feature->oacp_patch == 1)
    {
        NRF_LOG_INFO("\tPatching of Objects Supported.");
    }
    if (p_feature->oacp_abort == 1)
    {
        NRF_LOG_INFO("\tAbort OP Code Supported.");
    }

    // PRINT OLCP FEATURES
    if (p_feature->olcp_goto == 1)
    {
        NRF_LOG_INFO("\tGoto OP Code Supported.");
    }
    if (p_feature->olcp_order == 1)
    {
        NRF_LOG_INFO("\tOrder OP Code Supported.");
    }
    if (p_feature->olcp_req_num == 1)
    {
        NRF_LOG_INFO("\tRequest Number OP Code Supported.");
    }
    if (p_feature->olcp_req_num == 1)
    {
        NRF_LOG_INFO("\tClear Marking OP Code Supported.");
    }
    NRF_LOG_INFO("\r\n");
}

static void print_response(nrf_ble_ots_c_oacp_response_t * response)
{
    NRF_LOG_INFO("Request OP Code: %s", (uint32_t)lit_request[response->request_op_code]);
    NRF_LOG_INFO("Result Code    : %s", (uint32_t)lit_result[response->result_code]);
}


static bool peer_ready_for_object_write(nrf_ble_ots_c_evt_t const * const p_ots_c_evt)
{
    if (p_ots_c_evt->params.response.request_op_code == NRF_BLE_OTS_C_OACP_PROC_WRITE
       && p_ots_c_evt->params.response.result_code == NRF_BLE_OTS_C_OACP_RES_SUCCESS)
    {
        return true;
    }
    return false;
}

static bool peer_ready_for_object_read(nrf_ble_ots_c_evt_t const * const p_ots_c_evt)
{
    if (p_ots_c_evt->params.response.request_op_code == NRF_BLE_OTS_C_OACP_PROC_READ
       && p_ots_c_evt->params.response.result_code == NRF_BLE_OTS_C_OACP_RES_SUCCESS)
    {
        return true;
    }
    return false;
}

/**@brief Object Transfer Service event handler.
 */
static void ots_c_evt_handler(nrf_ble_ots_c_evt_t * p_ots_c_evt)
{
    ret_code_t err_code;

    switch (p_ots_c_evt->evt_type)
    {
        case NRF_BLE_OTS_C_EVT_DISCOVERY_COMPLETE:
        {
            err_code = nrf_ble_ots_c_handles_assign(&m_ots_c,
                                                    p_ots_c_evt->conn_handle,
                                                    &p_ots_c_evt->params.handles);
            APP_ERROR_CHECK(err_code);

            NRF_LOG_INFO("Object Transfer Service discovered.");

            NRF_LOG_DEBUG("L2CAP channel setup.");
            ble_l2cap_ch_setup_params_t params;

            params.rx_params.rx_mps = L2CAP_RX_MPS;
            params.rx_params.rx_mtu = L2CAP_RX_MTU;
            params.rx_params.sdu_buf.p_data = NULL;
            params.le_psm = BLE_OTS_PSM;    // Used when requesting setup of an L2CAP channel, ignored otherwise.
            params.status = 0;              // Used when requesting setup of an L2CAP channel, ignored otherwise.

            uint16_t cid  = BLE_L2CAP_CID_INVALID;

            err_code = sd_ble_l2cap_ch_setup(m_ots_c.conn_handle, &cid, &params);
            APP_ERROR_CHECK(err_code);

            m_ots_c.local_cid = cid;        // Set the local Channel ID of the Object Transfer Client Module.
            NRF_LOG_DEBUG("Read the features of the peer:");
            err_code = nrf_ble_ots_c_feature_read(&m_ots_c);
            APP_ERROR_CHECK(err_code);
        } break;

        case NRF_BLE_OTS_C_EVT_FEATURE_READ_RESP:
            NRF_LOG_INFO("\r\n");
            NRF_LOG_INFO("Supported features on the connected Server:");
            print_feature(&p_ots_c_evt->params.feature);

            err_code = nrf_ble_ots_c_indication_enable(&m_ots_c, true);
            APP_ERROR_CHECK(err_code);
            NRF_LOG_INFO("Indications enabled.");

            break;

        case NRF_BLE_OTS_C_EVT_SIZE_READ_RESP:
            NRF_LOG_INFO("Selected object: current size %i, allocated size %i",
                          p_ots_c_evt->params.size.current_size,
                          p_ots_c_evt->params.size.allocated_size);
            if (p_ots_c_evt->params.size.current_size > 0)
            {
                if (p_ots_c_evt->params.size.current_size > OBJECT_SIZE)
                {
                    p_ots_c_evt->params.size.current_size = OBJECT_SIZE;
                }
                NRF_LOG_INFO("Read %i bytes from the object.",
                              p_ots_c_evt->params.size.current_size);
                err_code = nrf_ble_ots_c_oacp_read_object(&m_ots_c,
                                                          0,
                                                          p_ots_c_evt->params.size.current_size);
                APP_ERROR_CHECK(err_code);
            }
            break;

        case NRF_BLE_OTS_C_EVT_OACP_RESP:
            NRF_LOG_INFO("Response received on Object Action Control Point.");
            print_response(&p_ots_c_evt->params.response);

            if (peer_ready_for_object_write(p_ots_c_evt))
            {
                m_sdu_buf.p_data = m_object;
                m_sdu_buf.len    = OBJECT_SIZE;
                err_code         = nrf_ble_ots_c_l2cap_obj_send(&m_ots_c, &m_sdu_buf);
                APP_ERROR_CHECK(err_code);
            }
            if (peer_ready_for_object_read(p_ots_c_evt))
            {
                m_sdu_buf.p_data = m_object_rx;
                m_sdu_buf.len    = OBJECT_SIZE;
                err_code = nrf_ble_ots_c_l2cap_obj_receive(&m_ots_c, &m_sdu_buf);
                APP_ERROR_CHECK(err_code);
            }
            break;

        case NRF_BLE_OTS_C_EVT_DISCONN_COMPLETE:
            NRF_LOG_INFO("NRF_BLE_OTS_C_EVT_DISCONN_COMPLETE.");
            print_response(&p_ots_c_evt->params.response);
            break;

        case NRF_BLE_OTS_C_EVT_CHANNEL_RELEASED:
            NRF_LOG_INFO("NRF_BLE_OTS_C_EVT_CHANNEL_RELEASED.");
            break;

        case NRF_BLE_OTS_C_EVT_OBJ_READ:
            NRF_LOG_INFO("NRF_BLE_OTS_C_EVT_OBJ_READ.");
            break;

        default:
            break;
    }
}

/**@brief Function for filling an array with ascending and looping numbers from 0 to FF.
 *
 * @param[in] erase_bonds  Indicates whether bonding information should be cleared from
 *                         persistent storage during initialization of the Peer Manager.
 */
void fill_array(uint8_t * data, uint32_t len)
{
    for (uint32_t i = 0; i < len; i++)
    {
        data[i] = (uint8_t) i;
    }
}


/**
 * @brief Function for filling our object with numbers before sending it.
 */
void fill_current_object(void)
{
    fill_array(m_object, OBJECT_SIZE);
}


/**
 * @brief initialization.
 */
static void ots_c_init(void)
{
    fill_current_object();

    nrf_ble_ots_c_init_t init;
    init.evt_handler = ots_c_evt_handler;

    ret_code_t err_code = nrf_ble_ots_c_init(&m_ots_c, &init);
    APP_ERROR_CHECK(err_code);
}


/**@brief Function for the Peer Manager initialization.
 *
 * @param[in] erase_bonds  Indicates whether bonding information should be cleared from
 *                         persistent storage during initialization of the Peer Manager.
 */
static void peer_manager_init()
{
    ble_gap_sec_params_t sec_param;
    ret_code_t           err_code;

    err_code = pm_init();
    APP_ERROR_CHECK(err_code);

    memset(&sec_param, 0, sizeof(ble_gap_sec_params_t));

    // Security parameters to be used for all security procedures.
    sec_param.bond           = SEC_PARAM_BOND;
    sec_param.mitm           = SEC_PARAM_MITM;
    sec_param.lesc           = SEC_PARAM_LESC;
    sec_param.keypress       = SEC_PARAM_KEYPRESS;
    sec_param.io_caps        = SEC_PARAM_IO_CAPABILITIES;
    sec_param.oob            = SEC_PARAM_OOB;
    sec_param.min_key_size   = SEC_PARAM_MIN_KEY_SIZE;
    sec_param.max_key_size   = SEC_PARAM_MAX_KEY_SIZE;
    sec_param.kdist_own.enc  = 1;
    sec_param.kdist_own.id   = 1;
    sec_param.kdist_peer.enc = 1;
    sec_param.kdist_peer.id  = 1;

    err_code = pm_sec_params_set(&sec_param);
    APP_ERROR_CHECK(err_code);

    err_code = pm_register(pm_evt_handler);
    APP_ERROR_CHECK(err_code);
}


/**@brief Clear bond information from persistent storage.
 */
static void delete_bonds(void){
    ret_code_t err_code;

    NRF_LOG_INFO("Erase bonds");
    err_code = pm_peers_delete();
    APP_ERROR_CHECK(err_code);
}



/**
 * @brief Database discovery collector initialization.
 */
static void db_discovery_init(void)
{
    ret_code_t err_code = ble_db_discovery_init(db_disc_handler);
    APP_ERROR_CHECK(err_code);
}


/**@brief Retrive a list of peer manager peer IDs.
 *
 * @param[inout] p_peers   The buffer where to store the list of peer IDs.
 * @param[inout] p_size    In: The size of the @p p_peers buffer.
 *                         Out: The number of peers copied in the buffer.
 */
static void peer_list_get(pm_peer_id_t * p_peers, uint32_t * p_size)
{
    pm_peer_id_t peer_id;
    uint32_t     peers_to_copy;

    peers_to_copy = (*p_size < BLE_GAP_WHITELIST_ADDR_MAX_COUNT) ?
                     *p_size : BLE_GAP_WHITELIST_ADDR_MAX_COUNT;

    peer_id = pm_next_peer_id_get(PM_PEER_ID_INVALID);
    *p_size = 0;

    while ((peer_id != PM_PEER_ID_INVALID) && (peers_to_copy--))
    {
        p_peers[(*p_size)++] = peer_id;
        peer_id = pm_next_peer_id_get(peer_id);
    }
}


static void whitelist_load()
{
    ret_code_t   ret;
    pm_peer_id_t peers[8];
    uint32_t     peer_cnt;

    memset(peers, PM_PEER_ID_INVALID, sizeof(peers));
    peer_cnt = (sizeof(peers) / sizeof(pm_peer_id_t));

    // Load all peers from flash and whitelist them.
    peer_list_get(peers, &peer_cnt);

    ret = pm_whitelist_set(peers, peer_cnt);
    APP_ERROR_CHECK(ret);

    // Setup the device identies list.
    // Some SoftDevices do not support this feature.
    ret = pm_device_identities_list_set(peers, peer_cnt);
    if (ret != NRF_ERROR_NOT_SUPPORTED)
    {
        APP_ERROR_CHECK(ret);
    }
}

/**@brief Function to start scanning.
 */
static void scan_start(void)
{
    // If there is any pending write to flash, defer scanning until it completes.
    if (nrf_fstorage_is_busy(NULL))
    {
        m_memory_access_in_progress = true;
        return;
    }

    // Whitelist buffers.
    ble_gap_addr_t whitelist_addrs[8];
    ble_gap_irk_t  whitelist_irks[8];

    memset(whitelist_addrs, 0x00, sizeof(whitelist_addrs));
    memset(whitelist_irks,  0x00, sizeof(whitelist_irks));

    uint32_t addr_cnt = (sizeof(whitelist_addrs) / sizeof(ble_gap_addr_t));
    uint32_t irk_cnt  = (sizeof(whitelist_irks)  / sizeof(ble_gap_irk_t));

    // Reload the whitelist and whitelist all peers.
    whitelist_load();

    ret_code_t ret;

    // Get the whitelist previously set using pm_whitelist_set().
    ret = pm_whitelist_get(whitelist_addrs, &addr_cnt,
                           whitelist_irks,  &irk_cnt);

    m_scan_param.active   = 0;
    m_scan_param.interval = SCAN_INTERVAL;
    m_scan_param.window   = SCAN_WINDOW;

    if (((addr_cnt == 0) && (irk_cnt == 0)) ||
        (m_whitelist_disabled))
    {
        // Don't use whitelist.
        m_scan_param.use_whitelist  = 0;
        m_scan_param.adv_dir_report = 0;
        m_scan_param.timeout        = 0x0000; // No timeout.
    }
    else
    {
        // Use whitelist.
        m_scan_param.use_whitelist  = 1;
        m_scan_param.adv_dir_report = 0;
        m_scan_param.timeout        = 0x001E; // 30 seconds.
    }

    NRF_LOG_INFO("Starting scan.");

    ret = sd_ble_gap_scan_start(&m_scan_param);
    APP_ERROR_CHECK(ret);

    ret = bsp_indication_set(BSP_INDICATE_SCANNING);
    APP_ERROR_CHECK(ret);
}


/**@brief Function for initializing buttons and leds.
 *
 * @param[out] p_erase_bonds  Will be true if the clear bonding button was pressed to wake the application up.
 */
static void buttons_leds_init(bool * p_erase_bonds)
{
    ret_code_t err_code;
    bsp_event_t startup_event;

    err_code = bsp_init(BSP_INIT_LED | BSP_INIT_BUTTONS, bsp_event_handler);
    APP_ERROR_CHECK(err_code);

    err_code = bsp_btn_ble_init(NULL, &startup_event);
    APP_ERROR_CHECK(err_code);

    *p_erase_bonds = (startup_event == BSP_EVENT_CLEAR_BONDING_DATA);
}


/**@brief Function for initializing logging.
 */
static void log_init(void)
{
    ret_code_t err_code = NRF_LOG_INIT(NULL);
    APP_ERROR_CHECK(err_code);
    NRF_LOG_DEFAULT_BACKENDS_INIT();
}

/**@brief Function for initializing the timer.
 */
static void timer_init(void)
{
    ret_code_t err_code = app_timer_init();
    APP_ERROR_CHECK(err_code);
}


/**@brief Function for initializing the GATT module.
 */
static void gatt_init(void)
{
    ret_code_t err_code = nrf_ble_gatt_init(&m_gatt, NULL);
    APP_ERROR_CHECK(err_code);
}


/** @brief Function for the Power manager.
 */
static void power_manage(void)
{
    ret_code_t err_code = sd_app_evt_wait();
    APP_ERROR_CHECK(err_code);
}


int main(void)
{
    bool erase_bonds;

    // Initialize.
    log_init();
    timer_init();
    buttons_leds_init(&erase_bonds);
    ble_stack_init();
    gatt_init();
    peer_manager_init();
    db_discovery_init();
    ots_c_init();

    whitelist_load();

    // Start scanning for peripherals and initiate connection
    // with devices that advertise Object Transfer Service UUID.
    if (erase_bonds == true)
    {
        delete_bonds();
        // Scan is started by the PM_EVT_PEERS_DELETE_SUCCEEDED event.
    }
    else
    {
        scan_start();
    }

    NRF_LOG_INFO("Object Transfer client example started.");

    for (;;)
    {
        if (NRF_LOG_PROCESS() == false)
        {
            power_manage();
        }
    }
}
