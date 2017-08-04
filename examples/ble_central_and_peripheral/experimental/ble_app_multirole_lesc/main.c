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
/**
 * @brief BLE multirole LESC example application main file.
 *
 * @detail This application demonstrates bonding with LE Secure Connections both as a peripheral and as a central.
 *
 * Led layout:
 * LED 1: Central side is scanning       LED 2: Central side is connected to a peripheral
 * LED 3: Peripheral side is advertising LED 4: Peripheral side is connected to a central
 *
 * @note: This application requires the use of an external ECC library for public key and shared secret calculation.
 *        Refer to the application's documentation for more details.
 *
 */

#include "sdk_config.h"
#include <stdint.h>
#include <stdio.h>
#include <string.h>
#include "nordic_common.h"
#include "nrf_sdh.h"
#include "nrf_sdh_soc.h"
#include "nrf_sdh_ble.h"
#include "peer_manager.h"
#include "app_timer.h"
#include "bsp_btn_ble.h"
#include "ble.h"
#include "app_util.h"
#include "ble_advdata.h"
#include "ble_advertising.h"
#include "ble_conn_params.h"
#include "ble_db_discovery.h"
#include "ble_hrs.h"
#include "ble_hrs_c.h"
#include "ble_conn_state.h"
#include "fds.h"
#include "nrf_crypto.h"
#include "nrf_crypto_keys.h"
#include "nrf_ble_gatt.h"

#include "nrf_log.h"
#include "nrf_log_ctrl.h"
#include "nrf_log_default_backends.h"


#define LESC_DEBUG_MODE                 0                                               /**< Set to 1 to use LESC debug keys, allows you to use a sniffer to inspect traffic. */
#define LESC_MITM_NC                    1                                               /**< Use MITM (Numeric Comparison). */

/** @brief The maximum number of peripheral and central links combined. */
#define NRF_BLE_LINK_COUNT              (NRF_SDH_BLE_PERIPHERAL_LINK_COUNT + NRF_SDH_BLE_CENTRAL_LINK_COUNT)

#define APP_BLE_CONN_CFG_TAG            1                                               /**< A tag identifying the SoftDevice BLE configuration. */

#define CENTRAL_SCANNING_LED            BSP_BOARD_LED_0
#define CENTRAL_CONNECTED_LED           BSP_BOARD_LED_1
#define PERIPHERAL_ADVERTISING_LED      BSP_BOARD_LED_2
#define PERIPHERAL_CONNECTED_LED        BSP_BOARD_LED_3

#define SEC_PARAMS_BOND                 1                                               /**< Perform bonding. */
#if LESC_MITM_NC
#define SEC_PARAMS_MITM                 1                                               /**< Man In The Middle protection required. */
#define SEC_PARAMS_IO_CAPABILITIES      BLE_GAP_IO_CAPS_DISPLAY_YESNO                   /**< Display Yes/No to force Numeric Comparison. */
#else
#define SEC_PARAMS_MITM                 0                                               /**< Man In The Middle protection required. */
#define SEC_PARAMS_IO_CAPABILITIES      BLE_GAP_IO_CAPS_NONE                            /**< No I/O caps. */
#endif
#define SEC_PARAMS_LESC                 1                                               /**< LE Secure Connections pairing required. */
#define SEC_PARAMS_KEYPRESS             0                                               /**< Keypress notifications not required. */
#define SEC_PARAMS_OOB                  0                                               /**< Out Of Band data not available. */
#define SEC_PARAMS_MIN_KEY_SIZE         7                                               /**< Minimum encryption key size in octets. */
#define SEC_PARAMS_MAX_KEY_SIZE         16                                              /**< Maximum encryption key size in octets. */

#define BLE_GAP_LESC_P256_SK_LEN        32

#define MIN_CONNECTION_INTERVAL         (uint16_t) MSEC_TO_UNITS(7.5, UNIT_1_25_MS)     /**< Determines minimum connection interval in milliseconds. */
#define MAX_CONNECTION_INTERVAL         (uint16_t) MSEC_TO_UNITS(30, UNIT_1_25_MS)      /**< Determines maximum connection interval in milliseconds. */
#define SLAVE_LATENCY                   0                                               /**< Determines slave latency in terms of connection events. */
#define SUPERVISION_TIMEOUT             (uint16_t) MSEC_TO_UNITS(500, UNIT_10_MS)       /**< Determines supervision time-out in units of 10 milliseconds. */

#define FIRST_CONN_PARAMS_UPDATE_DELAY  APP_TIMER_TICKS(5000)                           /**< Time from initiating event (connect or start of notification) to first time sd_ble_gap_conn_param_update is called (5 seconds). */
#define NEXT_CONN_PARAMS_UPDATE_DELAY   APP_TIMER_TICKS(30000)                          /**< Time between each call to sd_ble_gap_conn_param_update after the first call (30 seconds). */
#define MAX_CONN_PARAMS_UPDATE_COUNT    3                                               /**< Number of attempts before giving up the connection parameter negotiation. */

#define APP_FEATURE_NOT_SUPPORTED       BLE_GATT_STATUS_ATTERR_APP_BEGIN + 2            /**< Reply when unsupported features are requested. */

/**@brief   Priority of the application BLE event handler.
 * @note    You shouldn't need to modify this value.
 */
#define APP_BLE_OBSERVER_PRIO           1

#define BLE_GAP_LESC_P256_SK_LEN        32


/**@brief Variable length data encapsulation in terms of length and pointer to data.
 */
typedef struct
{
    uint8_t  * p_data;    /**< Pointer to data. */
    uint16_t   data_len;  /**< Length of data. */
} data_t;

/**@brief GAP LE Secure Connections P-256 Private Key. */
typedef struct
{
  uint8_t   sk[BLE_GAP_LESC_P256_SK_LEN];   /**< LE Secure Connections Elliptic Curve Diffie-Hellman P-256 Private Key in little-endian. */
} ble_gap_lesc_p256_sk_t;

typedef struct
{
    bool           is_connected;
    ble_gap_addr_t address;
} conn_peer_t;


BLE_HRS_DEF(m_hrs);                                                         /**< Heart rate service instance. */
BLE_HRS_C_DEF(m_hrs_c);                                                     /**< Structure used to identify the heart rate client module. */
NRF_BLE_GATT_DEF(m_gatt);                                                   /**< GATT module instance. */
BLE_ADVERTISING_DEF(m_advertising);                                         /**< Advertising module instance. */
BLE_DB_DISCOVERY_DEF(m_db_disc);                                            /**< DB discovery module instance. */

static uint16_t           m_conn_handle_hrs_c = BLE_CONN_HANDLE_INVALID;    /**< Connection handle for the HRS central application. */
static bool               m_numneric_match_requested = false;
static uint16_t           m_num_comp_conn_handle;
static conn_peer_t        m_connected_peers[NRF_BLE_LINK_COUNT];

/** @brief Parameters used when scanning. */
static ble_gap_scan_params_t const m_scan_params =
{
    .active   = 1,
    .interval = SCAN_INTERVAL,
    .window   = SCAN_WINDOW,
    .timeout  = SCAN_TIMEOUT,
    .use_whitelist = 0,
};

/**@brief Connection parameters requested for connection. */
static ble_gap_conn_params_t const m_connection_param =
{
    MIN_CONNECTION_INTERVAL,
    MAX_CONNECTION_INTERVAL,
    SLAVE_LATENCY,
    SUPERVISION_TIMEOUT
};

static char * roles_str[] =
{
    "INVALID_ROLE",
    "CENTRAL",
    "PERIPHERAL",
};

/**@brief names which the central applications will scan for, and which will be advertised by the peripherals.
 *  if these are set to empty strings, the UUIDs defined below will be used
 */
static const char m_target_periph_name[] = "";

/**@brief UUIDs which the central applications will scan for if the name above is set to an empty string,
 * and which will be advertised by the peripherals.
 */
static ble_uuid_t m_adv_uuids[] = {{BLE_UUID_HEART_RATE_SERVICE,         BLE_UUID_TYPE_BLE},
                                   {BLE_UUID_RUNNING_SPEED_AND_CADENCE,  BLE_UUID_TYPE_BLE}};

#if LESC_DEBUG_MODE

/**@brief Bluetooth SIG debug mode Private Key */
#error Generated private key is not supported.
__ALIGN(4) static const ble_gap_lesc_p256_sk_t m_lesc_private_key =
{{
    0xbd,0x1a,0x3c,0xcd,0xa6,0xb8,0x99,0x58,0x99,0xb7,0x40,0xeb,0x7b,0x60,0xff,0x4a,
    0x50,0x3f,0x10,0xd2,0xe3,0xb3,0xc9,0x74,0x38,0x5f,0xc5,0xa3,0xd4,0xf6,0x49,0x3f
}};

#else

#endif

__ALIGN(4) static ble_gap_lesc_p256_pk_t m_lesc_public_key;      /**< LESC ECC Public Key */
__ALIGN(4) static ble_gap_lesc_dhkey_t   m_lesc_dh_key;          /**< LESC ECC DH Key*/

/**@brief Allocated private key type to use for LESC DH generation
 */
NRF_CRYPTO_ECC_PRIVATE_KEY_CREATE(m_private_key, SECP256R1);

/**@brief Allocated public key type to use for LESC DH generation
 */
NRF_CRYPTO_ECC_PUBLIC_KEY_CREATE(m_public_key, SECP256R1);

/**@brief Allocated peer public key type to use for LESC DH generation
 */
NRF_CRYPTO_ECC_PUBLIC_KEY_CREATE(m_peer_public_key, SECP256R1);

/**@brief Allocated raw public key to use for LESC DH.
 */
NRF_CRYPTO_ECC_PUBLIC_KEY_RAW_CREATE_FROM_ARRAY(m_public_key_raw, SECP256R1, m_lesc_public_key.pk);

/**@brief Allocated shared instance to use for LESC DH.
 */
NRF_CRYPTO_ECDH_SHARED_SECRET_CREATE_FROM_ARRAY(m_dh_key, SECP256R1, m_lesc_dh_key.key);


/**@brief Function to generate private key */
uint32_t lesc_generate_key_pair(void)
{
    uint32_t ret_val;
    NRF_LOG_INFO("Generating key-pair");
    //Generate a public/private key pair.
    ret_val = nrf_crypto_ecc_key_pair_generate(NRF_CRYPTO_BLE_ECDH_CURVE_INFO, &m_private_key, &m_public_key);
    APP_ERROR_CHECK(ret_val);

    // Convert to a raw type
    NRF_LOG_INFO("Converting to raw type");
    ret_val = nrf_crypto_ecc_public_key_to_raw(NRF_CRYPTO_BLE_ECDH_CURVE_INFO, &m_public_key, &m_public_key_raw);
    APP_ERROR_CHECK(ret_val);

    // Set the public key in the PM.
    ret_val = pm_lesc_public_key_set(&m_lesc_public_key);
    APP_ERROR_CHECK(ret_val);
    return ret_val;
}


/**@brief Function to handle asserts in the SoftDevice.
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


/**@brief Function for handling errors from the Connection Parameters module.
 *
 * @param[in] nrf_error  Error code containing information about what went wrong.
 */
static void conn_params_error_handler(uint32_t nrf_error)
{
    APP_ERROR_HANDLER(nrf_error);
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
    uint32_t   index = 0;
    uint8_t  * p_data;

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


/**@brief Function for initiating scanning.
 */
static void scan_start(void)
{
    ret_code_t err_code;

    (void) sd_ble_gap_scan_stop();

    err_code = sd_ble_gap_scan_start(&m_scan_params);
        APP_ERROR_CHECK(err_code);

    NRF_LOG_INFO("Scanning");
}


/**@brief Function for initiating advertising and scanning.
 */
static void adv_scan_start(void)
{
    ret_code_t err_code;

        scan_start();

        // Turn on the LED to signal scanning.
        bsp_board_led_on(CENTRAL_SCANNING_LED);

        // Start advertising.
        err_code = ble_advertising_start(&m_advertising, BLE_ADV_MODE_FAST);
        APP_ERROR_CHECK(err_code);

        NRF_LOG_INFO("Advertising");
    }


/**@brief Function for handling File Data Storage events.
 *
 * @param[in] p_evt  Peer Manager event.
 * @param[in] cmd
 */
static void fds_evt_handler(fds_evt_t const * const p_fds_evt)
{
    if (p_fds_evt->id == FDS_EVT_GC)
    {
        NRF_LOG_DEBUG("GC completed");
    }
}


/**@brief Function for handling Peer Manager events.
 *
 * @param[in] p_evt  Peer Manager event.
 */
static void pm_evt_handler(pm_evt_t const * p_evt)
{
    ret_code_t err_code;
    uint16_t role = ble_conn_state_role(p_evt->conn_handle);

    switch (p_evt->evt_id)
    {
        case PM_EVT_BONDED_PEER_CONNECTED:
        {
            NRF_LOG_DEBUG("%s : PM_EVT_BONDED_PEER_CONNECTED: peer_id=%d",
                           nrf_log_push(roles_str[role]),
                           p_evt->peer_id);
        } break;

        case PM_EVT_CONN_SEC_START:
        {
            NRF_LOG_DEBUG("%s : PM_EVT_CONN_SEC_START: peer_id=%d",
                           nrf_log_push(roles_str[role]),
                           p_evt->peer_id);
        } break;

        case PM_EVT_CONN_SEC_SUCCEEDED:
        {
            NRF_LOG_INFO("%s : PM_EVT_CONN_SEC_SUCCEEDED conn_handle: %d, Procedure: %d",
                           nrf_log_push(roles_str[role]),
                           p_evt->conn_handle,
                           p_evt->params.conn_sec_succeeded.procedure);
        } break;

        case PM_EVT_CONN_SEC_FAILED:
        {
            NRF_LOG_DEBUG("%s: PM_EVT_CONN_SEC_FAILED: peer_id=%d, error=%d",
                          nrf_log_push(roles_str[role]),
                          p_evt->peer_id,
                          p_evt->params.conn_sec_failed.error);

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
        } break;

        case PM_EVT_PEER_DATA_UPDATE_SUCCEEDED:
        {
            NRF_LOG_DEBUG("%s: PM_EVT_PEER_DATA_UPDATE_SUCCEEDED: peer_id=%d data_id=0x%x action=0x%x",
                           nrf_log_push(roles_str[role]),
                           p_evt->peer_id,
                           p_evt->params.peer_data_update_succeeded.data_id,
                           p_evt->params.peer_data_update_succeeded.action);
        } break;

        case PM_EVT_PEERS_DELETE_SUCCEEDED:
        {
            adv_scan_start();
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

        case PM_EVT_PEER_DELETE_SUCCEEDED:
        case PM_EVT_LOCAL_DB_CACHE_APPLIED:
        case PM_EVT_SERVICE_CHANGED_IND_SENT:
        case PM_EVT_SERVICE_CHANGED_IND_CONFIRMED:
        default:
            break;
    }
}


/**@brief Handles events coming from the Heart Rate central module.
 */
static void hrs_c_evt_handler(ble_hrs_c_t * p_hrs_c, ble_hrs_c_evt_t * p_hrs_c_evt)
{
    switch (p_hrs_c_evt->evt_type)
    {
        case BLE_HRS_C_EVT_DISCOVERY_COMPLETE:
        {
            if (m_conn_handle_hrs_c == BLE_CONN_HANDLE_INVALID)
            {
                ret_code_t err_code;

                m_conn_handle_hrs_c = p_hrs_c_evt->conn_handle;
                NRF_LOG_INFO("CENTRAL: HRS discovered on conn_handle 0x%x",
                                m_conn_handle_hrs_c);

                err_code = ble_hrs_c_handles_assign(p_hrs_c,
                                                    m_conn_handle_hrs_c,
                                                    &p_hrs_c_evt->params.peer_db);
                APP_ERROR_CHECK(err_code);

                // Initiate bonding.
                NRF_LOG_INFO("CENTRAL: initiate security");
                err_code = pm_conn_secure(m_conn_handle_hrs_c, false);
                if (err_code != NRF_SUCCESS)
                {
                   NRF_LOG_INFO("Securing the connection failed. Reason: %d", err_code);
                }
                if (err_code != NRF_ERROR_INVALID_STATE)
                {
                    APP_ERROR_CHECK(err_code);
                }

                // Heart rate service discovered. Enable notification of Heart Rate Measurement.
                err_code = ble_hrs_c_hrm_notif_enable(p_hrs_c);
                APP_ERROR_CHECK(err_code);
            }
        } break; // BLE_HRS_C_EVT_DISCOVERY_COMPLETE

        case BLE_HRS_C_EVT_HRM_NOTIFICATION:
        {
            NRF_LOG_INFO("CENTRAL: Heart Rate = %d", p_hrs_c_evt->params.hrm.hr_value);
        } break;

        default:
            // No implementation needed.
            break;
    }
}


/**@brief Function for searching a given name in the advertisement packets.
 *
 * @details Use this function to parse received advertising data and to find a given
 * name in them either as 'complete_local_name' or as 'short_local_name'.
 *
 * @param[in]   p_adv_report   advertising data to parse.
 * @param[in]   name_to_find   name to search.
 * @return   true if the given name was found, false otherwise.
 */
static bool find_adv_name(ble_gap_evt_adv_report_t const * p_adv_report, char const * name_to_find)
{
    ret_code_t err_code;
    data_t     adv_data;
    data_t     dev_name;

    // Initialize advertisement report for parsing
    adv_data.p_data   = (uint8_t *)p_adv_report->data;
    adv_data.data_len = p_adv_report->dlen;

    //search for advertising names
    err_code = adv_report_parse(BLE_GAP_AD_TYPE_COMPLETE_LOCAL_NAME, &adv_data, &dev_name);
    if (err_code == NRF_SUCCESS)
    {
        if (memcmp(name_to_find, dev_name.p_data, dev_name.data_len) == 0)
        {
            return true;
        }
    }
    else
    {
        // Look for the short local name if it was not found as complete
        err_code = adv_report_parse(BLE_GAP_AD_TYPE_SHORT_LOCAL_NAME, &adv_data, &dev_name);
        if (err_code != NRF_SUCCESS)
        {
            return false;
        }
        if (memcmp(m_target_periph_name, dev_name.p_data, dev_name.data_len) == 0)
        {
            return true;
        }
    }
    return false;
}


/**@brief Function for searching a UUID in the advertisement packets.
 *
 * @details Use this function to parse received advertising data and to find a given
 * UUID in them.
 *
 * @param[in]   p_adv_report   advertising data to parse.
 * @param[in]   uuid_to_find   UUIID to search.
 * @return   true if the given UUID was found, false otherwise.
 */
static bool find_adv_uuid(ble_gap_evt_adv_report_t const * p_adv_report, uint16_t uuid_to_find)
{
    ret_code_t err_code;
    data_t     adv_data;
    data_t     type_data;

    // Initialize advertisement report for parsing.
    adv_data.p_data   = (uint8_t *)p_adv_report->data;
    adv_data.data_len = p_adv_report->dlen;

    err_code = adv_report_parse(BLE_GAP_AD_TYPE_16BIT_SERVICE_UUID_MORE_AVAILABLE,
                                &adv_data,
                                &type_data);

    if (err_code != NRF_SUCCESS)
    {
        // Look for the services in 'complete' if it was not found in 'more available'.
        err_code = adv_report_parse(BLE_GAP_AD_TYPE_16BIT_SERVICE_UUID_COMPLETE,
                                    &adv_data,
                                    &type_data);

        if (err_code != NRF_SUCCESS)
        {
            // If we can't parse the data, then exit.
            return false;
        }
    }

    // Verify if any UUID match the given UUID.
    for (uint32_t i = 0; i < (type_data.data_len / sizeof(uint16_t)); i++)
    {
        uint16_t extracted_uuid = uint16_decode(&type_data.p_data[i * sizeof(uint16_t)]);
        if (extracted_uuid == uuid_to_find)
        {
            return true;
        }
    }
    return false;
}


/**@brief Function for checking if a link already exists with a new connected peer.
 *
 * @details This function checks if a newly connected device is already connected
 *
 * @param[in]   p_connected_evt Bluetooth connected event.
 * @return                      True if the peer's address is found in the list of connected peers,
 *                              false otherwise.
 */
static bool is_already_connected(ble_gap_addr_t const * p_connected_adr)
{
    for (uint32_t i = 0; i < NRF_BLE_LINK_COUNT; i++)
    {
        if (m_connected_peers[i].is_connected)
        {
            if (m_connected_peers[i].address.addr_type == p_connected_adr->addr_type)
            {
                if (memcmp(m_connected_peers[i].address.addr,
                           p_connected_adr->addr,
                           sizeof(m_connected_peers[i].address.addr)) == 0)
                {
                    return true;
                }
            }
        }
    }
    return false;
}


/**@brief Function for handling BLE Stack events common to both the central and peripheral roles.
 * @param[in] conn_handle Connection Handle.
 * @param[in] p_ble_evt  Bluetooth stack event.
 */
static void on_ble_evt(uint16_t conn_handle, ble_evt_t const * p_ble_evt)
{
    ret_code_t err_code;
    char passkey[BLE_GAP_PASSKEY_LEN + 1];
    uint16_t role = ble_conn_state_role(conn_handle);

    switch (p_ble_evt->header.evt_id)
    {
        case BLE_GAP_EVT_CONNECTED:
            m_connected_peers[conn_handle].is_connected = true;
            m_connected_peers[conn_handle].address = p_ble_evt->evt.gap_evt.params.connected.peer_addr;
            break;

        case BLE_GAP_EVT_DISCONNECTED:
            memset(&m_connected_peers[conn_handle], 0x00, sizeof(m_connected_peers[0]));
            break;

        case BLE_GAP_EVT_SEC_PARAMS_REQUEST:
            NRF_LOG_INFO("%s: BLE_GAP_EVT_SEC_PARAMS_REQUEST", nrf_log_push(roles_str[role]));
            break;

        case BLE_GAP_EVT_PASSKEY_DISPLAY:
            memcpy(passkey, p_ble_evt->evt.gap_evt.params.passkey_display.passkey, BLE_GAP_PASSKEY_LEN);
            passkey[BLE_GAP_PASSKEY_LEN] = 0x00;
            NRF_LOG_INFO("%s: BLE_GAP_EVT_PASSKEY_DISPLAY: passkey=%s match_req=%d",
                         nrf_log_push(roles_str[role]),
                         nrf_log_push(passkey),
                         p_ble_evt->evt.gap_evt.params.passkey_display.match_request);

            if (p_ble_evt->evt.gap_evt.params.passkey_display.match_request)
            {
                NRF_LOG_INFO("Press Button 1 to confirm, Button 2 to reject");
                m_num_comp_conn_handle = conn_handle;
                m_numneric_match_requested = true;
            }
            break;

        case BLE_GAP_EVT_AUTH_KEY_REQUEST:
            NRF_LOG_INFO("%s: BLE_GAP_EVT_AUTH_KEY_REQUEST", nrf_log_push(roles_str[role]));
            break;

        case BLE_GAP_EVT_LESC_DHKEY_REQUEST:
            NRF_LOG_INFO("%s: BLE_GAP_EVT_LESC_DHKEY_REQUEST", nrf_log_push(roles_str[role]));

            static nrf_value_length_t peer_public_key_raw = {0};

            peer_public_key_raw.p_value = &p_ble_evt->evt.gap_evt.params.lesc_dhkey_request.p_pk_peer->pk[0];
            peer_public_key_raw.length = BLE_GAP_LESC_P256_PK_LEN;

            err_code = nrf_crypto_ecc_public_key_from_raw(NRF_CRYPTO_BLE_ECDH_CURVE_INFO,
                                                          &peer_public_key_raw,
                                                          &m_peer_public_key);
            APP_ERROR_CHECK(err_code);

            err_code = nrf_crypto_ecdh_shared_secret_compute(NRF_CRYPTO_BLE_ECDH_CURVE_INFO,
                                                             &m_private_key,
                                                             &m_peer_public_key,
                                                             &m_dh_key);
            APP_ERROR_CHECK(err_code);

            err_code = sd_ble_gap_lesc_dhkey_reply(conn_handle, &m_lesc_dh_key);
            APP_ERROR_CHECK(err_code);
            break;

         case BLE_GAP_EVT_AUTH_STATUS:
             NRF_LOG_INFO("%s: BLE_GAP_EVT_AUTH_STATUS: status=0x%x bond=0x%x lv4: %d kdist_own:0x%x kdist_peer:0x%x",
                          nrf_log_push(roles_str[role]),
                          p_ble_evt->evt.gap_evt.params.auth_status.auth_status,
                          p_ble_evt->evt.gap_evt.params.auth_status.bonded,
                          p_ble_evt->evt.gap_evt.params.auth_status.sm1_levels.lv4,
                          *((uint8_t *)&p_ble_evt->evt.gap_evt.params.auth_status.kdist_own),
                          *((uint8_t *)&p_ble_evt->evt.gap_evt.params.auth_status.kdist_peer));
            break;

        default:
            // No implementation needed.
            break;
    }
}


/**@brief Function for handling BLE Stack events concerning central applications.
 *
 * @details This function keeps the connection handles of central applications up-to-date. It
 * parses scanning reports, initiating a connection attempt to peripherals when a target UUID
 * is found, and manages connection parameter update requests. Additionally, it updates the status
 * of LEDs used to report central applications activity.
 *
 * @note        Since this function updates connection handles, @ref BLE_GAP_EVT_DISCONNECTED events
 *              should be dispatched to the target application before invoking this function.
 *
 * @param[in]   p_ble_evt   Bluetooth stack event.
 */
static void on_ble_central_evt(ble_evt_t const * p_ble_evt)
{
    ble_gap_evt_t const * p_gap_evt = &p_ble_evt->evt.gap_evt;
    ret_code_t            err_code;

    switch (p_ble_evt->header.evt_id)
    {
        // Upon connection, check which peripheral has connected (HR or RSC), initiate DB
        //  discovery, update LEDs status and resume scanning if necessary.
        case BLE_GAP_EVT_CONNECTED:
        {
            NRF_LOG_INFO("CENTRAL: Connected, handle: %d.", p_gap_evt->conn_handle);
            // If no Heart Rate sensor is currently connected, try to find them on this peripheral.
            if (m_conn_handle_hrs_c == BLE_CONN_HANDLE_INVALID)
            {
                NRF_LOG_INFO("CENTRAL: Searching for HRS on conn_handle 0x%x", p_gap_evt->conn_handle);

                memset(&m_db_disc, 0x00, sizeof(ble_db_discovery_t));
                err_code = ble_db_discovery_start(&m_db_disc, p_gap_evt->conn_handle);
                APP_ERROR_CHECK(err_code);
            }
            // Update LEDs status.
            bsp_board_led_off(CENTRAL_SCANNING_LED);
            bsp_board_led_on(CENTRAL_CONNECTED_LED);
        } break; // BLE_GAP_EVT_CONNECTED

        // Upon disconnection, reset the connection handle of the peer which disconnected, update
        // the LEDs status and start scanning again.
        case BLE_GAP_EVT_DISCONNECTED:
        {
            NRF_LOG_INFO("CENTRAL: Disconnected, handle: %d, reason: 0x%x",
                         p_gap_evt->conn_handle,
                       p_gap_evt->params.disconnected.reason);

            // Update LEDs status.
            bsp_board_led_off(CENTRAL_CONNECTED_LED);
            bsp_board_led_on(CENTRAL_SCANNING_LED);

            m_conn_handle_hrs_c = BLE_CONN_HANDLE_INVALID;
            scan_start();
        } break; // BLE_GAP_EVT_DISCONNECTED

        case BLE_GAP_EVT_ADV_REPORT:
        {
            if (is_already_connected(&p_gap_evt->params.adv_report.peer_addr))
            {
                break;
            }

            bool do_connect = false;
            if (strlen(m_target_periph_name) != 0)
            {
                if (find_adv_name(&p_gap_evt->params.adv_report, m_target_periph_name))
                {
                    do_connect = true;
                }
            }
            else if (   find_adv_uuid(&p_gap_evt->params.adv_report, BLE_UUID_HEART_RATE_SERVICE)
                    && (m_conn_handle_hrs_c == BLE_CONN_HANDLE_INVALID))
            {
                // We do not want to connect to two peripherals offering the same service, so when
                // a UUID is matched, we check that we are not already connected to a peer which
                // offers the same service.
                    do_connect = true;
            }

            if (do_connect)
            {
                // Initiate connection.
                NRF_LOG_INFO("CENTRAL: Connecting...");
                err_code = sd_ble_gap_connect(&p_gap_evt->params.adv_report.peer_addr,
                                              &m_scan_params, &m_connection_param,
                                              APP_BLE_CONN_CFG_TAG);

                if (err_code != NRF_SUCCESS)
                {
                    NRF_LOG_DEBUG("Connection Request Failed, reason %d", err_code);
                }
            }
        } break; // BLE_GAP_ADV_REPORT

        case BLE_GAP_EVT_TIMEOUT:
        {
            // We have not specified a timeout for scanning, so only connection attemps can timeout.
            if (p_gap_evt->params.timeout.src == BLE_GAP_TIMEOUT_SRC_CONN)
            {
                NRF_LOG_DEBUG("CENTRAL: Connection Request timed out.");
            }
        } break;

        case BLE_GAP_EVT_CONN_PARAM_UPDATE_REQUEST:
        {
            // Accept parameters requested by peer.
            err_code = sd_ble_gap_conn_param_update(p_gap_evt->conn_handle,
                                        &p_gap_evt->params.conn_param_update_request.conn_params);
            APP_ERROR_CHECK(err_code);
        } break;

        case BLE_GATTC_EVT_TIMEOUT:
            // Disconnect on GATT Client timeout event.
            NRF_LOG_DEBUG("CENTRAL: GATT Client Timeout.");
            err_code = sd_ble_gap_disconnect(p_ble_evt->evt.gattc_evt.conn_handle,
                                             BLE_HCI_REMOTE_USER_TERMINATED_CONNECTION);
            APP_ERROR_CHECK(err_code);
            break;

        case BLE_GATTS_EVT_TIMEOUT:
            // Disconnect on GATT Server timeout event.
            NRF_LOG_DEBUG("CENTRAL: GATT Server Timeout.");
            err_code = sd_ble_gap_disconnect(p_ble_evt->evt.gatts_evt.conn_handle,
                                             BLE_HCI_REMOTE_USER_TERMINATED_CONNECTION);
            APP_ERROR_CHECK(err_code);
            break;

        default:
            // No implementation needed.
            break;
    }
}


/**@brief Function for handling BLE Stack events involving peripheral applications. Manages the
 * LEDs used to report the status of the peripheral applications.
 *
 * @param[in] p_ble_evt  Bluetooth stack event.
 */
static void on_ble_peripheral_evt(ble_evt_t const * p_ble_evt)
{
    ret_code_t err_code;
    switch (p_ble_evt->header.evt_id)
    {
        case BLE_GAP_EVT_CONNECTED:
            NRF_LOG_INFO("PERIPHERAL: Connected, handle %d.", p_ble_evt->evt.gap_evt.conn_handle);
            bsp_board_led_off(PERIPHERAL_ADVERTISING_LED);
            bsp_board_led_on(PERIPHERAL_CONNECTED_LED);
            break;

        case BLE_GAP_EVT_DISCONNECTED:
            NRF_LOG_INFO("PERIPHERAL: Disconnected, handle %d, reason 0x%x.",
                         p_ble_evt->evt.gap_evt.conn_handle,
                         p_ble_evt->evt.gap_evt.params.disconnected.reason);
            bsp_board_led_off(PERIPHERAL_CONNECTED_LED);
        break;

        case BLE_GATTC_EVT_TIMEOUT:
            // Disconnect on GATT Client timeout event.
            NRF_LOG_DEBUG("PERIPHERAL: GATT Client Timeout.");
            err_code = sd_ble_gap_disconnect(p_ble_evt->evt.gattc_evt.conn_handle,
                                             BLE_HCI_REMOTE_USER_TERMINATED_CONNECTION);
            APP_ERROR_CHECK(err_code);
            break;

        case BLE_GATTS_EVT_TIMEOUT:
            // Disconnect on GATT Server timeout event.
            NRF_LOG_DEBUG("PERIPHERAL: GATT Server Timeout.");
            err_code = sd_ble_gap_disconnect(p_ble_evt->evt.gatts_evt.conn_handle,
                                             BLE_HCI_REMOTE_USER_TERMINATED_CONNECTION);
            APP_ERROR_CHECK(err_code);
            break;

        case BLE_EVT_USER_MEM_REQUEST:
            err_code = sd_ble_user_mem_reply(p_ble_evt->evt.gap_evt.conn_handle, NULL);
            APP_ERROR_CHECK(err_code);
            break;

        case BLE_GATTS_EVT_RW_AUTHORIZE_REQUEST:
        {
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
                    auth_reply.params.write.gatt_status = APP_FEATURE_NOT_SUPPORTED;
                    err_code = sd_ble_gatts_rw_authorize_reply(p_ble_evt->evt.gatts_evt.conn_handle,
                                                               &auth_reply);
                    APP_ERROR_CHECK(err_code);
                }
            }
        } break; // BLE_GATTS_EVT_RW_AUTHORIZE_REQUEST

        default:
            // No implementation needed.
            break;
    }
}


/**@brief Function for handling advertising events.
 *
 * @param[in] ble_adv_evt  Advertising event.
 */
static void on_adv_evt(ble_adv_evt_t ble_adv_evt)
{
    switch (ble_adv_evt)
    {
        case BLE_ADV_EVT_FAST:
            bsp_board_led_on(PERIPHERAL_ADVERTISING_LED);
            break;

        case BLE_ADV_EVT_IDLE:
        {
            ret_code_t err_code = ble_advertising_start(&m_advertising, BLE_ADV_MODE_FAST);
            APP_ERROR_CHECK(err_code);
        } break;

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
    uint16_t conn_handle = p_ble_evt->evt.gap_evt.conn_handle;
    uint16_t role        = ble_conn_state_role(conn_handle);

    if (    (p_ble_evt->header.evt_id == BLE_GAP_EVT_CONNECTED)
        &&  (is_already_connected(&p_ble_evt->evt.gap_evt.params.connected.peer_addr)))
    {
        NRF_LOG_INFO("%s: Already connected to this device as %s (handle: %d), disconnecting.",
                     (role == BLE_GAP_ROLE_PERIPH) ? "PERIPHERAL" : "CENTRAL",
                     (role == BLE_GAP_ROLE_PERIPH) ? "CENTRAL"    : "PERIPHERAL",
                     conn_handle);

        (void)sd_ble_gap_disconnect(conn_handle, BLE_HCI_REMOTE_USER_TERMINATED_CONNECTION);

        // Do not process the event further.
        return;
    }

    on_ble_evt(conn_handle, p_ble_evt);

    if (role == BLE_GAP_ROLE_PERIPH)
    {
        // Manages peripheral LEDs.
        on_ble_peripheral_evt(p_ble_evt);
    }
    else if ((role == BLE_GAP_ROLE_CENTRAL) || (p_ble_evt->header.evt_id == BLE_GAP_EVT_ADV_REPORT))
    {
        on_ble_central_evt(p_ble_evt);
    }
}


/**@brief Function for initializing the Heart Rate client service. */
static void hrs_c_init(void)
{
    ret_code_t       err_code;
    ble_hrs_c_init_t hrs_c_init_obj;

    hrs_c_init_obj.evt_handler = hrs_c_evt_handler;

    err_code = ble_hrs_c_init(&m_hrs_c, &hrs_c_init_obj);
    APP_ERROR_CHECK(err_code);
}


/**@brief Function for initializing the BLE stack.
 *
 * @details Initializes the SoftDevice and the BLE event interrupts.
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

    // Register a handler for BLE events.
    NRF_SDH_BLE_OBSERVER(m_ble_observer, APP_BLE_OBSERVER_PRIO, ble_evt_handler, NULL);
}


/**@brief Function for initializing the Peer Manager. */
static void peer_manager_init(void)
{
    ble_gap_sec_params_t sec_params;
    ret_code_t err_code;

    err_code = pm_init();
    APP_ERROR_CHECK(err_code);

    memset(&sec_params, 0, sizeof(ble_gap_sec_params_t));

    // Security parameters to be used for all security procedures.
    sec_params.bond           = SEC_PARAMS_BOND;
    sec_params.mitm           = SEC_PARAMS_MITM;
    sec_params.lesc           = SEC_PARAMS_LESC;
    sec_params.keypress       = SEC_PARAMS_KEYPRESS;
    sec_params.io_caps        = SEC_PARAMS_IO_CAPABILITIES;
    sec_params.oob            = SEC_PARAMS_OOB;
    sec_params.min_key_size   = SEC_PARAMS_MIN_KEY_SIZE;
    sec_params.max_key_size   = SEC_PARAMS_MAX_KEY_SIZE;
    sec_params.kdist_own.enc  = 1;
    sec_params.kdist_own.id   = 1;
    sec_params.kdist_peer.enc = 1;
    sec_params.kdist_peer.id  = 1;

    err_code = pm_sec_params_set(&sec_params);
    APP_ERROR_CHECK(err_code);

    err_code = pm_register(pm_evt_handler);
    APP_ERROR_CHECK(err_code);

    err_code = fds_register(fds_evt_handler);
    APP_ERROR_CHECK(err_code);

    // Private public keypair must be generated at least once for each device. It can be stored
    // beyond this point. Here it is generated at bootup.
    err_code = lesc_generate_key_pair();
    APP_ERROR_CHECK(err_code);
}


/** @brief Delete all data stored for all peers. */
static void delete_bonds(void)
{
    ret_code_t err_code;

    NRF_LOG_INFO("Erase bonds!");

    err_code = pm_peers_delete();
    APP_ERROR_CHECK(err_code);
}


/**@brief Function for handling events from the BSP module.
 *
 * @param[in]   event   Event generated by button press.
 */
static void bsp_event_handler(bsp_event_t event)
{
    ret_code_t err_code;
    switch (event)
    {
        case BSP_EVENT_KEY_0:
            err_code = ble_hrs_heart_rate_measurement_send(&m_hrs, 87);
            if ((err_code != NRF_SUCCESS) &&
                (err_code != NRF_ERROR_INVALID_STATE) &&
                (err_code != NRF_ERROR_RESOURCES) &&
                (err_code != BLE_ERROR_GATTS_SYS_ATTR_MISSING)
                )
            {
                APP_ERROR_HANDLER(err_code);
            }
            if (m_numneric_match_requested)
            {
                NRF_LOG_INFO("Numeric Match");
                err_code = sd_ble_gap_auth_key_reply(m_num_comp_conn_handle, BLE_GAP_AUTH_KEY_TYPE_PASSKEY, NULL);
                APP_ERROR_CHECK(err_code);
                m_numneric_match_requested = false;
            }
            break;

      case BSP_EVENT_KEY_1:
            if (m_numneric_match_requested)
            {
                NRF_LOG_INFO("Numeric REJECT");
                err_code = sd_ble_gap_auth_key_reply(m_num_comp_conn_handle, BLE_GAP_AUTH_KEY_TYPE_NONE, NULL);//Reject
                APP_ERROR_CHECK(err_code);
                m_numneric_match_requested = false;
            }
            break;

        default:
            break;
    }
}


/**@brief Function for initializing buttons and leds.
 *
 * @param[out] p_erase_bonds  Will be true if the clear bonding button was pressed to
 *                            wake the application up.
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


/**@brief Function for the GAP initialization.
 *
 * @details This function sets up all the necessary GAP (Generic Access Profile) parameters of the
 *          device including the device name, appearance, and the preferred connection parameters.
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

    gap_conn_params.min_conn_interval = MIN_CONNECTION_INTERVAL;
    gap_conn_params.max_conn_interval = MAX_CONNECTION_INTERVAL;
    gap_conn_params.slave_latency     = SLAVE_LATENCY;
    gap_conn_params.conn_sup_timeout  = SUPERVISION_TIMEOUT;

    err_code = sd_ble_gap_ppcp_set(&gap_conn_params);
    APP_ERROR_CHECK(err_code);
}


/**@brief Function for initializing the GATT module. */
static void gatt_init(void)
{
    ret_code_t err_code = nrf_ble_gatt_init(&m_gatt, NULL);
    APP_ERROR_CHECK(err_code);
}


/**@brief Function for initializing the Connection Parameters module. */
static void conn_params_init(void)
{
    ret_code_t             err_code;
    ble_conn_params_init_t cp_init;

    memset(&cp_init, 0, sizeof(cp_init));

    cp_init.p_conn_params                  = NULL;
    cp_init.first_conn_params_update_delay = FIRST_CONN_PARAMS_UPDATE_DELAY;
    cp_init.next_conn_params_update_delay  = NEXT_CONN_PARAMS_UPDATE_DELAY;
    cp_init.max_conn_params_update_count   = MAX_CONN_PARAMS_UPDATE_COUNT;
    cp_init.start_on_notify_cccd_handle    = BLE_CONN_HANDLE_INVALID; // Start upon connection.
    cp_init.disconnect_on_fail             = true;
    cp_init.evt_handler                    = NULL;  // Ignore events.
    cp_init.error_handler                  = conn_params_error_handler;

    err_code = ble_conn_params_init(&cp_init);
    APP_ERROR_CHECK(err_code);
}


/**@brief Function for handling database discovery events.
 *
 * @details This function is callback function to handle events from the database discovery module.
 *          Depending on the UUIDs that are discovered, this function should forward the events
 *          to their respective services.
 *
 * @param[in] p_event  Pointer to the database discovery event.
 */
static void db_disc_handler(ble_db_discovery_evt_t * p_evt)
{
    ble_hrs_on_db_disc_evt(&m_hrs_c, p_evt);
}


/**@brief Function for initializing the DB discovery module. */
static void db_discovery_init(void)
{
    ret_code_t err_code = ble_db_discovery_init(db_disc_handler);
    APP_ERROR_CHECK(err_code);
}


/**@brief Function for initializing the Heart Rate service. */
static void hrs_init(void)
{
    ret_code_t     err_code;
    ble_hrs_init_t hrs_init;
    uint8_t        body_sensor_location;

    // Initialize the Heart Rate Service.
    body_sensor_location = BLE_HRS_BODY_SENSOR_LOCATION_FINGER;

    memset(&hrs_init, 0, sizeof(hrs_init));

    hrs_init.evt_handler                 = NULL;
    hrs_init.is_sensor_contact_supported = true;
    hrs_init.p_body_sensor_location      = &body_sensor_location;

    // Require LESC with MITM (Numeric Comparison)
    BLE_GAP_CONN_SEC_MODE_SET_LESC_ENC_WITH_MITM(&hrs_init.hrs_hrm_attr_md.cccd_write_perm);
    BLE_GAP_CONN_SEC_MODE_SET_NO_ACCESS(&hrs_init.hrs_hrm_attr_md.read_perm);
    BLE_GAP_CONN_SEC_MODE_SET_NO_ACCESS(&hrs_init.hrs_hrm_attr_md.write_perm);

    // Require LESC with MITM (Numeric Comparison)
    BLE_GAP_CONN_SEC_MODE_SET_LESC_ENC_WITH_MITM(&hrs_init.hrs_bsl_attr_md.read_perm);
    BLE_GAP_CONN_SEC_MODE_SET_NO_ACCESS(&hrs_init.hrs_bsl_attr_md.write_perm);

    err_code = ble_hrs_init(&m_hrs, &hrs_init);
    APP_ERROR_CHECK(err_code);
}


/**@brief Function for initializing the Advertising functionality. */
static void advertising_init(void)
{
    ret_code_t             err_code;
    ble_advertising_init_t init;

    memset(&init, 0, sizeof(init));

    init.advdata.name_type               = BLE_ADVDATA_FULL_NAME;
    init.advdata.include_appearance      = true;
    init.advdata.flags                   = BLE_GAP_ADV_FLAGS_LE_ONLY_GENERAL_DISC_MODE;
    init.advdata.uuids_complete.uuid_cnt = sizeof(m_adv_uuids) / sizeof(m_adv_uuids[0]);
    init.advdata.uuids_complete.p_uuids  = m_adv_uuids;

    init.config.ble_adv_fast_enabled  = true;
    init.config.ble_adv_fast_interval = ADV_INTERVAL;
    init.config.ble_adv_fast_timeout  = ADV_TIMEOUT_IN_SECONDS;

    init.evt_handler = on_adv_evt;

    err_code = ble_advertising_init(&m_advertising, &init);
    APP_ERROR_CHECK(err_code);

    ble_advertising_conn_cfg_tag_set(&m_advertising, APP_BLE_CONN_CFG_TAG);
}


/**@brief Function for initializing logging. */
static void log_init(void)
{
    ret_code_t err_code = NRF_LOG_INIT(NULL);
    APP_ERROR_CHECK(err_code);

    NRF_LOG_DEFAULT_BACKENDS_INIT();
}


/**@brief Function for initializing the timer. */
static void timer_init(void)
{
    ret_code_t err_code = app_timer_init();
    APP_ERROR_CHECK(err_code);
}


/**@brief Function for initializing the cryptography module. */
static void crypto_init(void)
{
    NRF_LOG_INFO("Initializing nrf_crypto.");
    ret_code_t err_code = nrf_crypto_init();
    APP_ERROR_CHECK(err_code);
    NRF_LOG_INFO("Initialized nrf_crypto.");
}


/** @brief Function to sleep until a BLE event is received by the application. */
static void power_manage(void)
{
    ret_code_t err_code = sd_app_evt_wait();
    APP_ERROR_CHECK(err_code);
}


int main(void)
{
    bool erase_bonds;

    log_init();
    timer_init();
    buttons_leds_init(&erase_bonds);
    crypto_init();
    ble_stack_init();
    gap_params_init();
    gatt_init();
    conn_params_init();
    db_discovery_init();
    hrs_init();
    hrs_c_init();
    peer_manager_init();
    advertising_init();

    if (erase_bonds == true)
    {
        delete_bonds();
        // Scanning and advertising is started by PM_EVT_PEERS_DELETE_SUCEEDED.
    }
    else
    {
        adv_scan_start();
    }

    NRF_LOG_INFO("LE Secure Connections example started.");

    for (;;)
    {
        if (NRF_LOG_PROCESS() == false)
        {
            // Wait for BLE events.
            power_manage();
        }
    }
}
