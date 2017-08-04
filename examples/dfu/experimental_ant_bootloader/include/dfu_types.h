/**
 * Copyright (c) 2013 - 2017, Nordic Semiconductor ASA
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

/**@file
 *
 * @defgroup nrf_dfu_types Types and definitions.
 * @{
 *
 * @ingroup nrf_dfu
 *
 * @brief Device Firmware Update module type and definitions.
 */

#ifndef DFU_TYPES_H__
#define DFU_TYPES_H__

#include <stdint.h>
#include "nrf.h"
#include "app_util.h"
#include "nrf_sdm.h"
#include "ant_dfu_constrains.h"
#include "nrf_mbr.h"

#ifdef __cplusplus
extern "C" {
#endif

#define NRF_UICR_BOOT_START_ADDRESS     (NRF_UICR_BASE + 0x14)                                  /**< Register where the bootloader start address is stored in the UICR register. */
#define NRF_UICR_NRFFW_1                (NRF_UICR_BASE + 0x18)                                  /**< Register where the MBR retaining address is stored in the UICR register. */

#define CODE_REGION_1_START             SD_SIZE_GET(MBR_SIZE)                                   /**< This field should correspond to the size of Code Region 0, (which is identical to Start of Code Region 1), found in UICR.CLEN0 register. This value is used for compile safety, as the linker will fail if application expands into bootloader. Runtime, the bootloader will use the value found in UICR.CLEN0. */

#define SOFTDEVICE_REGION_START         MBR_SIZE                                                /**< This field should correspond to start address of the bootloader, found in UICR.RESERVED, 0x10001014, register. This value is used for sanity check, so the bootloader will fail immediately if this value differs from runtime value. The value is used to determine max application size for updating. */

#define DFU_REGION_TOTAL_SIZE           (BOOTLOADER_REGION_START - CODE_REGION_1_START)         /**< Total size of the region between SD and Bootloader. */

#define DFU_APP_DATA_RESERVED           0x0000                                                  /**< Size of Application Data that must be preserved between application updates. This value must be a multiple of page size. Page size is 0x400 (1024d) bytes, thus this value must be 0x0000, 0x0400, 0x0800, 0x0C00, 0x1000, etc. */
#define DFU_IMAGE_MAX_SIZE_FULL         (DFU_REGION_TOTAL_SIZE - DFU_APP_DATA_RESERVED)         /**< Maximum size of a application, excluding save data from the application. */

#define DFU_IMAGE_MAX_SIZE_BANKED       (((((DFU_REGION_TOTAL_SIZE)/2) - DFU_APP_DATA_RESERVED) / CODE_PAGE_SIZE) * CODE_PAGE_SIZE)  /**< Maximum size of a application in dual bank mode, excluding save data from the application. */

#define DFU_BL_IMAGE_MAX_SIZE           (BOOTLOADER_SETTINGS_ADDRESS - BOOTLOADER_REGION_START) /**< Maximum size of a bootloader, excluding save data from the current bootloader. */

#define DFU_BANK_0_REGION_START         CODE_REGION_1_START                                     /**< Bank 0 region start. */
#define DFU_BANK_1_REGION_START         (DFU_BANK_0_REGION_START + DFU_IMAGE_MAX_SIZE_BANKED)   /**< Bank 1 region start. */

#define PACKET_SIZE                     512                                                     /**< Size of each data packet. Also used for initial receiving of packets from transport layer. */
#define PACKET_HEADER_SIZE              sizeof(uint32_t)                                        /**< Size of the data packet header. */

#define EMPTY_FLASH_MASK                0xFFFFFFFF                                              /**< Bit mask that defines an empty address in flash. */

#define INVALID_PACKET                  0x00                                                    /**< Invalid packet identifies. */
#define INIT_PACKET                     0x01                                                    /**< Packet identifies for initialization packet. */
#define START_PACKET                    0x02                                                    /**< Packet identifies for the Data Start Packet. */
#define DATA_PACKET                     0x03                                                    /**< Packet identifies for a Data Packet. */
#define STOP_DATA_PACKET                0x04                                                    /**< Packet identifies for the Data Stop Packet. */


// Safe guard to ensure during compile time that the DFU_APP_DATA_RESERVED is a multiple of page size.
STATIC_ASSERT((((DFU_APP_DATA_RESERVED) & (CODE_PAGE_SIZE - 1)) == 0x00));

typedef enum
{
    DFU_UPDATE_NONE = 0x00,                                                                     /**< Bit field indicating no update is ongoing. */
    DFU_UPDATE_SD   = 0x01,                                                                     /**< Bit field indicating update of SoftDevice is ongoing. */
    DFU_UPDATE_BL   = 0x02,                                                                     /**< Bit field indicating update of bootloader is ongoing. */
    DFU_UPDATE_APP  = 0x04                                                                      /**< Bit field indicating update of application is ongoing. */
} dfu_update_mode_t;


/**@brief Structure holding a bootloader packet received on the UART.
 */
typedef struct
{
    uint32_t total_image_size;
} dfu_init_packet_t;

typedef struct
{
    dfu_update_mode_t   dfu_update_mode;                                                        /**< Packet type, used to identify the content of the received packet referenced by data packet. */
    uint32_t            sd_image_size;                                                          /** Size of the SoftDevice image to be transferred. Zero if no SoftDevice image will be transfered. */
    uint32_t            bl_image_size;                                                          /** Size of the Bootloader image to be transferred. Zero if no Bootloader image will be transfered. */
    uint32_t            app_image_size;                                                         /** Size of the application image to be transmitted. Zero if no Bootloader image will be transfered. */
    uint32_t            info_bytes_size;
} dfu_start_packet_t;

typedef struct
{
    uint32_t   packet_length;                                                                   /**< Packet length of the data packet. Each data is word size, meaning length of 4 is 4 words, not bytes. */
    uint32_t * p_data_packet;                                                                   /**< Data Packet received. Each data is a word size entry. */
} dfu_data_packet_t;

typedef struct
{
    uint32_t   packet_type;                                                                     /**< Packet type, used to identify the content of the received packet referenced by data packet. */
    union {
        dfu_init_packet_t   init_packet;
        dfu_data_packet_t   data_packet;                                                        /**< Used when packet type is INIT_PACKET or DATA_PACKET. Packet contains data received for init or data. */
        dfu_start_packet_t  start_packet;                                                       /**< Used when packet type is START_DATA_PACKET. Will contain information on software to be updtaed, i.e. SoftDevice, Bootloader and/or Application along with image sizes. */
    } params;
} dfu_update_packet_t;

/**@brief DFU status error codes.
*/
typedef enum
{
    DFU_UPDATE_NEW_IMAGES,
    DFU_UPDATE_SD_SWAPPED,
    DFU_UPDATE_BL_SWAPPED,
    DFU_UPDATE_AP_SWAPPED,
    DFU_UPDATE_AP_INVALIDATED,

    DFU_BANK_0_ERASED,                                                                          /**< Status bank 0 erased.*/
    DFU_BANK_1_ERASED,                                                                          /**< Status bank 1 erased.*/
    DFU_TIMEOUT,                                                                                /**< Status timeout.*/
    DFU_RESET                                                                                   /**< Status Reset to indicate current update procedure has been aborted and system should reset. */
} dfu_update_status_code_t;



/**@brief Structure holding DFU complete event.
*/
typedef struct
{
    dfu_update_status_code_t    status_code;                                                    /**< Device Firmware Update status. */
    uint32_t                    sd_image_size;                                                  /**< Size of the recieved SoftDevice. */
    uint32_t                    bl_image_size;                                                  /**< Size of the recieved BootLoader. */
    uint32_t                    ap_image_size;                                                  /**< Size of the recieved Application. */
    uint32_t                    src_image_address;
    uint8_t                     bank_used;                                                      /**< Bank location */
} dfu_update_status_t;

/**@brief Update complete handler type. */
typedef void (*dfu_complete_handler_t)(dfu_update_status_t dfu_update_status);


#ifdef __cplusplus
}
#endif

#endif // DFU_TYPES_H__

/**@} */
