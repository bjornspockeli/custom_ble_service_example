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
 * @defgroup nrf_bootloader_types Types and definitions.
 * @{
 *
 * @ingroup nrf_bootloader
 *
 * @brief Bootloader module type and definitions.
 */

#ifndef BOOTLOADER_TYPES_H__
#define BOOTLOADER_TYPES_H__

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

#define BOOTLOADER_SETTINGS_INVALID_APPLICATION                 0xDEADBEEF
#define BOOTLOADER_SETTINGS_VALID_APPLICATION                   0x00000000

#define BOOTLOADER_SETTINGS_VALID_SLOT_ADR_OFFSET               (0UL)
#define BOOTLOADER_SETTINGS_AP_VALIDITY_ADR_OFFSET              (4UL)
#define BOOTLOADER_SETTINGS_SD_IMAGE_SIZE_ADR_OFFSET            (8UL)
#define BOOTLOADER_SETTINGS_BL_IMAGE_SIZE_ADR_OFFSET            (12UL)
#define BOOTLOADER_SETTINGS_AP_IMAGE_SIZE_ADR_OFFSET            (16UL)
#define BOOTLOADER_SETTINGS_RESERVED_1_ADR__OFFSET              (20UL)
#define BOOTLOADER_SETTINGS_RESERVED_2_ADR_OFFSET               (24UL)
#define BOOTLOADER_SETTINGS_RESERVED_3_ADR_OFFSET               (28UL)

#define NEW_IMAGE_BANK_DONE                                     (0UL)
#define NEW_IMAGE_BANK_0                                        (1UL)
#define NEW_IMAGE_BANK_1                                        (2UL)
#define NEW_IMAGE_BANK_INVALID                                  (3UL)

#define NEW_IMAGE_SIZE_UNUSED                                   (0x3FFFFFFF)
#define NEW_IMAGE_SIZE_EMPTY                                    (0x00000000)

#define NEW_IMAGE_INVALID                                       (0xFFFFFFFF)
#define NEW_IMAGE_USED                                          (0x00000000)
typedef union
{
    uint32_t all;
    struct
    {
        uint32_t size               :   30;     /**< Size of the new image*/
        uint32_t bank               :   2;      /**< Which bank it is stored*/
    }st;
}new_image_t;

/**@brief Structure holding bootloader settings for application and bank data.
 * NOTE: If there is a need to update the structure make sure offsets above are still true.
 */
typedef struct
{
    uint32_t            valid_slot;         /**< Valid bootloader_settings slot. */
    uint32_t            valid_app;          /**< Valid application is present if value is 0xFFFFFFFF or 0x00000000 */
    new_image_t         sd_image;           /**< New Softdevice image size */
    new_image_t         bl_image;           /**< New Bootloader image size */
    new_image_t         ap_image;           /**< New Application image size */
    uint32_t            src_image_address;  /**< New Images storage starting address */
    uint32_t            reserved_2;
    uint32_t            reserved_3;
} bootloader_settings_t;


#ifdef __cplusplus
}
#endif

#endif // BOOTLOADER_TYPES_H__

/**@} */
