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
 * @defgroup nrf_bootloader Bootloader API.
 * @{
 *
 * @brief Bootloader module interface.
 */

#ifndef BOOTLOADER_H__
#define BOOTLOADER_H__

#include <stdbool.h>
#include <stdint.h>
#include "bootloader_types.h"
#include <dfu_types.h>

#ifdef __cplusplus
extern "C" {
#endif

/**@brief Function for initializing the Bootloader.
 *
 * @retval     NRF_SUCCESS If bootloader was succesfully initialized.
 */
uint32_t bootloader_init(void);

/**@brief Function for validating application region.
 *
 * @param[in]  app_addr      Address to the region where the application is stored.
 *
 * @retval     true          If Application region is valid.
 * @retval     false         If Application region is not valid.
 */
bool bootloader_app_is_valid(uint32_t app_addr);

/**@brief Function for starting the Device Firmware Update.
 *
 * @retval     NRF_SUCCESS If new appliction image was successfully transfered.
 */
uint32_t bootloader_dfu_start(void);

/**@brief Function for
 *
 * @param[in]  app_addr      Address to the region where the application is stored.
 */
void bootloader_app_start(uint32_t app_addr);

/**@brief Function for processing DFU status update.
 *
 * @param[in]  update_status DFU update status.
 */
void bootloader_dfu_update_process(dfu_update_status_t update_status);

/**@brief Function for continuing the Device Firmware Update of a SoftDevice.
 *
 * @retval     NRF_SUCCESS If the final stage of SoftDevice update was successful.
 */
uint32_t bootloader_dfu_sd_update_continue (void);

uint32_t bootloader_dfu_sd_update_validate(void);

uint32_t bootloader_dfu_bl_update_continue(void);

uint32_t bootloader_dfu_ap_update_continue(void);
/**@brief Function for finalizing the Device Firmware Update of a SoftDevice.
 *
 * @retval     NRF_SUCCESS If the final stage of SoftDevice update was successful.
 */
uint32_t bootloader_dfu_sd_update_finalize(void);

/**@brief Function for writing word into flash.
 *
 * @param[in]  p_dst        Address to write.
 * @param[in]  data         Data to write.
 * @retval     NRF_SUCCESS  If the write operation was successful.
 */
uint32_t blocking_flash_word_write(uint32_t * const p_dst, uint32_t data);



#ifdef __cplusplus
}
#endif

#endif // BOOTLOADER_H__

/**@} */
