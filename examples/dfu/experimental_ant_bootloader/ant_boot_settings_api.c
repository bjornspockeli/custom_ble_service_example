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

#include <stdint.h>
#include <string.h>

#include "nrf.h"
#include "ant_interface.h"
#include "ant_parameters.h"
#include "nrf_soc.h"
#include "nrf_sdm.h"


#include "bootloader_types.h"
#include "ant_boot_settings_api.h"

uint8_t  m_ant_boot_settings[ANT_BOOT_SETTINGS_SIZE] __attribute__((at(ANT_BOOT_SETTINGS_LOCATION)));          /**< This variable reserves a codepage for bootloader specific settings, to ensure the compiler doesn't locate any code or variables at his location. */

volatile uint8_t mb_flash_busy = false;
/*
 * sd_flash_page_erase() and sd_flash_write() generates an event at SD_EVT_IRQHandler
 * Please include run this function inside SD_EVT_IRQHandler
 *
 */
void ant_boot_settings_event (uint32_t ulEvent)
{
    if ((ulEvent == NRF_EVT_FLASH_OPERATION_SUCCESS) || (ulEvent == NRF_EVT_FLASH_OPERATION_ERROR))
    {
        mb_flash_busy = false;
    }
}


uint32_t ant_boot_settings_save(ant_boot_settings_t * boot_settings)
{
    uint32_t ulErrorCode = NRF_SUCCESS;

    mb_flash_busy = true;
    ulErrorCode = sd_flash_write((uint32_t*)ANT_BOOT_SETTINGS_LOCATION , (uint32_t*)boot_settings, ANT_BOOT_SETTINGS_SIZE / 4);
    if (ulErrorCode == NRF_SUCCESS)
    {
        while (mb_flash_busy); // wait until it is done
    }
    else
    {
        return ulErrorCode;
    }

    return ulErrorCode;
}

uint32_t ant_boot_settings_clear(ant_boot_settings_t * boot_settings)
{
    uint32_t ulErrorCode = NRF_SUCCESS;

    // Clears \ presets the bootloader_settings memory
    memset(boot_settings, 0xFF, sizeof(ant_boot_settings_t));

    // Erases entire bootloader_settings in flash
    mb_flash_busy = true;
    ulErrorCode = sd_flash_page_erase(FLASH_LAST_PAGE); // last flash page
    if (ulErrorCode == NRF_SUCCESS)
    {
        while (mb_flash_busy); // wait until it is done
    }
    else
    {
        return ulErrorCode;
    }

    return ulErrorCode;
}

void ant_boot_settings_get(const ant_boot_settings_t ** pp_boot_settings)
{
    // Read only pointer to antfs boot settings in flash.
    static ant_boot_settings_t const * const p_boot_settings =
        (ant_boot_settings_t *)&m_ant_boot_settings[0];

    *pp_boot_settings = p_boot_settings;
}

void ant_boot_settings_validate(uint8_t enter_boot_mode)
{
    uint32_t ulErrorCode = NRF_SUCCESS;
    uint32_t param_flags;

    if (enter_boot_mode)
    {
        param_flags = 0xFFFFFFFC;
    }
    else
    {
        param_flags = 0xFFFFFFFE;
    }

    mb_flash_busy = true;
    ulErrorCode = sd_flash_write((uint32_t*)ANT_BOOT_PARAM_FLAGS_BASE , (uint32_t*)&param_flags, 1);
    if (ulErrorCode == NRF_SUCCESS)
    {
        while (mb_flash_busy); // wait until it is done
    }
}


