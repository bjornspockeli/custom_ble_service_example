/**
 * This software is subject to the ANT+ Shared Source License
 * www.thisisant.com/swlicenses
 * Copyright (c) Dynastream Innovations, Inc. 2015
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

#ifndef ANT_DFU_CONSTRAINS_H__
#define ANT_DFU_CONSTRAINS_H__

#ifdef __cplusplus
extern "C" {
#endif

/**@file
 *
 * @defgroup ant_dfu_constrains Memory constraints for ANT DFU
 * @{
 *
 * @ingroup nrf_dfu
 *
 */

#ifdef NRF51 // nrf51 @ S210
    /** End of nRF51 flash */
    #define NRF5x_FLASH_END                 0x00040000UL

    /** This field should correspond to the start address of the bootloader, found in the
        UICR.BOOTLOADERADDR, 0x10001014, register. This value is used for a sanity check,
        so the bootloader will fail immediately if this value differs from the runtime value.
        The value is used to determine the maximum DFU region size. */
    #define BOOTLOADER_REGION_START         0x0003B800

    /** Page location of the bootloader settings address. */
    #define BOOTLOADER_SETTINGS_ADDRESS     0x0003FC00

    /** Size of a flash codepage. Used for size of the reserved flash space in the bootloader
        region. Will be runtime checked against NRF_UICR->CODEPAGESIZE to ensure the region is
        correct. */
    #define CODE_PAGE_SIZE                  1024

#elif defined(NRF52) // nrf52 @ S212 and S332
    /** End of nRF52 flash */
    #define NRF5x_FLASH_END                 0x00080000UL

    /** This field should correspond to the start address of the bootloader, found in the
        UICR.BOOTLOADERADDR, 0x10001014, register. This value is used for a sanity check,
        so the bootloader will fail immediately if this value differs from the runtime value.
        The value is used to determine the maximum DFU region size. */
    #define BOOTLOADER_REGION_START         0x00079000

    /** Page location of the bootloader settings address. */
    #define BOOTLOADER_SETTINGS_ADDRESS     (NRF5x_FLASH_END - CODE_PAGE_SIZE)

    /** The sd_mbr_command call may require parameters to be retained in a separate flash page provided by the application.
        The uicr register UICR.NRFFW[1] must be set to an address corresponding to this page in the application flash space. */
    #define BOOTLOADER_MBR_RETAINING_PAGE_ADDRESS     (BOOTLOADER_SETTINGS_ADDRESS - CODE_PAGE_SIZE)

    /** Size of a flash codepage. Used for size of the reserved flash space in the bootloader
        region. Will be runtime checked against NRF_UICR->CODEPAGESIZE to ensure the region is
        correct. */
    #define CODE_PAGE_SIZE                  4096

#else
    #error Unknown platform for ANT DFU
#endif

/**@} */


#ifdef __cplusplus
}
#endif

#endif //ANT_DFU_CONSTRAINS_H__
