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

#ifndef ANT_BOOT_SETTINGS_API_H__
#define ANT_BOOT_SETTINGS_API_H__

#include <stdint.h>
#include "ant_boot_settings.h"

#ifdef __cplusplus
extern "C" {
#endif
/*
*
* A soft-reset(NVIC_SystemReset()) must be executed after the information for the bootloader has been filled in.
* i.e.
*  {
*     ant_boot_settings_t ant_boot_settings;
*
*     ant_boot_settings_clear(&ant_boot_settings);                                  // Clears and set FFs to the memory block
*     ant_boot_settings.app_version[0] = version[0];                               // Start filling parameters
*     ant_boot_settings.app_version[1] = version[1];
*     ant_boot_settings.app_version[2] = version[2];
*     ant_boot_settings_save(&ant_boot_settings);
*     ant_boot_settings_validate(1);                                                // Sets in the magic number. Must be done last before the reset!!!
*     NVIC_SystemReset();                                                           // Do the soft reset
*  }
*/
void ant_boot_settings_event (uint32_t ulEvent);
void ant_boot_settings_get(const ant_boot_settings_t ** pp_boot_settings);
uint32_t ant_boot_settings_clear(ant_boot_settings_t * boot_settings);
uint32_t ant_boot_settings_save(ant_boot_settings_t * boot_settings);
void ant_boot_settings_validate(uint8_t enter_boot_mode);


#ifdef __cplusplus
}
#endif

#endif //ANT_BOOT_SETTINGS_API_H__
