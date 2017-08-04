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

#ifndef DEBUG_PIN_H__
#define DEBUG_PIN_H__

#include "nrf.h"
//#include "nrf51_bitfields.h"

#ifdef __cplusplus
extern "C" {
#endif

/**********************************************************************************/
/* Comment this out to DISABLE all Debugging pins especially on official releases.*/
//#define DEBUGGING_PINS_ENABLE

#if defined (DEBUGGING_PINS_ENABLE)
   #define DEBUG_DFU_BOOTLOADER
//   #define DEBUG_UART_STACKCHECK

#define DEBUG_USE_UART_OUT
#endif // STACK_DEBUGGING_PINS_ENABLE

/**********************************************************************************/

#define DEBUG_PIN_ON(pin)        { NRF_GPIO->OUTSET = (1UL << (pin)); }
#define DEBUG_PIN_OFF(pin)       { NRF_GPIO->OUTCLR = (1UL << (pin)); }
#define DEBUG_PIN_RISE(pin)      { NRF_GPIO->OUTCLR = (1UL << (pin)); NRF_GPIO->OUTSET = (1UL << (pin));}
#define DEBUG_PIN_FALL(pin)      { NRF_GPIO->OUTSET = (1UL << (pin)); NRF_GPIO->OUTCLR = (1UL << (pin));}

/*DEBUG OUT, !!!!WARNING THIS USES UART0!!!! */

#if defined (DEBUG_USE_UART_OUT)

#define DEBUG_UART_INIT(pin)        NRF_UART0->PSELRXD = 0xFFFFFFFF;\
                                    NRF_UART0->PSELTXD = pin;\
                                    NRF_UART0->CONFIG = 0x00;\
                                    NRF_UART0->BAUDRATE = UART_BAUDRATE_BAUDRATE_Baud1M;\
                                    NRF_UART0->ENABLE = UART_ENABLE_ENABLE_Enabled << UART_ENABLE_ENABLE_Pos;\
                                    NRF_UART0->TASKS_STARTTX = 1;
#define DEBUG_UART_OUT(val)         NRF_UART0->TXD = val

#else

#define DEBUG_UART_INIT(pin)
#define DEBUG_UART_OUT(val)

#endif

//////////////////////////////////////////////////////////////////
// CONFIGURE DEBUG PINS HERE
//////////////////////////////////////////////////////////////////

/*****************************************
 * APP_DFU
 */
#if defined (DEBUG_DFU_BOOTLOADER)

/*starts at 24 ends at 32*/
#define DBG_DFU_BOOTLOADER_PATH             24
#define DBG_DFU_FLASH_IMAGE_STATUS          25
#define DBG_DFU_CKPT_PINC                   25
#define DBG_DFU_CKPT_PINA                   28
#define DBG_DFU_CKPT_PINB                   29
//#define DBG_DFU_FLASH_PIN                   29
//    #define DBG_DFU_FLASH_ERASE                 8
//    #define DBG_DFU_FLASH_WRITE                 9
//    #define DBG_DFU_FLASH_RESP                  10
//    #define DBG_DFU_FLASH_PSTORAGE_CB           11
//    #define DBG_DFU_FLASH_CB_HANDLER            12

#define DBG_DFU_UART_OUT_PIN                30          //antfs_event_process
#define DBG_UART_DFU_ANTFS_EVENT_PIN        30
#define DBG_UART_ANTFS_DFU_STATE_PIN        30
#define DBG_UART_DFU_DATA_OFFSET_PIN      30


   #define DBG_PIN_DIR_INIT               { NRF_GPIO->DIRSET = 0xFFFF0000;\
                                             DEBUG_UART_INIT(DBG_DFU_UART_OUT_PIN);}
#endif //DEBUG_DFU_BOOTLOADER
/*
 * APP_DFU END
 *****************************************/

#ifndef DBG_PIN_DIR_INIT
   #define DBG_PIN_DIR_INIT
#endif

void stack_debug_Manchester_Start(uint8_t ucPin, uint8_t ucCode);
void stack_debug_Manchester_Stop(uint8_t ucPin);


#ifdef __cplusplus
}
#endif

#endif /* DEBUG_PIN_H_ */
