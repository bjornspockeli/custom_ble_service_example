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

/*
 * Before compiling this example for NRF52, complete the following steps:
 * - Download the S212 SoftDevice from <a href="https://www.thisisant.com/developer/components/nrf52832" target="_blank">thisisant.com</a>.
 * - Extract the downloaded zip file and copy the S212 SoftDevice headers to <tt>\<InstallFolder\>/components/softdevice/s212/headers</tt>.
 * If you are using Keil packs, copy the files into a @c headers folder in your example folder.
 * - Make sure that @ref ANT_LICENSE_KEY in @c nrf_sdm.h is uncommented.
 */

#include "dfu.h"
#include "dfu_transport.h"
#include "bootloader.h"
#include "bootloader_util.h"
#include <stdint.h>
#include <string.h>
#include <stddef.h>
#include "nordic_common.h"
#include "nrf.h"
#include "app_error.h"
#include "nrf_gpio.h"
#include "nrf_soc.h"
#include "nrf_delay.h"
#include "ant_interface.h"
#include "ant_parameters.h"
#include "ant_error.h"
#include "nrf.h"
#include "app_scheduler.h"
#include "app_timer.h"
#include "nrf_error.h"
#include "boards.h"
#include "nrf_sdh.h"
#include "nrf_sdh_ant.h"
#include "ant_boot_settings_api.h"
#include "antfs_ota.h"
#if !defined (S210_V3_STACK)
#include "nrf_mbr.h"
#endif // !S210_V3_STACK
#include "debug_pin.h"

#define ENABLE_BUTTON // include button detection
//#define ENABLE_IO_LED // include LED status on N5DK1 i/o board

#if defined (ENABLE_BUTTON)
   #if defined (BOARD_N5DK1)
      #define BOOTLOADER_BUTTON_PIN     BUTTON_D                    /**< Button used to enter SW update mode. */
   #else
      #define BOOTLOADER_BUTTON_PIN     BUTTON_1                    /**< Button used to enter SW update mode. */
   #endif
#endif

#if defined (ENABLE_IO_LED)
   #define BOOTLOADER_ERROR_LED         LED_C                       /**< N5DK Leds, set=led off, clr=led on  */
   #define BOOTLOADER_ACTIVE_LED        LED_D
#endif // ENABLE_IO_LED

#define SCHED_MAX_EVENT_DATA_SIZE       NRF_SDH_ANT_EVT_BUF_SIZE    /**< Maximum size of scheduler events. */

#define SCHED_QUEUE_SIZE                20                          /**< Maximum number of events in the scheduler queue. */


/**@brief Function for error handling, which is called when an error has occurred.
 *
 * @warning This handler is an example only and does not fit a final product. You need to analyze
 *          how your product is supposed to react in case of error.
 *
 * @param[in] error_code  Error code supplied to the handler.
 * @param[in] line_num    Line number where the handler is called.
 * @param[in] p_file_name Pointer to the file name.
 */
#if defined (DEBUG_DFU_BOOTLOADER)
uint32_t error_code_;
uint32_t line_num_;
const uint8_t * p_file_name_;
#endif // DEBUG_DFU_BOOTLOADER

void app_error_fault_handler(uint32_t id, uint32_t pc, uint32_t info)
{
#if defined (DEBUG_DFU_BOOTLOADER)
    app_error_save_and_stop(id, pc, info);
#endif // DEBUG_DFU_BOOTLOADER

#if defined (ENABLE_IO_LED)
//    nrf_gpio_pin_set(BOOTLOADER_ERROR_LED);
#endif // ENABLE_IO_LED

    // This call can be used for debug purposes during application development.
    // On assert, the system can only recover on reset.
    NVIC_SystemReset();
}

void HardFault_Handler(uint32_t ulProgramCounter, uint32_t ulLinkRegister)
{
   (void)ulProgramCounter;
   (void)ulLinkRegister;

   NVIC_SystemReset();
}

/**@brief Callback function for asserts in the SoftDevice.
 *
 * @details This function will be called in case of an assert in the SoftDevice.
 *
 * @warning This handler is an example only and does not fit a final product. You need to analyze
 *          how your product is supposed to react in case of Assert.
 * @warning On assert from the SoftDevice, the system can only recover on reset.
 *
 * @param[in] line_num    Line number of the failing ASSERT call.
 * @param[in] file_name   File name of the failing ASSERT call.
 */
void assert_nrf_callback(uint16_t line_num, const uint8_t * p_file_name)
{
    app_error_handler(0xDEADBEEF, line_num, p_file_name);
}


#if defined (ENABLE_IO_LED)
/**@brief Function for initialization of LEDs.
 *
 * @details Initializes all LEDs used by the application.
 */
static void leds_init(void)
{
    nrf_gpio_cfg_output(LED_A);
    nrf_gpio_cfg_output(LED_B);
    nrf_gpio_cfg_output(LED_C);
    nrf_gpio_cfg_output(LED_D);

     // turn on all leds
     nrf_gpio_pin_clear(LED_A);
     nrf_gpio_pin_clear(LED_B);
     nrf_gpio_pin_clear(LED_C);
     nrf_gpio_pin_clear(LED_D);
}
#endif // ENABLE_IO_LED


#if defined (ENABLE_IO_LED)
/**@brief Function for clearing the LEDs.
 *
 * @details Clears all LEDs used by the application.
 */
static void leds_off(void)
{
    nrf_gpio_pin_set(LED_A); // unused
    nrf_gpio_pin_set(LED_B); // unused
    nrf_gpio_pin_set(LED_C);
    nrf_gpio_pin_set(LED_D);
}
#endif // ENABLE_IO_LED


/**@brief Function for the Timer initialization.
 *
 * @details Initializes the timer module.
 */
static void timers_init(void)
{
    ret_code_t err_code = app_timer_init();
    APP_ERROR_CHECK(err_code);
}


#if defined (ENABLE_BUTTON)
/**@brief Function for initializing the button module.
 */
static void buttons_init(void)
{
    nrf_gpio_cfg_sense_input(BOOTLOADER_BUTTON_PIN,
                             BUTTON_PULL,
                             NRF_GPIO_PIN_SENSE_LOW);
}
#endif // ENABLE_BUTTON


/**@brief Function for initializing the ANT stack. */
static void ant_stack_init(void)
{
    ret_code_t err_code;

    err_code = nrf_sdh_enable_request();
    APP_ERROR_CHECK(err_code);

    // Enable ANT stack.
    err_code = nrf_sdh_ant_enable();
    APP_ERROR_CHECK(err_code);
}


/**@brief Function for event scheduler initialization.
 */
static void scheduler_init(void)
{
    APP_SCHED_INIT(SCHED_MAX_EVENT_DATA_SIZE, SCHED_QUEUE_SIZE);
}

static uint32_t enter_boot_get (void )
{
    uint32_t val = PARAM_FLAGS_ENTER_BOOT_BypassInit;

    if (((*ANT_BOOT_PARAM_FLAGS & PARAM_FLAGS_PARAM_VALID_Msk) >> PARAM_FLAGS_PARAM_VALID_Pos) == PARAM_FLAGS_PARAM_VALID_True )
    {
        val = (*ANT_BOOT_PARAM_FLAGS & PARAM_FLAGS_ENTER_BOOT_Msk) >> PARAM_FLAGS_ENTER_BOOT_Pos;
    }

    return val;
}

static void enter_boot_set (uint32_t value)
{
    uint32_t ant_boot_param_flags = *ANT_BOOT_PARAM_FLAGS;

    uint32_t enter_boot = (ant_boot_param_flags >> PARAM_FLAGS_ENTER_BOOT_Pos) & PARAM_FLAGS_ENTER_BOOT_Msk;
    if (enter_boot == value)
    {
        return; // no need to rewrite the same value.
    }

    ant_boot_param_flags &= ~PARAM_FLAGS_ENTER_BOOT_Msk;
    ant_boot_param_flags |= value << PARAM_FLAGS_ENTER_BOOT_Pos;

    uint32_t err_code = blocking_flash_word_write(ANT_BOOT_PARAM_FLAGS, ant_boot_param_flags);
    APP_ERROR_CHECK(err_code);
}

static void enter_boot_update (void)
{
    const bootloader_settings_t * p_bootloader_settings;

    bootloader_util_settings_get(&p_bootloader_settings);

   if (p_bootloader_settings->ap_image.st.bank == NEW_IMAGE_BANK_0 || p_bootloader_settings->ap_image.st.bank == NEW_IMAGE_BANK_1)
   {
       enter_boot_set(PARAM_FLAGS_ENTER_BOOT_BypassDone);
   }
   else
   {
       if (p_bootloader_settings->valid_app != BOOTLOADER_SETTINGS_INVALID_APPLICATION)
       {
           enter_boot_set(PARAM_FLAGS_ENTER_BOOT_BypassDone);
           return;
       }
       enter_boot_set(PARAM_FLAGS_ENTER_BOOT_EnterBoot);
   }

   // If the current application has been invalidated, then application's self protection is of no use now.
   // Lets clear it.
   if (p_bootloader_settings->valid_app == BOOTLOADER_SETTINGS_INVALID_APPLICATION)
   {
       if (*ANT_BOOT_APP_SIZE != APP_SIZE_Empty)
       {
            uint32_t err_code = blocking_flash_word_write(ANT_BOOT_APP_SIZE, APP_SIZE_Clear);
            APP_ERROR_CHECK(err_code);
       }
   }
}

/**@brief Function for application main entry.
 */
int main(void)
{
    uint32_t err_code;
    bool     bootloader_is_pushed = false;

#if defined (ENABLE_IO_LED)
    leds_init();
#endif // ENABLE_IO_LED
#if defined (ENABLE_BUTTON)
    buttons_init();
#endif

#if defined (DEBUG_DFU_BOOTLOADER)
    NRF_GPIO->DIRSET = 0x40000908;  //stack debugging
    DBG_PIN_DIR_INIT;
#endif //DEBUG_DFU_BOOTLOADER
#if defined (DBG_DFU_BOOTLOADER_PATH)
    DEBUG_PIN_FALL(DBG_DFU_BOOTLOADER_PATH);
#endif //DBG_DFU_BOOTLOADER_PATH

    // This check ensures that the defined fields in the bootloader corresponds with actual
    // setting in the chip.
    APP_ERROR_CHECK_BOOL(*((uint32_t *)NRF_UICR_BOOT_START_ADDRESS) == BOOTLOADER_REGION_START);
    APP_ERROR_CHECK_BOOL(NRF_FICR->CODEPAGESIZE == CODE_PAGE_SIZE);

#if !defined (S210_V3_STACK)
    sd_mbr_command_t com = {SD_MBR_COMMAND_INIT_SD, };

    err_code = sd_mbr_command(&com);
    APP_ERROR_CHECK(err_code);

    err_code = sd_softdevice_vector_table_base_set(BOOTLOADER_REGION_START);
    APP_ERROR_CHECK(err_code);

    err_code = bootloader_dfu_sd_update_continue();
    APP_ERROR_CHECK(err_code);

    err_code = bootloader_dfu_bl_update_continue();
    APP_ERROR_CHECK(err_code);
#endif // !S210_V3_STACK

    // Initialize.
    timers_init();
    (void)bootloader_init();
    ant_stack_init();
    scheduler_init();

#if defined (ENABLE_BUTTON)
    // Push button switch
    bootloader_is_pushed = ((nrf_gpio_pin_read(BOOTLOADER_BUTTON_PIN) == 0) ? true: false);
    if (bootloader_is_pushed)
    {
        enter_boot_set(PARAM_FLAGS_ENTER_BOOT_EnterBoot);
    }
#endif // ENABLE_BUTTON

    if ((enter_boot_get() == PARAM_FLAGS_ENTER_BOOT_EnterBoot)  ||
        (bootloader_is_pushed)                                  ||
        (!bootloader_app_is_valid(DFU_BANK_0_REGION_START)))
    {
        #if defined (DBG_DFU_BOOTLOADER_PATH)
        DEBUG_PIN_FALL(DBG_DFU_BOOTLOADER_PATH);DEBUG_PIN_FALL(DBG_DFU_BOOTLOADER_PATH);
        #endif //DBG_DFU_BOOTLOADER_PATH

#if defined (ENABLE_IO_LED)
        leds_off();
        nrf_gpio_pin_clear(BOOTLOADER_ACTIVE_LED);
#endif // ENABLE_IO_LED

        // Initiate an update of the firmware.
        err_code = bootloader_dfu_start();
        APP_ERROR_CHECK(err_code);

        enter_boot_update();
    }

#if defined (ENABLE_IO_LED)
        leds_off();
#endif // ENABLE_IO_LED

    err_code = bootloader_dfu_ap_update_continue();
    APP_ERROR_CHECK(err_code);

    if (bootloader_app_is_valid(DFU_BANK_0_REGION_START))
    {
        #if defined (DBG_DFU_BOOTLOADER_PATH)
        DEBUG_PIN_FALL(DBG_DFU_BOOTLOADER_PATH);DEBUG_PIN_FALL(DBG_DFU_BOOTLOADER_PATH);DEBUG_PIN_FALL(DBG_DFU_BOOTLOADER_PATH);
        #endif //DBG_DFU_BOOTLOADER_PATH
        // Select a bank region to use as application region.
        // @note: Only applications running from DFU_BANK_0_REGION_START is supported.
        bootloader_app_start(DFU_BANK_0_REGION_START);
    }

#if defined (DBG_DFU_BOOTLOADER_PATH)
DEBUG_PIN_FALL(DBG_DFU_BOOTLOADER_PATH);DEBUG_PIN_FALL(DBG_DFU_BOOTLOADER_PATH);DEBUG_PIN_FALL(DBG_DFU_BOOTLOADER_PATH);DEBUG_PIN_FALL(DBG_DFU_BOOTLOADER_PATH);
#endif //DBG_DFU_BOOTLOADER_PATH
    NVIC_SystemReset();
}
