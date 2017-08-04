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
#include <stdint.h>
#include <stdbool.h>
#include <stddef.h>

#include "nrf.h"
#include "app_util_platform.h"
#include "nrf_drv_usbd.h"
#include "nrf_drv_clock.h"
#include "nrf_gpio.h"
#include "nrf_drv_power.h"

#include "app_timer.h"
#include "app_usbd.h"
#include "app_usbd_core.h"
#include "app_usbd_hid_generic.h"
#include "app_usbd_hid_mouse.h"
#include "app_usbd_hid_kbd.h"
#include "app_error.h"
#include "bsp.h"


#include "nrf_log.h"
#include "nrf_log_ctrl.h"
#include "nrf_log_default_backends.h"

/**
 * @brief Enable USB power detection
 */
#ifndef USBD_POWER_DETECTION
#define USBD_POWER_DETECTION true
#endif

/**
 * @brief HID generic class interface number.
 * */
#define HID_GENERIC_INTERFACE  0

/**
 * @brief HID generic class endpoint number.
 * */
#define HID_GENERIC_EPIN       NRF_DRV_USBD_EPIN1

/**
 * @brief Mouse speed (value sent via HID when board button is pressed).
 * */
#define CONFIG_MOUSE_MOVE_SPEED (3)

/**
 * @brief Mouse move repeat time in milliseconds
 */
#define CONFIG_MOUSE_MOVE_TIME_MS (5)


/* GPIO used as LED & buttons in this example */
#define LED_USB_START    (BSP_BOARD_LED_0)
#define LED_HID_REP_IN   (BSP_BOARD_LED_2)

#define BTN_MOUSE_X_POS  0
#define BTN_MOUSE_Y_POS  1
#define BTN_MOUSE_LEFT   2
#define BTN_MOUSE_RIGHT  3

/**
 * @brief Left button mask in buttons report
 */
#define HID_BTN_LEFT_MASK  (1U << 0)

/**
 * @brief Right button mask in buttons report
 */
#define HID_BTN_RIGHT_MASK (1U << 1)

/* HID report layout */
#define HID_BTN_IDX   0 /**< Button bit mask position */
#define HID_X_IDX     1 /**< X offset position */
#define HID_Y_IDX     2 /**< Y offset position */
#define HID_W_IDX     3 /**< Wheel position  */
#define HID_REP_SIZE  4 /**< The size of the report */

/**
 * @brief Number of reports defined in report descriptor.
 */
#define REPORT_IN_QUEUE_SIZE    1

/**
 * @brief Size of maximum output report. HID generic class will reserve
 *        this buffer size + 1 memory space. */
#define REPORT_OUT_MAXSIZE  0

/**
 * @brief HID generic class endpoints count.
 * */
#define HID_GENERIC_EP_COUNT  1

/**
 * @brief List of HID generic class endpoints.
 * */
#define ENDPOINT_LIST()                                      \
(                                                            \
        HID_GENERIC_EPIN                                     \
)

/**
 * @brief Additional key release events
 *
 * This example needs to process release events of used buttons
 */
enum {
    BSP_USER_EVENT_RELEASE_0 = BSP_EVENT_KEY_LAST + 1, /**< Button 0 released */
    BSP_USER_EVENT_RELEASE_1,                          /**< Button 1 released */
    BSP_USER_EVENT_RELEASE_2,                          /**< Button 2 released */
    BSP_USER_EVENT_RELEASE_3,                          /**< Button 3 released */
    BSP_USER_EVENT_RELEASE_4,                          /**< Button 4 released */
    BSP_USER_EVENT_RELEASE_5,                          /**< Button 5 released */
    BSP_USER_EVENT_RELEASE_6,                          /**< Button 6 released */
    BSP_USER_EVENT_RELEASE_7,                          /**< Button 7 released */
};

/**
 * @brief HID generic mouse action types
 */
typedef enum {
    HID_GENERIC_MOUSE_X,
    HID_GENERIC_MOUSE_Y,
    HID_GENERIC_MOUSE_BTN_LEFT,
    HID_GENERIC_MOUSE_BTN_RIGHT,
} hid_generic_mouse_action_t;

/**
 * @brief User event handler.
 * */
static void hid_user_ev_handler(app_usbd_class_inst_t const * p_inst,
                                app_usbd_hid_user_event_t event);

/**
 * @brief Reuse HID mouse report descriptor for HID generic class
 */
static const uint8_t m_hid_generic_rep_descriptor[] = APP_USBD_HID_MOUSE_REPORT_DSC_BUTTON(2);

/**
 * @brief HID generic class descriptors.
 * */
static const uint8_t m_hid_generic_class_descriptors[] ={
        APP_USBD_HID_GENERIC_INTERFACE_DSC(HID_GENERIC_INTERFACE,
                                           HID_GENERIC_EP_COUNT,
                                           APP_USBD_HID_SUBCLASS_BOOT,
                                           APP_USBD_HID_PROTO_MOUSE)
        APP_USBD_HID_GENERIC_HID_DSC(m_hid_generic_rep_descriptor)
        APP_USBD_HID_GENERIC_EP_DSC(HID_GENERIC_EPIN)
};

/*lint -save -e26 -e64 -e123 -e505 -e651*/

/**
 * @brief Global HID generic instance
 */
APP_USBD_HID_GENERIC_GLOBAL_DEF(m_app_hid_generic,
                                HID_GENERIC_INTERFACE,
                                hid_user_ev_handler,
                                ENDPOINT_LIST(),
                                m_hid_generic_class_descriptors,
                                m_hid_generic_rep_descriptor,
                                REPORT_IN_QUEUE_SIZE,
                                REPORT_OUT_MAXSIZE);

/*lint -restore*/


/**
 * @brief Mouse state
 *
 * Current mouse status
 */
struct
{
    int16_t acc_x;    /**< Accumulated x state */
    int16_t acc_y;    /**< Accumulated y state */
    uint8_t btn;      /**< Current btn state */
    uint8_t last_btn; /**< Last transfered button state */
}m_mouse_state;

/**
 * @brief Mark the ongoing transmission
 *
 * Marks that the report buffer is busy and cannot be used until transmission finishes
 * or invalidates (by USB reset or suspend event).
 */
static bool m_report_pending;

/**
 * @brief  USB connection status
 */
static bool m_usb_connected = false;

/**
 * @brief Timer to repeat mouse move
 */
APP_TIMER_DEF(m_mouse_move_timer);

/**
 * @brief Get maximal allowed accumulated value
 *
 * Function gets maximal value from the accumulated input.
 * @sa m_mouse_state::acc_x, m_mouse_state::acc_y
 */
static int8_t hid_acc_for_report_get(int16_t acc)
{
    if(acc > INT8_MAX)
    {
        return INT8_MAX;
    }
    else if(acc < INT8_MIN)
    {
        return INT8_MIN;
    }
    else
    {
        return (int8_t)(acc);
    }
}

/**
 * @brief Internal function that process mouse state
 *
 * This function checks current mouse state and tries to send
 * new report if required.
 * If report sending was successful it clears accumulated positions
 * and mark last button state that was transfered.
 */
static void hid_generic_mouse_process_state(void)
{
    if (m_report_pending)
        return;
    if ((m_mouse_state.acc_x != 0) ||
        (m_mouse_state.acc_y != 0) ||
        (m_mouse_state.btn != m_mouse_state.last_btn))
    {
        ret_code_t ret;
        static uint8_t report[HID_REP_SIZE];
        /* We have some status changed that we need to transfer */
        report[HID_BTN_IDX] = m_mouse_state.btn;
        report[HID_X_IDX]   = (uint8_t)hid_acc_for_report_get(m_mouse_state.acc_x);
        report[HID_Y_IDX]   = (uint8_t)hid_acc_for_report_get(m_mouse_state.acc_y);
        /* Start the transfer */
        ret = app_usbd_hid_generic_in_report_set(
            &m_app_hid_generic,
            report,
            sizeof(report));
        if (ret == NRF_SUCCESS)
        {
            m_report_pending = true;
            m_mouse_state.last_btn = report[HID_BTN_IDX];
            CRITICAL_REGION_ENTER();
            /* This part of the code can fail if interrupted by BSP keys processing.
             * Lock interrupts to be safe */
            m_mouse_state.acc_x   -= (int8_t)report[HID_X_IDX];
            m_mouse_state.acc_y   -= (int8_t)report[HID_Y_IDX];
            CRITICAL_REGION_EXIT();
        }
    }
}

/**
 * @brief HID generic IN report send handling
 * */
static void hid_generic_mouse_action(hid_generic_mouse_action_t action, int8_t param)
{
    CRITICAL_REGION_ENTER();
    /*
     * Update mouse state
     */
    switch (action)
    {
        case HID_GENERIC_MOUSE_X:
            m_mouse_state.acc_x += param;
            break;
        case HID_GENERIC_MOUSE_Y:
            m_mouse_state.acc_y += param;
            break;
        case HID_GENERIC_MOUSE_BTN_RIGHT:
            if(param == 1)
            {
                m_mouse_state.btn |= HID_BTN_RIGHT_MASK;
            }
            else
            {
                m_mouse_state.btn &= ~HID_BTN_RIGHT_MASK;
            }
            break;
        case HID_GENERIC_MOUSE_BTN_LEFT:
            if(param == 1)
            {
                m_mouse_state.btn |= HID_BTN_LEFT_MASK;
            }
            else
            {
                m_mouse_state.btn &= ~HID_BTN_LEFT_MASK;
            }
            break;
    }
    CRITICAL_REGION_EXIT();
}

/**
 * @brief Class specific event handler.
 *
 * @param p_inst    Class instance.
 * @param event     Class specific event.
 * */
static void hid_user_ev_handler(app_usbd_class_inst_t const * p_inst,
                                app_usbd_hid_user_event_t event)
{
    switch (event)
    {
        case APP_USBD_HID_USER_EVT_OUT_REPORT_READY:
        {
            /* No output report defined for this example.*/
            ASSERT(0);
            break;
        }
        case APP_USBD_HID_USER_EVT_IN_REPORT_DONE:
        {
            m_report_pending = false;
            hid_generic_mouse_process_state();
            bsp_board_led_invert(LED_HID_REP_IN);
            break;
        }
        case APP_USBD_HID_USER_EVT_SET_BOOT_PROTO:
        {
            NRF_LOG_INFO("SET_BOOT_PROTO");
            break;
        }
        case APP_USBD_HID_USER_EVT_SET_REPORT_PROTO:
        {
            NRF_LOG_INFO("SET_REPORT_PROTO");
            break;
        }
        default:
            break;
    }
}

/**
 * @brief USBD library specific event handler.
 *
 * @param event     USBD library event.
 * */
static void usbd_user_ev_handler(app_usbd_event_type_t event)
{
    switch (event)
    {
        case APP_USBD_EVT_DRV_SOF:
            break;
        case APP_USBD_EVT_DRV_RESET:
            m_report_pending = false;
            break;
        case APP_USBD_EVT_DRV_SUSPEND:
            m_report_pending = false;
            app_usbd_suspend_req(); // Allow the library to put the peripheral into sleep mode
            bsp_board_leds_off();
            break;
        case APP_USBD_EVT_DRV_RESUME:
            bsp_board_led_on(LED_USB_START);
            break;
        case APP_USBD_EVT_STARTED:
            m_report_pending = false;
            bsp_board_led_on(LED_USB_START);
            break;
        case APP_USBD_EVT_STOPPED:
            app_usbd_disable();
            bsp_board_leds_off();
            break;
        default:
            break;
    }
}

static void power_usb_event_handler(nrf_drv_power_usb_evt_t event)
{
    switch (event)
    {
        case NRF_DRV_POWER_USB_EVT_DETECTED:
            NRF_LOG_INFO("USB power detected");

            if (!nrf_drv_usbd_is_enabled())
            {
                app_usbd_enable();
            }
            break;
        case NRF_DRV_POWER_USB_EVT_REMOVED:
            NRF_LOG_INFO("USB power removed");
            m_usb_connected = false;
            break;
        case NRF_DRV_POWER_USB_EVT_READY:
            NRF_LOG_INFO("USB ready");
            m_usb_connected = true;
            break;
        default:
            ASSERT(false);
    }
}


static void usb_start(void)
{
    if (USBD_POWER_DETECTION)
    {
        static const nrf_drv_power_usbevt_config_t config =
        {
            .handler = power_usb_event_handler
        };

        ret_code_t ret;
        ret = nrf_drv_power_usbevt_init(&config);
        APP_ERROR_CHECK(ret);
    }
    else
    {
        NRF_LOG_INFO("No USB power detection enabled\r\nStarting USB now");

        app_usbd_enable();
        app_usbd_start();
        m_usb_connected = true;
    }
}

static bool usb_connection_handle(bool last_usb_conn_status)
{
    if (last_usb_conn_status != m_usb_connected)
    {
        last_usb_conn_status = m_usb_connected;
        m_usb_connected ? app_usbd_start() : app_usbd_stop();
    }

    return last_usb_conn_status;
}

static void init_power_clock(void)
{
    ret_code_t ret;
    ret = nrf_drv_clock_init();
    APP_ERROR_CHECK(ret);
    ret = nrf_drv_power_init(NULL);
    APP_ERROR_CHECK(ret);

    nrf_drv_clock_lfclk_request(NULL);
    while(!nrf_drv_clock_lfclk_is_running())
    {
        /* Just waiting */
    }

    ret = app_timer_init();
    APP_ERROR_CHECK(ret);

    /* Avoid warnings if assertion is disabled */
    UNUSED_VARIABLE(ret);
}

static void mouse_move_timer_handler(void * p_context)
{
    UNUSED_PARAMETER(p_context);
    bool used = false;

    if (bsp_button_is_pressed(BTN_MOUSE_X_POS))
    {
        hid_generic_mouse_action(HID_GENERIC_MOUSE_X, CONFIG_MOUSE_MOVE_SPEED);
        used = true;
    }
    if (bsp_button_is_pressed(BTN_MOUSE_Y_POS))
    {
        hid_generic_mouse_action(HID_GENERIC_MOUSE_Y, CONFIG_MOUSE_MOVE_SPEED);
        used = true;
    }

    if(!used)
    {
        UNUSED_RETURN_VALUE(app_timer_stop(m_mouse_move_timer));
    }
}

static void bsp_event_callback(bsp_event_t ev)
{
    switch ((unsigned int)ev)
    {
        case CONCAT_2(BSP_EVENT_KEY_, BTN_MOUSE_X_POS):
            hid_generic_mouse_action(HID_GENERIC_MOUSE_X, CONFIG_MOUSE_MOVE_SPEED);
            UNUSED_RETURN_VALUE(app_timer_start(m_mouse_move_timer, APP_TIMER_TICKS(CONFIG_MOUSE_MOVE_TIME_MS), NULL));
            break;

        case CONCAT_2(BSP_EVENT_KEY_, BTN_MOUSE_Y_POS):
            hid_generic_mouse_action(HID_GENERIC_MOUSE_Y, CONFIG_MOUSE_MOVE_SPEED);
            UNUSED_RETURN_VALUE(app_timer_start(m_mouse_move_timer, APP_TIMER_TICKS(CONFIG_MOUSE_MOVE_TIME_MS), NULL));
            break;

        case CONCAT_2(BSP_EVENT_KEY_, BTN_MOUSE_RIGHT):
            hid_generic_mouse_action(HID_GENERIC_MOUSE_BTN_RIGHT, 1);
            break;
        case CONCAT_2(BSP_USER_EVENT_RELEASE_, BTN_MOUSE_RIGHT):
            hid_generic_mouse_action(HID_GENERIC_MOUSE_BTN_RIGHT, -1);
            break;

        case CONCAT_2(BSP_EVENT_KEY_, BTN_MOUSE_LEFT):
            hid_generic_mouse_action(HID_GENERIC_MOUSE_BTN_LEFT, 1);
            break;
        case CONCAT_2(BSP_USER_EVENT_RELEASE_, BTN_MOUSE_LEFT):
            hid_generic_mouse_action(HID_GENERIC_MOUSE_BTN_LEFT, -1);
            break;

        default:
            return; // no implementation needed
    }
}


/**
 * @brief Auxiliary internal macro
 *
 * Macro used only in @ref init_bsp to simplify the configuration
 */
#define INIT_BSP_ASSIGN_RELEASE_ACTION(btn)                      \
    APP_ERROR_CHECK(                                             \
        bsp_event_to_button_action_assign(                       \
            btn,                                                 \
            BSP_BUTTON_ACTION_RELEASE,                           \
            (bsp_event_t)CONCAT_2(BSP_USER_EVENT_RELEASE_, btn)) \
    )

static void init_bsp(void)
{
    ret_code_t ret;
    ret = bsp_init(BSP_INIT_BUTTONS, bsp_event_callback);
    APP_ERROR_CHECK(ret);

    INIT_BSP_ASSIGN_RELEASE_ACTION(BTN_MOUSE_LEFT );
    INIT_BSP_ASSIGN_RELEASE_ACTION(BTN_MOUSE_RIGHT);

    /* Configure LEDs */
    bsp_board_leds_init();
}

int main(void)
{
    ret_code_t ret;
    static const app_usbd_config_t usbd_config = {
        .ev_state_proc = usbd_user_ev_handler
    };

    init_power_clock();

    ret = NRF_LOG_INIT(NULL);
    APP_ERROR_CHECK(ret);
    NRF_LOG_DEFAULT_BACKENDS_INIT();
    NRF_LOG_INFO("Hello USB!");

    ret = app_timer_create(&m_mouse_move_timer, APP_TIMER_MODE_REPEATED, mouse_move_timer_handler);
    APP_ERROR_CHECK(ret);

    init_bsp();

    ret = app_usbd_init(&usbd_config);
    APP_ERROR_CHECK(ret);

    app_usbd_class_inst_t const * class_inst_generic;
    class_inst_generic = app_usbd_hid_generic_class_inst_get(&m_app_hid_generic);
    ret = app_usbd_class_append(class_inst_generic);
    APP_ERROR_CHECK(ret);

    bool last_usb_conn_status = false;
    usb_start();

    while (true)
    {
        while (app_usbd_event_queue_process())
        {
            /* Nothing to do */
        }
        last_usb_conn_status = usb_connection_handle(last_usb_conn_status);
        hid_generic_mouse_process_state();

        UNUSED_RETURN_VALUE(NRF_LOG_PROCESS());
        /* Sleep CPU only if there was no interrupt since last loop processing */
        __WFE();
    }
}
