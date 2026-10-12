/* SPDX-License-Identifier: MIT */
/*
 * Bluepad32 Wiimote Adapter Example
 *
 * Turns an original ESP32 into a wireless Game Controller to Wiimote Adapter:
 *   - Connects modern Bluetooth gamepads (DualSense, DualShock 4, Xbox Wireless,
 *     Switch Pro, 8BitDo, etc.) as HID Host using Bluepad32.
 *   - Simultaneously connects to a real Nintendo Wii as HID Device (emulating a
 *     Nintendo RVL-CNT-01 with Classic Controller extension) using WiimoteAnything.
 *   - Runs both Host and Device roles on a single ESP32 sharing BTstack!
 */

#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include <btstack_port_esp32.h>
#include <btstack_run_loop.h>
#include <btstack_stdio_esp32.h>
#include <uni.h>

#include "esp_log.h"
#include "sdkconfig.h"
#include "wiimote.h"

static const char *TAG = "bp32_wiimote_adapter";

static uni_hid_device_t *s_active_controller = NULL;

/* Callback when Wii console changes rumble motor state */
static void on_wiimote_rumble(bool on, void *user)
{
    (void)user;
    if (s_active_controller != NULL && s_active_controller->report_parser.play_dual_rumble != NULL) {
        uint8_t mag = on ? 0xc0 : 0x00;
        s_active_controller->report_parser.play_dual_rumble(
            s_active_controller, 0 /* delayed start ms */, on ? 250 : 0 /* duration ms */, mag, mag);
    }
}

/* Callback when Wii console changes player LEDs */
static void on_wiimote_leds(uint8_t mask, void *user)
{
    (void)user;
    ESP_LOGI(TAG, "Wii assigned Player LED mask: 0x%02x", mask);
    if (s_active_controller != NULL && s_active_controller->report_parser.set_player_leds != NULL) {
        s_active_controller->report_parser.set_player_leds(s_active_controller, mask & 0x0f);
    }
}

static void on_wiimote_connected(void *user)
{
    (void)user;
    if (s_active_controller == NULL) {
        ESP_LOGI(TAG, "Connected to Wii console! Starting Bluepad32 gamepad scanning...");
        uni_bt_start_scanning_and_autoconnect_safe();
    } else {
        ESP_LOGI(TAG, "Connected to Wii console! Gamepad already active.");
    }
}

static void on_wiimote_disconnected(void *user)
{
    (void)user;
    ESP_LOGI(TAG, "Disconnected from Wii console");
}

static wiimote_callbacks_t s_wm_cb = {
    .connected = on_wiimote_connected,
    .disconnected = on_wiimote_disconnected,
    .rumble = on_wiimote_rumble,
    .leds = on_wiimote_leds,
};

static wiimote_config_t s_wm_cfg = {
    .extension = WIIMOTE_EXT_CLASSIC,
    .battery = 0xC0,
    .callbacks = &s_wm_cb,
    .external_stack = true, /* Bluepad32 manages the BTstack run loop */
};

/* Bluepad32 platform callbacks */
static void adapter_init(int argc, const char **argv)
{
    (void)argc;
    (void)argv;
    ESP_LOGI(TAG, "Bluepad32 custom platform init");
}

static float apply_deadzone(float v, float dz)
{
    if (v > -dz && v < dz) {
        return 0.0f;
    }
    return v;
}

static void adapter_on_init_complete(void)
{
    ESP_LOGI(TAG, "Bluepad32 init complete; starting 20s Wiimote sync window (press red SYNC on Wii console)");

    /* Disable Bluepad32 BLE configuration service and DualSense virtual touchpad device */
    uni_bt_service_set_enabled(false);
    uni_virtual_device_set_enabled(false);

    /* Initialize WiimoteAnything into the running BTstack */
    ESP_ERROR_CHECK(wiimote_init(&s_wm_cfg));

    /* Start 20s Wiimote sync window first (without BR/EDR periodic inquiry hogging the radio) */
    ESP_ERROR_CHECK(wiimote_sync());
}

static uni_error_t adapter_on_device_discovered(bd_addr_t addr, const char *name, uint16_t cod, uint8_t rssi)
{
    (void)addr;
    (void)name;
    (void)cod;
    (void)rssi;
    if (s_active_controller != NULL) {
        return UNI_ERROR_IGNORE_DEVICE;
    }
    return UNI_ERROR_SUCCESS;
}

static void adapter_on_device_connected(uni_hid_device_t *d)
{
    ESP_LOGI(TAG, "Gamepad connected: %p", (void *)d);
    if (s_active_controller == NULL) {
        s_active_controller = d;
    }
    uni_bt_stop_scanning_safe();
}

static void adapter_on_device_disconnected(uni_hid_device_t *d)
{
    ESP_LOGI(TAG, "Gamepad disconnected: %p", (void *)d);
    if (s_active_controller == d) {
        s_active_controller = NULL;
        uni_bt_start_scanning_and_autoconnect_safe();
    }
}

static uni_error_t adapter_on_device_ready(uni_hid_device_t *d)
{
    ESP_LOGI(TAG, "Gamepad ready: %p", (void *)d);
    uni_bt_stop_scanning_safe();
    return UNI_ERROR_SUCCESS;
}

/* Map gamepad buttons and axes to Wiimote + Classic Controller */
static void adapter_on_controller_data(uni_hid_device_t *d, uni_controller_t *ctl)
{
    if (d != s_active_controller || ctl->klass != UNI_CONTROLLER_CLASS_GAMEPAD) {
        return;
    }

    uni_gamepad_t *gp = &ctl->gamepad;

    bool dpad_up    = (gp->dpad & DPAD_UP) != 0;
    bool dpad_down  = (gp->dpad & DPAD_DOWN) != 0;
    bool dpad_left  = (gp->dpad & DPAD_LEFT) != 0;
    bool dpad_right = (gp->dpad & DPAD_RIGHT) != 0;

    /* D-pad (both Core Wiimote and Classic Controller) */
    wiimote_set_button(WIIMOTE_BUTTON_UP, dpad_up);
    wiimote_set_button(WIIMOTE_BUTTON_DOWN, dpad_down);
    wiimote_set_button(WIIMOTE_BUTTON_LEFT, dpad_left);
    wiimote_set_button(WIIMOTE_BUTTON_RIGHT, dpad_right);
    wiimote_set_button(WIIMOTE_BUTTON_CLASSIC_UP, dpad_up);
    wiimote_set_button(WIIMOTE_BUTTON_CLASSIC_DOWN, dpad_down);
    wiimote_set_button(WIIMOTE_BUTTON_CLASSIC_LEFT, dpad_left);
    wiimote_set_button(WIIMOTE_BUTTON_CLASSIC_RIGHT, dpad_right);

    /* Main face buttons (mapped to Classic Controller A/B/X/Y and Core Wiimote A/B/1/2) */
    bool btn_a = (gp->buttons & BUTTON_A) != 0;
    bool btn_b = (gp->buttons & BUTTON_B) != 0;
    bool btn_x = (gp->buttons & BUTTON_X) != 0;
    bool btn_y = (gp->buttons & BUTTON_Y) != 0;

    wiimote_set_button(WIIMOTE_BUTTON_CLASSIC_A, btn_a);
    wiimote_set_button(WIIMOTE_BUTTON_CLASSIC_B, btn_b);
    wiimote_set_button(WIIMOTE_BUTTON_CLASSIC_X, btn_x);
    wiimote_set_button(WIIMOTE_BUTTON_CLASSIC_Y, btn_y);

    wiimote_set_button(WIIMOTE_BUTTON_A, btn_a);
    wiimote_set_button(WIIMOTE_BUTTON_B, btn_b || ((gp->buttons & BUTTON_SHOULDER_R) != 0));
    wiimote_set_button(WIIMOTE_BUTTON_ONE, btn_x);
    wiimote_set_button(WIIMOTE_BUTTON_TWO, btn_y);

    /* System buttons (both Core Wiimote and Classic Controller) */
    bool btn_plus  = (gp->misc_buttons & MISC_BUTTON_START) != 0;
    bool btn_minus = (gp->misc_buttons & MISC_BUTTON_SELECT) != 0;
    bool btn_home  = (gp->misc_buttons & MISC_BUTTON_SYSTEM) != 0;

    wiimote_set_button(WIIMOTE_BUTTON_PLUS, btn_plus);
    wiimote_set_button(WIIMOTE_BUTTON_MINUS, btn_minus);
    wiimote_set_button(WIIMOTE_BUTTON_HOME, btn_home);
    wiimote_set_button(WIIMOTE_BUTTON_CLASSIC_PLUS, btn_plus);
    wiimote_set_button(WIIMOTE_BUTTON_CLASSIC_MINUS, btn_minus);
    wiimote_set_button(WIIMOTE_BUTTON_CLASSIC_HOME, btn_home);

    /* Classic Shoulder and Trigger Buttons */
    bool sh_l = (gp->buttons & BUTTON_SHOULDER_L) != 0;
    bool sh_r = (gp->buttons & BUTTON_SHOULDER_R) != 0;
    bool tr_l = (gp->buttons & BUTTON_TRIGGER_L) != 0 || (gp->brake > 256);
    bool tr_r = (gp->buttons & BUTTON_TRIGGER_R) != 0 || (gp->throttle > 256);

    wiimote_set_button(WIIMOTE_BUTTON_CLASSIC_L, tr_l);
    wiimote_set_button(WIIMOTE_BUTTON_CLASSIC_R, tr_r);
    wiimote_set_button(WIIMOTE_BUTTON_CLASSIC_ZL, sh_l);
    wiimote_set_button(WIIMOTE_BUTTON_CLASSIC_ZR, sh_r);

    /* Analog Sticks (-1.0 to +1.0; Bluepad32 axis values are -512 to +511) */
    float lx = apply_deadzone((float)gp->axis_x / 512.0f, 0.06f);
    float ly = apply_deadzone(-(float)gp->axis_y / 512.0f, 0.06f);
    float rx = apply_deadzone((float)gp->axis_rx / 512.0f, 0.06f);
    float ry = apply_deadzone(-(float)gp->axis_ry / 512.0f, 0.06f);

    wiimote_set_axis(WIIMOTE_AXIS_CLASSIC_LX, lx);
    wiimote_set_axis(WIIMOTE_AXIS_CLASSIC_LY, ly);
    wiimote_set_axis(WIIMOTE_AXIS_CLASSIC_RX, rx);
    wiimote_set_axis(WIIMOTE_AXIS_CLASSIC_RY, ry);

    /* Analog Triggers (0.0 to 1.0) */
    float lt = (float)gp->brake / 1023.0f;
    float rt = (float)gp->throttle / 1023.0f;
    wiimote_set_axis(WIIMOTE_AXIS_CLASSIC_LT, lt);
    wiimote_set_axis(WIIMOTE_AXIS_CLASSIC_RT, rt);
}

static const uni_property_t *adapter_get_property(uni_property_idx_t idx)
{
    (void)idx;
    return NULL;
}

static void adapter_on_oob_event(uni_platform_oob_event_t event, void *data)
{
    (void)event;
    (void)data;
}

static struct uni_platform *get_adapter_platform(void)
{
    static struct uni_platform plat = {
        .name = "WiimoteAdapter",
        .init = adapter_init,
        .on_init_complete = adapter_on_init_complete,
        .on_device_discovered = adapter_on_device_discovered,
        .on_device_connected = adapter_on_device_connected,
        .on_device_disconnected = adapter_on_device_disconnected,
        .on_device_ready = adapter_on_device_ready,
        .on_oob_event = adapter_on_oob_event,
        .on_controller_data = adapter_on_controller_data,
        .get_property = adapter_get_property,
    };
    return &plat;
}

int app_main(void)
{
#ifdef CONFIG_ESP_CONSOLE_UART
#ifndef CONFIG_BLUEPAD32_USB_CONSOLE_ENABLE
    btstack_stdio_init();
#endif
#endif

    /* 1. Configure BTstack for ESP32 VHCI Controller */
    btstack_init();

    /* 2. Register custom platform callbacks before uni_init() */
    uni_platform_set_custom(get_adapter_platform());

    /* 3. Initialize Bluepad32 */
    uni_init(0, NULL);

    /* 4. Run BTstack event loop (does not return) */
    btstack_run_loop_execute();

    return 0;
}
