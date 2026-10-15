/* SPDX-License-Identifier: MIT */
/*
 * Bluepad32 -> Wii adapter.
 *
 * One original ESP32 is both:
 *   - an HID host for one modern gamepad (Bluepad32: DualSense, DualShock 4,
 *     Switch Pro, 8BitDo, Xbox, ...), and
 *   - a Wii Remote with a Classic Controller plugged in (WiimoteAnything).
 *
 * Both share one BTstack instance. WiimoteAnything owns L2CAP PSM 0x11/0x13
 * and hands every non-console channel to Bluepad32 (see wiimote_btstack.h).
 *
 * Everything below runs on the BTstack thread: Bluepad32 platform callbacks,
 * WiimoteAnything callbacks, and the 20 ms housekeeping timer. So the
 * Bluepad32 "_unsafe" functions are the right ones to call.
 *
 * Pairing, once:
 *   1. Flash and boot. Put the gamepad in pairing mode (DualSense: Create+PS;
 *      8BitDo SN30 Pro: Start+Y for Switch mode, then hold Pair).
 *   2. When the gamepad is ready, the sync window opens by itself for 20 s:
 *      press the red SYNC button on the Wii.
 *      (Later: press BOOT, or hold Select+Start for 3 s, to sync again.)
 * Afterwards both are remembered. Press PS/Home on the gamepad: it reconnects
 * to the ESP32, and the ESP32 reconnects to the Wii (any button retries).
 * Hold BOOT for 5 s to forget the Wii and all gamepads.
 */

#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include <btstack_port_esp32.h>
#include <btstack_run_loop.h>
#include <btstack_stdio_esp32.h>
#include <uni.h>

#include "driver/gpio.h"
#include "esp_log.h"
#include "esp_timer.h"
#include "sdkconfig.h"

#include "wiimote.h"
#include "wiimote_btstack.h"

#ifndef CONFIG_BLUEPAD32_PLATFORM_CUSTOM
#error "Set CONFIG_BLUEPAD32_PLATFORM_CUSTOM=y (see sdkconfig.defaults)"
#endif

/* ------------------------------------------------------------ options -- */

/* BOOT button on most dev boards. Short press: sync. Hold 5 s: forget all. -1 disables. */
#define ADAPTER_SYNC_GPIO 0
/* BLE scanning costs radio time the console link needs. Classic gamepads
 * (DualSense, DS4, Switch Pro, 8BitDo in Switch/D-input mode) do not need it. */
#define ADAPTER_ENABLE_BLE 0
/* Also press the matching Wii Remote buttons, so screens that only listen to
 * the remote ("Press A", Wii Menu, HOME menu) work from the gamepad. */
#define ADAPTER_MIRROR_CORE_BUTTONS 1
/* 1: the Wii's a/b/x/y follow the gamepad's labels instead of positions. */
#define ADAPTER_LABEL_LAYOUT 0

#define STICK_DEADZONE 0.08f
#define TRIGGER_DIGITAL_THRESHOLD 0.85f
#define SYNC_COMBO_HOLD_MS 3000
#define FORGET_HOLD_MS 5000
#define RECONNECT_RETRY_MS 6000
/* After the Wii dropped us (e.g. when a game starts it resets every remote),
 * page it again on our own for this long, like a real remote does. */
#define AUTO_RECONNECT_WINDOW_MS 60000
#define AUTO_RECONNECT_FIRST_MS 300
/* Re-page this soon after dropping a link on which the ESP32 is the slave. */
#define REPAGE_DELAY_MS 1000
#define RUMBLE_REFRESH_MS 4000
#define TICK_MS 20

static const char *TAG = "wii_adapter";

/* -------------------------------------------------------------- state -- */

static uni_hid_device_t *s_pad;
static bool s_wii_connected;
static bool s_syncing;
static bool s_scanning;
static uint8_t s_wii_leds;
static bool s_rumble_on;
static int64_t s_rumble_sent_ms;
static int64_t s_last_connect_ms = -RECONNECT_RETRY_MS;
static uint8_t s_battery_sent;
static int64_t s_auto_reconnect_until_ms = -1;
static int64_t s_repage_at_ms = -1;
static bool s_we_dropped_wii;

static uint16_t s_buttons;
static uint8_t s_misc;
static int64_t s_combo_since_ms = -1;

static btstack_timer_source_t s_tick;
static int64_t s_boot_down_ms = -1;
static bool s_boot_long_fired;

static int64_t now_ms(void)
{
    return esp_timer_get_time() / 1000;
}

/* ------------------------------------------------------ orchestration -- */

/* Inquiry for new gamepads only when there is no gamepad and no sync window.
 * Bonded gamepads reconnect by paging us, which needs no inquiry at all. */
static void update_scanning(void)
{
    bool want = (s_pad == NULL) && !s_syncing;
    if (want == s_scanning) {
        return;
    }
    s_scanning = want;
    if (want) {
        ESP_LOGI(TAG, "Looking for a gamepad (put it in pairing mode, or press its PS/Home button)");
        uni_bt_start_scanning_and_autoconnect_unsafe();
    } else {
        uni_bt_stop_scanning_unsafe();
    }
}

static void try_connect_wii(const char *why)
{
    int64_t now = now_ms();
    if (s_wii_connected || s_syncing || !wiimote_has_paired_host()) {
        return;
    }
    if (s_pad == NULL) {
        /* Paging the Wii without a gamepad would only leave us as its slave. */
        return;
    }
    if (now - s_last_connect_ms < RECONNECT_RETRY_MS) {
        return;
    }
    s_last_connect_ms = now;
    ESP_LOGI(TAG, "Connecting to the Wii (%s)", why);
    wiimote_connect();
}

static void start_sync(void)
{
    ESP_LOGI(TAG, "Sync: press the red SYNC button on the Wii now (%d s)", WIIMOTE_SYNC_WINDOW_MS / 1000);
    s_auto_reconnect_until_ms = -1;
    s_repage_at_ms = -1;
    if (s_wii_connected) {
        s_we_dropped_wii = true;
        wiimote_disconnect();
    }
    wiimote_sync();
}

static void forget_everything(void)
{
    ESP_LOGW(TAG, "Forgetting the Wii and all gamepads");
    wiimote_forget_host();
    if (s_pad != NULL) {
        uni_hid_device_disconnect(s_pad);
        uni_hid_device_delete(s_pad);
        s_pad = NULL;
    }
    uni_bt_del_keys_unsafe();
    update_scanning();
}

/* ----------------------------------------------------- gamepad outputs -- */

static void pad_set_leds(uint8_t mask)
{
    if (s_pad != NULL && s_pad->report_parser.set_player_leds != NULL) {
        s_pad->report_parser.set_player_leds(s_pad, mask & 0x0f);
    }
}

static void pad_rumble(bool on)
{
    if (s_pad == NULL || s_pad->report_parser.play_dual_rumble == NULL) {
        return;
    }
    if (on) {
        /* The Wii only says on/off. Ask for longer than the refresh period. */
        s_pad->report_parser.play_dual_rumble(s_pad, 0, RUMBLE_REFRESH_MS + 1000, 0x80, 0xff);
    } else {
        s_pad->report_parser.play_dual_rumble(s_pad, 0, 0, 0, 0);
    }
    s_rumble_sent_ms = now_ms();
}

/* -------------------------------------------------- Wiimote callbacks -- */

static void on_wii_connected(void *user)
{
    (void)user;
    s_wii_connected = true;
    if (s_pad == NULL) {
        /* The ESP32 radio cannot reach a gamepad while it is the slave on a
         * Wii link, so never sit on the Wii link without a gamepad. The
         * pairing is stored; the Wii is paged again when a gamepad is ready. */
        ESP_LOGI(TAG, "Wii paired. Releasing it until a gamepad is connected.");
        s_we_dropped_wii = true;
        wiimote_disconnect();
        return;
    }
    if (!wiimote_link_is_master()) {
        /* The Wii paged us (sync). As its slave the ESP32 radio starves the
         * gamepad link, so hang up and page the Wii ourselves: the pager
         * stays master of both links. */
        ESP_LOGI(TAG, "Wii connected with the ESP32 as slave; reconnecting as master.");
        s_we_dropped_wii = true;
        s_repage_at_ms = now_ms() + REPAGE_DELAY_MS;
        wiimote_disconnect();
        return;
    }
    s_auto_reconnect_until_ms = -1;
    s_repage_at_ms = -1;
    ESP_LOGI(TAG, "Wii connected");
}

static void on_wii_disconnected(void *user)
{
    (void)user;
    s_wii_connected = false;
    s_wii_leds = 0;
    if (s_rumble_on) {
        s_rumble_on = false;
        pad_rumble(false);
    }
    pad_set_leds(0);

    uint8_t reason = wiimote_last_disconnect_reason();
    bool ours = s_we_dropped_wii;
    s_we_dropped_wii = false;
    ESP_LOGI(TAG, "Wii disconnected (reason 0x%02x%s)", reason, ours ? ", by us" : "");
    if (ours || s_pad == NULL) {
        return;
    }
    if (reason == 0x15) {
        /* Remote device power off: the Wii was switched off. */
        ESP_LOGI(TAG, "The Wii switched off. Press a gamepad button to wake it.");
        return;
    }
    /* The Wii hung up while still on (a game starting, the Home menu's
     * "reconnect" or radio trouble): come back on our own. */
    int64_t now = now_ms();
    s_auto_reconnect_until_ms = now + AUTO_RECONNECT_WINDOW_MS;
    s_last_connect_ms = now + AUTO_RECONNECT_FIRST_MS - RECONNECT_RETRY_MS;
}

static void on_wii_leds(uint8_t mask, void *user)
{
    (void)user;
    s_wii_leds = mask;
    ESP_LOGI(TAG, "Wii player LEDs: 0x%x", mask);
    pad_set_leds(mask);
}

static void on_wii_rumble(bool on, void *user)
{
    (void)user;
    if (on == s_rumble_on) {
        return;
    }
    s_rumble_on = on;
    pad_rumble(on);
}

static void on_wii_sync(bool active, void *user)
{
    (void)user;
    s_syncing = active;
    /* Also fires right before `connected` when the Wii paired, so ask the
     * library rather than our own flag. */
    if (!active && !wiimote_is_connected()) {
        ESP_LOGI(TAG, "Sync window closed without a Wii. Press BOOT (or hold Select+Start) to retry.");
    }
    update_scanning();
}

static const wiimote_callbacks_t s_wm_callbacks = {
    .connected = on_wii_connected,
    .disconnected = on_wii_disconnected,
    .leds = on_wii_leds,
    .rumble = on_wii_rumble,
    .sync_changed = on_wii_sync,
};

/* ------------------------------------------------------------ mapping -- */

static void stick(int32_t raw_x, int32_t raw_y, float *out_x, float *out_y)
{
    /* Bluepad32: -512..511, +y is down. Wii: -1..1, +y is up. */
    float x = (float)raw_x / 512.0f;
    float y = -(float)raw_y / 512.0f;
    float mag = sqrtf(x * x + y * y);
    if (mag < STICK_DEADZONE) {
        *out_x = 0.0f;
        *out_y = 0.0f;
        return;
    }
    /* Radial deadzone, rescaled so the edge still reaches full deflection. */
    float scale = (mag - STICK_DEADZONE) / (1.0f - STICK_DEADZONE) / mag;
    if (mag * scale > 1.0f) {
        scale = 1.0f / mag;
    }
    *out_x = x * scale;
    *out_y = y * scale;
}

static void map_gamepad(const uni_gamepad_t *gp)
{
    const bool up = (gp->dpad & DPAD_UP) != 0;
    const bool down = (gp->dpad & DPAD_DOWN) != 0;
    const bool left = (gp->dpad & DPAD_LEFT) != 0;
    const bool right = (gp->dpad & DPAD_RIGHT) != 0;

    /* Bluepad32 names face buttons by position, Xbox style:
     * A = south, B = east, X = west, Y = north.
     * Classic Controller (Nintendo layout): a = east, b = south, x = north, y = west. */
#if ADAPTER_LABEL_LAYOUT
    const bool cc_a = (gp->buttons & BUTTON_A) != 0;
    const bool cc_b = (gp->buttons & BUTTON_B) != 0;
    const bool cc_x = (gp->buttons & BUTTON_X) != 0;
    const bool cc_y = (gp->buttons & BUTTON_Y) != 0;
#else
    const bool cc_a = (gp->buttons & BUTTON_B) != 0;
    const bool cc_b = (gp->buttons & BUTTON_A) != 0;
    const bool cc_x = (gp->buttons & BUTTON_Y) != 0;
    const bool cc_y = (gp->buttons & BUTTON_X) != 0;
#endif
    const bool plus = (gp->misc_buttons & MISC_BUTTON_START) != 0;
    const bool minus = (gp->misc_buttons & MISC_BUTTON_SELECT) != 0;
    const bool home = (gp->misc_buttons & MISC_BUTTON_SYSTEM) != 0;

    /* Analog triggers (0..1023). Digital-only triggers (SN30 Pro) set the button. */
    float lt = (float)gp->brake / 1023.0f;
    float rt = (float)gp->throttle / 1023.0f;
    if (gp->buttons & BUTTON_TRIGGER_L) {
        lt = 1.0f;
    }
    if (gp->buttons & BUTTON_TRIGGER_R) {
        rt = 1.0f;
    }
    if (lt > 1.0f) lt = 1.0f;
    if (rt > 1.0f) rt = 1.0f;

    float lx, ly, rx, ry;
    stick(gp->axis_x, gp->axis_y, &lx, &ly);
    stick(gp->axis_rx, gp->axis_ry, &rx, &ry);

    /* Classic Controller: L/R are the analog triggers (with a click at the end
     * of travel), ZL/ZR the small inner buttons -> gamepad L1/R1. */
    wiimote_set_button(WIIMOTE_BUTTON_CLASSIC_UP, up);
    wiimote_set_button(WIIMOTE_BUTTON_CLASSIC_DOWN, down);
    wiimote_set_button(WIIMOTE_BUTTON_CLASSIC_LEFT, left);
    wiimote_set_button(WIIMOTE_BUTTON_CLASSIC_RIGHT, right);
    wiimote_set_button(WIIMOTE_BUTTON_CLASSIC_A, cc_a);
    wiimote_set_button(WIIMOTE_BUTTON_CLASSIC_B, cc_b);
    wiimote_set_button(WIIMOTE_BUTTON_CLASSIC_X, cc_x);
    wiimote_set_button(WIIMOTE_BUTTON_CLASSIC_Y, cc_y);
    wiimote_set_button(WIIMOTE_BUTTON_CLASSIC_PLUS, plus);
    wiimote_set_button(WIIMOTE_BUTTON_CLASSIC_MINUS, minus);
    wiimote_set_button(WIIMOTE_BUTTON_CLASSIC_HOME, home);
    wiimote_set_button(WIIMOTE_BUTTON_CLASSIC_ZL, (gp->buttons & BUTTON_SHOULDER_L) != 0);
    wiimote_set_button(WIIMOTE_BUTTON_CLASSIC_ZR, (gp->buttons & BUTTON_SHOULDER_R) != 0);
    wiimote_set_button(WIIMOTE_BUTTON_CLASSIC_L, lt >= TRIGGER_DIGITAL_THRESHOLD);
    wiimote_set_button(WIIMOTE_BUTTON_CLASSIC_R, rt >= TRIGGER_DIGITAL_THRESHOLD);
    wiimote_set_axis(WIIMOTE_AXIS_CLASSIC_LT, lt);
    wiimote_set_axis(WIIMOTE_AXIS_CLASSIC_RT, rt);
    wiimote_set_axis(WIIMOTE_AXIS_CLASSIC_LX, lx);
    wiimote_set_axis(WIIMOTE_AXIS_CLASSIC_LY, ly);
    wiimote_set_axis(WIIMOTE_AXIS_CLASSIC_RX, rx);
    wiimote_set_axis(WIIMOTE_AXIS_CLASSIC_RY, ry);

#if ADAPTER_MIRROR_CORE_BUTTONS
    wiimote_set_button(WIIMOTE_BUTTON_UP, up);
    wiimote_set_button(WIIMOTE_BUTTON_DOWN, down);
    wiimote_set_button(WIIMOTE_BUTTON_LEFT, left);
    wiimote_set_button(WIIMOTE_BUTTON_RIGHT, right);
    wiimote_set_button(WIIMOTE_BUTTON_A, cc_a);
    wiimote_set_button(WIIMOTE_BUTTON_B, cc_b);
    wiimote_set_button(WIIMOTE_BUTTON_ONE, cc_y);
    wiimote_set_button(WIIMOTE_BUTTON_TWO, cc_x);
    wiimote_set_button(WIIMOTE_BUTTON_PLUS, plus);
    wiimote_set_button(WIIMOTE_BUTTON_MINUS, minus);
#endif
    /* HOME on the remote always works, even in screens that ignore the CC. */
    wiimote_set_button(WIIMOTE_BUTTON_HOME, home);
}

static void release_all_inputs(void)
{
    uni_gamepad_t idle;
    memset(&idle, 0, sizeof(idle));
    map_gamepad(&idle);
}

/* ----------------------------------------------- Bluepad32 callbacks -- */

static void adapter_init(int argc, const char **argv)
{
    (void)argc;
    (void)argv;
    /* Runs after Bluepad32 loaded its properties, before Bluetooth setup. */
    uni_bt_le_set_enabled(ADAPTER_ENABLE_BLE);
}

static void adapter_on_init_complete(void)
{
    /* One physical DualSense = one slot (no virtual touchpad mouse). */
    uni_virtual_device_set_enabled(false);

    if (wiimote_has_paired_host()) {
        ESP_LOGI(TAG, "Ready. Press PS/Home on your gamepad; the adapter then connects to the Wii.");
    } else {
        /* Pair the gamepad first: connecting to a gamepad is much harder for
         * the ESP32 radio once the Wii link exists. The sync window opens by
         * itself when the gamepad is ready. */
        ESP_LOGI(TAG, "First run: pair the gamepad now; the Wii sync window opens afterwards.");
    }
    update_scanning();
}

static uni_error_t adapter_on_device_discovered(bd_addr_t addr, const char *name, uint16_t cod, uint8_t rssi)
{
    (void)addr;
    (void)rssi;
    if (s_pad != NULL) {
        return UNI_ERROR_IGNORE_DEVICE;
    }
    /* Keyboards and mice are peripherals too; this adapter wants gamepads. */
    if ((cod & UNI_BT_COD_MINOR_MASK) & UNI_BT_COD_MINOR_KEYBOARD_AND_MICE) {
        ESP_LOGI(TAG, "Ignoring keyboard/mouse %s", name ? name : "");
        return UNI_ERROR_IGNORE_DEVICE;
    }
    return UNI_ERROR_SUCCESS;
}

static void adapter_on_device_connected(uni_hid_device_t *d)
{
    ESP_LOGI(TAG, "Gamepad connecting: %s", bd_addr_to_str(d->conn.btaddr));
}

static void adapter_on_device_disconnected(uni_hid_device_t *d)
{
    if (d != s_pad) {
        return;
    }
    ESP_LOGI(TAG, "Gamepad disconnected");
    s_pad = NULL;
    s_combo_since_ms = -1;
    s_buttons = 0;
    s_misc = 0;
    release_all_inputs();
    s_auto_reconnect_until_ms = -1;
    s_repage_at_ms = -1;
    if (s_wii_connected && !wiimote_link_is_master()) {
        /* Only as the Wii's slave would the link block the gamepad from
         * coming back. As master, keep it: dropping a remote mid-game makes
         * some games hang on the "reconnect" screen. Inputs are released,
         * so the Wii simply sees an idle remote until the gamepad returns. */
        ESP_LOGI(TAG, "Releasing the Wii until the gamepad is back");
        s_we_dropped_wii = true;
        wiimote_disconnect();
    } else if (s_wii_connected) {
        ESP_LOGI(TAG, "Keeping the Wii link; waiting for the gamepad to come back");
    }
    update_scanning();
}

static uni_error_t adapter_on_device_ready(uni_hid_device_t *d)
{
    if (s_pad != NULL && s_pad != d) {
        ESP_LOGI(TAG, "Already have a gamepad; rejecting %s", bd_addr_to_str(d->conn.btaddr));
        return UNI_ERROR_IGNORE_DEVICE;
    }
    s_pad = d;
    s_battery_sent = 0;
    ESP_LOGI(TAG, "Gamepad ready: %s (%s)", d->name, bd_addr_to_str(d->conn.btaddr));
    update_scanning();
    pad_set_leds(s_wii_leds);
    if (s_rumble_on) {
        pad_rumble(true);
    }
    s_last_connect_ms = -RECONNECT_RETRY_MS; /* Allow an immediate attempt. */
    if (!wiimote_has_paired_host() && !s_syncing) {
        ESP_LOGI(TAG, "No Wii paired yet: opening the sync window.");
        start_sync();
    } else {
        try_connect_wii("gamepad ready");
    }
    return UNI_ERROR_SUCCESS;
}

static void adapter_on_controller_data(uni_hid_device_t *d, uni_controller_t *ctl)
{
    if (d != s_pad || ctl->klass != UNI_CONTROLLER_CLASS_GAMEPAD) {
        return;
    }
    const uni_gamepad_t *gp = &ctl->gamepad;

    s_buttons = gp->buttons;
    s_misc = gp->misc_buttons;

    if (ctl->battery != UNI_CONTROLLER_BATTERY_NOT_AVAILABLE && ctl->battery != s_battery_sent) {
        /* Bluepad32 1..251 (DualSense: 10 % steps) -> Wii 0x20..0xC8. The Wii
         * warns below 0x20, so only an empty pad (1) reports low battery. */
        s_battery_sent = ctl->battery;
        wiimote_set_battery(ctl->battery <= 1 ? 0x10 : (uint8_t)(0x20 + (ctl->battery * 0xa8u) / 255u));
    }

    map_gamepad(gp);
    /* Send this report now instead of at the next 5 ms flush. */
    wiimote_send_now();

    /* A bonded remote reconnects when a button is pressed. */
    if (!s_wii_connected && (gp->buttons || gp->misc_buttons || gp->dpad)) {
        try_connect_wii("button press");
    }
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

static struct uni_platform s_platform = {
    .name = "Wii adapter",
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

/* ------------------------------------------------------ housekeeping -- */

static void tick(btstack_timer_source_t *ts)
{
    int64_t now = now_ms();

    /* Select + Start held: sync again (e.g. to pair with another Wii). */
    const uint8_t combo = MISC_BUTTON_SELECT | MISC_BUTTON_START;
    if (s_pad != NULL && (s_misc & combo) == combo) {
        if (s_combo_since_ms < 0) {
            s_combo_since_ms = now;
        } else if (now - s_combo_since_ms >= SYNC_COMBO_HOLD_MS && !s_syncing) {
            s_combo_since_ms = now + 60000; /* once per hold */
            start_sync();
        }
    } else {
        s_combo_since_ms = -1;
    }

#if ADAPTER_SYNC_GPIO >= 0
    bool down = gpio_get_level(ADAPTER_SYNC_GPIO) == 0;
    if (down) {
        if (s_boot_down_ms < 0) {
            s_boot_down_ms = now;
            s_boot_long_fired = false;
        } else if (!s_boot_long_fired && now - s_boot_down_ms >= FORGET_HOLD_MS) {
            s_boot_long_fired = true;
            forget_everything();
        }
    } else if (s_boot_down_ms >= 0) {
        if (!s_boot_long_fired && now - s_boot_down_ms >= 50 && !s_syncing) {
            start_sync();
        }
        s_boot_down_ms = -1;
    }
#endif

    if (!s_wii_connected && s_pad != NULL) {
        if (s_repage_at_ms >= 0 && now >= s_repage_at_ms) {
            s_repage_at_ms = -1;
            s_last_connect_ms = -RECONNECT_RETRY_MS;
            try_connect_wii("as master");
        } else if (s_auto_reconnect_until_ms >= 0) {
            if (now >= s_auto_reconnect_until_ms) {
                s_auto_reconnect_until_ms = -1;
                ESP_LOGI(TAG, "Wii did not come back. Press a gamepad button to retry.");
            } else {
                try_connect_wii("auto-reconnect");
            }
        }
    }

    if (s_rumble_on && now - s_rumble_sent_ms >= RUMBLE_REFRESH_MS) {
        pad_rumble(true);
    }

    btstack_run_loop_set_timer(ts, TICK_MS);
    btstack_run_loop_add_timer(ts);
}

/* ---------------------------------------------------------------- main -- */

int app_main(void)
{
#ifdef CONFIG_ESP_CONSOLE_UART
#ifndef CONFIG_BLUEPAD32_USB_CONSOLE_ENABLE
    btstack_stdio_init();
#endif
#endif

#if ADAPTER_SYNC_GPIO >= 0
    gpio_config_t io = {
        .pin_bit_mask = 1ULL << ADAPTER_SYNC_GPIO,
        .mode = GPIO_MODE_INPUT,
        .pull_up_en = GPIO_PULLUP_ENABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type = GPIO_INTR_DISABLE,
    };
    gpio_config(&io);
#endif

    /* 1. BTstack with the ESP32 VHCI controller (also sets up NVS + TLV). */
    btstack_init();

    /* 2. WiimoteAnything before Bluepad32, so it sees the console first.
     *    Non-console traffic on the HID PSMs goes to Bluepad32. */
    wiimote_btstack_set_fallback_handler(uni_bt_packet_handler);
    const wiimote_config_t wm_cfg = {
        .extension = WIIMOTE_EXT_CLASSIC,
        .battery = 0xC0,
        .callbacks = &s_wm_callbacks,
        .external_stack = true,
    };
    ESP_ERROR_CHECK(wiimote_init(&wm_cfg));

    /* 3. Bluepad32 (powers the controller on). */
    uni_platform_set_custom(&s_platform);
    uni_init(0, NULL);

    btstack_run_loop_set_timer_handler(&s_tick, tick);
    btstack_run_loop_set_timer(&s_tick, TICK_MS);
    btstack_run_loop_add_timer(&s_tick);

    /* 4. Does not return. */
    btstack_run_loop_execute();
    return 0;
}
