/* SPDX-License-Identifier: MIT */
/**
 * WiimoteAnything — Bluetooth Classic Wiimote peripheral for the original ESP32.
 *
 * The C API is the stable surface. Protocol tables live in the C sources.
 * Requires the original ESP32 (BR/EDR). ESP32-S3/C3/C6/H2 cannot host this.
 */
#ifndef WIIMOTE_H
#define WIIMOTE_H

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

#include "esp_err.h"
#include "wiimote_ids.h"

#ifdef __cplusplus
extern "C" {
#endif

/** One analog channel. GPIO is an ADC1-capable pin, or -1 if unused. */
typedef struct {
    int gpio;
    int center;   /**< Raw ADC count at rest. */
    int min;      /**< Raw ADC count at the low end of travel. */
    int max;      /**< Raw ADC count at the high end of travel. */
    int deadzone; /**< Counts around center that still report center. */
} wiimote_axis_cal_t;

/**
 * Board wiring. Every gpio field starts at -1 (unused).
 * Buttons share one active level: active_low means pressed reads as 0.
 * Rumble is a plain GPIO unless rumble_pwm is set, in which case LEDC drives it.
 * Player LEDs are optional; the LED callback still fires when no pins are set.
 */
typedef struct {
    int button_gpio[WIIMOTE_BUTTON_COUNT];
    bool buttons_active_low;
    wiimote_axis_cal_t axis[WIIMOTE_AXIS_COUNT];
    int rumble_gpio;
    bool rumble_pwm;
    int led_gpio[4];
} wiimote_pins_t;

/**
 * Motion sample source used when no IMU driver is bound.
 * Accel is milli-g (1000 = 1 g). Gyro is degrees per second.
 * The callback writes through the pointers.
 */
typedef void (*wiimote_motion_cb_t)(int16_t *accel_mg_x,
                                   int16_t *accel_mg_y,
                                   int16_t *accel_mg_z,
                                   int16_t *pitch_dps,
                                   int16_t *roll_dps,
                                   int16_t *yaw_dps,
                                   void *user);

/** How long wiimote_sync() keeps the ESP32 discoverable. */
#define WIIMOTE_SYNC_WINDOW_MS 20000

/**
 * Callbacks. With the BTstack backend they all run on the BTstack thread, so
 * they may call BTstack (and Bluepad32 "_unsafe") functions directly. They must
 * not block.
 */
typedef struct {
    void (*connected)(void *user);
    void (*disconnected)(void *user);
    /** mask bit 0 is player LED 1. Called when the Wii changes the LEDs. */
    void (*leds)(uint8_t mask, void *user);
    void (*rumble)(bool on, void *user);
    /** Invoked while building a status report. Write the level (0x00–0xFF). */
    void (*battery)(uint8_t *level, void *user);
    void (*extension_changed)(wiimote_extension_t extension, void *user);
    void *user;
    /** The sync (discoverable) window opened (true) or closed (false). */
    void (*sync_changed)(bool active, void *user);
} wiimote_callbacks_t;

typedef struct {
    wiimote_extension_t extension;
    uint8_t battery; /**< Starting battery byte. 0xC0 is a healthy pack. */
    const wiimote_pins_t *pins;
    const wiimote_callbacks_t *callbacks;
    /**
     * BTstack only. false: wiimote_init() calls btstack_init(), powers the
     * controller on and runs the BTstack loop in its own task.
     * true: the application owns BTstack (e.g. Bluepad32). Call wiimote_init()
     * after btstack_init() and before the stack is powered on (before
     * uni_init()). Calling it later also works, but then the host stack sees
     * the console's connection request first. See wiimote_btstack.h.
     */
    bool external_stack;
} wiimote_config_t;

/** Set every GPIO to -1, buttons active-low, ADC spans 0..4095 centered at 2048. */
void wiimote_pins_init(wiimote_pins_t *pins);

esp_err_t wiimote_init(const wiimote_config_t *config);

/** Deprecated: wiimote_init() registers the services. Kept for compatibility. */
esp_err_t wiimote_btstack_register_services(void);

/**
 * The red SYNC button on the back of a remote. For WIIMOTE_SYNC_WINDOW_MS, be
 * connectable and limited-discoverable so a Wii whose own SYNC button was
 * pressed can find us and pair. The console that pairs is remembered.
 * Afterwards stay connectable and non-discoverable.
 */
esp_err_t wiimote_sync(void);
bool wiimote_is_syncing(void);

/**
 * What a bonded remote does when you press a button: page the console synced
 * last and open the HID channels. Asynchronous; `connected` fires on success.
 * ESP_ERR_NOT_FOUND when nothing was synced yet. BTstack backend only.
 */
esp_err_t wiimote_connect(void);
/**
 * A page to the console (wiimote_connect) or an incoming console connection is
 * in progress. Calling wiimote_connect() meanwhile does nothing.
 */
bool wiimote_is_connecting(void);
/** Drop the console link (it can reconnect later). */
esp_err_t wiimote_disconnect(void);
/** A console has been synced and is remembered across reboots. */
bool wiimote_has_paired_host(void);
/** Forget the remembered console and its link key. */
esp_err_t wiimote_forget_host(void);

/**
 * Send any changed input to the console now instead of on the next 5 ms tick
 * (still at most one report per reporting period). Call after a batch of
 * setters, e.g. once per gamepad report. Thread-safe.
 */
void wiimote_send_now(void);
/** The ESP32 is master of the console link (BTstack; false when unknown). */
bool wiimote_link_is_master(void);
/**
 * HCI reason of the last console disconnect: 0x08 link timeout,
 * 0x13 console hung up, 0x15 console powered off, 0x16 we hung up.
 */
uint8_t wiimote_last_disconnect_reason(void);

/** Disconnect, stop being discoverable, and (standalone) stop being connectable. */
esp_err_t wiimote_stop(void);

esp_err_t wiimote_set_extension(wiimote_extension_t extension);
esp_err_t wiimote_set_button(wiimote_button_t button, bool pressed);

/**
 * Sticks: -1.0 .. 1.0, center 0. Triggers: 0.0 .. 1.0.
 * Mapped into the nunchuk (0–255) or classic (6-bit / 5-bit) ranges.
 */
esp_err_t wiimote_set_axis(wiimote_axis_t axis, float value);
esp_err_t wiimote_set_nunchuk_stick(uint8_t x, uint8_t y);

/** 10-bit raw samples, same units as the remote (0 g ≈ 512, +1 g ≈ +106). */
esp_err_t wiimote_set_accel(int16_t x, int16_t y, int16_t z);
esp_err_t wiimote_set_nunchuk_accel(int16_t x, int16_t y, int16_t z);
/** Body gyro in degrees per second. Pitch/roll/yaw follow the mapped sensor axes. */
esp_err_t wiimote_set_gyro(float pitch_dps, float roll_dps, float yaw_dps);

/** Dot index 0..3. size < 0 hides the dot. Coordinates are camera pixels (x 0–1023, y 0–767). */
esp_err_t wiimote_set_ir_dot(int index, int x, int y, int size);
/** Replace packed IR bytes for the current reporting mode. len 0 clears the override. */
esp_err_t wiimote_set_ir_raw(const uint8_t *data, size_t len);

esp_err_t wiimote_set_battery(uint8_t level);

/**
 * Bind a software motion source. Pass NULL to clear it.
 * While set, it is sampled instead of an IMU driver. Manual setters still work
 * between samples, and the sampler overwrites them on the next tick.
 */
esp_err_t wiimote_set_motion_callback(wiimote_motion_cb_t cb, void *user);

/**
 * Map sensor axes onto the Wiimote axes. map[i] is ±1, ±2, or ±3 (X, Y, Z).
 * NULL restores identity. +Z up while the remote lies face-up is +1 g.
 * Gyro: mapped X = pitch, Y = roll, Z = yaw.
 */
esp_err_t wiimote_set_imu_map(const int8_t map[3]);

/** addr 0 tries the module's default address, then the alternate. */
esp_err_t wiimote_use_mpu6050(int sda, int scl, uint8_t addr);
esp_err_t wiimote_use_mpu6500(int sda, int scl, uint8_t addr);
esp_err_t wiimote_use_lsm6ds3(int sda, int scl, uint8_t addr);

bool wiimote_is_connected(void);
/** Player LEDs last requested by the host. Bit 0 is LED 1. */
uint8_t wiimote_led_mask(void);

#ifdef __cplusplus
}
#endif

#endif
