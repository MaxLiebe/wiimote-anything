/* SPDX-License-Identifier: MIT */
/*
 * Standalone BTstack Wiimote Example
 *
 * Runs WiimoteAnything on the original ESP32 using BTstack directly (no Arduino, no Bluepad32).
 */
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "wiimote.h"

static const char *TAG = "wiimote_btstack_app";

static void on_connected(void *user)
{
    (void)user;
    ESP_LOGI(TAG, "Wiimote connected to Wii console!");
}

static void on_disconnected(void *user)
{
    (void)user;
    ESP_LOGI(TAG, "Wiimote disconnected from Wii console");
}

static void on_leds(uint8_t mask, void *user)
{
    (void)user;
    ESP_LOGI(TAG, "Wii console updated LEDs: 0x%02x", mask);
}

static void on_rumble(bool on, void *user)
{
    (void)user;
    ESP_LOGI(TAG, "Wii console rumble: %s", on ? "ON" : "OFF");
}

void app_main(void)
{
    ESP_LOGI(TAG, "Starting standalone BTstack Wiimote...");

    wiimote_pins_t pins;
    wiimote_pins_init(&pins);

    /* GPIO button wiring (active low to GND) */
    pins.button_gpio[WIIMOTE_BUTTON_A] = 4;
    pins.button_gpio[WIIMOTE_BUTTON_B] = 5;
    pins.led_gpio[0] = 26;
    pins.rumble_gpio = 25;

    wiimote_callbacks_t cb = {
        .connected = on_connected,
        .disconnected = on_disconnected,
        .leds = on_leds,
        .rumble = on_rumble,
    };

    wiimote_config_t cfg = {
        .extension = WIIMOTE_EXT_NONE,
        .battery = 0xC0,
        .pins = &pins,
        .callbacks = &cb,
        .external_stack = false, /* WiimoteAnything manages BTstack standalone */
    };

    ESP_ERROR_CHECK(wiimote_init(&cfg));
    ESP_ERROR_CHECK(wiimote_sync());

    ESP_LOGI(TAG, "Advertising as Nintendo RVL-CNT-01 (press SYNC on Wii console)");

    while (true) {
        vTaskDelay(pdMS_TO_TICKS(1000));
    }
}
