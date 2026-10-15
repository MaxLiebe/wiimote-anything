/* SPDX-License-Identifier: MIT */
/*
 * Standalone BTstack Wiimote: WiimoteAnything owns BTstack (no Bluepad32).
 *
 * GPIO 4 / 5 are Wii Remote A / B (to GND). The BOOT button (GPIO 0) is the
 * remote's SYNC button. After the first sync the console is remembered; on the
 * next boot, press A to reconnect, the way a bonded remote does.
 */
#include "driver/gpio.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "wiimote.h"

static const char *TAG = "wiimote_standalone";

#define SYNC_GPIO 0
#define BUTTON_A_GPIO 4
#define BUTTON_B_GPIO 5

static void on_connected(void *user)
{
    (void)user;
    ESP_LOGI(TAG, "Connected to the Wii");
}

static void on_disconnected(void *user)
{
    (void)user;
    ESP_LOGI(TAG, "Disconnected from the Wii");
}

static void on_leds(uint8_t mask, void *user)
{
    (void)user;
    ESP_LOGI(TAG, "Player LEDs 0x%x", mask);
}

static void on_rumble(bool on, void *user)
{
    (void)user;
    ESP_LOGI(TAG, "Rumble %s", on ? "on" : "off");
}

void app_main(void)
{
    wiimote_pins_t pins;
    wiimote_pins_init(&pins);
    pins.button_gpio[WIIMOTE_BUTTON_A] = BUTTON_A_GPIO;
    pins.button_gpio[WIIMOTE_BUTTON_B] = BUTTON_B_GPIO;
    pins.led_gpio[0] = 26;
    pins.rumble_gpio = 25;

    static const wiimote_callbacks_t cb = {
        .connected = on_connected,
        .disconnected = on_disconnected,
        .leds = on_leds,
        .rumble = on_rumble,
    };
    const wiimote_config_t cfg = {
        .extension = WIIMOTE_EXT_NONE,
        .battery = 0xC0,
        .pins = &pins,
        .callbacks = &cb,
        .external_stack = false, /* WiimoteAnything starts BTstack itself. */
    };
    ESP_ERROR_CHECK(wiimote_init(&cfg));

    gpio_config_t io = {
        .pin_bit_mask = 1ULL << SYNC_GPIO,
        .mode = GPIO_MODE_INPUT,
        .pull_up_en = GPIO_PULLUP_ENABLE,
    };
    gpio_config(&io);

    if (!wiimote_has_paired_host()) {
        ESP_LOGI(TAG, "No Wii paired yet. Press the red SYNC button on the Wii.");
        wiimote_sync();
    } else {
        ESP_LOGI(TAG, "Paired. Press A to connect, BOOT to sync again.");
    }

    bool was_down = false;
    bool a_was_down = false;
    while (true) {
        bool down = gpio_get_level(SYNC_GPIO) == 0;
        bool a_down = gpio_get_level(BUTTON_A_GPIO) == 0;
        if (down && !was_down) {
            wiimote_sync();
        }
        if (a_down && !a_was_down && !wiimote_is_connected() && !wiimote_is_syncing()) {
            wiimote_connect();
        }
        was_down = down;
        a_was_down = a_down;
        vTaskDelay(pdMS_TO_TICKS(30));
    }
}
