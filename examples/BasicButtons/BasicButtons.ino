/*
 * Buttons, player LEDs, and rumble. No IMU.
 * Enough to sync to a Wii and walk the system menu.
 *
 * Wiring (original ESP32 only, buttons active-low to GND):
 *   A 4, B 5, 1 13, 2 14, + 16, - 17, Home 18
 *   Up 19, Down 23, Left 21, Right 22
 *   Player LEDs 26, 27, 32, 33 (active high)
 *   Rumble motor transistor on 25
 *
 * Open the serial monitor at 115200. Reset the ESP32, then while it is
 * inside the 20 second window press the red SYNC button on the Wii
 * console face (not just power). The Wii's player LEDs blink while it
 * searches. The sketch prints "connected" once both HID channels are up.
 */

#include <Wiimote.h>

static Wiimote wiimote;
static volatile bool g_connected;
static volatile bool g_event;

void setup()
{
    Serial.begin(115200);
    delay(200);
    Serial.println("BasicButtons");

    wiimote_pins_t pins;
    wiimote_pins_init(&pins); /* every pin unused, buttons active-low */
    pins.button_gpio[WIIMOTE_BUTTON_A] = 4;
    pins.button_gpio[WIIMOTE_BUTTON_B] = 5;
    pins.button_gpio[WIIMOTE_BUTTON_ONE] = 13;
    pins.button_gpio[WIIMOTE_BUTTON_TWO] = 14;
    pins.button_gpio[WIIMOTE_BUTTON_PLUS] = 16;
    pins.button_gpio[WIIMOTE_BUTTON_MINUS] = 17;
    pins.button_gpio[WIIMOTE_BUTTON_HOME] = 18;
    pins.button_gpio[WIIMOTE_BUTTON_UP] = 19;
    pins.button_gpio[WIIMOTE_BUTTON_DOWN] = 23;
    pins.button_gpio[WIIMOTE_BUTTON_LEFT] = 21;
    pins.button_gpio[WIIMOTE_BUTTON_RIGHT] = 22;
    pins.led_gpio[0] = 26;
    pins.led_gpio[1] = 27;
    pins.led_gpio[2] = 32;
    pins.led_gpio[3] = 33;
    pins.rumble_gpio = 25;

    wiimote.onConnect([] {
        g_connected = true;
        g_event = true;
    });
    wiimote.onDisconnect([] {
        g_connected = false;
        g_event = true;
    });

    if (!wiimote.begin(WIIMOTE_EXT_NONE, pins)) {
        Serial.println("begin failed");
        return;
    }
    if (wiimote.sync()) {
        Serial.println("press the red SYNC button on the Wii now (20 s)");
    }
}

void loop()
{
    if (g_event) {
        g_event = false;
        Serial.println(g_connected ? "connected" : "disconnected");
    }
    delay(20);
}
