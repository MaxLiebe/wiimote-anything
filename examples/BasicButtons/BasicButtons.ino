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
 * Open the serial monitor at 115200. Press the red sync button on the Wii,
 * then reset the ESP32 (or wait for the LED chase). The remote stays
 * connectable after the 20 second discoverable window.
 */

#include <Wiimote.h>

#define PIN_A 4
#define PIN_B 5
#define PIN_ONE 13
#define PIN_TWO 14
#define PIN_PLUS 16
#define PIN_MINUS 17
#define PIN_HOME 18
#define PIN_UP 19
#define PIN_DOWN 23
#define PIN_LEFT 21
#define PIN_RIGHT 22
#define PIN_LED1 26
#define PIN_LED2 27
#define PIN_LED3 32
#define PIN_LED4 33
#define PIN_RUMBLE 25

static Wiimote remote;
static volatile bool link_up;
static volatile bool link_event;

static void on_connected(void *user)
{
    (void)user;
    link_up = true;
    link_event = true;
}

static void on_disconnected(void *user)
{
    (void)user;
    link_up = false;
    link_event = true;
}

void setup()
{
    wiimote_pins_t pins;
    wiimote_callbacks_t callbacks = {};
    wiimote_config_t config = {};

    Serial.begin(115200);
    delay(200);
    Serial.println("BasicButtons: syncing as Nintendo RVL-CNT-01");

    wiimote_pins_init(&pins);
    pins.button_gpio[WIIMOTE_BUTTON_A] = PIN_A;
    pins.button_gpio[WIIMOTE_BUTTON_B] = PIN_B;
    pins.button_gpio[WIIMOTE_BUTTON_ONE] = PIN_ONE;
    pins.button_gpio[WIIMOTE_BUTTON_TWO] = PIN_TWO;
    pins.button_gpio[WIIMOTE_BUTTON_PLUS] = PIN_PLUS;
    pins.button_gpio[WIIMOTE_BUTTON_MINUS] = PIN_MINUS;
    pins.button_gpio[WIIMOTE_BUTTON_HOME] = PIN_HOME;
    pins.button_gpio[WIIMOTE_BUTTON_UP] = PIN_UP;
    pins.button_gpio[WIIMOTE_BUTTON_DOWN] = PIN_DOWN;
    pins.button_gpio[WIIMOTE_BUTTON_LEFT] = PIN_LEFT;
    pins.button_gpio[WIIMOTE_BUTTON_RIGHT] = PIN_RIGHT;
    pins.led_gpio[0] = PIN_LED1;
    pins.led_gpio[1] = PIN_LED2;
    pins.led_gpio[2] = PIN_LED3;
    pins.led_gpio[3] = PIN_LED4;
    pins.rumble_gpio = PIN_RUMBLE;

    callbacks.connected = on_connected;
    callbacks.disconnected = on_disconnected;
    config.extension = WIIMOTE_EXT_NONE;
    config.battery = 0xC0;
    config.pins = &pins;
    config.callbacks = &callbacks;

    if (!remote.begin(&config)) {
        Serial.println("wiimote_init failed");
        return;
    }
    if (!remote.sync()) {
        Serial.println("sync failed");
        return;
    }
    Serial.println("discoverable for 20 seconds");
}

void loop()
{
    if (link_event) {
        link_event = false;
        Serial.println(link_up ? "connected" : "disconnected");
    }
    delay(20);
}
