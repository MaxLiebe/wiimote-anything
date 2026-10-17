/*
 * Core buttons plus an analog stick reported as a Nunchuk.
 * Accel stays at the idle +1 g pose unless USE_MPU6050 is set.
 *
 * Wiring:
 *   Same face buttons as BasicButtons, except Left/Right move off 21/22
 *   when the MPU6050 is enabled (those pins become I2C).
 *   Stick X GPIO 36 (ADC1 ch0), stick Y GPIO 39 (ADC1 ch3).
 *   Nunchuk C / Z are not wired; set pins.button_gpio[WIIMOTE_BUTTON_NUNCHUK_C]
 *   and [WIIMOTE_BUTTON_NUNCHUK_Z] to add them.
 *   Player LEDs 26, 27, 32, 33. Rumble 25.
 *   MPU6050 (optional): SDA 21, SCL 22, AD0 low so the address is 0x68.
 *   VCC 3V3, not 5 V.
 *
 * ADC defaults: 12-bit, center 2048, deadzone 200. Edit pins.axis[] if your
 * stick does not rest at mid-scale.
 */

#include <Wiimote.h>

#define USE_MPU6050 0

static Wiimote wiimote;
static volatile bool g_connected;
static volatile bool g_event;

void setup()
{
    Serial.begin(115200);
    delay(200);
    Serial.println("NunchukFromJoysticks");

    wiimote_pins_t pins;
    wiimote_pins_init(&pins);
    pins.button_gpio[WIIMOTE_BUTTON_A] = 4;
    pins.button_gpio[WIIMOTE_BUTTON_B] = 5;
    pins.button_gpio[WIIMOTE_BUTTON_ONE] = 13;
    pins.button_gpio[WIIMOTE_BUTTON_TWO] = 14;
    pins.button_gpio[WIIMOTE_BUTTON_PLUS] = 16;
    pins.button_gpio[WIIMOTE_BUTTON_MINUS] = 17;
    pins.button_gpio[WIIMOTE_BUTTON_HOME] = 18;
    pins.button_gpio[WIIMOTE_BUTTON_UP] = 19;
    pins.button_gpio[WIIMOTE_BUTTON_DOWN] = 23;
#if !USE_MPU6050
    pins.button_gpio[WIIMOTE_BUTTON_LEFT] = 21;
    pins.button_gpio[WIIMOTE_BUTTON_RIGHT] = 22;
#endif
    pins.axis[WIIMOTE_AXIS_NUNCHUK_X].gpio = 36;
    pins.axis[WIIMOTE_AXIS_NUNCHUK_Y].gpio = 39;
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

    if (!wiimote.begin(WIIMOTE_EXT_NUNCHUK, pins)) {
        Serial.println("begin failed");
        return;
    }
#if USE_MPU6050
    Serial.println(wiimote.useMpu6050(21, 22) ? "MPU6050 feeding the Wiimote accel"
                                              : "MPU6050 not found; accel stays idle");
#endif
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
