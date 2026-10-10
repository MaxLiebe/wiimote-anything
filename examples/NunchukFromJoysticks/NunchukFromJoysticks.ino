/*
 * Core buttons plus two analog sticks reported as a Nunchuk.
 * Accel stays at the idle +1 g pose unless USE_MPU6050 is set.
 *
 * Wiring:
 *   Same face buttons as BasicButtons, except Left/Right move off 21/22
 *   when the MPU6050 is enabled (those pins become I2C).
 *   Stick X GPIO 36 (ADC1 ch0), stick Y GPIO 39 (ADC1 ch3).
 *   Player LEDs 26, 27, 32, 33. Rumble 25.
 *   MPU6050 (optional): SDA 21, SCL 22, AD0 low so the address is 0x68.
 *   VCC 3V3, not 5 V.
 *
 * ADC defaults: 12-bit, center 2048, deadzone 200. Edit the pin struct if
 * your sticks do not rest at mid-scale.
 */

#include <Wiimote.h>

#define USE_MPU6050 0

#define PIN_A 4
#define PIN_B 5
#define PIN_ONE 13
#define PIN_TWO 14
#define PIN_PLUS 16
#define PIN_MINUS 17
#define PIN_HOME 18
#define PIN_UP 19
#define PIN_DOWN 23
#define PIN_LED1 26
#define PIN_LED2 27
#define PIN_LED3 32
#define PIN_LED4 33
#define PIN_RUMBLE 25
#define PIN_STICK_X 36
#define PIN_STICK_Y 39
#define PIN_SDA 21
#define PIN_SCL 22

#if USE_MPU6050
#define PIN_LEFT -1
#define PIN_RIGHT -1
#else
#define PIN_LEFT 21
#define PIN_RIGHT 22
#endif

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
    Serial.println("NunchukFromJoysticks");

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
    pins.button_gpio[WIIMOTE_BUTTON_NUNCHUK_C] = -1;
    pins.button_gpio[WIIMOTE_BUTTON_NUNCHUK_Z] = -1;
    pins.led_gpio[0] = PIN_LED1;
    pins.led_gpio[1] = PIN_LED2;
    pins.led_gpio[2] = PIN_LED3;
    pins.led_gpio[3] = PIN_LED4;
    pins.rumble_gpio = PIN_RUMBLE;
    pins.axis[WIIMOTE_AXIS_NUNCHUK_X].gpio = PIN_STICK_X;
    pins.axis[WIIMOTE_AXIS_NUNCHUK_Y].gpio = PIN_STICK_Y;

    callbacks.connected = on_connected;
    callbacks.disconnected = on_disconnected;
    config.extension = WIIMOTE_EXT_NUNCHUK;
    config.battery = 0xC0;
    config.pins = &pins;
    config.callbacks = &callbacks;

    if (!remote.begin(&config)) {
        Serial.println("wiimote_init failed");
        return;
    }
#if USE_MPU6050
    if (!remote.useMpu6050(PIN_SDA, PIN_SCL, 0x68)) {
        Serial.println("MPU6050 not found; nunchuk accel stays idle");
    } else {
        Serial.println("MPU6050 feeding the Wiimote accel");
    }
#else
    Serial.println("nunchuk accel is the idle +1g pose");
#endif
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
