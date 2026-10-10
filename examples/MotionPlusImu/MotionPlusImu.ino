/*
 * MPU6050 accel and gyro, exposed as a MotionPlus.
 * The body accel is the Wiimote accel. Gyro is pitch/roll/yaw in the
 * MotionPlus report, including the fast/slow rotation bit.
 *
 * Wiring:
 *   MPU6050 (GY-521) VCC 3V3, GND, SDA 21, SCL 22, AD0 to GND (address 0x68).
 *   A 4, B 5, + 16, - 17, Home 18, Up 19, Down 23, Left 13, Right 14.
 *   Player LEDs 26, 27, 32, 33. Rumble 25.
 *   Do not use GPIO 21 or 22 as buttons in this sketch.
 *
 * If the IMU is missing the remote still connects and reports the rest
 * gyro code (about 0x1F7F) plus a face-up +1 g accelerometer sample.
 *
 * Axis map is identity: sensor +X/+Y/+Z are copied onto the Wiimote axes.
 * Face-up +Z is +1 g. Pitch is mapped X, roll is mapped Y, yaw is mapped Z.
 * Pass a map to remote.setImuMap() if your module is rotated.
 */

#include <Wiimote.h>

#define PIN_A 4
#define PIN_B 5
#define PIN_PLUS 16
#define PIN_MINUS 17
#define PIN_HOME 18
#define PIN_UP 19
#define PIN_DOWN 23
#define PIN_LEFT 13
#define PIN_RIGHT 14
#define PIN_LED1 26
#define PIN_LED2 27
#define PIN_LED3 32
#define PIN_LED4 33
#define PIN_RUMBLE 25
#define PIN_SDA 21
#define PIN_SCL 22

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
    Serial.println("MotionPlusImu");

    wiimote_pins_init(&pins);
    pins.button_gpio[WIIMOTE_BUTTON_A] = PIN_A;
    pins.button_gpio[WIIMOTE_BUTTON_B] = PIN_B;
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
    config.extension = WIIMOTE_EXT_MOTION_PLUS;
    config.battery = 0xC0;
    config.pins = &pins;
    config.callbacks = &callbacks;

    if (!remote.begin(&config)) {
        Serial.println("wiimote_init failed");
        return;
    }
    if (!remote.useMpu6050(PIN_SDA, PIN_SCL, 0x68)) {
        Serial.println("MPU6050 not found; sending idle accel and gyro");
    } else {
        Serial.println("MPU6050 feeding accel + MotionPlus gyro");
    }
    if (!remote.sync()) {
        Serial.println("sync failed");
        return;
    }
    Serial.println("syncing for 20 seconds");
}

void loop()
{
    if (link_event) {
        link_event = false;
        Serial.println(link_up ? "connected" : "disconnected");
    }
    delay(20);
}
