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
 * Pass a map to wiimote.setImuMap() if your module is rotated.
 */

#include <Wiimote.h>

static Wiimote wiimote;
static volatile bool g_connected;
static volatile bool g_event;

void setup()
{
    Serial.begin(115200);
    delay(200);
    Serial.println("MotionPlusImu");

    wiimote_pins_t pins;
    wiimote_pins_init(&pins);
    pins.button_gpio[WIIMOTE_BUTTON_A] = 4;
    pins.button_gpio[WIIMOTE_BUTTON_B] = 5;
    pins.button_gpio[WIIMOTE_BUTTON_PLUS] = 16;
    pins.button_gpio[WIIMOTE_BUTTON_MINUS] = 17;
    pins.button_gpio[WIIMOTE_BUTTON_HOME] = 18;
    pins.button_gpio[WIIMOTE_BUTTON_UP] = 19;
    pins.button_gpio[WIIMOTE_BUTTON_DOWN] = 23;
    pins.button_gpio[WIIMOTE_BUTTON_LEFT] = 13;
    pins.button_gpio[WIIMOTE_BUTTON_RIGHT] = 14;
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

    if (!wiimote.begin(WIIMOTE_EXT_MOTION_PLUS, pins)) {
        Serial.println("begin failed");
        return;
    }
    Serial.println(wiimote.useMpu6050(21, 22, 0x68) ? "MPU6050 feeding accel + MotionPlus gyro"
                                                    : "MPU6050 not found; sending idle accel and gyro");
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
