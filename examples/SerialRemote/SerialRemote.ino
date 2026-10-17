/*
 * A Wii Remote with a Classic Controller, driven from the serial monitor.
 * Nothing to wire: a bare original-ESP32 dev board is enough. Use it to try
 * the library, or as the starting point for feeding input from your own code.
 *
 * Open the serial monitor at 115200 (line ending: any). Then, within 20 s,
 * press the red SYNC button on the Wii (behind the SD card door).
 *
 * Each character taps one button for 100 ms:
 *   a b 1 2      Wii Remote A, B, 1, 2
 *   + - h        Plus, Minus, Home
 *   w s q e      D-pad up, down, left, right   (q/e: left/right)
 *   x y z        Classic x, y, ZL
 *   < >          Classic left stick full left / right for 300 ms
 *   S            open the sync window again
 */

#include <Wiimote.h>

static Wiimote wiimote;

static volatile bool g_event;
static volatile bool g_connected;
static volatile uint8_t g_leds;

static wiimote_button_t g_held_button = WIIMOTE_BUTTON_COUNT;
static uint32_t g_release_at;
static bool g_stick_moved;
static uint32_t g_stick_release_at;

static void tap(wiimote_button_t button)
{
    if (g_held_button != WIIMOTE_BUTTON_COUNT) {
        wiimote.release(g_held_button);
    }
    g_held_button = button;
    g_release_at = millis() + 100;
    wiimote.press(button);
}

static void push_stick(float x)
{
    wiimote.setAxis(WIIMOTE_AXIS_CLASSIC_LX, x);
    g_stick_moved = true;
    g_stick_release_at = millis() + 300;
}

static void handle_char(char c)
{
    switch (c) {
    case 'a': tap(WIIMOTE_BUTTON_A); break;
    case 'b': tap(WIIMOTE_BUTTON_B); break;
    case '1': tap(WIIMOTE_BUTTON_ONE); break;
    case '2': tap(WIIMOTE_BUTTON_TWO); break;
    case '+': tap(WIIMOTE_BUTTON_PLUS); break;
    case '-': tap(WIIMOTE_BUTTON_MINUS); break;
    case 'h': tap(WIIMOTE_BUTTON_HOME); break;
    case 'w': tap(WIIMOTE_BUTTON_UP); break;
    case 's': tap(WIIMOTE_BUTTON_DOWN); break;
    case 'q': tap(WIIMOTE_BUTTON_LEFT); break;
    case 'e': tap(WIIMOTE_BUTTON_RIGHT); break;
    case 'x': tap(WIIMOTE_BUTTON_CLASSIC_X); break;
    case 'y': tap(WIIMOTE_BUTTON_CLASSIC_Y); break;
    case 'z': tap(WIIMOTE_BUTTON_CLASSIC_ZL); break;
    case '<': push_stick(-1.0f); break;
    case '>': push_stick(1.0f); break;
    case 'S':
        Serial.println(wiimote.sync() ? "sync window open for 20 s" : "sync failed");
        break;
    default:
        break;
    }
}

void setup()
{
    Serial.begin(115200);
    delay(200);
    Serial.println("SerialRemote");

    wiimote.onConnect([] {
        g_connected = true;
        g_event = true;
    });
    wiimote.onDisconnect([] {
        g_connected = false;
        g_event = true;
    });
    wiimote.onLeds([](uint8_t mask) {
        g_leds = mask;
        g_event = true;
    });

    if (!wiimote.begin(WIIMOTE_EXT_CLASSIC)) {
        Serial.println("begin failed (original ESP32 with Bluetooth Classic only)");
        return;
    }
    if (wiimote.hasPairedHost() && wiimote.connect()) {
        Serial.println("reconnecting to the Wii synced before");
    } else if (wiimote.sync()) {
        Serial.println("press the red SYNC button on the Wii now (20 s)");
    }
}

void loop()
{
    while (Serial.available() > 0) {
        handle_char((char)Serial.read());
    }
    uint32_t now = millis();
    if (g_held_button != WIIMOTE_BUTTON_COUNT && (int32_t)(now - g_release_at) >= 0) {
        wiimote.release(g_held_button);
        g_held_button = WIIMOTE_BUTTON_COUNT;
    }
    if (g_stick_moved && (int32_t)(now - g_stick_release_at) >= 0) {
        wiimote.setAxis(WIIMOTE_AXIS_CLASSIC_LX, 0.0f);
        g_stick_moved = false;
    }
    if (g_event) {
        g_event = false;
        Serial.printf("%s, player LEDs 0x%x\n", g_connected ? "connected" : "disconnected", g_leds);
    }
    delay(5);
}
