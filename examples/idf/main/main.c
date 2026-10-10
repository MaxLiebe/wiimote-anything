/* Smoke app so the component can be built with idf.py. Not an Arduino sketch. */
#include "esp_log.h"
#include "wiimote.h"

void app_main(void)
{
    wiimote_pins_t pins;
    wiimote_config_t cfg = {
        .extension = WIIMOTE_EXT_NONE,
        .battery = 0xC0,
        .pins = &pins,
    };

    wiimote_pins_init(&pins);
    pins.button_gpio[WIIMOTE_BUTTON_A] = 4;
    pins.led_gpio[0] = 26;
    pins.rumble_gpio = 25;

    ESP_ERROR_CHECK(wiimote_init(&cfg));
    ESP_ERROR_CHECK(wiimote_sync());
    ESP_LOGI("wiimote", "smoke app advertising as Nintendo RVL-CNT-01");
}
