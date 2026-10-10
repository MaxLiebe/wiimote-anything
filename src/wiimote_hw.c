/* SPDX-License-Identifier: MIT */
#include "wiimote_hw.h"

#include "wiimote_internal.h"
#include "wiimote_target.h"

#include <stdlib.h>
#include <string.h>

#include "driver/gpio.h"
#include "driver/ledc.h"
#include "esp_idf_version.h"
#include "esp_log.h"

#if ESP_IDF_VERSION_MAJOR >= 5
#include "esp_adc/adc_oneshot.h"
#else
#include "driver/adc.h"
#endif

static const char *TAG = "wiimote";

static wiimote_pins_t s_pins;
static bool s_ready;
static bool s_pwm;
#if ESP_IDF_VERSION_MAJOR >= 5
static adc_oneshot_unit_handle_t s_adc;
static bool s_adc_ready[8];
#endif

static int gpio_to_adc1(int gpio)
{
    switch (gpio) {
    case 36: return 0;
    case 37: return 1;
    case 38: return 2;
    case 39: return 3;
    case 32: return 4;
    case 33: return 5;
    case 34: return 6;
    case 35: return 7;
    default: return -1;
    }
}

static int map_bipolar(int raw, const wiimote_axis_cal_t *cal, int out_min, int out_max, int center)
{
    int span;
    if (abs(raw - cal->center) <= cal->deadzone) {
        return center;
    }
    if (raw <= cal->center) {
        span = cal->center - cal->min;
        if (span <= 0) {
            return center;
        }
        if (raw < cal->min) {
            raw = cal->min;
        }
        return out_min + ((raw - cal->min) * (center - out_min)) / span;
    }
    span = cal->max - cal->center;
    if (span <= 0) {
        return center;
    }
    if (raw > cal->max) {
        raw = cal->max;
    }
    return center + ((raw - cal->center) * (out_max - center)) / span;
}

static int map_trigger(int raw, const wiimote_axis_cal_t *cal)
{
    int span = cal->max - cal->min;
    if (span <= 0) {
        return 0;
    }
    if (raw < cal->min + cal->deadzone) {
        return 0;
    }
    if (raw > cal->max) {
        raw = cal->max;
    }
    return ((raw - cal->min) * 31) / span;
}

static int axis_out_range(wiimote_axis_t axis, int *lo, int *hi, int *center, bool *trigger)
{
    *trigger = false;
    switch (axis) {
    case WIIMOTE_AXIS_NUNCHUK_X:
    case WIIMOTE_AXIS_NUNCHUK_Y:
        *lo = 0; *hi = 255; *center = 128;
        return 0;
    case WIIMOTE_AXIS_CLASSIC_LX:
    case WIIMOTE_AXIS_CLASSIC_LY:
        *lo = 0; *hi = 63; *center = 32;
        return 0;
    case WIIMOTE_AXIS_CLASSIC_RX:
    case WIIMOTE_AXIS_CLASSIC_RY:
        *lo = 0; *hi = 31; *center = 16;
        return 0;
    case WIIMOTE_AXIS_CLASSIC_LT:
    case WIIMOTE_AXIS_CLASSIC_RT:
        *lo = 0; *hi = 31; *center = 0;
        *trigger = true;
        return 0;
    default:
        return -1;
    }
}

static int read_adc(int gpio)
{
    int channel = gpio_to_adc1(gpio);
    int raw = 0;
    if (channel < 0) {
        return 0;
    }
#if ESP_IDF_VERSION_MAJOR >= 5
    if (s_adc == NULL || !s_adc_ready[channel]) {
        return 0;
    }
    if (adc_oneshot_read(s_adc, (adc_channel_t)channel, &raw) != ESP_OK) {
        return 0;
    }
#else
    raw = adc1_get_raw((adc1_channel_t)channel);
#endif
    return raw;
}

static void config_output(int gpio)
{
    gpio_config_t cfg = {
        .pin_bit_mask = 1ULL << gpio,
        .mode = GPIO_MODE_OUTPUT,
        .pull_up_en = GPIO_PULLUP_DISABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type = GPIO_INTR_DISABLE,
    };
    gpio_config(&cfg);
    gpio_set_level((gpio_num_t)gpio, 0);
}

void wiimote_hw_init(const wiimote_pins_t *pins)
{
    int i;
    bool need_adc = false;
    if (pins == NULL) {
        s_ready = false;
        return;
    }
    s_pins = *pins;
    for (i = 0; i < WIIMOTE_BUTTON_COUNT; i++) {
        int gpio = s_pins.button_gpio[i];
        gpio_config_t cfg;
        if (gpio < 0 || gpio > 39) {
            s_pins.button_gpio[i] = -1;
            continue;
        }
        cfg.pin_bit_mask = 1ULL << gpio;
        cfg.mode = GPIO_MODE_INPUT;
        cfg.pull_up_en = s_pins.buttons_active_low ? GPIO_PULLUP_ENABLE : GPIO_PULLUP_DISABLE;
        cfg.pull_down_en = s_pins.buttons_active_low ? GPIO_PULLDOWN_DISABLE : GPIO_PULLDOWN_ENABLE;
        cfg.intr_type = GPIO_INTR_DISABLE;
        gpio_config(&cfg);
    }
    for (i = 0; i < 4; i++) {
        if (s_pins.led_gpio[i] >= 0) {
            config_output(s_pins.led_gpio[i]);
        }
    }
    if (s_pins.rumble_gpio >= 0 && !s_pins.rumble_pwm) {
        config_output(s_pins.rumble_gpio);
    }
    if (s_pins.rumble_gpio >= 0 && s_pins.rumble_pwm) {
        ledc_timer_config_t timer = {
            .speed_mode = LEDC_LOW_SPEED_MODE,
            .duty_resolution = LEDC_TIMER_8_BIT,
            .timer_num = LEDC_TIMER_0,
            .freq_hz = 1000,
            .clk_cfg = LEDC_AUTO_CLK,
        };
        ledc_channel_config_t channel = {
            .gpio_num = s_pins.rumble_gpio,
            .speed_mode = LEDC_LOW_SPEED_MODE,
            .channel = LEDC_CHANNEL_0,
            .intr_type = LEDC_INTR_DISABLE,
            .timer_sel = LEDC_TIMER_0,
            .duty = 0,
            .hpoint = 0,
        };
        if (ledc_timer_config(&timer) == ESP_OK && ledc_channel_config(&channel) == ESP_OK) {
            s_pwm = true;
        } else {
            ESP_LOGW(TAG, "LEDC rumble setup failed; rumble pin left idle");
        }
    }
    for (i = 0; i < WIIMOTE_AXIS_COUNT; i++) {
        if (s_pins.axis[i].gpio >= 0) {
            need_adc = true;
        }
    }
    if (need_adc) {
#if ESP_IDF_VERSION_MAJOR >= 5
        adc_oneshot_unit_init_cfg_t unit = {
            .unit_id = ADC_UNIT_1,
            .ulp_mode = ADC_ULP_MODE_DISABLE,
        };
        if (adc_oneshot_new_unit(&unit, &s_adc) != ESP_OK) {
            ESP_LOGW(TAG, "ADC1 init failed; sticks stay centered");
            s_adc = NULL;
        }
#else
        adc1_config_width(ADC_WIDTH_BIT_12);
#endif
        for (i = 0; i < WIIMOTE_AXIS_COUNT; i++) {
            int channel = gpio_to_adc1(s_pins.axis[i].gpio);
            if (s_pins.axis[i].gpio < 0) {
                continue;
            }
            if (channel < 0) {
                ESP_LOGW(TAG, "GPIO %d is not an ADC1 channel", s_pins.axis[i].gpio);
                s_pins.axis[i].gpio = -1;
                continue;
            }
#if ESP_IDF_VERSION_MAJOR >= 5
            if (s_adc != NULL) {
                adc_oneshot_chan_cfg_t chan = {
                    .bitwidth = ADC_BITWIDTH_12,
                    /* 12 dB is the same register value as the old 11 dB name.
                     * IDF 5.0.0–5.0.4 and 5.1.0–5.1.2 only have ADC_ATTEN_DB_11. */
#if ESP_IDF_VERSION >= ESP_IDF_VERSION_VAL(5, 1, 3) || \
    (ESP_IDF_VERSION >= ESP_IDF_VERSION_VAL(5, 0, 5) && ESP_IDF_VERSION < ESP_IDF_VERSION_VAL(5, 1, 0))
                    .atten = ADC_ATTEN_DB_12,
#else
                    .atten = ADC_ATTEN_DB_11,
#endif
                };
                if (adc_oneshot_config_channel(s_adc, (adc_channel_t)channel, &chan) == ESP_OK) {
                    s_adc_ready[channel] = true;
                }
            }
#else
            adc1_config_channel_atten((adc1_channel_t)channel, ADC_ATTEN_DB_11);
#endif
        }
    }
    s_ready = true;
}

void wiimote_hw_set_leds(uint8_t mask)
{
    int i;
    if (!s_ready) {
        return;
    }
    for (i = 0; i < 4; i++) {
        if (s_pins.led_gpio[i] >= 0) {
            gpio_set_level((gpio_num_t)s_pins.led_gpio[i], (mask & (1u << i)) ? 1 : 0);
        }
    }
}

void wiimote_hw_blink(uint8_t led_count, bool lit)
{
    uint8_t mask = 0;
    uint8_t i;
    if (led_count > 4) {
        led_count = 4;
    }
    if (lit) {
        for (i = 0; i < led_count; i++) {
            mask |= (uint8_t)(1u << i);
        }
    }
    wiimote_hw_set_leds(mask);
}

void wiimote_hw_set_rumble(bool on)
{
    if (!s_ready || s_pins.rumble_gpio < 0) {
        return;
    }
    if (s_pwm) {
        ledc_set_duty(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL_0, on ? 200 : 0);
        ledc_update_duty(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL_0);
        return;
    }
    gpio_set_level((gpio_num_t)s_pins.rumble_gpio, on ? 1 : 0);
}

void wiimote_hw_poll(void)
{
    wiimote_proto_t *proto;
    int i;
    if (!s_ready || !wiimote_ready()) {
        return;
    }
    proto = wiimote_state();
    wiimote_lock();
    for (i = 0; i < WIIMOTE_BUTTON_COUNT; i++) {
        int level;
        bool down;
        if (s_pins.button_gpio[i] < 0) {
            continue;
        }
        level = gpio_get_level((gpio_num_t)s_pins.button_gpio[i]);
        down = s_pins.buttons_active_low ? (level == 0) : (level != 0);
        wiimote_proto_set_button(proto, (wiimote_button_t)i, down);
    }
    for (i = 0; i < WIIMOTE_AXIS_COUNT; i++) {
        int lo, hi, center, raw;
        bool trigger = false;
        if (s_pins.axis[i].gpio < 0) {
            continue;
        }
        if (axis_out_range((wiimote_axis_t)i, &lo, &hi, &center, &trigger) != 0) {
            continue;
        }
        raw = read_adc(s_pins.axis[i].gpio);
        if (trigger) {
            wiimote_proto_set_axis_raw(proto, (wiimote_axis_t)i, map_trigger(raw, &s_pins.axis[i]));
        } else {
            wiimote_proto_set_axis_raw(proto, (wiimote_axis_t)i,
                                       map_bipolar(raw, &s_pins.axis[i], lo, hi, center));
        }
    }
    wiimote_unlock();
}
