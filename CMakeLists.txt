# WiimoteAnything as an ESP-IDF component (the component name is this
# directory's name).
set(WIIMOTE_REQUIRES bt nvs_flash esp_timer freertos)

if(DEFINED IDF_VERSION_MAJOR AND IDF_VERSION_MAJOR GREATER_EQUAL 5)
    list(APPEND WIIMOTE_REQUIRES esp_adc)
endif()

if(DEFINED IDF_VERSION_MAJOR AND (IDF_VERSION_MAJOR GREATER 5 OR (IDF_VERSION_MAJOR EQUAL 5 AND IDF_VERSION_MINOR GREATER_EQUAL 3)))
    # 5.3+ split the drivers into their own components.
    list(APPEND WIIMOTE_REQUIRES esp_driver_gpio esp_driver_ledc esp_driver_i2c)
else()
    list(APPEND WIIMOTE_REQUIRES driver)
endif()

# BTstack backend when the project provides BTstack (Bluepad32 or BTstack's
# ESP32 port set BTSTACK_ROOT); Bluedroid otherwise.
if(TARGET btstack OR DEFINED BTSTACK_ROOT OR DEFINED ENV{BTSTACK_ROOT})
    list(APPEND WIIMOTE_REQUIRES btstack)
endif()

idf_component_register(
    SRCS
        "src/wiimote.c"
        "src/wiimote_bt.c"
        "src/wiimote_hw.c"
        "src/wiimote_imu.c"
        "src/wiimote_proto.c"
        "src/wiimote_ext.c"
        "src/wiimote_crypto.c"
    INCLUDE_DIRS "include"
    REQUIRES ${WIIMOTE_REQUIRES}
)
