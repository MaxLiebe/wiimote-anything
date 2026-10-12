cmake_minimum_required(VERSION 3.16)
set(CMAKE_C_STANDARD 11)
set(CMAKE_CXX_STANDARD 17)

set(WIIMOTE_ROOT "${CMAKE_CURRENT_LIST_DIR}/../..")

# Locate Bluepad32 (either in example local components/bluepad32 or repo root components/bluepad32)
if(EXISTS "${CMAKE_CURRENT_LIST_DIR}/components/bluepad32")
    set(BLUEPAD32_ROOT "${CMAKE_CURRENT_LIST_DIR}/components/bluepad32")
elseif(EXISTS "${WIIMOTE_ROOT}/components/bluepad32")
    set(BLUEPAD32_ROOT "${WIIMOTE_ROOT}/components/bluepad32")
else()
    message(FATAL_ERROR "Bluepad32 not found. Clone it with: git clone --recursive https://github.com/ricardoquesada/bluepad32.git components/bluepad32")
endif()

set(ENV{BTSTACK_ROOT} "${BLUEPAD32_ROOT}/external/btstack")

# Point EXTRA_COMPONENT_DIRS at WiimoteAnything and Bluepad32's components
set(EXTRA_COMPONENT_DIRS
    "${WIIMOTE_ROOT}"
    "${BLUEPAD32_ROOT}/src/components"
)

include($ENV{IDF_PATH}/tools/cmake/project.cmake)
include($ENV{BTSTACK_ROOT}/port/esp32/components/btstack/cmake/BTstack_project.cmake)

project(bluepad32_wiimote_adapter)
