cmake_minimum_required(VERSION 3.16)
set(CMAKE_C_STANDARD 11)
set(CMAKE_CXX_STANDARD 17)

set(WIIMOTE_ROOT "${CMAKE_CURRENT_LIST_DIR}/../../..")

if(NOT DEFINED ENV{BTSTACK_ROOT})
    if(EXISTS "${WIIMOTE_ROOT}/components/bluepad32/external/btstack")
        set(ENV{BTSTACK_ROOT} "${WIIMOTE_ROOT}/components/bluepad32/external/btstack")
    elseif(EXISTS "${WIIMOTE_ROOT}/components/btstack")
        set(ENV{BTSTACK_ROOT} "${WIIMOTE_ROOT}/components/btstack")
    elseif(EXISTS "${CMAKE_CURRENT_LIST_DIR}/components/btstack")
        set(ENV{BTSTACK_ROOT} "${CMAKE_CURRENT_LIST_DIR}/components/btstack")
    endif()
endif()

set(EXTRA_COMPONENT_DIRS
    "${WIIMOTE_ROOT}"
)

include($ENV{IDF_PATH}/tools/cmake/project.cmake)
if(DEFINED ENV{BTSTACK_ROOT})
    include($ENV{BTSTACK_ROOT}/port/esp32/components/btstack/cmake/BTstack_project.cmake)
endif()

project(btstack_wiimote_standalone)
