cmake_minimum_required(VERSION 3.16)

# The library root is an ESP-IDF component (its CMakeLists.txt calls idf_component_register).
set(EXTRA_COMPONENT_DIRS "${CMAKE_CURRENT_LIST_DIR}/../../..")

include($ENV{IDF_PATH}/tools/cmake/project.cmake)
project(wiimote_smoke)
