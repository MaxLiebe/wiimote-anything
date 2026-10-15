# The component name is the directory name of the library checkout.
get_filename_component(WIIMOTE_ROOT "${CMAKE_CURRENT_LIST_DIR}/../../../.." ABSOLUTE)
get_filename_component(WIIMOTE_COMPONENT "${WIIMOTE_ROOT}" NAME)

idf_component_register(
    SRCS "main.c"
    REQUIRES "${WIIMOTE_COMPONENT}"
)
