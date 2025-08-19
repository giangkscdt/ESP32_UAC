# The following lines of boilerplate have to be in your project's
# CMakeLists in this exact order for cmake to work correctly
cmake_minimum_required(VERSION 3.5)

# This example uses an extra component for common functions such as Wi-Fi and Ethernet connection.
include($ENV{IDF_PATH}/tools/cmake/project.cmake)
idf_build_set_property(COMPILE_DEFINITIONS "TU_FIFO_MUTEX=1" APPEND)
idf_build_set_property(COMPILE_DEFINITIONS "CFG_TUSB_OS=OPT_OS_FREERTOS" APPEND)

add_compile_options(-Wno-ignored-qualifiers)

project(usb_uac)
