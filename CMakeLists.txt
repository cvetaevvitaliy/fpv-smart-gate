# CMake fpv-smart-gate
# SPDX-License-Identifier: Apache-2.0
#
# example cmake command for config and build
# cmake -DBOARD=nrf52840dk_nrf52840 ..
# cmake --build cmake-build-debug --target menuconfig
#

cmake_minimum_required(VERSION 3.13.1)
set(QEMU_EXTRA_FLAGS -s)
set(BOARD nrf52840dk_nrf52840)
set(CMAKE_BUILD_TYPE DEBUG)
set(OPTIMIZATION_FLAG -O0)
set(CONF_FILE "prj-mesh.conf")
set(DTC_OVERLAY_FILE "nrf21540dk_nrf52840.overlay")
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

# Default to GNU Arm Embedded toolchain if no toolchain is set
set(ZEPHYR_TOOLCHAIN_VARIANT "gnuarmemb")
find_program(GNU_ARM_GCC arm-none-eabi-gcc)
if(NOT ${GNU_ARM_GCC} STREQUAL GNU_ARM_GCC-NOTFOUND)
    # The toolchain root is located above the path to the compiler.
    get_filename_component(GNUARMEMB_TOOLCHAIN_PATH ${GNU_ARM_GCC}/../.. ABSOLUTE)
endif()

# To ensure final path is absolute and does not contain ../.. in variable.
get_filename_component(APPLICATION_PROJECT_DIR
                       ${CMAKE_CURRENT_LIST_DIR}/../../..
                       ABSOLUTE
)

# Add this project to list of board roots
list(APPEND BOARD_ROOT ${APPLICATION_PROJECT_DIR})

#find_package(Zephyr 2.2.0)
find_package(Zephyr REQUIRED HINTS $ENV{ZEPHYR_BASE})
project(mesh)

target_sources(app PRIVATE src/main.c src/bluetooth.c)

if(NODE_ADDR)
    zephyr_compile_definitions(NODE_ADDR=${NODE_ADDR})
endif()

if(IBEACON_RSSI)
    zephyr_compile_definitions(IBEACON_RSSI=${IBEACON_RSSI})
endif()

# Helper to ignore IDE, do not look for sources in the build folder 
if($ENV{CLION_IDE})
    file(WRITE "${CMAKE_BINARY_DIR}/CMakeLists.txt" "add_custom_target(fake)")
    include("${CMAKE_BINARY_DIR}/CMakeLists.txt")
endif()



