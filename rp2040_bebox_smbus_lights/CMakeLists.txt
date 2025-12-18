# Generated Cmake Pico project file

cmake_minimum_required(VERSION 3.13)

set(CMAKE_C_STANDARD 11)
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

# Initialise pico_sdk from installed location
# (note this can come from environment, CMake cache etc)

# == DO NOT EDIT THE FOLLOWING LINES for the Raspberry Pi Pico VS Code Extension to work ==
if(WIN32)
    set(USERHOME $ENV{USERPROFILE})
else()
    set(USERHOME $ENV{HOME})
endif()
set(sdkVersion 2.2.0)
set(toolchainVersion 14_2_Rel1)
set(picotoolVersion 2.2.0-a4)
set(picoVscode ${USERHOME}/.pico-sdk/cmake/pico-vscode.cmake)
if (EXISTS ${picoVscode})
    include(${picoVscode})
endif()
# ====================================================================================
set(PICO_BOARD pico CACHE STRING "Board type")

# Pull in Raspberry Pi Pico SDK (must be before project)
include(pico_sdk_import.cmake)

project(bebox_smbus_lights C CXX ASM)

# Initialise the Raspberry Pi Pico SDK
pico_sdk_init()

add_executable(bebox_smbus_lights)

# Add executable. Default name is the project name, version 0.1
file(MAKE_DIRECTORY ${CMAKE_CURRENT_LIST_DIR}/generated)

# generate the header file into the source tree as it is included in the RP2040 datasheet
pico_generate_pio_header(bebox_smbus_lights ${CMAKE_CURRENT_LIST_DIR}/ws2812.pio OUTPUT_DIR ${CMAKE_CURRENT_LIST_DIR}/generated)

target_sources(bebox_smbus_lights PRIVATE bebox_smbus_lights.c)

pico_set_program_name(bebox_smbus_lights "bebox_smbus_lights")
pico_set_program_version(bebox_smbus_lights "0.1")

# Modify the below lines to enable/disable output over UART/USB
pico_enable_stdio_uart(bebox_smbus_lights 0)
pico_enable_stdio_usb(bebox_smbus_lights 0)

# Add the standard library to the build
target_link_libraries(bebox_smbus_lights
        pico_stdlib pico_stdlib pico_i2c_slave hardware_pio hardware_i2c
    )

# Add the standard include files to the build

pico_add_extra_outputs(bebox_smbus_lights)

