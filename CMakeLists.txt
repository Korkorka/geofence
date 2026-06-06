# Generated Cmake Pico project file

cmake_minimum_required(VERSION 3.13)
# set(PICO_SDK_PATH "C:/Program Files/Raspberry Pi/Pico SDK v1.5.1")
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
set(PICO_BOARD pico_w CACHE STRING "Board type")

# Pull in Raspberry Pi Pico SDK (must be before project)
include(pico_sdk_import.cmake)

project(blink C CXX ASM)

# Initialise the Raspberry Pi Pico SDK
pico_sdk_init()

# Add executable. Default name is the project name, version 0.1
add_executable(blink blink.c)

# *** IMPROVEMENT: Use a relative path for your include directory ***
# This makes your project portable to other computers or locations.
# Assumes your mavlink headers are in an 'include' folder inside your project.
target_include_directories(blink PRIVATE ${PROJECT_SOURCE_DIR}/include)


# pull in common dependencies
target_link_libraries(blink 
    pico_stdlib
    hardware_watchdog
    pico_multicore
    hardware_timer
    hardware_gpio
    hardware_irq
    hardware_uart
    m
    # pico_standard_math  # *** ESSENTIAL: Links the floating-point math library ***
)

# This is correct for using the Pico W onboard LED without WiFi.
if (PICO_CYW43_SUPPORTED)
    target_link_libraries(blink pico_cyw43_arch_none)
endif()

pico_enable_stdio_usb(blink 1)
pico_enable_stdio_uart(blink 0)

# create map/bin/hex file etc.
pico_add_extra_outputs(blink)

# add url via pico_set_program_url