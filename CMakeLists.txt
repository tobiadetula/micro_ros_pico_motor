cmake_minimum_required(VERSION 3.12)

# Pull in Pico SDK (from the submodule)
include($ENV{PICO_SDK_PATH}/external/pico_sdk_import.cmake)

project(ros_pico_motorcontrol C CXX ASM)
set(CMAKE_C_STANDARD 11)
set(CMAKE_CXX_STANDARD 17)

pico_sdk_init()

# Pull in micro-ROS
link_directories(sdk/libmicroros)
add_executable(ros_pico_motorcontrol
    src/main.c
    sdk/pico_uart_transport.c   # required - provides the serial transport
)

# Find the pre-built pico-servo library
find_library(PICO_SERVO_LIB 
    NAMES pico-servo 
    HINTS "${CMAKE_SOURCE_DIR}/libs/pico-servo/build/libpico-servo.a"
)

if(NOT PICO_SERVO_LIB)
    message(FATAL_ERROR "pico-servo library not found. Build it first with 'make build-deps'")
endif()


target_link_libraries(ros_pico_motorcontrol
    pico_stdlib
    microros
    hardware_pwm
    ${PICO_SERVO_LIB}
)

target_include_directories(ros_pico_motorcontrol PUBLIC
    sdk/libmicroros/include
    sdk # for the transport header
    libs/pico-servo/include # for the servo library headers
)

SET(CMAKE_C_FLAGS  "${CMAKE_C_FLAGS} -ffunction-sections -fdata-sections")
SET(CMAKE_CXX_FLAGS  "${CMAKE_CXX_FLAGS} -ffunction-sections -fdata-sections")


# Optional: enable USB serial output for debugging
pico_enable_stdio_usb(ros_pico_motorcontrol 1)
pico_enable_stdio_uart(ros_pico_motorcontrol 0)
add_compile_definitions(PICO_UART_ENABLE_CRLF_SUPPORT=0)
add_compile_definitions(PICO_STDIO_ENABLE_CRLF_SUPPORT=0)
add_compile_definitions(PICO_STDIO_DEFAULT_CRLF=0)

# Generate .uf2, .bin, map etc.
pico_add_extra_outputs(ros_pico_motorcontrol)