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
set(PICO_SERVO_LIB "${CMAKE_SOURCE_DIR}/libs/pico-servo/build/libpico-servo.a")
set(FIXMATH_LIB "${CMAKE_SOURCE_DIR}/libs/pico-servo/build/3rdparty/pico-libfixmath/libfixmath.a")

message(STATUS "CMAKE_SOURCE_DIR: ${CMAKE_SOURCE_DIR}")
if(NOT EXISTS ${PICO_SERVO_LIB})
    message(FATAL_ERROR "pico-servo library not found at ${PICO_SERVO_LIB}. Run 'make build-deps' first.")
endif()
message(STATUS "Found PICO_SERVO_LIB: ${PICO_SERVO_LIB}")

if(NOT EXISTS ${FIXMATH_LIB})
    message(FATAL_ERROR "libfixmath not found. Run 'make build-deps' first.")
endif()
message(STATUS "Found libfixmath")
# find_library(PICO_SERVO_LIB 
#     NAMES pico-servo 
#     HINTS "${CMAKE_SOURCE_DIR}/libs/pico-servo/build"
# )



target_link_libraries(ros_pico_motorcontrol
    pico_stdlib
    microros
    hardware_pwm
    hardware_adc
    hardware_gpio
    ${PICO_SERVO_LIB}
    ${FIXMATH_LIB}
)

target_include_directories(ros_pico_motorcontrol PUBLIC
    sdk/libmicroros/include
    sdk # for the transport header
    libs/pico-servo/include # for the servo library headers
    include/
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