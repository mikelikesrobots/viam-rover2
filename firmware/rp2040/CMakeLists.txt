cmake_minimum_required(VERSION 3.13)

include(pico_sdk_import.cmake)

project(rover2_rp2040 C CXX ASM)
set(CMAKE_C_STANDARD 11)
set(CMAKE_CXX_STANDARD 17)

pico_sdk_init()

add_executable(rover2_rp2040
    src/main.c
    src/motor.c
    src/encoder.c
    src/protocol.c
    src/telemetry.c
    src/i2c_sensors.c
)

target_include_directories(rover2_rp2040 PRIVATE src)

target_link_libraries(rover2_rp2040
    pico_stdlib
    pico_time
    hardware_pwm
    hardware_gpio
    hardware_uart
    hardware_i2c
)

# We handle UART0 manually for host communication — disable stdio on both
pico_enable_stdio_usb(rover2_rp2040 0)
pico_enable_stdio_uart(rover2_rp2040 0)

# Generate .uf2 output for flashing
pico_add_extra_outputs(rover2_rp2040)
