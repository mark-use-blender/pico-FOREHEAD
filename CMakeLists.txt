cmake_minimum_required(VERSION 3.13)
include(pico_sdk_import.cmake)
project(pico-tail-controller C CXX ASM)
set(CMAKE_C_STANDARD 11)
set(CMAKE_CXX_STANDARD 17)
pico_sdk_init()
add_executable(pico-tail-controller
  main.c
)

pico_add_extra_outputs(pico-tail-controller)
target_link_libraries(pico-tail-controller pico_stdlib hardware_spi hardware_gpio hardware_adc hardware_pwm hardware_uart)