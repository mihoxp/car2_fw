cmake_minimum_required(VERSION 3.13)

# pico_w (RP2040) or pico2_w (RP2350):  cmake -DPICO_BOARD=pico2_w ...
if(NOT PICO_BOARD)
    set(PICO_BOARD pico_w)
endif()
include($ENV{PICO_SDK_PATH}/external/pico_sdk_import.cmake)

project(car2_fw C CXX ASM)
set(CMAKE_C_STANDARD 11)
set(CMAKE_CXX_STANDARD 17)

pico_sdk_init()

if(PICO_BOARD STREQUAL "pico2_w")
    set(MICROROS_DIR ${CMAKE_CURRENT_LIST_DIR}/../libmicroros_rp2350)
else()
    # Rebuilt for RP2040 with 8 subscriptions (the official prebuilt one allows 5)
    set(MICROROS_DIR ${CMAKE_CURRENT_LIST_DIR}/../libmicroros_rp2040)
endif()

add_executable(car2_fw
    main.c
    udp_transport.c
    ultrasonic.c
    ws2812.c
    ssd1306.c
    ir.c
    buzzer.c
    ble_v7rc.c
    servo.c
    sound.c
    gps.c
    sbus.c
)
target_include_directories(car2_fw PRIVATE
    ${CMAKE_CURRENT_LIST_DIR}
    ${MICROROS_DIR}/include
)
target_link_directories(car2_fw PRIVATE ${MICROROS_DIR})
target_link_libraries(car2_fw
    pico_stdlib
    hardware_pwm
    hardware_adc
    hardware_pio
    hardware_i2c
    hardware_uart
    pico_cyw43_arch_lwip_threadsafe_background
    pico_btstack_ble
    pico_btstack_cyw43
    microros
)

pico_generate_pio_header(car2_fw ${CMAKE_CURRENT_LIST_DIR}/ws2812.pio)
pico_btstack_make_gatt_header(car2_fw PRIVATE ${CMAKE_CURRENT_LIST_DIR}/car2_ble.gatt)

target_compile_options(car2_fw PRIVATE -ffunction-sections -fdata-sections)

# USB stdio for debug prints; micro-ROS itself goes over WiFi
pico_enable_stdio_usb(car2_fw 1)
pico_enable_stdio_uart(car2_fw 0)

pico_add_extra_outputs(car2_fw)
