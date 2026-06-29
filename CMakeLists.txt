cmake_minimum_required(VERSION 3.13)

# Initialize pico-sdk from fetchcontent
include(FetchContent)
FetchContent_Declare(
  pico_sdk
  GIT_REPOSITORY https://github.com/raspberrypi/pico-sdk
  GIT_TAG master
)
FetchContent_MakeAvailable(pico_sdk)

# Initialize the SDK
include(pico_sdk_import.cmake)

project(FFBSteeringWheel C CXX ASM)
set(CMAKE_C_STANDARD 11)
set(CMAKE_CXX_STANDARD 20)

pico_sdk_init()

add_executable(ffb_steering_wheel
    main.cpp
    usb_descriptors.cpp
    usb_hid.cpp
    button_reader.cpp
    pedal_reader.cpp
    led_controller.cpp
    core1_entry.cpp
    as5600_parser.cpp
    ffb_processor.cpp
    motor_control.cpp
    i2c_dma.cpp
    calibration.cpp
    flash_storage.cpp
    debug_serial.cpp
)

# Pull in our pico_stdlib which aggregates commonly used features
target_link_libraries(ffb_steering_wheel
    pico_stdlib
    pico_multicore
    hardware_pwm
    hardware_i2c
    hardware_spi
    hardware_dma
    hardware_adc
    hardware_flash
    hardware_sync
    hardware_timer
    tinyusb_device
    tinyusb_board
)

# Create map/bin/hex/uf2 files
set_target_properties(ffb_steering_wheel PROPERTIES SUFFIX ".elf")
pico_add_extra_outputs(ffb_steering_wheel)

target_include_directories(ffb_steering_wheel PRIVATE ${CMAKE_CURRENT_SOURCE_DIR})

# Enable USB, disable UART
pico_enable_stdio_usb(ffb_steering_wheel 0)
pico_enable_stdio_uart(ffb_steering_wheel 1)
