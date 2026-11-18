cmake_minimum_required(VERSION 3.20.0)

# Make custom boards discoverable (workspace-level boards/ folder)
list(APPEND BOARD_ROOT
    ${CMAKE_CURRENT_SOURCE_DIR}/..
)

# Use default overlay unless caller provides a different one
if(NOT DTC_OVERLAY_FILE)
    set(DTC_OVERLAY_FILE "${CMAKE_CURRENT_SOURCE_DIR}/boards/Traumschreiber2603.overlay")
endif()

find_package(Zephyr REQUIRED HINTS $ENV{ZEPHYR_BASE})

project(traumschreiber2603)

# Add include directories
target_include_directories(app PRIVATE 
    include 
    include/hw_abstraction
    include/drivers
    include/subsystems
    src 
    src/drivers 
    src/subsystems 
    src/hw_abstraction 
    src/ble
)

# Add source files
target_sources(app PRIVATE 
    src/main.c
    src/debug_ads131_osr.c
    src/hw_abstraction/rgb_led.c
    src/hw_abstraction/button.c
    src/subsystems/stimulator.c
    src/subsystems/data_logger.c
    src/subsystems/audio_player.c
    src/subsystems/battery_monitor.c
    src/subsystems/wav_writer.c
    src/drivers/i2s_out.c
    src/drivers/sd_card_manager.c
    src/drivers/ads131m08_ti.c
    src/drivers/nordic_adc.c
    src/ble/ble_manager.c
    src/ble/lbs_service.c
    src/ble/bas_service.c
    src/ble/ads_service.c
    src/ble/ain_service.c
    src/subsystems/system_config.c
    src/subsystems/noise_test.c
    src/subsystems/led_animation.c
    src/subsystems/charge_guard.c
    src/selftest.c
)

# Optional hardware self-test image (compares the TS2607-changed parts vs TS2603).
# Enable with:  west build ... -- -DTS_HW_SELFTEST=ON
# HW-Selbsttest-Build. Der Sysbuild-Image-Var (-DFirmware_TS_HW_SELFTEST=ON) greift
# in diesem App-CMake NICHT zuverlaessig -> zuverlaessiger Weg ist die ENV-Variable:
#   TS_HW_SELFTEST=1 west build --pristine -b Traumschreiber2603/nrf52840 -- \
#     -DBOARD_ROOT="$PWD/.." -DDTC_OVERLAY_FILE=boards/Traumschreiber2603.overlay
# Produktions-Builds (ohne die ENV-Var) enthalten den Testcode NICHT.
if(TS_HW_SELFTEST OR DEFINED ENV{TS_HW_SELFTEST})
    target_compile_definitions(app PRIVATE TS_HW_SELFTEST)
    message(STATUS "TS_HW_SELFTEST enabled: building hardware self-test firmware")
endif()
