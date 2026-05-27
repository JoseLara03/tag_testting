cmake_minimum_required(VERSION 3.20.0)
find_package(Zephyr REQUIRED HINTS $ENV{ZEPHYR_BASE})

project(tag_testting)

target_sources(app PRIVATE
    src/main.c
    src/ble_log.c
    src/batt.c
    src/lis2hh12_if.c
    src/uwb.c
    src/nfc_tag.c
    drivers/lis2hh12-pid/lis2hh12_reg.c
    platform/port.c
    platform/deca_spi.c
    platform/deca_probe_interface.c
    platform/deca_mutex.c
    platform/deca_sleep.c
)

target_include_directories(app PRIVATE
    drivers/lis2hh12-pid
    platform
    Shared/dwt_uwb_driver/Inc
)

target_link_libraries(app PRIVATE
    ${CMAKE_CURRENT_SOURCE_DIR}/Shared/dwt_uwb_driver/lib/libdwt_uwb_driver-m4-sfp-6.0.7.a
)

zephyr_linker_sources(SECTIONS dw_drivers.ld)
