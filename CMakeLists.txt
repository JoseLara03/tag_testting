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
    src/phy_config.c
    src/uwb_ss_initiator.c
    src/motion.c
    src/tag_ui.c
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

# The precompiled Decawave archive registers per-chip drivers into the
# .dw_drivers linker section via static objects. Nothing in app code references
# them directly, so the linker would skip those .o files and the section ends
# up empty. Force-pull dw3000_driver via --undefined; this drags in its .o
# (and the .dw_drivers entries it carries) without --whole-archive.
add_library(dwt_uwb_driver STATIC IMPORTED GLOBAL)
set_target_properties(dwt_uwb_driver PROPERTIES
    IMPORTED_LOCATION
    ${CMAKE_CURRENT_SOURCE_DIR}/Shared/dwt_uwb_driver/lib/libdwt_uwb_driver-m4-sfp-6.0.7.a
)
target_link_libraries(app PRIVATE
    dwt_uwb_driver
)
zephyr_ld_options(-Wl,--undefined=dw3000_driver)

zephyr_linker_sources(SECTIONS ${CMAKE_CURRENT_SOURCE_DIR}/dw_drivers.ld)
