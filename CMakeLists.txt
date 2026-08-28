cmake_minimum_required(VERSION 3.20.0)
find_package(Zephyr REQUIRED HINTS $ENV{ZEPHYR_BASE})

project(tag_testting)

target_sources(app PRIVATE
    src/main.c
    src/ble_log.c
    src/batt.c
    src/batt_curve.c
    src/batt_window.c
    src/lis2hh12_if.c
    src/uwb.c
    src/nfc_tag.c
    src/phy_config.c
    src/cal_math.c
    src/cal.c
    src/storage.c
    src/pos_solver.c
    src/pos_residual.c
    src/pos_ekf.c
    src/pos_cfg.c
    src/blink_cfg.c
    src/blink_frame.c
    # TEMPORARY: raw-range capture for EKF tuning. Delete this line with the
    # module -- see spec/2026-08-22-position-filtering-design.md.
    src/pos_dbg.c
    src/uwb_ss_initiator.c
    src/tag_alert.c
    src/motion.c
    src/tag_ui.c
    src/tag_ui_color.c
    src/uwb_frame_802_15_4z.c
    src/alert_relay.c
    src/tag_alert_core.c
    src/uwb_net.c
    src/uwb_net_runner.c
    src/uwb_radio_owner.c
    src/tag_cmd.c
    src/rx_stats_core.c
    src/rx_stats.c
    src/beacon_track_core.c
    src/beacon_sched_core.c
    src/scan_backoff_core.c
    src/wdt.c
    drivers/lis2hh12-pid/lis2hh12_reg.c
    platform/port.c
    platform/deca_spi.c
    platform/deca_probe_interface.c
    platform/deca_mutex.c
    platform/deca_sleep.c
)

target_sources_ifdef(CONFIG_TAG_CAL_MODE app PRIVATE
    src/cal_diag.c
    src/cal_led.c
    src/cal_run_math.c
    src/cal_run.c
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
