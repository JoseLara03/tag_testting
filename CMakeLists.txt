cmake_minimum_required(VERSION 3.20.0)
find_package(Zephyr REQUIRED HINTS $ENV{ZEPHYR_BASE})

project(tag_testting)

target_sources(app PRIVATE
    src/main.c
    drivers/lis2hh12-pid/lis2hh12_reg.c
)

target_include_directories(app PRIVATE
    drivers/lis2hh12-pid
)
