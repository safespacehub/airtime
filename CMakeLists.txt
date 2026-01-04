# SPDX-License-Identifier: Apache-2.0

cmake_minimum_required(VERSION 3.20.0)
find_package(Zephyr REQUIRED HINTS $ENV{ZEPHYR_BASE})
project(flight_timer)

target_sources(app PRIVATE
    src/main.c
    src/supl_assist.c
    src/gps_state.c
    src/baro_state.c
    src/rtc_sync.c
    src/sd_logger.c
    src/session_mgr.c
    src/flight_timer.c
)

target_include_directories(app PRIVATE src)
