#
# Copyright (c) 2020 Nordic Semiconductor
#
# SPDX-License-Identifier: LicenseRef-Nordic-5-Clause
#

cmake_minimum_required(VERSION 3.20.0)

find_package(Zephyr REQUIRED HINTS $ENV{ZEPHYR_BASE})
project(sokosti_system_integration_test)

# NORDIC SDK APP START
target_sources(app PRIVATE
    app/src/main.cpp
    drivers/bhi360/bhi360_driver.cpp
    app/src/threads.cpp
    app/src/ble/ble_nus.cpp
    app/src/usb/usb_cdc.cpp
    app/src/usb/sample_usbd_init.c
    drivers/ads1299/ads1299_driver.cpp
    drivers/ads1299/ads1299_configs.cpp
)

file(GLOB BHY2_SOURCES
    "${CMAKE_CURRENT_SOURCE_DIR}/drivers/bhi360/BHY2-Sensor-API/*.c"
)

target_sources(app PRIVATE ${BHY2_SOURCES})
target_include_directories(app PRIVATE
    ${CMAKE_CURRENT_SOURCE_DIR}/app/src
    ${CMAKE_CURRENT_SOURCE_DIR}/app/src/ble
    ${CMAKE_CURRENT_SOURCE_DIR}/app/src/usb
    ${CMAKE_CURRENT_SOURCE_DIR}/drivers/ads1299
    ${CMAKE_CURRENT_SOURCE_DIR}/drivers/bhi360
    ${CMAKE_CURRENT_SOURCE_DIR}/drivers/bhi360/BHY2-Sensor-API
)
target_compile_definitions(app PRIVATE
    BHY2_CFG_DELEGATE_FIFO_PARSE_CB_INFO_MGMT=0
)
# NORDIC SDK APP END