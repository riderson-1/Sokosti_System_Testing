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
    src/main.cpp
    src/bhi360_driver.cpp
    src/threads.cpp
    src/usb_cdc.cpp
    src/ads1299_driver.cpp
    src/sample_usbd_init.c
    src/ads1299_configs.cpp
)

file(GLOB BHY2_SOURCES
    "/home/karl/nordic/myApps/BHI-360/src/BHY2-Sensor-API/*.c"
)

target_sources(app PRIVATE ${BHY2_SOURCES})
target_include_directories(app PRIVATE
    /home/karl/nordic/myApps/BHI-360/src
    /home/karl/nordic/myApps/BHI-360/src/BHY2-Sensor-API
)
target_compile_definitions(app PRIVATE
    BHY2_CFG_DELEGATE_FIFO_PARSE_CB_INFO_MGMT=0
)
# NORDIC SDK APP END