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
    src/threads.cpp
    src/usb_cdc.cpp
    src/ads1299_driver.cpp
    src/sample_usbd_init.c
    src/ads1299_configs.cpp
)
# NORDIC SDK APP END