/**
 * @file bhi360_driver.hpp
 * @brief BHI360 firmware, SPI, FIFO, and callback integration.
 */
#pragma once

#include <stddef.h>

/** Initialize all configured BHI360 devices. */
void bhi360_init(void);

/** Poll and process FIFO data for all initialized BHI360 devices. */
void bhi360_process_fifo(void);
