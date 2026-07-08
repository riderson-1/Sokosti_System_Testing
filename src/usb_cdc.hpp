/**
 * @file usb_cdc.hpp
 * @brief USB CDC-ACM (virtual serial port) bring-up and byte-level output.
 * @version 0.1
 * @date 2026-07-08
 *
 * Extracted from main.c: wraps USB device stack init, DTR handshake, and
 * the poll-out byte writer used to stream CSV samples to the host.
 */

#pragma once

#include <cstddef>

namespace usb_cdc {

/**
 * Checks the CDC-ACM UART device is ready, brings up the USB device stack,
 * and blocks (up to 500 ms) waiting for the host to assert DTR — i.e. for
 * a terminal/serial monitor to open the port.
 *
 * @return 0 on success, negative errno on failure.
 */
int init(void);

/** Writes raw bytes out over the CDC-ACM UART (blocking, byte-by-byte). */
void print(const char *msg, size_t len);

} // namespace usb_cdc