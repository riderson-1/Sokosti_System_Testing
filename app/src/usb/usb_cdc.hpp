/**
 * @file usb_cdc.hpp
 * @brief USB CDC-ACM (virtual serial port) bring-up and byte-level output.
 * @version 0.2
 * @date 2026-09-01
 *
 * Wraps USB device stack init, DTR handshake detection, and the poll-out
 * byte writer used to stream binary samples to the host.
 */

#pragma once

#include <cstddef>

namespace usb_cdc {

/**
 * Checks the CDC-ACM UART device is ready and brings up the USB device
 * stack. Does NOT block waiting for a host; use connected() to poll the
 * DTR line state afterwards.
 *
 * @return 0 on success, negative errno on failure.
 */
int init(void);

/**
 * Returns true when a host has opened the virtual COM port (DTR asserted).
 * Safe to call at any time after init().
 */
bool connected(void);

/** Writes raw bytes out over the CDC-ACM UART (blocking, byte-by-byte). */
void print(const char *msg, size_t len);

} // namespace usb_cdc