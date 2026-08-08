#pragma once

#include <stdint.h>

/** Initialize Bluetooth, NUS, and connectable advertising. */
int ble_nus_init(void);

/**
 * Try to send one NUS notification.
 *
 * This function never waits for controller buffers. A negative return value
 * means that the packet was not accepted and the caller may drop it.
 */
int ble_nus_send(const uint8_t *data, uint16_t len);

/**
 * Send a logical packet as consecutive NUS notifications.
 *
 * The payload is not modified: if the negotiated ATT MTU is smaller than the
 * logical packet, it is split into contiguous chunks. The host can reconstruct
 * the original stream by concatenating notification payloads.
 */
int ble_nus_send_stream(const uint8_t *data, uint16_t len);

/** Return true when a connection has subscribed to NUS TX notifications. */
bool ble_nus_ready(void);