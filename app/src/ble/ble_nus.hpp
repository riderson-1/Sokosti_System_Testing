#pragma once

#include <stdint.h>

/** Initialize Bluetooth and NUS. Does NOT start advertising. */
int ble_nus_init(void);

/** Start connectable advertising (idempotent). */
void ble_nus_start_advertising(void);

/** Stop advertising (idempotent). */
void ble_nus_stop_advertising(void);

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

/** Return true when a BLE central is currently connected. */
bool ble_nus_connected(void);