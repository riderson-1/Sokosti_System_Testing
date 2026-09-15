/**
 * @file sd_log_backend.hpp
 * @brief Custom Zephyr log backend that mirrors log output to the SD card
 *        as an always-on boot log file, in addition to the RTT console.
 * @version 0.2
 * @date 2026-09-15
 */

#pragma once

#include <zephyr/kernel.h>

#define SD_LOG_LINE_MAX 192
#define SD_LOG_QUEUE_SIZE 16

struct SdLogLine {
    uint16_t len;
    char data[SD_LOG_LINE_MAX];
};

/* Message queue carrying formatted log lines to the SD writer thread. */
extern struct k_msgq sd_log_queue;

/**
 * @brief Activate the SD log backend so it starts capturing log lines.
 *        Call once after the logging subsystem is up (e.g. from threads_setup).
 */
void sd_log_backend_enable(void);

/**
 * @brief Open the always-on boot log file for the SD log backend.
 * @param path  Absolute path, e.g. "/SD:/logs/boot_0001.log".
 * @return 0 on success, negative errno on failure.
 */
int sd_log_open(const char *path);

/**
 * @brief Flush and close the boot log file.
 */
void sd_log_close(void);

/**
 * @brief Drain queued log lines into the open boot log file.
 *        Called periodically by the SD writer thread.
 */
void sd_log_drain(void);

/**
 * @brief Flush buffered data to the SD card (fs_sync).
 *        Call periodically so a power cut loses at most the last few lines.
 */
void sd_log_sync(void);