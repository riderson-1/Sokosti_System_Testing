/**
 * @file sd_log_backend.cpp
 * @brief Custom Zephyr log backend that mirrors log output to the SD card
 *        as a per-session .log file, in addition to the RTT console.
 *
 * The backend registers a Zephyr logging backend (LOG_BACKEND_DEFINE) whose
 * output function formats each log message exactly like the standard RTT/UART
 * backend (same flags). In immediate mode the formatter delivers the output
 * one byte at a time, so this backend accumulates bytes into a line buffer
 * and enqueues a complete line (terminated by '\n') into a message queue.
 * A dedicated thread (sd_log_writer_thread) drains that queue into the
 * currently open session .log file on the SD card.
 *
 * Because the backend's output function runs in the context of the logging
 * call (immediate mode), it must never block or touch the filesystem. It only
 * appends to a line buffer and enqueues into a bounded queue; if the queue is
 * full the line is dropped (counted) rather than stalling the logger.
 *
 * @version 0.1
 * @date 2026-09-15
 */

#include "sd_log_backend.hpp"

#include <zephyr/logging/log_backend.h>
#include <zephyr/logging/log_core.h>
#include <zephyr/logging/log_output.h>
#include <zephyr/logging/log_backend_std.h>
#include <zephyr/logging/log_ctrl.h>
#include <zephyr/logging/log.h>
#include <zephyr/fs/fs.h>
#include <zephyr/sys/atomic.h>
#include <zephyr/spinlock.h>

#include <string.h>

LOG_MODULE_REGISTER(sd_log, LOG_LEVEL_DBG);

/* ---------------------------------------------------------------------------
 * Message queue carrying formatted log lines to the SD writer thread.
 * ------------------------------------------------------------------------- */
K_MSGQ_DEFINE(sd_log_queue, sizeof(SdLogLine), SD_LOG_QUEUE_SIZE, 4);

/* Instrumentation: lines dropped because the queue was full. */
static atomic_t sd_log_drop_cnt = ATOMIC_INIT(0);

/* ---------------------------------------------------------------------------
 * Session log file state (owned by the SD writer thread).
 * ------------------------------------------------------------------------- */
static struct fs_file_t sd_log_file;
static bool sd_log_file_open = false;

/* ---------------------------------------------------------------------------
 * Line accumulator.
 *
 * The output function is called from arbitrary threads (immediate mode), so
 * the accumulator is protected by a spinlock to avoid interleaved corruption.
 * ------------------------------------------------------------------------- */
static struct k_spinlock sd_log_lock;
static char sd_log_line[SD_LOG_LINE_MAX];
static size_t sd_log_line_len = 0;

static void sd_log_enqueue_line(void)
{
    SdLogLine line;
    size_t n = sd_log_line_len > SD_LOG_LINE_MAX ? SD_LOG_LINE_MAX : sd_log_line_len;
    memcpy(line.data, sd_log_line, n);
    line.len = (uint16_t)n;

    if (k_msgq_put(&sd_log_queue, &line, K_NO_WAIT) != 0) {
        atomic_inc(&sd_log_drop_cnt);
    }
}

/* ---------------------------------------------------------------------------
 * Backend output function.
 *
 * Runs in the context of the logging call (immediate mode). Must be fast and
 * non-blocking: it only appends bytes to the line buffer and enqueues the
 * line once a newline is seen.
 * ------------------------------------------------------------------------- */
static int sd_log_char_out(uint8_t *data, size_t length, void *ctx)
{
    (void)ctx;

    k_spinlock_key_t key = k_spin_lock(&sd_log_lock);

    for (size_t i = 0; i < length; i++) {
        char c = (char)data[i];

        if (sd_log_line_len < SD_LOG_LINE_MAX) {
            sd_log_line[sd_log_line_len++] = c;
        }

        if (c == '\n') {
            sd_log_enqueue_line();
            sd_log_line_len = 0;
        }
    }

    k_spin_unlock(&sd_log_lock, key);

    /* Always report full consumption so the logger does not retry. */
    return (int)length;
}

/* ---------------------------------------------------------------------------
 * Backend API.
 * ------------------------------------------------------------------------- */
/* Output buffer + instance. The buffer is only used transiently by the
 * formatter; the actual data is forwarded to sd_log_char_out. */
static uint8_t sd_log_buf[256];
LOG_OUTPUT_DEFINE(sd_log_output, sd_log_char_out, sd_log_buf, sizeof(sd_log_buf));

static void sd_log_process(const struct log_backend *const backend,
                           union log_msg_generic *msg)
{
    /* Use the same formatting as the standard RTT/UART backend, but strip the
     * ANSI color codes — they render fine on a terminal but show up as garbage
     * (e.g. "[0m") in a plain-text log file on the SD card. */
    uint32_t flags = log_backend_std_get_flags() & ~LOG_OUTPUT_FLAG_COLORS;
    log_format_func_t fmt = log_format_func_t_get(LOG_OUTPUT_TEXT);

    fmt(&sd_log_output, &msg->log, flags);
}

static void sd_log_panic(const struct log_backend *const backend)
{
    /* Nothing to flush in panic mode — the SD writer thread may not be
     * schedulable. Keep the backend simple and non-blocking. */
}

static void sd_log_dropped(const struct log_backend *const backend, uint32_t cnt)
{
    (void)backend;
    (void)cnt;
}

static void sd_log_init(const struct log_backend *const backend)
{
    (void)backend;
    log_output_ctx_set(&sd_log_output, NULL);
}

static int sd_log_is_ready(const struct log_backend *const backend)
{
    (void)backend;
    return 0; /* ready immediately */
}

static int sd_log_format_set(const struct log_backend *const backend,
                             uint32_t log_type)
{
    (void)backend;
    (void)log_type;
    return 0;
}

static const struct log_backend_api sd_log_api = {
    .process     = sd_log_process,
    .dropped     = sd_log_dropped,
    .panic       = sd_log_panic,
    .init        = sd_log_init,
    .is_ready    = sd_log_is_ready,
    .format_set  = sd_log_format_set,
};

/* Register the backend. autostart = false: we enable it explicitly from
 * threads_setup() once the SD card is mounted, so no log lines are lost
 * before the card is ready. */
LOG_BACKEND_DEFINE(sd_log_backend, sd_log_api, false);

/* ---------------------------------------------------------------------------
 * Public API used by the SD writer thread.
 * ------------------------------------------------------------------------- */
void sd_log_backend_enable(void)
{
    const struct log_backend *backend = log_backend_get_by_name("sd_log_backend");
    if (backend == NULL) {
        return;
    }
    log_backend_enable(backend, NULL, LOG_LEVEL_DBG);
}

int sd_log_open(const char *path)
{
    if (sd_log_file_open) {
        return -EALREADY;
    }

    fs_file_t_init(&sd_log_file);
    int ret = fs_open(&sd_log_file, path, FS_O_CREATE | FS_O_WRITE | FS_O_APPEND);
    if (ret != 0) {
        return ret;
    }

    sd_log_file_open = true;
    return 0;
}

void sd_log_close(void)
{
    if (!sd_log_file_open) {
        return;
    }
    sd_log_drain();
    fs_sync(&sd_log_file);
    fs_close(&sd_log_file);
    sd_log_file_open = false;
}

void sd_log_drain(void)
{
    if (!sd_log_file_open) {
        /* No boot log file open yet (e.g. SD card not mounted). Drop the
         * queued lines — they cannot be persisted. */
        SdLogLine line;
        while (k_msgq_get(&sd_log_queue, &line, K_NO_WAIT) == 0) {
        }
        return;
    }

    SdLogLine line;
    while (k_msgq_get(&sd_log_queue, &line, K_NO_WAIT) == 0) {
        fs_write(&sd_log_file, line.data, line.len);
    }
}

void sd_log_sync(void)
{
    if (sd_log_file_open) {
        fs_sync(&sd_log_file);
    }
}