/**
 * @file threads.cpp
 * @brief RTOS thread definitions for ADS1299 EMG acquisition and streaming.
 * @version 0.6
 * @date 2026-07-29
 */

#include "threads.hpp"
#include "ads1299_driver.hpp"
#include "ble/ble_nus.hpp"
#include "usb/usb_cdc.hpp"
#include "bhi360_driver.hpp"
#include "sd_log_backend.hpp"

#include <zephyr/logging/log.h>
#include <zephyr/drivers/pwm.h>
#include <zephyr/sys/atomic.h>
#include <zephyr/fs/fs.h>
#include <zephyr/storage/disk_access.h>

// Safe C wrapper for ELM FatFs definitions in C++
extern "C" {
#include <ff.h>
}

#include <string.h>
#include <stdio.h>
#include <math.h>

/* M_PI is POSIX, not standard C++ — picolibc's strict headers omit it. */
#ifndef M_PI
#define M_PI 3.14159265358979323846f
#endif

LOG_MODULE_REGISTER(threads, LOG_LEVEL_DBG);

/* ---------------------------------------------------------------------------
 * Message queues
 * ------------------------------------------------------------------------- */
K_MSGQ_DEFINE(emg_queue, sizeof(EmgSamplePacket), 1, 4);
K_MSGQ_DEFINE(imu_queue, sizeof(ImuSamplePacket), 8, 8);

// Backup SD logging queues (larger buffers to absorb SD write latencies)
K_MSGQ_DEFINE(emg_sd_queue, sizeof(EmgSamplePacket), 32, 4);
K_MSGQ_DEFINE(imu_sd_queue, sizeof(ImuSamplePacket), 64, 4);

volatile bool measurement_active = true;

// SD record toggle (Button 1): starts OFF — data is measured and streamed
// over BLE/USB from boot, but nothing is written to the SD card until the
// button is pressed.
volatile bool sd_recording = false;

/* ---------------------------------------------------------------------------
 * BLE writer tunables
 * ------------------------------------------------------------------------- */
static constexpr size_t BLE_EMG_GROUP_SAMPLES = 8;
static constexpr size_t MAX_IMU_PACKETS_PER_WAKEUP = 4;

/* Instrumentation: per-stream success/-ENOMEM counts (transmission stage) */
static atomic_t emg_send_ok      = ATOMIC_INIT(0);
static atomic_t emg_send_enomem  = ATOMIC_INIT(0);
static atomic_t imu_send_ok      = ATOMIC_INIT(0);
static atomic_t imu_send_enomem  = ATOMIC_INIT(0);

/* Instrumentation: acquisition stage (ground truth for what hardware
 * actually produced) and queue-loss stage (dropped before ever reaching
 * the transport). These are the counters that let you attribute loss to
 * a specific stage of the pipeline rather than just seeing "fewer samples
 * arrived on the PC than expected". */
static atomic_t emg_samples_collected = ATOMIC_INIT(0); /* raw ADS1299 reads */
static atomic_t emg_batches_produced  = ATOMIC_INIT(0); /* completed 8-sample batches */
static atomic_t emg_live_queue_drops  = ATOMIC_INIT(0); /* batches purged from emg_queue (full) */
static atomic_t emg_sd_queue_drops    = ATOMIC_INIT(0); /* batches dropped from emg_sd_queue (full) */

/* IMU-side counters. These are declared here (and in threads.hpp) so that
 * bhi360_driver.cpp — where imu_queue / imu_sd_queue are actually fed —
 * can increment them at the point of production/drop. threads.cpp cannot
 * count these itself: imu_thread() only calls bhi360_process_fifo(), which
 * is opaque from here. See threads.hpp for the increment points to add in
 * bhi360_driver.cpp. */
atomic_t imu_samples_collected = ATOMIC_INIT(0);
atomic_t imu_live_queue_drops  = ATOMIC_INIT(0);
atomic_t imu_sd_queue_drops    = ATOMIC_INIT(0);

/* ---------------------------------------------------------------------------
 * Thread stacks and control blocks
 * ------------------------------------------------------------------------- */
K_THREAD_STACK_DEFINE(acq_stack, 4096);
K_THREAD_STACK_DEFINE(ble_stack, 4096);
K_THREAD_STACK_DEFINE(led_stack, 4096);
K_THREAD_STACK_DEFINE(log_stack, 4096);
K_THREAD_STACK_DEFINE(imu_stack, 4096);
K_THREAD_STACK_DEFINE(sd_stack, 8192); // Generous 8KB stack for safety
K_THREAD_STACK_DEFINE(sd_log_stack, 4096);

static struct k_thread acq_thread_data;
static struct k_thread ble_thread_data;
static struct k_thread led_thread_data;
static struct k_thread log_thread_data;
static struct k_thread imu_thread_data;
static struct k_thread sd_thread_data;
static struct k_thread sd_log_thread_data;

// ---------------------------------------------------------------------------
// Static File System Allocations (Removes them completely from the stack!)
// ---------------------------------------------------------------------------
static FATFS fat_fs; 
static struct fs_mount_t sd_mount = {
    .type = FS_FATFS,
    .mnt_point = "/SD:",
    .fs_data = &fat_fs,
};

static struct fs_file_t log_file;
static struct fs_file_t test_file;
static struct fs_dirent file_info;

// Button 1 definition (sw0 alias)
static const struct gpio_dt_spec button_1 = GPIO_DT_SPEC_GET(DT_ALIAS(sw0), gpios);
static struct gpio_callback button_cb_data;

void button_pressed_cb(const struct device *dev, struct gpio_callback *cb, uint32_t pins)
{
    sd_recording = !sd_recording;
    if (sd_recording) {
        LOG_INF("");
        LOG_INF("=============================================================");
        LOG_INF("  [USER EVENT] Button 1 Pressed!");
        LOG_INF("  -> SD recording ENABLED. Streams will be written to SD.");
        LOG_INF("=============================================================");
    } else {
        LOG_INF("");
        LOG_INF("=============================================================");
        LOG_INF("  [USER EVENT] Button 1 Pressed!");
        LOG_INF("  -> SD recording DISABLED. Streaming continues without SD.");
        LOG_INF("=============================================================");
    }
}

/* ---------------------------------------------------------------------------
 * Helpers
 * ------------------------------------------------------------------------- */

template <typename Packet>
static uint8_t computeChecksum(const Packet &p)
{
    const uint8_t *bytes = reinterpret_cast<const uint8_t *>(&p);
    uint8_t sum = 0;
    for (size_t i = 0; i < sizeof(Packet) - 1; i++) {
        sum ^= bytes[i];
    }
    return sum;
}

/**
 * Send a raw buffer over the active transport.
 */
static int transport_send(const uint8_t *data, size_t len)
{
    if (usb_cdc::connected()) {
        usb_cdc::print(reinterpret_cast<const char *>(data), len);
        return 0;
    }
    return ble_nus_send_stream(data, static_cast<uint16_t>(len));
}

/* ---------------------------------------------------------------------------
 * Thread entry functions
 * ------------------------------------------------------------------------- */

static void ble_writer_thread(void *, void *, void *)
{
    EmgSamplePacket out_batch;
    ImuSamplePacket imu_packet;
    struct k_poll_event events[2] = {
        K_POLL_EVENT_STATIC_INITIALIZER(K_POLL_TYPE_MSGQ_DATA_AVAILABLE,
                                        K_POLL_MODE_NOTIFY_ONLY, &emg_queue, 0),
        K_POLL_EVENT_STATIC_INITIALIZER(K_POLL_TYPE_MSGQ_DATA_AVAILABLE,
                                        K_POLL_MODE_NOTIFY_ONLY, &imu_queue, 0),
    };

    while (measurement_active || k_msgq_num_used_get(&emg_queue) > 0 || k_msgq_num_used_get(&imu_queue) > 0) {
        int ret = k_poll(events, ARRAY_SIZE(events), K_MSEC(50));
        if (ret != 0) {
            continue;
        }

        if (k_msgq_get(&emg_queue, &out_batch, K_NO_WAIT) == 0) {
            const bool ble_active = !usb_cdc::connected();
            const size_t group_samples =
                ble_active ? BLE_EMG_GROUP_SAMPLES : 1;
            constexpr size_t total_samples = ARRAY_SIZE(out_batch.samples);
            static_assert(total_samples % BLE_EMG_GROUP_SAMPLES == 0,
                          "BATCH_SIZE must be a multiple of "
                          "BLE_EMG_GROUP_SAMPLES");

            for (size_t i = 0; i < total_samples; i += group_samples) {
                const uint8_t *group_ptr = reinterpret_cast<const uint8_t *>(
                    &out_batch.samples[i]);
                const size_t group_len = group_samples * sizeof(SamplePacket);

                int send_ret = transport_send(group_ptr, group_len);
                if (send_ret == 0) {
                    atomic_inc(&emg_send_ok);
                } else if (send_ret == -ENOMEM) {
                    atomic_inc(&emg_send_enomem);
                } else if (send_ret != -ENOTCONN) {
                    LOG_WRN("EMG send dropped: %d", send_ret);
                }
            }
        }

        for (size_t sent = 0; sent < MAX_IMU_PACKETS_PER_WAKEUP; sent++) {
            if (k_msgq_get(&imu_queue, &imu_packet, K_NO_WAIT) != 0) {
                break;
            }

            int send_ret = transport_send(
                reinterpret_cast<const uint8_t *>(&imu_packet),
                sizeof(imu_packet));
            if (send_ret == 0) {
                atomic_inc(&imu_send_ok);
            } else if (send_ret == -ENOMEM) {
                atomic_inc(&imu_send_enomem);
            } else if (send_ret != -ENOTCONN) {
                LOG_WRN("IMU send dropped: %d", send_ret);
            }
        }

        if (k_msgq_num_used_get(&emg_queue) == 0)
            events[0].state = K_POLL_STATE_NOT_READY;
        if (k_msgq_num_used_get(&imu_queue) == 0)
            events[1].state = K_POLL_STATE_NOT_READY;
    }
    LOG_INF("BLE Writer Thread cleanly finished draining and stopped.");
}

static void acquisition_thread(void *, void *, void *)
{
    int ret;
    uint8_t frame[ADS_DAISY_FRAME_BYTES];
    uint32_t sample_idx = 0;

    EmgSamplePacket batch;
    size_t batch_count = 0;

    while (measurement_active) {
        /* Block until the next DRDY pulse. The semaphore wakes us
         * immediately (no polling latency); it is binary, so pulses that
         * arrive while we are processing collapse into it — the backlog
         * and isr_total counters expose exactly how many were lost. */
        ret = k_sem_take(&ADS1299::drdy_sem, K_MSEC(50));
        if (ret != 0 || !measurement_active) {
            continue;
        }
        atomic_dec(&ADS1299::drdy_backlog);

        ret = ads.readFrameRdatac(frame);
        if (ret) {
            LOG_ERR("Frame read failed: %d", ret);
            continue;
        }

        last_ch1_code = ads.decode24(&frame[3]);

        SamplePacket &pkt = batch.samples[batch_count];

        pkt.sync[0]     = 0xAA;
        pkt.sync[1]     = 0x55;
        pkt.sample_idx  = sample_idx;
        pkt.status1_ok  = ((frame[0] & 0xF0) == 0xC0) ? 1 : 0;
        pkt.status2_ok  = ((frame[27] & 0xF0) == 0xC0) ? 1 : 0;

        memcpy(pkt.ch_data,      &frame[3],  24);
        memcpy(pkt.ch_data + 24, &frame[30], 24);

        pkt.checksum = computeChecksum(pkt);

        atomic_inc(&emg_samples_collected);
        batch_count++;

        if (batch_count == 8) {   // BATCH_SIZE
            atomic_inc(&emg_batches_produced);

            /* SD copy: only while sd_recording is toggled on by Button 1.
             * Non-blocking; if the SD queue is ever full the batch is
             * dropped from SD only — never purge or block here. */
            if (sd_recording) {
                if (k_msgq_put(&emg_sd_queue, &batch, K_NO_WAIT) != 0) {
                    atomic_inc(&emg_sd_queue_drops);
                }
            }

            if (k_msgq_put(&emg_queue, &batch, K_NO_WAIT) != 0) {
                /* k_msgq_purge() discards every batch currently queued, not
                 * just one — count them all so the stat reflects actual
                 * batches lost, not just "a purge happened". */
                atomic_add(&emg_live_queue_drops, k_msgq_num_used_get(&emg_queue));
                k_msgq_purge(&emg_queue);
                (void)k_msgq_put(&emg_queue, &batch, K_NO_WAIT);
            }
            batch_count = 0;
        }
        sample_idx++;
    }
    LOG_INF("Acquisition Thread stopped sampling.");
}

static void led_toggling(void *, void *, void *)
{
    static const struct pwm_dt_spec led_blue =
        PWM_DT_SPEC_GET(DT_ALIAS(led0));
    static const struct pwm_dt_spec led_yellow =
        PWM_DT_SPEC_GET(DT_ALIAS(led1));

    if (!pwm_is_ready_dt(&led_blue) || !pwm_is_ready_dt(&led_yellow)) {
        LOG_ERR("Status LEDs not ready");
        return;
    }

    const uint32_t period_ns = led_blue.period;
    const uint32_t max_pulse_ns = period_ns;   /* 100 % duty */
    const uint32_t min_pulse_ns = period_ns / 20; /* ~5 % floor */

    const uint32_t breath_period_ms = 2400; /* full bright->dark cycle */
    const uint32_t tick_ms = 20;            /* LED update tick         */

    uint32_t t_ms = 0;

    while (true) {
        float phase = (float)(t_ms % breath_period_ms) /
                      (float)breath_period_ms;
        float level = 0.5f * (1.0f - cosf(2.0f * (float)M_PI * phase));
        float level_floored = 0.00f + 1.0f * level;

        uint32_t pulse_ns = min_pulse_ns +
            (uint32_t)((float)(max_pulse_ns - min_pulse_ns) * level_floored);

        /* ---- Blue LED: transport state ---- */
        if (usb_cdc::connected()) {
            pwm_set_pulse_dt(&led_blue, pulse_ns);
        } else if (ble_nus_connected()) {
            pwm_set_pulse_dt(&led_blue, max_pulse_ns);  /* solid on */
        } else {
            pwm_set_pulse_dt(&led_blue,
                             ((t_ms / 500) % 2) ? max_pulse_ns : 0U);
        }

        /* ---- Yellow LED: SD recording state ----
         * Blinking = measuring, not saving to SD.
         * Solid on = actively saving to SD card. */
        if (sd_recording) {
            pwm_set_pulse_dt(&led_yellow, max_pulse_ns);  /* solid on */
        } else {
            pwm_set_pulse_dt(&led_yellow,
                             ((t_ms / 500) % 2) ? max_pulse_ns : 0U);
        }

        k_sleep(K_MSEC(tick_ms));
        t_ms += tick_ms;
    }
}

static void live_log(void *, void *, void *)
{
    uint32_t emg_ok_prev = 0, emg_enomem_prev = 0;
    uint32_t imu_ok_prev = 0, imu_enomem_prev = 0;
    uint32_t emg_collected_prev = 0, emg_live_drop_prev = 0, emg_sd_drop_prev = 0;
    uint32_t imu_collected_prev = 0, imu_live_drop_prev = 0, imu_sd_drop_prev = 0;
    uint32_t drdy_isr_prev = 0;

    while (true) {
        k_sleep(K_MSEC(1000));
        // int32_t ch1 = last_ch1_code;
        // LOG_INF("ch1 code = %d  (%.2f uV)", ch1,
        //         (double)ch1 * 5.0 / 8.0 / 8388608.0 * 1e6);

        /* --- Transmission stage: what the transport layer accepted/rejected --- */
        uint32_t emg_ok = atomic_get(&emg_send_ok);
        uint32_t emg_enomem = atomic_get(&emg_send_enomem);
        uint32_t imu_ok = atomic_get(&imu_send_ok);
        uint32_t imu_enomem = atomic_get(&imu_send_enomem);

        LOG_INF("BLE TX/s: EMG ok=%u enomem=%u | IMU ok=%u enomem=%u",
                emg_ok - emg_ok_prev, emg_enomem - emg_enomem_prev,
                imu_ok - imu_ok_prev, imu_enomem - imu_enomem_prev);

        /* --- Acquisition stage: ground truth vs. queue loss, per stream --- */
        uint32_t emg_collected = atomic_get(&emg_samples_collected);
        uint32_t emg_live_drop = atomic_get(&emg_live_queue_drops);
        uint32_t emg_sd_drop   = atomic_get(&emg_sd_queue_drops);
        uint32_t imu_collected = atomic_get(&imu_samples_collected);
        uint32_t imu_live_drop = atomic_get(&imu_live_queue_drops);
        uint32_t imu_sd_drop   = atomic_get(&imu_sd_queue_drops);
        uint32_t drdy_isr      = (uint32_t)atomic_get(&ADS1299::drdy_isr_total);

        LOG_INF("EMG/s: collected=%u live_q_drop=%u sd_q_drop=%u | "
                "drdy_isr=%u drdy_backlog=%lu",
                emg_collected - emg_collected_prev,
                emg_live_drop - emg_live_drop_prev,
                emg_sd_drop - emg_sd_drop_prev,
                drdy_isr - drdy_isr_prev,
                static_cast<unsigned long>(atomic_get(&ADS1299::drdy_backlog)));
        LOG_INF("IMU/s: collected=%u live_q_drop=%u sd_q_drop=%u",
                imu_collected - imu_collected_prev,
                imu_live_drop - imu_live_drop_prev,
                imu_sd_drop - imu_sd_drop_prev);

        emg_ok_prev = emg_ok;
        emg_enomem_prev = emg_enomem;
        imu_ok_prev = imu_ok;
        imu_enomem_prev = imu_enomem;
        emg_collected_prev = emg_collected;
        emg_live_drop_prev = emg_live_drop;
        emg_sd_drop_prev = emg_sd_drop;
        drdy_isr_prev = drdy_isr;
        imu_collected_prev = imu_collected;
        imu_live_drop_prev = imu_live_drop;
        imu_sd_drop_prev = imu_sd_drop;
    }
}

static void imu_thread(void *, void *, void *)
{
    while (measurement_active) {
        bhi360_process_fifo();
        k_msleep(10);
    }
    LOG_INF("IMU Thread stopped data collection.");
}

// ---------------------------------------------------------------------------
// Unified SD Card Writer Thread
// ---------------------------------------------------------------------------
static void sd_writer_thread_entry(void *, void *, void *)
{
    int ret;
    char filepath[40];

    // Initialize globally allocated files (Safe from stack overflows!)
    fs_file_t_init(&log_file);
    fs_file_t_init(&test_file);

    LOG_INF("SD CARD: Initializing card interface...");
    ret = disk_access_init("SD");
    if (ret != 0) {
        LOG_ERR("SD CARD ERROR: Disk interface initialization failed: %d", ret);
        return;
    }

    LOG_INF("SD CARD: Mounting filesystem...");
    ret = fs_mount(&sd_mount);
    if (ret != 0) {
        LOG_ERR("SD CARD ERROR: Mount failed: %d", ret);
        return;
    }
    LOG_INF("SD CARD: Mount successful.");

    // Create logs directory
    ret = fs_stat("/SD:/logs", &file_info);
    if (ret == -ENOENT) {
        ret = fs_mkdir("/SD:/logs");
        if (ret != 0) {
            LOG_ERR("SD CARD: Failed to create '/SD:/logs' subfolder: %d", ret);
            // consider aborting here — without the dir the session_XXXX scan will fail
        } else {
            LOG_INF("SD CARD: Directory '/SD:/logs' created.");
        }
    } else if (ret == 0) {
        LOG_INF("SD CARD: Directory '/SD:/logs' already exists.");
    } else {
        LOG_ERR("SD CARD: stat on '/SD:/logs' failed: %d", ret);
    }

    LOG_INF("SD CARD: Ready. Press Button 1 to start a new recording session.");

    /* ---- Open the always-on boot log file for the log backend ----
     * The filename increments for every boot (boot_0001.log, boot_0002.log,
     * ...). It stays open for the lifetime of the device and captures every
     * log line, including those before any Button-1 press. */
    {
        char logpath[48];
        int boot_idx = 1;
        while (boot_idx < 1000) {
            snprintf(logpath, sizeof(logpath), "/SD:/logs/boot_%04d.log", boot_idx);

            ret = fs_open(&test_file, logpath, FS_O_READ);
            if (ret == 0) {
                fs_close(&test_file);
                boot_idx++;
            } else if (ret == -ENOENT) {
                ret = sd_log_open(logpath);
                if (ret == 0) {
                    LOG_INF("SD LOG: Always-on boot log opened: %s", logpath);
                } else {
                    LOG_ERR("SD LOG: Failed to open boot log %s: %d", logpath, ret);
                }
                break;
            } else {
                LOG_ERR("SD LOG: stat on %s failed: %d", logpath, ret);
                break;
            }
        }
    }

    EmgSamplePacket emg_pkt;
    ImuSamplePacket imu_pkt;
    uint32_t write_counter = 0;
    int file_idx = 1;

    // Session loop: each Button-1 press starts a FRESH session_XXXX.bin;
    // the next press closes it. Streaming via BLE/USB is unaffected either
    // way. The card stays mounted for the lifetime of the device.
    while (true) {
        /* ---- Wait for the recording toggle to go active ---- */
        if (!sd_recording) {
            k_sleep(K_MSEC(10));
            continue;
        }

        /* ---- Start of a new session: pick the next free index ---- */
        bool opened = false;
        while (file_idx < 1000) {
            snprintf(filepath, sizeof(filepath), "/SD:/logs/session_%04d.bin", file_idx);

            // Try opening candidate as Read-Only to verify if it exists
            ret = fs_open(&test_file, filepath, FS_O_READ);
            if (ret == 0) {
                // File exists, close it and try the next index increment
                fs_close(&test_file);
                file_idx++;
            } else if (ret == -ENOENT) {
                // File does NOT exist! Safe to create it.
                ret = fs_open(&log_file, filepath, FS_O_CREATE | FS_O_WRITE | FS_O_APPEND);
                if (ret == 0) {
                    opened = true;
                    LOG_INF("=============================================================");
                    LOG_INF("  [SD SESSION STARTED]");
                    LOG_INF("  -> Created File: %s", filepath);
                    LOG_INF("  -> All data streams will be interleaved inside this file.");
                    LOG_INF("=============================================================");
                    break;
                } else {
                    LOG_ERR("SD CARD ERROR: Failed to create write handle at %s: %d", filepath, ret);
                    break;
                }
            } else {
                LOG_ERR("SD CARD ERROR: Directory check failed at %s: %d", filepath, ret);
                break;
            }
        }

        if (!opened) {
            LOG_ERR("SD CARD ERROR: Unable to allocate a valid log file! "
                    "SD recording disabled until next button press.");
            // Drop whatever accumulated in the queues while idle, then wait
            // for the next toggle-on before retrying with a new index.
            while (k_msgq_get(&emg_sd_queue, &emg_pkt, K_NO_WAIT) == 0) {}
            while (k_msgq_get(&imu_sd_queue, &imu_pkt, K_NO_WAIT) == 0) {}
            while (sd_recording) {
                k_sleep(K_MSEC(10));
            }
            continue;
        }

        /* ---- Active session: drain queues while toggle stays on ---- */
        write_counter = 0;
        while (sd_recording) {
            bool idle = true;

            if (k_msgq_get(&emg_sd_queue, &emg_pkt, K_NO_WAIT) == 0) {
                fs_write(&log_file, &emg_pkt, sizeof(EmgSamplePacket));
                idle = false;
                write_counter++;
            }

            if (k_msgq_get(&imu_sd_queue, &imu_pkt, K_NO_WAIT) == 0) {
                fs_write(&log_file, &imu_pkt, sizeof(ImuSamplePacket));
                idle = false;
                write_counter++;
            }

            if (write_counter >= 50) {
                fs_sync(&log_file);
                write_counter = 0;
            }

            if (idle) {
                k_sleep(K_MSEC(10));
            }
        }

        /* ---- Toggle turned off: flush and close this session ---- */
        // Drain any packets still queued so nothing recorded is lost.
        while (k_msgq_get(&emg_sd_queue, &emg_pkt, K_NO_WAIT) == 0) {
            fs_write(&log_file, &emg_pkt, sizeof(EmgSamplePacket));
            write_counter++;
        }
        while (k_msgq_get(&imu_sd_queue, &imu_pkt, K_NO_WAIT) == 0) {
            fs_write(&log_file, &imu_pkt, sizeof(ImuSamplePacket));
            write_counter++;
        }
        fs_sync(&log_file);

        ret = fs_close(&log_file);
        if (ret == 0) {
            LOG_INF("=============================================================");
            LOG_INF("  [SD SESSION STOPPED]");
            LOG_INF("  -> File closed: %s", filepath);
            LOG_INF("  -> Data flushed. SAFE TO REMOVE CARD.");
            LOG_INF("=============================================================");
        } else {
            LOG_ERR("SD CARD ERROR: Error closing file: %d", ret);
        }
    }
}

/* ---------------------------------------------------------------------------
 * SD Log Writer Thread
 *
 * Drains the sd_log_queue (fed by the custom log backend) into the always-on
 * boot log file. Runs independently of the binary SD writer so that log lines
 * are written continuously, regardless of whether a binary session is active.
 * Periodically syncs so a power cut loses at most the last few seconds.
 * ------------------------------------------------------------------------- */
static void sd_log_writer_thread(void *, void *, void *)
{
    uint32_t ticks = 0;
    while (true) {
        sd_log_drain();
        if (++ticks >= 50) {          /* ~1 s at 20 ms period */
            sd_log_sync();
            ticks = 0;
        }
        k_sleep(K_MSEC(20));
    }
}

/* ---------------------------------------------------------------------------
 * threads_setup — create all five RTOS threads, called once from main()
 * ------------------------------------------------------------------------- */
void threads_setup(void)
{
    // Configure Button 1 interrupt input (sw0 node)
    if (!gpio_is_ready_dt(&button_1)) {
        LOG_ERR("Button 1 GPIO hardware not ready!");
    } else {
        gpio_pin_configure_dt(&button_1, GPIO_INPUT);
        gpio_pin_interrupt_configure_dt(&button_1, GPIO_INT_EDGE_TO_ACTIVE);
        gpio_init_callback(&button_cb_data, button_pressed_cb, BIT(button_1.pin));
        gpio_add_callback(button_1.port, &button_cb_data);
        LOG_INF("Button 1 configured as SD recording toggle");
    }

    k_thread_create(
        &acq_thread_data,
        acq_stack,
        K_THREAD_STACK_SIZEOF(acq_stack),
        acquisition_thread,
        NULL, NULL, NULL,
        1,          // high priority
        0,
        K_NO_WAIT
    );

    k_thread_create(
        &ble_thread_data,
        ble_stack,
        K_THREAD_STACK_SIZEOF(ble_stack),
        ble_writer_thread,
        NULL, NULL, NULL,
        5,
        0,
        K_NO_WAIT
    );

    k_thread_create(
        &led_thread_data,
        led_stack,
        K_THREAD_STACK_SIZEOF(led_stack),
        led_toggling,
        NULL, NULL, NULL,
        10,
        0,
        K_NO_WAIT
    );

    k_thread_create(
        &log_thread_data,
        log_stack,
        K_THREAD_STACK_SIZEOF(log_stack),
        live_log,
        NULL, NULL, NULL,
        10,
        0,
        K_NO_WAIT
    );

    k_thread_create(
        &imu_thread_data,
        imu_stack,
        K_THREAD_STACK_SIZEOF(imu_stack),
        imu_thread,
        NULL, NULL, NULL,
        10,
        0,
        K_NO_WAIT
    );

    // Start SD writer backup thread
    k_thread_create(
        &sd_thread_data,
        sd_stack,
        K_THREAD_STACK_SIZEOF(sd_stack),
        sd_writer_thread_entry,
        NULL, NULL, NULL,
        6, // Slightly lower than BLE stream thread to prioritize transmission
        0,
        K_NO_WAIT
    );

    // Start SD log writer thread (drains the log backend queue to the SD card)
    k_thread_create(
        &sd_log_thread_data,
        sd_log_stack,
        K_THREAD_STACK_SIZEOF(sd_log_stack),
        sd_log_writer_thread,
        NULL, NULL, NULL,
        6,
        0,
        K_NO_WAIT
    );

    // Activate the SD log backend so it starts capturing log lines.
    // (The backend is autostart=false; enabling it here means no lines are
    //  captured before the SD card is mounted.)
    sd_log_backend_enable();
}