/**
 * @file threads.cpp
 * @brief RTOS thread definitions for ADS1299 EMG acquisition and streaming.
 * @version 0.1
 * @date 2026-07-29
 */

#include "threads.hpp"
#include "ads1299_driver.hpp"
#include "ble/ble_nus.hpp"
#include "usb/usb_cdc.hpp"
#include "bhi360_driver.hpp"

#include <zephyr/logging/log.h>
#include <zephyr/drivers/pwm.h>
#include <zephyr/sys/atomic.h>
#include <zephyr/fs/fs.h>
#include <zephyr/storage/disk_access.h>

#include <string.h>
#include <math.h>

/* M_PI is POSIX, not standard C++ — picolibc's strict headers omit it. */
#ifndef M_PI
#define M_PI 3.14159265358979323846f
#endif

LOG_MODULE_REGISTER(threads, LOG_LEVEL_DBG);

/* ---------------------------------------------------------------------------
 * Message queue - EmgSamplePacket conduit between acquisition and USB writer
 * ------------------------------------------------------------------------- */
K_MSGQ_DEFINE(emg_queue, sizeof(EmgSamplePacket), 1, 4);
K_MSGQ_DEFINE(imu_queue, sizeof(ImuSamplePacket), 8, 8);

// SD backup queues (separates streaming pipeline from file I/O pipeline)
K_MSGQ_DEFINE(emg_sd_queue, sizeof(EmgSamplePacket), 8, 4);
K_MSGQ_DEFINE(imu_sd_queue, sizeof(ImuSamplePacket), 16, 4);

volatile bool recording_active = true;

/* ---------------------------------------------------------------------------
 * BLE writer tunables (see ble_streaming_debug_history.md, steps 1 & 2)
 * ------------------------------------------------------------------------- */
static constexpr size_t BLE_EMG_GROUP_SAMPLES = 8;
static constexpr size_t MAX_IMU_PACKETS_PER_WAKEUP = 4;

/* Instrumentation: per-stream success/-ENOMEM counts, drained by live_log() */
static atomic_t emg_send_ok      = ATOMIC_INIT(0);
static atomic_t emg_send_enomem  = ATOMIC_INIT(0);
static atomic_t imu_send_ok      = ATOMIC_INIT(0);
static atomic_t imu_send_enomem  = ATOMIC_INIT(0);

/* ---------------------------------------------------------------------------
 * Thread stacks and control blocks
 * ------------------------------------------------------------------------- */
K_THREAD_STACK_DEFINE(acq_stack, 4096);
K_THREAD_STACK_DEFINE(ble_stack, 4096);
K_THREAD_STACK_DEFINE(led_stack, 4096);
K_THREAD_STACK_DEFINE(log_stack, 4096);
K_THREAD_STACK_DEFINE(imu_stack, 4096);
K_THREAD_STACK_DEFINE(sd_stack, 4096);

static struct k_thread acq_thread_data;
static struct k_thread ble_thread_data;
static struct k_thread led_thread_data;
static struct k_thread log_thread_data;
static struct k_thread imu_thread_data;
static struct k_thread sd_thread_data;

// SD mount configuration
static struct fs_mount_t sd_mount = {
    .type = FS_FATFS,
    .fs_impl_sec_to_date = NULL,
    .mnt_point = "/SD:",
};

// Button 1 definition (sw0 alias)
static const struct gpio_dt_spec button_1 = GPIO_DT_SPEC_GET(DT_ALIAS(sw0), gpios);
static struct gpio_callback button_cb_data;

void button_pressed_cb(const struct device *dev, struct gpio_callback *cb, uint32_t pins)
{
    if (recording_active) {
        recording_active = false;
        LOG_INF("Button 1 pressed! Stopping measurement and safely unmounting SD card...");
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
 * Send a raw buffer over the active transport. USB (when a host has the
 * port open) takes precedence; otherwise BLE NUS is used. Returns 0 on
 * success or the transport error code.
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

    while (true) {
        int ret = k_poll(events, ARRAY_SIZE(events), K_FOREVER);
        if (ret != 0) {
            continue;
        }

        /* Fair interleaving: drain one EMG batch, then drain up to
         * MAX_IMU_PACKETS_PER_WAKEUP IMU packets, ensuring neither stream
         * can monopolize the ACL TX buffers in a single wakeup. */
        if (k_msgq_get(&emg_queue, &out_batch, K_NO_WAIT) == 0) {
            // Backup copy to SD card logging pipeline
            if (recording_active) {
                (void)k_msgq_put(&emg_sd_queue, &out_batch, K_NO_WAIT);
            }

            /* Group samples into one transport_send() call at a time. */
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
            // Backup copy to SD card logging pipeline
            if (recording_active) {
                (void)k_msgq_put(&imu_sd_queue, &imu_packet, K_NO_WAIT);
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

        /* Only clear a wakeup flag if its queue is actually empty. */
        if (k_msgq_num_used_get(&emg_queue) == 0)
            events[0].state = K_POLL_STATE_NOT_READY;
        if (k_msgq_num_used_get(&imu_queue) == 0)
            events[1].state = K_POLL_STATE_NOT_READY;
    }
}

static void acquisition_thread(void *, void *, void *)
{
    int ret;
    uint8_t frame[ADS_DAISY_FRAME_BYTES];
    uint32_t sample_idx = 0;

    EmgSamplePacket batch;
    size_t batch_count = 0;

    while (true) {
        k_sem_take(&ADS1299::drdy_sem, K_FOREVER);

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

        batch_count++;

        if (batch_count == 8) {   // BATCH_SIZE
            /* Non-blocking put with purge-on-full ensures the queue always
             * holds the freshest batch without ever blocking the acquisition
             * thread or missing DRDY interrupts. */
            if (k_msgq_put(&emg_queue, &batch, K_NO_WAIT) != 0) {
                k_msgq_purge(&emg_queue);
                (void)k_msgq_put(&emg_queue, &batch, K_NO_WAIT);
            }
            batch_count = 0;
        }
        sample_idx++;
    }
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
        /* Raised-cosine breathing level, 0..1, never fully dark. */
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

        /* ---- Yellow LED: measurement breathing fade ---- */
        if (recording_active) {
            pwm_set_pulse_dt(&led_yellow, pulse_ns);
        } else {
            pwm_set_pulse_dt(&led_yellow, 0U); // Stop pulsing completely
        }

        k_sleep(K_MSEC(tick_ms));
        t_ms += tick_ms;
    }
}

static void live_log(void *, void *, void *)
{
    uint32_t emg_ok_prev = 0, emg_enomem_prev = 0;
    uint32_t imu_ok_prev = 0, imu_enomem_prev = 0;

    while (true) {
        k_sleep(K_MSEC(500));
        int32_t ch1 = last_ch1_code;
        LOG_INF("ch1 code = %d  (%.2f uV)", ch1,
                (double)ch1 * 5.0 / 8.0 / 8388608.0 * 1e6);

        uint32_t emg_ok = atomic_get(&emg_send_ok);
        uint32_t emg_enomem = atomic_get(&emg_send_enomem);
        uint32_t imu_ok = atomic_get(&imu_send_ok);
        uint32_t imu_enomem = atomic_get(&imu_send_enomem);

        LOG_INF("BLE TX/500ms: EMG ok=%u enomem=%u | IMU ok=%u enomem=%u",
                emg_ok - emg_ok_prev, emg_enomem - emg_enomem_prev,
                imu_ok - imu_ok_prev, imu_enomem - imu_enomem_prev);

        emg_ok_prev = emg_ok;
        emg_enomem_prev = emg_enomem;
        imu_ok_prev = imu_ok;
        imu_enomem_prev = imu_enomem;
    }
}

static void imu_thread(void *, void *, void *)
{
    while (true) {
        bhi360_process_fifo();
        k_msleep(10);
    }
}

// ---------------------------------------------------------------------------
// SD Card Writer Thread
// ---------------------------------------------------------------------------
static void sd_writer_thread_entry(void *, void *, void *)
{
    int ret;
    struct fs_file_t emg_file;
    struct fs_file_t imu_file;

    fs_file_t_init(&emg_file);
    fs_file_t_init(&imu_file);

    // Initialize SD Disk Interface
    if (disk_access_init("SD") != 0) {
        LOG_ERR("SD card disk interface registration failed!");
        return;
    }

    // Mount FATFS
    ret = fs_mount(&sd_mount);
    if (ret != 0) {
        LOG_ERR("Failed to mount SD card filesystem: %d", ret);
        return;
    }
    LOG_INF("SD Card filesystem mounted successfully.");

    // Open logs in raw binary format append mode
    ret = fs_open(&emg_file, "/SD:/emg_log.bin", FS_O_CREATE | FS_O_WRITE | FS_O_APPEND);
    if (ret < 0) {
        LOG_ERR("Failed to open /SD:/emg_log.bin: %d", ret);
    }
    ret = fs_open(&imu_file, "/SD:/imu_log.bin", FS_O_CREATE | FS_O_WRITE | FS_O_APPEND);
    if (ret < 0) {
        LOG_ERR("Failed to open /SD:/imu_log.bin: %d", ret);
    }

    EmgSamplePacket emg_pkt;
    ImuSamplePacket imu_pkt;
    uint32_t write_counter = 0;

    while (recording_active) {
        bool idle = true;

        if (k_msgq_get(&emg_sd_queue, &emg_pkt, K_NO_WAIT) == 0) {
            fs_write(&emg_file, &emg_pkt, sizeof(EmgSamplePacket));
            idle = false;
            write_counter++;
        }

        if (k_msgq_get(&imu_sd_queue, &imu_pkt, K_NO_WAIT) == 0) {
            fs_write(&imu_file, &imu_pkt, sizeof(ImuSamplePacket));
            idle = false;
            write_counter++;
        }

        // Periodic flush to prevent loss of data blocks in case of sudden power cut
        if (write_counter >= 50) {
            fs_sync(&emg_file);
            fs_sync(&imu_file);
            write_counter = 0;
        }

        if (idle) {
            k_sleep(K_MSEC(10)); // Rest when queue is dry to allow other lower threads to execute
        }
    }

    // --- SHUTDOWN DRAIN (Safely write anything remaining in queues) ---
    LOG_INF("Draining remaining logs to SD card before safe eject...");
    while (k_msgq_get(&emg_sd_queue, &emg_pkt, K_NO_WAIT) == 0) {
        fs_write(&emg_file, &emg_pkt, sizeof(EmgSamplePacket));
    }
    while (k_msgq_get(&imu_sd_queue, &imu_pkt, K_NO_WAIT) == 0) {
        fs_write(&imu_file, &imu_pkt, sizeof(ImuSamplePacket));
    }

    fs_close(&emg_file);
    fs_close(&imu_file);

    fs_unmount(&sd_mount);
    LOG_INF("SD Card filesystem successfully unmounted. It is now safe to disconnect.");
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
        LOG_INF("Button 1 configured as Safe Eject Shutdown trigger");
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
}
