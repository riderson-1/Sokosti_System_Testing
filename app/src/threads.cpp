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

/* ---------------------------------------------------------------------------
 * BLE writer tunables (see ble_streaming_debug_history.md, steps 1 & 2)
 *
 * BLE_EMG_GROUP_SAMPLES: samples concatenated into one transport_send() call
 * when BLE is the active transport. At BATCH_SIZE=8 and 500 sps, a group of
 * 8 (456 B, under the 498 B MTU) yields ~62.5 notifications/s, well under the
 * observed ~128 notifications/s link ceiling at the 7.5 ms / latency-0
 * connection parameters. USB has no per-notification overhead, so it keeps
 * sending one SamplePacket (57 B) at a time regardless of this constant.
 *
 * MAX_IMU_PACKETS_PER_WAKEUP: caps how many queued IMU packets are drained
 * in a single writer-thread wakeup. Without this cap, a backlog in
 * imu_queue (depth 8) can be flushed in one burst, consuming most/all of
 * CONFIG_BT_BUF_ACL_TX_COUNT ACL TX buffers before EMG gets a turn.
 * ------------------------------------------------------------------------- */
static constexpr size_t BLE_EMG_GROUP_SAMPLES = 8;
static constexpr size_t MAX_IMU_PACKETS_PER_WAKEUP = 4;

/* Instrumentation: per-stream success/-ENOMEM counts, drained by live_log()
 * so scheduling share between EMG and IMU can be verified against the
 * hypothesis in ble_streaming_debug_history.md §3a rather than assumed. */
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

static struct k_thread acq_thread_data;
static struct k_thread ble_thread_data;
static struct k_thread led_thread_data;
static struct k_thread log_thread_data;
static struct k_thread imu_thread_data;

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
            /* Group samples into one transport_send() call at a time.
             * USB: group size 1 (unchanged, per-sample, no overhead to save).
             * BLE: group size BLE_EMG_GROUP_SAMPLES, cutting notification
             * count roughly 4x so EMG demand fits the link's ~128 notif/s
             * ceiling instead of needing 500/s. */
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
    /*
     * Status LED thread (hardware PWM).
     *
     * LED0 (blue) — transport status:
     *   - slow blink  : BLE advertising, no connection
     *   - solid on    : BLE connected
     *   - breathing   : USB host connected (CDC DTR asserted)
     *
     * LED1 (yellow) — measurement status:
     *   - breathing PWM fade (never fully off) while the ADS1299
     *     acquisition thread and the IMU thread are running.
     */
    static const struct pwm_dt_spec led_blue =
        PWM_DT_SPEC_GET(DT_ALIAS(led0));
    static const struct pwm_dt_spec led_yellow =
        PWM_DT_SPEC_GET(DT_ALIAS(led1));

    if (!pwm_is_ready_dt(&led_blue) || !pwm_is_ready_dt(&led_yellow)) {
        LOG_ERR("Status LEDs not ready");
        return;
    }

    /* PWM period from the devicetree node (10 ms -> 100 Hz). */
    const uint32_t period_ns = led_blue.period;
    const uint32_t max_pulse_ns = period_ns;   /* 100 % duty */
    const uint32_t min_pulse_ns = period_ns / 20; /* ~5 % floor */

    /*
     * Breathing parameters. The fade uses a raised-cosine (half-wave
     * rectified sine) profile: smooth at both ends, never fully dark.
     *   level = (1 - cos(pi * phase)) / 2   in [0, 1]
     */
    const uint32_t breath_period_ms = 2400; /* full bright->dark cycle */
    const uint32_t tick_ms = 20;            /* LED update tick         */

    uint32_t t_ms = 0;

    while (true) {
        /* Raised-cosine breathing level, 0..1, never fully dark. */
        float phase = (float)(t_ms % breath_period_ms) /
                      (float)breath_period_ms;
        float level = 0.5f * (1.0f - cosf(2.0f * (float)M_PI * phase));
        /* Floor so the LED never goes fully off. */
        float level_floored = 0.00f + 1.0f * level;

        uint32_t pulse_ns = min_pulse_ns +
            (uint32_t)((float)(max_pulse_ns - min_pulse_ns) * level_floored);

        /* ---- Blue LED: transport state ---- */
        if (usb_cdc::connected()) {
            /* Breathing pulse. */
            pwm_set_pulse_dt(&led_blue, pulse_ns);
        } else if (ble_nus_connected()) {
            pwm_set_pulse_dt(&led_blue, max_pulse_ns);  /* solid on */
        } else {
            /* Slow blink: advertising. 500 ms on / 500 ms off. */
            pwm_set_pulse_dt(&led_blue,
                             ((t_ms / 500) % 2) ? max_pulse_ns : 0U);
        }

        /* ---- Yellow LED: measurement breathing fade ---- */
        pwm_set_pulse_dt(&led_yellow, pulse_ns);

        k_sleep(K_MSEC(tick_ms));
        t_ms += tick_ms;
    }
}

static void live_log(void *, void *, void *)
{
    /* Snapshot of the atomic counters at the previous print, so we report
     * a per-500ms delta rather than a lifetime total. */
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

/* ---------------------------------------------------------------------------
 * threads_setup — create all four RTOS threads, called once from main()
 * ------------------------------------------------------------------------- */
void threads_setup(void)
{
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
}