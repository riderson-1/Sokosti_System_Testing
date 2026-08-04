/**
 * @file threads.cpp
 * @brief RTOS thread definitions for ADS1299 EMG acquisition and streaming.
 * @version 0.1
 * @date 2026-07-29
 */

#include "threads.hpp"
#include "ads1299_driver.hpp"
#include "usb_cdc.hpp"
#include "bhi360_driver.hpp"

#include <zephyr/logging/log.h>

#include <string.h>

LOG_MODULE_REGISTER(threads, LOG_LEVEL_DBG);

/* ---------------------------------------------------------------------------
 * Message queue - EmgSamplePacket conduit between acquisition and USB writer
 * ------------------------------------------------------------------------- */
K_MSGQ_DEFINE(emg_queue, sizeof(EmgSamplePacket), 4, 4);
K_MSGQ_DEFINE(imu_queue, sizeof(ImuSamplePacket), 8, 8);

/* ---------------------------------------------------------------------------
 * Thread stacks and control blocks
 * ------------------------------------------------------------------------- */
K_THREAD_STACK_DEFINE(acq_stack, 4096);
K_THREAD_STACK_DEFINE(usb_stack, 4096);
K_THREAD_STACK_DEFINE(led_stack, 4096);
K_THREAD_STACK_DEFINE(log_stack, 4096);
K_THREAD_STACK_DEFINE(imu_stack, 4096);

static struct k_thread acq_thread_data;
static struct k_thread usb_thread_data;
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

/* ---------------------------------------------------------------------------
 * Thread entry functions
 * ------------------------------------------------------------------------- */

static void usb_writer_thread(void *, void *, void *)
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

        /* Drain EMG first: it is higher-rate and the queue is shallower. */
        while (k_msgq_get(&emg_queue, &out_batch, K_NO_WAIT) == 0) {
            usb_cdc::print(
                reinterpret_cast<const char *>(out_batch.samples),
                sizeof(out_batch.samples)
            );
        }

        while (k_msgq_get(&imu_queue, &imu_packet, K_NO_WAIT) == 0) {
            usb_cdc::print(reinterpret_cast<const char *>(&imu_packet),
                           sizeof(imu_packet));
        }

        events[0].state = K_POLL_STATE_NOT_READY;
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
            gpio_pin_toggle_dt(&led);
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
            int qret = k_msgq_put(&emg_queue, &batch, K_NO_WAIT);
            if (qret != 0) {
                /* Queue full — USB cannot keep up; silently drop. */
            }
            batch_count = 0;
        }
        sample_idx++;
    }
}

static void led_toggling(void *, void *, void *)
{
    while (true) {
        k_sleep(K_MSEC(500));
        gpio_pin_toggle_dt(&led);
    }
}

static void live_log(void *, void *, void *)
{
    while (true) {
        k_sleep(K_MSEC(500));
        int32_t ch1 = last_ch1_code;
        LOG_INF("ch1 code = %d  (%.2f uV)", ch1,
                (double)ch1 * 5.0 / 8.0 / 8388608.0 * 1e6);
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
        &usb_thread_data,
        usb_stack,
        K_THREAD_STACK_SIZEOF(usb_stack),
        usb_writer_thread,
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