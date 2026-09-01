/**
 * @file threads.hpp
 * @brief Thread declarations and shared packet structures for Sokosti EMG acquisition.
 * @version 0.1
 * @date 2026-07-29
 */

#pragma once

#include <zephyr/kernel.h>
#include <zephyr/drivers/gpio.h>

#include <stdint.h>

#include "ads1299_definitions.h"

/* ---------------------------------------------------------------------------
 * Forward declarations — objects owned by main.cpp
 * ------------------------------------------------------------------------- */
class ADS1299;

extern ADS1299               ads;
extern int32_t               last_ch1_code;

/* ---------------------------------------------------------------------------
 * Packet structures shared between acquisition and USB-output threads
 * ------------------------------------------------------------------------- */

#pragma pack(push, 1)
struct SamplePacket {
    uint8_t  sync[2];      // 0xAA 0x55 — resync marker for the host parser
    uint32_t sample_idx;
    uint8_t  status1_ok;
    uint8_t  status2_ok;
    uint8_t  ch_data[48];  // 16 channels * 3 bytes raw, unmodified 24-bit values
    uint8_t  checksum;     // simple XOR or sum, so host can detect a corrupted packet
};
#pragma pack(pop)

struct EmgSamplePacket {
    SamplePacket samples[8];  // BATCH_SIZE = 8
};

#define IMU_MAX_DATA_BYTES 10

#pragma pack(push, 1)
struct ImuSamplePacket {
    uint8_t  sync[2];      // 0xBB 0x66 — IMU stream marker
    uint32_t sample_idx;
    uint8_t  sensor_id;    // BHI2 sensor ID
    uint8_t  data_len;
    uint8_t  data[IMU_MAX_DATA_BYTES];
    uint8_t  checksum;
};
#pragma pack(pop)

/* ---------------------------------------------------------------------------
 * Message queue — defined by K_MSGQ_DEFINE in threads.cpp
 * ------------------------------------------------------------------------- */
extern struct k_msgq emg_queue;
extern struct k_msgq imu_queue;

/* ---------------------------------------------------------------------------
 * Thread setup — called once from main() after ADS initialisation
 * ------------------------------------------------------------------------- */
void threads_setup(void);

/** Start the polling thread for the BHI360 devices. */
void imu_thread_setup(void);