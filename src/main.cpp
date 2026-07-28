/**
 * @file main.cpp
 * @brief ADS1299 EMG acquisition entry point, streaming CSV over USB CDC-ACM.
 * @version 0.2
 * @date 2026-07-08
 */

#include <zephyr/kernel.h>
#include <zephyr/device.h>
#include <zephyr/drivers/spi.h>
#include <zephyr/drivers/gpio.h>
#include <zephyr/drivers/pwm.h>
#include <zephyr/devicetree.h>
#include <zephyr/logging/log.h>

#include <errno.h>
#include <stdint.h>
#include <stddef.h>
#include <stdio.h>

#include "ads1299_definitions.h"
#include "ads1299_driver.hpp"
#include "usb_cdc.hpp"
#include "ads1299_configs.hpp"

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

#define ADS_NODE        DT_NODELABEL(ads1299)
#define LED_NODE        DT_ALIAS(led0)
#define RESET_NODE      DT_NODELABEL(reset_ads)
#define START_NODE      DT_NODELABEL(global_start)
#define DRDY_NODE       DT_NODELABEL(drdy_ads)
#define ADS_CLK_NODE    DT_ALIAS(adsclk)

/*
 * ADS1299 SPI timing is CPOL = 0, CPHA = 1 according to the datasheet [14].
 */
static const struct spi_dt_spec ads_spi =
    SPI_DT_SPEC_GET(ADS_NODE,
            SPI_WORD_SET(8) | SPI_TRANSFER_MSB | SPI_MODE_CPHA);

static const struct gpio_dt_spec led =
    GPIO_DT_SPEC_GET(LED_NODE, gpios);

static const struct gpio_dt_spec reset_pin =
    GPIO_DT_SPEC_GET(RESET_NODE, gpios);

static const struct gpio_dt_spec start_pin =
    GPIO_DT_SPEC_GET(START_NODE, gpios);

static const struct gpio_dt_spec drdy_pin =
    GPIO_DT_SPEC_GET(DRDY_NODE, gpios);

static const struct pwm_dt_spec ads_clk_pwm =
    PWM_DT_SPEC_GET(ADS_CLK_NODE);

/* Single ADS1299 instance for this board, bound to its SPI bus + RESET pin. */
static ADS1299 ads(ads_spi, reset_pin);

static K_SEM_DEFINE(drdy_sem, 0, 1);
static struct gpio_callback drdy_cb_data;

constexpr size_t BATCH_SIZE = 8;

LOG_MODULE_REGISTER(main, LOG_LEVEL_DBG);

struct SampleBatch {
    SamplePacket samples[BATCH_SIZE];
};

K_MSGQ_DEFINE(batch_queue, sizeof(SampleBatch), 4, 4);

K_THREAD_STACK_DEFINE(acq_stack, 4096);
K_THREAD_STACK_DEFINE(usb_stack, 4096);
K_THREAD_STACK_DEFINE(led_stack, 4096);
K_THREAD_STACK_DEFINE(log_stack, 4096);

static struct k_thread acq_thread_data;
static struct k_thread usb_thread_data;
static struct k_thread led_thread_data;
static struct k_thread log_thread_data;

static int32_t last_ch1_code = 0;

static uint8_t computeChecksum(const SamplePacket &p) {
    const uint8_t *bytes = reinterpret_cast<const uint8_t *>(&p);
    uint8_t sum = 0;
    for (size_t i = 0; i < sizeof(SamplePacket) - 1; i++) sum ^= bytes[i]; // exclude checksum field itself
    return sum;
}

static void drdy_isr(const struct device *dev, struct gpio_callback *cb, uint32_t pins)
{
    k_sem_give(&drdy_sem);
}

static int setup_drdy_interrupt(void)
{
    int ret = gpio_pin_interrupt_configure_dt(&drdy_pin, GPIO_INT_EDGE_FALLING);
    if (ret) {
        return ret;
    }

    gpio_init_callback(&drdy_cb_data, drdy_isr, BIT(drdy_pin.pin));
    gpio_add_callback(drdy_pin.port, &drdy_cb_data);

    return 0;
}

static int start_ads_pwm_clock(void)
{
    if (!pwm_is_ready_dt(&ads_clk_pwm)) {
        LOG_ERR("ADS clock PWM not ready");
        return -ENODEV;
    }

    /*
     * ~2 MHz: 500 ns period, 250 ns pulse.
     * ADS1299 typical external fCLK is 2.048 MHz in datasheet examples [14].
     */
    int ret = pwm_set_dt(&ads_clk_pwm, PWM_NSEC(500), PWM_NSEC(250));
    if (ret) {
        LOG_ERR("PWM start failed: %d", ret);
        return ret;
    }

    LOG_INF("ADS clock PWM started");
    return 0;
}

static int setup_gpios(void)
{
    int ret;

    if (!gpio_is_ready_dt(&led) ||
        !gpio_is_ready_dt(&reset_pin) ||
        !gpio_is_ready_dt(&start_pin) ||
        !gpio_is_ready_dt(&drdy_pin)) {
        LOG_ERR("GPIO not ready");
        return -ENODEV;
    }

    ret = gpio_pin_configure_dt(&led, GPIO_OUTPUT_INACTIVE);
    if (ret) {
        return ret;
    }

    ret = gpio_pin_configure_dt(&reset_pin, GPIO_OUTPUT_INACTIVE);
    if (ret) {
        return ret;
    }

    ret = gpio_pin_configure_dt(&start_pin, GPIO_OUTPUT_INACTIVE);
    if (ret) {
        return ret;
    }

    ret = gpio_pin_configure_dt(&drdy_pin, GPIO_INPUT);
    if (ret) {
        return ret;
    }

    return 0;
}

static int setup_ads(AdsPreset preset)
{
    ADS1299Settings cfg = makeAdsSettings(preset);
    
    int ret = ads.configure(cfg);
    if (ret) {
        LOG_ERR("ADS configuration failed: %d", ret);
    }
    return ret;
}

static void usb_writer_thread(void *arg1, void *arg2, void *arg3)
{
    SampleBatch out_batch;

    while (true) {
        int ret = k_msgq_get(&batch_queue, &out_batch, K_FOREVER);
        if (ret == 0) {
            usb_cdc::print(
                reinterpret_cast<const char *>(out_batch.samples),
                sizeof(out_batch.samples)
            );
        }
    }
}

static void acquisition_thread(void *arg1, void *arg2, void *arg3)
{
    int ret;
    uint8_t frame[ADS_DAISY_FRAME_BYTES];
    uint32_t sample_idx = 0;

    SampleBatch batch;
    size_t batch_count = 0;

    while (true) {
        k_sem_take(&drdy_sem, K_FOREVER);

        ret = ads.readFrameRdatac(frame);
        if (ret) {
            LOG_ERR("Frame read failed: %d", ret);
            gpio_pin_toggle_dt(&led);
            continue;
        }

        last_ch1_code = ads.decode24(&frame[3]);  // ch1, device 1 for debugging

        SamplePacket &pkt = batch.samples[batch_count];

        pkt.sync[0] = 0xAA;      // ← FIXED
        pkt.sync[1] = 0x55;      // ← FIXED
        pkt.sample_idx  = sample_idx;
        pkt.status1_ok  = ((frame[0] & 0xF0) == 0xC0) ? 1 : 0;   // ← FIXED
        pkt.status2_ok  = ((frame[27] & 0xF0) == 0xC0) ? 1 : 0;  // ← FIXED

        memcpy(pkt.ch_data,      &frame[3],  24);   // ← FIXED: device 1
        memcpy(pkt.ch_data + 24, &frame[30], 24);   // ← FIXED: device 2

        pkt.checksum = computeChecksum(pkt);

        batch_count++;

        if (batch_count == BATCH_SIZE) {
            int qret = k_msgq_put(&batch_queue, &batch, K_NO_WAIT);

            if (qret != 0) {
                /*
                 * Queue full. USB cannot keep up.
                 * Do not block acquisition. Count/drop/report later.
                 */
                // dropped_batches++;
            }

            batch_count = 0;
        }
        sample_idx++;
    }
}

static void led_toggling(void *arg1, void *arg2, void *arg3)
{
    while (true) {
        k_sleep(K_MSEC(500));
        gpio_pin_toggle_dt(&led);
    }
}

static void live_log(void *arg1, void *arg2, void *arg3)
{
    while (true) {
        k_sleep(K_MSEC(500));
        int32_t ch1 = last_ch1_code;  // atomic read on Cortex-M33
        LOG_INF("ch1 code = %d  (%.2f uV)", ch1,
                (double)ch1 * 5.0 / 8.0 / 8388608.0 * 1e6);
    }
}

int main(void)
{
    int ret;
    uint8_t id = 0;

    // ---------------------------------------
    // board setup
    // ---------------------------------------

    ret = usb_cdc::init();
    if (ret != 0) {
        LOG_ERR("USB CDC init failed: %d", ret);
        return 0;
    }

    k_msleep(100);
    k_sleep(K_MSEC(500));
    LOG_INF("ADS1299 internal test signal capture starting...");

    if (!spi_is_ready_dt(&ads_spi)) {
        LOG_ERR("SPI device not ready");
        return 0;
    }

    ret = setup_gpios();
    if (ret) {
        LOG_ERR("GPIO setup failed: %d", ret);
        return 0;
    }

    ret = setup_drdy_interrupt();
    if (ret) {
        LOG_ERR("DRDY interrupt setup failed: %d", ret);
        return 0;
    }

    /*
     * Keep START pin low and use the SPI START command.
     * Datasheet says when using START command, hold START pin low [9], [27].
     */
    gpio_pin_set_dt(&start_pin, 0);

    ret = start_ads_pwm_clock();
    if (ret) {
        LOG_ERR("Warning: ADS clock not running: %d", ret);
    }

    // ---------------------------------------
    // ADS1299 setup
    // ---------------------------------------

    ret = ads.init(&id);
    if (ret) {
        LOG_ERR("ADS1299 init failed: %d", ret);
        return 0;
    }

     
    ret = setup_ads(AdsPreset::AllChannelsMeasurement);

    /*
     * Dump registers before RDATAC. Register access should be done outside
     * RDATAC mode.
     */
    ret = ads.dumpTestRegisters();
    if (ret) {
        LOG_ERR("ADS register dump failed: %d", ret);
        return 0;
    }

    // ret = ads.dumpTestRegistersUsb();
    // if (ret) {
    //     LOG_ERR("ADS register dump failed: %d", ret);
    //     return 0;
    // }

    ret = ads.startContinuousRead();
    if (ret) {
        LOG_ERR("ADS RDATAC failed: %d", ret);
        return 0;
    }

    ret = ads.startConversions();
    if (ret) {
        LOG_ERR("ADS START failed: %d", ret);
        return 0;
    }

    // create RTOS threads
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
        5,          // low priority
        0,
        K_NO_WAIT
    );

    k_thread_create(
        &led_thread_data,
        led_stack,
        K_THREAD_STACK_SIZEOF(led_stack),
        led_toggling,
        NULL, NULL, NULL,
        5,          // low priority
        0,
        K_NO_WAIT
    );

    k_thread_create(
        &log_thread_data,
        log_stack,
        K_THREAD_STACK_SIZEOF(log_stack),
        live_log,
        NULL, NULL, NULL,
        5,
        0,
        K_NO_WAIT
    );


    // ---------------------------------------
    // main loop
    // ---------------------------------------

    while (true) {
        k_sleep(K_SECONDS(1));
    }

    return 0;
}