/**
 * @file main.cpp
 * @brief ADS1299 EMG acquisition entry point, streaming CSV over USB CDC-ACM.
 * @version 0.2
 * @date 2026-07-08
 */

#include <zephyr/kernel.h>
#include <zephyr/sys/printk.h>
#include <zephyr/device.h>
#include <zephyr/drivers/spi.h>
#include <zephyr/drivers/gpio.h>
#include <zephyr/drivers/pwm.h>
#include <zephyr/devicetree.h>

#include <errno.h>
#include <stdint.h>
#include <stddef.h>
#include <stdio.h>


#include "ads1299_definitions.h"
#include "ads1299_driver.hpp"
#include "usb_cdc.hpp"
#include "ads1299_configs.hpp"

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
        printk("ADS clock PWM not ready\n");
        return -ENODEV;
    }

    /*
     * ~2 MHz: 500 ns period, 250 ns pulse.
     * ADS1299 typical external fCLK is 2.048 MHz in datasheet examples [14].
     */
    int ret = pwm_set_dt(&ads_clk_pwm, PWM_NSEC(500), PWM_NSEC(250));
    if (ret) {
        printk("PWM start failed: %d\n", ret);
        return ret;
    }

    printk("ADS clock PWM started\n");
    return 0;
}

static int setup_gpios(void)
{
    int ret;

    if (!gpio_is_ready_dt(&led) ||
        !gpio_is_ready_dt(&reset_pin) ||
        !gpio_is_ready_dt(&start_pin) ||
        !gpio_is_ready_dt(&drdy_pin)) {
        printk("GPIO not ready\n");
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
        printk("ADS configuration failed: %d\n", ret);
    }
    return ret;
}

int main(void)
{
    int ret;
    uint8_t id = 0;
    uint8_t frame[ADS_FRAME_BYTES];
    uint32_t sample_idx = 0;

    // ---------------------------------------
    // board setup
    // ---------------------------------------

    ret = usb_cdc::init();
    if (ret != 0) {
        printk("USB CDC init failed: %d\n", ret);
        return 0;
    }

    k_msleep(100);
    k_sleep(K_MSEC(500));
    printk("ADS1299 internal test signal capture starting...\n");

    if (!spi_is_ready_dt(&ads_spi)) {
        printk("SPI device not ready\n");
        return 0;
    }

    ret = setup_gpios();
    if (ret) {
        printk("GPIO setup failed: %d\n", ret);
        return 0;
    }

    ret = setup_drdy_interrupt();
    if (ret) {
        printk("DRDY interrupt setup failed: %d\n", ret);
        return 0;
    }

    /*
     * Keep START pin low and use the SPI START command.
     * Datasheet says when using START command, hold START pin low [9], [27].
     */
    gpio_pin_set_dt(&start_pin, 0);

    ret = start_ads_pwm_clock();
    if (ret) {
        printk("Warning: ADS clock not running: %d\n", ret);
    }

    // ---------------------------------------
    // ADS1299 setup
    // ---------------------------------------

    ret = ads.init(&id);
    if (ret) {
        printk("ADS1299 init failed: %d\n", ret);
        return 0;
    }

     
    ret = setup_ads(AdsPreset::SingleChannelTest);

    /*
     * Dump registers before RDATAC. Register access should be done outside
     * RDATAC mode.
     */
    ret = ads.dumpTestRegisters();
    if (ret) {
        printk("ADS register dump failed: %d\n", ret);
        return 0;
    }

    ads.startContinuousRead();
    if (ret) {
        printk("ADS RDATAC failed: %d\n", ret);
        return 0;
    }

    ads.startConversions();
    if (ret) {
        printk("ADS START failed: %d\n", ret);
        return 0;
    }

    // ---------------------------------------
    // main loop
    // ---------------------------------------

    /* Send once, before entering the loop */
    static const char hdr[] =
    "sample,status1_ok,status2_ok,"
    "ch1,ch2,ch3,ch4,ch5,ch6,ch7,ch8,"
    "ch9,ch10,ch11,ch12,ch13,ch14,ch15,ch16\r\n";

    usb_cdc::print(hdr, sizeof(hdr) - 1);

    while (1) {
        k_sem_take(&drdy_sem, K_FOREVER);
        
        /*
        * NOTE:
        * With your definitions:
        *   ADS_NUM_CHANNELS = 16
        *   ADS_FRAME_BYTES  = 3 + 16 * 3 = 51
        *
        * However, in ADS1299 daisy-chain mode, the datasheet says status/data
        * from device 1 appear first, followed by status/data from device 2 [5].
        * One 8-channel ADS1299 frame is 24 status bits + 8 * 24 data bits = 27 bytes [2].
        * Therefore two daisy-chained 8-channel devices produce 54 bytes, not 51.
        *
        * So this reads ADS_FRAME_BYTES + 3 bytes:
        *   device 1: 3 status + 8 channels * 3 = 27 bytes
        *   device 2: 3 status + 8 channels * 3 = 27 bytes
        */

        ret = ads.readFrameRdatac(frame);
        if (ret) {
            printk("Frame read failed: %d\n", ret);
            gpio_pin_toggle_dt(&led);
            continue;
        }

        // Device 1 (logical channels 1–8)
        bool status1_ok = ((frame[0] & 0xF0) == 0xC0);   // was frame[1]
        int32_t ch1  = ADS1299::decode24(&frame[3]);
        int32_t ch2  = ADS1299::decode24(&frame[6]);
        int32_t ch3  = ADS1299::decode24(&frame[9]);
        int32_t ch4  = ADS1299::decode24(&frame[12]);
        int32_t ch5  = ADS1299::decode24(&frame[15]);
        int32_t ch6  = ADS1299::decode24(&frame[18]);
        int32_t ch7  = ADS1299::decode24(&frame[21]);
        int32_t ch8  = ADS1299::decode24(&frame[24]);

        // Device 2 (logical channels 9–16)
        bool status2_ok = ((frame[27] & 0xF0) == 0xC0);  // was frame[12] — that was actually inside device 1's ch4!
        int32_t ch9  = ADS1299::decode24(&frame[30]);
        int32_t ch10 = ADS1299::decode24(&frame[33]);
        int32_t ch11 = ADS1299::decode24(&frame[36]);
        int32_t ch12 = ADS1299::decode24(&frame[39]);
        int32_t ch13 = ADS1299::decode24(&frame[42]);
        int32_t ch14 = ADS1299::decode24(&frame[45]);
        int32_t ch15 = ADS1299::decode24(&frame[48]);
        int32_t ch16 = ADS1299::decode24(&frame[51]);

        char msg[192];
        int len = snprintf(msg, sizeof(msg),
            "%lu,%d,%d,%ld,%ld,%ld,%ld,%ld,%ld,%ld,%ld,%ld,%ld,%ld,%ld,%ld,%ld,%ld,%ld\r\n",
            (unsigned long)sample_idx,
            status1_ok ? 1 : 0,
            status2_ok ? 1 : 0,
            (long)ch1,  (long)ch2,  (long)ch3,  (long)ch4,
            (long)ch5,  (long)ch6,  (long)ch7,  (long)ch8,
            (long)ch9,  (long)ch10, (long)ch11, (long)ch12,
            (long)ch13, (long)ch14, (long)ch15, (long)ch16);

        if (len > 0 && len < static_cast<int>(sizeof(msg))) {
            usb_cdc::print(msg, len);
        } else {
            printk("CSV message truncated: len=%d\n", len);
        }

        if ((sample_idx % 250U) == 0U) {
            gpio_pin_toggle_dt(&led);
        }

        sample_idx++;
    }

    return 0;
}