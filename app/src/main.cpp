/**
 * @file main.cpp
 * @brief ADS1299 EMG acquisition entry point, streaming binary data over BLE NUS.
 * @version 0.2
 * @date 2026-07-08
 */

#include <zephyr/kernel.h>
#include <zephyr/device.h>
#include <zephyr/devicetree.h>
#include <zephyr/logging/log.h>

#include <errno.h>
#include <stdint.h>
#include <stddef.h>
#include <stdio.h>

#include "ads1299_definitions.h"
#include "ads1299_driver.hpp"
#include "ble/ble_nus.hpp"
#include "usb/usb_cdc.hpp"
#include "ads1299_configs.hpp"
#include "threads.hpp"
#include "bhi360_driver.hpp"

/*
 * Devicetree aliases / node labels for the Sokosti board.
 * Status LEDs are owned by the LED thread in threads.cpp.
 */
#define ADS_NODE        DT_NODELABEL(ads1299)
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

static const struct gpio_dt_spec reset_pin =
    GPIO_DT_SPEC_GET(RESET_NODE, gpios);

static const struct gpio_dt_spec start_pin =
    GPIO_DT_SPEC_GET(START_NODE, gpios);

static const struct gpio_dt_spec drdy_pin =
    GPIO_DT_SPEC_GET(DRDY_NODE, gpios);

static const struct pwm_dt_spec ads_clk_pwm =
    PWM_DT_SPEC_GET(ADS_CLK_NODE);

/* Single ADS1299 instance — all board pins are bound at construction. */
ADS1299 ads(ads_spi, reset_pin, start_pin, drdy_pin, ads_clk_pwm);

static ADS1299Settings cfg = makeAdsSettings(AdsPreset::AllChannelsMeasurement);

LOG_MODULE_REGISTER(main, LOG_LEVEL_DBG);

int32_t last_ch1_code = 0;

int main(void)
{
    int ret;
    uint8_t id = 0;
    bool usb_active = false;

    // ---------------------------------------
    // USB CDC bring-up (non-blocking)
    // ---------------------------------------

    ret = usb_cdc::init();
    if (ret != 0) {
        LOG_WRN("USB CDC init failed: %d — BLE-only mode", ret);
    }

    // ---------------------------------------
    // Bluetooth NUS bring-up (advertising deferred)
    // ---------------------------------------

    ret = ble_nus_init();
    if (ret != 0) {
        LOG_ERR("BLE NUS init failed: %d", ret);
        return 0;
    }

    /*
     * Transport arbitration: if a USB host has already opened the virtual
     * COM port (DTR asserted), USB is the active transport and BLE stays
     * silent. Otherwise BLE starts advertising as the fallback path.
     */
    if (usb_cdc::connected()) {
        usb_active = true;
        LOG_INF("USB host detected — USB transport active, BLE idle");
    } else {
        ble_nus_start_advertising();
        LOG_INF("No USB host — BLE advertising started");
    }

    LOG_INF("ADS1299 internal test signal capture starting...");

    // ---------------------------------------
    // Board bring-up — GPIOs, DRDY IRQ, PWM clock
    // ---------------------------------------

    ret = ads.boardBringUp();
    if (ret) {
        LOG_ERR("Board bring-up failed: %d", ret);
        return 0;
    }

    k_sleep(K_MSEC(500));

    // ---------------------------------------
    // ADS1299 initialisation & configuration
    // ---------------------------------------

    ret = ads.init(&id);
    if (ret) {
        LOG_ERR("ADS1299 init failed: %d", ret);
        return ret;
    }
    
    ret = ads.configure(cfg);
    if (ret) {
        LOG_ERR("ADS configuration failed: %d", ret);
        return ret;
    }

    /*
     * Dump registers before RDATAC. Register access should be done outside
     * RDATAC mode.
     */
    ret = ads.dumpTestRegisters();
    if (ret) {
        LOG_ERR("ADS register dump failed: %d", ret);
        return 0;
    }

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

    // ---------------------------------------
    // BHI360 initialisation (independent nrfx_spim path)
    // ---------------------------------------

    bhi360_init();

    // create RTOS threads
    threads_setup();


    // ---------------------------------------
    // main loop — transport arbitration & safety stop
    // ---------------------------------------
    bool ads_conversions_running = true;

    while (true) {
        // Safe backup shutdown: stop ADC hardware physical sampling when recording finishes
        if (!recording_active && ads_conversions_running) {
            ads_conversions_running = false;
            ads.stopConversions();
            LOG_INF("ADC Conversion stopped cleanly.");
        }

        bool usb_now = usb_cdc::connected();

        if (usb_now != usb_active) {
            if (usb_now) {
                /* USB host just opened the port: USB wins, BLE goes quiet. */
                usb_active = true;
                ble_nus_stop_advertising();
                LOG_INF("USB host connected. BLE advertising stopped.");
            } else {
                /* USB host released the port: fall back to BLE. */
                usb_active = false;
                ble_nus_start_advertising();
                LOG_INF("USB host disconnected. BLE advertising started.");
            }
        }

        k_sleep(K_MSEC(500));
    }

    return 0;
}
