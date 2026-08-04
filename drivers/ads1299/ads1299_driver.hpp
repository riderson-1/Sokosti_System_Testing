/**
 * @file ADS1299.hpp
 * @brief Zephyr C++ driver class for the TI ADS1299 8-channel EMG ADC.
 * @version 0.3
 * @date 2026-07-29
 *
 * C++ OOP port of ads1299_driver.c/.h, structured after BME_ADS1299.h/.cpp
 * (Christian Antfolk, modified from Conor Russomanno's Arduino driver).
 * Hardware bindings (SPI, RESET, START, DRDY, LED, PWM clock) are all bound
 * at construction via devicetree specs.  boardBringUp() orchestrates the
 * entire board-level initialisation (GPIOs, DRDY interrupt, PWM clock) so
 * that main() only needs to call that one function before ads.init().
 */

#pragma once

#include <zephyr/kernel.h>
#include <zephyr/sys/printk.h>
#include <zephyr/drivers/spi.h>
#include <zephyr/drivers/gpio.h>
#include <zephyr/drivers/pwm.h>
#include <zephyr/logging/log.h>

#include <errno.h>
#include <stdint.h>
#include <stddef.h>
#include <stdio.h>

#include "ads1299_definitions.h"

struct ADS1299ChannelSettings {
    uint8_t powerDown   = 0;        // PDn: 0 = powered on, 1 = powered down
    uint8_t gain        = 1;        // PGA gain: 1,2,4,6,8,12,24
    uint8_t srb2        = 0;        // 1 = connect channel N's SRB2 switch
    uint8_t mux         = 0;        // 0=normal,1=shorted,2=bias meas,3=MVDD,4=temp,5=test,6=BIAS_DRP,7=BIAS_DRN
};

struct ADS1299DeviceSettings {
    uint16_t samplingRate   = 250;  // SPS: 250,500,1000,2000,4000,8000,16000
    uint8_t  nDaisyChain    = 0;    // DAISY_EN: 0 = daisy-chain mode, 1 = multiple readback
    uint8_t  clkEn          = 0;    // 1 = output internal osc on CLK pin

    uint8_t  intCal         = 1;    // CONFIG2: internal test signal enabled
    uint8_t  calAmp         = 1;    // 0 = 1x test amplitude, 1 = 2x
    uint8_t  calFreq        = 1;    // 0 = fCLK/2^21 pulsed, 1 = fCLK/2^20 pulsed, 3 = dc

    uint8_t  nPdRefBuf      = 1;    // CONFIG3: enable internal reference buffer
    uint8_t  biasMeas       = 0;    // routes BIAS_IN into a channel for measurement
    uint8_t  biasRefInt     = 1;    // 1 = BIASREF internal, 0 = external
    uint8_t  nPdBias        = 0;    // 1 = BIAS buffer enabled
    uint8_t  biasLoffSens   = 0;    // bias lead-off detection

    uint8_t  compThreshold  = 0;    // LOFF comparator threshold, 0-7
    uint8_t  iLeadOff       = 0;    // lead-off current magnitude, 0-3
    uint8_t  fLeadOff       = 0;    // lead-off frequency, 0-3

    uint8_t  biasSensP  = 0xFF;     // per-channel bitmask, ch1=bit0..ch8=bit7
    uint8_t  biasSensN  = 0x00;
    uint8_t  loffSensP  = 0x00;
    uint8_t  loffSensN  = 0x00;
    uint8_t  loffFlip   = 0x00;

    uint8_t  gpio   = 0x0F;
    uint8_t  srb1   = 1;            // MISC1: connect SRB1 to all inverting inputs

    uint8_t  singleShot  = 0;       // CONFIG4
    uint8_t  nPdLoffComp = 0;       // lead-off comparator power down
};

struct ADS1299Settings {
    ADS1299DeviceSettings  device;
    ADS1299ChannelSettings channel[ADS_NUM_CHANNELS];
};

class ADS1299 {
public:
    /**
     * @param spi         SPI bus + config (from devicetree, e.g. SPI_DT_SPEC_GET(...))
     * @param reset_gpio  RESET pin spec
     * @param start_pin   START pin spec (held low; START command used instead)
     * @param drdy_pin    DRDY pin spec (falling-edge interrupt)
     * @param led_pin     LED pin spec (heartbeat toggling)
     * @param ads_clk_pwm PWM clock spec (~2 MHz for ADS1299 external fCLK)
     */
    ADS1299(const struct spi_dt_spec &spi,
            const struct gpio_dt_spec &reset_gpio,
            const struct gpio_dt_spec &start_pin,
            const struct gpio_dt_spec &drdy_pin,
            const struct gpio_dt_spec &led_pin,
            const struct pwm_dt_spec   &ads_clk_pwm);

    /**
     * @brief One-shot board bring-up: checks HW ready, configures GPIOs,
     *        sets up the DRDY falling-edge interrupt, starts the PWM clock,
     *        and holds START low (START command is used instead).
     * @return 0 on success, negative errno on failure.
     */
    int boardBringUp();

    /**
     * @brief Initialise the ADS1299: HW reset, CMD_RESET, SDATAC, read ID.
     * @param[out] id_out  The REG_ID value read back.
     * @return 0 on success, negative errno on failure.
     */
    int init(uint8_t *id_out);

    /**
     * @brief Apply a full device + channel configuration.
     * @param cfg  The settings to write.
     * @return 0 on success, negative errno on failure.
     */
    int configure(const ADS1299Settings &cfg);

    int dumpTestRegisters();
    int dumpTestRegistersUsb();

    /* streaming */
    int stopContinuousRead();
    int stopConversions();
    int startConversions();
    int startContinuousRead();

    /** Read one frame in RDATAC mode (dual-device daisy-chain = 54 bytes). */
    int readFrameRdatac(uint8_t frame[ADS_DAISY_FRAME_BYTES]);

    /** Sign-extend a 24-bit two's-complement value. */
    static int32_t decode24(const uint8_t *p);

    /** DRDY semaphore — given by ISR, taken by acquisition thread. */
    static struct k_sem drdy_sem;

private:
    int  spiWriteBytes(const uint8_t *data, size_t len);
    int  spiTransceiveBytes(const uint8_t *tx_data, uint8_t *rx_data, size_t len);
    int  sendCommand(uint8_t cmd);
    void hwReset();

    int  readRegister(uint8_t reg, uint8_t *value);
    int  writeRegister(uint8_t reg, uint8_t value);
    int  writeRegisters(uint8_t start_reg, const uint8_t *values, size_t count);
    static const char *registerName(uint8_t addr);

    /* ---- board bring-up helpers (called by boardBringUp) ---- */
    int  setupGpios();
    int  setupDrdyInterrupt();
    int  startAdsPwmClock();

    /** DRDY falling-edge ISR — gives the drdy_sem semaphore. */
    static void drdyIsr(const struct device *dev,
                        struct gpio_callback *cb,
                        uint32_t pins);

    /* ---- hardware handles ---- */
    struct spi_dt_spec      spi_;
    struct gpio_dt_spec     reset_gpio_;
    struct gpio_dt_spec     start_pin_;
    struct gpio_dt_spec     drdy_pin_;
    struct gpio_dt_spec     led_pin_;
    struct pwm_dt_spec      ads_clk_pwm_;

    static struct gpio_callback drdy_cb_data_;

    ADS1299Settings settings;
};