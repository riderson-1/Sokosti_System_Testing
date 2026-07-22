/**
 * @file ADS1299.hpp
 * @brief Zephyr C++ driver class for the TI ADS1299 8-channel EMG ADC.
 * @version 0.2
 * @date 2026-07-08
 *
 * C++ OOP port of ads1299_driver.c/.h, structured after BME_ADS1299.h/.cpp
 * (Christian Antfolk, modified from Conor Russomanno's Arduino driver).
 * Hardware bindings (SPI bus, RESET GPIO) are bound once at construction
 * via devicetree specs, instead of the file-scope globals used by the
 * original C driver.
 */

#pragma once // Exclude multiple includes

#include <zephyr/kernel.h>
#include <zephyr/sys/printk.h>
#include <zephyr/drivers/spi.h>
#include <zephyr/drivers/gpio.h>

#include <errno.h>
#include <stdint.h>
#include <stddef.h>

#include "ads1299_definitions.h"

struct ADS1299ChannelSettings {
    uint8_t powerDown   = 0;        // PDn: 0 = powered on, 1 = powered down
    uint8_t gain        = 1;        // PGA gain: 1,2,4,6,8,12,24
    uint8_t srb2        = 0;        // 1 = connect channel N's SRB2 switch (needed for reference-electrode-free bias/ref schemes)
    uint8_t mux         = 0;        // 0=normal electrode,1=shorted,2=bias meas,3=MVDD,4=temp,5=test signal,6=BIAS_DRP,7=BIAS_DRN
};

struct ADS1299DeviceSettings {
    uint16_t samplingRate   = 250; // SPS: 250,500,1000,2000,4000,8000,16000
    uint8_t  nDaisyChain    = 0;   // DAISY_EN: 0 = daisy-chain mode (what the dual-ADS1299 setup needs), 1 = multiple readback mode
    uint8_t  clkEn          = 0;   // 1 = output internal osc on CLK pin

    uint8_t  intCal     = 1;        // CONFIG2: 1 = internal test signal enabled
    uint8_t  calAmp     = 1;        // 0 = 1x test amplitude, 1 = 2x
    uint8_t  calFreq    = 1;        // 0 = fCLK/2^21 pulsed, 1 = fCLK/2^20 pulsed, 3 = dc

    uint8_t  nPdRefBuf      = 1;    // CONFIG3: 1 = enable internal reference buffer
    uint8_t  biasMeas       = 0;    // diagnostic only; routes BIAS_IN back into a channel for measurement
    uint8_t  biasRefInt     = 1;    // 1 = BIASREF = (AVDD+AVSS)/2 generated internally, 0 externally
    uint8_t  nPdBias        = 0;    // 1 = BIAS buffer enabled, 0 = disable bias electrode output
    uint8_t  biasLoffSens   = 0;    // bias lead off detection

    uint8_t  compThreshold  = 0;    // LOFF comparator threshold, 0-7
    uint8_t  iLeadOff       = 0;    // lead-off current magnitude, 0-3
    uint8_t  fLeadOff       = 0;    // lead-off frequency, 0-3

    uint8_t  biasSensP  = 0xFF;     // per-channel bitmask, ch1=bit0..ch8=bit7, all channels
    uint8_t  biasSensN  = 0x00;     // not active because monopolar measurement
    uint8_t  loffSensP  = 0x00;     
    uint8_t  loffSensN  = 0x00;
    uint8_t  loffFlip   = 0x00;

    uint8_t  gpio   = 0x0F;
    uint8_t  srb1   = 1;            // MISC1: 1 = connect SRB1 to all inverting inputs

    uint8_t  singleShot = 0;        // CONFIG4
    uint8_t  nPdLoffComp = 0;       // lead off comparator power down
};

struct ADS1299Settings {
    ADS1299DeviceSettings  device;
    ADS1299ChannelSettings channel[ADS_NUM_CHANNELS];
};

class ADS1299 {
public:
    /**
     * @param spi         SPI bus + config for this device
     *                     (from devicetree, e.g. SPI_DT_SPEC_GET(...))
     * @param reset_gpio  RESET pin spec
     *                     (from devicetree, e.g. GPIO_DT_SPEC_GET(...))
     */
    ADS1299(const struct spi_dt_spec &spi, const struct gpio_dt_spec &reset_gpio);

    /**
     * @brief initialization of ads1299
     * 
     * @return int 
     */
    int init(uint8_t *id_out);
    
    /**
     * @brief 
     * 
     * @param cfg full devie+channel settings to apply
     * @return int 
     */
    int configure(const ADS1299Settings &cfg);
    int dumpTestRegisters();

    /* streaming */
    int stopContinuousRead();
    int stopConversions();
    int startConversions();
    int startContinuousRead();

    // ---- Data acquisition (RDATAC mode) ----
    int readFrameRdatac(uint8_t frame[ADS_DAISY_FRAME_BYTES]);

private:
    
    int spiWriteBytes(const uint8_t *data, size_t len);
    int spiTransceiveBytes(const uint8_t *tx_data, uint8_t *rx_data, size_t len);

    // ---- System commands (Datasheet, pg. 35) ----
    int sendCommand(uint8_t cmd);
    void hwReset();

    // ---- Register access ----
    int readRegister(uint8_t reg, uint8_t *value);
    int writeRegister(uint8_t reg, uint8_t value);
    int writeRegisters(uint8_t start_reg, const uint8_t *values, size_t count);


    struct spi_dt_spec spi_;
    struct gpio_dt_spec reset_gpio_;

    ADS1299Settings settings;   // public, or add a reference-returning accessor if you prefer encapsulation
};