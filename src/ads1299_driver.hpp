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

    /* configuration */
    int dumpTestRegisters();
    int configureExternalInputsAll();
    int configureInternalTestSignal();

    /* streaming */
    int stopContinuousRead();
    int startConversions();
    int startContinuousRead();

    // ---- Data acquisition (RDATAC mode) ----
    int readFrameRdatac(uint8_t frame[ADS_FRAME_BYTES]);

    // ---- Utility ----
    static int32_t decode24(const uint8_t *p);

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
};