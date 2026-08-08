/**
 * @file ADS1299.cpp
 * @brief Zephyr C++ driver class for the TI ADS1299 8-channel EMG ADC on Skosti board.
 * @version 0.2
 * @date 2026-07-08
 */

#include "ads1299_driver.hpp"

LOG_MODULE_REGISTER(ads1299_driver, LOG_LEVEL_DBG);

ADS1299::ADS1299(const struct spi_dt_spec &spi,
                 const struct gpio_dt_spec &reset_gpio,
                 const struct gpio_dt_spec &start_pin,
                 const struct gpio_dt_spec &drdy_pin,
                 const struct gpio_dt_spec &led_pin,
                 const struct pwm_dt_spec   &ads_clk_pwm)
    : spi_(spi),
      reset_gpio_(reset_gpio),
      start_pin_(start_pin),
      drdy_pin_(drdy_pin),
      led_pin_(led_pin),
      ads_clk_pwm_(ads_clk_pwm)
{
    for (auto &ch : settings.channel) {
        ch.mux = 0;  // normal electrode input
    }
}

int ADS1299::configure(const ADS1299Settings &cfg)
{
    int ret;

    //copy callers config
    settings = cfg; 

    ret = sendCommand(CMD_SDATAC);
    if (ret) return ret;
    k_sleep(K_MSEC(2));

    ret = sendCommand(CMD_STOP);
    if (ret) return ret;
    k_sleep(K_MSEC(2));

    // CONFIG1: bit7=1 fixed, bits4:3=10 fixed -> base 0x90
    uint8_t config1 = 0x90;
    if (settings.device.nDaisyChain) config1 |= 0x40; // DAISY_EN=1
    if (settings.device.clkEn)            config1 |= 0x20;
    switch (settings.device.samplingRate) {
        case 16000: config1 |= 0x00; break;
        case 8000:  config1 |= 0x01; break;
        case 4000:  config1 |= 0x02; break;
        case 2000:  config1 |= 0x03; break;
        case 1000:  config1 |= 0x04; break;
        case 500:   config1 |= 0x05; break;
        case 250:
        default:    config1 |= 0x06; break;
    }
    ret = writeRegister(REG_CONFIG1, config1);
    if (ret) return ret;

    // CONFIG2: bits7:6=11 fixed -> base 0xC0
    uint8_t config2 = 0xC0;
    if (settings.device.intCal) config2 |= 0x10;
    if (settings.device.calAmp) config2 |= 0x04;
    config2 |= (settings.device.calFreq & 0x03);
    ret = writeRegister(REG_CONFIG2, config2);
    if (ret) return ret;

    // CONFIG3: bits6:5=11 fixed -> base 0x60
    uint8_t config3 = 0x60;
    if (settings.device.nPdRefBuf)     config3 |= 0x80;
    if (settings.device.biasMeas)     config3 |= 0x10;
    if (settings.device.biasRefInt)   config3 |= 0x08;
    if (settings.device.nPdBias)       config3 |= 0x04;
    if (settings.device.biasLoffSens) config3 |= 0x02;
    ret = writeRegister(REG_CONFIG3, config3);
    if (ret) return ret;

    // LOFF
    uint8_t loff = (uint8_t)((settings.device.compThreshold & 0x07) << 5) |
                   (uint8_t)((settings.device.iLeadOff      & 0x03) << 2) |
                   (uint8_t)(settings.device.fLeadOff       & 0x03);
    ret = writeRegister(REG_LOFF, loff);
    if (ret) return ret;

    ret = writeRegister(REG_BIAS_SENSP, settings.device.biasSensP);
    if (ret) return ret;
    ret = writeRegister(REG_BIAS_SENSN, settings.device.biasSensN);
    if (ret) return ret;
    ret = writeRegister(REG_LOFF_SENSP, settings.device.loffSensP);
    if (ret) return ret;
    ret = writeRegister(REG_LOFF_SENSN, settings.device.loffSensN);
    if (ret) return ret;
    ret = writeRegister(REG_LOFF_FLIP, settings.device.loffFlip);
    if (ret) return ret;
    ret = writeRegister(REG_GPIO, settings.device.gpio);
    if (ret) return ret;
    ret = writeRegister(REG_MISC1, settings.device.srb1 ? 0x20 : 0x00);
    if (ret) return ret;

    uint8_t config4 = 0x00;
    if (settings.device.singleShot) config4 |= 0x04;
    if (settings.device.nPdLoffComp) config4 |= 0x02;
    ret = writeRegister(REG_CONFIG4, config4);
    if (ret) return ret;

    // Per-channel: build all 8 CHnSET bytes and burst-write in one WREG
    uint8_t chset[ADS_NUM_CHANNELS];
    for (int i = 0; i < ADS_NUM_CHANNELS; i++) {
        uint8_t v = 0;
        if (settings.channel[i].powerDown) v |= 0x80;
        switch (settings.channel[i].gain) {
            case 1:  v |= 0x00; break;
            case 2:  v |= 0x10; break;
            case 4:  v |= 0x20; break;
            case 6:  v |= 0x30; break;
            case 8:  v |= 0x40; break;
            case 12: v |= 0x50; break;
            case 24:
            default: v |= 0x60; break;
        }
        if (settings.channel[i].srb2) v |= 0x08;
        v |= (settings.channel[i].mux & 0x07);
        chset[i] = v;
    }
    ret = writeRegisters(REG_CH1SET, chset, ADS_NUM_CHANNELS);
    if (ret) return ret;

    k_sleep(K_MSEC(2));
    LOG_INF("ADS1299 configured (mux[0]=%u, gain[0]=%u, intCal=%u)",
           settings.channel[0].mux, settings.channel[0].gain, settings.device.intCal);
    return 0;
}

int ADS1299::spiWriteBytes(const uint8_t *data, size_t len)
{
    /*
     * NOTE: designated initializers (.buf = ..., .len = ...) are avoided
     * here on purpose. They're a C++20 feature (GCC accepts them earlier
     * as an extension, but it's not portable across -std= settings), so
     * plain field assignment is used instead for a clean C -> C++ port.
     */
    struct spi_buf txb;
    txb.buf = (void *)data;
    txb.len = len;

    struct spi_buf_set txs;
    txs.buffers = &txb;
    txs.count = 1;

    return spi_write_dt(&spi_, &txs);
}

int ADS1299::spiTransceiveBytes(const uint8_t *tx_data, uint8_t *rx_data, size_t len)
{
    struct spi_buf txb;
    txb.buf = (void *)tx_data;
    txb.len = len;

    struct spi_buf_set txs;
    txs.buffers = &txb;
    txs.count = 1;

    struct spi_buf rxb;
    rxb.buf = rx_data;
    rxb.len = len;

    struct spi_buf_set rxs;
    rxs.buffers = &rxb;
    rxs.count = 1;

    return spi_transceive_dt(&spi_, &txs, &rxs);
}

int ADS1299::sendCommand(uint8_t cmd)
{
    int ret = spiWriteBytes(&cmd, 1);

    /*
     * ADS1299 commands require decode time. The datasheet says commands
     * require 4 tCLK cycles to decode/execute [17].
     */
    k_busy_wait(10);

    return ret;
}

int ADS1299::readRegister(uint8_t reg, uint8_t *value)
{
    uint8_t tx[3] = { (uint8_t)(CMD_RREG | reg), 0x00, 0x00 };
    uint8_t rx[3] = { 0 };
    int ret = spiTransceiveBytes(tx, rx, sizeof(tx));
    if (ret == 0) {
        *value = rx[2];
    }
    return ret;
}

int ADS1299::writeRegister(uint8_t reg, uint8_t value)
{
    uint8_t tx[3] = {
        static_cast<uint8_t>(CMD_WREG | reg),
        0x00,
        value,
    };

    int ret = spiWriteBytes(tx, sizeof(tx));
    k_busy_wait(10);
    return ret;
}

int ADS1299::writeRegisters(uint8_t start_reg, const uint8_t *values, size_t count)
{
    uint8_t tx[3 + ADS_NUM_CHANNELS];   /* cmd + count_byte + up to 8 data bytes */

    if (count == 0 || count > ADS_NUM_CHANNELS) {
        return -EINVAL;
    }

    tx[0] = (uint8_t)(CMD_WREG | start_reg);
    tx[1] = (uint8_t)(count - 1);
    memcpy(&tx[2], values, count);

    int ret = spiWriteBytes(tx, 2 + count);
    k_busy_wait(10);
    return ret;
}
/*
 * In RDATAC mode, after DRDY goes low, clock out:
 * 24 status bits + 24 bits per channel.
 * For 8 channels this is 27 bytes total [22], [23].
 */
int ADS1299::readFrameRdatac(uint8_t frame[ADS_DAISY_FRAME_BYTES])
{
    uint8_t tx[ADS_DAISY_FRAME_BYTES] = { 0 };
    uint8_t rx[ADS_DAISY_FRAME_BYTES] = { 0 };

    int ret = spiTransceiveBytes(tx, rx, sizeof(tx));
    if (ret) {
        return ret;
    }

    memcpy(frame, rx, ADS_DAISY_FRAME_BYTES);
    return 0;
}

void ADS1299::hwReset()
{
    gpio_pin_set_dt(&reset_gpio_, 1);
    k_sleep(K_MSEC(10));

    gpio_pin_set_dt(&reset_gpio_, 0);
    k_sleep(K_MSEC(50));
}

int ADS1299::init(uint8_t *id_out)
{
    int ret;
    uint8_t id = 0;

    k_sleep(K_MSEC(20));

    // 1. hw reset
    hwReset();

    // 2. send reset command
    ret = sendCommand(CMD_RESET);
    if (ret) {
        LOG_ERR("CMD_RESET failed: %d", ret);
        return ret;
    }

    // 3. wait longer than 18 tCLK
    k_sleep(K_MSEC(10));

    // 4. send SDATAC
    ret = stopContinuousRead();
    if (ret) {
        LOG_ERR("ADS1299: SDATAC failed: %d", ret);
        return ret;
    }

    // 5. read ID register
    ret = readRegister(REG_ID, &id);
    if (ret != 0) {
        LOG_ERR("ADS1299: ID register read failed");
        return ret;
    }
    
    // 6. validate device
    /* ADS1299 family ID: lower 5 bits should read 0x1E (Datasheet, REG_ID) */
    if ((id & 0x1F) != 0x1E) {
        LOG_ERR("ADS1299: unexpected ID 0x%02X (expected low 5 bits = 0x1E)", id);
        return -ENODEV;
    }

    LOG_INF("ADS1299 detected, ID = 0x%02X", id);

    *id_out = id;

    // 7. Leave ADS in SDATAC so registers can be configured
    return 0;
}

const char* ADS1299::registerName(uint8_t addr)
{
    switch (addr) {
        case 0x00: return "ID";
        case 0x01: return "CONFIG1";
        case 0x02: return "CONFIG2";
        case 0x03: return "CONFIG3";
        case 0x04: return "LOFF";
        case 0x05: return "CH1SET";
        case 0x06: return "CH2SET";
        case 0x07: return "CH3SET";
        case 0x08: return "CH4SET";
        case 0x09: return "CH5SET";
        case 0x0A: return "CH6SET";
        case 0x0B: return "CH7SET";
        case 0x0C: return "CH8SET";
        case 0x0D: return "BIAS_SENSP";
        case 0x0E: return "BIAS_SENSN";
        case 0x0F: return "LOFF_SENSP";
        case 0x10: return "LOFF_SENSN";
        case 0x11: return "LOFF_FLIP";
        case 0x12: return "LOFF_STATP";
        case 0x13: return "LOFF_STATN";
        case 0x14: return "GPIO";
        case 0x15: return "MISC1";
        case 0x16: return "MISC2";
        case 0x17: return "CONFIG4";
        default:   return "UNKNOWN";
    }
}

int ADS1299::dumpTestRegisters()
{
    int ret;
    uint8_t regs[0x18];

    for (uint8_t addr = 0x00; addr <= 0x17; addr++) {
        ret = readRegister(addr, &regs[addr]);
        if (ret) {
            LOG_ERR("REG 0x%02X read failed: %d", addr, ret);
            return ret;
        }
        k_msleep(2);
    }

    LOG_INF("ADS1299 Register Dump");
    
    for (uint8_t addr = 0x00; addr <= 0x17; addr++) {
        LOG_INF("0x%02X  %-13s 0x%02X", addr, registerName(addr), regs[addr]);
    }

    return 0;
}

int ADS1299::stopContinuousRead()
{
    int ret = sendCommand(CMD_SDATAC);
    if (ret) {
        return ret;
    }

    k_sleep(K_MSEC(2));

    return 0;
}

int ADS1299::startContinuousRead()
{
    int ret = sendCommand(CMD_RDATAC);
    if (ret) {
        return ret;
    }

    k_sleep(K_MSEC(2));
    return 0;
}

int ADS1299::startConversions()
{
    int ret = sendCommand(CMD_START);
    if (ret) {
        return ret;
    }

    /*
     * Conversions begin when START pin is high or START command is sent.
     * If using the START command, keep START pin low.
     */
    k_sleep(K_MSEC(20));
    return 0;
}

int ADS1299::stopConversions()
{
    int ret = sendCommand(CMD_STOP);
    if (ret) {
        return ret;
    }

    k_sleep(K_MSEC(2));
    return 0;
}

int32_t ADS1299::decode24(const uint8_t *p)
{
    int32_t v = ((int32_t)p[0] << 16) |
                ((int32_t)p[1] << 8)  |
                ((int32_t)p[2]);

    if (v & 0x800000) {
        v |= (int32_t)0xFF000000;
    }

    return v;
}

/* ------------------------------------------------------------------ */
/*  Static members (defined here, declared in the header)             */
/* ------------------------------------------------------------------ */

struct k_sem ADS1299::drdy_sem;
struct gpio_callback ADS1299::drdy_cb_data_;

/* ------------------------------------------------------------------ */
/*  Board bring-up                                                    */
/* ------------------------------------------------------------------ */

int ADS1299::boardBringUp()
{
    int ret;

    /* 0. Initialise the DRDY semaphore (binary) */
    k_sem_init(&drdy_sem, 0, 1);

    /* 1. Check SPI is ready */
    if (!spi_is_ready_dt(&spi_)) {
        LOG_ERR("SPI device not ready");
        return -ENODEV;
    }

    /* 2. Configure all GPIOs */
    ret = setupGpios();
    if (ret) {
        LOG_ERR("GPIO setup failed: %d", ret);
        return ret;
    }

    /* 3. Keep START pin low — we use the START command over SPI */
    gpio_pin_set_dt(&start_pin_, 0);

    /* 4. Set up DRDY falling-edge interrupt */
    ret = setupDrdyInterrupt();
    if (ret) {
        LOG_ERR("DRDY interrupt setup failed: %d", ret);
        return ret;
    }

    /* 5. Start the PWM clock (~2 MHz) for the ADS1299 external fCLK */
    ret = startAdsPwmClock();
    if (ret) {
        LOG_ERR("Warning: ADS clock not running: %d", ret);
        /* non-fatal — the board may use an external crystal */
    }

    LOG_INF("ADS1299 board bring-up complete");
    return 0;
}

/* ------------------------------------------------------------------ */
/*  Private helpers                                                    */
/* ------------------------------------------------------------------ */

int ADS1299::setupGpios()
{
    int ret;

    if (!gpio_is_ready_dt(&led_pin_) ||
        !gpio_is_ready_dt(&reset_gpio_) ||
        !gpio_is_ready_dt(&start_pin_) ||
        !gpio_is_ready_dt(&drdy_pin_)) {
        LOG_ERR("GPIO not ready");
        return -ENODEV;
    }

    ret = gpio_pin_configure_dt(&led_pin_, GPIO_OUTPUT_INACTIVE);
    if (ret) return ret;

    ret = gpio_pin_configure_dt(&reset_gpio_, GPIO_OUTPUT_INACTIVE);
    if (ret) return ret;

    ret = gpio_pin_configure_dt(&start_pin_, GPIO_OUTPUT_INACTIVE);
    if (ret) return ret;

    ret = gpio_pin_configure_dt(&drdy_pin_, GPIO_INPUT);
    if (ret) return ret;

    return 0;
}

int ADS1299::setupDrdyInterrupt()
{
    int ret = gpio_pin_interrupt_configure_dt(&drdy_pin_, GPIO_INT_EDGE_FALLING);
    if (ret) return ret;

    gpio_init_callback(&drdy_cb_data_, drdyIsr, BIT(drdy_pin_.pin));
    gpio_add_callback(drdy_pin_.port, &drdy_cb_data_);

    return 0;
}

int ADS1299::startAdsPwmClock()
{
    if (!pwm_is_ready_dt(&ads_clk_pwm_)) {
        LOG_ERR("ADS clock PWM not ready");
        return -ENODEV;
    }

    /*
     * ~2 MHz: 500 ns period, 250 ns pulse.
     * ADS1299 typical external fCLK is 2.048 MHz in datasheet examples [14].
     */
    int ret = pwm_set_dt(&ads_clk_pwm_, PWM_NSEC(500), PWM_NSEC(250));
    if (ret) {
        LOG_ERR("PWM start failed: %d", ret);
        return ret;
    }

    LOG_INF("ADS clock PWM started");
    return 0;
}

void ADS1299::drdyIsr(const struct device *dev,
                      struct gpio_callback *cb,
                      uint32_t pins)
{
    k_sem_give(&drdy_sem);
}
