/**
 * @file ADS1299.cpp
 * @brief Zephyr C++ driver class for the TI ADS1299 8-channel EMG ADC.
 * @version 0.2
 * @date 2026-07-08
 */

#include "ads1299_driver.hpp"

ADS1299::ADS1299(const struct spi_dt_spec &spi, const struct gpio_dt_spec &reset_gpio)
    : spi_(spi), reset_gpio_(reset_gpio)
{
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
    uint8_t tx[4] = {
        (uint8_t)(CMD_RREG | reg),
        0x00,
        0x00,
    };
    uint8_t rx[4] = { 0 };

    int ret = spiTransceiveBytes(tx, rx, sizeof(tx));
    if (ret == 0) {
        *value = rx[3];
    }

    return ret;
}

int ADS1299::writeRegister(uint8_t reg, uint8_t value)
{
    uint8_t tx[4] = {
        (uint8_t)(CMD_WREG | reg),
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

/*
 * In RDATAC mode, after DRDY goes low, clock out:
 * 24 status bits + 24 bits per channel.
 * For 8 channels this is 27 bytes total [22], [23].
 */
int ADS1299::readFrameRdatac(uint8_t frame[ADS_FRAME_BYTES])
{
    uint8_t tx[ADS_FRAME_BYTES] = { 0 };
    uint8_t rx[ADS_FRAME_BYTES] = { 0 };

    int ret = spiTransceiveBytes(tx, rx, sizeof(tx));
    if (ret) {
        return ret;
    }

    memcpy(frame, rx, ADS_FRAME_BYTES);
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
        printk("CMD_RESET failed: %d\n", ret);
        return ret;
    }

    // 3. wait longer than 18 tCLK
    k_sleep(K_MSEC(10));

    // 4. send SDATAC
    stopContinuousRead();
    if (ret) {
        printk("ADS1299: SDATAC failed: %d\n", ret);
        return ret;
    }

    // 5. read ID register
    ret = readRegister(REG_ID, &id);
    if (ret != 0) {
        printk("ADS1299: ID register read failed\n");
        return false;
    }
    
    // 6. validate device
    /* ADS1299 family ID: lower 5 bits should read 0x1E (Datasheet, REG_ID) */
    if ((id & 0x1F) != 0x1E) {
        printk("ADS1299: unexpected ID 0x%02X (expected low 5 bits = 0x1E)\n", id);
        return false;
    }

    printk("ADS1299 detected, ID = 0x%02X\n", id);

    // 7. Leave ADS in SDATAC so registers can be configured
    return 0;
}

int ADS1299::dumpTestRegisters()
{
    int ret;
    uint8_t v;

    const uint8_t regs[] = {
        REG_ID,
        REG_CONFIG1,
        REG_CONFIG2,
        REG_CONFIG3,
        REG_CH1SET,
        REG_CH2SET,
        REG_CH3SET,
        REG_CH4SET,
        REG_CH5SET,
        REG_CH6SET,
        REG_CH7SET,
        REG_CH8SET,
    };

    for (size_t i = 0; i < sizeof(regs); i++) {
        ret = readRegister(regs[i], &v);
        if (ret) {
            printk("REG 0x%02X read failed: %d\n", regs[i], ret);
            return ret;
        }

        printk("REG 0x%02X = 0x%02X\n", regs[i], v);
    }

    return 0;
}

int ADS1299::configureExternalInputsAll()
{
    int ret;

    /* Stop continuous data mode and conversions before writing registers */
    ret = stopContinuousRead();
    if (ret) {
        return ret;
    }

    ret = stopConversions();
    if (ret) {
        return ret;
    }

    /* CONFIG1 = 0x96: 250 SPS, DAISY disabled, internal clock (same as you had) */
    ret = writeRegister(REG_CONFIG1, 0x96);
    if (ret) {
        return ret;
    }

    /* CONFIG2 = 0xC0: INT_CAL=0 -> internal test signals disabled [8] */
    ret = writeRegister(REG_CONFIG2, 0xC0);
    if (ret) {
        return ret;
    }

    /* CONFIG3 = 0xE0: PD_REFBUF=1 (enable internal reference buffer) [5] */
    ret = writeRegister(REG_CONFIG3, 0xE0);
    if (ret) {
        return ret;
    }

    /*
     * CHnSET = 0x60:
     *   PDn = 0 (powered), GAIN = 110 (x24), SRB2 = 0, MUX = 000 (normal electrode input) [9].
     *   Do this for all eight channels.
     */
    const uint8_t chset_all[ADS_NUM_CHANNELS] = {
        0x60, 0x60, 0x60, 0x60,
        0x60, 0x60, 0x60, 0x60,
    };
    ret = writeRegisters(REG_CH1SET, chset_all, sizeof(chset_all));
    if (ret) {
        return ret;
    }

    k_sleep(K_MSEC(2));
    printk("ADS1299 external input mode registers written\n");
    return 0;
}

int ADS1299::configureInternalTestSignal()
{
    int ret;

    /*
     * CHnSET = 0x05:
     *   PDn = 0     : channel powered up
     *   GAINn = 000 : PGA gain 1
     *   SRB2 = 0
     *   MUXn = 101  : internal test signal
     *
     * CHnSET[2:0] = 101 selects the internally generated test signals [12], [19].
     */
    const uint8_t chset_all[ADS_NUM_CHANNELS] = {
        0x05, 0x05, 0x05, 0x05,
        0x05, 0x05, 0x05, 0x05,
    };

    const uint8_t chset_selected[ADS_NUM_CHANNELS] = {
        0x05, 0x81, 0x81, 0x81,
        0x81, 0x81, 0x81, 0x81,
    };

    printk("Configuring ADS1299 internal test signal mode...\n");

    /*
     * Stop continuous data mode before writing registers. RDATAC is the
     * default after power-up and is cancelled with SDATAC [25].
     */
    ret = sendCommand(CMD_SDATAC);
    if (ret) {
        return ret;
    }
    k_sleep(K_MSEC(2));

    /*
     * Optional but clean: stop conversions while configuring.
     */
    ret = sendCommand(CMD_STOP);
    if (ret) {
        return ret;
    }
    k_sleep(K_MSEC(2));

    /*
    * CONFIG1 = 0x96:
    *   bit7 = 1 reserved
    *   DAISY_EN = 0 -> daisy-chain mode according to datasheet table
    *   CLK_EN = 0 -> do not output internal oscillator
    *   bits4:3 = 10 reserved
    *   DR[2:0] = 110 -> 250 SPS
    */
    ret = writeRegister(REG_CONFIG1, 0x96);
    if (ret) {
        return ret;
    }

    /*
     * CONFIG2 = 0xD0:
     *   INT_CAL = 1 -> internally generated test signal
     *   CAL_AMP = 0 -> 1x test amplitude
     *   CAL_FREQ = 00 -> fCLK / 2^21 pulsed signal [13], [28].
     */
    ret = writeRegister(REG_CONFIG2, 0xD0);
    if (ret) {
        return ret;
    }

    /*
     * CONFIG3 = 0xE0:
     *   PD_REFBUF = 1 -> enable internal reference buffer
     *   reserved bits6:5 = 11
     *   BIASREF_INT = 0 in strict bit decoding for 0xE0.
     *
     * Note: the user-requested value is 0xE0. In the ADS1299 register map,
     * BIASREF_INT is bit 3, so setting BIASREF_INT = 1 would make this 0xE8
     * if no other bits change [21], [28]. The datasheet's test-signal flow
     * explicitly shows WREG CONFIG3 E0h when using the internal reference [6],
     * so this code writes 0xE0 as requested.
     */
    ret = writeRegister(REG_CONFIG3, 0xE0);
    if (ret) {
        return ret;
    }

    ret = writeRegisters(REG_CH1SET, chset_selected, sizeof(chset_selected));
    if (ret) {
        return ret;
    }

    k_sleep(K_MSEC(2));

    printk("ADS1299 internal test signal registers written\n");
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
