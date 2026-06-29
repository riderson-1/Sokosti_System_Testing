#include <zephyr/kernel.h>
#include <zephyr/sys/printk.h>
#include <zephyr/drivers/spi.h>
#include <zephyr/drivers/gpio.h>
#include <zephyr/drivers/pwm.h>
#include <zephyr/devicetree.h>
#include <errno.h>
#include <stdint.h>
#include <stddef.h>
#include <string.h>

#define CMD_WAKEUP   0x02
#define CMD_STANDBY  0x04
#define CMD_RESET    0x06
#define CMD_START    0x08
#define CMD_STOP     0x0A
#define CMD_RDATAC   0x10
#define CMD_SDATAC   0x11
#define CMD_RDATA    0x12
#define CMD_RREG     0x20
#define CMD_WREG     0x40

#define REG_ID       0x00
#define REG_CONFIG1  0x01
#define REG_CONFIG2  0x02
#define REG_CONFIG3  0x03
#define REG_LOFF     0x04
#define REG_CH1SET   0x05
#define REG_CH2SET   0x06
#define REG_CH3SET   0x07
#define REG_CH4SET   0x08
#define REG_CH5SET   0x09
#define REG_CH6SET   0x0A
#define REG_CH7SET   0x0B
#define REG_CH8SET   0x0C

#define ADS_NUM_CHANNELS 8
#define ADS_FRAME_BYTES  (3 + ADS_NUM_CHANNELS * 3)

#define LOOP_DELAY_MS 1000

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

static int ads_spi_write_bytes(const uint8_t *data, size_t len)
{
    struct spi_buf txb = {
        .buf = (void *)data,
        .len = len,
    };
    struct spi_buf_set txs = {
        .buffers = &txb,
        .count = 1,
    };

    return spi_write_dt(&ads_spi, &txs);
}

static int ads_spi_transceive_bytes(const uint8_t *tx_data,
                    uint8_t *rx_data,
                    size_t len)
{
    struct spi_buf txb = {
        .buf = (void *)tx_data,
        .len = len,
    };
    struct spi_buf_set txs = {
        .buffers = &txb,
        .count = 1,
    };

    struct spi_buf rxb = {
        .buf = rx_data,
        .len = len,
    };
    struct spi_buf_set rxs = {
        .buffers = &rxb,
        .count = 1,
    };

    return spi_transceive_dt(&ads_spi, &txs, &rxs);
}

static int ads_send_cmd(uint8_t cmd)
{
    int ret = ads_spi_write_bytes(&cmd, 1);

    /*
     * ADS1299 commands require decode time. The datasheet says commands
     * require 4 tCLK cycles to decode/execute [17].
     */
    k_busy_wait(10);

    return ret;
}

static int ads_read_reg(uint8_t reg, uint8_t *value)
{
    uint8_t tx[4] = {
        (uint8_t)(CMD_RREG | reg),
        0x00,
        0x00,
    };
    uint8_t rx[4] = { 0 };

    int ret = ads_spi_transceive_bytes(tx, rx, sizeof(tx));
    if (ret == 0) {
        *value = rx[3];
    }

    return ret;
}

static int ads_write_reg(uint8_t reg, uint8_t value)
{
    uint8_t tx[4] = {
        (uint8_t)(CMD_WREG | reg),
        0x00,
        value,
    };

    int ret = ads_spi_write_bytes(tx, sizeof(tx));
    k_busy_wait(10);
    return ret;
}

static int ads_write_regs(uint8_t start_reg, const uint8_t *values, size_t count)
{
    uint8_t tx[3 + ADS_NUM_CHANNELS];   /* cmd + count_byte + up to 8 data bytes */

    if (count == 0 || count > ADS_NUM_CHANNELS) {
        return -EINVAL;
    }

    tx[0] = (uint8_t)(CMD_WREG | start_reg);
    tx[1] = (uint8_t)(count - 1);
    memcpy(&tx[2], values, count);

    int ret = ads_spi_write_bytes(tx, 2 + count);
    k_busy_wait(10);
    return ret;
}

static int ads_wait_drdy_low_timeout_ms(int timeout_ms)
{
    int loops = timeout_ms * 10;

    for (int i = 0; i < loops; i++) {
        int val = gpio_pin_get_dt(&drdy_pin);

        if (val < 0) {
            return val;
        }

        if (val == 0) {
            return 0;
        }

        k_busy_wait(100);
    }

    return -ETIMEDOUT;
}

static int32_t ads_decode24(const uint8_t *p)
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
static int ads_read_frame_rdatac(uint8_t frame[ADS_FRAME_BYTES])
{
    uint8_t tx[ADS_FRAME_BYTES] = { 0 };
    uint8_t rx[ADS_FRAME_BYTES] = { 0 };

    int ret = ads_spi_transceive_bytes(tx, rx, sizeof(tx));
    if (ret) {
        return ret;
    }

    memcpy(frame, rx, ADS_FRAME_BYTES);
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

static void ads_hw_reset(void)
{
    gpio_pin_set_dt(&reset_pin, 1);
    k_sleep(K_MSEC(10));

    gpio_pin_set_dt(&reset_pin, 0);
    k_sleep(K_MSEC(50));
}

static int ads_read_id(uint8_t *id)
{
    int ret;

    /*
     * Device defaults to RDATAC after power-up; stop continuous mode before
     * register reads/writes [25].
     */
    ret = ads_send_cmd(CMD_SDATAC);
    if (ret) {
        return ret;
    }

    k_sleep(K_MSEC(2));

    return ads_read_reg(REG_ID, id);
}

static int ads_dump_test_regs(void)
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
        ret = ads_read_reg(regs[i], &v);
        if (ret) {
            printk("REG 0x%02X read failed: %d\n", regs[i], ret);
            return ret;
        }

        printk("REG 0x%02X = 0x%02X\n", regs[i], v);
    }

    return 0;
}

static int ads_configure_internal_test_signal(void)
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

    printk("Configuring ADS1299 internal test signal mode...\n");

    /*
     * Stop continuous data mode before writing registers. RDATAC is the
     * default after power-up and is cancelled with SDATAC [25].
     */
    ret = ads_send_cmd(CMD_SDATAC);
    if (ret) {
        return ret;
    }
    k_sleep(K_MSEC(2));

    /*
     * Optional but clean: stop conversions while configuring.
     */
    ret = ads_send_cmd(CMD_STOP);
    if (ret) {
        return ret;
    }
    k_sleep(K_MSEC(2));

    /*
     * CONFIG1 = 0x96:
     *   Reserved bit7 = 1
     *   DAISY_EN = 0 / standalone-style setting requested
     *   CLK_EN = 0
     *   Reserved bits4:3 = 10
     *   DR[2:0] = 110 -> 250 SPS [10], [13].
     */
    ret = ads_write_reg(REG_CONFIG1, 0x96);
    if (ret) {
        return ret;
    }

    /*
     * CONFIG2 = 0xD0:
     *   INT_CAL = 1 -> internally generated test signal
     *   CAL_AMP = 0 -> 1x test amplitude
     *   CAL_FREQ = 00 -> fCLK / 2^21 pulsed signal [13], [28].
     */
    ret = ads_write_reg(REG_CONFIG2, 0xD0);
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
    ret = ads_write_reg(REG_CONFIG3, 0xE0);
    if (ret) {
        return ret;
    }

    ret = ads_write_regs(REG_CH1SET, chset_all, sizeof(chset_all));
    if (ret) {
        return ret;
    }

    k_sleep(K_MSEC(2));

    printk("ADS1299 internal test signal registers written\n");
    return 0;
}

int main(void)
{
    int ret;
    uint8_t id = 0;
    uint8_t frame[ADS_FRAME_BYTES];
    uint32_t sample_idx = 0;

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

    /*
     * Keep START pin low and use the SPI START command.
     * Datasheet says when using START command, hold START pin low [9], [27].
     */
    gpio_pin_set_dt(&start_pin, 0);

    ret = start_ads_pwm_clock();
    if (ret) {
        printk("Warning: ADS clock not running: %d\n", ret);
    }

    k_sleep(K_MSEC(20));

    ads_hw_reset();

    ret = ads_send_cmd(CMD_RESET);
    if (ret) {
        printk("CMD_RESET failed: %d\n", ret);
        return 0;
    }

    /*
     * RESET command requires 18 tCLK cycles; wait longer than required [8], [26].
     */
    k_sleep(K_MSEC(10));

    ret = ads_read_id(&id);
    if (ret) {
        printk("ADS1299 ID read failed: %d\n", ret);
        return 0;
    }

    printk("ADS1299 ID: 0x%02X\n", id);

    ret = ads_configure_internal_test_signal();
    if (ret) {
        printk("ADS test signal configuration failed: %d\n", ret);
        return 0;
    }

    /*
     * Dump registers before RDATAC. Register access should be done outside
     * RDATAC mode.
     */
    ret = ads_dump_test_regs();
    if (ret) {
        printk("ADS register dump failed: %d\n", ret);
        return 0;
    }

    /*
     * Put device back into Read Data Continuous mode, then start conversions.
     * RDATAC streams data after DRDY when START pin is high or START command
     * has been sent [24].
     */
    ret = ads_send_cmd(CMD_RDATAC);
    if (ret) {
        printk("RDATAC failed: %d\n", ret);
        return 0;
    }

    k_sleep(K_MSEC(2));

    ret = ads_send_cmd(CMD_START);
    if (ret) {
        printk("START command failed: %d\n", ret);
        return 0;
    }

    printk("Streaming internal test signal. Expect ~1 Hz square wave.\n");
    printk("Polling DRDY and reading %d-byte frames.\n", ADS_FRAME_BYTES);

    while (1) {
        ret = ads_wait_drdy_low_timeout_ms(1000);
        if (ret) {
            printk("DRDY timeout/error: %d\n", ret);
            gpio_pin_toggle_dt(&led);
            continue;
        }

        ret = ads_read_frame_rdatac(frame);
        if (ret) {
            printk("Frame read failed: %d\n", ret);
            gpio_pin_toggle_dt(&led);
            continue;
        }

        /*
         * Status format begins with 1100 for ADS1299 data frames [22].
         */
        bool status_header_ok = ((frame[0] & 0xF0) == 0xC0);

        int32_t ch1 = ads_decode24(&frame[3]);
        int32_t ch2 = ads_decode24(&frame[6]);
        int32_t ch3 = ads_decode24(&frame[9]);
        int32_t ch4 = ads_decode24(&frame[12]);
        int32_t ch5 = ads_decode24(&frame[15]);
        int32_t ch6 = ads_decode24(&frame[18]);
        int32_t ch7 = ads_decode24(&frame[21]);
        int32_t ch8 = ads_decode24(&frame[24]);

        if ((sample_idx % 25U) == 0U) {
            gpio_pin_toggle_dt(&led);

            printk("sample=%lu status=%02X%02X%02X header=%s "
                   "ch1=%ld ch2=%ld ch3=%ld ch4=%ld "
                   "ch5=%ld ch6=%ld ch7=%ld ch8=%ld\n",
                   (unsigned long)sample_idx,
                   frame[1], frame[2], frame[3],
                   status_header_ok ? "OK" : "BAD",
                   (long)ch1, (long)ch2, (long)ch3, (long)ch4,
                   (long)ch5, (long)ch6, (long)ch7, (long)ch8);
        }

        sample_idx++;
    }

    return 0;
}
