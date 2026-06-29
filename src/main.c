#include <zephyr/kernel.h>
#include <zephyr/device.h>
#include <zephyr/devicetree.h>
#include <zephyr/drivers/gpio.h>
#include <zephyr/drivers/spi.h>
#include <zephyr/drivers/pwm.h>
#include <zephyr/sys/printk.h>

#include <errno.h>
#include <stdint.h>
#include <stddef.h>

#define SPI_NODE        DT_NODELABEL(spi4)
#define ADS_NODE        DT_NODELABEL(ads1299)
#define LED_NODE        DT_ALIAS(led0)
#define RESET_NODE      DT_NODELABEL(reset_ads)
#define START_NODE      DT_NODELABEL(global_start)
#define ADS_CLK_NODE    DT_ALIAS(adsclk)

#define CMD_RESET       0x06
#define CMD_SDATAC      0x11
#define CMD_RREG        0x20

#define REG_ID          0x00
#define REG_CONFIG1     0x01

#define LOOP_DELAY_MS   1000

static const struct device *spi_dev = DEVICE_DT_GET(SPI_NODE);

static const struct gpio_dt_spec led =
    GPIO_DT_SPEC_GET(LED_NODE, gpios);

static const struct gpio_dt_spec reset_pin =
    GPIO_DT_SPEC_GET(RESET_NODE, gpios);

static const struct gpio_dt_spec start_pin =
    GPIO_DT_SPEC_GET(START_NODE, gpios);

static const struct gpio_dt_spec cs_pin =
    GPIO_DT_SPEC_GET_BY_IDX(SPI_NODE, cs_gpios, 0);

static const struct pwm_dt_spec ads_clk_pwm =
    PWM_DT_SPEC_GET(ADS_CLK_NODE);

static struct spi_cs_control ads_cs;
static struct spi_config ads_spi_cfg;

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

    return spi_write(spi_dev, &ads_spi_cfg, &txs);
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

    return spi_transceive(spi_dev, &ads_spi_cfg, &txs, &rxs);
}

static int ads_send_cmd(uint8_t cmd)
{
    int ret = ads_spi_write_bytes(&cmd, 1);
    k_busy_wait(10);
    return ret;
}

static int ads_read_reg(uint8_t reg, uint8_t *value)
{
    uint8_t tx[3] = {
        (uint8_t)(CMD_RREG | reg),
        0x00,
        0x00,
    };

    uint8_t rx[3] = { 0 };

    int ret = ads_spi_transceive_bytes(tx, rx, sizeof(tx));
    if (ret == 0) {
        *value = rx[2];
    }

    return ret;
}

static int start_ads_pwm_clock(void)
{
    if (!pwm_is_ready_dt(&ads_clk_pwm)) {
        printk("ADS clock PWM not ready\n");
        return -ENODEV;
    }

    int ret = pwm_set_dt(&ads_clk_pwm, PWM_NSEC(500), PWM_NSEC(250));;
    if (ret) {
        printk("PWM start failed: %d\n", ret);
        return ret;
    }

    printk("ADS clock PWM started\n");
    return 0;
}

static int setup_spi(void)
{
    if (!device_is_ready(spi_dev)) {
        printk("SPI device not ready\n");
        return -ENODEV;
    }

    if (!gpio_is_ready_dt(&cs_pin)) {
        printk("CS GPIO not ready\n");
        return -ENODEV;
    }

    ads_cs.gpio = cs_pin;
    ads_cs.delay = 2;

    ads_spi_cfg.frequency = 1000000U;
    ads_spi_cfg.operation = SPI_WORD_SET(8) | SPI_TRANSFER_MSB | SPI_MODE_CPHA;
    ads_spi_cfg.slave = DT_REG_ADDR(ADS_NODE);
    ads_spi_cfg.cs = ads_cs;

    return 0;
}

static int setup_gpios(void)
{
    int ret;

    if (!gpio_is_ready_dt(&led)) {
        printk("LED GPIO not ready\n");
        return -ENODEV;
    }

    if (!gpio_is_ready_dt(&reset_pin)) {
        printk("RESET GPIO not ready\n");
        return -ENODEV;
    }

    if (!gpio_is_ready_dt(&start_pin)) {
        printk("START GPIO not ready\n");
        return -ENODEV;
    }

    ret = gpio_pin_configure_dt(&led, GPIO_OUTPUT_INACTIVE);
    if (ret) {
        printk("LED configure failed: %d\n", ret);
        return ret;
    }

    ret = gpio_pin_configure_dt(&reset_pin, GPIO_OUTPUT_INACTIVE);
    if (ret) {
        printk("RESET configure failed: %d\n", ret);
        return ret;
    }

    ret = gpio_pin_configure_dt(&start_pin, GPIO_OUTPUT_INACTIVE);
    if (ret) {
        printk("START configure failed: %d\n", ret);
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

    ret = ads_send_cmd(CMD_SDATAC);
    if (ret) {
        return ret;
    }

    k_sleep(K_MSEC(2));

    return ads_read_reg(REG_ID, id);
}

int main(void)
{
    int ret;
    uint32_t loop_count = 0;

    printk("ADS1299 SPI ID read test starting...\n");

    k_sleep(K_MSEC(500));

    ret = setup_spi();
    if (ret) {
        return 0;
    }

    ret = setup_gpios();
    if (ret) {
        return 0;
    }

    gpio_pin_set_dt(&start_pin, 0);

    ret = start_ads_pwm_clock();
    if (ret) {
        printk("Warning: ADS clock not running\n");
    }

    k_sleep(K_MSEC(20));

    ads_hw_reset();

    ret = ads_send_cmd(CMD_RESET);
    if (ret) {
        printk("CMD_RESET failed: %d\n", ret);
    } else {
        k_sleep(K_MSEC(10));
    }

    printk("Reading ADS1299 ID every second...\n");

    while (1) {
        uint8_t id = 0;
        uint8_t config1 = 0;

        loop_count++;
        gpio_pin_toggle_dt(&led);

        printk("\n--- Loop %u at %lld ms ---\n",
               loop_count,
               k_uptime_get());

        ret = ads_read_id(&id);
        if (ret) {
            printk("ID read failed: %d\n", ret);
        } else {
            printk("ADS1299 ID: 0x%02X\n", id);
        }

        ret = ads_read_reg(REG_CONFIG1, &config1);
        if (ret) {
            printk("CONFIG1 read failed: %d\n", ret);
        } else {
            printk("CONFIG1: 0x%02X\n", config1);
        }

        k_msleep(LOOP_DELAY_MS);
    }

    return 0;
}
