#include <zephyr/kernel.h>
#include <zephyr/device.h>
#include <zephyr/devicetree.h>
#include <zephyr/drivers/gpio.h>
#include <zephyr/drivers/spi.h>
#include <zephyr/sys/printk.h>

#include <nrfx.h>
#include <nrfx_timer.h>
#include <nrfx_gpiote.h>
#include <helpers/nrfx_gppi.h>

#define SPI_NODE     DT_NODELABEL(spi4)
#define ADS_NODE     DT_NODELABEL(ads1299)
#define CLK_NODE     DT_NODELABEL(clk_ads)
#define RESET_NODE   DT_NODELABEL(reset_ads)
#define START_NODE   DT_NODELABEL(global_start)
#define LED_NODE     DT_ALIAS(led0)

#define ADS_SDATAC   0x11
#define ADS_RESET    0x06
#define ADS_RREG     0x20
#define ADS_ID       0x00

static const struct gpio_dt_spec led =
    GPIO_DT_SPEC_GET(LED_NODE, gpios);

static const struct gpio_dt_spec clk =
    GPIO_DT_SPEC_GET(CLK_NODE, gpios);

static const struct gpio_dt_spec reset =
    GPIO_DT_SPEC_GET(RESET_NODE, gpios);

static const struct gpio_dt_spec start =
    GPIO_DT_SPEC_GET(START_NODE, gpios);

static const struct gpio_dt_spec cs =
    GPIO_DT_SPEC_GET_BY_IDX(SPI_NODE, cs_gpios, 0);

static const struct device *spi_dev = DEVICE_DT_GET(SPI_NODE);

static nrfx_timer_t timer = NRFX_TIMER_INSTANCE(NRF_TIMER_INST_GET(2));
static nrfx_gpiote_t gpiote = NRFX_GPIOTE_INSTANCE(NRF_GPIOTE_INST_GET(0));

static nrfx_gppi_t gppi;

static struct spi_config spi_cfg = {
    .frequency = 1000000,
    .operation = SPI_WORD_SET(8) | SPI_TRANSFER_MSB | SPI_MODE_CPHA,
    .slave = 0,
};

static void cs_low(void)
{
    gpio_pin_set_dt(&cs, 1);
}

static void cs_high(void)
{
    gpio_pin_set_dt(&cs, 0);
}

static void ads_clk_start_2mhz(void)
{
    int r;
    uint8_t gpiote_ch;
    nrfx_gppi_handle_t gppi_ch;

    /*
     * TIMER = 16 MHz.
     * Toggle every 4 ticks.
     * Full period = 8 ticks.
     * 16 MHz / 8 = 2 MHz.
     */
    const uint32_t cc = 4;

    r = nrfx_gpiote_init(&gpiote, 6);
    printk("100 %d\n", r);

    r = nrfx_gpiote_channel_alloc(&gpiote, &gpiote_ch);
    printk("101 %d\n", r);

    nrfx_gpiote_output_config_t out_cfg =
        NRFX_GPIOTE_DEFAULT_OUTPUT_CONFIG;

    nrfx_gpiote_task_config_t task_cfg = {
        .task_ch = gpiote_ch,
        .polarity = NRF_GPIOTE_POLARITY_TOGGLE,
        .init_val = NRF_GPIOTE_INITIAL_VALUE_LOW,
    };

    r = nrfx_gpiote_output_configure(&gpiote, clk.pin, &out_cfg, &task_cfg);
    printk("102 %d\n", r);

    nrfx_gpiote_out_task_enable(&gpiote, clk.pin);

    nrfx_timer_config_t tcfg = NRFX_TIMER_DEFAULT_CONFIG(16000000);
    tcfg.bit_width = NRF_TIMER_BIT_WIDTH_32;

    r = nrfx_timer_init(&timer, &tcfg, NULL);
    printk("103 %d\n", r);

    nrfx_timer_clear(&timer);

    nrfx_timer_extended_compare(
        &timer,
        NRF_TIMER_CC_CHANNEL0,
        cc,
        NRF_TIMER_SHORT_COMPARE0_CLEAR_MASK,
        false
    );

    nrfx_gppi_init(&gppi);
    printk("104 %d\n", r);

    uint32_t evt = nrfx_timer_compare_event_address_get(
        &timer,
        NRF_TIMER_CC_CHANNEL0
    );

    uint32_t task = nrfx_gpiote_out_task_address_get(
        &gpiote,
        clk.pin
    );

    r = nrfx_gppi_conn_alloc(evt, task, &gppi_ch);
    printk("105 %d\n", r);

    nrfx_gppi_conn_enable(gppi_ch);
    nrfx_timer_enable(&timer);

    printk("106 %u\n", clk.pin);
}

static int ads_cmd(uint8_t cmd)
{
    struct spi_buf b = {
        .buf = &cmd,
        .len = 1,
    };

    struct spi_buf_set s = {
        .buffers = &b,
        .count = 1,
    };

    cs_low();
    int r = spi_write(spi_dev, &spi_cfg, &s);
    cs_high();

    k_busy_wait(10);

    return r;
}

static int ads_read_reg(uint8_t reg, uint8_t *val)
{
    uint8_t tx[3] = {
        (uint8_t)(ADS_RREG | reg),
        0x00,
        0x00,
    };

    uint8_t rx[3] = {0};

    struct spi_buf txb = {
        .buf = tx,
        .len = sizeof(tx),
    };

    struct spi_buf rxb = {
        .buf = rx,
        .len = sizeof(rx),
    };

    struct spi_buf_set txs = {
        .buffers = &txb,
        .count = 1,
    };

    struct spi_buf_set rxs = {
        .buffers = &rxb,
        .count = 1,
    };

    cs_low();
    int r = spi_transceive(spi_dev, &spi_cfg, &txs, &rxs);
    cs_high();

    k_busy_wait(10);

    *val = rx[2];

    return r;
}

static void ads_hw_reset(void)
{
    /*
     * reset_ads is GPIO_ACTIVE_LOW.
     * logical 1 = physical low = reset active.
     * logical 0 = physical high = reset inactive.
     */
    gpio_pin_set_dt(&reset, 0);
    k_msleep(10);

    gpio_pin_set_dt(&reset, 1);
    k_msleep(2);

    gpio_pin_set_dt(&reset, 0);
    k_msleep(50);
}

int main(void)
{
    int r;
    uint8_t id;
    uint32_t loop = 0;

    printk("0\n");

    r = gpio_pin_configure_dt(&led, GPIO_OUTPUT_INACTIVE);
    printk("1 %d\n", r);

    r = gpio_pin_configure_dt(&cs, GPIO_OUTPUT_INACTIVE);
    printk("2 %d\n", r);

    r = gpio_pin_configure_dt(&reset, GPIO_OUTPUT_INACTIVE);
    printk("3 %d\n", r);

    r = gpio_pin_configure_dt(&start, GPIO_OUTPUT_INACTIVE);
    printk("4 %d\n", r);

    cs_high();

    /*
     * Keep START low. For ID read and register access we do not need
     * conversions running.
     */
    gpio_pin_set_dt(&start, 0);

    /*
     * Start external clock before ADS reset.
     */
    ads_clk_start_2mhz();

    k_msleep(20);

    ads_hw_reset();

    r = ads_cmd(ADS_SDATAC);
    printk("10 %d\n", r);
    k_msleep(2);

    r = ads_cmd(ADS_RESET);
    printk("11 %d\n", r);
    k_msleep(10);

    r = ads_cmd(ADS_SDATAC);
    printk("12 %d\n", r);
    k_msleep(2);

    while (1) {
        id = 0;

        r = ads_read_reg(ADS_ID, &id);

        gpio_pin_toggle_dt(&led);

        printk("20 %u %d 0x%02x\n", loop, r, id);

        loop++;
        k_msleep(1000);
    }
}
