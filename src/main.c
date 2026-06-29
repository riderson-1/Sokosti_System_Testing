/* Simple ADS1299 ID read test for Sokosti nRF5340 board
 * - hardware clock on P0.15 using TIMER+PPI+GPIOTE, 2 MHz
 * - SPI transfers to read ID register from two ADS1299 chips
 * - toggles LED every loop iteration
 */
#include <zephyr/kernel.h>
#include <zephyr/device.h>
#include <zephyr/devicetree.h>
#include <zephyr/drivers/gpio.h>
#include <zephyr/drivers/spi.h>

#include <nrfx.h>
#include <nrfx_timer.h>
#include <nrfx_gpiote.h>
#include <helpers/nrfx_gppi.h>

/* Devicetree nodes */
#define SPI_NODE DT_NODELABEL(spi4)
#define CLK_NODE DT_NODELABEL(clk_ads)
#define RESET_NODE DT_NODELABEL(reset_ads)

/* LED alias */
#define LED1_NODE DT_ALIAS(led0)

/* ADS1299 commands */
#define ADS_RREG 0x20
#define ADS_ID_REG 0x00

#define LOOP_DELAY_MS 1000

static const struct gpio_dt_spec led = GPIO_DT_SPEC_GET(LED1_NODE, gpios);

/* Timer and GPIOTE instances for generating a 2 MHz clock */
static nrfx_timer_t timer_inst = NRFX_TIMER_INSTANCE(NRF_TIMER_INST_GET(0));
static nrfx_gpiote_t gpiote_inst = NRFX_GPIOTE_INSTANCE(NRF_GPIOTE_INST_GET(0));
static nrfx_gppi_t gppi_inst;

static void setup_clock_2mhz(uint32_t pin)
{
    int rc;
    uint8_t out_channel;
    const uint32_t timer_freq = 16000000UL;
    const uint32_t desired_out = 2000000UL;

    /*
     * Toggle output every compare event.
     * Full output period needs two toggles.
     *
     * 16 MHz / (2 * 2 MHz) = 4 timer ticks
     */
    const uint32_t cc = timer_freq / (2 * desired_out);

    if (pin > 31) {
        printk("Invalid pin for clock: %u\n", pin);
        return;
    }

    rc = nrfx_gpiote_init(&gpiote_inst, NRFX_GPIOTE_DEFAULT_CONFIG_IRQ_PRIORITY);
    if (rc != 0) {
        printk("GPIOTE init failed: %d\n", rc);
        return;
    }

    rc = nrfx_gpiote_channel_alloc(&gpiote_inst, &out_channel);
    if (rc != 0) {
        printk("GPIOTE channel alloc failed: %d\n", rc);
        return;
    }

    static const nrfx_gpiote_output_config_t out_cfg = NRFX_GPIOTE_DEFAULT_OUTPUT_CONFIG;
    const nrfx_gpiote_task_config_t task_cfg = {
        .task_ch = out_channel,
        .polarity = NRF_GPIOTE_POLARITY_TOGGLE,
        .init_val = NRF_GPIOTE_INITIAL_VALUE_LOW,
    };

    rc = nrfx_gpiote_output_configure(&gpiote_inst, pin, &out_cfg, &task_cfg);
    if (rc != 0) {
        printk("GPIOTE output configure failed: %d\n", rc);
        return;
    }

    nrfx_gpiote_out_task_enable(&gpiote_inst, pin);

    const nrfx_timer_config_t tcfg = NRFX_TIMER_DEFAULT_CONFIG(timer_freq);

    rc = nrfx_timer_init(&timer_inst, &tcfg, NULL);
    if (rc != 0) {
        printk("TIMER init failed: %d\n", rc);
        return;
    }

    nrfx_timer_clear(&timer_inst);

    /*
     * IMPORTANT:
     * Use extended compare with CLEAR shortcut.
     * Without this, the compare event happens only once.
     */
    nrfx_timer_extended_compare(
        &timer_inst,
        NRF_TIMER_CC_CHANNEL0,
        cc,
        NRF_TIMER_SHORT_COMPARE0_CLEAR_MASK,
        false
    );

    nrfx_gppi_init(&gppi_inst);

    uint32_t evt = nrfx_timer_compare_event_address_get(
        &timer_inst,
        NRF_TIMER_CC_CHANNEL0
    );

    uint32_t task = nrfx_gpiote_out_task_address_get(
        &gpiote_inst,
        pin
    );

    nrfx_gppi_handle_t gppi_h;
    rc = nrfx_gppi_conn_alloc(evt, task, &gppi_h);
    if (rc != 0) {
        printk("GPPI conn alloc failed: %d\n", rc);
        return;
    }

    nrfx_gppi_conn_enable(gppi_h);
    nrfx_timer_enable(&timer_inst);

    printk("2 MHz clock enabled on pin %u, CC=%u\n", pin, cc);
}

static int spi_read_id_single(
    const struct device *spi_dev,
    struct spi_config *cfg,
    const struct gpio_dt_spec *cs,
    uint8_t *id
)
{
    uint8_t cmd[3] = { ADS_RREG | ADS_ID_REG, 0x00 };
    struct spi_buf tx_bufs[2] = {
        { .buf = cmd, .len = sizeof(cmd) }
    };
    const struct spi_buf_set tx = {
        .buffers = tx_bufs,
        .count = 1
    };

    uint8_t rx = 0;
    struct spi_buf rx_bufs[2] = {
        { .buf = &rx, .len = 1 }
    };
    const struct spi_buf_set rx_set = {
        .buffers = rx_bufs,
        .count = 1
    };

    gpio_pin_set_dt(cs, 0);
    int rc = spi_write(spi_dev, cfg, &tx);
    if (rc) {
        gpio_pin_set_dt(cs, 1);
        return rc;
    }
    rc = spi_read(spi_dev, cfg, &rx_set);
    gpio_pin_set_dt(cs, 1);

    if (!rc) {
        *id = rx;
    }
    return rc;
}

static int spi_read_id_daisy(
    const struct device *spi_dev,
    struct spi_config *cfg,
    const struct gpio_dt_spec *cs,
    uint8_t *ids,
    size_t n
)
{
    if (n > 8) {
        return -EINVAL;
    }

    uint8_t cmd[3] = { ADS_RREG | ADS_ID_REG, 0x00 };
    struct spi_buf tx_cmd = {
        .buf = cmd,
        .len = sizeof(cmd)
    };
    const struct spi_buf_set tx = {
        .buffers = &tx_cmd,
        .count = 1
    };

    uint8_t rxbuf[5] = {0};
    struct spi_buf rxb = {
        .buf = rxbuf,
        .len = n
    };
    const struct spi_buf_set rx = {
        .buffers = &rxb,
        .count = 1
    };

    gpio_pin_set_dt(cs, 0);
    int rc = spi_write(spi_dev, cfg, &tx);
    if (rc) {
        gpio_pin_set_dt(cs, 1);
        return rc;
    }
    rc = spi_read(spi_dev, cfg, &rx);
    gpio_pin_set_dt(cs, 1);

    if (!rc) {
        for (size_t i = 0; i < n; i++) {
            ids[i] = rxbuf[i];
        }
    }
    return rc;
}

int main(void)
{
    int ret;
    printk("ADS1299 SPI ID read test starting...\n");

    const struct device *spi_dev = DEVICE_DT_GET(SPI_NODE);
    if (!device_is_ready(spi_dev)) {
        printk("SPI device not ready\n");
        return -1;
    }

    if (!device_is_ready(led.port)) {
        printk("LED GPIO device not ready\n");
        return -1;
    }

    ret = gpio_pin_configure_dt(&led, GPIO_OUTPUT_INACTIVE);
    if (ret < 0) {
        printk("LED configure failed: %d\n", ret);
        return -1;
    }

    const struct gpio_dt_spec clk = GPIO_DT_SPEC_GET_OR(CLK_NODE, gpios, {0});
    const struct gpio_dt_spec cs1 = GPIO_DT_SPEC_GET_BY_IDX(SPI_NODE, cs_gpios, 0);
#if DT_PROP_LEN(SPI_NODE, cs_gpios) > 1
    const struct gpio_dt_spec cs2 = GPIO_DT_SPEC_GET_BY_IDX(SPI_NODE, cs_gpios, 1);
#endif

    if (!gpio_is_ready_dt(&cs1)) {
        printk("Warning: CS1 GPIO not ready or missing\n");
    } else {
        ret = gpio_pin_configure_dt(&cs1, GPIO_OUTPUT_INACTIVE);
        if (ret < 0) {
            printk("CS1 configure failed: %d\n", ret);
        }
        gpio_pin_set_dt(&cs1, 1);
    }

#if DT_PROP_LEN(SPI_NODE, cs_gpios) > 1
    if (!gpio_is_ready_dt(&cs2)) {
        printk("Warning: CS2 GPIO not ready\n");
    } else {
        ret = gpio_pin_configure_dt(&cs2, GPIO_OUTPUT_INACTIVE);
        if (ret < 0) {
            printk("CS2 configure failed: %d\n", ret);
        }
        gpio_pin_set_dt(&cs2, 1);
    }
#else
    printk("Only one CS entry found in devicetree. Daisy-chain mode used.\n");
#endif

    if (DT_NODE_HAS_STATUS(CLK_NODE, okay) && gpio_is_ready_dt(&clk)) {
        setup_clock_2mhz(clk.pin);
    } else {
        printk("clk_ads node missing or not ready. Clock not started.\n");
    }

    struct spi_config spi_cfg = {
        .frequency = 1000000U,
        .operation = SPI_WORD_SET(8) | SPI_TRANSFER_MSB,
        .slave = 0,
        .cs = {
            .gpio = {0},
            .delay = 0
        },
    };

    printk("Reading ADS1299 IDs every second...\n");

    uint32_t loop_count = 0;
    while (1) {
        loop_count++;
        gpio_pin_toggle_dt(&led);
        printk("\n--- Loop %u at %lld ms ---\n", loop_count, k_uptime_get());

        if (gpio_is_ready_dt(&cs1)) {
            uint8_t id1 = 0;
            ret = spi_read_id_single(spi_dev, &spi_cfg, &cs1, &id1);
            if (ret == 0) {
                printk("ADS1 ID: 0x%02x\n", id1);
            } else {
                printk("ADS1 read failed: %d\n", ret);
            }
        }

#if DT_PROP_LEN(SPI_NODE, cs_gpios) > 1
        if (gpio_is_ready_dt(&cs2)) {
            uint8_t id2 = 0;
            ret = spi_read_id_single(spi_dev, &spi_cfg, &cs2, &id2);
            if (ret == 0) {
                printk("ADS2 ID: 0x%02x\n", id2);
            } else {
                printk("ADS2 read failed: %d\n", ret);
            }
        }
#else
        if (gpio_is_ready_dt(&cs1)) {
            uint8_t ids[3] = {0};
            ret = spi_read_id_daisy(spi_dev, &spi_cfg, &cs1, ids, 2);
            if (ret == 0) {
                printk("Daisy IDs: 0x%02x 0x%02x\n", ids[1], ids[2]);
            } else {
                printk("Daisy read failed: %d\n", ret);
            }
        }
#endif

        k_msleep(LOOP_DELAY_MS);
    }

    return 0;
}
