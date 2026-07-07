/**
 * @file main.c
 * @author your name (you@domain.com)
 * @brief 
 * @version 0.1
 * @date 2026-07-07
 * 
 * @copyright Copyright (c) 2026
 * 
 */

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

#include <stdio.h>
#include <zephyr/device.h>
#include <zephyr/drivers/uart.h>
#include <zephyr/logging/log.h>
#include <zephyr/usb/usbd.h>

#include "sample_usbd.h"
#include "ads1299_definitions.h"
#include "ads1299_driver.h"

LOG_MODULE_REGISTER(cdc_acm_print, LOG_LEVEL_INF);

#define LOOP_DELAY_MS 1000

#define ADS_NODE        DT_NODELABEL(ads1299)
#define LED_NODE        DT_ALIAS(led0)
#define RESET_NODE      DT_NODELABEL(reset_ads)
#define START_NODE      DT_NODELABEL(global_start)
#define DRDY_NODE       DT_NODELABEL(drdy_ads)
#define ADS_CLK_NODE    DT_ALIAS(adsclk)

const struct device *const uart_dev = DEVICE_DT_GET_ONE(zephyr_cdc_acm_uart);

static struct usbd_context *sample_usbd;
K_SEM_DEFINE(dtr_sem, 0, 1);

static void sample_msg_cb(struct usbd_context *const ctx, const struct usbd_msg *msg)
{
	LOG_INF("USBD message: %s", usbd_msg_type_string(msg->type));

	if (usbd_can_detect_vbus(ctx)) {
		if (msg->type == USBD_MSG_VBUS_READY) {
			if (usbd_enable(ctx)) {
				LOG_ERR("Failed to enable device support");
			}
		}
		if (msg->type == USBD_MSG_VBUS_REMOVED) {
			if (usbd_disable(ctx)) {
				LOG_ERR("Failed to disable device support");
			}
		}
	}

	if (msg->type == USBD_MSG_CDC_ACM_CONTROL_LINE_STATE) {
		uint32_t dtr = 0U;

		uart_line_ctrl_get(msg->dev, UART_LINE_CTRL_DTR, &dtr);
		if (dtr) {
			k_sem_give(&dtr_sem);
		}
	}
}

static int enable_usb_device_next(void)
{
	int err;

	sample_usbd = sample_usbd_init_device(sample_msg_cb);
	if (sample_usbd == NULL) {
		LOG_ERR("Failed to initialize USB device");
		return -ENODEV;
	}

	if (!usbd_can_detect_vbus(sample_usbd)) {
		err = usbd_enable(sample_usbd);
		if (err) {
			LOG_ERR("Failed to enable device support");
			return err;
		}
	}

	LOG_INF("USB device support enabled");
	return 0;
}

static void cdc_print(const char *msg, size_t len)
{
    for (size_t i = 0; i < len; i++) {
        uart_poll_out(uart_dev, msg[i]);
    }
}

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

static K_SEM_DEFINE(drdy_sem, 0, 1);
static struct gpio_callback drdy_cb_data;

static void drdy_isr(const struct device *dev, struct gpio_callback *cb, uint32_t pins)
{
    k_sem_give(&drdy_sem);
}

static int setup_drdy_interrupt(void)
{
    int ret = gpio_pin_interrupt_configure_dt(&drdy_pin, GPIO_INT_EDGE_FALLING);
    if (ret) {
        return ret;
    }

    gpio_init_callback(&drdy_cb_data, drdy_isr, BIT(drdy_pin.pin));
    gpio_add_callback(drdy_pin.port, &drdy_cb_data);

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

int main(void)
{
    int ret;
    uint8_t id = 0;
    uint8_t frame[ADS_FRAME_BYTES];
    uint32_t sample_idx = 0;

    if (!device_is_ready(uart_dev)) {
        LOG_ERR("CDC ACM device not ready");
        return 0;
    }

    ret = enable_usb_device_next();
    if (ret != 0) {
        LOG_ERR("Failed to enable USB device support");
        return 0;
    }

    LOG_INF("Wait for DTR");
    k_sem_take(&dtr_sem, K_MSEC(500));
    LOG_INF("DTR set");

    k_msleep(100);

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

    ret = setup_drdy_interrupt();
    if (ret) {
        printk("DRDY interrupt setup failed: %d\n", ret);
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

    /* ret = ads_configure_internal_test_signal();
    if (ret) {
        printk("ADS test signal configuration failed: %d\n", ret);
        return 0;
    } */

    ret = ads_configure_external_inputs_all();
    if (ret) {
        printk("ADS external input configuration failed: %d\n", ret);
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

    ads_send_cmd(CMD_SDATAC);
    k_sleep(K_MSEC(2));

    ret = ads_configure_external_inputs_all();
    if (ret) {
        printk("ADS external input configuration failed: %d\n", ret);
        return 0;
    }

    ads_send_cmd(CMD_RDATAC);
    k_sleep(K_MSEC(2));

    ads_send_cmd(CMD_START);
    k_sleep(K_MSEC(20));

    //printk("Streaming internal test signal. Expect ~1 Hz square wave.\n");
    //printk("Polling DRDY and reading %d-byte frames.\n", ADS_FRAME_BYTES);

    /* Send once, before entering the loop */
    static const char hdr[] = "sample,status_ok,ch1,ch2,ch3,ch4,ch5,ch6,ch7,ch8\r\n";
    cdc_print(hdr, sizeof(hdr) - 1);

    while (1) {
        k_sem_take(&drdy_sem, K_FOREVER);
        ret = ads_read_frame_rdatac(frame);
        if (ret) {
            printk("Frame read failed: %d\n", ret);
            gpio_pin_toggle_dt(&led);
            continue;
        }

        bool status_header_ok = ((frame[0] & 0xF0) == 0xC0);
        int32_t ch1 = ads_decode24(&frame[3]);
        int32_t ch2 = ads_decode24(&frame[6]);
        int32_t ch3 = ads_decode24(&frame[9]);
        int32_t ch4 = ads_decode24(&frame[12]);
        int32_t ch5 = ads_decode24(&frame[15]);
        int32_t ch6 = ads_decode24(&frame[18]);
        int32_t ch7 = ads_decode24(&frame[21]);
        int32_t ch8 = ads_decode24(&frame[24]);

        char msg[96];
        int len = snprintf(msg, sizeof(msg),
            "%lu,%d,%ld,%ld,%ld,%ld,%ld,%ld,%ld,%ld\r\n",
            (unsigned long)sample_idx,
            status_header_ok ? 1 : 0,
            (long)ch1, (long)ch2, (long)ch3, (long)ch4,
            (long)ch5, (long)ch6, (long)ch7, (long)ch8);
        cdc_print(msg, len);

        if ((sample_idx % 250U) == 0U) {
            gpio_pin_toggle_dt(&led);   /* heartbeat, once per second */
        }

        sample_idx++;
    }

    return 0;
}