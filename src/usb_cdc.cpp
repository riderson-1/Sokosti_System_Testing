/**
 * @file usb_cdc.cpp
 * @brief USB CDC-ACM (virtual serial port) bring-up and byte-level output.
 * @version 0.1
 * @date 2026-07-08
 */

#include "usb_cdc.hpp"

#include <zephyr/kernel.h>
#include <zephyr/device.h>
#include <zephyr/drivers/uart.h>
#include <zephyr/logging/log.h>
#include <zephyr/usb/usbd.h>

/*
 * sample_usbd.h (and its implementation in sample_usbd_init.c) is plain C
 * sample code with no extern "C" guards. Without this wrapper, the C++
 * compiler mangles sample_usbd_init_device()/sample_usbd_setup_device(),
 * and the linker can't match them to the unmangled C symbols produced by
 * compiling sample_usbd_init.c with the C compiler.
 */
extern "C" {
#include "sample_usbd.h"
}

LOG_MODULE_REGISTER(usb_cdc, LOG_LEVEL_INF);

namespace {

const struct device *const uart_dev = DEVICE_DT_GET_ONE(zephyr_cdc_acm_uart);

struct usbd_context *sample_usbd;
K_SEM_DEFINE(dtr_sem, 0, 1);

void sample_msg_cb(struct usbd_context *const ctx, const struct usbd_msg *msg)
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

int enable_usb_device_next(void)
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

} // namespace

namespace usb_cdc {

int init(void)
{
    if (!device_is_ready(uart_dev)) {
        LOG_ERR("CDC ACM device not ready");
        return -ENODEV;
    }

    int ret = enable_usb_device_next();
    if (ret != 0) {
        LOG_ERR("Failed to enable USB device support");
        return ret;
    }

    LOG_INF("Wait for DTR");
    k_sem_take(&dtr_sem, K_MSEC(500));
    LOG_INF("DTR set");

    return 0;
}

void print(const char *msg, size_t len)
{
    for (size_t i = 0; i < len; i++) {
        uart_poll_out(uart_dev, msg[i]);
    }
}

} // namespace usb_cdc