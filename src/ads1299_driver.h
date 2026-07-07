/**
 * @file ads1299_driver.h
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

#include <stdint.h>
#include <stddef.h>
#include <string.h>

#include "ads1299_definitions.h"

int ads_spi_write_bytes(const uint8_t *data, size_t len);

int ads_spi_transceive_bytes(const uint8_t *tx_data,
                    uint8_t *rx_data,
                    size_t len);

int ads_send_cmd(uint8_t cmd);

int ads_read_reg(uint8_t reg, uint8_t *value);

int ads_write_reg(uint8_t reg, uint8_t value);

int ads_write_regs(uint8_t start_reg, const uint8_t *values, size_t count);

int32_t ads_decode24(const uint8_t *p);

int ads_read_frame_rdatac(uint8_t frame[ADS_FRAME_BYTES]);

void ads_hw_reset(void);

int ads_read_id(uint8_t *id);

int ads_dump_test_regs(void);

int ads_configure_external_inputs_all(void);

int ads_configure_internal_test_signal(void);
