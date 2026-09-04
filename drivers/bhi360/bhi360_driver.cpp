 #include "bhi360_driver.hpp"
 #include "threads.hpp"

 #include <zephyr/kernel.h>
 #include <zephyr/logging/log.h>
 #include <zephyr/drivers/spi.h>
 #include <zephyr/drivers/gpio.h>
 #include <zephyr/device.h>
 #include <stdio.h>
 #include <string.h>

 extern "C" {
 #include "bhy2.h"
 #include "bhy2_parse.h"
 #include "firmware/bhi360/BHI360_Aux_BMM150.fw.h"
 }

static const char *get_api_error(int8_t error_code)
{
    switch (error_code) {
        case BHY2_OK:              return "BHY2_OK";
        case BHY2_E_NULL_PTR:      return "BHY2_E_NULL_PTR";
        case BHY2_E_INVALID_PARAM: return "BHY2_E_INVALID_PARAM";
        case BHY2_E_IO:            return "BHY2_E_IO";
        case BHY2_E_MAGIC:         return "BHY2_E_MAGIC";
        case BHY2_E_TIMEOUT:       return "BHY2_E_TIMEOUT";
        case BHY2_E_BUFFER:        return "BHY2_E_BUFFER";
        default:                   return "Unknown error code";
    }
}

// static const char *get_sensor_error_text(uint8_t sensor_error)
// {
//     switch (sensor_error) {
//         case 0:  return "No error";
//         default: return "Unknown sensor error";
//     }
// }

// static const char *get_sensor_name(uint8_t sensor_id)
// {
//     switch (sensor_id) {
//         case BHY2_SENSOR_ID_RV: return "Rotation Vector";
//         default:                return "Unknown sensor";
//     }
// }

 #define BHY2_RD_WR_LEN 256
 #define WORK_BUFFER_SIZE 2048
 #define QUAT_SENSOR_ID BHY2_SENSOR_ID_GAMERV
 #define LACC_SENSOR_ID BHY2_SENSOR_ID_ACC

 LOG_MODULE_REGISTER(bhi360_driver, LOG_LEVEL_DBG);

 typedef struct {
	 struct bhy2_dev bhy2;
	 bool initialized;
	 char name[32];
 } imu_device_t;

static imu_device_t imu_devices[] = {
 { .initialized = false, .name = "IMU_1" }
};
#define NUM_IMUS (sizeof(imu_devices) / sizeof(imu_devices[0]))

#define BHI360_NODE DT_NODELABEL(imu_main)
#define BHI360_RESET_NODE DT_NODELABEL(reset_imu_main)
#define BHI360_SPI_OPERATION (SPI_WORD_SET(8) | SPI_TRANSFER_MSB)

static const struct spi_dt_spec bhi360_spi =
	 SPI_DT_SPEC_GET(BHI360_NODE, BHI360_SPI_OPERATION);
static const struct gpio_dt_spec bhi360_reset =
	 GPIO_DT_SPEC_GET(BHI360_RESET_NODE, gpios);

 static uint32_t imu_sample_idx;

 static void print_api_error(int8_t rslt, struct bhy2_dev *dev)
{
    if (rslt != BHY2_OK) {
        LOG_ERR("API error: %s", get_api_error(rslt));  // was "%d", rslt
        if (rslt == BHY2_E_IO && dev != NULL) {
            LOG_ERR("Interface error: %d", dev->hif.intf_rslt);
            dev->hif.intf_rslt = BHY2_INTF_RET_SUCCESS;
        }
    }
}

 static int8_t upload_firmware(struct bhy2_dev *dev)
 {
	 uint32_t incr = 256;
	 uint32_t len = sizeof(bhy2_firmware_image);
	 int8_t rslt = BHY2_OK;
	 for (uint32_t i = 0; i < len && rslt == BHY2_OK; i += incr) {
		 if (incr > len - i) {
			 incr = len - i;
			 if (incr % 4) incr = ((incr >> 2) + 1) << 2;
		 }
		 rslt = bhy2_upload_firmware_to_ram_partly(&bhy2_firmware_image[i], len, i, incr, dev);
	 }
	 return rslt;
 }

static bool setup_SPI(void)
 {
	 if (!spi_is_ready_dt(&bhi360_spi)) {
		 LOG_ERR("BHI360 SPI device or chip select is not ready");
		 return false;
	 }
	 if (!gpio_is_ready_dt(&bhi360_reset)) {
		 LOG_ERR("BHI360 reset GPIO is not ready");
		 return false;
	 }
	 int ret = gpio_pin_configure_dt(&bhi360_reset, GPIO_OUTPUT_INACTIVE);
	 if (ret != 0) {
		 LOG_ERR("BHI360 reset GPIO configuration failed: %d", ret);
		 return false;
	 }
	 ret = gpio_pin_set_dt(&bhi360_reset, 1);
	 if (ret != 0) {
		 LOG_ERR("BHI360 reset assertion failed: %d", ret);
		 return false;
	 }
	 k_msleep(2);
	 ret = gpio_pin_set_dt(&bhi360_reset, 0);
	 if (ret != 0) {
		 LOG_ERR("BHI360 reset release failed: %d", ret);
		 return false;
	 }
	 k_msleep(10);
	 LOG_DBG("BHI360 SPI device is ready");
	 return true;
 }

static bool test_bhi360_spi(void)
{
	 /* First CS pulse switches I/F from I2C to SPI - ignore result.
	  * Read Fuser2 Identifier (0x1C) twice; discard first, check second == 0x89. */
	 uint8_t tx[2] = { 0x00, 0x00 };
	 uint8_t rx[2] = { 0x00, 0x00 };

	 struct spi_buf tx_buf;
	 tx_buf.buf = tx;
	 tx_buf.len = sizeof(tx);
	 struct spi_buf_set tx_set;
	 tx_set.buffers = &tx_buf;
	 tx_set.count = 1;

	 struct spi_buf rx_buf;
	 rx_buf.buf = rx;
	 rx_buf.len = sizeof(rx);
	 struct spi_buf_set rx_set;
	 rx_set.buffers = &rx_buf;
	 rx_set.count = 1;

	 /* First read - discard (switches I2C to SPI) */
	 tx[0] = 0x1C | 0x80;  // Read Fuser2 Identifier register
	 spi_transceive_dt(&bhi360_spi, &tx_set, &rx_set);

	 /* Second read - check value */
	 tx[0] = 0x1C | 0x80;  // Read Fuser2 Identifier register
	 int ret = spi_transceive_dt(&bhi360_spi, &tx_set, &rx_set);

	 LOG_INF("BHI360 SPI test: ret=%d, fuser2_id=0x%02x (expected 0x89)", ret, rx[1]);

	 if (ret != 0) {
		 LOG_ERR("SPI read failed");
		 return false;
	 }
	 if (rx[1] != 0x89) {
		 LOG_ERR("Fuser2 Identifier mismatch: got 0x%02x, expected 0x89", rx[1]);
		 return false;
	 }
	 return true;
 } /* extern "C" from line 13 */

 static int8_t bhi360_spi_read(uint8_t reg, uint8_t *data, uint32_t length, void *ptr)
 {
	 ARG_UNUSED(ptr);
	 if (length > BHY2_RD_WR_LEN) {
		 LOG_ERR("SPI read length %u exceeds buffer", length);
		 return -1;
	 }
	 /* TODO: static buffers are fine for single-threaded init, but if this
	  * is ever called concurrently (e.g. from ISR or multiple threads),
	  * these become shared-mutable-state hazards and need a mutex. */
	 static uint8_t tx[BHY2_RD_WR_LEN + 1];
	 static uint8_t rx[BHY2_RD_WR_LEN + 1];
	 memset(tx, 0, sizeof(tx));
	 tx[0] = static_cast<uint8_t>(reg | 0x80);

	 struct spi_buf tx_buf;
	 tx_buf.buf = tx;
	 tx_buf.len = length + 1;
	 struct spi_buf_set tx_set;
	 tx_set.buffers = &tx_buf;
	 tx_set.count = 1;
	 struct spi_buf rx_buf;
	 rx_buf.buf = rx;
	 rx_buf.len = length + 1;
	 struct spi_buf_set rx_set;
	 rx_set.buffers = &rx_buf;
	 rx_set.count = 1;

	 int err = spi_transceive_dt(&bhi360_spi, &tx_set, &rx_set);
	 if (err != 0) {
		 LOG_ERR("SPIM read failed: %d", err);
		 return -1;
	 }
	 memcpy(data, &rx[1], length);
	 return BHY2_INTF_RET_SUCCESS;
 }

 static int8_t bhi360_spi_write(uint8_t reg, const uint8_t *data, uint32_t length, void *ptr)
 {
	 ARG_UNUSED(ptr);
	 if (length > BHY2_RD_WR_LEN) {
		 LOG_ERR("SPI write length %u exceeds buffer", length);
		 return -1;
	 }
	 /* TODO: static buffers are fine for single-threaded init, but if this
	  * is ever called concurrently (e.g. from ISR or multiple threads),
	  * these become shared-mutable-state hazards and need a mutex. */
	 static uint8_t tx[BHY2_RD_WR_LEN + 1];
	 memset(tx, 0, sizeof(tx));
	 tx[0] = reg & 0x7F;  // Clear bit 7 for write operation
	 memcpy(tx + 1, data, length);

	 struct spi_buf tx_buf;
	 tx_buf.buf = tx;
	 tx_buf.len = length + 1;
	 struct spi_buf_set tx_set;
	 tx_set.buffers = &tx_buf;
	 tx_set.count = 1;

	 int err = spi_write_dt(&bhi360_spi, &tx_set);
	 if (err != 0) {
		 LOG_ERR("SPIM write failed: %d", err);
		return -1;
	 }
	 return BHY2_INTF_RET_SUCCESS;
 }

 static void bhi360_delay_us(uint32_t period_us, void *) { k_usleep(period_us); }

 static uint8_t checksum(const ImuSamplePacket &packet)
 {
	 const auto *bytes = reinterpret_cast<const uint8_t *>(&packet);
	 uint8_t result = 0;
	 for (size_t i = 0; i < sizeof(packet) - 1; ++i) result ^= bytes[i];
	 return result;
 }

 static void parse_quaternion(const struct bhy2_fifo_parse_data_info *info, void *)
 {
	 if (info->data_size != 11) return;
	 struct bhy2_data_quaternion unused;
	 bhy2_parse_quaternion(info->data_ptr, &unused);
	 ImuSamplePacket packet{};
	 packet.sync[0] = 0xBB; packet.sync[1] = 0x66;
	 packet.sample_idx = imu_sample_idx++;
	 packet.sensor_id = info->sensor_id;
	 packet.data_len = 10;
	 memcpy(packet.data, info->data_ptr, packet.data_len);
	 packet.checksum = checksum(packet);
	 /* Independent SD copy: capture every packet regardless of BLE state.
	  * Non-blocking; if the SD queue is ever full the packet is dropped from
	  * SD only — never purge or block here. */
	 (void)k_msgq_put(&imu_sd_queue, &packet, K_NO_WAIT);
	 /* Latest-wins: if the queue is full, purge the stale backlog and keep
	  * only the freshest packet, so IMU never accumulates old data. */
	 if (k_msgq_put(&imu_queue, &packet, K_NO_WAIT) != 0) {
		 k_msgq_purge(&imu_queue);
		 (void)k_msgq_put(&imu_queue, &packet, K_NO_WAIT);
	 }
 }

 static void parse_linear_acceleration(const struct bhy2_fifo_parse_data_info *info, void *)
 {
	 if (info->data_size < 7) return;
	 ImuSamplePacket packet{};
	 packet.sync[0] = 0xBB; packet.sync[1] = 0x66;
	 packet.sample_idx = imu_sample_idx++;
	 packet.sensor_id = info->sensor_id;
	 packet.data_len = 6;
	 memcpy(packet.data, info->data_ptr, packet.data_len);
	 packet.checksum = checksum(packet);
	 /* Independent SD copy: capture every packet regardless of BLE state.
	  * Non-blocking; if the SD queue is ever full the packet is dropped from
	  * SD only — never purge or block here. */
	 (void)k_msgq_put(&imu_sd_queue, &packet, K_NO_WAIT);
	 /* Latest-wins: if the queue is full, purge the stale backlog and keep
	  * only the freshest packet, so IMU never accumulates old data. */
	 if (k_msgq_put(&imu_queue, &packet, K_NO_WAIT) != 0) {
		 k_msgq_purge(&imu_queue);
		 (void)k_msgq_put(&imu_queue, &packet, K_NO_WAIT);
	 }
 }

 static void parse_meta_event(const struct bhy2_fifo_parse_data_info *info, void *)
 {
	//LOG_INF("IMU meta event %u", info->data_ptr[0]);
 }

 static bool initialize_imu(imu_device_t *imu)
 {
	 uint8_t product_id = 0, boot_status;
	 uint16_t version = 0;
	 k_msleep(10);
	 int8_t rslt = bhy2_init(BHY2_SPI_INTERFACE, bhi360_spi_read, bhi360_spi_write,
							  bhi360_delay_us, BHY2_RD_WR_LEN, imu, &imu->bhy2);
	 if (rslt != BHY2_OK) {
		 LOG_ERR("%s: bhy2_init failed: %d", imu->name, rslt);
		 return false;
	 }
	 rslt = bhy2_soft_reset(&imu->bhy2);
	 if (rslt != BHY2_OK) {
		 LOG_ERR("%s: BHI360 soft reset failed: %d", imu->name, rslt);
		 return false;
	 }
	 bool id_ok = false;
	 for (int retry = 0; retry < 20; ++retry) {
		 rslt = bhy2_get_product_id(&product_id, &imu->bhy2);
		 if (rslt == BHY2_OK && product_id == BHY2_PRODUCT_ID) { id_ok = true; break; }
		 k_msleep(10);
	 }
	 if (!id_ok) {
		 LOG_ERR("%s: product ID failed: result=%d, ID=0x%02x", imu->name, rslt, product_id);
		 return false;
	 }
	 uint8_t hintr_ctrl = BHY2_ICTL_DISABLE_STATUS_FIFO | BHY2_ICTL_DISABLE_DEBUG;
	 print_api_error(bhy2_set_host_interrupt_ctrl(hintr_ctrl, &imu->bhy2), &imu->bhy2);
	 print_api_error(bhy2_set_host_intf_ctrl(0, &imu->bhy2), &imu->bhy2);
	 rslt = bhy2_get_boot_status(&boot_status, &imu->bhy2);
	 if (rslt != BHY2_OK || !(boot_status & BHY2_BST_HOST_INTERFACE_READY)) {
		 LOG_ERR("%s: host interface not ready: result=%d, status=0x%02x", imu->name, rslt, boot_status);
		 return false;
	 }
	 rslt = upload_firmware(&imu->bhy2);
	 if (rslt != BHY2_OK) {
		 LOG_ERR("%s: firmware upload failed: %d", imu->name, rslt);
		 return false;
	 }
	 rslt = bhy2_boot_from_ram(&imu->bhy2);
	 if (rslt != BHY2_OK) {
		 LOG_ERR("%s: boot from RAM failed: %d", imu->name, rslt);
		 return false;
	 }
	 rslt = bhy2_get_kernel_version(&version, &imu->bhy2);
	 if (rslt != BHY2_OK || version == 0) {
		 LOG_ERR("%s: kernel version failed: result=%d, version=0x%04x", imu->name, rslt, version);
		 return false;
	 }
	 print_api_error(bhy2_update_virtual_sensor_list(&imu->bhy2), &imu->bhy2);
	 /* IMU ODR reduced from 100 Hz to 50 Hz to free BLE bandwidth for EMG.
	  * At 50 Hz the IMU uses ~50 notifications/s; combined with 8-sample EMG
	  * grouping (~62.5 notif/s) the total ~112.5 notif/s fits under the
	  * ~128 notif/s link ceiling. */
	 print_api_error(bhy2_set_virt_sensor_cfg(QUAT_SENSOR_ID, 50.0f, 0, &imu->bhy2), &imu->bhy2);
	 print_api_error(bhy2_set_virt_sensor_cfg(LACC_SENSOR_ID, 50.0f, 0, &imu->bhy2), &imu->bhy2);
	 print_api_error(bhy2_register_fifo_parse_callback(BHY2_SYS_ID_META_EVENT, parse_meta_event, imu, &imu->bhy2), &imu->bhy2);
	 if (bhy2_is_sensor_available(QUAT_SENSOR_ID, &imu->bhy2)) print_api_error(bhy2_register_fifo_parse_callback(QUAT_SENSOR_ID, parse_quaternion, imu, &imu->bhy2), &imu->bhy2);
	 if (bhy2_is_sensor_available(LACC_SENSOR_ID, &imu->bhy2)) print_api_error(bhy2_register_fifo_parse_callback(LACC_SENSOR_ID, parse_linear_acceleration, imu, &imu->bhy2), &imu->bhy2);
	 imu->initialized = true;
	 return true;
 }

 void bhi360_init(void)
 {
	 for (size_t i = 0; i < NUM_IMUS; ++i) {
		 if (!setup_SPI()) {
			 LOG_ERR("%s: SPI initialization failed", imu_devices[i].name);
			 continue;
		 }
		 if (!test_bhi360_spi()) {
			 LOG_ERR("%s: SPI communication test failed", imu_devices[i].name);
			 continue;
		 }
		 if (!initialize_imu(&imu_devices[i])) {
			 LOG_ERR("%s: IMU initialization failed", imu_devices[i].name);
		 }
	 }
 }

 void bhi360_process_fifo(void)
 {
	 static uint8_t work_buffer[WORK_BUFFER_SIZE];
	 for (size_t i = 0; i < NUM_IMUS; ++i) {
		 if (imu_devices[i].initialized) {
			 int8_t rslt = bhy2_get_and_process_fifo(work_buffer, sizeof(work_buffer), &imu_devices[i].bhy2);
			 if (rslt != BHY2_OK) LOG_WRN("%s FIFO processing error", imu_devices[i].name);
		 }
	 }
 }
