 #include "bhi360_driver.hpp"
 #include "threads.hpp"

 #include <zephyr/kernel.h>
 #include <zephyr/logging/log.h>
 #include <nrfx.h>
 #include <hal/nrf_gpio.h>
 #include <nrfx_spim.h>
 #include <string.h>

 extern "C" {
 #include "bhy2.h"
 #include "bhy2_parse.h"
 #include "firmware/bhi360/BHI360_Aux_BMM150.fw.h"
 }

 #define BHY2_RD_WR_LEN 256
 #define WORK_BUFFER_SIZE 2048
 #define QUAT_SENSOR_ID BHY2_SENSOR_ID_GAMERV
 #define LACC_SENSOR_ID BHY2_SENSOR_ID_ACC

 LOG_MODULE_REGISTER(bhi360_driver, LOG_LEVEL_DBG);

 typedef struct {
	 uint8_t cs_pin;
	 struct bhy2_dev bhy2;
	 bool initialized;
	 char name[32];
 } imu_device_t;

 static imu_device_t imu_devices[] = {
	 { .cs_pin = NRF_GPIO_PIN_MAP(1, 11), .initialized = false, .name = "IMU_1" }
 };
 #define NUM_IMUS (sizeof(imu_devices) / sizeof(imu_devices[0]))

 static const nrfx_spim_t m_spi = NRFX_SPIM_INSTANCE(3);
 #define BSP_SPI_MISO NRF_GPIO_PIN_MAP(1, 8)
 #define BSP_SPI_MOSI NRF_GPIO_PIN_MAP(0, 30)
 #define BSP_SPI_CLK NRF_GPIO_PIN_MAP(0, 31)
 static uint32_t imu_sample_idx;

 static void print_api_error(int8_t rslt, struct bhy2_dev *dev)
 {
	 if (rslt != BHY2_OK) {
		 LOG_ERR("API error: %d", rslt);
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

 #define APP_ERROR_CHECK(err_code) do { if ((err_code) != NRFX_SUCCESS) LOG_ERR("Error %d", (err_code)); } while (0)

 static void setup_SPI(imu_device_t *imu)
 {
	 nrfx_spim_config_t config = NRFX_SPIM_DEFAULT_CONFIG(BSP_SPI_CLK, BSP_SPI_MOSI, BSP_SPI_MISO, imu->cs_pin);
	 config.ss_pin = imu->cs_pin;
	 config.bit_order = NRF_SPIM_BIT_ORDER_MSB_FIRST;
	 config.frequency = NRF_SPIM_FREQ_32M;
	 config.mode = NRF_SPIM_MODE_0;
	 static bool initialized;
	 if (!initialized) {
		 APP_ERROR_CHECK(nrfx_spim_init(&m_spi, &config, NULL, NULL));
		 initialized = true;
	 }
 }

 static int8_t bhi360_spi_read(uint8_t reg, uint8_t *data, uint32_t length, void *ptr)
 {
	 auto *imu = static_cast<imu_device_t *>(ptr);
	 uint8_t tx[1] = { static_cast<uint8_t>(reg | 0x80) };
	 uint8_t rx[length + 1];
	 volatile uint32_t *end = reinterpret_cast<uint32_t *>(nrfx_spim_end_event_get(&m_spi));
	 auto desc = NRFX_SPIM_XFER_TRX(tx, sizeof(tx), rx, sizeof(rx));
	 int err = nrfx_spim_xfer(&m_spi, &desc, NRFX_SPIM_FLAG_NO_XFER_EVT_HANDLER);
	 APP_ERROR_CHECK(err);
	 if (err == NRFX_SUCCESS) { while (*end == 0) {} *end = 0; }
	 nrf_gpio_pin_set(imu->cs_pin);
	 memcpy(data, &rx[1], length);
	 return BHY2_INTF_RET_SUCCESS;
 }

 static int8_t bhi360_spi_write(uint8_t reg, const uint8_t *data, uint32_t length, void *ptr)
 {
	 auto *imu = static_cast<imu_device_t *>(ptr);
	 uint8_t tx[length + 1];
	 volatile uint32_t *end = reinterpret_cast<uint32_t *>(nrfx_spim_end_event_get(&m_spi));
	 tx[0] = reg;
	 memcpy(tx + 1, data, length);
	 auto desc = NRFX_SPIM_XFER_TX(tx, length + 1);
	 int err = nrfx_spim_xfer(&m_spi, &desc, NRFX_SPIM_FLAG_NO_XFER_EVT_HANDLER);
	 APP_ERROR_CHECK(err);
	 if (err == NRFX_SUCCESS) { while (*end == 0) {} *end = 0; }
	 nrf_gpio_pin_set(imu->cs_pin);
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
	 (void)k_msgq_put(&imu_queue, &packet, K_NO_WAIT);
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
	 (void)k_msgq_put(&imu_queue, &packet, K_NO_WAIT);
 }

 static void parse_meta_event(const struct bhy2_fifo_parse_data_info *info, void *)
 {
	 LOG_INF("IMU meta event %u", info->data_ptr[0]);
 }

 static bool initialize_imu(imu_device_t *imu)
 {
	 uint8_t product_id = 0, boot_status;
	 uint16_t version = 0;
	 nrf_gpio_cfg_output(imu->cs_pin);
	 nrf_gpio_pin_clear(imu->cs_pin);
	 k_sleep(K_USEC(1));
	 int8_t rslt = bhy2_init(BHY2_SPI_INTERFACE, bhi360_spi_read, bhi360_spi_write,
							  bhi360_delay_us, BHY2_RD_WR_LEN, imu, &imu->bhy2);
	 if (rslt != BHY2_OK || bhy2_soft_reset(&imu->bhy2) != BHY2_OK) return false;
	 bool id_ok = false;
	 for (int retry = 0; retry < 20; ++retry) {
		 rslt = bhy2_get_product_id(&product_id, &imu->bhy2);
		 if (rslt == BHY2_OK && product_id == BHY2_PRODUCT_ID) { id_ok = true; break; }
		 k_msleep(10);
	 }
	 if (!id_ok) return false;
	 uint8_t hintr_ctrl = BHY2_ICTL_DISABLE_STATUS_FIFO | BHY2_ICTL_DISABLE_DEBUG;
	 print_api_error(bhy2_set_host_interrupt_ctrl(hintr_ctrl, &imu->bhy2), &imu->bhy2);
	 print_api_error(bhy2_set_host_intf_ctrl(0, &imu->bhy2), &imu->bhy2);
	 rslt = bhy2_get_boot_status(&boot_status, &imu->bhy2);
	 if (rslt != BHY2_OK || !(boot_status & BHY2_BST_HOST_INTERFACE_READY)) return false;
	 if (upload_firmware(&imu->bhy2) != BHY2_OK || bhy2_boot_from_ram(&imu->bhy2) != BHY2_OK) return false;
	 if (bhy2_get_kernel_version(&version, &imu->bhy2) != BHY2_OK || version == 0) return false;
	 print_api_error(bhy2_update_virtual_sensor_list(&imu->bhy2), &imu->bhy2);
	 print_api_error(bhy2_set_virt_sensor_cfg(QUAT_SENSOR_ID, 100.0f, 0, &imu->bhy2), &imu->bhy2);
	 print_api_error(bhy2_set_virt_sensor_cfg(LACC_SENSOR_ID, 100.0f, 0, &imu->bhy2), &imu->bhy2);
	 print_api_error(bhy2_register_fifo_parse_callback(BHY2_SYS_ID_META_EVENT, parse_meta_event, imu, &imu->bhy2), &imu->bhy2);
	 if (bhy2_is_sensor_available(QUAT_SENSOR_ID, &imu->bhy2)) print_api_error(bhy2_register_fifo_parse_callback(QUAT_SENSOR_ID, parse_quaternion, imu, &imu->bhy2), &imu->bhy2);
	 if (bhy2_is_sensor_available(LACC_SENSOR_ID, &imu->bhy2)) print_api_error(bhy2_register_fifo_parse_callback(LACC_SENSOR_ID, parse_linear_acceleration, imu, &imu->bhy2), &imu->bhy2);
	 imu->initialized = true;
	 return true;
 }

 void bhi360_init(void)
 {
	 for (size_t i = 0; i < NUM_IMUS; ++i) {
		 setup_SPI(&imu_devices[i]);
		 if (!initialize_imu(&imu_devices[i])) LOG_ERR("%s: IMU initialization failed", imu_devices[i].name);
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
