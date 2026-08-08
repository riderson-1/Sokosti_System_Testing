#include "ble_nus.hpp"

#include <errno.h>

#include <zephyr/kernel.h>
#include <zephyr/sys/util.h>
#include <zephyr/logging/log.h>
#include <zephyr/bluetooth/bluetooth.h>
#include <zephyr/bluetooth/conn.h>
#include <zephyr/bluetooth/gatt.h>
#include <bluetooth/services/nus.h>

LOG_MODULE_REGISTER(ble_nus, LOG_LEVEL_INF);

namespace {

struct bt_conn *current_conn;
bool notifications_enabled;
struct k_mutex state_mutex;
struct k_work_delayable adv_work;
struct bt_gatt_exchange_params exchange_params;

static const struct bt_data ad[] = {
	BT_DATA_BYTES(BT_DATA_FLAGS, (BT_LE_AD_GENERAL | BT_LE_AD_NO_BREDR)),
	BT_DATA_BYTES(BT_DATA_UUID128_ALL, BT_UUID_NUS_VAL),
};

static const struct bt_data sd[] = {
	BT_DATA(BT_DATA_NAME_COMPLETE, CONFIG_BT_DEVICE_NAME,
		sizeof(CONFIG_BT_DEVICE_NAME) - 1),
};

void start_advertising(struct k_work *work)
{
	ARG_UNUSED(work);

	int err = bt_le_adv_start(BT_LE_ADV_CONN_FAST_1, ad, ARRAY_SIZE(ad),
				  sd, ARRAY_SIZE(sd));
	if (err) {
		LOG_ERR("Advertising failed to start: %d", err);
	} else {
		LOG_INF("Advertising as %s", CONFIG_BT_DEVICE_NAME);
	}
}

void mtu_exchange_cb(struct bt_conn *conn, uint8_t att_err,
			     struct bt_gatt_exchange_params *params)
{
	ARG_UNUSED(params);
	LOG_INF("ATT MTU for %p: %u%s", (void *)conn, bt_gatt_get_mtu(conn),
		att_err ? " (exchange failed)" : "");
}

void connected(struct bt_conn *conn, uint8_t err)
{
	if (err) {
		LOG_WRN("BLE connection failed: %u", err);
		return;
	}

	k_mutex_lock(&state_mutex, K_FOREVER);
	if (current_conn) {
		bt_conn_unref(current_conn);
	}
	current_conn = bt_conn_ref(conn);
	notifications_enabled = false;
	k_mutex_unlock(&state_mutex);

	exchange_params.func = mtu_exchange_cb;
	err = bt_gatt_exchange_mtu(conn, &exchange_params);
	if (err && err != -EALREADY) {
		LOG_WRN("MTU exchange request failed: %d", err);
	}
	LOG_INF("BLE connected");
}

void disconnected(struct bt_conn *conn, uint8_t reason)
{
	k_mutex_lock(&state_mutex, K_FOREVER);
	if (current_conn) {
		bt_conn_unref(current_conn);
		current_conn = nullptr;
	}
	notifications_enabled = false;
	k_mutex_unlock(&state_mutex);

	LOG_INF("BLE disconnected, reason %u", reason);
	k_work_reschedule(&adv_work, K_MSEC(100));
}

void nus_send_enabled(enum bt_nus_send_status status)
{
	k_mutex_lock(&state_mutex, K_FOREVER);
	notifications_enabled = (status == BT_NUS_SEND_STATUS_ENABLED);
	k_mutex_unlock(&state_mutex);
	LOG_INF("NUS notifications %s",
		status == BT_NUS_SEND_STATUS_ENABLED ? "enabled" : "disabled");
}

struct bt_nus_cb nus_callbacks = {
	.received = nullptr,
	.sent = nullptr,
	.send_enabled = nus_send_enabled,
};

BT_CONN_CB_DEFINE(conn_callbacks) = {
	.connected = connected,
	.disconnected = disconnected,
};

} // namespace

int ble_nus_init(void)
{
	k_mutex_init(&state_mutex);
	k_work_init_delayable(&adv_work, start_advertising);

	int err = bt_enable(nullptr);
	if (err) {
		LOG_ERR("Bluetooth init failed: %d", err);
		return err;
	}

	err = bt_nus_init(&nus_callbacks);
	if (err) {
		LOG_ERR("NUS init failed: %d", err);
		return err;
	}

	start_advertising(nullptr);
	return 0;
}

bool ble_nus_ready(void)
{
	k_mutex_lock(&state_mutex, K_FOREVER);
	bool ready = current_conn != nullptr && notifications_enabled;
	k_mutex_unlock(&state_mutex);
	return ready;
}

int ble_nus_send(const uint8_t *data, uint16_t len)
{
	k_mutex_lock(&state_mutex, K_FOREVER);
	if (!current_conn || !notifications_enabled) {
		k_mutex_unlock(&state_mutex);
		return -ENOTCONN;
	}

	/* bt_nus_send is non-blocking; it returns -ENOMEM when buffers are full. */
	int err = bt_nus_send(current_conn, data, len);
	k_mutex_unlock(&state_mutex);
	return err;
}

int ble_nus_send_stream(const uint8_t *data, uint16_t len)
{
	uint16_t offset = 0;

	while (offset < len) {
		uint16_t chunk_len;

		k_mutex_lock(&state_mutex, K_FOREVER);
		if (!current_conn || !notifications_enabled) {
			k_mutex_unlock(&state_mutex);
			return -ENOTCONN;
		}

		/* NUS payload capacity is ATT MTU minus the notification header. */
		uint32_t mtu = bt_nus_get_mtu(current_conn);
		if (mtu == 0U) {
			k_mutex_unlock(&state_mutex);
			return -EMSGSIZE;
		}
		chunk_len = (uint16_t)MIN((uint32_t)(len - offset), mtu);

		int err = bt_nus_send(current_conn, data + offset, chunk_len);
		k_mutex_unlock(&state_mutex);
		if (err) {
			return err;
		}

		offset += chunk_len;
	}

	return 0;
}