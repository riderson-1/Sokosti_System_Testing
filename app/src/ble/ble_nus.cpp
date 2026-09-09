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
bool advertising_active;
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
	if (err == -EALREADY) {
		advertising_active = true;
	} else if (err) {
		LOG_ERR("Advertising failed to start: %d", err);
	} else {
		advertising_active = true;
		LOG_INF("Advertising as %s", CONFIG_BT_DEVICE_NAME);
	}
}

void stop_advertising(void)
{
	if (!advertising_active) {
		return;
	}

	int err = bt_le_adv_stop();
	if (err && err != -EALREADY) {
		LOG_WRN("Advertising stop failed: %d", err);
	} else {
		advertising_active = false;
		LOG_INF("Advertising stopped");
	}
}

void mtu_exchange_cb(struct bt_conn *conn, uint8_t att_err,
			     struct bt_gatt_exchange_params *params)
{
	ARG_UNUSED(params);
	LOG_INF("ATT MTU for %p: %u%s", (void *)conn, bt_gatt_get_mtu(conn),
		att_err ? " (exchange failed)" : "");
}

const char *phy_str(uint8_t phy)
{
	switch (phy) {
	case BT_GAP_LE_PHY_1M:    return "1M";
	case BT_GAP_LE_PHY_2M:    return "2M";
	case BT_GAP_LE_PHY_CODED: return "Coded";
	default:                  return "Unknown";
	}
}

/* Fired whenever the LL connection interval / latency / timeout actually
 * in use changes — including the very first negotiation after connect.
 * This is the ground truth for what bt_conn_le_param_update() achieved;
 * the central can reject/renegotiate the requested values. */
void le_param_updated(struct bt_conn *conn, uint16_t interval,
			      uint16_t latency, uint16_t timeout)
{
	ARG_UNUSED(conn);
	LOG_INF("Conn params updated: interval=%u units (%u.%02u ms) "
		"latency=%u timeout=%u units (%u ms)",
		interval, (interval * 125U) / 100U, (interval * 125U) % 100U,
		latency, timeout, timeout * 10U);
}

#if defined(CONFIG_BT_USER_PHY_UPDATE)
/* Fired when the active PHY changes — including the response to our 2M
 * request in connected(). Confirms whether the central actually granted 2M
 * or the link stayed on 1M (e.g. central doesn't support 2M). */
void le_phy_updated(struct bt_conn *conn, struct bt_conn_le_phy_info *param)
{
	ARG_UNUSED(conn);
	LOG_INF("PHY updated: tx=%s rx=%s", phy_str(param->tx_phy),
		phy_str(param->rx_phy));
}
#endif /* CONFIG_BT_USER_PHY_UPDATE */

#if defined(CONFIG_BT_USER_DATA_LEN_UPDATE)
/* Fired when the negotiated LL Data Length changes — confirms whether the
 * network core's DLE request (251 B / max time) was actually granted, or
 * whether the link is still capped at the legacy 27-byte PDU. */
void le_data_len_updated(struct bt_conn *conn,
				 struct bt_conn_le_data_len_info *info)
{
	ARG_UNUSED(conn);
	LOG_INF("Data length updated: tx_len=%u tx_time=%uus "
		"rx_len=%u rx_time=%uus",
		info->tx_max_len, info->tx_max_time,
		info->rx_max_len, info->rx_max_time);
}
#endif /* CONFIG_BT_USER_DATA_LEN_UPDATE */

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

	/* Request fast connection parameters for high-throughput streaming:
	 * 7.5 - 15 ms connection interval (6 - 12 units of 1.25 ms),
	 * 0 slave latency (so peripheral responds every event without skipping),
	 * 1000 ms supervision timeout. */
	struct bt_le_conn_param param = BT_LE_CONN_PARAM_INIT(
		6,                                    /* min interval: 6 * 1.25 ms = 7.5 ms  */
		12,                                   /* max interval: 12 * 1.25 ms = 15.0 ms */
		0,                                    /* slave latency = 0 (critical)        */
		BT_GAP_MS_TO_CONN_TIMEOUT(1000));     /* supervision timeout                 */
	(void)bt_conn_le_param_update(conn, &param);

	/* Request 2M PHY to roughly double the raw air rate. The central must
	 * also support 2M PHY, otherwise the request is rejected/ignored.
	 * (BT_CONN_LE_PHY_PARAM_2M is a compound-literal array macro whose
	 * address cannot be taken in C++, so declare a local struct instead.) */
	struct bt_conn_le_phy_param phy_param = BT_CONN_LE_PHY_PARAM_INIT(
		BT_GAP_LE_PHY_2M, BT_GAP_LE_PHY_2M);
	err = bt_conn_le_phy_update(conn, &phy_param);
	if (err) {
		LOG_WRN("PHY update to 2M failed: %d", err);
	}

	/* Request Data Length Extension (DLE): 251-byte payload per packet at
	 * the maximum packet time. This lets the controller send full 251-byte
	 * packets instead of the default 27 bytes, roughly tripling the raw
	 * air rate per packet. Requires CONFIG_BT_USER_DATA_LEN_UPDATE=y. */
	struct bt_conn_le_data_len_param dle_param =
		BT_CONN_LE_DATA_LEN_PARAM_INIT(BT_GAP_DATA_LEN_MAX, BT_GAP_DATA_TIME_MAX);
	err = bt_conn_le_data_len_update(conn, &dle_param);
	if (err) {
		LOG_WRN("DLE update failed: %d", err);
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
	.le_param_updated = le_param_updated,
#if defined(CONFIG_BT_USER_PHY_UPDATE)
	.le_phy_updated = le_phy_updated,
#endif
#if defined(CONFIG_BT_USER_DATA_LEN_UPDATE)
	.le_data_len_updated = le_data_len_updated,
#endif
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

	/* Advertising is NOT started here. The application decides when to
	 * advertise via ble_nus_start_advertising() (e.g. only when USB is
	 * not connected). */
	return 0;
}

void ble_nus_start_advertising(void)
{
	k_work_schedule(&adv_work, K_NO_WAIT);
}

void ble_nus_stop_advertising(void)
{
	stop_advertising();
}

bool ble_nus_ready(void)
{
	k_mutex_lock(&state_mutex, K_FOREVER);
	bool ready = current_conn != nullptr && notifications_enabled;
	k_mutex_unlock(&state_mutex);
	return ready;
}

bool ble_nus_connected(void)
{
	k_mutex_lock(&state_mutex, K_FOREVER);
	bool connected_now = current_conn != nullptr;
	k_mutex_unlock(&state_mutex);
	return connected_now;
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