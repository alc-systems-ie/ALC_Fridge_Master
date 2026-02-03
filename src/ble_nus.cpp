#include "ble_nus.hpp"

#include <zephyr/kernel.h>
#include <zephyr/bluetooth/bluetooth.h>
#include <zephyr/bluetooth/conn.h>
#include <zephyr/bluetooth/gatt.h>
#include <bluetooth/services/nus.h>
#include <zephyr/logging/log.h>

LOG_MODULE_REGISTER(ble_nus, LOG_LEVEL_INF);

namespace fridge 
{

 // ========== Static State ==========

  static K_SEM_DEFINE(s_SemConnection, 0, 1);
  static K_SEM_DEFINE(s_SemNusEnabled, 0, 1);
  static struct bt_conn* s_currentConnection { nullptr };
  static NusRxCallback s_rxCallback { nullptr };

  // ========== Advertising Data ==========

  #define DEVICE_NAME     CONFIG_BT_DEVICE_NAME
  #define DEVICE_NAME_LEN (sizeof(DEVICE_NAME) - 1)

  static const struct bt_data s_advData[] {
    BT_DATA_BYTES(BT_DATA_FLAGS, (BT_LE_AD_GENERAL | BT_LE_AD_NO_BREDR)),
    BT_DATA(BT_DATA_NAME_COMPLETE, DEVICE_NAME, DEVICE_NAME_LEN),
  };

  static const struct bt_data s_sdData[] {
    BT_DATA_BYTES(BT_DATA_UUID128_ALL, BT_UUID_NUS_VAL),
  };

  // ========== BT Connection Callbacks ==========

  static void onConnected(struct bt_conn* connection, uint8_t error)
  {
    if (error) {
      LOG_ERR("Connection failed: %u!", error);
      return;
    }
    LOG_INF("Connected.");
    s_currentConnection = bt_conn_ref(connection);
    k_sem_give(&s_SemConnection);
  }

  static void onDisconnected(struct bt_conn* connection, uint8_t reason)
  {
    LOG_INF("Disconnected (reason %u).", reason);
    if (s_currentConnection) {
      bt_conn_unref(s_currentConnection);
      s_currentConnection = nullptr;
    }
  }

  BT_CONN_CB_DEFINE(s_connCallbacks) = {
    .connected = onConnected,
    .disconnected = onDisconnected,
  };

  // ========== NUS Callbacks ==========

  static void onNusReceived(struct bt_conn* connection, const uint8_t* const data, uint16_t length)
  {
    LOG_INF("NUS RX: %u bytes.", length);
    if (s_rxCallback) {
      s_rxCallback(data, length);
    }
  }

  static void onNusSendEnabled(enum bt_nus_send_status status)
  {
    if (status == BT_NUS_SEND_STATUS_ENABLED) {
      LOG_INF("NUS notifications enabled.");
      k_sem_give(&s_SemNusEnabled);
    } else {
      LOG_INF("NUS notifications disabled.");
    }
  }

  static struct bt_nus_cb s_nusCallbacks {
    .received = onNusReceived,
    .send_enabled = onNusSendEnabled,
  };

  // ========== BleNus Implementation ==========

  constexpr uint32_t M_NET_CORE_BOOT_MS { 2000 };

  int BleNus::Init(NusRxCallback rxCallback)
  {
    constexpr int OK { 0 };
    s_rxCallback = rxCallback;

    // Wait for the network core to boot after System OFF wake before
    // attempting BLE init. Without this delay, the IPC endpoint binding
    // fails and leaves the HCI transport in a bad state.
    k_msleep(M_NET_CORE_BOOT_MS);

    int result { bt_enable(nullptr) };
    if (result == -EALREADY) { result = 0; }
    if (result < 0) {
      LOG_ERR("Bluetooth init failed: %d!", result);
      return result;
    }
    LOG_INF("Bluetooth initialised.");

    result = bt_nus_init(&s_nusCallbacks);
    if (result < 0) {
      LOG_ERR("NUS init failed: %d!", result);
      return result;
    }

    return OK;
  }

  int BleNus::StartAdvertising()
  {
    constexpr int OK { 0 };
    int result { bt_le_adv_start(BT_LE_ADV_CONN_FAST_2, 
                                 s_advData, ARRAY_SIZE(s_advData), 
                                 s_sdData, ARRAY_SIZE(s_sdData)) };
    if (result < 0) {
      LOG_ERR("Advertising start failed: %d!", result);
      return result;
    }

    LOG_INF("Advertising started.");

    return OK;
  }

  bool BleNus::WaitForConnection(uint32_t timeoutMs)
  {
    k_sem_reset(&s_SemConnection);
    return k_sem_take(&s_SemConnection, K_MSEC(timeoutMs)) == 0;
  }

  bool BleNus::WaitForNusEnabled(uint32_t timeoutMs)
  {
    k_sem_reset(&s_SemNusEnabled);
    return k_sem_take(&s_SemNusEnabled, K_MSEC(timeoutMs)) == 0;
  }

  int BleNus::SendEvent(const uint8_t* data, uint16_t length)
  {
    if (!s_currentConnection) {
      LOG_ERR("No connection for NUS send!");
      return -ENOTCONN;
    }

    uint32_t mtu { bt_nus_get_mtu(s_currentConnection) };
    if (mtu == 0) {
      constexpr uint32_t M_DEFAULT_ATT_PAYLOAD { 20 };
      mtu = M_DEFAULT_ATT_PAYLOAD;
    }

    uint16_t offset { 0 };
    while (offset < length) {
      uint16_t chunk { static_cast<uint16_t>((length - offset) > mtu ? mtu : (length - offset)) };
      int result { bt_nus_send(s_currentConnection, (data + offset), chunk) };
      if (result < 0) {
        LOG_ERR("NUS send failed at offset %u: %d!", offset, result);
        return result;
      }
      offset += chunk;
    }

    LOG_INF("Sent %u bytes over NUS.", length);
    return 0;
  }

  void BleNus::Disconnect()
  {
    if (s_currentConnection) {
      bt_conn_disconnect(s_currentConnection, BT_HCI_ERR_REMOTE_USER_TERM_CONN);
    }
  }
} 
