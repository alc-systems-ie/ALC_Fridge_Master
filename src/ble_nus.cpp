#include "ble_nus.hpp"

#include <zephyr/kernel.h>
#include <zephyr/bluetooth/bluetooth.h>
#include <zephyr/bluetooth/conn.h>
#include <zephyr/bluetooth/gatt.h>
#include <zephyr/bluetooth/hci.h>
#include <bluetooth/services/nus.h>
#include <zephyr/logging/log.h>

LOG_MODULE_REGISTER(ble_nus, LOG_LEVEL_INF);

namespace fridge
{

 // ========== Static State ==========

  static K_SEM_DEFINE(s_SemBtReady, 0, 1);
  static K_SEM_DEFINE(s_SemConnection, 0, 1);
  static K_SEM_DEFINE(s_SemNusEnabled, 0, 1);
  static struct bt_conn* s_currentConnection { nullptr };
  static NusRxCallback s_rxCallback { nullptr };
  static bool s_nusEnabled { false };
  static bool s_btReady { false };
  static int s_btInitError { 0 };
  static bool s_disconnectExpected { false };

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
    if (!s_disconnectExpected) {
      LOG_INF("Disconnected (reason %u).", reason);
    }
    if (s_currentConnection) {
      bt_conn_unref(s_currentConnection);
      s_currentConnection = nullptr;
    }
    s_nusEnabled = false;
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
      s_nusEnabled = true;
      k_sem_give(&s_SemNusEnabled);
    } else {
      // Disabled is expected during disconnect, don't log.
      s_nusEnabled = false;
    }
  }

  static struct bt_nus_cb s_nusCallbacks {
    .received = onNusReceived,
    .send_enabled = onNusSendEnabled,
  };

  // ========== BT Ready Callback ==========

  static void onBtReady(int error)
  {
    if (error) {
      LOG_ERR("Bluetooth init failed in callback: %d!", error);
      s_btInitError = error;
    } else {
      LOG_INF("Bluetooth ready (callback).");
      s_btReady = true;
    }
    k_sem_give(&s_SemBtReady);
  }

  // ========== BleNus Implementation ==========

  int BleNus::StartInit(NusRxCallback rxCallback)
  {
    s_rxCallback = rxCallback;
    s_btReady = false;
    s_btInitError = 0;
    s_disconnectExpected = false;
    s_nusEnabled = false;

    // Start BLE init with callback - returns immediately.
    // The callback will be invoked when BT subsystem is ready
    // (including net core boot on nRF5340).
    int result { bt_enable(onBtReady) };
    if (result == -EALREADY) {
      // Already enabled, mark as ready.
      s_btReady = true;
      k_sem_give(&s_SemBtReady);
      return 0;
    }
    if (result < 0) {
      LOG_ERR("bt_enable() failed: %d!", result);
      return result;
    }

    LOG_INF("BLE init started (non-blocking).");
    return 0;
  }

  bool BleNus::WaitForReady(uint32_t timeoutMs)
  {
    if (s_btReady) {
      return true;
    }

    if (k_sem_take(&s_SemBtReady, K_MSEC(timeoutMs)) != 0) {
      LOG_ERR("Timeout waiting for BT ready!");
      return false;
    }

    return s_btReady && (s_btInitError == 0);
  }

  bool BleNus::IsReady()
  {
    return s_btReady && (s_btInitError == 0);
  }

  int BleNus::CompleteInit()
  {
    if (!s_btReady) {
      LOG_ERR("BT not ready, cannot complete init!");
      return -EAGAIN;
    }
    if (s_btInitError != 0) {
      return s_btInitError;
    }

    int result { bt_nus_init(&s_nusCallbacks) };
    if (result < 0) {
      LOG_ERR("NUS init failed: %d!", result);
      return result;
    }

    LOG_INF("NUS initialised.");
    return 0;
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

  bool BleNus::IsConnected()
  {
    return s_currentConnection != nullptr;
  }

  bool BleNus::IsNusEnabled()
  {
    return s_nusEnabled;
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
    s_disconnectExpected = true;
    s_nusEnabled = false;

    // Don't actively disconnect - the gateway disconnects after receiving
    // our packet, and we're about to enter System OFF anyway. Just clean
    // up our connection reference to avoid any pending operations.
    if (s_currentConnection) {
      bt_conn_unref(s_currentConnection);
      s_currentConnection = nullptr;
    }
  }

  int8_t BleNus::GetRssi()
  {
    constexpr int8_t M_RSSI_INVALID { 0 };

    if (!s_currentConnection) {
      LOG_WRN("No connection for RSSI read.");
      return M_RSSI_INVALID;
    }

    // Get HCI connection handle.
    uint16_t handle { 0 };
    int result { bt_hci_get_conn_handle(s_currentConnection, &handle) };
    if (result < 0) {
      LOG_ERR("Failed to get connection handle: %d!", result);
      return M_RSSI_INVALID;
    }

    struct net_buf* rsp { nullptr };
    struct net_buf* buf { bt_hci_cmd_create(BT_HCI_OP_READ_RSSI, sizeof(uint16_t)) };
    if (!buf) {
      LOG_ERR("Failed to create HCI command buffer!");
      return M_RSSI_INVALID;
    }

    // Add connection handle to command.
    net_buf_add_le16(buf, handle);

    result = bt_hci_cmd_send_sync(BT_HCI_OP_READ_RSSI, buf, &rsp);
    if (result < 0) {
      LOG_ERR("Failed to send HCI read RSSI command: %d!", result);
      return M_RSSI_INVALID;
    }

    // Parse response: status (1 byte) + handle (2 bytes) + rssi (1 byte).
    struct bt_hci_rp_read_rssi* rssiRsp {
      reinterpret_cast<struct bt_hci_rp_read_rssi*>(rsp->data)
    };
    int8_t rssi { rssiRsp->rssi };

    net_buf_unref(rsp);

    LOG_INF("Connection RSSI: %d dBm.", rssi);
    return rssi;
  }
} 
