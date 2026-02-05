#pragma once

/**
 * @file ble_nus.hpp
 * @brief BLE NUS (Nordic UART Service) peripheral for fridge event transmission.
 *
 * Manages BLE advertising, connection, NUS data transfer, and disconnection.
 * Sends door open/close events to gateway.
 */

#include <cstdint>

namespace fridge
{
  // ========== Callbacks ==========

  /** @brief Callback type for data received via NUS RX characteristic. */
  using NusRxCallback = void (*)(const uint8_t* data, uint16_t length);

  // ========== BLE NUS Class ==========

  class BleNus
  {
    public:
      // ========== Constructor ==========

      BleNus() = default;

      // ========== Initialisation ==========

      /**
       * @brief Start BLE subsystem initialisation (non-blocking).
       * Kicks off bt_enable with a callback. Returns immediately.
       * Use WaitForReady() or IsReady() to check when complete.
       */
      int StartInit(NusRxCallback rxCallback);

      /** @brief Check if BLE subsystem is ready (non-blocking). */
      bool IsReady();

      /** @brief Block until BLE subsystem is ready or timeout expires. */
      bool WaitForReady(uint32_t timeoutMs);

      /** @brief Complete initialisation by registering NUS service. Call after BT is ready. */
      int CompleteInit();

      // ========== Advertising ==========

      /** @brief Start connectable BLE advertising. */
      int StartAdvertising();

      // ========== Connection ==========

      /** @brief Block until a central connects or timeout expires. */
      bool WaitForConnection(uint32_t timeoutMs);

      /** @brief Block until NUS notifications are enabled or timeout expires. */
      bool WaitForNusEnabled(uint32_t timeoutMs);

      /** @brief Check if currently connected (non-blocking). */
      bool IsConnected();

      /** @brief Check if NUS notifications are enabled (non-blocking). */
      bool IsNusEnabled();

      /** @brief Disconnect the current connection. */
      void Disconnect();

      /** @brief Reset state to allow reconnection after disconnect. */
      void PrepareReconnect();

      /** @brief Get the current connection RSSI. */
      int8_t GetRssi();

      // ========== Data Transfer ==========

      /**
       * @brief Send data over NUS TX, chunked to MTU size.
       * @param data Pointer to data buffer.
       * @param len  Length of data in bytes.
       */
      int SendEvent(const uint8_t* data, uint16_t length);
  };
}
