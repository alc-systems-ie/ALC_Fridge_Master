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

      // ========== TX Power Control ==========

      /**
       * @brief Set TX power for current connection.
       *
       * Uses Nordic vendor-specific HCI command to set TX power.
       * With nRF21540 FEM in POUTB mode (+10dB gain), effective antenna power is:
       *   antenna_power = SoC_power + 10dB
       *
       * @param powerDbm Desired SoC TX power in dBm (typically -20 to +3).
       * @return Actual TX power set, or 127 on error.
       */
      int8_t SetTxPower(int8_t powerDbm);

      /**
       * @brief Set TX power for advertising.
       *
       * Must be called before StartAdvertising() to affect advertising packets.
       * Uses Nordic vendor-specific HCI command.
       *
       * @param powerDbm Desired SoC TX power in dBm.
       * @return Actual TX power set, or 127 on error.
       */
      int8_t SetAdvTxPower(int8_t powerDbm);

      // ========== Data Transfer ==========

      /**
       * @brief Send data over NUS TX, chunked to MTU size.
       * @param data Pointer to data buffer.
       * @param len  Length of data in bytes.
       */
      int SendEvent(const uint8_t* data, uint16_t length);
  };
}
