#pragma once

/**
 * @file ble_nus.hpp
 * @brief BLE NUS (Nordic UART Service) peripheral for drawer event transmission.
 *
 * Manages BLE advertising, connection, NUS data transfer, and disconnection.
 * Receives a timestamp from the gateway via NUS RX.
 */

#include <cstdint>

namespace drawer 
{
  /** @brief Callback type for data received via NUS RX characteristic. */
  using NusRxCallback = void (*)(const uint8_t* data, uint16_t length);

  class BleNus
  {
    public:
      // ========== Constructor ==========

      BleNus() = default;

      // ========== Initialisation ==========

      /** @brief Initialise BLE subsystem and register NUS service. */
      int Init(NusRxCallback rxCallback);

      // ========== Advertising ==========

      /** @brief Start connectable BLE advertising. */
      int StartAdvertising();

      // ========== Connection ==========

      /** @brief Block until a central connects or timeout expires. */
      bool WaitForConnection(uint32_t timeoutMs);

      /** @brief Block until NUS notifications are enabled or timeout expires. */
      bool WaitForNusEnabled(uint32_t timeoutMs);

      /** @brief Disconnect the current connection. */
      void Disconnect();

      // ========== Data Transfer ==========

      /**
       * @brief Send data over NUS TX, chunked to MTU size.
       * @param data Pointer to data buffer.
       * @param len  Length of data in bytes.
       */
      int SendEvent(const uint8_t* data, uint16_t length);
  };
} 
