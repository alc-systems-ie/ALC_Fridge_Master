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
  // ========== Event Types ==========

  enum class FridgeEventType : uint8_t {
    DoorOpen  = 0x01,
    DoorClose = 0x02,
    DoorAlert = 0x03,  // Door left open too long.
  };

  struct __packed FridgeOpenEvent {
    uint8_t  deviceId;
    uint8_t  eventType;   // FridgeEventType::DoorOpen
    uint16_t red;
    uint16_t green;
    uint16_t blue;
    uint16_t ir;
  };  // 10 bytes

  struct __packed FridgeCloseEvent {
    uint8_t  deviceId;
    uint8_t  eventType;    // FridgeEventType::DoorClose or DoorAlert
    uint16_t durationSecs; // Visit duration.
  };  // 4 bytes

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
