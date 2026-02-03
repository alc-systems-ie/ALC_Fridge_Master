/*
 * Fridge Light Monitor
 *
 * Monitors fridge door open/close events using BH1749 light sensor.
 * - Thingy:53: INT pin on P1.05
 * - Thingy:91: INT pin on P0.27
 */

#include <zephyr/kernel.h>
#include <zephyr/device.h>
#include <zephyr/drivers/gpio.h>
#include <zephyr/drivers/i2c.h>
#include <zephyr/logging/log.h>
#include <helpers/nrfx_reset_reason.h>

// #include <string.h>

#include "bh1749_light.hpp"

#if defined(CONFIG_BT)
#include "ble_nus.hpp"
#endif

LOG_MODULE_REGISTER(main, LOG_LEVEL_INF);

// ========== Hardware Definitions ==========

// BH1749 I2C bus from devicetree.
static const struct i2c_dt_spec s_lightI2c = I2C_DT_SPEC_GET(DT_NODELABEL(bh1749));

// BH1749 INT pin from devicetree.
static const struct gpio_dt_spec s_lightInt = GPIO_DT_SPEC_GET(DT_NODELABEL(bh1749), int_gpios);

// Status LED (red). 
static const struct gpio_dt_spec s_led = GPIO_DT_SPEC_GET(DT_ALIAS(led0), gpios);

// ========== Constants ==========

// Light thresholds for door detection.
constexpr uint16_t M_LIGHT_THRESHOLD_OPEN  { 200 };  // Door open when light exceeds this.
constexpr uint16_t M_LIGHT_THRESHOLD_CLOSE { 50 };   // Door closed when light drops below this.

// Polling intervals.
constexpr uint32_t M_POLL_INTERVAL_IDLE_MS { 1000 };  // Polling when door is closed.
constexpr uint32_t M_POLL_INTERVAL_OPEN_MS { 500 };   // Polling when door is open.

// Door open timeout before alert (default 60 seconds).
constexpr uint16_t M_DOOR_OPEN_TIMEOUT_SECS { 60 };

#if defined(CONFIG_BT)
// BLE timeouts.
constexpr uint32_t M_BLE_CONNECT_TIMEOUT_MS { 10000 };
constexpr uint32_t M_BLE_NUS_ENABLED_TIMEOUT_MS { 5000 };
#endif

// Device ID for this sensor node.
constexpr uint8_t M_DEVICE_ID { 1 };

// ========== Static State ==========

static fridge::Bh1749Light s_light { &s_lightI2c };

#if defined(CONFIG_BT)
static fridge::BleNus s_ble;
static bool s_bleInitialised { false };
#endif

// Door state tracking.
enum class DoorState { Closed, Open };
static DoorState s_doorState { DoorState::Closed };

// ========== Helper Functions ==========

static void logResetReason(uint32_t reason)
{
  if (reason == 0) {
    LOG_INF("Reset reason: Power-on reset (no flags set).");
    return;
  }

  LOG_INF("Reset reason register: 0x%08x.", reason);

  if (reason & NRFX_RESET_REASON_RESETPIN_MASK) {
    LOG_INF("  - Reset pin");
  }
  if (reason & NRFX_RESET_REASON_SREQ_MASK) {
    LOG_INF("  - Soft reset");
  }
  if (reason & NRFX_RESET_REASON_OFF_MASK) {
    LOG_INF("  - Wake from System OFF (GPIO)");
  }
  if (reason & NRFX_RESET_REASON_CTRLAP_MASK) {
    LOG_INF("  - Debug interface");
  }
}

static bool configureLightSensor()
{
  LOG_INF("Configuring BH1749...");

  int result { s_light.Init() };
  if (result < 0) {
    LOG_ERR("BH1749 init failed: %d!", result);
    return false;
  }

  LOG_INF("BH1749 configured.");
  return true;
}

static bool readLight(fridge::LightReading& reading)
{
  int result { s_light.Read(reading) };
  if (result == -EAGAIN) {
    // Data not ready yet, not an error.
    return false;
  }
  if (result < 0) {
    LOG_ERR("Light read failed: %d!", result);
    return false;
  }

  reading.deviceId = M_DEVICE_ID;
  return true;
}

static void configureIntPin()
{
  // Configure INT pin as input. The devicetree specifies GPIO_ACTIVE_LOW,
  // so gpio_pin_get_dt() returns 1 when the interrupt is asserted (physical LOW).
  int result = gpio_pin_configure_dt(&s_lightInt, GPIO_INPUT);
  if (result < 0) {
    LOG_ERR("Failed to configure INT pin: %d!", result);
    return;
  }

  LOG_INF("INT pin configured: port=%s, pin=%u.", s_lightInt.port->name, s_lightInt.pin);
}

#if defined(CONFIG_BT)
static void onNusRx(const uint8_t* data, uint16_t length)
{
  if (length == sizeof(uint32_t)) {
    uint32_t timestamp;
    memcpy(&timestamp, data, sizeof(timestamp));
    LOG_INF("Received timestamp: %u.", timestamp);
  }
}

static bool initBle()
{
  LOG_INF("Initialising BLE...");

  int result { s_ble.Init(onNusRx) };
  if (result < 0) {
    LOG_ERR("BLE init failed: %d!", result);
    return false;
  }

  s_bleInitialised = true;
  return true;
}

static bool connectBle()
{
  if (!s_bleInitialised) {
    if (!initBle()) {
      return false;
    }
  }

  LOG_INF("Starting BLE advertising...");

  int result { s_ble.StartAdvertising() };
  if (result < 0) {
    LOG_ERR("Advertising start failed: %d!", result);
    return false;
  }

  LOG_INF("Waiting for gateway connection...");
  if (!s_ble.WaitForConnection(M_BLE_CONNECT_TIMEOUT_MS)) {
    LOG_WRN("Connection timeout.");
    return false;
  }

  if (!s_ble.WaitForNusEnabled(M_BLE_NUS_ENABLED_TIMEOUT_MS)) {
    LOG_WRN("NUS enable timeout.");
    s_ble.Disconnect();
    return false;
  }

  return true;
}

static bool sendOpenEvent(const fridge::LightReading& reading)
{
  fridge::FridgeOpenEvent event {
    .deviceId  = M_DEVICE_ID,
    .eventType = static_cast<uint8_t>(fridge::FridgeEventType::DoorOpen),
    .red       = reading.red,
    .green     = reading.green,
    .blue      = reading.blue,
    .ir        = reading.ir
  };

  int result { s_ble.SendEvent(reinterpret_cast<const uint8_t*>(&event), sizeof(event)) };
  if (result < 0) {
    LOG_ERR("Failed to send open event: %d!", result);
    return false;
  }

  LOG_INF("Door OPEN event sent.");
  return true;
}

static bool sendCloseEvent(uint16_t durationSecs, bool alert)
{
  fridge::FridgeCloseEvent event {
    .deviceId     = M_DEVICE_ID,
    .eventType    = static_cast<uint8_t>(alert ? fridge::FridgeEventType::DoorAlert
                                               : fridge::FridgeEventType::DoorClose),
    .durationSecs = durationSecs
  };

  int result { s_ble.SendEvent(reinterpret_cast<const uint8_t*>(&event), sizeof(event)) };
  if (result < 0) {
    LOG_ERR("Failed to send close event: %d!", result);
    return false;
  }

  LOG_INF("Door %s event sent (duration=%u s).", alert ? "ALERT" : "CLOSE", durationSecs);
  return true;
}
#endif // CONFIG_BT

// ========== Main ==========

int main()
{
  // Read and clear the reset reason immediately.
  uint32_t resetReason { nrfx_reset_reason_get() };
  nrfx_reset_reason_clear(resetReason);

  LOG_INF("=== Fridge Light Monitor ===");
  logResetReason(resetReason);

  // Configure LED.
  gpio_pin_configure_dt(&s_led, GPIO_OUTPUT_INACTIVE);

  // Configure INT pin.
  configureIntPin();

  // Configure BH1749.
  if (!configureLightSensor()) {
    LOG_ERR("Failed to configure light sensor, halting!");
    return -1;
  }

  // Wait for first valid measurement.
  k_msleep(150);

  LOG_INF("Entering main polling loop...");
  LOG_INF("  Open threshold: green > %u", M_LIGHT_THRESHOLD_OPEN);
  LOG_INF("  Close threshold: green < %u", M_LIGHT_THRESHOLD_CLOSE);

  fridge::LightReading reading {};
  uint32_t doorOpenTime { 0 };
  bool alertSent { false };

  while (true) {
    // Wait for next measurement.
    uint32_t pollInterval { (s_doorState == DoorState::Closed)
                            ? M_POLL_INTERVAL_IDLE_MS
                            : M_POLL_INTERVAL_OPEN_MS };
    k_msleep(pollInterval);

    // Read light sensor.
    if (!readLight(reading)) {
      continue;
    }

    uint16_t green { reading.green };

    // Also log INT pin state periodically.
    int intPin = gpio_pin_get_dt(&s_lightInt);

    // State machine.
    switch (s_doorState) {
      case DoorState::Closed:
        if (green > M_LIGHT_THRESHOLD_OPEN) {
          // Door just opened!
          LOG_INF("*** DOOR OPENED (green=%u > %u, INT=%d) ***",
                  green, M_LIGHT_THRESHOLD_OPEN, intPin);
          s_doorState = DoorState::Open;
          doorOpenTime = k_uptime_get_32();
          alertSent = false;

          // LED on.
          gpio_pin_set_dt(&s_led, true);

#if defined(CONFIG_BT)
          if (connectBle()) {
            k_msleep(100);
            sendOpenEvent(reading);
            k_msleep(100);
            s_ble.Disconnect();
            k_msleep(100);
          }
#endif

          gpio_pin_set_dt(&s_led, false);
        } else {
          // Still closed, occasional log.
          static uint32_t lastLog { 0 };
          if (k_uptime_get_32() - lastLog > 10000) {
            LOG_INF("Door closed, green=%u, INT=%d.", green, intPin);
            lastLog = k_uptime_get_32();
          }
        }
        break;

      case DoorState::Open:
        {
          uint32_t elapsedMs { k_uptime_get_32() - doorOpenTime };
          uint16_t elapsedSecs { static_cast<uint16_t>(elapsedMs / 1000) };

          // Check for timeout alert.
          if (!alertSent && (elapsedSecs >= M_DOOR_OPEN_TIMEOUT_SECS)) {
            LOG_WRN("*** DOOR OPEN TIMEOUT (%u s) ***", elapsedSecs);

            gpio_pin_set_dt(&s_led, true);

#if defined(CONFIG_BT)
            if (connectBle()) {
              k_msleep(100);
              sendCloseEvent(elapsedSecs, true);  // Alert event.
              k_msleep(100);
              s_ble.Disconnect();
              k_msleep(100);
            }
#endif

            gpio_pin_set_dt(&s_led, false);
            alertSent = true;
          }

          // Check if door closed.
          if (green < M_LIGHT_THRESHOLD_CLOSE) {
            LOG_INF("*** DOOR CLOSED (green=%u < %u, duration=%u s, INT=%d) ***",
                    green, M_LIGHT_THRESHOLD_CLOSE, elapsedSecs, intPin);
            s_doorState = DoorState::Closed;

            gpio_pin_set_dt(&s_led, true);

#if defined(CONFIG_BT)
            if (connectBle()) {
              k_msleep(100);
              sendCloseEvent(elapsedSecs, false);  // Normal close event.
              k_msleep(100);
              s_ble.Disconnect();
              k_msleep(100);
            }
#endif

            gpio_pin_set_dt(&s_led, false);
          } else {
            // Still open, log periodically.
            static uint32_t lastOpenLog { 0 };
            if (k_uptime_get_32() - lastOpenLog > 5000) {
              LOG_INF("Door open, green=%u, elapsed=%u s, INT=%d.", green, elapsedSecs, intPin);
              lastOpenLog = k_uptime_get_32();
            }
          }
        }
        break;
    }
  }

  return 0;
}
