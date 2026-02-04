/*
 * Fridge Light Monitor — ALC Carefree Product Family
 *
 * System OFF sleep with BH1749 INT wake for ultra-low power.
 * Sends single packet on door close with duration.
 * Warning alert if door open > configurable timeout.
 *
 * Hardware: Thingy:53 (nRF5340) with BH1749 light sensor.
 * INT pin: P1.05 (active-low, open-drain with pull-up).
 */

#include <zephyr/kernel.h>
#include <zephyr/device.h>
#include <zephyr/drivers/gpio.h>
#include <zephyr/drivers/i2c.h>
#include <zephyr/sys/poweroff.h>
#include <zephyr/logging/log.h>
#include <helpers/nrfx_reset_reason.h>
#include <string.h>

#include "bh1749_light.hpp"
#include "ble_nus.hpp"

LOG_MODULE_REGISTER(main, LOG_LEVEL_INF);

// ========== Hardware Definitions ==========

// BH1749 I2C bus from devicetree.
static const struct i2c_dt_spec s_lightI2c = I2C_DT_SPEC_GET(DT_NODELABEL(bh1749));

// BH1749 INT pin from devicetree.
static const struct gpio_dt_spec s_lightInt = GPIO_DT_SPEC_GET(DT_NODELABEL(bh1749), int_gpios);

// Status LED (red).
static const struct gpio_dt_spec s_led = GPIO_DT_SPEC_GET(DT_ALIAS(led0), gpios);

// ========== Configuration Constants ==========

// Light threshold for door closed detection (absolute minimum).
constexpr uint16_t M_LIGHT_THRESHOLD_LOW { 50 };     // Door closed when green drops below this.

// Percentage of startup light level to use as wake threshold.
constexpr uint8_t M_WAKE_THRESHOLD_PERCENT { 90 };

// Minimum wake threshold to avoid spurious wakes in very dark environments.
constexpr uint16_t M_WAKE_THRESHOLD_MIN { 30 };

// Door open timeout before alert.
constexpr uint16_t M_DOOR_OPEN_TIMEOUT_SECS { 180 };  // 3 minutes.

// Valid measurement timeout.
constexpr uint16_t M_VALID_MEAS_TIMEOUT_MS { 500 };  // 0.5 secs.
                                                     // //
// BLE timeouts.
constexpr uint32_t M_BLE_CONNECT_TIMEOUT_MS { 10000 };
constexpr uint32_t M_BLE_NUS_ENABLED_TIMEOUT_MS { 5000 };

// Polling interval while waiting for door close.
constexpr uint32_t M_POLL_INTERVAL_MS { 500 };

// ========== Static State ==========

static fridge::Bh1749Light s_light { &s_lightI2c };
static fridge::BleNus s_ble;

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

static bool isSystemOffWake(uint32_t reason)
{
  return (reason & NRFX_RESET_REASON_OFF_MASK) != 0;
}

static void configureLed()
{
  gpio_pin_configure_dt(&s_led, GPIO_OUTPUT_INACTIVE);
}

static void ledOn()
{
  gpio_pin_set_dt(&s_led, true);
}

static void ledOff()
{
  gpio_pin_set_dt(&s_led, false);
}

static void ledBlink(int count, int onMs, int offMs)
{
  for (int i = 0; i < count; i++) {
    ledOn();
    k_msleep(onMs);
    ledOff();
    if (i < count - 1) {
      k_msleep(offMs);
    }
  }
}

static bool configureIntPin()
{
  // Configure INT pin as input for reading current state.
  int result { gpio_pin_configure_dt(&s_lightInt, GPIO_INPUT) };
  if (result < 0) {
    LOG_ERR("Failed to configure INT pin: %d!", result);
    return false;
  }

  LOG_INF("INT pin configured: port=%s, pin=%u.", s_lightInt.port->name, s_lightInt.pin);
  return true;
}

static void configureWakeOnLightHigh()
{
  // Configure INT pin to wake on rising edge (light detected).
  // The BH1749 INT is active-low (asserts LOW when threshold exceeded).
  // After asserting, when light drops, INT goes HIGH again.
  // We want to wake when INT goes LOW (light detected), so sense on falling edge.
  //
  // However, with GPIO_ACTIVE_LOW in devicetree, the sense polarity is inverted:
  // - NRF_GPIO_PIN_SENSE_HIGH on an ACTIVE_LOW pin triggers when physical pin goes LOW.
  //
  // For System OFF wake, we use the raw sense configuration.
  // Physical LOW (interrupt asserted) = light detected = wake.

  int result { gpio_pin_configure_dt(&s_lightInt, GPIO_INPUT) };
  if (result < 0) {
    LOG_ERR("Failed to configure INT for wake: %d!", result);
    return;
  }

  // Configure sense for wake on LOW (INT asserted = light detected).
  result = gpio_pin_interrupt_configure_dt(&s_lightInt, GPIO_INT_LEVEL_LOW);
  if (result < 0) {
    LOG_ERR("Failed to configure INT sense: %d!", result);
  }

  LOG_INF("Wake configured: sense LOW (light detection).");
}

static bool initLightSensor()
{
  LOG_INF("Initialising BH1749...");

  int result { s_light.Init() };
  if (result < 0) {
    LOG_ERR("BH1749 init failed: %d!", result);
    return false;
  }

  LOG_INF("BH1749 initialised.");
  return true;
}

static uint16_t calculateWakeThreshold(uint16_t doorOpenLight)
{
  uint16_t threshold { static_cast<uint16_t>((doorOpenLight * M_WAKE_THRESHOLD_PERCENT) / 100) };
  if (threshold < M_WAKE_THRESHOLD_MIN) {
    threshold = M_WAKE_THRESHOLD_MIN;
  }
  return threshold;
}

static bool configureWakeInterrupt(uint16_t wakeThreshold)
{
  // For System OFF wake, we only want to trigger on light INCREASE (door open).
  // Set low threshold to 0 so only high threshold can trigger.
  // The BH1749 interrupts when measurement goes outside the window (above high OR below low).
  constexpr uint16_t M_LOW_THRESHOLD_DISABLED { 0 };

  int result { s_light.ConfigureInterrupt(wakeThreshold, M_LOW_THRESHOLD_DISABLED) };
  if (result < 0) {
    LOG_ERR("Failed to configure interrupt thresholds: %d!", result);
    return false;
  }

  result = s_light.EnableInterrupt(true);
  if (result < 0) {
    LOG_ERR("Failed to enable interrupt: %d!", result);
    return false;
  }

  LOG_INF("Wake interrupt: trigger on green > %u.", wakeThreshold);
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

  return true;
}

static void enterSystemOff()
{
  LOG_INF("Entering System OFF...");

  // Configure wake source before sleeping.
  configureWakeOnLightHigh();

  // Give time for log to flush.
  k_msleep(100);

  // Enter System OFF.
  sys_poweroff();

  // Should never reach here.
  LOG_ERR("sys_poweroff() returned!");
}

// ========== BLE Functions ==========

static void onNusRx(const uint8_t* data, uint16_t length)
{
  LOG_INF("NUS RX: %u bytes.", length);
}

static void buildPacket(fridge::FridgeGatewayPacket& packet,
                        fridge::FridgeEventType eventType,
                        uint16_t durationSecs,
                        const fridge::LightReading& reading,
                        int8_t rssiDbm)
{
  // Clear packet.
  memset(&packet, 0, sizeof(packet));

  // Copy device ID (32-char hex string).
  strncpy(packet.deviceId, fridge::M_DEVICE_ID, sizeof(packet.deviceId));

  // Copy device name from BLE config.
  strncpy(packet.deviceName, CONFIG_BT_DEVICE_NAME, sizeof(packet.deviceName) - 1);

  packet.eventType = static_cast<uint8_t>(eventType);
  packet.durationSecs = durationSecs;
  packet.red = reading.red;
  packet.green = reading.green;
  packet.blue = reading.blue;
  packet.ir = reading.ir;
  packet.rssiDbm = rssiDbm;
}

// ========== Main State Machine ==========

static void handleFreshBoot()
{
  LOG_INF("Fresh boot: configuring sensor.");

  // Initialise sensor.
  if (!initLightSensor()) {
    while (true) {
      ledBlink(3, 100, 100);
      k_msleep(1000);
      LOG_ERR("Light sensor failed to initialise!");
    }
  }

  // Wait for first valid measurement.
  if (!s_light.WaitForValid(M_VALID_MEAS_TIMEOUT_MS)) {
    LOG_ERR("Timeout waiting for valid measurement!");
  }

  // Read current light level with door open (installation scenario).
  fridge::LightReading reading { };
  if (!readLight(reading)) {
    LOG_ERR("Failed to read initial light level!");
  }

  uint16_t doorOpenLight { reading.green };
  LOG_INF("Door open light level: green=%u.", doorOpenLight);

  // Always wait for door to close before sleeping.
  // Use doorOpenLight > 0 check to ensure we got a valid reading.
  // If reading failed (green=0), we still wait to ensure door is actually closed.
  if (doorOpenLight == 0 || reading.green >= M_LIGHT_THRESHOLD_LOW) {
    LOG_INF("Waiting for door close before sleeping...");

    while (true) {
      k_msleep(M_POLL_INTERVAL_MS);

      if (!readLight(reading)) {
        continue;
      }

      if (reading.green < M_LIGHT_THRESHOLD_LOW) {
        LOG_INF("Door closed (green=%u).", reading.green);
        break;
      }

      // Log periodically.
      static uint32_t lastLog { 0 };
      if (k_uptime_get_32() - lastLog > 5000) {
        LOG_INF("Waiting for door close: green=%u.", reading.green);
        lastLog = k_uptime_get_32();
      }
    }
  }

  // Calculate wake threshold as 90% of door-open light level.
  uint16_t wakeThreshold { calculateWakeThreshold(doorOpenLight) };
  LOG_INF("Wake threshold: %u (90%% of %u, min %u).", wakeThreshold, doorOpenLight, M_WAKE_THRESHOLD_MIN);

  // Configure interrupt thresholds.
  configureWakeInterrupt(wakeThreshold);

  // Clear any pending interrupt before sleeping.
  s_light.ClearInterrupt();

  LOG_INF("Entering System OFF, wake on green > %u.", wakeThreshold);

  // Enter System OFF, wake on light detection.
  enterSystemOff();
}

static void handleSystemOffWake()
{
  LOG_INF("System OFF wake: door opened, starting monitoring.");

  // Record wake time for duration tracking.
  uint32_t wakeTime { k_uptime_get_32() };

  // Start BLE init FIRST - non-blocking, runs in background while we do sensor init.
  // This kicks off the net core boot which takes ~2 seconds.
  LOG_INF("Starting BLE init (non-blocking)...");
  int bleResult { s_ble.StartInit(onNusRx) };
  if (bleResult < 0) {
    LOG_ERR("BLE start init failed: %d!", bleResult);
  }

  // Initialise sensor while BLE is starting up in background.
  if (!initLightSensor()) {
    LOG_ERR("Sensor init failed after wake!");
    enterSystemOff();
    return;
  }

  // Wait for valid measurement.
  if (!s_light.WaitForValid(M_VALID_MEAS_TIMEOUT_MS)) {
    LOG_WRN("Timeout waiting for valid measurement on wake.");
  }

  // Wait for next measurement cycle (120ms mode) to ensure fresh data.
  k_msleep(150);

  // Read initial light level - this is the door-open level.
  fridge::LightReading reading {};
  if (!readLight(reading)) {
    LOG_WRN("Failed to read light on wake.");
  }

  uint16_t doorOpenLight { reading.green };
  LOG_INF("Wake light (door open): green=%u.", doorOpenLight);

  // If light is below minimum threshold, this might be a spurious wake.
  if (doorOpenLight < M_WAKE_THRESHOLD_MIN) {
    LOG_WRN("Light below minimum threshold on wake, spurious? Going back to sleep.");
    s_light.ClearInterrupt();
    enterSystemOff();
    return;
  }

  // Door is open. Now wait for BLE to be ready and start advertising.
  LOG_INF("Door OPEN detected. Waiting for BLE ready...");

  bool bleReady { false };
  bool bleConnected { false };
  bool nusEnabled { false };
  int8_t rssi { 0 };

  // Helper lambda to complete BLE init and start advertising.
  auto completeBleAndAdvertise = [&bleReady, &bleResult]() {
    LOG_INF("BLE ready, completing init...");
    if (s_ble.CompleteInit() == 0) {
      bleReady = true;
      int advResult { s_ble.StartAdvertising() };
      if (advResult < 0) {
        LOG_ERR("Advertising start failed: %d!", advResult);
        bleResult = advResult;
      }
    } else {
      LOG_ERR("CompleteInit failed!");
      bleResult = -1;
    }
  };

  // Check if BLE is ready yet (non-blocking first).
  if (bleResult == 0 && s_ble.IsReady()) {
    completeBleAndAdvertise();
  }

  bool alertSent { false };

  while (true) {
    // Check for BLE ready (non-blocking).
    if (bleResult == 0 && !bleReady) {
      if (s_ble.IsReady()) {
        completeBleAndAdvertise();
      }
    }

    // Non-blocking check for BLE connection.
    if (bleReady && bleResult == 0 && !bleConnected) {
      if (s_ble.IsConnected()) {
        LOG_INF("Gateway connected while door open.");
        bleConnected = true;
      }
    }

    // Non-blocking check for NUS enabled.
    if (bleConnected && !nusEnabled) {
      if (s_ble.IsNusEnabled()) {
        LOG_INF("NUS enabled while door open.");
        nusEnabled = true;
        // Read RSSI now while connected.
        rssi = s_ble.GetRssi();
      }
    }

    k_msleep(M_POLL_INTERVAL_MS);

    // Read light sensor.
    if (!readLight(reading)) {
      continue;
    }

    uint32_t elapsedMs { k_uptime_get_32() - wakeTime };
    uint16_t elapsedSecs { static_cast<uint16_t>(elapsedMs / 1000) };

    // Check for timeout alert.
    if (!alertSent && (elapsedSecs >= M_DOOR_OPEN_TIMEOUT_SECS)) {
      LOG_WRN("*** DOOR OPEN TIMEOUT (%u s) ***", elapsedSecs);

      // Send alert packet if connected.
      if (nusEnabled) {
        fridge::FridgeGatewayPacket packet;
        buildPacket(packet, fridge::FridgeEventType::DoorAlert, elapsedSecs, reading, rssi);
        s_ble.SendEvent(reinterpret_cast<const uint8_t*>(&packet), sizeof(packet));
        LOG_INF("Sent ALERT packet.");
      }
      alertSent = true;
    }

    // Check if door closed.
    if (reading.green < M_LIGHT_THRESHOLD_LOW) {
      LOG_INF("*** DOOR CLOSED (green=%u, duration=%u s) ***", reading.green, elapsedSecs);

      // Send close packet.
      if (nusEnabled) {
        fridge::FridgeGatewayPacket packet;
        buildPacket(packet, fridge::FridgeEventType::DoorClose, elapsedSecs, reading, rssi);
        s_ble.SendEvent(reinterpret_cast<const uint8_t*>(&packet), sizeof(packet));
        LOG_INF("Sent CLOSE packet (duration=%u s, rssi=%d dBm).", elapsedSecs, rssi);
      } else if (bleResult == 0) {
        // BLE not connected yet, try blocking waits.
        if (!bleReady) {
          LOG_INF("Waiting for BLE ready...");
          if (s_ble.WaitForReady(M_BLE_CONNECT_TIMEOUT_MS)) {
            s_ble.CompleteInit();
            bleReady = true;
            s_ble.StartAdvertising();
          }
        }
        if (bleReady) {
          LOG_INF("Waiting for gateway connection...");
          if (s_ble.WaitForConnection(M_BLE_CONNECT_TIMEOUT_MS)) {
            if (s_ble.WaitForNusEnabled(M_BLE_NUS_ENABLED_TIMEOUT_MS)) {
              rssi = s_ble.GetRssi();
              fridge::FridgeGatewayPacket packet;
              buildPacket(packet, fridge::FridgeEventType::DoorClose, elapsedSecs, reading, rssi);
              s_ble.SendEvent(reinterpret_cast<const uint8_t*>(&packet), sizeof(packet));
              LOG_INF("Sent CLOSE packet (duration=%u s, rssi=%d dBm).", elapsedSecs, rssi);
            }
          }
        }
      }

      // Disconnect BLE.
      s_ble.Disconnect();
      k_msleep(100);

      // Calculate new wake threshold from this door-open cycle.
      uint16_t wakeThreshold { calculateWakeThreshold(doorOpenLight) };
      LOG_INF("New wake threshold: %u (90%% of %u).", wakeThreshold, doorOpenLight);

      // Configure thresholds for next wake.
      configureWakeInterrupt(wakeThreshold);

      // Clear interrupt and go back to sleep.
      s_light.ClearInterrupt();
      enterSystemOff();
      return;
    }

    // Log periodically.
    static uint32_t lastLog { 0 };
    if (k_uptime_get_32() - lastLog > 5000) {
      LOG_INF("Door still open: green=%u, elapsed=%u s, ble=%s.",
              reading.green, elapsedSecs, nusEnabled ? "ready" : "waiting");
      lastLog = k_uptime_get_32();
    }
  }
}

// ========== Main ==========

int main()
{
  // Read and clear the reset reason immediately.
  uint32_t resetReason { nrfx_reset_reason_get() };
  nrfx_reset_reason_clear(resetReason);

  LOG_INF("=== Fridge Light Monitor ===");
  logResetReason(resetReason);

  // Configure LED.
  configureLed();

  // Visual indicator of boot.
  ledBlink(2, 50, 50);

  // Configure INT pin.
  if (!configureIntPin()) {
    LOG_ERR("INT pin configuration failed, halting!");
    return -1;
  }

  // Branch based on reset reason.
  if (isSystemOffWake(resetReason)) {
    handleSystemOffWake();
  } else {
    handleFreshBoot();
  }

  // Should never reach here.
  LOG_ERR("Main loop exit, halting!");
  return 0;
}
