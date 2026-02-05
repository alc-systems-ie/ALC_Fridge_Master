#include "app.hpp"

#include <zephyr/kernel.h>
#include <zephyr/device.h>
#include <zephyr/devicetree.h>
#include <zephyr/drivers/gpio.h>
#include <zephyr/drivers/i2c.h>
#include <zephyr/drivers/retained_mem.h>
#include <zephyr/sys/poweroff.h>
#include <zephyr/sys/crc.h>
#include <zephyr/sys/byteorder.h>
#include <zephyr/logging/log.h>
#include <helpers/nrfx_reset_reason.h>
#include <hal/nrf_gpio.h>

#include <string.h>

#include "bh1749_light.hpp"
#include "ble_nus.hpp"

LOG_MODULE_REGISTER(app, LOG_LEVEL_INF);

namespace fridge
{
  // ========== Hardware Definitions ==========

  // Main user button (sw0 / SW2 on silkscreen) - P1.14, active low with pull-up.
  static const struct gpio_dt_spec s_button = GPIO_DT_SPEC_GET(DT_ALIAS(sw0), gpios);

  // BH1749 INT pin (P1.05 on Thingy:53).
  static const struct gpio_dt_spec s_lightInt = GPIO_DT_SPEC_GET(DT_NODELABEL(bh1749), int_gpios);

  // RGB LED pins.
  static const struct gpio_dt_spec s_ledRed = GPIO_DT_SPEC_GET(DT_ALIAS(led0), gpios);
  static const struct gpio_dt_spec s_ledGreen = GPIO_DT_SPEC_GET(DT_ALIAS(led1), gpios);
  static const struct gpio_dt_spec s_ledBlue = GPIO_DT_SPEC_GET(DT_ALIAS(led2), gpios);

  // BH1749 I2C bus from devicetree.
  static const struct i2c_dt_spec s_lightI2c = I2C_DT_SPEC_GET(DT_NODELABEL(bh1749));

  // ========== Static Hardware Objects ==========

  static Bh1749Light s_light { &s_lightI2c };
  static BleNus s_ble;

  // ========== Configuration Constants ==========

  // Minimum wake threshold to avoid spurious wakes in very dark environments.
  constexpr uint16_t M_WAKE_THRESHOLD_MIN { 15 };

  // Maximum wake threshold to ensure device wakes even if bench-tested with bright light.
  constexpr uint16_t M_WAKE_THRESHOLD_MAX { 200 };

  // Valid measurement timeout.
  constexpr uint16_t M_VALID_MEAS_TIMEOUT_MS { 500 };

  // BLE timeouts.
  constexpr uint32_t M_BLE_CONNECT_TIMEOUT_MS { 10000 };
  constexpr uint32_t M_BLE_NUS_ENABLED_TIMEOUT_MS { 5000 };
  constexpr uint32_t M_BLE_READY_TIMEOUT_MS { 5000 };

  // ========== Retained Memory (survives System OFF) ==========

  // Structure to hold data that persists across System OFF.
  // Uses Zephyr's retained_mem driver for proper nRF5340 support.
  // CRC32 used to validate data integrity.
  struct RetainedData {
    CalibrationData calibration;    // Light sensor calibration.
    AppState lastState;             // State before System OFF.
    uint32_t crc;                   // CRC32 for validation.
  };

  // CRC covers all fields except the CRC itself.
  constexpr size_t M_RETAINED_CRC_OFFSET { offsetof(RetainedData, crc) };

  // CRC-32/ISO-HDLC residue (what you get when CRCing data + its CRC).
  constexpr uint32_t M_CRC32_RESIDUE { 0x2144df1c };

  // Get the retained memory device from devicetree.
#if DT_NODE_HAS_STATUS_OKAY(DT_ALIAS(retainedmemdevice))
  static const struct device* const s_retainedMemDevice {
    DEVICE_DT_GET(DT_ALIAS(retainedmemdevice))
  };
  constexpr bool M_HAS_RETAINED_MEM { true };
#else
  static const struct device* const s_retainedMemDevice { nullptr };
  constexpr bool M_HAS_RETAINED_MEM { false };
#endif

  // Local copy of retained data (read from/written to retained memory).
  static RetainedData s_retained;

  // ========== Free Functions ==========

  const char* appStateToString(AppState state)
  {
    switch (state) {
      case AppState::PowerOff:   return "PowerOff";
      case AppState::Activating: return "Activating";
      case AppState::NoCentral:  return "NoCentral";
      case AppState::Standby:    return "Standby";
      case AppState::Ready:      return "Ready";
      case AppState::Monitoring: return "Monitoring";
      default:                   return "Unknown";
    }
  }

  const char* wakeSourceToString(WakeSource source)
  {
    switch (source) {
      case WakeSource::PowerOn:        return "PowerOn";
      case WakeSource::Button:         return "Button";
      case WakeSource::LightInterrupt: return "LightInterrupt";
      default:                         return "Unknown";
    }
  }

  // ========== Constructor ==========

  App::App()
    : m_state { AppState::PowerOff }
    , m_wakeSource { WakeSource::PowerOn }
    , m_calibration {}
    , m_doorOpenLight { 0 }
  {
  }

  // ========== Init ==========

  bool App::Init()
  {
    LOG_INF("========================================");
    LOG_INF("   ALC FRIDGE LIGHT MONITOR v0.2.0");
    LOG_INF("   (Activation State Machine)");
    LOG_INF("========================================");

    // Configure button GPIO.
    if (!gpio_is_ready_dt(&s_button)) {
      LOG_ERR("Button GPIO not ready!");
      return false;
    }
    gpio_pin_configure_dt(&s_button, GPIO_INPUT);

    // Configure light INT GPIO.
    if (!gpio_is_ready_dt(&s_lightInt)) {
      LOG_ERR("Light INT GPIO not ready!");
      return false;
    }
    gpio_pin_configure_dt(&s_lightInt, GPIO_INPUT);

    // Configure LED pins.
    if (!gpio_is_ready_dt(&s_ledRed) ||
        !gpio_is_ready_dt(&s_ledGreen) ||
        !gpio_is_ready_dt(&s_ledBlue)) {
      LOG_ERR("LED GPIO not ready!");
      return false;
    }
    gpio_pin_configure_dt(&s_ledRed, GPIO_OUTPUT_INACTIVE);
    gpio_pin_configure_dt(&s_ledGreen, GPIO_OUTPUT_INACTIVE);
    gpio_pin_configure_dt(&s_ledBlue, GPIO_OUTPUT_INACTIVE);

    // Detect wake source.
    m_wakeSource = detectWakeSource();
    LOG_INF("Wake source: %s", wakeSourceToString(m_wakeSource));

    // Load calibration from retained memory.
    loadCalibration();

    LOG_INF("Application initialised.");
    return true;
  }

  // ========== Run ==========

  void App::Run()
  {
    LOG_INF("Running with wake source: %s", wakeSourceToString(m_wakeSource));

    switch (m_wakeSource) {
      case WakeSource::PowerOn:
        handlePowerOnWake();
        break;

      case WakeSource::Button:
        handleButtonWake();
        break;

      case WakeSource::LightInterrupt:
        handleLightInterruptWake();
        break;
    }

    // After handling wake event, enter System OFF.
    LOG_INF("Wake handling complete.");
    enterSystemOff();
  }

  // ========== Wake Source Detection ==========

  WakeSource App::detectWakeSource()
  {
    // Read and clear reset reason.
    uint32_t resetReason { nrfx_reset_reason_get() };
    nrfx_reset_reason_clear(resetReason);

    LOG_INF("Reset reason register: 0x%08x", resetReason);

    // Check if this was a System OFF wake.
    if (resetReason & NRFX_RESET_REASON_OFF_MASK) {
      LOG_INF("Woke from System OFF via GPIO.");

      // Use GPIO latch registers to detect which pin triggered the wake.
      // Button is P1.14 (absolute pin 32+14=46), Light INT is P1.05 (32+5=37).
      constexpr uint32_t M_BUTTON_PIN_ABS { 32 + 14 };  // P1.14
      constexpr uint32_t M_LIGHT_INT_PIN_ABS { 32 + 5 };  // P1.05

      uint32_t buttonLatch { nrf_gpio_pin_latch_get(M_BUTTON_PIN_ABS) };
      uint32_t lightLatch { nrf_gpio_pin_latch_get(M_LIGHT_INT_PIN_ABS) };

      // Also read current GPIO states for debugging.
      int buttonState { gpio_pin_get_raw(s_button.port, s_button.pin) };
      int lightState { gpio_pin_get_raw(s_lightInt.port, s_lightInt.pin) };

      LOG_INF("GPIO latches: Button=%u, Light=%u", buttonLatch, lightLatch);
      LOG_INF("GPIO states:  Button(P1.14)=%d, Light(P1.05)=%d",
              buttonState, lightState);

      // Clear latches after reading.
      nrf_gpio_pin_latch_clear(M_BUTTON_PIN_ABS);
      nrf_gpio_pin_latch_clear(M_LIGHT_INT_PIN_ABS);

      // Check latches to determine wake source.
      // Button latch set = button was pressed (even if now released).
      if (buttonLatch) {
        LOG_INF("Button latch set - button wake.");
        return WakeSource::Button;
      }

      // Light INT latch set = light threshold exceeded.
      if (lightLatch) {
        LOG_INF("Light INT latch set - light interrupt wake.");
        return WakeSource::LightInterrupt;
      }

      // Fallback: if no latch set, use current GPIO state.
      LOG_WRN("No latch set - using current GPIO state.");
      if (buttonState == 0) {
        return WakeSource::Button;
      }
      // Light INT is active-low, so physical LOW = interrupt asserted.
      if (lightState == 0) {
        return WakeSource::LightInterrupt;
      }

      // Default to light interrupt if we can't tell.
      return WakeSource::LightInterrupt;
    }

    // Check other reset reasons for debugging.
    if (resetReason & NRFX_RESET_REASON_RESETPIN_MASK) {
      LOG_INF("Reset pin triggered.");
    }
    if (resetReason & NRFX_RESET_REASON_SREQ_MASK) {
      LOG_INF("Soft reset (NVIC).");
    }
    if (resetReason & NRFX_RESET_REASON_CTRLAP_MASK) {
      LOG_INF("Debug interface reset.");
    }
    if (resetReason == 0) {
      LOG_INF("Power-on reset (no flags).");
    }

    return WakeSource::PowerOn;
  }

  // ========== Wake Handlers ==========

  void App::handlePowerOnWake()
  {
    LOG_INF("========================================");
    LOG_INF("       FRESH BOOT / POWER ON");
    LOG_INF("========================================");

    // Visual indicator of boot.
    ledBlink(2, 50, 50);

    // Check if we have valid calibration.
    if (isCalibrated()) {
      // Device is calibrated - configure light sensor for motion detection.
      LOG_INF("Device is calibrated - configuring for light wake.");
      if (initLightSensor()) {
        configureLightSensorForWake();
      }
      setState(AppState::Ready);
    } else {
      // Device not calibrated - wait for button press.
      LOG_INF("Device not calibrated - waiting for button press.");
      LOG_INF("Press button to activate.");
      setState(AppState::Standby);
    }

    // Brief delay for logs.
    k_msleep(100);
  }

  void App::handleButtonWake()
  {
    LOG_INF("========================================");
    LOG_INF("           BUTTON WAKE");
    LOG_INF("========================================");

    if (isCalibrated()) {
      // Device is calibrated - check for factory reset (10s hold).
      if (checkFactoryReset()) {
        // Factory reset triggered - calibration cleared.
        LOG_INF("Factory reset complete.");
        setState(AppState::Standby);
      } else {
        // Short press on calibrated device - just go back to sleep.
        LOG_INF("Short press - returning to System OFF.");
        setState(AppState::Ready);
      }
    } else {
      // Device is uncalibrated - run activation sequence.
      LOG_INF("Device uncalibrated - starting activation.");

      // Start LED pulsing amber to indicate activation in progress.
      ledPulseAmber();

      // Run activation sequence.
      bool success { runActivationSequence() };

      ledStopPulse();

      if (success) {
        // Activation successful - brief green flash.
        ledSetGreen();
        k_msleep(1000);
        ledOff();
        setState(AppState::Ready);
        LOG_INF("Activation complete - device is ready.");
      } else {
        // Activation failed - red LED for 3 seconds.
        ledSetRed();
        k_msleep(3000);
        ledOff();
        setState(AppState::Standby);
        LOG_INF("Activation failed - will retry on next button press.");
      }
    }
  }

  bool App::checkFactoryReset()
  {
    // Check if button is still held and wait for factory reset threshold.
    // Button is active-low with pull-up, so pressed = raw value 0.
    constexpr uint32_t M_CHECK_INTERVAL_MS { 100 };
    uint32_t heldMs { 0 };

    // Show amber LED while checking for long press.
    ledSetAmber();

    while (gpio_pin_get_raw(s_button.port, s_button.pin) == 0) {
      k_msleep(M_CHECK_INTERVAL_MS);
      heldMs += M_CHECK_INTERVAL_MS;

      if (heldMs >= M_BUTTON_FACTORY_RESET_MS) {
        // Factory reset triggered!
        LOG_INF("Factory reset triggered (held %u ms).", heldMs);

        // Clear calibration.
        clearCalibration();

        // Confirm with amber LED blinks.
        ledOff();
        for (int i = 0; i < 3; i++) {
          k_msleep(200);
          ledSetAmber();
          k_msleep(200);
          ledOff();
        }

        LOG_INF("Calibration cleared - entering System OFF.");
        return true;
      }

      // Log progress every second.
      if (heldMs % 1000 == 0) {
        LOG_INF("Button held: %u ms / %u ms for factory reset.",
                heldMs, M_BUTTON_FACTORY_RESET_MS);
      }
    }

    // Button released before factory reset threshold.
    LOG_INF("Button released after %u ms.", heldMs);
    ledOff();
    return false;
  }

  void App::handleLightInterruptWake()
  {
    LOG_INF("========================================");
    LOG_INF("       LIGHT INTERRUPT WAKE");
    LOG_INF("========================================");

    // Check if device is calibrated.
    if (!isCalibrated()) {
      LOG_WRN("Device not calibrated - ignoring light wake.");
      LOG_INF("Press button to activate device first.");
      return;
    }

    setState(AppState::Monitoring);

    // Send door open event and go back to sleep.
    sendDoorOpenAndSleep();

    setState(AppState::Ready);
  }

  // ========== Activation Sequence ==========

  bool App::runActivationSequence()
  {
    LOG_INF("Starting activation sequence...");
    setState(AppState::Activating);

    // LED off for BLE and light operations.
    ledOff();

    // Step 1: Find gateway and establish connection.
    if (!findGateway()) {
      LOG_ERR("Gateway not found!");
      setState(AppState::NoCentral);
      // Red LED for 3 seconds to indicate failure.
      ledSetRed();
      k_msleep(3000);
      ledOff();
      return false;
    }

    // Step 2: Calibrate light sensor (two-phase: open then closed).
    if (!calibrateLightSensor()) {
      LOG_ERR("Light calibration failed!");
      // Error LED already shown by calibrateLightSensor().
      s_ble.Disconnect();
      return false;
    }

    // Step 3: Save calibration to persistent storage.
    saveCalibration();

    // Step 4: Disconnect BLE.
    s_ble.Disconnect();
    k_msleep(100);

    // Step 5: Configure light sensor for wake.
    configureLightSensorForWake();

    LOG_INF("Activation sequence complete.");
    return true;
  }

  bool App::findGateway()
  {
    LOG_INF("Searching for gateway...");

    // Start BLE init (non-blocking).
    int result { s_ble.StartInit(nullptr) };
    if (result < 0) {
      LOG_ERR("BLE init failed: %d!", result);
      return false;
    }

    // Wait for BLE to be ready.
    if (!s_ble.WaitForReady(M_BLE_READY_TIMEOUT_MS)) {
      LOG_ERR("BLE init timeout!");
      return false;
    }

    result = s_ble.CompleteInit();
    if (result < 0) {
      LOG_ERR("BLE complete init failed: %d!", result);
      return false;
    }

    result = s_ble.StartAdvertising();
    if (result < 0) {
      LOG_ERR("Advertising start failed: %d!", result);
      return false;
    }

    // Wait for connection.
    LOG_INF("Waiting for gateway connection...");
    if (!s_ble.WaitForConnection(M_ACTIVATION_TIMEOUT_MS)) {
      LOG_WRN("Connection timeout - gateway not in range.");
      return false;
    }

    LOG_INF("Gateway connected.");

    // Wait for NUS to be enabled.
    if (!s_ble.WaitForNusEnabled(M_BLE_NUS_ENABLED_TIMEOUT_MS)) {
      LOG_WRN("NUS enable timeout.");
      s_ble.Disconnect();
      return false;
    }

    LOG_INF("NUS enabled - gateway ready.");
    return true;
  }

  bool App::calibrateLightSensor()
  {
    LOG_INF("Starting two-phase light calibration...");

    int8_t rssi { s_ble.GetRssi() };

    // LED MUST be off during light measurement.
    ledOff();

    // Initialise light sensor.
    if (!initLightSensor()) {
      sendCalibrationEvent(static_cast<uint8_t>(CalibrationStatus::ErrorSensorFailure), rssi);
      ledSetRed();
      k_msleep(3000);
      ledOff();
      return false;
    }

    // Wait for measurement to be ready.
    if (!s_light.WaitForValid(M_VALID_MEAS_TIMEOUT_MS)) {
      LOG_WRN("Timeout waiting for valid measurement.");
    }

    // ========== Phase 1: Sample door-open light level ==========
    LOG_INF("Phase 1: Sampling door-open light level...");

    uint16_t doorOpenLight { sampleLightLevel() };
    if (doorOpenLight < 10) {
      // Door-open should have significant light. A very low reading suggests
      // the door isn't actually open or there's a sensor issue.
      LOG_ERR("Door-open light level too low: %u (expected > 10)!", doorOpenLight);
      sendCalibrationEvent(static_cast<uint8_t>(CalibrationStatus::ErrorSensorFailure), rssi);
      ledSetRed();
      k_msleep(3000);
      ledOff();
      return false;
    }

    LOG_INF("Door-open light level: green=%u.", doorOpenLight);
    m_calibration.doorOpenLight = doorOpenLight;

    // ========== Signal user to close door ==========
    // Green LED for 5 seconds tells user: "Close the door now."
    LOG_INF("Signalling user to close door (green LED 5s)...");
    ledSetGreen();
    k_msleep(5000);
    ledOff();

    // Wait 500ms before starting closed sampling.
    k_msleep(500);

    // ========== Phase 2: Wait for door close and sample closed level ==========
    LOG_INF("Phase 2: Waiting for door to close (timeout %u ms)...", M_CALIBRATION_TIMEOUT_MS);

    uint32_t startTime { k_uptime_get_32() };
    uint16_t closeThreshold { static_cast<uint16_t>((doorOpenLight * M_DOOR_CLOSE_THRESHOLD_PERCENT) / 100) };
    uint8_t consecutiveLowReadings { 0 };

    LOG_INF("Door-close detection threshold: green < %u (30%% of %u).", closeThreshold, doorOpenLight);

    while ((k_uptime_get_32() - startTime) < M_CALIBRATION_TIMEOUT_MS) {
      // Sample light level (5 readings averaged).
      // Note: 0 is a valid reading in a dark fridge, not an error.
      uint16_t currentLight { sampleLightLevel() };

      LOG_INF("Current light: green=%u (threshold=%u, consecutive=%u).",
              currentLight, closeThreshold, consecutiveLowReadings);

      if (currentLight < closeThreshold) {
        consecutiveLowReadings++;

        if (consecutiveLowReadings >= M_DOOR_CLOSE_CONSECUTIVE) {
          // Door closed detected! Use the average of the consecutive low readings.
          LOG_INF("Door close detected!");

          // Sample the closed light level (already low, just get a fresh reading).
          uint16_t doorClosedLight { sampleLightLevel() };

          LOG_INF("Door-closed light level: green=%u.", doorClosedLight);
          m_calibration.doorClosedLight = doorClosedLight;

          // Validate: closed level must be significantly lower than open level.
          uint16_t maxClosedLevel { static_cast<uint16_t>((doorOpenLight * M_VALID_CALIBRATION_PERCENT) / 100) };
          if (doorClosedLight >= maxClosedLevel) {
            LOG_ERR("Calibration invalid: closed=%u >= %u%% of open=%u!",
                    doorClosedLight, M_VALID_CALIBRATION_PERCENT, doorOpenLight);
            sendCalibrationEvent(static_cast<uint8_t>(CalibrationStatus::ErrorLevelsTooClose), rssi);
            ledSetRed();
            k_msleep(3000);
            ledOff();
            return false;
          }

          // Calculate wake threshold.
          m_calibration.wakeThreshold = calculateWakeThreshold(doorOpenLight, doorClosedLight);
          m_calibration.txPowerDbm = 0;  // Default to full power for fridge.
          m_calibration.valid = M_CALIBRATION_VALID_MAGIC;

          LOG_INF("Calibration complete: open=%u, closed=%u, wake=%u.",
                  m_calibration.doorOpenLight, m_calibration.doorClosedLight,
                  m_calibration.wakeThreshold);

          // Send success calibration event.
          sendCalibrationEvent(static_cast<uint8_t>(CalibrationStatus::Success), rssi);

          // No LED on success - just proceed to sleep.
          return true;
        }
      } else {
        // Reset consecutive count if light goes back up.
        consecutiveLowReadings = 0;
      }

      // Poll interval between samples.
      k_msleep(M_POLL_INTERVAL_MS);
    }

    // Timeout - door was never closed.
    LOG_ERR("Calibration timeout - door was not closed within %u ms!", M_CALIBRATION_TIMEOUT_MS);
    sendCalibrationEvent(static_cast<uint8_t>(CalibrationStatus::ErrorTimeout), rssi);

    // Amber blink to indicate timeout.
    for (int i = 0; i < 3; i++) {
      ledSetAmber();
      k_msleep(200);
      ledOff();
      k_msleep(200);
    }

    return false;
  }

  // ========== Normal Operation ==========

  void App::sendDoorOpenAndSleep()
  {
    // The BH1749 interrupt triggered, telling us the light level crossed a threshold.
    // We don't need to re-read the sensor - just check which direction we were detecting.
    //
    // If we were detecting door-open (high threshold): light went HIGH -> send OPEN event.
    // If we were detecting door-close (low threshold): light went LOW -> just reconfigure.

    // Initialise sensor to reconfigure thresholds (resets it but we need I2C access).
    if (!initLightSensor()) {
      LOG_ERR("Sensor init failed!");
      return;
    }

    // Quick read to determine if this is door-open or door-close.
    // The sensor was just reset, so read what we can (may be stale but direction is clear).
    LightReading reading {};
    s_light.Read(reading);
    m_doorOpenLight = reading.green;

    // If light is low, this was a door-close wake. Reconfigure for door-open and sleep.
    if (m_doorOpenLight < m_calibration.wakeThreshold) {
      LOG_INF("Door closed (green=%u) - reconfiguring for door-open detection.", m_doorOpenLight);
      configureLightSensorForWake();
      return;
    }

    // Door is open - send OPEN event.
    LOG_INF("Door opened (green=%u) - sending OPEN event.", m_doorOpenLight);

    // Start BLE.
    int bleResult { s_ble.StartInit(nullptr) };
    if (bleResult < 0) {
      LOG_ERR("BLE start init failed: %d!", bleResult);
    }

    // Connect to gateway and send OPEN event.
    bool sent { false };
    int8_t rssi { 0 };

    if (bleResult == 0) {
      if (s_ble.WaitForReady(M_BLE_READY_TIMEOUT_MS)) {
        if (s_ble.CompleteInit() == 0) {
          if (s_ble.StartAdvertising() == 0) {
            if (s_ble.WaitForConnection(M_BLE_CONNECT_TIMEOUT_MS)) {
              if (s_ble.WaitForNusEnabled(M_BLE_NUS_ENABLED_TIMEOUT_MS)) {
                rssi = s_ble.GetRssi();
                sent = sendOpenEvent(rssi);
                if (sent) {
                  LOG_INF("Sent OPEN packet (rssi=%d dBm).", rssi);
                }
              }
            }
          }
        }
      }
    }

    if (!sent) {
      LOG_WRN("Failed to send OPEN event - BLE not ready.");
    }

    // Disconnect BLE.
    s_ble.Disconnect();
    k_msleep(100);

    // Configure sensor to wake on door CLOSE (light drops below threshold).
    // This prevents immediate re-wake while door is still open.
    configureLightSensorForDoorClose();
  }

  bool App::sendOpenEvent(int8_t rssi)
  {
    FridgeGatewayPacket packet {};
    memset(&packet, 0, sizeof(packet));
    memcpy(packet.deviceId, M_DEVICE_ID, sizeof(packet.deviceId));
    strncpy(packet.deviceName, CONFIG_BT_DEVICE_NAME, sizeof(packet.deviceName) - 1);
    packet.eventType = static_cast<uint8_t>(FridgeEventType::DoorOpen);
    packet.durationSecs = 0;
    // Light data fields left as 0 - we don't read the sensor on wake anymore.
    packet.green = m_doorOpenLight;  // Include the quick read we did for diagnostics.
    packet.rssiDbm = rssi;

    int result { s_ble.SendEvent(reinterpret_cast<const uint8_t*>(&packet), sizeof(packet)) };
    return (result == 0);
  }

  // ========== Hardware Configuration ==========

  bool App::initLightSensor()
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

  void App::configureLightSensorForWake()
  {
    // For System OFF wake, we only want to trigger on light INCREASE (door open).
    // Set low threshold to 0 so only high threshold can trigger.
    constexpr uint16_t M_LOW_THRESHOLD_DISABLED { 0 };

    int result { s_light.ConfigureInterrupt(m_calibration.wakeThreshold, M_LOW_THRESHOLD_DISABLED) };
    if (result < 0) {
      LOG_ERR("Failed to configure interrupt thresholds: %d!", result);
      return;
    }

    result = s_light.EnableInterrupt(true);
    if (result < 0) {
      LOG_ERR("Failed to enable interrupt: %d!", result);
      return;
    }

    // Clear any pending interrupt.
    s_light.ClearInterrupt();

    LOG_INF("Wake configured: trigger on green > %u (door open detection).", m_calibration.wakeThreshold);
  }

  void App::configureLightSensorForDoorClose()
  {
    // After door open event, configure to wake on light DECREASE (door close).
    // Set high threshold to 0xFFFF (disabled) so only low threshold can trigger.
    // Use the calibrated closed level as the low threshold.
    constexpr uint16_t M_HIGH_THRESHOLD_DISABLED { 0xFFFF };

    // Use a threshold slightly above the calibrated closed level to ensure reliable detection.
    // The door-close threshold should be below the wake threshold but above the closed level.
    uint16_t closeThreshold { static_cast<uint16_t>(m_calibration.wakeThreshold / 2) };
    if (closeThreshold < M_WAKE_THRESHOLD_MIN) {
      closeThreshold = M_WAKE_THRESHOLD_MIN;
    }

    int result { s_light.ConfigureInterrupt(M_HIGH_THRESHOLD_DISABLED, closeThreshold) };
    if (result < 0) {
      LOG_ERR("Failed to configure door-close thresholds: %d!", result);
      return;
    }

    result = s_light.EnableInterrupt(true);
    if (result < 0) {
      LOG_ERR("Failed to enable interrupt: %d!", result);
      return;
    }

    // Clear any pending interrupt.
    s_light.ClearInterrupt();

    LOG_INF("Wake configured: trigger on green < %u (door close detection).", closeThreshold);
  }

  // ========== LED Feedback ==========

  void App::ledSetAmber()
  {
    gpio_pin_set_dt(&s_ledRed, 1);
    gpio_pin_set_dt(&s_ledGreen, 1);
    gpio_pin_set_dt(&s_ledBlue, 0);
  }

  void App::ledSetRed()
  {
    gpio_pin_set_dt(&s_ledRed, 1);
    gpio_pin_set_dt(&s_ledGreen, 0);
    gpio_pin_set_dt(&s_ledBlue, 0);
  }

  void App::ledSetGreen()
  {
    gpio_pin_set_dt(&s_ledRed, 0);
    gpio_pin_set_dt(&s_ledGreen, 1);
    gpio_pin_set_dt(&s_ledBlue, 0);
  }

  void App::ledOff()
  {
    gpio_pin_set_dt(&s_ledRed, 0);
    gpio_pin_set_dt(&s_ledGreen, 0);
    gpio_pin_set_dt(&s_ledBlue, 0);
  }

  void App::ledBlink(int count, int onMs, int offMs)
  {
    for (int i = 0; i < count; i++) {
      ledSetRed();
      k_msleep(onMs);
      ledOff();
      if (i < count - 1) {
        k_msleep(offMs);
      }
    }
  }

  void App::ledPulseAmber()
  {
    // TODO: Implement proper PWM pulsing using pwm_leds.
    // For now, just set solid amber.
    ledSetAmber();
  }

  void App::ledStopPulse()
  {
    ledOff();
  }

  // ========== Power Management ==========

  void App::configureWakeSources()
  {
    LOG_INF("Configuring wake sources for System OFF...");

    // Only enable light wake if device is calibrated.
    if (isCalibrated()) {
      // Dump BH1749 interrupt registers for diagnostics.
      s_light.DumpInterruptRegisters();

      // Read a fresh light value to see current level.
      LightReading reading {};
      if (s_light.Read(reading) == 0) {
        LOG_INF("Current light: green=%u (wake threshold=%u)", reading.green, m_calibration.wakeThreshold);
      }

      // Clear any pending interrupt just before configuring GPIO.
      s_light.ClearInterrupt();

      // Read INT pin state after clearing.
      int intPinRaw = gpio_pin_get_raw(s_lightInt.port, s_lightInt.pin);
      LOG_INF("INT pin after clear: raw=%d (expect 1=HIGH=idle)", intPinRaw);

      // Configure light INT sense for wake on LOW (INT asserted = light detected).
      // BH1749 INT is active-low: goes LOW when threshold exceeded.
      int result = gpio_pin_interrupt_configure_dt(&s_lightInt, GPIO_INT_LEVEL_LOW);
      if (result < 0) {
        LOG_ERR("Failed to configure light wake: %d!", result);
      } else {
        LOG_INF("Light wake enabled: sense LOW, threshold=%u", m_calibration.wakeThreshold);
      }
    } else {
      LOG_INF("Device NOT calibrated - light wake disabled.");
      gpio_pin_interrupt_configure_dt(&s_lightInt, GPIO_INT_DISABLE);
    }

    // Configure button for wake (sense LOW since button is active-low).
    int result = gpio_pin_interrupt_configure_dt(&s_button, GPIO_INT_LEVEL_LOW);
    if (result < 0) {
      LOG_ERR("Failed to configure button wake: %d!", result);
    } else {
      LOG_INF("Button wake enabled.");
    }
  }

  void App::enterSystemOff()
  {
    LOG_INF("Preparing for System OFF...");

    // Configure wake sources.
    configureWakeSources();

    LOG_INF("Entering System OFF...");

    // Give logger time to flush.
    k_msleep(100);

    // sys_poweroff() does not return.
    sys_poweroff();

    // Should never reach here.
    LOG_ERR("sys_poweroff() returned!");
  }

  // ========== State Management ==========

  void App::setState(AppState newState)
  {
    if (m_state == newState) {
      return;
    }

    LOG_INF("State: %s -> %s", appStateToString(m_state), appStateToString(newState));
    m_state = newState;
  }

  // ========== Calibration Persistence ==========

  bool App::loadCalibration()
  {
    LOG_INF("Loading calibration from retained memory...");

    if constexpr (!M_HAS_RETAINED_MEM) {
      LOG_WRN("Retained memory not configured - calibration will not persist.");
      return false;
    }

    // Check retained memory device is ready.
    if (!device_is_ready(s_retainedMemDevice)) {
      LOG_ERR("Retained memory device not ready!");
      return false;
    }

    // Read retained data from device.
    int result { retained_mem_read(s_retainedMemDevice, 0,
                                   reinterpret_cast<uint8_t*>(&s_retained),
                                   sizeof(s_retained)) };
    if (result < 0) {
      LOG_ERR("Failed to read retained memory: %d!", result);
      return false;
    }

    // Validate CRC.
    uint32_t crc { crc32_ieee(reinterpret_cast<const uint8_t*>(&s_retained),
                              M_RETAINED_CRC_OFFSET + sizeof(s_retained.crc)) };

    if (crc != M_CRC32_RESIDUE) {
      LOG_INF("Retained memory CRC invalid (computed residue=0x%08x, expected=0x%08x).",
              crc, M_CRC32_RESIDUE);
      memset(&s_retained, 0, sizeof(s_retained));
      return false;
    }

    LOG_INF("Retained memory CRC valid.");

    // Check if calibration is valid.
    if (s_retained.calibration.valid != M_CALIBRATION_VALID_MAGIC) {
      LOG_INF("No valid calibration in retained memory.");
      return false;
    }

    // Copy calibration to local member.
    m_calibration = s_retained.calibration;

    LOG_INF("Calibration loaded: wakeThreshold=%u, doorOpenLight=%u, TX=%d dBm",
            m_calibration.wakeThreshold, m_calibration.doorOpenLight,
            m_calibration.txPowerDbm);
    return true;
  }

  void App::saveCalibration()
  {
    LOG_INF("Saving calibration to retained memory...");

    if constexpr (!M_HAS_RETAINED_MEM) {
      LOG_WRN("Retained memory not configured - calibration will not persist.");
      return;
    }

    // Check retained memory device is ready.
    if (!device_is_ready(s_retainedMemDevice)) {
      LOG_ERR("Retained memory device not ready!");
      return;
    }

    // Copy calibration to retained structure.
    s_retained.calibration = m_calibration;

    // Compute CRC over data (excluding CRC field itself).
    uint32_t crc { crc32_ieee(reinterpret_cast<const uint8_t*>(&s_retained),
                              M_RETAINED_CRC_OFFSET) };
    s_retained.crc = sys_cpu_to_le32(crc);

    // Write to retained memory.
    int result { retained_mem_write(s_retainedMemDevice, 0,
                                    reinterpret_cast<const uint8_t*>(&s_retained),
                                    sizeof(s_retained)) };
    if (result < 0) {
      LOG_ERR("Failed to write retained memory: %d!", result);
      return;
    }

    LOG_INF("Calibration saved: wakeThreshold=%u, doorOpenLight=%u, TX=%d dBm",
            m_calibration.wakeThreshold, m_calibration.doorOpenLight,
            m_calibration.txPowerDbm);
  }

  void App::clearCalibration()
  {
    LOG_INF("Clearing calibration...");
    m_calibration = {};
    memset(&s_retained, 0, sizeof(s_retained));

    if constexpr (M_HAS_RETAINED_MEM) {
      // Write zeroed data to retained memory to invalidate.
      if (device_is_ready(s_retainedMemDevice)) {
        retained_mem_write(s_retainedMemDevice, 0,
                           reinterpret_cast<const uint8_t*>(&s_retained),
                           sizeof(s_retained));
      }
    }
  }

  bool App::isCalibrated() const
  {
    return m_calibration.valid == M_CALIBRATION_VALID_MAGIC;
  }

  // ========== Calibration Events ==========

  bool App::sendCalibrationEvent(uint8_t status, int8_t rssi)
  {
    LOG_INF("Sending calibration event: status=%u.", status);

    FridgeGatewayPacket packet {};
    memset(&packet, 0, sizeof(packet));
    memcpy(packet.deviceId, M_DEVICE_ID, sizeof(packet.deviceId));
    strncpy(packet.deviceName, CONFIG_BT_DEVICE_NAME, sizeof(packet.deviceName) - 1);
    packet.eventType = static_cast<uint8_t>(FridgeEventType::Calibration);
    packet.durationSecs = status;  // Repurpose durationSecs for status code.
    packet.red = m_calibration.doorOpenLight;    // Repurpose for diagnostic data.
    packet.green = m_calibration.doorClosedLight;
    packet.blue = m_calibration.wakeThreshold;
    packet.ir = 0;
    packet.rssiDbm = rssi;

    int result { s_ble.SendEvent(reinterpret_cast<const uint8_t*>(&packet), sizeof(packet)) };
    if (result == 0) {
      LOG_INF("Sent CALIBRATION packet: status=%u, open=%u, closed=%u, wake=%u, rssi=%d dBm.",
              status, m_calibration.doorOpenLight, m_calibration.doorClosedLight,
              m_calibration.wakeThreshold, rssi);
    }
    return (result == 0);
  }

  // ========== Helper Functions ==========

  uint16_t App::sampleLightLevel()
  {
    // Take M_SAMPLES_PER_READING samples and return the average green value.
    // Note: Returns 0 for both "very dark" and "all samples failed" cases.
    // This is acceptable because 0 is below any reasonable threshold.
    constexpr uint32_t M_SAMPLE_INTERVAL_MS { 130 };  // BH1749 at 120ms mode + margin.

    uint32_t sumGreen { 0 };
    uint8_t validSamples { 0 };

    for (uint8_t i = 0; i < M_SAMPLES_PER_READING; i++) {
      LightReading reading {};
      int result { s_light.Read(reading) };
      if (result == 0) {
        sumGreen += reading.green;
        validSamples++;
      } else {
        LOG_WRN("Sample %u failed: %d", i, result);
      }
      if (i < M_SAMPLES_PER_READING - 1) {
        k_msleep(M_SAMPLE_INTERVAL_MS);
      }
    }

    if (validSamples == 0) {
      LOG_ERR("All %u samples failed in sampleLightLevel!", M_SAMPLES_PER_READING);
      return 0;
    }

    return static_cast<uint16_t>(sumGreen / validSamples);
  }

  uint16_t App::calculateWakeThreshold(uint16_t doorOpenLight, uint16_t doorClosedLight)
  {
    // Wake threshold = closed level + 30% of the range between closed and open.
    // This ensures we wake on any meaningful light increase above the closed baseline.
    uint16_t range { static_cast<uint16_t>(doorOpenLight - doorClosedLight) };
    uint16_t offset { static_cast<uint16_t>((range * M_WAKE_THRESHOLD_OFFSET_PERCENT) / 100) };
    uint16_t threshold { static_cast<uint16_t>(doorClosedLight + offset) };

    // Clamp to sensible bounds.
    if (threshold < M_WAKE_THRESHOLD_MIN) {
      threshold = M_WAKE_THRESHOLD_MIN;
    }
    if (threshold > M_WAKE_THRESHOLD_MAX) {
      threshold = M_WAKE_THRESHOLD_MAX;
    }

    LOG_INF("Wake threshold calculation: closed=%u + (%u * 30%%) = %u (clamped to %u-%u).",
            doorClosedLight, range, doorClosedLight + offset, M_WAKE_THRESHOLD_MIN, M_WAKE_THRESHOLD_MAX);

    return threshold;
  }

} // namespace fridge
