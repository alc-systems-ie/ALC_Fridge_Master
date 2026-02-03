/*
 * Drawer Activity Sensor for Thingy:53 (nRF5340)
 *
 * Wakes from System OFF on ADXL362 motion detection, reads BH1749 light sensor,
 * transmits data over BLE NUS to gateway, then returns to System OFF.
 */

#include <zephyr/kernel.h>
#include <zephyr/sys/poweroff.h>
#include <zephyr/device.h>
#include <zephyr/drivers/gpio.h>
#include <zephyr/drivers/spi.h>
#include <zephyr/drivers/i2c.h>
#include <zephyr/logging/log.h>
#include <helpers/nrfx_reset_reason.h>

#include <string.h>

#include "adxl362.hpp"
#include "bh1749_light.hpp"
#include "ble_nus.hpp"

LOG_MODULE_REGISTER(main, LOG_LEVEL_INF);

// ========== Hardware Definitions ==========

// ADXL362 INT1 pin (P0.19 on Thingy:53).
static const struct gpio_dt_spec s_wakePin =
    GPIO_DT_SPEC_GET(DT_NODELABEL(adxl362), int1_gpios);

// ADXL362 SPI bus from devicetree.
static const struct spi_dt_spec s_accelSpi =
    SPI_DT_SPEC_GET(DT_NODELABEL(adxl362), SPI_WORD_SET(8) | SPI_TRANSFER_MSB, 0);

// BH1749 I2C bus from devicetree.
static const struct i2c_dt_spec s_lightI2c =
    I2C_DT_SPEC_GET(DT_NODELABEL(bh1749));

// Status LED (red).
static const struct gpio_dt_spec s_led =
    GPIO_DT_SPEC_GET(DT_ALIAS(led0), gpios);

// ========== Constants ==========

// 10 minutes at 100 Hz = 60,000 samples.
constexpr uint16_t M_INACTIVITY_TIME_SAMPLES { 60000 };

// BLE timeouts.
constexpr uint32_t M_BLE_CONNECT_TIMEOUT_MS { 10000 };
constexpr uint32_t M_BLE_NUS_ENABLED_TIMEOUT_MS { 5000 };

// Device ID for this sensor node.
constexpr uint8_t M_DEVICE_ID { 1 };

// Dummy config to force AWAKE=0 during init.
static constexpr drawer::Adxl362::ActivityConfig M_DUMMY_CONFIG {
  .activityMode         = drawer::Adxl362::ActivityMode::Absolute,
  .inactivityMode       = drawer::Adxl362::ActivityMode::Absolute,
  .linkLoop             = drawer::Adxl362::LinkLoopMode::Loop,
  .activityThresholdMg  = 1,
  .activityTime         = 0,
  .inactivityThresholdMg = 2000,
  .inactivityTime       = 1
};

// Real config for motion detection.
static constexpr drawer::Adxl362::ActivityConfig M_MOTION_CONFIG {
  .activityMode         = drawer::Adxl362::ActivityMode::Referenced,
  .inactivityMode       = drawer::Adxl362::ActivityMode::Referenced,
  .linkLoop             = drawer::Adxl362::LinkLoopMode::Loop,
  .activityThresholdMg  = 120,
  .activityTime         = 2,
  .inactivityThresholdMg = 150,
  .inactivityTime       = M_INACTIVITY_TIME_SAMPLES
};

// ========== Static State ==========

static drawer::Adxl362 s_accel { &s_accelSpi };
static drawer::Bh1749Light s_light { &s_lightI2c };
static drawer::BleNus s_ble;

// Stored light reading for BLE transmission.
static drawer::LightReading s_lightReading {};

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

static void configureWakePin(bool senseHigh)
{
  if (!gpio_is_ready_dt(&s_wakePin)) {
    LOG_ERR("Wake pin device not ready!");
    return;
  }

  // Configure as input.
  int ret { gpio_pin_configure_dt(&s_wakePin, GPIO_INPUT) };
  if (ret < 0) {
    LOG_ERR("Failed to configure wake pin: %d!", ret);
    return;
  }

  // Configure sense based on current pin state.
  // If senseHigh=true: wake when pin goes HIGH (normal activity detection).
  // If senseHigh=false: wake when pin goes LOW (after inactivity clears AWAKE).
  gpio_flags_t flags { senseHigh ? GPIO_INT_LEVEL_HIGH : GPIO_INT_LEVEL_LOW };
  ret = gpio_pin_interrupt_configure_dt(&s_wakePin, flags);
  if (ret < 0) {
    LOG_ERR("Failed to configure wake interrupt: %d!", ret);
    return;
  }

  uint8_t port { static_cast<uint8_t>(s_wakePin.port == DEVICE_DT_GET(DT_NODELABEL(gpio0)) ? 0 : 1) };
  LOG_INF("Wake pin configured (P%u.%02u, sense %s).", port, s_wakePin.pin, senseHigh ? "HIGH" : "LOW");
}

static void enterSystemOff()
{
  LOG_INF("Preparing for System OFF...");

  // Check current INT1 level.
  int pinLevel { gpio_pin_get_dt(&s_wakePin) };
  LOG_INF("INT1 level: %d.", pinLevel);

  // Configure sense based on current pin state:
  // - If INT1 is HIGH (AWAKE=1): sense LOW to wake when inactivity clears AWAKE.
  // - If INT1 is LOW (AWAKE=0): sense HIGH to wake on next activity.
  // This allows immediate sleep without waiting for the inactivity timeout.
  bool senseHigh { pinLevel == 0 };

  LOG_INF("Entering System OFF (will wake on %s edge)...", senseHigh ? "rising" : "falling");

  // Give the logger time to flush.
  k_msleep(100);

  configureWakePin(senseHigh);

  // sys_poweroff() does not return — the chip resets on wake.
  sys_poweroff();

  LOG_ERR("sys_poweroff() returned!");
}

static void configureAdxl362()
{
  LOG_INF("Configuring ADXL362...");

  int result { s_accel.Init() };
  if (result < 0) {
    LOG_ERR("ADXL362 init failed: %d!", result);
    return;
  }

  result = s_accel.SetRange(drawer::Adxl362::Range::Range2g);
  if (result < 0) { return; }

  result = s_accel.SetOdr(drawer::Adxl362::ODR::Odr100);
  if (result < 0) { return; }

  // Step 1: Configure with dummy thresholds to force AWAKE=0.
  result = s_accel.ConfigureActivity(M_DUMMY_CONFIG);
  if (result < 0) { return; }

  result = s_accel.MapAwakeToInt1();
  if (result < 0) { return; }

  result = s_accel.StartMeasurement();
  if (result < 0) { return; }

  // Wait for AWAKE to clear.
  LOG_INF("Waiting for AWAKE to clear...");
  k_msleep(200);

  while (s_accel.IsAwake()) {
    LOG_INF("AWAKE still high, waiting...");
    k_msleep(100);
  }
  LOG_INF("AWAKE cleared.");

  // Step 2: Reconfigure with real thresholds.
  result = s_accel.ConfigureActivity(M_MOTION_CONFIG);
  if (result < 0) { return; }

  LOG_INF("ADXL362 configured with %u sample inactivity timeout.", M_INACTIVITY_TIME_SAMPLES);
}

static void configureLightSensor()
{
  LOG_INF("Configuring BH1749...");

  int result { s_light.Init() };
  if (result < 0) {
    LOG_ERR("BH1749 init failed: %d!", result);
    return;
  }

  // Init() now configures 1x gain, 120ms mode (same as Zephyr driver defaults).
  LOG_INF("BH1749 configured.");
}

static bool readLight()
{
  int result { s_light.Read(s_lightReading) };
  if (result < 0) {
    LOG_ERR("Light read failed: %d!", result);
    return false;
  }

  s_lightReading.deviceId = M_DEVICE_ID;
  LOG_INF("Light: R=%u G=%u B=%u IR=%u.",
          s_lightReading.red, s_lightReading.green, s_lightReading.blue, s_lightReading.ir);
  return true;
}

static void onNusRx(const uint8_t* data, uint16_t length)
{
  // Gateway sends 4-byte timestamp.
  if (length == sizeof(uint32_t)) {
    uint32_t timestamp;
    memcpy(&timestamp, data, sizeof(timestamp));
    LOG_INF("Received timestamp: %u.", timestamp);
  }
}

static void sendLightReading()
{
  LOG_INF("Initialising BLE...");

  int result { s_ble.Init(onNusRx) };
  if (result < 0) {
    LOG_ERR("BLE init failed: %d!", result);
    return;
  }

  result = s_ble.StartAdvertising();
  if (result < 0) {
    LOG_ERR("Advertising start failed: %d!", result);
    return;
  }

  LOG_INF("Waiting for gateway connection...");
  if (!s_ble.WaitForConnection(M_BLE_CONNECT_TIMEOUT_MS)) {
    LOG_WRN("Connection timeout, skipping BLE transmission.");
    return;
  }

  if (!s_ble.WaitForNusEnabled(M_BLE_NUS_ENABLED_TIMEOUT_MS)) {
    LOG_WRN("NUS enable timeout.");
    s_ble.Disconnect();
    return;
  }

  // Small delay for gateway to send timestamp (optional).
  k_msleep(100);

  // Send light reading (10 bytes).
  result = s_ble.SendEvent(reinterpret_cast<const uint8_t*>(&s_lightReading),
                           sizeof(s_lightReading));
  if (result < 0) {
    LOG_ERR("Failed to send light reading: %d!", result);
  } else {
    LOG_INF("Light reading sent to gateway.");
  }

  // Brief delay before disconnect.
  k_msleep(100);
  s_ble.Disconnect();
  k_msleep(100);
}

// ========== Main ==========

int main()
{
  // Read and clear the reset reason immediately.
  uint32_t resetReason { nrfx_reset_reason_get() };
  nrfx_reset_reason_clear(resetReason);

  LOG_INF("=== ADXL362 System OFF Wake Test ===");
  logResetReason(resetReason);

  // Configure LED.
  gpio_pin_configure_dt(&s_led, GPIO_OUTPUT_INACTIVE);

  // Check if we woke from System OFF.
  if (resetReason & NRFX_RESET_REASON_OFF_MASK) {
    // Check INT1 level to determine why we woke.
    int pinLevel { gpio_pin_get_dt(&s_wakePin) };
    LOG_INF("Woke from System OFF via GPIO! INT1 level: %d.", pinLevel);

    if (pinLevel) {
      // INT1 is HIGH = activity detected (drawer opened).
      LOG_INF("Activity wake — doing work...");

      // Init and read light sensor (LED must be OFF during measurement!).
      configureLightSensor();

      // Wait for measurement (BH1749 needs 120ms at default mode).
      k_msleep(150);

      bool lightOk { readLight() };

      // LED on to indicate BLE activity.
      gpio_pin_set_dt(&s_led, true);

      if (lightOk) {
        // Send to gateway over BLE.
        sendLightReading();
      }

      gpio_pin_set_dt(&s_led, false);
      LOG_INF("Wake work complete.");
    } else {
      // INT1 is LOW = inactivity detected (drawer settled).
      // This happens when we slept with sense LOW and AWAKE cleared.
      // Just go back to sleep with sense HIGH for next activity.
      LOG_INF("Inactivity wake — AWAKE cleared, returning to sleep.");
    }
  } else {
    LOG_INF("Normal boot (not from System OFF).");

    // Configure ADXL362 on fresh boot only.
    configureAdxl362();

    LOG_INF("Will enter System OFF in 2 seconds...");
    k_msleep(2000);
  }

  // Go (back) to System OFF.
  enterSystemOff();

  return 0;
}
