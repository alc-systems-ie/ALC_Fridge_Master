#pragma once

/**
 * @file app.hpp
 * @brief ALC Fridge Light Monitor Application State Machine.
 *
 * Manages the device lifecycle from activation through normal operation:
 *
 * States:
 * - PowerOff:    Deep sleep (System OFF), waiting for button or light wake.
 * - Activating:  User pressed button, searching for gateway, calibrating.
 * - NoCentral:   Gateway not found during activation.
 * - Standby:     Activation failed or uncalibrated, waiting for button retry.
 * - Ready:       Activated and calibrated, waiting for light wake.
 * - Monitoring:  Light detected (door open), monitoring for close.
 *
 * Button (sw0 / SW2 on Thingy:53 silkscreen, P1.14):
 * - Short press from PowerOff/Standby: Start activation.
 * - Long press (3s) from any state: Power off.
 * - Very long press (10s): Factory reset (clear calibration).
 *
 * Light sensor (BH1749 INT on P1.05):
 * - Wake from System OFF when light exceeds calibrated threshold.
 * - Send OPEN event, monitor for door close, send CLOSE event with duration.
 * - Alert if door open > timeout.
 */

#include <cstdint>

namespace fridge
{
  // ========== Application States ==========

  enum class AppState : uint8_t {
    PowerOff,       // System OFF, waiting for wake.
    Activating,     // Button pressed, finding gateway + calibrating.
    NoCentral,      // Gateway not found.
    Standby,        // Failed activation, waiting for retry.
    Ready,          // Activated, waiting for light wake.
    Monitoring      // Door open, monitoring for close.
  };

  // ========== Wake Source Identification ==========

  enum class WakeSource : uint8_t {
    PowerOn,        // Fresh boot (power-on reset, debug reset, etc.).
    Button,         // User pressed button (sw0).
    LightInterrupt  // BH1749 light threshold exceeded (door opened).
  };

  // ========== Calibration Data ==========

  struct CalibrationData {
    uint16_t wakeThreshold;     // Light level to trigger wake (green channel).
    uint16_t doorOpenLight;     // Measured door-open light level (green channel).
    uint16_t doorClosedLight;   // Measured door-closed light level (green channel).
    int8_t txPowerDbm;          // Calibrated TX power.
    uint8_t valid;              // Magic number to detect uninitialised.
  };

  // Magic value indicating valid calibration.
  constexpr uint8_t M_CALIBRATION_VALID_MAGIC { 0xCA };

  // ========== Timing Constants ==========

  constexpr uint32_t M_BUTTON_LONG_PRESS_MS { 3000 };      // Power off threshold.
  constexpr uint32_t M_BUTTON_FACTORY_RESET_MS { 10000 };  // Factory reset threshold.
  constexpr uint32_t M_ACTIVATION_TIMEOUT_MS { 30000 };    // Gateway search timeout.
  constexpr uint32_t M_CALIBRATION_TIMEOUT_MS { 300000 };  // 5 minutes for door close.
  constexpr uint32_t M_DOOR_OPEN_TIMEOUT_SECS { 60 };      // Alert threshold (testing).
  constexpr uint32_t M_POLL_INTERVAL_MS { 500 };           // Door monitoring poll interval.

  // ========== Calibration Constants ==========

  constexpr uint8_t M_SAMPLES_PER_READING { 5 };           // Samples averaged per reading.
  constexpr uint8_t M_DOOR_CLOSE_CONSECUTIVE { 3 };        // Consecutive low readings to confirm closed.
  constexpr uint8_t M_DOOR_CLOSE_THRESHOLD_PERCENT { 30 }; // % of open level to detect closed.
  constexpr uint8_t M_VALID_CALIBRATION_PERCENT { 50 };    // Max closed/open ratio for valid cal.
  constexpr uint8_t M_WAKE_THRESHOLD_OFFSET_PERCENT { 30 };// Wake threshold: closed + 30% of range.
  constexpr uint32_t M_DOOR_SETTLE_TIME_MS { 10000 };      // Wait time after door close detected.

  // ========== Application Class ==========

  class App
  {
    public:
      App();

      /**
       * @brief Initialise application hardware.
       * @return true on success.
       */
      bool Init();

      /**
       * @brief Run the application state machine.
       *
       * Handles the current wake event and transitions to appropriate state.
       * May enter System OFF at the end.
       */
      void Run();

      /**
       * @brief Get the wake source that caused this boot.
       * @return The detected wake source.
       */
      WakeSource GetWakeSource() const { return m_wakeSource; }

      /**
       * @brief Get the current application state.
       * @return The current state.
       */
      AppState GetState() const { return m_state; }

    private:
      // ========== State Handlers ==========

      void handlePowerOnWake();
      void handleButtonWake();
      void handleLightInterruptWake();

      // ========== Button Handling ==========

      bool checkFactoryReset();

      // ========== Activation Sequence ==========

      bool runActivationSequence();
      bool findGateway();
      bool calibrateLightSensor();

      // ========== Normal Operation ==========

      void monitorDoor();
      bool sendOpenEvent(int8_t rssi);
      bool sendCloseEvent(uint16_t durationSecs, int8_t rssi);
      bool sendAlertEvent(uint16_t durationSecs, int8_t rssi);

      // ========== Calibration Events ==========

      bool sendCalibrationEvent(uint8_t status, int8_t rssi);

      // ========== Hardware Configuration ==========

      bool initLightSensor();
      void configureLightSensorForWake();

      // ========== Wake Source Detection ==========

      WakeSource detectWakeSource();

      // ========== LED Feedback ==========

      void ledSetAmber();
      void ledSetRed();
      void ledSetGreen();
      void ledOff();
      void ledBlink(int count, int onMs, int offMs);
      void ledPulseAmber();
      void ledStopPulse();

      // ========== Power Management ==========

      void configureWakeSources();
      void enterSystemOff();

      // ========== State Management ==========

      void setState(AppState newState);

      // ========== Calibration Persistence ==========

      bool loadCalibration();
      void saveCalibration();
      void clearCalibration();
      bool isCalibrated() const;

      // ========== Helper Functions ==========

      uint16_t calculateWakeThreshold(uint16_t doorOpenLight, uint16_t doorClosedLight);
      uint16_t sampleLightLevel();  // Takes M_SAMPLES_PER_READING samples and averages.

      // ========== Member Variables ==========

      AppState m_state;
      WakeSource m_wakeSource;
      CalibrationData m_calibration;
      uint32_t m_wakeTime;           // Timestamp when door opened.
      uint16_t m_doorOpenLight;      // Light level when door opened (this wake cycle).
  };

  // ========== Free Functions ==========

  const char* appStateToString(AppState state);
  const char* wakeSourceToString(WakeSource source);

} // namespace fridge
