#pragma once

#include <cstdint>
#include <zephyr/drivers/i2c.h>

namespace fridge {

struct __packed LightReading {
  uint8_t  deviceId;
  uint8_t  reserved;
  uint16_t red;
  uint16_t green;
  uint16_t blue;
  uint16_t ir;
};

enum class Bh1749Gain : uint8_t {
  Gain1x  = 0,
  Gain32x = 1,
};

/**
 * @brief Direct I2C driver for BH1749 — bypasses Zephyr sensor API to avoid SYS_INIT.
 *
 * Provides light measurement and interrupt threshold configuration for fridge
 * door open/close detection via light level changes.
 */
class Bh1749Light {
public:
  explicit Bh1749Light(const struct i2c_dt_spec* i2cSpec);

  int Init();
  int SetGain(Bh1749Gain rgbGain, Bh1749Gain irGain);
  int Read(LightReading& reading);

  /**
   * @brief Configure interrupt thresholds for door detection.
   * @param thresholdHigh Upper threshold (light-on detection for wake).
   * @param thresholdLow Lower threshold (light-off detection for door close).
   * @return 0 on success, negative errno on failure.
   */
  int ConfigureInterrupt(uint16_t thresholdHigh, uint16_t thresholdLow);

  /**
   * @brief Enable or disable the interrupt output.
   * @param enable True to enable interrupt, false to disable.
   * @return 0 on success, negative errno on failure.
   */
  int EnableInterrupt(bool enable);

  /**
   * @brief Clear the interrupt latch.
   * @return 0 on success, negative errno on failure.
   */
  int ClearInterrupt();

  /**
   * @brief Check if current green channel reading exceeds threshold.
   * @param threshold Threshold value to compare against.
   * @return True if green reading >= threshold.
   */
  bool IsLightAboveThreshold(uint16_t threshold);

  /**
   * @brief Get the most recent green channel reading.
   * @return Green channel value from last Read() call.
   */
  uint16_t GetGreen() const { return m_lastGreen; }

  /**
   * @brief Dump all interrupt-related registers for diagnostics.
   * Logs INTERRUPT (0x60), PERSISTENCE (0x61), and threshold registers.
   * @return 0 on success, negative errno on failure.
   */
  int DumpInterruptRegisters();

  /**
   * @brief Read the INTERRUPT register and return its value.
   * @param status Output parameter for the register value.
   * @return 0 on success, negative errno on failure.
   */
  int ReadInterruptStatus(uint8_t& status);

private:
  const struct i2c_dt_spec* m_i2c;
  bool m_ready { false };
  uint16_t m_lastGreen { 0 };

  int readRegister(uint8_t reg, uint8_t& value);
  int writeRegister(uint8_t reg, uint8_t value);
  int readRegisters(uint8_t startReg, uint8_t* buf, size_t len);
  int writeRegisters(uint8_t startReg, const uint8_t* buf, size_t len);
};

} // namespace fridge
