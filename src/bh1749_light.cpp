#include <zephyr/kernel.h>
#include <zephyr/drivers/i2c.h>
#include <zephyr/logging/log.h>

#include "bh1749_light.hpp"

LOG_MODULE_REGISTER(bh1749_light, LOG_LEVEL_INF);

namespace fridge 
{
  // ========== Register Addresses ==========

  constexpr uint8_t M_REG_SYSTEM_CONTROL   { 0x40 };
  constexpr uint8_t M_REG_MODE_CONTROL1    { 0x41 };
  constexpr uint8_t M_REG_MODE_CONTROL2    { 0x42 };
  constexpr uint8_t M_REG_RED_DATA_LSB     { 0x50 };
  constexpr uint8_t M_REG_MANUFACTURER_ID  { 0x92 };

  // Interrupt registers.
  constexpr uint8_t M_REG_INTERRUPT        { 0x60 };
  constexpr uint8_t M_REG_PERSISTENCE      { 0x61 };
  constexpr uint8_t M_REG_TH_HIGH_LSB      { 0x62 };
  constexpr uint8_t M_REG_TH_HIGH_MSB      { 0x63 };
  constexpr uint8_t M_REG_TH_LOW_LSB       { 0x64 };
  constexpr uint8_t M_REG_TH_LOW_MSB       { 0x65 };

  // ========== Register Values ==========

  // SYSTEM_CONTROL.
  constexpr uint8_t M_PART_ID_MASK    { 0x3F };
  constexpr uint8_t M_PART_ID         { 0x0D };
  constexpr uint8_t M_SW_RESET        { 0x80 };

  // MODE_CONTROL1.
  constexpr uint8_t M_MEAS_MODE_35MS  { 0x05 };
  constexpr uint8_t M_MEAS_MODE_120MS { 0x02 };
  constexpr uint8_t M_RGB_GAIN_1X     { 0x01 << 3 };
  constexpr uint8_t M_RGB_GAIN_32X    { 0x03 << 3 };
  constexpr uint8_t M_IR_GAIN_1X      { 0x01 << 5 };
  constexpr uint8_t M_IR_GAIN_32X     { 0x03 << 5 };

  // MODE_CONTROL2.
  constexpr uint8_t M_RGB_EN          { 0x10 };
  constexpr uint8_t M_VALID_MASK      { 0x80 };

  // INTERRUPT register bits.
  constexpr uint8_t M_INT_ENABLE       { 0x01 };
  constexpr uint8_t M_INT_SOURCE_RED   { 0x00 << 2 };  // Red channel as interrupt source.
  constexpr uint8_t M_INT_SOURCE_GREEN { 0x01 << 2 };  // Green channel as interrupt source.
  constexpr uint8_t M_INT_SOURCE_BLUE  { 0x02 << 2 };  // Blue channel as interrupt source.
  constexpr uint8_t M_INT_LATCH_YES    { 0x00 << 4 };  // Latched interrupt mode.
  constexpr uint8_t M_INT_LATCH_NO     { 0x01 << 4 };  // Non-latched interrupt mode.

  // PERSISTENCE register values.
  constexpr uint8_t M_PERSISTENCE_ACTIVE_END { 0x00 };  // After each measurement.
  constexpr uint8_t M_PERSISTENCE_UPDATE_END { 0x01 };  // Update measurement end.
  constexpr uint8_t M_PERSISTENCE_4_SAMPLES  { 0x02 };  // 4 consecutive samples.
  constexpr uint8_t M_PERSISTENCE_8_SAMPLES  { 0x03 };  // 8 consecutive samples.

  // MANUFACTURER_ID.
  constexpr uint8_t M_MANUFACTURER_ID { 0xE0 };

  // ========== Constructor ==========

  Bh1749Light::Bh1749Light(const struct i2c_dt_spec* i2cSpec)
      : m_i2c(i2cSpec)
  {
  }

  // ========== Private Methods ==========

  int Bh1749Light::readRegister(uint8_t reg, uint8_t& value)
  {
    return i2c_reg_read_byte_dt(m_i2c, reg, &value);
  }

  int Bh1749Light::writeRegister(uint8_t reg, uint8_t value)
  {
    return i2c_reg_write_byte_dt(m_i2c, reg, value);
  }

  int Bh1749Light::readRegisters(uint8_t startReg, uint8_t* buffer, size_t length)
  {
    return i2c_burst_read_dt(m_i2c, startReg, buffer, length);
  }

  int Bh1749Light::writeRegisters(uint8_t startReg, const uint8_t* buffer, size_t length)
  {
    return i2c_burst_write_dt(m_i2c, startReg, buffer, length);
  }

  // ========== Public Methods ==========

  int Bh1749Light::Init()
  {
    if (!device_is_ready(m_i2c->bus)) {
      LOG_ERR("I2C bus not ready!");
      return -ENODEV;
    }

    // Software reset to ensure clean state.
    int result { writeRegister(M_REG_SYSTEM_CONTROL, M_SW_RESET) };
    if (result < 0) {
      LOG_ERR("Failed to reset BH1749: %d!", result);
      return result;
    }

    // Wait for reset to complete.
    k_msleep(10);

    // Verify manufacturer ID.
    uint8_t mfgId;
    result = readRegister(M_REG_MANUFACTURER_ID, mfgId);
    if (result < 0) {
      LOG_ERR("Failed to read manufacturer ID: %d!", result);
      return result;
    }

    if (mfgId != M_MANUFACTURER_ID) {
      LOG_ERR("Wrong manufacturer ID: 0x%02X (expected 0x%02X)!", mfgId, M_MANUFACTURER_ID);
      return -ENODEV;
    }

    // Verify part ID.
    uint8_t sysCtrl { };
    result = readRegister(M_REG_SYSTEM_CONTROL, sysCtrl);
    if (result < 0) {
      LOG_ERR("Failed to read system control: %d!", result);
      return result;
    }

    uint8_t partId { static_cast<uint8_t>(sysCtrl & M_PART_ID_MASK) };
    if (partId != M_PART_ID) {
      LOG_ERR("Wrong part ID: 0x%02X (expected 0x%02X)!", partId, M_PART_ID);
      return -ENODEV;
    }

    // Enable RGB measurement first (Zephyr driver order).
    result = writeRegister(M_REG_MODE_CONTROL2, M_RGB_EN);
    if (result < 0) {
      LOG_ERR("Failed to enable RGB: %d!", result);
      return result;
    }

    // Set configuration: 32x gain, 120ms mode.
    // 0x02 = 120ms, 0x18 = RGB 32x (bits 4:3 = 11), 0x60 = IR 32x (bits 6:5 = 11)
    constexpr uint8_t MODE_CONTROL1_32X { 0x02 | 0x18 | 0x60 };  // = 0x7A
    result = writeRegister(M_REG_MODE_CONTROL1, MODE_CONTROL1_32X);
    if (result < 0) {
      LOG_ERR("Failed to set mode control 1: %d!", result);
      return result;
    }

    // Clear any pending interrupt from power-on.
    // The BH1749 thresholds default to 0x0000, so any measured value will trigger
    // the interrupt immediately. Reading register 0x60 clears the latch.
    uint8_t intStatus;
    result = readRegister(M_REG_INTERRUPT, intStatus);
    if (result < 0) {
      LOG_WRN("Failed to read/clear interrupt register: %d.", result);
      // Non-fatal, continue.
    } else {
      LOG_INF("Initial INT register: 0x%02X (cleared by reading).", intStatus);
    }

    // Set safe default thresholds to prevent immediate interrupt assertion.
    // High threshold = 0xFFFF (max), Low threshold = 0x0000 (min).
    // This means: interrupt only if value > 65535 (impossible) or < 0 (impossible).
    uint8_t safeThresholds[4] {
      0xFF, 0xFF,  // High threshold LSB, MSB = 0xFFFF.
      0x00, 0x00   // Low threshold LSB, MSB = 0x0000.
    };
    result = writeRegisters(M_REG_TH_HIGH_LSB, safeThresholds, 4);
    if (result < 0) {
      LOG_WRN("Failed to set safe thresholds: %d.", result);
      // Non-fatal, continue.
    } else {
      LOG_INF("Safe thresholds set (high=0xFFFF, low=0x0000).");
    }

    m_ready = true;
    LOG_INF("BH1749 initialised (mfg=0x%02X, part=0x%02X).", mfgId, partId);
    return 0;
  }

  int Bh1749Light::SetGain(Bh1749Gain rgbGain, Bh1749Gain irGain)
  {
    if (!m_ready) { return -ENODEV; }

    uint8_t regVal { M_MEAS_MODE_120MS };
    regVal |= (rgbGain == Bh1749Gain::Gain32x) ? M_RGB_GAIN_32X : M_RGB_GAIN_1X;
    regVal |= (irGain == Bh1749Gain::Gain32x) ? M_IR_GAIN_32X : M_IR_GAIN_1X;

    int result { writeRegister(M_REG_MODE_CONTROL1, regVal) };
    if (result < 0) {
      LOG_ERR("Failed to set gain: %d!", result);
      return result;
    }

    // Clear any stale VALID flag by reading the data registers.
    // This ensures the next Read() waits for fresh data.
    uint8_t dummy[10];
    readRegisters(M_REG_RED_DATA_LSB, dummy, sizeof(dummy));

    LOG_INF("BH1749 gain set: RGB=%s, IR=%s.",
            (rgbGain == Bh1749Gain::Gain32x) ? "32x" : "1x",
            (irGain == Bh1749Gain::Gain32x) ? "32x" : "1x");

    return 0;
  }

  int Bh1749Light::Read(LightReading& reading)
  {
    constexpr int OK { 0 };

    if (!m_ready) { return -ENODEV; }

    // Check if data is valid.
    // Reading MODE_CONTROL2 clears the VALID flag, so only read once.
    uint8_t ctrl2 { };
    int result { readRegister(M_REG_MODE_CONTROL2, ctrl2) };
    if (result < 0) {
      LOG_ERR("Failed to read MODE_CONTROL2: %d!", result);
      return result;
    }

    LOG_DBG("MODE_CONTROL2: 0x%02X", ctrl2);

    if (!(ctrl2 & M_VALID_MASK)) {
      LOG_INF("Data not valid yet (ctrl2=0x%02X).", ctrl2);
      return -EAGAIN;
    }

    // Read all color data (R, G, B, skip 2 bytes, IR) = 10 bytes from 0x50.
    // Layout: RED_L, RED_H, GREEN_L, GREEN_H, BLUE_L, BLUE_H, (2 reserved), IR_L, IR_H.
    uint8_t buffer[10] { };
    result = readRegisters(M_REG_RED_DATA_LSB, buffer, sizeof(buffer));
    if (result < 0) {
      LOG_ERR("Failed to read color data: %d!", result);
      return result;
    }

    reading.red   = static_cast<uint16_t>(buffer[0] | (buffer[1] << 8));
    reading.green = static_cast<uint16_t>(buffer[2] | (buffer[3] << 8));
    reading.blue  = static_cast<uint16_t>(buffer[4] | (buffer[5] << 8));
    reading.ir    = static_cast<uint16_t>(buffer[8] | (buffer[9] << 8));

    // Store green for threshold checks.
    m_lastGreen = reading.green;

    return OK;
  }

  int Bh1749Light::ConfigureInterrupt(uint16_t thresholdHigh, uint16_t thresholdLow)
  {
    constexpr int OK { 0 };

    if (!m_ready) { return -ENODEV; }

    // Write high threshold (little-endian).
    uint8_t thHighBuf[2] {
      static_cast<uint8_t>(thresholdHigh & 0xFF),
      static_cast<uint8_t>((thresholdHigh >> 8) & 0xFF)
    };
    int result { writeRegisters(M_REG_TH_HIGH_LSB, thHighBuf, 2) };
    if (result < 0) {
      LOG_ERR("Failed to set high threshold: %d!", result);
      return result;
    }

    // Write low threshold (little-endian).
    uint8_t thLowBuf[2] {
      static_cast<uint8_t>(thresholdLow & 0xFF),
      static_cast<uint8_t>((thresholdLow >> 8) & 0xFF)
    };
    result = writeRegisters(M_REG_TH_LOW_LSB, thLowBuf, 2);
    if (result < 0) {
      LOG_ERR("Failed to set low threshold: %d!", result);
      return result;
    }

    // Set persistence to trigger after each measurement for testing.
    result = writeRegister(M_REG_PERSISTENCE, M_PERSISTENCE_ACTIVE_END);
    if (result < 0) {
      LOG_ERR("Failed to set persistence: %d!", result);
      return result;
    }

    // Read back to verify.
    constexpr size_t VERIFY_LENGTH { 4 };
    uint8_t verifyBuffer[VERIFY_LENGTH];
    readRegisters(M_REG_TH_HIGH_LSB, verifyBuffer, VERIFY_LENGTH);
    uint16_t readHigh { static_cast<uint16_t>(verifyBuffer[0] | (verifyBuffer[1] << 8)) };
    uint16_t readLow { static_cast<uint16_t>(verifyBuffer[2] | (verifyBuffer[3] << 8)) };

    LOG_INF("BH1749 thresholds: high=%u (read=%u), low=%u (read=%u), persist=1.",
            thresholdHigh, readHigh, thresholdLow, readLow);

    return OK;
  }

  int Bh1749Light::EnableInterrupt(bool enable)
  {
    constexpr int OK { 0 };

    if (!m_ready) { return -ENODEV; }

    // Configure interrupt: green channel source, latched mode.
    // Latched mode keeps INT asserted until cleared by reading register 0x60.
    uint8_t intVal { 0 };
    if (enable) {
      intVal = M_INT_ENABLE | M_INT_SOURCE_GREEN | M_INT_LATCH_YES;
    }

    int result { writeRegister(M_REG_INTERRUPT, intVal) };
    if (result < 0) {
      LOG_ERR("Failed to %s interrupt: %d!", enable ? "enable" : "disable", result);
      return result;
    }

    LOG_INF("BH1749 interrupt %s.", enable ? "enabled" : "disabled");

    return OK;
  }

  int Bh1749Light::ClearInterrupt()
  {
    constexpr int OK { 0 };

    if (!m_ready) { return -ENODEV; }

    // Clear interrupt by READING register 0x60 (INTERRUPT).
    uint8_t intStatus;
    int result { readRegister(M_REG_INTERRUPT, intStatus) };
    if (result < 0) {
      LOG_ERR("Failed to clear interrupt: %d!", result);
      return result;
    }

    LOG_INF("BH1749 interrupt cleared (reg 0x60 was 0x%02X).", intStatus);

    return OK;
  }

  bool Bh1749Light::IsLightAboveThreshold(uint16_t threshold)
  {
    return m_lastGreen >= threshold;
  }

  int Bh1749Light::DumpInterruptRegisters()
  {
    constexpr int OK { 0 };

    if (!m_ready) { return -ENODEV; }

    // Read INTERRUPT register (0x60).
    uint8_t intReg;
    int result { readRegister(M_REG_INTERRUPT, intReg) };
    if (result < 0) {
      LOG_ERR("Failed to read INTERRUPT (0x60): %d!", result);
      return result;
    }

    // Read PERSISTENCE register (0x61).
    uint8_t persReg;
    result = readRegister(M_REG_PERSISTENCE, persReg);
    if (result < 0) {
      LOG_ERR("Failed to read PERSISTENCE (0x61): %d!", result);
      return result;
    }

    // Read threshold registers (0x62-0x65).
    uint8_t threshBuf[4];
    result = readRegisters(M_REG_TH_HIGH_LSB, threshBuf, 4);
    if (result < 0) {
      LOG_ERR("Failed to read thresholds: %d!", result);
      return result;
    }

    uint16_t thHigh { static_cast<uint16_t>(threshBuf[0] | (threshBuf[1] << 8)) };
    uint16_t thLow { static_cast<uint16_t>(threshBuf[2] | (threshBuf[3] << 8)) };

    LOG_INF("BH1749 INT regs: INT=0x%02X, PERS=0x%02X, TH_H=%u, TH_L=%u",
            intReg, persReg, thHigh, thLow);

    // Decode INTERRUPT register bits.
    bool intEn { (intReg & M_INT_ENABLE) != 0 };
    uint8_t intSrc { static_cast<uint8_t>((intReg >> 2) & 0x03) };
    bool latch { (intReg & M_INT_LATCH_NO) == 0 };  // 0 = latched, 1 = non-latched.

    const char* srcNames[] = { "RED", "GREEN", "BLUE", "??" };
    LOG_INF("  INT enable=%d, source=%s, latched=%d", intEn, srcNames[intSrc], latch);

    return OK;
  }

  int Bh1749Light::ReadInterruptStatus(uint8_t& status)
  {
    if (!m_ready) { return -ENODEV; }
    return readRegister(M_REG_INTERRUPT, status);
  }

  } // namespace fridge
