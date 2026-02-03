#include <zephyr/kernel.h>
#include <zephyr/drivers/i2c.h>
#include <zephyr/logging/log.h>

#include "bh1749_light.hpp"

LOG_MODULE_REGISTER(bh1749_light, LOG_LEVEL_INF);

namespace drawer {

// ========== Register Addresses ==========

constexpr uint8_t M_REG_SYSTEM_CONTROL   { 0x40 };
constexpr uint8_t M_REG_MODE_CONTROL1    { 0x41 };
constexpr uint8_t M_REG_MODE_CONTROL2    { 0x42 };
constexpr uint8_t M_REG_RED_DATA_LSB     { 0x50 };
constexpr uint8_t M_REG_MANUFACTURER_ID  { 0x92 };

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

int Bh1749Light::readRegisters(uint8_t startReg, uint8_t* buf, size_t len)
{
  return i2c_burst_read_dt(m_i2c, startReg, buf, len);
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
  uint8_t sysCtrl;
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
  if (!m_ready) { return -ENODEV; }

  // Check if data is valid.
  // Reading MODE_CONTROL2 clears the VALID flag, so only read once.
  uint8_t ctrl2;
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
  uint8_t buf[10];
  result = readRegisters(M_REG_RED_DATA_LSB, buf, sizeof(buf));
  if (result < 0) {
    LOG_ERR("Failed to read color data: %d!", result);
    return result;
  }

  reading.red   = static_cast<uint16_t>(buf[0] | (buf[1] << 8));
  reading.green = static_cast<uint16_t>(buf[2] | (buf[3] << 8));
  reading.blue  = static_cast<uint16_t>(buf[4] | (buf[5] << 8));
  reading.ir    = static_cast<uint16_t>(buf[8] | (buf[9] << 8));

  return 0;
}

} // namespace drawer
