#pragma once

#include <cstdint>
#include <zephyr/drivers/i2c.h>

namespace drawer {

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

// Direct I2C driver for BH1749 — bypasses Zephyr sensor API to avoid SYS_INIT.
class Bh1749Light {
public:
  explicit Bh1749Light(const struct i2c_dt_spec* i2cSpec);

  int Init();
  int SetGain(Bh1749Gain rgbGain, Bh1749Gain irGain);
  int Read(LightReading& reading);

private:
  const struct i2c_dt_spec* m_i2c;
  bool m_ready { false };

  int readRegister(uint8_t reg, uint8_t& value);
  int writeRegister(uint8_t reg, uint8_t value);
  int readRegisters(uint8_t startReg, uint8_t* buf, size_t len);
};

} // namespace drawer
