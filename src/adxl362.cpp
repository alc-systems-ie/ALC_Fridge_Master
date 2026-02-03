#include "adxl362.hpp"

#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>

LOG_MODULE_REGISTER(adxl362, LOG_LEVEL_INF);

namespace drawer {

  // ========== SPI Command Bytes ==========

  constexpr uint8_t M_CMD_WRITE_REG  { 0x0A };
  constexpr uint8_t M_CMD_READ_REG   { 0x0B };
  constexpr uint8_t M_CMD_READ_FIFO  { 0x0D };

  // ========== Register Addresses ==========

  constexpr uint8_t M_REG_DEVID_AD        { 0x00 };
  constexpr uint8_t M_REG_DEVID_MST       { 0x01 };
  constexpr uint8_t M_REG_PARTID          { 0x02 };
  constexpr uint8_t M_REG_STATUS          { 0x0B };
  constexpr uint8_t M_REG_FIFO_L          { 0x0C };
  // constexpr uint8_t M_REG_FIFO_H          { 0x0D };
  constexpr uint8_t M_REG_XDATA_L         { 0x0E };
  constexpr uint8_t M_REG_SOFT_RESET      { 0x1F };
  constexpr uint8_t M_REG_THRESH_ACT_L    { 0x20 };
  constexpr uint8_t M_REG_TIME_ACT        { 0x22 };
  constexpr uint8_t M_REG_THRESH_INACT_L  { 0x23 };
  constexpr uint8_t M_REG_TIME_INACT_L    { 0x25 };
  constexpr uint8_t M_REG_ACT_INACT_CTL   { 0x27 };
  constexpr uint8_t M_REG_FIFO_CTL        { 0x28 };
  constexpr uint8_t M_REG_FIFO_SAMPLES    { 0x29 };
  constexpr uint8_t M_REG_INTMAP1         { 0x2A };
  constexpr uint8_t M_REG_FILTER_CTL      { 0x2C };
  constexpr uint8_t M_REG_POWER_CTL       { 0x2D };

  // ========== Bit Masks and Shifts ==========

  // STATUS (0x0B).
  constexpr uint8_t M_STATUS_DATA_READY    { BIT(0) };
  constexpr uint8_t M_STATUS_FIFO_READY    { BIT(1) };
  constexpr uint8_t M_STATUS_FIFO_WM       { BIT(2) };
  constexpr uint8_t M_STATUS_FIFO_OVERRUN  { BIT(3) };
  constexpr uint8_t M_STATUS_ACT           { BIT(4) };
  constexpr uint8_t M_STATUS_INACT         { BIT(5) };
  constexpr uint8_t M_STATUS_AWAKE         { BIT(6) };
  constexpr uint8_t M_STATUS_ERR_USER_REGS { BIT(7) };

  // ACT_INACT_CTL (0x27).
  constexpr uint8_t M_ACT_EN              { BIT(0) };
  // constexpr uint8_t M_ACT_REF             { BIT(1) };
  constexpr uint8_t M_INACT_EN            { BIT(2) };
  // constexpr uint8_t M_INACT_REF           { BIT(3) };
  constexpr uint8_t M_LINKLOOP_SHIFT      { 4 };
  // constexpr uint8_t M_LINKLOOP_MASK       { 0x30 };

  // FIFO_CTL (0x28).
  // constexpr uint8_t M_FIFO_MODE_MASK      { 0x03 };
  // constexpr uint8_t M_FIFO_TEMP           { BIT(2) };
  constexpr uint8_t M_FIFO_AH             { BIT(3) };

  // FILTER_CTL (0x2C).
  constexpr uint8_t M_ODR_MASK            { 0x07 };
  constexpr uint8_t M_RANGE_SHIFT         { 6 };
  constexpr uint8_t M_RANGE_MASK          { 0xC0 };

  // POWER_CTL (0x2D).
  constexpr uint8_t M_MEASURE_MASK        { 0x03 };
  constexpr uint8_t M_MEASURE_ON          { 0x02 };
  constexpr uint8_t M_MEASURE_STANDBY     { 0x00 };
  // constexpr uint8_t M_AUTOSLEEP           { BIT(2) };
  // constexpr uint8_t M_WAKEUP              { BIT(3) };
  // constexpr uint8_t M_LOW_NOISE_SHIFT     { 4 };
  // constexpr uint8_t M_LOW_NOISE_MASK      { 0x30 };
  // constexpr uint8_t M_EXTCLK             { BIT(6) };

  // INTMAP1 (0x2A).
  // constexpr uint8_t M_INTMAP_DATA_READY   { BIT(0) };
  // constexpr uint8_t M_INTMAP_FIFO_READY   { BIT(1) };
  // constexpr uint8_t M_INTMAP_FIFO_WM      { BIT(2) };
  // constexpr uint8_t M_INTMAP_FIFO_OVERRUN { BIT(3) };
  // constexpr uint8_t M_INTMAP_ACT          { BIT(4) };
  // constexpr uint8_t M_INTMAP_INACT        { BIT(5) };
  constexpr uint8_t M_INTMAP_AWAKE        { BIT(6) };
  // constexpr uint8_t M_INTMAP_INT_LOW      { BIT(7) };

  // FIFO data format.
  // constexpr uint16_t M_FIFO_HDR_MASK      { 0xC000 };
  // constexpr uint8_t  M_FIFO_HDR_SHIFT     { 14 };
  constexpr uint16_t M_FIFO_DATA_MASK     { 0x3FFF };
  constexpr uint16_t M_FIFO_SIGN_BIT      { 0x2000 };
  constexpr uint16_t M_FIFO_SIGN_EXTEND   { 0xC000 };

  // Scale factors (mg per LSB).
  constexpr float M_SCALE_2G { 1.0f };     // 1000 LSB/g → 1 mg/LSB.
  constexpr float M_SCALE_4G { 2.0f };     // 500 LSB/g → 2 mg/LSB.
  constexpr float M_SCALE_8G { 4.255f };   // 235 LSB/g → ~4.255 mg/LSB.

  // Threshold scale (mg per LSB, same as data for ±2g default).
  // constexpr float M_THRESH_SCALE_2G { 1.0f };
  // constexpr float M_THRESH_SCALE_4G { 2.0f };
  // constexpr float M_THRESH_SCALE_8G { 4.255f };

  // FIFO entry count mask.
  // constexpr uint16_t M_FIFO_COUNT_MASK { 0x03FF };

  // ========== Constructor ==========

  Adxl362::Adxl362(const struct spi_dt_spec* spiSpec)
      : m_spi(spiSpec)
      , m_currentRange(Range::Range2g)
  {
  }

  // ========== SPI Helpers ==========

  int Adxl362::writeRegister(uint8_t reg, uint8_t value)
  {
    uint8_t txBuf[3] { M_CMD_WRITE_REG, reg, value };
    const struct spi_buf tx { .buf = txBuf, .len = sizeof(txBuf) };
    const struct spi_buf_set txSet { .buffers = &tx, .count = 1 };

    int result { spi_write_dt(m_spi, &txSet) };
    if (result < 0) {
      LOG_ERR("SPI write failed: reg=0x%02X, err=%d!", reg, result);
    }
    return result;
  }

  int Adxl362::readRegister(uint8_t reg, uint8_t& value)
  {
    uint8_t txBuf[2] { M_CMD_READ_REG, reg };
    const struct spi_buf txSpi { .buf = txBuf, .len = sizeof(txBuf) };
    const struct spi_buf_set txSet { .buffers = &txSpi, .count = 1 };

    uint8_t rxBuf[3] { 0 };
    const struct spi_buf rxSpi { .buf = rxBuf, .len = sizeof(rxBuf) };
    const struct spi_buf_set rxSet { .buffers = &rxSpi, .count = 1 };

    int result { spi_transceive_dt(m_spi, &txSet, &rxSet) };
    if (result < 0) {
      LOG_ERR("SPI read failed: reg=0x%02X, err=%d!", reg, result);
      return result;
    }

    // Data appears after 2-byte command in rx buffer.
    value = rxBuf[2];
    return 0;
  }

  int Adxl362::readRegisters(uint8_t reg, uint8_t* buffer, size_t length)
  {
    uint8_t txBuf[2] { M_CMD_READ_REG, reg };
    const struct spi_buf txSpi { .buf = txBuf, .len = sizeof(txBuf) };
    const struct spi_buf_set txSet { .buffers = &txSpi, .count = 1 };

    // RX: 2 dummy bytes (command echo) + data.
    uint8_t rxHeader[2] { 0 };
    struct spi_buf rxBufs[2] {
      { .buf = rxHeader, .len = sizeof(rxHeader) },
      { .buf = buffer,   .len = length },
    };
    const struct spi_buf_set rxSet { .buffers = rxBufs, .count = 2 };

    int result { spi_transceive_dt(m_spi, &txSet, &rxSet) };
    if (result < 0) {
      LOG_ERR("SPI burst read failed: reg=0x%02X, len=%u, err=%d!", reg, length, result);
    }
    return result;
  }

  int Adxl362::writeRegister16(uint8_t regLow, uint16_t value)
  {
    // ADXL362 multi-byte registers are little-endian.
    uint8_t txBuf[4] {
      M_CMD_WRITE_REG,
      regLow,
      static_cast<uint8_t>(value & 0xFF),
      static_cast<uint8_t>((value >> 8) & 0xFF)
    };
    const struct spi_buf tx { .buf = txBuf, .len = sizeof(txBuf) };
    const struct spi_buf_set txSet { .buffers = &tx, .count = 1 };

    int result { spi_write_dt(m_spi, &txSet) };
    if (result < 0) {
      LOG_ERR("SPI write16 failed: reg=0x%02X, err=%d!", regLow, result);
    }
    return result;
  }

  int Adxl362::updateRegister(uint8_t reg, uint8_t value, uint8_t mask)
  {
    uint8_t current { 0 };
    int result { readRegister(reg, current) };
    if (result < 0) { return result; }

    uint8_t updated { static_cast<uint8_t>((current & ~mask) | (value & mask)) };
    return writeRegister(reg, updated);
  }

  int Adxl362::readFifoBurst(uint8_t* buffer, size_t length)
  {
    uint8_t cmd { M_CMD_READ_FIFO };
    const struct spi_buf txSpi { .buf = &cmd, .len = 1 };
    const struct spi_buf_set txSet { .buffers = &txSpi, .count = 1 };

    // RX: 1 dummy byte (command echo) + FIFO data.
    uint8_t rxDummy { 0 };
    struct spi_buf rxBufs[2] {
      { .buf = &rxDummy, .len = 1 },
      { .buf = buffer,   .len = length },
    };
    const struct spi_buf_set rxSet { .buffers = rxBufs, .count = 2 };

    int result { spi_transceive_dt(m_spi, &txSet, &rxSet) };
    if (result < 0) {
      LOG_ERR("FIFO burst read failed: len=%u, err=%d!", length, result);
    }
    return result;
  }

  // ========== Conversion Helpers ==========

  float Adxl362::getScaleMgPerLsb() const
  {
    switch (m_currentRange) {
      case Range::Range2g: return M_SCALE_2G;
      case Range::Range4g: return M_SCALE_4G;
      case Range::Range8g: return M_SCALE_8G;
      default:             return M_SCALE_2G;
    }
  }

  uint16_t Adxl362::mgToLsb(uint16_t mg) const
  {
    float scale { getScaleMgPerLsb() };
    return static_cast<uint16_t>(static_cast<float>(mg) / scale);
  }

  int Adxl362::verifyDeviceId()
  {
    uint8_t adId { 0 };
    uint8_t memsId { 0 };
    uint8_t partId { 0 };

    int result { readRegister(M_REG_DEVID_AD, adId) };
    if (result < 0) { return result; }

    result = readRegister(M_REG_DEVID_MST, memsId);
    if (result < 0) { return result; }

    result = readRegister(M_REG_PARTID, partId);
    if (result < 0) { return result; }

    if (adId != M_DEVID_AD_EXPECTED || memsId != M_DEVID_MST_EXPECTED ||
        partId != M_PARTID_EXPECTED) {
      LOG_ERR("Device ID mismatch: AD=0x%02X MST=0x%02X PART=0x%02X!", adId, memsId, partId);
      return -ENODEV;
    }

    LOG_INF("Device ID verified: AD=0x%02X MST=0x%02X PART=0x%02X.", adId, memsId, partId);
    return 0;
  }

  // ========== Initialisation ==========

  int Adxl362::Init()
  {
    if (!spi_is_ready_dt(m_spi)) {
      LOG_ERR("SPI bus not ready!");
      return -ENODEV;
    }

    // Software reset.
    int result { writeRegister(M_REG_SOFT_RESET, M_SOFT_RESET_CODE) };
    if (result < 0) {
      LOG_ERR("Soft reset failed: %d!", result);
      return result;
    }

    k_msleep(M_RESET_DELAY_MS);

    result = verifyDeviceId();
    if (result < 0) { return result; }

    LOG_INF("ADXL362 initialised.");
    return 0;
  }

  // ========== Configuration ==========

  int Adxl362::ConfigureActivity(const ActivityConfig& config)
  {
    // Activity threshold (11-bit, little-endian across two registers).
    uint16_t actLsb { static_cast<uint16_t>(mgToLsb(config.activityThresholdMg) & 0x07FF) };
    int result { writeRegister16(M_REG_THRESH_ACT_L, actLsb) };
    if (result < 0) { return result; }

    // Activity time.
    result = writeRegister(M_REG_TIME_ACT, config.activityTime);
    if (result < 0) { return result; }

    // Inactivity threshold (11-bit).
    uint16_t inactLsb { static_cast<uint16_t>(mgToLsb(config.inactivityThresholdMg) & 0x07FF) };
    result = writeRegister16(M_REG_THRESH_INACT_L, inactLsb);
    if (result < 0) { return result; }

    // Inactivity time (16-bit).
    result = writeRegister16(M_REG_TIME_INACT_L, config.inactivityTime);
    if (result < 0) { return result; }

    // ACT_INACT_CTL: enable bits, reference mode, link/loop.
    uint8_t ctlValue { static_cast<uint8_t>(
        M_ACT_EN |
        (static_cast<uint8_t>(config.activityMode) << 1) |
        M_INACT_EN |
        (static_cast<uint8_t>(config.inactivityMode) << 3) |
        (static_cast<uint8_t>(config.linkLoop) << M_LINKLOOP_SHIFT)
    )};

    result = writeRegister(M_REG_ACT_INACT_CTL, ctlValue);
    if (result < 0) { return result; }

    LOG_INF("Activity configured: act=%umg/%u, inact=%umg/%u, loop=%u.",
            config.activityThresholdMg, config.activityTime,
            config.inactivityThresholdMg, config.inactivityTime,
            static_cast<uint8_t>(config.linkLoop));
    return 0;
  }

  int Adxl362::ConfigureFifo(FifoMode mode, uint16_t watermark)
  {
    uint8_t fifoCtl { static_cast<uint8_t>(mode) };
    if (watermark & 0x100) {
      fifoCtl |= M_FIFO_AH;
    }

    int result { writeRegister(M_REG_FIFO_CTL, fifoCtl) };
    if (result < 0) { return result; }

    result = writeRegister(M_REG_FIFO_SAMPLES, static_cast<uint8_t>(watermark & 0xFF));
    if (result < 0) { return result; }

    LOG_INF("FIFO configured: mode=%u, watermark=%u.", static_cast<uint8_t>(mode), watermark);
    return 0;
  }

  int Adxl362::SetOdr(ODR odr)
  {
    int result { updateRegister(M_REG_FILTER_CTL, static_cast<uint8_t>(odr), M_ODR_MASK) };
    if (result < 0) {
      LOG_ERR("Failed to set ODR: %d!", result);
      return result;
    }

    LOG_INF("ODR set to %u.", static_cast<uint8_t>(odr));
    return 0;
  }

  int Adxl362::SetRange(Range range)
  {
    uint8_t value { static_cast<uint8_t>(static_cast<uint8_t>(range) << M_RANGE_SHIFT) };
    int result { updateRegister(M_REG_FILTER_CTL, value, M_RANGE_MASK) };
    if (result < 0) {
      LOG_ERR("Failed to set range: %d!", result);
      return result;
    }

    m_currentRange = range;

    const char* rangeStr[] { "±2g", "±4g", "±8g" };
    LOG_INF("Range set to %s.", rangeStr[static_cast<uint8_t>(range)]);
    return 0;
  }

  int Adxl362::StartMeasurement()
  {
    int result { updateRegister(M_REG_POWER_CTL, M_MEASURE_ON, M_MEASURE_MASK) };
    if (result < 0) {
      LOG_ERR("Failed to start measurement: %d!", result);
      return result;
    }

    LOG_INF("Measurement mode started.");
    return 0;
  }

  int Adxl362::StopMeasurement()
  {
    int result { updateRegister(M_REG_POWER_CTL, M_MEASURE_STANDBY, M_MEASURE_MASK) };
    if (result < 0) {
      LOG_ERR("Failed to stop measurement: %d!", result);
      return result;
    }

    LOG_INF("Standby mode entered.");
    return 0;
  }

  // ========== FIFO ==========

  int Adxl362::ReadFifoEntries(uint16_t& entries)
  {
    uint8_t buf[2] { 0 };
    int result { readRegisters(M_REG_FIFO_L, buf, 2) };
    if (result < 0) { return result; }

    entries = static_cast<uint16_t>(buf[0] | ((buf[1] & 0x03) << 8));
    return 0;
  }

  int Adxl362::ReadFifo(Sample* samples, uint16_t maxSets, uint16_t& setsRead)
  {
    constexpr int OK { 0 };
    constexpr size_t TOTAL_AXES { 3 };
    constexpr size_t BYTES_PER_AXIS { 2 } ;
    constexpr size_t TOTAL_SETS { 512 };
    constexpr size_t BYTES_PER_SET { TOTAL_AXES * BYTES_PER_AXIS };

    setsRead = 0;

    // Read FIFO entry count.
    uint16_t entries { 0 };
    int result { ReadFifoEntries(entries) };
    if (result < 0) { return result; }

    LOG_INF("FIFO entries: %u.", entries);
    if (entries == 0) { return 0; }

    // Each XYZ set = 3 entries of 2 bytes each.
    uint16_t availableSets { static_cast<uint16_t>(entries / TOTAL_AXES) };
    uint16_t setsToRead { (availableSets > maxSets) ? maxSets : availableSets };
    uint16_t bytesToRead { static_cast<uint16_t>(setsToRead * BYTES_PER_SET) };

    if (bytesToRead == 0) { return 0; }

    // Read raw FIFO data.
    uint8_t raw[TOTAL_SETS * BYTES_PER_SET];  // 3072.
    if (bytesToRead > sizeof(raw)) {
      setsToRead = sizeof(raw) / BYTES_PER_SET;
      bytesToRead = static_cast<uint16_t>(setsToRead * BYTES_PER_SET);
    }

    result = readFifoBurst(raw, bytesToRead);
    if (result < 0) {
      LOG_ERR("FIFO burst read failed: %d!", result);
      return result;
    }

    // Decode samples from FIFO format.
    // Each entry: bits[15:14] = channel ID, bits[13:0] = signed 14-bit data.
    float scale { getScaleMgPerLsb() };

    for (uint16_t i { 0 }; i < setsToRead; ++i) 
    {
      for (uint8_t axis { 0 }; axis < TOTAL_AXES; ++axis) 
      {
        uint16_t offset { static_cast<uint16_t>((i * TOTAL_AXES + axis) * BYTES_PER_AXIS) };
        uint16_t raw16 { static_cast<uint16_t>(raw[offset] | (raw[offset + 1] << 8)) };

        // Extract 14-bit signed value.
        int16_t rawVal { static_cast<int16_t>(raw16 & M_FIFO_DATA_MASK) };
        if (rawVal & M_FIFO_SIGN_BIT) {
          rawVal |= static_cast<int16_t>(M_FIFO_SIGN_EXTEND);
        }

        int16_t mg { static_cast<int16_t>(static_cast<float>(rawVal) * scale) };

        switch (axis) {
          case 0: samples[i].x = mg; break;
          case 1: samples[i].y = mg; break;
          case 2: samples[i].z = mg; break;
        }
      }
    }

    setsRead = setsToRead;
    LOG_INF("Read %u sample sets from FIFO.", setsRead);

    return OK;
  }

  // ========== Status ==========

  int Adxl362::ReadStatus(Status& status)
  {
    constexpr int OK { 0 };
    uint8_t value { 0 };
    int result { readRegister(M_REG_STATUS, value) };
    if (result < 0) {
      LOG_ERR("Failed to read status: %d!", result);
      return result;
    }

    status.dataReady     = (value & M_STATUS_DATA_READY) != 0;
    status.fifoReady     = (value & M_STATUS_FIFO_READY) != 0;
    status.fifoWatermark = (value & M_STATUS_FIFO_WM) != 0;
    status.fifoOverrun   = (value & M_STATUS_FIFO_OVERRUN) != 0;
    status.activity      = (value & M_STATUS_ACT) != 0;
    status.inactivity    = (value & M_STATUS_INACT) != 0;
    status.awake         = (value & M_STATUS_AWAKE) != 0;
    status.errUserRegs   = (value & M_STATUS_ERR_USER_REGS) != 0;

    return OK;
  }

  bool Adxl362::IsAwake()
  {
    Status status;
    if (ReadStatus(status) < 0) {
      return true;  // Default to awake on error (safer).
    }
    return status.awake;
  }

  // ========== Interrupt Mapping ==========

  int Adxl362::MapAwakeToInt1()
  {
    constexpr int OK { 0 };
    int result { writeRegister(M_REG_INTMAP1, M_INTMAP_AWAKE) };
    if (result < 0) {
      LOG_ERR("Failed to map AWAKE to INT1: %d!", result);
      return result;
    }

    LOG_INF("AWAKE mapped to INT1.");

    return OK;
  }

  // ========== Data Register Reads ==========

  int Adxl362::ReadAxes(int16_t& x, int16_t& y, int16_t& z)
  {
    constexpr int OK { 0 };
    // Burst read 6 bytes: XDATA_L, XDATA_H, YDATA_L, YDATA_H, ZDATA_L, ZDATA_H.
    uint8_t raw[6];
    int result { readRegisters(M_REG_XDATA_L, raw, 6) };
    if (result < 0) {
      LOG_ERR("Failed to read axes: %d!", result);
      return result;
    }

    float scale { getScaleMgPerLsb() };

    // Data registers are 12-bit, little-endian, right-justified.
    auto decode = [scale](uint8_t lo, uint8_t hi) -> int16_t {
      int16_t rawVal { static_cast<int16_t>((hi << 8) | lo) };
      return static_cast<int16_t>(static_cast<float>(rawVal) * scale);
    };

    x = decode(raw[0], raw[1]);
    y = decode(raw[2], raw[3]);
    z = decode(raw[4], raw[5]);

    return OK;
  }
}
