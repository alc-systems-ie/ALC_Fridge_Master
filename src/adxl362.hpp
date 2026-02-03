#pragma once

/**
 * @file adxl362.hpp
 * @brief ADXL362 accelerometer driver — Zephyr-free class using direct SPI register access.
 *
 * Provides activity/inactivity detection (loop mode), FIFO streaming, and AWAKE→INT1
 * mapping for System OFF wake. Uses SPI via Zephyr's spi_dt_spec only for bus access;
 * all register configuration is handled directly (no Zephyr sensor API at runtime).
 */

#include <zephyr/device.h>
#include <zephyr/drivers/spi.h>

namespace drawer 
{
  class Adxl362
  {
    public:
      // ========== Enumerations ==========

      enum class Range : uint8_t {
        Range2g = 0x00,   ///< ±2g (1000 LSB/g).
        Range4g = 0x01,   ///< ±4g (500 LSB/g).
        Range8g = 0x02    ///< ±8g (235 LSB/g).
      };

      enum class ODR : uint8_t {
        Odr12_5  = 0x00,  ///< 12.5 Hz.
        Odr25    = 0x01,  ///< 25 Hz.
        Odr50    = 0x02,  ///< 50 Hz.
        Odr100   = 0x03,  ///< 100 Hz.
        Odr200   = 0x04,  ///< 200 Hz.
        Odr400   = 0x05   ///< 400 Hz.
      };

      enum class FifoMode : uint8_t {
        Disabled     = 0x00,
        OldestSaved  = 0x01,
        Stream       = 0x02,
        Triggered    = 0x03
      };

      enum class LinkLoopMode : uint8_t {
        Default = 0x00,   ///< Default (independent act/inact).
        Linked  = 0x01,   ///< Linked mode.
        Loop    = 0x03    ///< Loop mode (auto-acknowledged).
      };

      enum class ActivityMode : uint8_t {
        Absolute   = 0x00,
        Referenced = 0x01
      };

      // ========== Structures ==========

      /** @brief Decoded status register contents. */
      struct Status {
        bool dataReady;       ///< New data available.
        bool fifoReady;       ///< FIFO has at least one sample.
        bool fifoWatermark;   ///< FIFO watermark reached.
        bool fifoOverrun;     ///< FIFO overrun.
        bool activity;        ///< Activity detected.
        bool inactivity;      ///< Inactivity detected.
        bool awake;           ///< Device is in awake state.
        bool errUserRegs;     ///< SEU error detected in user registers.
      };

      /** @brief Single XYZ acceleration sample in mg. */
      struct Sample {
        int16_t x;
        int16_t y;
        int16_t z;
      };

      /** @brief Activity/inactivity detection configuration. */
      struct ActivityConfig {
        ActivityMode activityMode;
        ActivityMode inactivityMode;
        LinkLoopMode linkLoop;
        uint16_t activityThresholdMg;     ///< Activity threshold in mg.
        uint8_t  activityTime;            ///< Activity time in samples.
        uint16_t inactivityThresholdMg;   ///< Inactivity threshold in mg.
        uint16_t inactivityTime;          ///< Inactivity time in samples.
      };

      /** @brief Default config for drawer motion detection. */
      static constexpr ActivityConfig DEFAULT_DRAWER_CONFIG {
        .activityMode         = ActivityMode::Referenced,
        .inactivityMode       = ActivityMode::Referenced,
        .linkLoop             = LinkLoopMode::Loop,
        .activityThresholdMg  = 120,
        .activityTime         = 1,
        .inactivityThresholdMg = 150,
        .inactivityTime       = 200    // 2s at 100 Hz.
      };

      // ========== Constructor ==========

      /**
       * @brief Construct an ADXL362 driver instance.
       * @param spiSpec Pointer to a Zephyr spi_dt_spec (from devicetree).
       */
      explicit Adxl362(const struct spi_dt_spec* spiSpec);

      // ========== Initialisation ==========

      /** @brief Reset device, verify ID, and apply default configuration. */
      int Init();

      // ========== Configuration ==========

      /** @brief Configure activity/inactivity detection and link/loop mode. */
      int ConfigureActivity(const ActivityConfig& config);

      /** @brief Configure FIFO mode and watermark. */
      int ConfigureFifo(FifoMode mode, uint16_t watermark);

      /** @brief Set output data rate. */
      int SetOdr(ODR odr);

      /** @brief Set measurement range. */
      int SetRange(Range range);

      /** @brief Enter measurement mode. */
      int StartMeasurement();

      /** @brief Return to standby. */
      int StopMeasurement();

      // ========== FIFO ==========

      /** @brief Read FIFO entry count (number of 16-bit words). */
      int ReadFifoEntries(uint16_t& entries);

      /**
       * @brief Read complete XYZ sample sets from FIFO.
       * @param samples Output buffer for decoded samples.
       * @param maxSets Maximum number of XYZ sets to read.
       * @param setsRead Actual number of sets read.
       */
      int ReadFifo(Sample* samples, uint16_t maxSets, uint16_t& setsRead);

      // ========== Status ==========

      /** @brief Read and decode the status register. */
      int ReadStatus(Status& status);

      /** @brief Check if device is in awake state. */
      bool IsAwake();

      // ========== Interrupt Mapping ==========

      /** @brief Map AWAKE signal to INT1 for System OFF wake. */
      int MapAwakeToInt1();

      // ========== Data Register Reads ==========

      /** @brief Read XYZ from data registers (not FIFO). */
      int ReadAxes(int16_t& x, int16_t& y, int16_t& z);

    private:
      // ========== SPI Helpers ==========

      int writeRegister(uint8_t reg, uint8_t value);
      int readRegister(uint8_t reg, uint8_t& value);
      int readRegisters(uint8_t reg, uint8_t* buffer, size_t length);
      int writeRegister16(uint8_t regLow, uint16_t value);
      int updateRegister(uint8_t reg, uint8_t value, uint8_t mask);
      int readFifoBurst(uint8_t* buffer, size_t length);

      // ========== Conversion Helpers ==========

      uint16_t mgToLsb(uint16_t mg) const;
      float getScaleMgPerLsb() const;
      int verifyDeviceId();

      // ========== Member Variables ==========

      const struct spi_dt_spec* m_spi;
      Range m_currentRange;

      // ========== Constants ==========

      static constexpr uint8_t M_RESET_DELAY_MS    { 5 };
      static constexpr uint8_t M_SOFT_RESET_CODE   { 0x52 };
      static constexpr uint8_t M_DEVID_AD_EXPECTED  { 0xAD };
      static constexpr uint8_t M_DEVID_MST_EXPECTED { 0x1D };
      static constexpr uint8_t M_PARTID_EXPECTED    { 0xF2 };
  };
} 
