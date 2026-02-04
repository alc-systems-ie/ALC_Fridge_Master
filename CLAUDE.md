# Fridge Light Monitor — ALC Carefree Product Family

## Overview

Ultra-low-power fridge door monitor using System OFF sleep with BH1749 light sensor interrupt wake. Part of the ALC Carefree product family architecture.

**Key behaviour:**
- System OFF sleep until light detected (door opened)
- Non-blocking BLE init on wake (parallelised with sensor init)
- BLE connection established while door is open
- Sends CLOSE packet instantly when door closes (duration included)
- Warning ALERT if door open > 3 minutes (configurable)
- Adaptive wake threshold (90% of measured door-open light level)
- Graceful BLE failure handling (continues to sleep)
- FEM support at +20 dBm for extended range

## Hardware

- **Board:** Thingy:53 (nRF5340)
- **Light sensor:** BH1749 (I2C address 0x38)
- **INT pin:** P1.05 (active-low, open-drain with pull-up)
- **FEM:** nRF21540 (on net core, +20 dBm TX)

## State Machine

```
BOOT
  |
  +--[Power-on reset]--> CONFIGURE_SENSOR --> SLEEP (sense LOW for INT assert)
  |
  +--[System OFF wake]--> CHECK_LIGHT_LEVEL
                              |
                    +---------+---------+
                    |                   |
              [light high]        [light low]
              Door open           Spurious wake
                    |                   |
                    v                   v
              MONITOR_DOOR         BACK_TO_SLEEP
              (poll sensor)
                    |
           +--------+--------+
           |                 |
      [timeout]         [light low]
           |                 |
           v                 v
      SEND_ALERT       DOOR_CLOSED
      (eventType=3)    (eventType=2)
           |                 |
           +---------+-------+
                     |
                     v
                SEND_PACKET
                (BLE transmit)
                     |
                     v
                SLEEP
```

## Gateway Packet Format

```cpp
struct __packed FridgeGatewayPacket {
  char     deviceId[32];        // "ffff0000111122223333444455551111"
  char     deviceName[16];      // "FridgeMon"
  uint8_t  eventType;           // 1=open, 2=close, 3=alert
  uint16_t durationSecs;        // Door open duration (0 for open event)
  uint16_t red;                 // Light sensor data
  uint16_t green;
  uint16_t blue;
  uint16_t ir;
  int8_t   rssiDbm;             // Link quality
};  // 60 bytes
```

## Configuration Constants

| Constant | Value | Description |
|----------|-------|-------------|
| `M_LIGHT_THRESHOLD_LOW` | 50 | Door close detection (green channel) |
| `M_WAKE_THRESHOLD_PERCENT` | 90 | Wake threshold as % of door-open level |
| `M_WAKE_THRESHOLD_MIN` | 30 | Minimum wake threshold |
| `M_DOOR_OPEN_TIMEOUT_SECS` | 180 | Alert threshold (3 minutes) |
| `M_BLE_CONNECT_TIMEOUT_MS` | 10000 | BLE connection timeout |
| `M_BLE_NUS_ENABLED_TIMEOUT_MS` | 5000 | NUS enable timeout |
| `M_POLL_INTERVAL_MS` | 500 | Door monitoring poll interval |

## BH1749 Driver

Custom direct I2C driver in `bh1749_light.cpp` — bypasses Zephyr sensor API to avoid SYS_INIT issues and give full control over interrupt configuration.

### Key Registers

| Register | Address | Purpose |
|----------|---------|---------|
| SYSTEM_CONTROL | 0x40 | Software reset, part ID |
| MODE_CONTROL1 | 0x41 | Measurement mode, gain |
| MODE_CONTROL2 | 0x42 | RGB enable, valid flag |
| RED_DATA_LSB | 0x50 | Start of colour data (10 bytes) |
| INTERRUPT | 0x60 | INT enable, source, latch; **reading clears latch** |
| PERSISTENCE | 0x61 | How many samples before INT triggers |
| TH_HIGH_LSB | 0x62 | High threshold (2 bytes, little-endian) |
| TH_LOW_LSB | 0x64 | Low threshold (2 bytes, little-endian) |

### Interrupt Behaviour

- INT pin is **active-low, open-drain** — sits HIGH (pulled up) when idle, goes LOW when asserted.
- Thresholds default to 0x0000 on power-up, so any measurement will trigger an interrupt immediately.
- The driver sets safe thresholds (high=0xFFFF, low=0x0000) during Init() to prevent spurious interrupts.
- Clear pending interrupts by **reading** register 0x60.

## FEM Configuration

The Thingy:53 has an nRF21540 FEM on the net core. Configuration is in `sysbuild/hci_ipc.conf`:

```
CONFIG_MPSL_FEM=y
CONFIG_MPSL_FEM_NRF21540_GPIO=y
CONFIG_MPSL_FEM_NRF21540_TX_GAIN_DB=20
```

## Differences from ALC_Drawer_Master

| Aspect | Drawer | Fridge |
|--------|--------|--------|
| Wake source | ADXL362 motion (INT1) | BH1749 light threshold (INT) |
| Event timing | Send on wake (activity) | Send on door close (duration known) |
| Duration tracking | None (instant event) | Track open-to-close time |
| Alert | None | Door open > 3 minutes |

## Lessons Learned

### GPIO ACTIVE_LOW and gpio_pin_get_dt() — Critical Gotcha

When a GPIO is configured with `GPIO_ACTIVE_LOW` in devicetree (common for open-drain interrupt pins):

| Physical Pin | gpio_pin_get_dt() | gpio_pin_get_raw() | Meaning |
|--------------|-------------------|---------------------|---------|
| HIGH (1) | 0 | 1 | Idle (not asserted) |
| LOW (0) | 1 | 0 | Asserted |

**The `_dt` functions return the logical value (inverted for ACTIVE_LOW), not the physical state.**

**When debugging GPIO issues, always use `gpio_pin_get_raw()` to see the actual physical state**, then interpret based on the polarity flags.

### System OFF Wake Configuration

For GPIO wake from System OFF:
- Configure `gpio_pin_interrupt_configure_dt()` with the appropriate level/edge
- The sense configuration survives into System OFF
- On nRF5340, INT level LOW with ACTIVE_LOW pin means: sense physical LOW = INT asserted = light detected

### BLE Non-Blocking Init with Callback

The BLE subsystem uses a callback-based init pattern for fast startup:

1. `StartInit()` calls `bt_enable(callback)` and returns immediately
2. Sensor init runs in parallel while net core boots
3. `IsReady()` polls for BT ready status (non-blocking)
4. `CompleteInit()` registers NUS service once BT is ready
5. Advertising starts immediately after

This eliminates the fixed 2-second delay from the critical path. On subsequent wakes (net core already warm), BLE is ready in ~10ms. Typical wake-to-NUS-ready time is ~1.5 seconds.

### BLE Connection While Door Open

The system establishes BLE connection while the door is still open:
- Advertising starts within ~450ms of wake
- Gateway typically connects within ~600ms
- NUS ready within ~1.5s of wake
- When door closes, packet is sent instantly (already connected)

This ensures reliable transmission while the fridge door provides a clear RF path.

### Adaptive Wake Threshold

Rather than fixed thresholds, the wake threshold is calculated as 90% of the measured door-open light level (minimum 30). This adapts to different fridge lighting conditions and ambient light levels.

### BH1749 Interrupt Window

The BH1749 uses a window comparator - it triggers when the measurement goes outside the high/low threshold range. For wake-on-door-open, we set:
- High threshold: 90% of door-open light (adaptive)
- Low threshold: 0 (effectively disabled)

This ensures only light increases (door opening) trigger wake, not light decreases.
