# Fridge Light Monitor — ALC Carefree Product Family

## Overview

Ultra-low-power fridge door monitor using System OFF sleep with BH1749 light sensor interrupt wake. Part of the ALC Carefree product family architecture.

**Key behaviour:**
- Button-initiated commissioning with two-phase light calibration
- System OFF sleep until light detected (door opened)
- Non-blocking BLE init on wake (parallelised with sensor init)
- BLE connection established while door is open
- Sends OPEN packet when door opens, CLOSE packet when door closes (with duration)
- Warning ALERT if door open > timeout (configurable)
- Adaptive wake threshold based on calibrated open/closed light levels
- Factory reset via 10-second button hold
- FEM support at +20 dBm for extended range

## Hardware

- **Board:** Thingy:53 (nRF5340)
- **Light sensor:** BH1749 (I2C address 0x38)
- **INT pin:** P1.05 (active-low, open-drain with pull-up)
- **Button:** P1.14 (sw0, active-low with pull-up)
- **FEM:** nRF21540 (on net core, +20 dBm TX)

## Project Structure

```
src/
├── main.cpp          # Minimal entry point - creates App, calls Init()/Run()
├── app.hpp           # App class definition, states, calibration data
├── app.cpp           # State machine, commissioning, door monitoring
├── ble_nus.hpp       # BLE NUS class, packet structures, event types
├── ble_nus.cpp       # BLE implementation
├── bh1749_light.hpp  # Light sensor class, LightReading struct
└── bh1749_light.cpp  # Direct I2C driver for BH1749
```

## Application States

| State | Description |
|-------|-------------|
| PowerOff | System OFF, waiting for button or light wake |
| Activating | Button pressed, finding gateway + calibrating |
| NoCentral | Gateway not found during activation |
| Standby | Uncalibrated or failed activation, waiting for button |
| Ready | Calibrated, waiting for light wake |
| Monitoring | Door open, monitoring for close |

## Wake Sources

| Source | Trigger | Action |
|--------|---------|--------|
| PowerOn | Fresh boot, debug reset | Check calibration, configure accordingly |
| Button | P1.14 pressed | If uncalibrated: commission. If calibrated: check for factory reset |
| LightInterrupt | BH1749 threshold exceeded | Monitor door, send events |

## Commissioning Flow

```
Button press (uncalibrated device)
    │
    ├── Amber blink (confirm device on)
    ├── LED off
    ├── Init BLE, start advertising
    │
    ├── [Gateway not found in 30s] → Red LED 3s, System OFF
    │
    ├── Gateway connected, NUS enabled
    ├── Sample door-open light (5 readings averaged)
    ├── Green LED 5s (signal: "close door now")
    ├── LED off, wait 500ms
    │
    ├── Poll light every 500ms (5 readings each)
    │   │
    │   ├── [3 consecutive readings < 30% of open level]
    │   │   → Door closed detected
    │   │   ├── Wait 10s for light to settle
    │   │   ├── Sample door-closed light (5 readings)
    │   │   ├── Validate: closed < 50% of open
    │   │   │   ├── [Valid]
    │   │   │   │   ├── Calculate wake threshold
    │   │   │   │   ├── Save calibration to retained memory
    │   │   │   │   ├── Send calibration packet to gateway
    │   │   │   │   └── System OFF (ready for operation)
    │   │   │   │
    │   │   │   └── [Invalid - levels too close]
    │   │   │       ├── Send error packet to gateway
    │   │   │       ├── Red LED 3s
    │   │   │       └── System OFF (uncalibrated)
    │   │
    │   └── [5-minute timeout]
    │       ├── Send timeout error to gateway
    │       ├── Amber blink
    │       └── System OFF (uncalibrated)
```

## Normal Operation Flow

```
Light interrupt wake (calibrated device)
    │
    ├── Init BLE (non-blocking)
    ├── Init light sensor
    ├── Sample door-open light
    │
    ├── [Light < minimum threshold] → Spurious wake, System OFF
    │
    ├── Connect to gateway, send OPEN event
    │
    ├── Monitor for door close (poll every 500ms)
    │   │
    │   ├── [Light drops below threshold]
    │   │   ├── Send CLOSE event (with duration)
    │   │   ├── Update wake threshold from new open level
    │   │   └── System OFF
    │   │
    │   └── [Door open > timeout]
    │       ├── Send ALERT event
    │       └── Continue monitoring
```

## Gateway Packet Format

```cpp
struct __packed FridgeGatewayPacket {
  char     deviceId[32];        // "ffff0000111122223333444455551111"
  char     deviceName[16];      // "FridgeMon"
  uint8_t  eventType;           // 1=open, 2=close, 3=alert, 4=calibration
  uint16_t durationSecs;        // Door duration (or status code for calibration)
  uint16_t red;                 // Light data (or doorOpenLight for calibration)
  uint16_t green;               // Light data (or doorClosedLight for calibration)
  uint16_t blue;                // Light data (or wakeThreshold for calibration)
  uint16_t ir;                  // Light data
  int8_t   rssiDbm;             // Link quality
};  // 60 bytes
```

### Event Types

| Type | Value | Description |
|------|-------|-------------|
| DoorOpen | 1 | Door opened |
| DoorClose | 2 | Door closed (durationSecs = time open) |
| DoorAlert | 3 | Door open too long |
| Calibration | 4 | Calibration result (durationSecs = status code) |

### Calibration Status Codes

| Code | Meaning |
|------|---------|
| 0 | Success |
| 1 | Error: closed level too close to open level |
| 2 | Error: sensor read failure |
| 3 | Error: timeout (door never closed) |

## Configuration Constants

| Constant | Value | Description |
|----------|-------|-------------|
| `M_LIGHT_THRESHOLD_LOW` | 50 | Door close detection (green channel) |
| `M_WAKE_THRESHOLD_MIN` | 15 | Minimum wake threshold |
| `M_WAKE_THRESHOLD_MAX` | 200 | Maximum wake threshold |
| `M_DOOR_OPEN_TIMEOUT_SECS` | 60 | Alert threshold (testing) |
| `M_CALIBRATION_TIMEOUT_MS` | 300000 | 5 minutes for door close during calibration |
| `M_SAMPLES_PER_READING` | 5 | Samples averaged per light reading |
| `M_DOOR_CLOSE_CONSECUTIVE` | 3 | Consecutive low readings to confirm door closed |
| `M_DOOR_CLOSE_THRESHOLD_PERCENT` | 30 | % of open level to detect closed |
| `M_VALID_CALIBRATION_PERCENT` | 50 | Max closed/open ratio for valid calibration |
| `M_WAKE_THRESHOLD_OFFSET_PERCENT` | 30 | Wake = closed + 30% of (open - closed) |
| `M_DOOR_SETTLE_TIME_MS` | 10000 | Wait after door close before sampling |
| `M_BUTTON_FACTORY_RESET_MS` | 10000 | Hold time for factory reset |

## Wake Threshold Calculation

The wake threshold is calculated to reliably detect door opening while avoiding spurious wakes:

```
wakeThreshold = closedLevel + ((openLevel - closedLevel) * 30%)
```

Clamped to range [15, 200].

This places the threshold 30% of the way from closed to open, providing:
- Good margin above closed-door noise floor
- Reliable wake on any meaningful light increase
- Adaptation to different fridge lighting conditions

## Calibration Data (Retained Memory)

```cpp
struct CalibrationData {
  uint16_t wakeThreshold;     // Calculated wake trigger level
  uint16_t doorOpenLight;     // Sampled door-open level
  uint16_t doorClosedLight;   // Sampled door-closed level
  int8_t   txPowerDbm;        // TX power setting
  uint8_t  valid;             // 0xCA = valid calibration
};
```

Stored in retained SRAM (survives System OFF) with CRC32 validation.

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
| Commissioning | Single-phase (drawer open) | Two-phase (open then closed) |
| Calibration data | Light levels, TX power | Open/closed levels, wake threshold |
| Event timing | Send on wake (activity) | Send OPEN on wake, CLOSE on door close |
| Duration tracking | None (instant event) | Track open-to-close time |
| Alert | None | Door open > timeout |
| TX power control | RSSI-based adaptive | Fixed full power (reliability in metal box) |

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

### GPIO Latch for Wake Source Detection

On nRF5340, use GPIO latch registers to determine which pin triggered System OFF wake:
- `nrf_gpio_pin_latch_get(pin)` - check if pin triggered wake
- `nrf_gpio_pin_latch_clear(pin)` - clear after reading
- Works even if pin state has changed since wake (e.g., button released)

### BLE Non-Blocking Init with Callback

The BLE subsystem uses a callback-based init pattern for fast startup:

1. `StartInit()` calls `bt_enable(callback)` and returns immediately
2. Sensor init runs in parallel while net core boots
3. `IsReady()` polls for BT ready status (non-blocking)
4. `CompleteInit()` registers NUS service once BT is ready
5. Advertising starts immediately after

This eliminates the fixed 2-second delay from the critical path.

### Two-Phase Light Calibration

Fridge calibration requires sampling both door-open and door-closed light levels:
1. Sample with door open (user presses button in fridge with door open)
2. Signal user to close door (green LED)
3. Detect door close by light drop (3 consecutive readings < 30% of open)
4. Wait for light to settle (10s)
5. Sample closed level
6. Calculate wake threshold between the two levels

### LED Interference with Light Sensor

LEDs must be OFF during light sampling to avoid affecting BH1749 readings. The commissioning sequence carefully manages LED state:
- LED off during BLE init and light sampling
- Green LED only during "close door" signal phase
- LED off again before closed-level sampling begins
