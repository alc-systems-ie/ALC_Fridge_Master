# Fridge Light Monitor — Project Notes

## Overview

Monitors fridge door open/close events using the BH1749 light sensor on Thingy:53. Detects door opening via light level increase, tracks duration, and sends BLE notifications to a gateway.

## Hardware

- **Board:** Thingy:53 (nRF5340)
- **Light sensor:** BH1749 (I2C address 0x38)
- **INT pin:** P1.05 (active-low, open-drain with pull-up)

## BH1749 Driver

Custom direct I2C driver in `bh1749_light.cpp` — bypasses Zephyr sensor API to avoid SYS_INIT issues and give full control over interrupt configuration.

### Key registers

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

### Interrupt behaviour

- INT pin is **active-low, open-drain** — sits HIGH (pulled up) when idle, goes LOW when asserted.
- Thresholds default to 0x0000 on power-up, so any measurement will trigger an interrupt immediately.
- The driver sets safe thresholds (high=0xFFFF, low=0x0000) during Init() to prevent spurious interrupts.
- Clear pending interrupts by **reading** register 0x60.

## Door Detection

Uses green channel thresholds:
- **Door open:** green > 200
- **Door closed:** green < 50

State machine in main loop tracks open/close transitions and duration.

## Lessons Learned

### GPIO ACTIVE_LOW and gpio_pin_get_dt() — Critical Gotcha

When a GPIO is configured with `GPIO_ACTIVE_LOW` in devicetree (common for open-drain interrupt pins):

| Physical Pin | gpio_pin_get_dt() | gpio_pin_get_raw() | Meaning |
|--------------|-------------------|---------------------|---------|
| HIGH (1) | 0 | 1 | Idle (not asserted) |
| LOW (0) | 1 | 0 | Asserted |

**The `_dt` functions return the logical value (inverted for ACTIVE_LOW), not the physical state.**

This caused significant debugging confusion when the INT pin appeared "stuck at 0" — it was actually physically HIGH (idle), but `gpio_pin_get_dt()` returned 0 due to the active-low inversion.

**When debugging GPIO issues, always use `gpio_pin_get_raw()` to see the actual physical state**, then interpret based on the polarity flags.
