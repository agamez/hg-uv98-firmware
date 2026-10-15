# HG-UV98 Firmware Versions

## ⚠️ Two Separate Firmwares

The HG-UV98 has **two independent firmware images** - one for each processor:

| Firmware | Target MCU | Architecture | Source Available? |
|----------|-----------|--------------|-------------------|
| **Main MCU** | GD32F103RET6 | ARM Cortex-M3 | ❌ Binary only (proprietary) |
| **APRS Coprocessor** | STC8A8K64D4 | 8051 | ✅ Source available (this repo) |

**Important:** Both firmwares must be flashed together (or at least be compatible). Updating one without the other can brick the radio.

See [`docs/ARCHITECTURE.md`](./ARCHITECTURE.md) for details on the dual-MCU design.

---

## Main MCU Firmware (ARM GD32F103)

This firmware controls all radio functions, menus, display, and RF. **Source is NOT publicly available.**

### Known Versions

| Version | Date | Notes |
|---------|------|-------|
| V1.0.1_190416 | 2019-04-16 | Original firmware (radio serial AO9A00378) |
| V1.0.1_190619-KT | 2019-06-19 | **KT2KT modified version** - improved UI, "Sure" → "OK" label |
| Various | - | Updates via CPS (Channel Programming Software) |

### KT2KT Modified Firmware (V1.0.1_190619-KT)

- **Author:** Kenny KT2KT
- **Changes:**
  - Changed main menu button from "Sure" to "OK"
  - Improved menu descriptions
  - Better APRS function menu labels
  - **UI only** - does NOT replace radio firmware functionality
- **How to update:** TeraTerm or HyperTerminal at 115200 baud, Ymodem protocol
- **⚠️ Warning:** Separate from APRS firmware! Must be done AFTER flashing APRS firmware
- **⚠️ Known issue:** Some users report radio stuck in "Host UpData..." mode after failed flash

### Flashing Main MCU

1. Install CH340 USB drivers
2. Hold PF2 (lower side key) + power on → programming mode
3. Use CPS (Channel Programming Software) to write firmware/settings
4. Binary update requires proprietary tools (not publicly documented)

---

## APRS Coprocessor Firmware (STC8A8K64D4 8051)

**Source code available in this repository.**

### Version History

| Date | Version | Key Changes |
|------|---------|-------------|
| 2018-02-11 | APRS_51G3_4K_20180211A | GPS sync output, Bluetooth added 2 items |
| 2018-02-21 | APRS_51G3_4K_20180221 | Fixed decompression bug |
| 2018-04-08 | APRS_51G3_4K_20180408 | Support curve interface |
| 2018-09-22 | APRS_8K_20180922_D | Bluetooth: send 512 bytes to host after settings |
| 2019-02-12 | APRS upgrade package | Various improvements |
| 2020-09-23 | **APRS Firmware Sep 23rd 2020** | ✅ Fixed grid square calculation |
| 2022-04-30 | **UV98_868_220430_S4** | ✅ Latest - fixes wrong GPS coordinates, supports 4-5 decimal places |

### Source Code (UV98_868_220430_D4)

| Spec | Value |
|------|-------|
| **Target** | STC8A8K64D4 (8051/MCS-51 compatible) |
| **Clock** | 22.1184 MHz |
| **IDE** | Keil µVision (C51) |
| **Firmware size** | ~8KB (indicated by "8K CPU" in README.C) |
| **Location** | `src/firmware-source/UV98_868_220430_D4/` |
| **Total files** | 73 C/H files (~7,500+ lines of code) |
| **Total lines** | ~17,000 (including documentation) |

### Key Features in Latest Firmware

- Smart beacon (adaptive transmission based on movement, min 10s interval)
- MIC-E encoding support
- Dynamic navigation (distance, bearing, azimuth)
- Bluetooth control of rotator
- Meteorological data (temperature, pressure, altitude)
- Fixed station quick setup via GPS
- Mileage tracking (odometer, auto-reset at 5000km)
- Weather icons display
- Multi-system GPS (GPS, BeiDou, GLONASS)
- Custom TX PATH and DIGI aliases
- AT command interface for host MCU communication
- 4-5 decimal place GPS precision
- Southern hemisphere distance fix (Apr 2022)

---

## Flashing Procedures

### APRS Coprocessor Firmware

1. Install CH340 USB drivers
2. Connect USB cable to radio (CH340)
3. Hold **PTT button + power on** → "Host UpData..." appears
4. Open TeraTerm at **115200 baud, 8N1**
5. Press **"1"** when prompted
6. Go to **Transfer → Send File**
7. Select **Ymodem** protocol
8. Select firmware `.bin` file
9. Wait for completion (~3 minutes)
10. **Power cycle** the radio

### Both Firmwares (Recommended Order)

1. Flash APRS coprocessor firmware first (Hold PTT + power on)
2. Flash Main MCU firmware via CPS software (Hold PF2 + power on)
3. Both firmwares must be compatible versions

### Tools Required

| Tool | Purpose | Source |
|------|---------|--------|
| **TeraTerm** | APRS firmware update (Ymodem) | https://osdn.net/projects/ttssh2/ |
| **HyperTerminal** | Alternative for firmware update | Hilgraeve trial |
| **CH340 Drivers** | USB serial adapter | Included in firmware packages |
| **CPS v2.0** | Main MCU channel programming | HG-UV98 EN V2.0 setup.rar |
| **APRS Config** | APRS settings programming | APRS_51Serial_20181009.rar |

### ⚠️ Warning

- **Brick risk:** Flashing only one firmware without the other can leave the radio non-functional
- **"Host UpData..." lock:** Multiple users report radio stuck in update mode after failed flash
- **Always backup** before flashing
- **Use stable USB connection** - interruption during flash can brick the coprocessor

---

## Known Firmware Issues

### APRS Coprocessor

| Issue | Status | Notes |
|-------|--------|-------|
| ARISS path bug | Known | Radio requires hop number (ARISS-0), rejects zero-hop |
| Smart beacon lockup | Known | Some units stuck in "Host UpData..." mode |
| Coordinate calculation | ✅ Fixed Sep 2020 | Grid square calculation was incorrect |
| Southern hemisphere | ✅ Fixed Apr 2022 | Distance measurement was wrong |
| Bluetooth module | Varies by batch | Later units may have incomplete soldering |
| Metric conversion | Partial | Celsius/Fahrenheit toggle doesn't always work |

### Main MCU (ARM)

| Issue | Status | Notes |
|-------|--------|-------|
| Menu labels | Fixed in KT2KT | "Sure" → "OK" (V1.0.1_190619-KT) |
| Various UI quirks | Fixed in KT2KT | Improved menu descriptions |

---

## Download Sources (Archived)

Firmware files were distributed via HVDN Notebook (http://notebook.hvdn.org/) and Venus I-Tech reseller:

| File | Description |
|------|-------------|
| `HG-UV98 September 23rd 2020 APRS firmware.zip` | Grid square fix |
| `V1.0.1_190619-KT.zip` | KT2KT UI improvement |
| `UV98_868_220430_S4.exe` | Latest APRS firmware (GPS fix) |
| `HG UV-98 Radio Programming Software (v1)` | CPS for channel programming |
| `HG UV-98 APRS Configuration Software (v1)` | APRS settings tool |
| `CH34x_Install_Windows_v3_4.zip` | USB serial drivers |

> **Note:** Official manufacturer websites are often offline. Community members maintain archived copies.

---

## Related Resources

- **Architecture:** [`docs/ARCHITECTURE.md`](./ARCHITECTURE.md)
- **Source code:** [`docs/SOURCE_CODE_ANALYSIS.md`](./SOURCE_CODE_ANALYSIS.md)
- **Hardware specs:** [`docs/HARDWARE.md`](./HARDWARE.md)
- **Community:** [`docs/COMMUNITY.md`](./COMMUNITY.md)