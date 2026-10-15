# HG-UV98 Firmware Versions

## Official Firmware

### 1. Original Firmware - V1.0.1_190416
- **Date:** April 16, 2019
- **Hardware version:** 1.0.1
- **Radio serial:** AO9A00378 (review unit)
- **Notes:** Base firmware version

### 2. KT2KT Modified Firmware - V1.0.1_190619-KT
- **Date:** June 19, 2019
- **Author:** Kenny KT2KT
- **Major changes:**
  - Changed main menu button from "Sure" to "OK"
  - Improved menu descriptions (dual band display now shows "single" or "dual" instead of confusing terms)
  - Better APRS and function menu labels
  - **UI only** - does NOT replace APRS functionality
- **How to update:** Requires TeraTerm or HyperTerminal with Ymodem protocol at 115200 baud
- **Warning:** Separate from APRS firmware update! Must be done AFTER APRS update.
- **Known issues:** Some users report getting radio stuck in "Host UpData..." mode

### 3. APRS Firmware - September 23rd 2020 (V2)
- **Date:** September 23, 2020
- **Author:** Venus Information Technology (manufacturer)
- **Changes:**
  - Fixed grid square calculation
  - Fixes issue with wrong GPS coordinates reported
- **Binary available:** `HG-UV98 September 23rd 2020 APRS firmware.zip`
- **How to update:** Hold PTT + power on, type "1", upload via Ymodem in TeraTerm

### 4. APRS Firmware - April 30, 2022 (UV98_868_220430_S4)
- **Date:** April 30, 2022
- **Purpose:** Fixes issue with wrong GPS coordinates reported
- **Binary available:** `UV98_868_220430_S4.exe`

## Source Firmware

### UV98_868_220430_D4 (Source Code)
- **Target MCU:** STC8A8K64D4 (8051/MCS-51 compatible)
- **Clock:** 22.1184 MHz
- **IDE:** Keil µVision
- **Build date:** 2022-04-30
- **Location:** `repos/hg-uv98-firmware/UV98_868_220430_D4/`

### Version History from README.C
| Date | Version | Notes |
|------|---------|-------|
| 2018-02-11 | APRS_51G3_4K_20180211A | GPS sync output, Bluetooth added 2 items |
| 2018-02-21 | APRS_51G3_4K_20180221 | Fixed decompression bug |
| 2018-04-08 | APRS_51G3_4K_20180408 | Support curve interface |
| 2018-09-22 | APRS_8K_20180922_D | Bluetooth: send 512 bytes to host after writing settings |
| 2019-02-12 | APRS upgrade package | Various improvements |
| 2022-04-30 | 220430_D4 | Latest source firmware |

### Key Features Added Over Time
- Smart beacon (adaptive transmission interval based on movement)
- MIC-E encoding support
- Dynamic navigation (distance, azimuth)
- Bluetooth control of rotator
- Meteorological data display (temperature, pressure, altitude)
- Fixed station quick setup
- Mileage tracking with auto-reset at 5000km
- Weather icons display
- Multiple GPS system support (GPS, BeiDou, GLONASS)
- Custom TX PATH and DIGI aliases
- AT command interface for host communication