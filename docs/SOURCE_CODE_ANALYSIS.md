# HG-UV98 Source Code Analysis

## ⚠️ IMPORTANT: This is the APRS Coprocessor Firmware

**This source code is NOT the main radio firmware.** It is the firmware for the **dedicated APRS coprocessor** (STC8A8K64D4, 8051 architecture).

The HG-UV98 uses a **dual MCU architecture**:
- **Main MCU:** GD32F103RET6 (ARM Cortex-M3) - handles radio functions, display, menus, channels, RF
- **APRS Coprocessor:** STC8A8K64D4 (8051) - handles APRS, GPS, Bluetooth, sensors (THIS firmware)

See [`docs/ARCHITECTURE.md`](./ARCHITECTURE.md) for the full system diagram and CPU communication protocol.

## Overview

The source code is a complete embedded application for the **APRS coprocessor** of the HG-UV98, written in C for the STC8A8K64D4 microcontroller (8051 architecture). The project uses the Keil µVision IDE.

This firmware handles:
- APRS packet encoding/decoding (1200 baud FDMDO)
- GPS NMEA parsing (GPRMC, GPGGA)
- Bluetooth KISS TNC interface
- Smart beacon transmission
- Mic-E encoding
- Digipeater functionality
- Weather sensor data collection
- Navigation (distance/bearing)

## Hardware Platform (APRS Coprocessor)

| Component | Specification |
|-----------|--------------|
| **MCU** | STC8A8K64D4 (8051-compatible, 64KB Flash) |
| **Clock** | 22.1184 MHz (chosen for precise UART baud rates) |
| **IDE** | Keil µVision (C51) |
| **Firmware size** | ~8KB (indicated by "8K CPU" in README.C) |
| **EEPROM** | AT24C512 (64KB I2C) - shared with Main MCU |

### System Components (managed by this MCU)

| Component | Interface | Notes |
|-----------|-----------|-------|
| **GPS** | UART3 | Multi-system (GPS, BeiDou, GLONASS) |
| **Bluetooth** | UART2 | KISS TNC mode, pair code: 1234 |
| **BMP280** | I2C | Pressure + temperature (APRS weather) |
| **BMP085** | I2C | Older version (legacy support) |
| **DS18B20** | 1-Wire (P3.6) | Optional temperature sensor |
| **CMX865A** | SPI | RF codec for APRS modulation/demodulation |
| **FX604** | GPIO | 1200 baud FDMDO APRS codec |

## Source File Structure

### MAIN Directory (`UV98_868_220430_D4/MAIN/`)

| File | Lines | Description |
|------|-------|-------------|
| `GPRS_51GATE.c` | 279 | **Entry point** - Initial_uv98(), interrupt handlers, system init |
| `README.C` | 144 | Release notes and version history (2018-2022) |
| `PUBLIC_BUF.C` | 19 | Shared buffer (512 bytes) with Main MCU |
| `DELAY.C` | 77 | Delay functions for timing |
| `DISP_GPS.C` | 858 | GPS data display on Main MCU screen |
| `DIGI.C` | 640 | Digipeater forwarding |
| `BEACON.C` | 605 | APRS beacon transmission (smart, timed, PTT) |
| `CMX865A_DECODE.C` | 577 | RF demodulation (1200 baud FDMDO) |
| `MICE_DECODE.C` | 569 | MIC-E position decoding |
| `UARTX.C` | 560 | UART helper functions, AT command parsing |
| `GPS_JULI.C` | 559 | Distance/bearing calculation (Julian date) |
| `KISS_Analysis.C` | 544 | KISS packet analysis |
| `STC_EEPROM.C` | 509 | EEPROM emulation / 512-byte buffer management |
| `KISS2GAOMIN.C` | 506 | KISS to custom protocol conversion |
| `CMX865A_SPI.C` | 487 | SPI communication with CMX865A |
| `FUN_B.C` | 393 | Function B operations (menu/controls) |
| `BT.C` | 359 | Bluetooth module handling |
| `APRS_RF.C` | 352 | APRS RF codec (FDMDO 1200 baud) |
| `MICE_CODE.C` | 341 | MIC-E position encoding |
| `KISS2ASC.C` | 316 | KISS to ASCII conversion |
| `BEACON_SAVE.C` | 304 | Beacon persistence to EEPROM |
| `CMX865A_CODE.C` | 295 | Modulation codes for RF |
| `GPRS_51GATE.c` | 279 | System entry and interrupt management |
| `AT24C512.c` | 269 | I2C EEPROM driver (64KB shared memory) |
| `UART3.C` | 252 | UART3 driver (GPS module) |
| `tostring.c` | 167 | Integer to string conversion |
| `UART1.C` | 159 | UART1 driver (debug/programming) |
| `UART2.C` | 155 | UART2 driver (Bluetooth) |
| `GPS2.C` | 1,155 | GPS parsing, smart beacon, mileage tracking |
| `UART4.C` | 140 | UART4 driver (communication with Main MCU) |
| `README.C` | 144 | Version history and change log |
| `CHx.C` | 87 | Channel operations |
| `CRC16.C` | 96 | CRC16 checksum calculation |
| `ADC.C` | 83 | ADC driver for voltage/temp |
| `GPS_CHECKSUM.C` | 82 | NMEA checksum verification |
| `FUN_C.C` | 1000 | Function C operations |
| `LED.C` | 55 | LED indicator control |
| `HEX_TO_ASC.C` | 50 | Hex to ASCII conversion |
| `PTT_FUN.C` | 36 | PTT button functions |

### SENSOR Directory (`UV98_868_220430_D4/SENSOR/`)

| File | Description |
|------|-------------|
| `bmp280.c/h` | BMP280 I2C pressure/temperature sensor |
| `BMP085.C/H` | BMP085 (older version, legacy support) |
| `DS18B20.c/h` | DS18B20 1-Wire temperature sensor |

### MCU Headers

| File | Description |
|------|-------------|
| `STC8A8K64D4.H` | STC8A8K64D4 register definitions |
| `STC15Fxxxx.H` | STC15 series (compatibility) |
| `IO.H` | I/O pin definitions |

## Key Data Structures

### EEPROM Memory Map (512-byte shared buffer)

Both the Main MCU and this APRS coprocessor access the AT24C512 EEPROM through a shared 512-byte buffer:

| Address | Content |
|---------|---------|
| `0x00-0x01` | Beacon interval (timed beacon) |
| `0x02` | Timing beacon enable |
| `0x04` | Smart beacon mode (0=FAST, 1-5=increasing intervals) |
| `0x08-0x0F` | Callsign, SSID, APRS config |
| `0x10` | MIC-E encoding enable |
| `0x12-0x14` | Callsign fields |
| `0x20-0x2F` | Fixed station latitude (for fixed station mode) |
| `0x2A` | Mobile station mode (0=fixed, 1=mobile/GPS) |
| `0x30-0x3F` | Fixed station longitude, other settings |
| `0x3B` | GPS beacon enable |
| `0x3C` | Battery voltage telemetry enable |
| `0x3D` | Temperature telemetry enable |
| `0x3E-0x3F` | Mileage and altitude telemetry |
| `0x40-0x7F` | Beacon queue, call sign info |
| `0x80-0xBF` | GPS configuration, beacon settings |
| `0xFE00-0xFE03` | Odometer/mileage (4 bytes unsigned long) |
| `0x0129-0x012A` | Altitude calibration |
| `0x01D5` | Digipeater settings |

### GPS Data Structures

```c
// In GPS2.C / GPS2.H
uchar GPS_TIME[10];      // Current GPS time (HH:MM:SS format)
uchar GPS_DATE[13];      // Current GPS date (YYYY-MM-DD)
uchar GPS_WD[15];        // Latitude (supports 4-5 decimal places)
uchar GPS_JD[15];        // Longitude (supports 4-5 decimal places)
uint  GPS_NOW_DIR;       // Current heading (0-359°)
uint  GPS_NOW_SPEED_KNOT;// Speed in knots (0-255)
uint  GPS_SPEED_KM;      // Speed in km/h
long  GPS_ALT_MI;        // Altitude in feet
uchar GPS_HEIGHT[7];     // Altitude as string (feet)
uchar GPS_SIG[3];        // Satellite count (00-12)
uchar GPS_WAIT_TIME;     // Beacon interval counter (0-255)*10 seconds
uchar GPS_LOCKED;        // GPS position lock status (bit)
uchar GPS_ALT_ERR;       // Altitude data validity flag
bit   GPS_DIR_NOTHING;   // Stationary/moving flag (no heading when stationary)

// Mileage
unsigned long TOTAL_LC, AB_LC;  // Total and segment mileage
uchar lc_start;                   // First launch flag (0=first, 1=continuing)
uchar QUICK_SET_EN;               // Quick fixed station setup flag
```

### APRES Telemetry Format

```
;.{temp}V{voltage}H{altitude}M{pressure}S{speed}@{lat}/{lon}d{dir}m{km}
```

Example: `;.;u >>>/3r} 8.3V 25.6C 1016.3pa KG928*70`

- Temperature in Celsius
- Battery voltage in volts
- Altitude in feet
- Pressure in hPa/mBar
- Maidenhead grid square
- Speed in knots
- Direction in degrees
- Mileage (odometer, increments at 5000km threshold)

## Protocol Implementations

### APRS RF Codec (1200 baud FDMDO)

Uses the CMX865A and FX604 codec chips for 1200 baud APRS data transmission:
- **Timing:** 833us full-bit, 417us half-bit
- **Framing:** HDLC with 7E flag delimiters
- **Bit stuffing:** 5-bit rule (standard HDLC)
- **Checksum:** CRC16 verification
- **Max packet:** 128 bytes (extended to 190 bytes in later firmware)

### KISS Protocol

- Raw AX.25 packet interface
- Supported over UART2 (Bluetooth) and UART3 (UART)
- Commands: EN, DIS, TNC2 status
- Beacon queue management
- UI packet output
- GPWPL waypoint output
- Mixed GPS+UI output option

### Commands to Main MCU (via UART4)

| Command | Direction | Description |
|---------|-----------|-------------|
| `$E01\r\n` | APRS → Main | PTT HIGH, send beacon |
| `$E02\r\n` | APRS → Main | PTT HIGH, channel B TX |
| `$E00\r\n` | APRS → Main | PTT LOW, stop TX |
| `$F01\r\n` | APRS → Main | TX PATH 1 name |
| `$A09\r\n` | Main → APRS | PF2 key pressed |
| `$A10\r\n` | Main → APRS | Alarm key pressed |
| `$A11\r\n` | Main → APRS | PTT released |
| `$A50\r\n` | Main → APRS | Read version |
| `$A51\r\n` | Main → APRS | Read serial number |

### AT Commands for Host Communication

```
AT11\r\n   → Version number
AT12\r\n   → CPU ID
AT+TONE=1200  → Low tone test
AT+TONE=2200  → High tone test
AT+TONE=OFF   → Disable tones
AT+MICE=ON/OFF → MIC-E encoding
```

### Bluetooth Output Modes

| Mode | Description |
|------|-------------|
| KISS TNC | Standard AX.25 KISS protocol |
| UI packets | UI-frame APRS packets |
| GPWPL | Waypoint format |
| KISS ASC | KISS ASCII mode |
| Mixed GPS+UI | GPRMC/GPGGA + decoded data |
| Data analysis | Beacon data analysis only |

## Firmware Versions (APRS Coprocessor)

| Date | Version | Key Changes |
|------|---------|-------------|
| 2018-02-11 | APRS_51G3_4K_20180211A | GPS sync output, Bluetooth added 2 items |
| 2018-02-21 | APRS_51G3_4K_20180221 | Fixed decompression bug |
| 2018-04-08 | APRS_51G3_4K_20180408 | Support curve interface |
| 2018-09-22 | APRS_8K_20180922_D | Bluetooth: send 512 bytes to host |
| 2019-02-12 | APRS upgrade package | Various improvements |
| 2022-04-30 | 220430_D4 | Latest (4-5 decimal GPS, smart beacon) |

## Known Issues in APRS Firmware

1. **ARISS path bug:** Requires hop number (e.g., ARISS-0), rejects zero-hop paths
2. **Smart beacon lockup:** Some units stuck in "Host UpData..." mode after failed flash
3. **Bluetooth module:** Later batches may have incomplete soldering (affects KISS TNC)
4. **Coordinate calculation:** Fixed in Sep 2020 firmware
5. **Distance calculation:** Southern hemisphere users affected (fixed in Apr 2022)
6. **Metric conversion:** Celsius/Fahrenheit toggle sometimes doesn't work

## Compiling

### Requirements

1. **Keil µVision IDE** (C51 variant)
2. **STC8051 device pack** (or GD32F103-compatible header files)
3. Project file: `UV98_868_220430_D4.uvproj`

### Compilation Notes

- `uchar` = unsigned char, `uint` = unsigned int, `ulong` = unsigned long
- `bit` = C51 bit type
- `sbit` = special bit register
- Delay macros use hardware PCA timer
- Optimized for 22.1184 MHz (exact UART baud rate divisors)

### Build Output

- HEX file: `OBJ/UV98_868_220430_D4.hex`
- Size: ~8KB (fits in "8K CPU" firmware space)

## Relationship to Main MCU Firmware

| Aspect | APRS Coprocessor (THIS) | Main MCU (GD32F103) |
|--------|------------------------|---------------------|
| Architecture | 8051 (8-bit) | ARM Cortex-M3 (32-bit) |
| Clock | 22.1184 MHz | 108 MHz |
| Flash | 64KB (~8KB used) | 512KB |
| RAM | 2560 bytes XRAM | 64KB |
| Firmware | Source available (this repo) | Binary only (proprietary) |
| Role | APRS, GPS, BT, sensors | Radio, display, menus, RF |
| Updates | Via USB/Ymodem | Via CPS software |

**Key point:** The APRS coprocessor firmware runs independently but communicates with the Main MCU via UART. Both firmware versions must be compatible for the radio to function correctly.