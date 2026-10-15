# HG-UV98 Source Code Analysis

## Overview

The source code for the HG-UV98 is a complete embedded application for an APRS (Automatic Packet Reporting System) handheld radio, written in C for the STC8A8K64D4 microcontroller (8051 architecture). The project uses the Keil µVision IDE.

## Hardware Platform

| Component | Specification |
|-----------|--------------|
| **MCU** | STC8A8K64D4 (8051-compatible, 64KB Flash) |
| **Clock** | 22.1184 MHz |
| **RF Transceiver** | CMX865A (Cirrus Logic) |
| **APRS Codec** | FX604 |
| **EEPROM** | AT24C512 (64KB I2C) |
| **GPS** | Multi-system (GPS, BeiDou, GLONASS) |
| **Sensors** | BMP280 (pressure/temperature) or BMP085, DS18B20 (1-Wire temp) |
| **Bluetooth** | Integrated (KISS TNC mode) |
| **Display** | Color LCD |
| **Flash** | UV98_868_220430_D4.uvproj (Keil project) |

## Source File Structure

### MAIN Directory (`UV98_868_220430_D4/MAIN/`)

| File | Lines | Description |
|------|-------|-------------|
| `GPRS_51GATE.c` | 279 | Main application entry, interrupt handlers, system init |
| `AT24C512.c` | 269 | I2C EEPROM driver for AT24C512 |
| `DISP_GPS.c` | ~300 | GPS data display (estimated) |
| `DIGI.c` | 19330 | Digital mode handling (estimated) |
| `CMX865A_DECODE.c` | 15861 | CMX865A RF transceiver decode |
| `CMX865A_SPI.c` | 12952 | CMX865A SPI interface |
| `BEACON.c` | 16118 | APRS beacon transmission |
| `CMX865A_CODE.c` | 9493 | CMX865A code/modulation |
| `BT.c` | 9112 | Bluetooth module handling |
| `KISS_DECODE.c` | ~1500 | KISS protocol decoding |
| `APRS_RF.c` | 9295 | APRS RF codec (FDMDO 1200 baud) |
| `FUN_B.c` | ~2000 | Function B operations |
| `FUN_C.c` | ~2000 | Function C operations |
| `MICE_DECODE.c` | ~800 | MIC-E encoding/decoding |
| `MICE_CODE.c` | ~500 | MIC-E code tables |
| `KISS2GAOMIN.c` | ~500 | KISS to GAOMIN protocol |
| `KISS2ASC.c` | ~500 | KISS to ASCII conversion |
| `KISS_Analysis.c` | ~1000 | KISS packet analysis |
| `GPS2.c` | 1155 | GPS parsing (GPRMC, GPGGA) |
| `CHx.c` | 2086 | Channel handling |
| `PTT_FUN.c` | ~500 | PTT function |
| `CRC16.c` | 3978 | CRC16 checksum calculation |
| `ADC.c` | 1771 | ADC driver |
| `UART1.c` | ~500 | UART1 driver |
| `UART2.c` | ~500 | UART2 driver (Bluetooth) |
| `UART3.c` | ~500 | UART3 driver (GPS) |
| `UART4.c` | ~300 | UART4 driver (debug/EEPROM) |
| `BT.c` | 9112 | Bluetooth handling |
| `tostring.c` | 167 | Integer to string conversion |
| `LCD.c` | ~2000 | LCD display driver (estimated) |

### SENSOR Directory (`UV98_868_220430_D4/SENSOR/`)

| File | Description |
|------|-------------|
| `bmp280.c/h` | BMP280 pressure/temperature sensor |
| `bmp085.c/h` | BMP085 pressure/temperature sensor (older) |
| `DS18B20.c/h` | DS18B20 1-Wire temperature sensor |
| `DS18B20.h` | DS18B20 header |

## Key Data Structures

### EEPROM Memory Map
- **0x0FE00-0x0FE03:** Mileage storage (4 bytes, unsigned long)
- **0x20-0x29:** Fixed station latitude
- **0x30-0x39:** Fixed station longitude
- **0x04:** Smart beacon mode setting (0=FAST, 1=MID1, 2=MID2, 3=SLOW)
- **0x2A:** Mobile station mode
- **0x00BB:** Mileage memory enable flag
- Total buffer: 512 bytes (0x00-0xFF)

### GPS Data Structures
- `GPS_TIME[10]` - Current GPS time (HH:MM:SS format)
- `GPS_DATE[13]` - Current GPS date (YYYY-MM-DD)
- `GPS_WD[15]` - Latitude
- `GPS_JD[15]` - Longitude
- `GPS_NOW_DIR` - Current heading (0-359°)
- `GPS_NOW_SPEED_KNOT` - Speed in knots (0-255)
- `GPS_SPEED_KM` - Speed in km/h
- `GPS_ALT_MI` - Altitude in feet
- `GPS_HEIGHT[7]` - Altitude string
- `GPS_SIG[3]` - Satellite count
- `GPS_LOCKED` - GPS lock status

## Protocol Implementations

### APRS RF Codec (1200 baud FDMDO)
- Uses FX604 codec chip via CMX865A
- 833us bit timing
- 417us half-bit timing
- HDLC framing with 7E flag
- Bit stuffing (5-bit rule)
- CRC16 checksum verification

### KISS Protocol
- Raw AX.25 packet interface
- Supported over UART2 (Bluetooth) and UART3 (UART)
- Commands: EN, DIS, TNC2 status
- Beacon queue management

### AT Commands
- `AT11\r\n` - Read version number
- `AT12\r\n` - Read CPU ID
- `AT+TONE=1200` - Low tone test
- `AT+TONE=2200` - High tone test
- `AT+TONE=OFF` - Disable tones
- `AT+MICE=ON/OFF` - MIC-E encoding

### BT Output Modes
- KISS TNC mode
- UI packet output
- GPWPL format
- KISS ASC output
- Data analysis output (beacon only)

## Firmware Update Procedure

### Via USB (CH340)
1. Install CH340 USB drivers
2. Connect USB cable to radio
3. Hold PTT button + power on → "Host UpData..."
4. Open TeraTerm at 115200 baud
5. Press "1" when prompted
6. Transfer → Send File → Ymodem protocol
7. Select firmware file (.bin)
8. Wait for completion (~3 min)
9. Power cycle radio

### Required Software
- **TeraTerm:** https://osdn.net/projects/ttssh2/releases/
- **HyperTerminal Trial:** https://www.hilgraeve.com/hyperterminal-trial/
- **CH340 Drivers:** Standard Windows CH34x drivers

## Known Issues & Bugs

1. **ARISS path bug:** Path must include "0" (e.g., ARISS-0), radio rejects zero-hop paths
2. **Smart beacon on ISS:** Cannot use ARISS with 0 hops
3. **Host UpData lock:** Some users report radio stuck in update mode after failed flash
4. **Coordinate calculation:** Fixed in Sep 2020 firmware
5. **Bluetooth module:** Later batches may have incomplete soldering affecting KISS TNC functionality
6. **Metric conversion:** Celsius to Fahrenheit toggle doesn't always work
7. **Battery connector:** Some units ship with USB-C instead of Micro-USB (unknown variant)