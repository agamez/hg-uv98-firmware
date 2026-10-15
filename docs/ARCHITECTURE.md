# HG-UV98 Dual MCU Architecture

## Overview

The HG-UV98 uses a **dual MCU architecture** with two separate processors communicating via UART:

```
┌─────────────────────────────────────────────────┐
│              HG-UV98 System Block Diagram        │
│                                                  │
│  ┌──────────────────────┐    UART     ┌────────────────────────┐
│  │  MAIN MCU            │  Commands   │  APRS Coprocessor       │
│  │  (Radio Functions)   │◄──────────►│  (STC8A8K64D4 8051)     │
│  │  GD32F103RET6        │            │  STC8A8K64D4 8051        │
│  │  ARM Cortex-M3       │            │  22.1184 MHz             │
│  │  108 MHz, 512KB Flash│            │  ~8KB firmware           │
│  │  64KB RAM            │            │  64KB Flash              │
│  │                      │            │  2560 bytes RAM          │
│  │  Manages:            │            │                        │
│  │  • RF transceivers   │            │  Manages:                │
│  │  • LCD display       │            │  • APRS encoding/decoding│
│  │  • Keypad/buttons    │            │  • GPS (NMEA parsing)   │
│  │  • Audio (mic/speaker│            │  • Bluetooth (KISS TNC) │
│  │  • Memory channels   │            │  • Weather sensors      │
│  │  • VFO/channels      │            │  • Smart beacon logic   │
│  │  • PL/DCS tones      │            │  • Mic-E encoding       │
│  │  • General menu UI   │            │  • Digipeater           │
│  └──────────────────────┘            └────────────────────────┘
│              ▲                            ▲              │
│              │                            │              │
│        ┌─────┴──────────────────────────┴─────┐         │
│        │         Common Resources             │         │
│        │  • AT24C512 EEPROM (64KB)            │         │
│        │    - Configuration data shared       │         │
│        │    - 512-byte buffer for both CPUs   │         │
│        │  • GPS module (via UART3/APRS CPU)   │         │
│        │  • Bluetooth module (via UART2)      │         │
│        │  • BMP280/BMP085 sensors (I2C)       │         │
│        │  • CMX865A RF codec (SPI/I2C)        │         │
│        │  • FX604 1200 baud APRS codec        │         │
│        └──────────────────────────────────────┘         │
└─────────────────────────────────────────────────────────┘
```

## Main MCU (Radio Controller)

| Specification | Value |
|---------------|-------|
| **Part** | GD32F103RET6 |
| **Family** | GigaDevice GD32 (STM32-compatible) |
| **Architecture** | ARM Cortex-M3 (32-bit) |
| **Clock** | Up to 108 MHz |
| **Flash** | 512 KB (RET6 package) |
| **RAM** | 64 KB |
| **Peripherals** | Multiple UART, SPI, I2C, ADC, timers, USB |
| **Role** | Controls all radio functions, display, menus, channels, RF |

**Firmware:** Not publicly available. This is the "main" firmware that handles all radio operations.

## APRS Coprocessor

| Specification | Value |
|---------------|-------|
| **Part** | STC8A8K64D4 |
| **Family** | STC 8051 (MCS-51 compatible) |
| **Architecture** | 8051 (8-bit) |
| **Clock** | 22.1184 MHz (optimized for UART baud rates) |
| **Flash** | 64 KB (but firmware is ~8KB) |
| **RAM** | 2560 bytes XRAM |
| **Role** | Dedicated APRS/GPS/Bluetooth processing |

**Firmware:** Available in this repository (`src/firmware-source/UV98_868_220430_D4/`)

## Communication Between CPUs

### UART Protocol

The two processors communicate via UART using a custom AT-command protocol:

| Command | Direction | Description |
|---------|-----------|-------------|
| `$A09\r\n` | → Main MCU | Side key 2 (PF2) pressed |
| `$A10\r\n` | → Main MCU | Alarm key pressed |
| `$A11\r\n` | → Main MCU | PTT key released |
| `$A50\r\n` | → Main MCU | Read version number |
| `$A51\r\n` | → Main MCU | Read serial number |
| `$E01\r\n` | ← APRS CPU | PTT pulled HIGH, send beacon |
| `$E02\r\n` | ← APRS CPU | PTT pulled HIGH, channel B transmit |
| `$E00\r\n` | ← APRS CPU | PTT pulled LOW (stop transmit) |
| `$F01\r\n` | ← APRS CPU | Transmit path 1 name |
| `AT11\r\n` | ← Host | Read version number |
| `AT12\r\n` | ← Host | Read CPU ID |

**Full protocol documentation:** Refer to the file `APRS CPU与对讲机CPU通讯协议.xlsx` (Protocolo de comunicación entre CPU APRS y CPU de radio) - this was in the original firmware package but not included in source form.

### Memory Sharing

Both CPUs access the **AT24C512 EEPROM (64KB)** through a **512-byte shared buffer** (`EEPROM_Buffer[512]`):

| Address Range | Content |
|---------------|---------|
| `0x00-0x0F` | Beacon settings (SSID, PATH, interval, callsign) |
| `0x10-0x14` | MIC-E encoding, callsign, APRS config |
| `0x20-0x2F` | Fixed station latitude/longitude |
| `0x30-0x3F` | Fixed station longitude, other settings |
| `0x40-0x7F` | Beacon queue, call sign info |
| `0x80-0xBF` | GPS configuration, beacon interval |
| `0xFE00-0xFE03` | Mileage odometer (4 bytes) |
| `0x0129-0x012A` | Altitude sensor calibration |
| `0x01D5` | Digipeater settings |
| `0x00BB` | Mileage memory enable flag |
| `0x02A` | Mobile station mode (0=fixed, 1=mobile) |

The EEPROM serves as a **non-volatile configuration storage** shared by both processors.

## Firmware Update Paths

### Main MCU Firmware Update
- Via USB programming (CH340)
- Hold PF2 (lower side key) + power on → programming mode
- Uses CPS (Channel Programming Software) to write channels/settings
- Full firmware update requires special tools (not publicly documented)

### APRS Coprocessor Firmware Update
- Same USB port, but different procedure
- Hold PTT + power on → "Host UpData..."
- Use TeraTerm at 115200 baud, Ymodem protocol
- Separate `.bin` file for the 8051 firmware
- **Two separate firmware files are required** (main + APRS)

## Why This Architecture?

1. **Cost optimization:** The 8051 coprocessor handles APRS-specific tasks, leaving the ARM MCU free for radio functions
2. **Modularity:** APRS features can be enabled/disabled independently
3. **Legacy compatibility:** Many Chinese radios use similar dual-CPU designs (8051 for TNC/data, ARM for radio)
4. **Development time:** Venus Tech/BH4TDV likely reused existing APRS firmware across multiple products

## Known Issues with Dual-CPU Design

1. **Synchronization:** If one CPU hangs, the other may not function correctly
2. **Brick risk:** Flashing one firmware without the other can leave radio unusable
3. **"Host UpData..." lock:** If UART communication fails or flash fails, radio appears bricked
4. **Later production batches:** Some units have incomplete Bluetooth module soldering, affecting KISS TNC functionality
5. **Coordinate bugs:** Fixed in separate firmware releases (Sep 2020, Apr 2022)

## References

- **Main MCU teardown:** https://www.reddit.com/r/amateurradio/comments/c3wurz/teardown_of_hguv98_the_first_chinese_aprs_handheld/
- **APRS Firmware (8051):** https://groups.io/g/HG-UV98-users/topic/hg_uv98_firmware_images/79373250
- **GD32F103:** https://www.gigadevice.com/product/mcu/main-stream-mcus/gd32f10x-series