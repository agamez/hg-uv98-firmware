# HG-UV98 - Comprehensive Reference Repository

A comprehensive repository of documentation, source code, and community resources for the **Lanch HG-UV98 APRS** handheld radio.

## ⚠️ Dual-Processor Architecture

The HG-UV98 uses **two independent microcontrollers** communicating over UART:

| Processor | MCU | Architecture | Role | Firmware |
|-----------|-----|--------------|------|----------|
| **Main MCU** | GD32F103RET6 | ARM Cortex-M3 (32-bit, 108 MHz) | FM radio, display, menus, RF, channels | ❌ No source → 🛠️ RE in [`reconstruction/`](reconstruction/README.md) |
| **APRS Coprocessor** | STC8A8K64D4 | 8051 (8-bit, 22.1184 MHz) | APRS, GPS, Bluetooth, sensors | ✅ **Source available** |

The **original** source code in this repository is for the **APRS coprocessor (8051)**; only a binary exists for the main processor (ARM), whose documented reconstruction is underway in [`reconstruction/`](reconstruction/README.md).

## Repository Structure

```
HG-UV98/
├── README.md                          # This file
├── LICENSE                            # Provisional BSD-3-Clause
├── UV98_868_220430_D4/                # Original coprocessor firmware dump, as provided
│   ├── MAIN/   (37 .C/.H files)       #   coprocessor main firmware
│   ├── SENSOR/ (6 .C/.H files)        #   sensor drivers (BMP280, BMP085, DS18B20)
│   └── OBJ/, *.uvproj, *.doc/xlsx     #   build output and vendor documentation
├── src/firmware-source/               # Working copy (C/H sources only) referenced by the docs
├── firmware/                          # Official binaries for both CPUs + flashing instructions
├── docs/                              # Documentation
│   ├── ARCHITECTURE.md                # Dual-CPU architecture + diagram
│   ├── FIRMWARE_VERSIONS.md           # Firmware version history (both CPUs)
│   ├── FIRMWARE_INVENTORY.md          # Inventory of binaries and artifacts
│   ├── SOURCE_CODE_ANALYSIS.md        # Coprocessor source code analysis
│   ├── SOURCE_INDEX.md                # Index of all 73 source files
│   ├── HARDWARE.md                    # Hardware and sensor reference
│   ├── COMMUNITY.md                   # Community resources
│   └── RESEARCH_LOG.md                # Main-MCU reverse-engineering log (sessions 1-19)
├── reconstruction/                    # Documented C reconstruction of the Main-MCU firmware
│   ├── src/                           #   recovered modules (protocol, display, font, rf, uart, ...)
│   ├── ucos2/, vendor/                #   vendored uC/OS-II kernel + GD32F10x SPL
│   ├── tests/                         #   host protocol-conformance tests
│   └── README.md                      #   status, provenance, and licenses
├── tools/                             # Analysis tooling (recon_map.py)
└── repos/                             # Community reference submodules
    ├── marrold-HG-UV98/
    └── pwnj0hn-HG-UV98/
```

## APRS Coprocessor Source Code

| Specification | Value |
|---------------|-------|
| **MCU** | STC8A8K64D4 (8051/MCS-51 compatible) |
| **Clock** | 22.1184 MHz (precise for UART baud rates) |
| **IDE** | Keil µVision (C51) |
| **Firmware size** | ~8KB ("8K CPU") |
| **C/H files** | 73 files |
| **Lines of code** | ~7,500+ lines of C |
| **Total repository** | ~17,000 lines (including docs) |

### Firmware modules
- APRS RF codec (1200 baud FSK via CMX865A/FX604)
- GPS NMEA parsing (GPRMC, GPGGA - GPS, BeiDou, GLONASS)
- Bluetooth KISS TNC mode
- Smart beacon (movement-based adaptive interval)
- Mic-E encoding
- Digipeater forwarding
- Weather sensors (BMP280/BMP085, DS18B20)
- Navigation (distance, bearing, azimuth)
- Odometer/mileage tracking
- AT command interface (communicates with the Main MCU)

## Firmware (Both CPUs)

### APRS Coprocessor Firmware Versions

| Version | Date | Notes |
|---------|------|-------|
| 2018-02-11 | APRS_51G3_4K_20180211A | GPS sync, Bluetooth |
| 2018-04-08 | APRS_51G3_4K_20180408 | Curve interface |
| 2018-09-22 | APRS_8K_20180922_D | Bluetooth 512 bytes to host |
| 2020-09-23 | APRS Sep 23rd 2020 | ✅ Grid square fix |
| 2022-04-30 | UV98_868_220430_S4 | ✅ GPS coordinates fix |

### Main MCU (ARM) Firmware Versions

| Version | Date | Notes |
|---------|------|-------|
| V1.0.1_190416 | 2019-04 | Original firmware |
| V1.0.1_190619-KT | 2019-06 | KT2KT modified (UI improvements) |

**⚠️ Important:** Both firmwares must be compatible. Flashing only one of them may leave the radio unusable.

## 🛠️ Main MCU (ARM) Reverse Engineering

The Main-MCU firmware is distributed as a binary only. This repository carries a documented, buildable C reconstruction of it, verified against static evidence (level 3 of 4):

- **Recovered architecture**: uC/OS-II kernel, 7 tasks, 60-IRQ dispatch, the UART protocol spoken with the APRS coprocessor (verified by executed host conformance tests), ST7735 display stack, two AT1846S transceivers (frequency formula and register map cross-checked against the AT1846S Programming Guide 1.4 and OpenRTX), CJK/ASCII font chain in external SPI flash.
- **Vendored code**: uC/OS-II V2.93.01 (Apache-2.0) and the GigaDevice GD32F10x SPL V2.2.1.
- **Status and provenance**: [`reconstruction/README.md`](reconstruction/README.md)
- **Full research log with addresses and evidence**: [`docs/RESEARCH_LOG.md`](docs/RESEARCH_LOG.md)

Without an emulator or bench hardware, claims are limited to "builds and is consistent with the static evidence"; the protocol layer passes the executed conformance tests.

## Hardware

| Specification | Value |
|---------------|-------|
| **VHF** | 136-174 MHz, 5W High/1W Low |
| **UHF** | 400-470 MHz, 4W High/1W Low |
| **APRS** | 1200 baud (hardware TNC) |
| **GPS** | GPS, BeiDou, GLONASS |
| **Bluetooth** | KISS TNC mode, pair code: 1234 |
| **Battery** | 7.4V 2500mAh Li-Po (USB Micro or USB-C in recent variants) |
| **Channels** | 128 |
| **Sensors** | BMP280/BMP085, DS18B20 |
| **Display** | Color LCD with day/night mode |
| **Antenna** | SMA-J (center pin) |

## In-Depth Documentation

| Topic | File | Description |
|-------|------|-------------|
| Dual-CPU architecture | [`docs/ARCHITECTURE.md`](docs/ARCHITECTURE.md) | Block diagram, UART protocol, shared memory |
| Firmware inventory | [`docs/FIRMWARE_INVENTORY.md`](docs/FIRMWARE_INVENTORY.md) | All available firmwares (bin, hex, exe) with links and status |
| Source code analysis | [`docs/SOURCE_CODE_ANALYSIS.md`](docs/SOURCE_CODE_ANALYSIS.md) | Functions, variables, protocols, file structure |
| Source index | [`docs/SOURCE_INDEX.md`](docs/SOURCE_INDEX.md) | Complete list of the 73 source files with descriptions |
| Firmware versions | [`docs/FIRMWARE_VERSIONS.md`](docs/FIRMWARE_VERSIONS.md) | Full history for both CPUs, flashing instructions |
| Hardware and sensors | [`docs/HARDWARE.md`](docs/HARDWARE.md) | Technical specifications, pinout, telemetry format |
| Community and resources | [`docs/COMMUNITY.md`](docs/COMMUNITY.md) | Groups.io, GitHub, blog, contributors, tools |
| Reverse-engineering log | [`docs/RESEARCH_LOG.md`](docs/RESEARCH_LOG.md) | Sessions 1-19: addresses, evidence, corrections for the Main MCU |

## Community Resources

- **Groups.io:** https://groups.io/g/HG-UV98-users (132+ members)
- **Main GitHub:** https://github.com/marrold/HG-UV98
- **Reseller:** https://www.venus-itech.com/product/hg-uv98-handheld-with-aprs/
- **HVDN Reviews:** https://hvdnnotebook.blogspot.com/search/label/HG-UV98

## Licenses

The APRS coprocessor source code was provided by Wuxi Venus Information Technology Co., Ltd with permission to modify and distribute. The original license is not attached explicitly, but BSD-3-Clause has been used provisionally.

The Main-MCU reconstruction vendors uC/OS-II V2.93.01 (Apache-2.0) and the GigaDevice GD32F10x SPL V2.2.1 (SLA-GD0001, redistribution permitted for execution on GD devices); details in [`reconstruction/README.md`](reconstruction/README.md).

## Contributing

This repository is a community effort to preserve and document HG-UV98 information. Contributions are welcome:
- Documentation corrections
- Source code analysis (especially of the Main MCU if it ever surfaces)
- Troubleshooting and workarounds
- Translations
- Reverse engineering of the ARM firmware (GD32F103) — continue the work in [`reconstruction/`](reconstruction/README.md), see [`docs/RESEARCH_LOG.md`](docs/RESEARCH_LOG.md)
