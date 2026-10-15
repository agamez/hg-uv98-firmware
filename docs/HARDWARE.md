# HG-UV98 Hardware Reference

## Specifications

### Radio
| Parameter | Value |
|-----------|-------|
| **Frequency Range** | 136-174 MHz (VHF), 400-470 MHz (UHF) |
| **Transmit Power** | VHF: 5W High, 1W Low / UHF: 4W High, 1W Low |
| **Channels** | 128 memory channels |
| **Modulation** | Analog FM |
| **APRS Speed** | 1200 baud (hardware TNC) |
| **Dual Watch** | Same band (not same-channel dual receive) |
| **Dual RX** | Yes, VHF + UHF simultaneously |

### Battery & Power
| Parameter | Value |
|-----------|-------|
| **Battery Type** | 7.4V 2500mAh Li-Po |
| **Charging** | USB Micro (or USB-C on later variants) |
| **Charger** | 12V→5V cigar adapter or 120V AC→5W wall wart |
| **Optional** | Drop-in charge cradle |
| **Voltage Telemetry** | Built-in, sent via APRS |

### APRS Features
| Feature | Available |
|---------|-----------|
| **GPS** | GPS, BeiDou, GLONASS selectable |
| **Bluetooth** | Yes (KISS TNC mode) |
| **Temperature** | BMP280 or DS18B20 |
| **Barometric Pressure** | BMP280/BMP085 |
| **Beacon** | Timed, PTT-triggered, Smart Beacon |
| **DIGI** | Built-in digipeater |
| **MIC-E** | Encoded position reporting |
| **Navigation** | Dynamic distance and bearing |
| **Path** | WIDE1-1, WIDE2-1, custom |
| **UNPROTO** | Per-schedule configuration |

### Physical
| Parameter | Value |
|-----------|-------|
| **Display** | Color LCD with day/night mode |
| **Antenna Connector** | SMA-J (center pin) |
| **Speaker Mic** | 2-pin Kenwood/Baofeng compatible |
| **Bottom LED** | Programmable |
| **Buttons** | 2x PF programmable + PTT |
| **Belt Clip** | Included |

### Software/Programming
| Tool | Description |
|------|-------------|
| **CPS** | HG-UV98 EN V2.0 setup.rar - Channel programming |
| **APRS Config** | APRS_51Serial_20181009.rar or APRS 51TNC-ENG.zip |
| **Firmware Update** | Via USB + TeraTerm (Ymodem) |
| **Bluetooth** | Pair code: 1234 (default) |
| **APRSdroid** | Compatible via Bluetooth |
| **Pinpoint APRS** | Compatible via Bluetooth |

## Manufacturer

| Detail | Value |
|--------|-------|
| **Company** | Wuxi Venus Information Technology Co., Ltd |
| **Brand** | Lanch |
| **Website** | http://yw.qzlcdz.com (often offline) |
| **Current Website** | http://www.qzlcdz.cn/ |
| **Reseller** | Venus iTech - https://www.venus-itech.com/product/hg-uv98-handheld-with-aprs/ |
| **WeChat** | 13013684000 |
| **Taobao** | BH4TDV.TAOBAO.com |

## Microcontroller Details

### STC8A8K64D4 (STC International)
| Spec | Value |
|------|-------|
| **Architecture** | 8051 (MCS-51 compatible) |
| **Flash** | 64KB |
| **Clock** | Up to 35 MHz (running at 22.1184 MHz) |
| **RAM** | 2560 bytes (XRAM) |
| **EEPROM** | Built-in (external AT24C512 used) |
| **UARTs** | 4x UART (UART1-4 used in firmware) |
| **PWM** | Multiple channels (PCA-based) |
| **ADC** | 10-bit ADC |
| **GPIO** | 47 I/O pins |

## Sensor Hardware

### BMP280 (Primary)
| Spec | Value |
|------|-------|
| **Interface** | I2C |
| **Measurement** | Temperature, Barometric pressure |
| **Pressure Range** | 300-1100 hPa |
| **Temperature Range** | -40°C to +85°C |
| **APRS Output** | METAR format weather telemetry |

### BMP085 (Legacy)
| Spec | Value |
|------|-------|
| **Interface** | I2C |
| **Older replacement** | Used in earlier hardware revisions |
| **Pressure Range** | 300-1100 hPa |

### DS18B20 (Optional)
| Spec | Value |
|------|-------|
| **Interface** | 1-Wire (P3.6) |
| **Temperature Range** | -55°C to +125°C |
| **Resolution** | 9-12 bit configurable |

## Pinout & Port Map

### From Firmware (UART4.C / BT.C / MAIN files)
| Pin | Function | Description |
|-----|----------|-------------|
| P2.1 | PTT | Push-to-talk (push-pull) |
| P2.2 | FX604_TX | Codec transmit |
| P2.3 | FX604_DET | Codec detect |
| P2.4 | FX604_RX | Codec receive |
| P2.6 | FX604_M0 | Codec mode |
| P2.7 | GPS_EN | GPS enable (push-pull) |
| P3.0 | RXD1 | UART1 receive |
| P3.1 | TXD1 | UART1 transmit (debug) |
| P3.6 | Side Key 2 (A09) | PF2 - Programming mode |
| P3.7 | Side Key 3 (A10) | PF3 - Alarm |
| P0.0 | RXD3 | UART3 receive (GPS) |
| P0.1 | TXD3 | UART3 transmit (GPS) |
| P4.2 | LED indicators | Status LED |

## Telemetry Packet Format

APRSTel beacon payload format:
```
;.{temp}V{voltage}H{altitude}M{pressure}S{speed}@{lat}/{lon}d{dir}m{km}
```

Example:
```
;.;u >>>/3r} 8.3V 25.6C 1016.3pa KG928*70
```

- Temperature in Celsius
- Battery voltage in volts
- Altitude in feet
- Pressure in hPa/mBar
- Grid square (Maidenhead locator)
- Speed in knots
- Direction in degrees
- Mileage (odometer)