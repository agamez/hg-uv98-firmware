# Índice de Código Fuente del Firmware HG-UV98

## Resumen
- **Archivo:** UV98_868_220430_D4.uvproj (Keil µVision)
- **MCU:** STC8A8K64D4 (8051, 22.1184 MHz)
- **Total archivos:** 83
- **Total líneas C/H:** ~7,500+

## Archivos de la Main (MAIN/)

### Nucleo del Sistema
| Archivo | Líneas | Descripción |
|---------|--------|-------------|
| `GPRS_51GATE.c` | 279 | **Main entry** - Initial_uv98(), timer interrupts, system init |
| `README.C` | 144 | Release notes y cambios de versión desde 2018 |
| `PUBLIC_BUF.C` | 19 | Buffer global compartido (512 bytes EEPROM) |
| `DELAY.C` | 77 | Retardos y delay functions |
| `DELAY.H` | 8 | Header delays |

### GPS y Posicionamiento
| Archivo | Líneas | Descripción |
|---------|--------|-------------|
| `GPS2.C` | 1,155 | Parsing GPRMC/GPGGA, smart beacon, mileage |
| `GPS2.H` | 62 | GPS variables extern declarations |
| `GPS_JULI.C` | 559 | Distance/bearing calculation (Julian date) |
| `GPS_JULI.H` | 12 | GPS juli header |
| `GPS_CHECKSUM.C` | 82 | GPS NMEA checksum verification |
| `GPS_CHECKSUM.H` | 13 | Checksum header |
| `DISP_GPS.C` | 858 | GPS data display routines |
| `DISP_GPS.H` | 27 | Display header |

### APRS RF Codec
| Archivo | Líneas | Descripción |
|---------|--------|-------------|
| `APRS_RF.C` | 352 | FDMDO 1200 baud codec, HDLC framing, CRC16 |
| `APRS_RF.H` | 8 | APRS RF header |

### KISS Protocol
| Archivo | Líneas | Descripción |
|---------|--------|-------------|
| `KISS_DECODE.C` | 64 | KISS protocol decoding |
| `KISS_DECODE.H` | 16 | KISS header |
| `KISS2ASC.C` | 316 | KISS to ASCII conversion |
| `KISS2ASC.H` | 11 | KISS2ASC header |
| `KISS2GAOMIN.C` | 509 | KISS to custom protocol |
| `KISS2GAOMIN.H` | 8 | KISS2GAOMIN header |
| `KISS_Analysis.C` | 544 | KISS packet analysis |
| `KISS_Analysis.H` | 13 | KISS analysis header |

### APRS Beacon & Digipeater
| Archivo | Líneas | Descripción |
|---------|--------|-------------|
| `BEACON.C` | 605 | APRS beacon transmission logic |
| `BEACON.H` | 11 | Beacon header |
| `BEACON_SAVE.C` | 304 | Beacon persistence to EEPROM |
| `BEACON_SAVE.H` | 4 | Beacon save header |
| `DIGI.C` | 640 | Digipeater functionality |
| `DIGI.H` | 24 | Digipeater header |

### MIC-E Encoding
| Archivo | Líneas | Descripción |
|---------|--------|-------------|
| `MICE_DECODE.C` | 569 | MIC-E decode |
| `MICE_DECODE.H` | 12 | MIC-E decode header |
| `MICE_CODE.C` | 341 | MIC-E encode |
| `MICE_CODE.H` | 12 | MIC-E encode header |

### CMX865A RF Transceiver
| Archivo | Líneas | Descripción |
|---------|--------|-------------|
| `CMX865A_DECODE.C` | 577 | RF demodulation/decode |
| `CMX865A_DECODE.H` | 20 | Decode header |
| `CMX865A_SPI.C` | 487 | SPI communication with CMX865A |
| `CMX865A_SPI.H` | 35 | SPI header |
| `CMX865A_CODE.C` | 295 | Modulation codes |
| `CMX865A_CODE.H` | 11 | Code header |

### Bluetooth
| Archivo | Líneas | Descripción |
|---------|--------|-------------|
| `BT.C` | 359 | Bluetooth module handling, KISS TNC |
| `BT.H` | 10 | Bluetooth header |

### UART (Serial Communication)
| Archivo | Líneas | Descripción |
|---------|--------|-------------|
| `UART1.C` | 159 | UART1 - Debug/programming |
| `UART1.H` | 9 | UART1 header |
| `UART2.C` | 155 | UART2 - Bluetooth interface |
| `UART2.H` | 9 | UART2 header |
| `UART3.C` | 252 | UART3 - GPS module |
| `UART3.H` | 8 | UART3 header |
| `UART4.C` | 140 | UART4 - EEPROM/refresh |
| `UART4.H` | 7 | UART4 header |
| `UARTX.C` | 560 | Generic UART helpers |
| `UARTX.H` | 14 | UARTX header |

### Memory (EEPROM/I2C)
| Archivo | Líneas | Descripción |
|---------|--------|-------------|
| `STC_EEPROM.C` | 509 | Internal EEPROM emulation |
| `STC_EEPROM.H` | 29 | EEPROM header |
| `AT24C512.C` | 269 | I2C AT24C512 EEPROM driver |
| `AT24C512.H` | 38 | AT24C512 header |

### ADC and Sensors
| Archivo | Líneas | Descripción |
|---------|--------|-------------|
| `ADC.C` | 83 | ADC driver |
| `ADC.H` | 23 | ADC header |

### Channel Management
| Archivo | Líneas | Descripción |
|---------|--------|-------------|
| `CHx.C` | 87 | Channel operations |
| `CHx.H` | 13 | Channel header |

### Functions
| Archivo | Líneas | Descripción |
|---------|--------|-------------|
| `FUN_B.C` | 393 | Function group B |
| `FUN_B.H` | 15 | FUN_B header |
| `FUN_C.C` | 1000 | Function group C |
| `FUN_C.H` | 15 | FUN_C header |
| `PTT_FUN.C` | 36 | PTT functions |
| `PTT_FUN.H` | 8 | PTT header |

### Utilities
| Archivo | Líneas | Descripción |
|---------|--------|-------------|
| `CRC16.C` | 96 | CRC16 calculation |
| `CRC16.H` | 20 | CRC16 header |
| `tostring.C` | 167 | Integer to string |
| `tostring.H` | 32 | tostring header |
| `HEX_TO_ASC.C` | 50 | Hex to ASCII |
| `HEX_TO_ASC.H` | 13 | Hex header |
| `LED.C` | 55 | LED control |
| `LED.H` | 9 | LED header |

### MCU Headers
| Archivo | Descripción |
|---------|-------------|
| `STC15Fxxxx.H` | STC15 series MCU (compatibility) |
| `STC8A8K64D4.H` | STC8A8K64D4 MCU header |
| `IO.H` | I/O pin definitions |

## Archivos del Sensor (SENSOR/)

| Archivo | Líneas | Descripción |
|---------|--------|-------------|
| `bmp280.C` | 512 | BMP280 I2C pressure/temp sensor |
| `bmp280.H` | 15 | BMP280 header |
| `BMP085.C` | 499 | BMP085 (older) pressure/temp |
| `BMP085.H` | 20 | BMP085 header |
| `DS18B20.C` | 184 | 1-Wire temperature sensor |
| `DS18B20.H` | 21 | DS18B20 header |

## Compilación

### Requisitos
1. Keil µVision IDE (MDK-ARM o C51)
2. STC8051 device pack
3. Proyecto: `UV98_868_220430_D4.uvproj`

### Output
- El hex file se genera en: `OBJ/UV98_868_220430_D4.hex`

## Notas de Compilación

- El código usa delay macros específicos del hardware 8051
- `uchar` = unsigned char, `uint` = unsigned int, `ulong` = unsigned long
- `bit` = C51 bit type
- `sbit` = special bit register
- La compilación no es trivial: requiere el toolchain exacto de Keil para 8051