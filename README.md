# HG-UV98 - Comprehensive Reference Repository

Repositorio integral de documentación, código fuente y recursos comunitarios para el radio handheld **Lanch HG-UV98 APRS**.

## ⚠️ Arquitectura de Doble Procesador

El HG-UV98 utiliza **dos microcontroladores independientes** comunicándose vía UART:

| Procesador | MCU | Arquitectura | Función | Firmware |
|-----------|-----|--------------|---------|----------|
| **Main MCU** | GD32F103RET6 | ARM Cortex-M3 (32-bit, 108 MHz) | Radio FM, display, menús, RF, canales | ❌ Binario solo (privado) |
| **APRS Coprocessor** | STC8A8K64D4 | 8051 (8-bit, 22.1184 MHz) | APRS, GPS, Bluetooth, sensores | ✅ **Código fuente disponible** |

El código fuente en este repositorio es del **coprocesador APRS (8051)**, NO del procesador principal de radio (ARM).

## Estructura del Repositorio

```
HG-UV98/
├── README.md                          # Este archivo
├── .gitignore
├── docs/                              # Documentación
│   ├── ARCHITECTURE.md                # Arquitectura dual de CPUs + diagrama
│   ├── FIRMWARE_VERSIONS.md           # Historial de versiones de firmware (ambos CPUs)
│   ├── SOURCE_CODE_ANALYSIS.md        # Análisis del código fuente del coprocesor APRS
│   ├── SOURCE_INDEX.md                # Índice completo de los 73 archivos fuente
│   ├── HARDWARE.md                    # Referencia de hardware y sensores
│   └── COMMUNITY.md                   # Recursos y comunidad
├── src/firmware-source/               # Código fuente del coprocesor APRS
│   └── UV98_868_220430_D4/
│       ├── MAIN/   (37 archivos .C/.H)    # Firmware principal del coprocesor
│       └── SENSOR/ (6 archivos .C/.H)     # Drivers de sensores (BMP280, BMP085, DS18B20)
└── repos/                             # Repositorios clonados (referencia)
    ├── pwnj0hn-HG-UV98/               # Binarios + software + manual del radio
    ├── marrold-HG-UV98/               # Repo principal de la comunidad
    └── hg-uv98-firmware/              # Proyecto Keil (referencia, no incluido en commit)
```

## Código Fuente del Coprocesor APRS

| Especificación | Valor |
|----------------|-------|
| **MCU** | STC8A8K64D4 (8051/MCS-51 compatible) |
| **Clock** | 22.1184 MHz (preciso para baud rates UART) |
| **IDE** | Keil µVision (C51) |
| **Tamaño firmware** | ~8KB ("8K CPU") |
| **Archivos C/H** | 73 archivos |
| **Líneas de código** | ~7,500+ líneas de C |
| **Total repositorio** | ~17,000 líneas (incluyendo docs) |

### Módulos del firmware
- APRS RF codec (1200 baud FDMDO via CMX865A/FX604)
- GPS NMEA parsing (GPRMC, GPGGA - GPS, BeiDou, GLONASS)
- Bluetooth KISS TNC mode
- Smart beacon (intervalo adaptativo basado en movimiento)
- Mic-E encoding
- Digipeater forwarding
- Weather sensors (BMP280/BMP085, DS18B20)
- Navigation (distance, bearing, azimuth)
- Odometer/mileage tracking
- AT command interface (communicates with Main MCU)

## Firmware (Ambos CPUs)

### Versiones de Firmware del Coprocesor APRS

| Versión | Fecha | Descripción |
|---------|-------|-------------|
| 2018-02-11 | APRS_51G3_4K_20180211A | GPS sync, Bluetooth |
| 2018-04-08 | APRS_51G3_4K_20180408 | Curve interface |
| 2018-09-22 | APRS_8K_20180922_D | Bluetooth 512 bytes to host |
| 2020-09-23 | APRS Sep 23rd 2020 | ✅ Grid square fix |
| 2022-04-30 | UV98_868_220430_S4 | ✅ GPS coordinates fix |

### Firmware del Main MCU (ARM)

| Versión | Fecha | Descripción |
|---------|-------|-------------|
| V1.0.1_190416 | 2019-04 | Firmware original |
| V1.0.1_190619-KT | 2019-06 | KT2KT modified (UI improvements) |

**⚠️ Importante:** Ambos firmwares deben ser compatibles. Flashar solo uno sin el otro puede dejar el radio inutilizable.

## Hardware

| Especificación | Valor |
|----------------|-------|
| **VHF** | 136-174 MHz, 5W High/1W Low |
| **UHF** | 400-470 MHz, 4W High/1W Low |
| **APRS** | 1200 baud (hardware TNC) |
| **GPS** | GPS, BeiDou, GLONASS |
| **Bluetooth** | KISS TNC mode, pair code: 1234 |
| **Batería** | 7.4V 2500mAh Li-Po (USB Micro o USB-C en variantes recientes) |
| **Canales** | 128 |
| **Sensores** | BMP280/BMP085, DS18B20 |
| **Display** | Color LCD con modo día/noche |
| **Antena** | SMA-J (center pin) |

## Documentación Profunda

| Tema | Archivo | Descripción |
|------|---------|-------------|
| Arquitectura dual de CPUs | [`docs/ARCHITECTURE.md`](docs/ARCHITECTURE.md) | Diagrama de bloques, protocolo UART, memoria compartida |
| Análisis del código fuente | [`docs/SOURCE_CODE_ANALYSIS.md`](docs/SOURCE_CODE_ANALYSIS.md) | Funciones, variables, protocolos, estructura de archivos |
| Índice de fuentes | [`docs/SOURCE_INDEX.md`](docs/SOURCE_INDEX.md) | Lista completa de los 73 archivos fuente con descripción |
| Versiones de firmware | [`docs/FIRMWARE_VERSIONS.md`](docs/FIRMWARE_VERSIONS.md) | Historial completo para ambos CPUs, instrucciones de flash |
| Hardware y sensores | [`docs/HARDWARE.md`](docs/HARDWARE.md) | Especificaciones técnicas, pinout, formato de telemetría |
| Comunidad y recursos | [`docs/COMMUNITY.md`](docs/COMMUNITY.md) | Grupos.io, GitHub, blog, contribuidores, herramientas |

## Recursos Comunitarios

- **Grupos.io:** https://groups.io/g/HG-UV98-users (132+ miembros)
- **GitHub Principal:** https://github.com/marrold/HG-UV98
- **Reseller:** https://www.venus-itech.com/product/hg-uv98-handheld-with-aprs/
- **Reviews HVDN:** https://hvdnnotebook.blogspot.com/search/label/HG-UV98

## Licencias

El código fuente del coprocesor APRS fue proporcionado por Wuxi Venus Information Technology Co., Ltd con permiso para modificar y distribuir. La licencia original no está adjunta explícitamente, pero se ha utilizado BSD-3c provisionalmente.

## Contribuir

Este repositorio es un esfuerzo comunitario para preservar y documentar la información del HG-UV98. Contribuciones son bienvenidas:
- Correcciones de documentación
- Análisis de código fuente (especialmente del Main MCU si se consigue)
- Troubleshooting y workarounds
- Traducciones
- Análisis de ingeniería inversa del firmware ARM (GD32F103)