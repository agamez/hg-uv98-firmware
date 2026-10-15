# HG-UV98 - Comprehensive Reference Repository

Repositorio integral de documentación, código fuente, firmware y recursos comunitarios para el radio handheld **Lanch HG-UV98 APRS**.

## Descripción

El **HG-UV98** es un radio portátil dual banda (2m/70cm) con capacidades APRS (Automatic Packet Reporting System), GPS integrado, Bluetooth y sensores meteorológicos. Fabricado por Wuxi Venus Information Technology (Lanch), fue uno de los primeros radios APRS económicos chinos con funcionalidad completa.

## Estructura del Repositorio

```
HG-UV98/
├── docs/                          # Documentación
│   ├── FIRMWARE_VERSIONS.md       # Historial de versiones de firmware
│   ├── SOURCE_CODE_ANALYSIS.md    # Análisis del código fuente
│   ├── HARDWARE.md                # Referencia de hardware
│   └── COMMUNITY.md               # Recursos y comunidad
├── repos/                         # Repositorios clonados (referencia)
│   ├── pwnj0hn-HG-UV98/           # Binarios + software + manual
│   ├── marrold-HG-UV98/           # Repo principal comunidad
│   └── hg-uv98-firmware/          # Código fuente Keil (STC8051)
├── .gitignore
└── README.md
```

## Firmware

### Versiones de Firmware Disponibles

| Versión | Fecha | Descripción |
|---------|-------|-------------|
| V1.0.1_190416 | 2019-04 | Firmware original |
| V1.0.1_190619-KT | 2019-06 | Modificado por KT2KT (UI improvements) |
| APRS Sep 23 2020 | 2020-09 | Corrección grid square |
| UV98_868_220430_S4 | 2022-04 | Corrección coordenadas GPS |

### Código Fuente
El código fuente completo del firmware está disponible en `repos/hg-uv98-firmware/`:
- **MCU:** STC8A8K64D4 (8051 compatible)
- **IDE:** Keil µVision
- **Proyecto:** `UV98_868_220430_D4.uvproj`
- **Total líneas:** ~4000+ líneas de C

### Actualización de Firmware
1. Instalar drivers CH340 USB
2. Conectar USB al radio
3. Mantener PTT pulsado + encender → "Host UpData..."
4. Abrir TeraTerm a 115200 baudios
5. Pulsar "1"
6. Transfer → Send File → Ymodem
7. Seleccionar archivo .bin
8. Esperar (~3 min)
9. Reiniciar radio

## Hardware

| Especificación | Valor |
|----------------|-------|
| **VHF** | 136-174 MHz, 5W High/1W Low |
| **UHF** | 400-470 MHz, 4W High/1W Low |
| **APRS** | 1200 baud (hardware TNC) |
| **GPS** | GPS, BeiDou, GLONASS |
| **Bluetooth** | KISS TNC mode |
| **Batería** | 7.4V 2500mAh Li-Po |
| **Canales** | 128 |
| **Sensores** | BMP280/BMP085, DS18B20 |

## Recursos Comunitarios

- **Grupos.io:** https://groups.io/g/HG-UV98-users (132+ miembros)
- **GitHub Principal:** https://github.com/marrold/HG-UV98
- **Reseller:** https://www.venus-itech.com/product/hg-uv98-handheld-with-aprs/
- **Reviews HVDN:** https://hvdnnotebook.blogspot.com/search/label/HG-UV98

## Licencias del Firmware

El código fuente del firmware fue proporcionado por Wuxi Venus Information Technology Co., Ltd con permiso para modificar y distribuir. La licencia original no está adjunta explícitamente, pero se ha utilizado BSD-3c provisionalmente. Ver `repos/hg-uv98-firmware/LICENSE` para más detalles.

## Contribuir

Este repositorio es un esfuerzo comunitario para preservar y documentar la información del HG-UV98. Contribuciones son bienvenidas:
- Correcciones de documentación
- Análisis de código fuente
- Troubleshooting y workarounds
- Traducciones