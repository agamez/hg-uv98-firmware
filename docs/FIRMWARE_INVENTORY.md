# HG-UV98 Firmware Inventory

## Main MCU Firmware (GD32F103RET6 - ARM Cortex-M3)

| Version | Date | File | Size | Source | Status |
|---------|------|------|------|--------|--------|
| V1.0.1_190416 | 2019-04-16 | `MAIN-MCU-HG-UV98-V1.0.1-190416.bin` | 267 KB | [groups.io attachment](https://groups.io/g/HG-UV98-users/attachment/60/0/HG-UV98-V1.0.1-190416.bin) | ✅ Downloaded |
| V1.0.1_190619-KT | 2019-06-19 | `extracted/V1.0.1_190619-KT.bin` | 496 KB | [pwnj0hn/HG-UV98](https://github.com/pwnj0hn/HG-UV98/blob/main/Firmware%20radio/V1.0.1_190619-KT.zip) (KT2KT modified) | ✅ Downloaded |
| V1.01_190606 | 2019-06-06 | N/A | - | Mentioned in groups.io (APRS Setup program) | ❌ Not available |
| V1.2_210517 | 2021-05-17 | N/A | - | [chirpmyradio issue #11281](https://www.chirpmyradio.com/issues/11281) - different ident string (P3LL vs P3107) | ❌ Not available |
| V1.2_211026 | 2021-10-26 | N/A | - | [groups.io](https://groups.io/g/HG-UV98-users/topics) - reported by user | ❌ Not available |

**Notes:**
- V1.0.1_190416 is the original firmware version that came with the radio
- V1.0.1_190619-KT is a UI modification by KT2KT (changes "Sure" to "OK", improves menu labels)
- V1.2 versions have a different identification string (`P3LL` vs `P3107`) and require new driver support in CHIRP
- V1.2 firmwares are NOT publicly available - only reported by users who have them installed

### Flashing Main MCU Firmware

1. Install CH340 USB drivers
2. Hold PF2 (lower side key) + power on → programming mode
3. Use CPS (Channel Programming Software) to write channels/settings
4. Binary update requires proprietary tools (not publicly documented)

---

## APRS Coprocessor Firmware (STC8A8K64D4 - 8051 Architecture)

| Version | Date | File | Size | Source | Status |
|---------|------|------|------|--------|--------|
| APRS_51G3_4K_20180211A | 2018-02-11 | N/A | - | README.C version history | ❌ Not available |
| APRS_51G3_4K_20180221 | 2018-02-21 | N/A | - | README.C version history | ❌ Not available |
| APRS_51G3_4K_20180408 | 2018-04-08 | N/A | - | README.C version history | ❌ Not available |
| APRS_8K_20180922_D | 2018-09-22 | N/A | - | README.C version history | ❌ Not available |
| HG_UV98_20190501 | 2019-05-01 | N/A | - | Mentioned by user in groups.io | ❌ Not available |
| APRS_HG_UV98_20190212 | 2019-02-12 | N/A | - | README.C version history | ❌ Not available |
| HG_UV98_20200923 | 2020-09-23 | `Venus_RAR_extracted/APRS_HG_UV98_20200923 firmware.exe` | 232 KB | [Venus i-Tech RAR](http://venus-itech.com/download/HG-UV98%20aprs%20firmware.rar) (extracted) | ✅ Downloaded |
| UV98_868_220430_S4 | 2022-04-30 | `UV98_868_220430_S4.exe` | 248 KB | [pwnj0hn/HG-UV98](https://github.com/pwnj0hn/HG-UV98/blob/main/Firmware%20APRS/UV98_868_220430_S4.exe) | ✅ Downloaded |
| UV98_868_220430_D4 (Source) | 2022-04-30 | `UV98_868_220430_D4.hex` | 129 KB | [agamez/hg-uv98-firmware](https://github.com/agamez/hg-uv98-firmware) (compiled from source) | ✅ Downloaded |
| UV98_868_220430_D4 (Source) | 2022-04-30 | `UV98_868_220430_D4.compressed.bin` | 1.8 MB | [agamez/hg-uv98-firmware](https://github.com/agamez/hg-uv98-firmware) (compiled, compressed) | ✅ Downloaded |

**Notes:**
- The latest publicly available APRS firmware is **UV98_868_220430_S4** (April 2022)
- The source code (UV98_868_220430_D4) compiles to a 129KB hex file
- The manufacturer (Venus Information Technology/Lanch) website is now offline (404)
- The September 2020 version fixed the grid square calculation bug
- The April 2022 version fixed GPS coordinate calculation issues (including southern hemisphere)

### Flashing APRS Coprocessor Firmware

1. Install CH340 USB drivers
2. Connect USB cable to radio (CH340)
3. Hold **PTT button + power on** → "Host UpData..." appears
4. Open TeraTerm at **115200 baud, 8N1**
5. Press **"1"** when prompted
6. Go to **Transfer → Send File**
7. Select **Ymodem** protocol
8. Select firmware `.bin` file (not `.exe` - must be binary)
9. Wait for completion (~3 minutes)
10. **Power cycle** the radio

**⚠️ Warning:** Flashing only one firmware without the other can leave the radio non-functional.

---

## Flash File Formats

### Main MCU (ARM)
- **Format:** Raw binary (.bin)
- **Size:** ~267-496 KB (varies by version)
- **Flash size:** 512 KB (GD32F103RET6)
- **Tool:** CPS (Channel Programming Software) or proprietary flasher

### APRS Coprocessor (8051)
- **Source:** Keil µVision project → generates .hex (Intel HEX format)
- **Compiled:** .hex (129 KB) or compressed binary (1.8 MB)
- **Manufacturer distribution:** Windows .exe (self-extracting/flasher tool)
- **Flash size:** 64 KB (STC8A8K64D4) but firmware is only ~8KB
- **Tool:** STC-ISP software (for direct programming) or TeraTerm/Ymodem (for radio upgrade mode)

---

## Known Firmware Issues

### Main MCU (ARM)
| Issue | Fixed In |
|-------|----------|
| Menu labels say "Sure" instead of "OK" | V1.0.1_190619-KT |
| Confusing menu descriptions | V1.0.1_190619-KT |
| V1.2 hardware incompatibility with CHIRP | Unknown (new ident string) |

### APRS Coprocessor (8051)
| Issue | Fixed In |
|-------|----------|
| Grid square calculation (western latitudes) | HG_UV98_20200923 (Sep 2020) |
| GPS coordinate calculation (southern hemisphere) | UV98_868_220430_S4 (Apr 2022) |
| ARISS path bug (requires hop number) | Not fixed - still present |
| Smart beacon lockup (Host UpData... mode) | Not fixed - hardware issue |
| Bluetooth module quality (later batches) | Not fixed - hardware issue |
| Celsius/Fahrenheit toggle | Not fixed |

---

## Source Code Availability

| Component | Source Available? | Location |
|-----------|------------------|----------|
| APRS Coprocessor (8051) | ✅ YES | [agamez/hg-uv98-firmware](https://github.com/agamez/hg-uv98-firmware) |
| Main MCU (ARM GD32) | ❌ NO | Proprietary - not publicly available |

The APRS coprocessor source was released by Wuxi Venus Information Technology with permission to modify and distribute. The source compiles to the 8KB firmware that runs on the STC8A8K64D4 (8051) MCU.

---

## Related Resources

- [Architecture documentation](./ARCHITECTURE.md) - Dual MCU system design
- [Source code analysis](./SOURCE_CODE_ANALYSIS.md) - Detailed breakdown of APRS firmware
- [Hardware reference](./HARDWARE.md) - Full specifications
- [Community resources](./COMMUNITY.md) - Groups.io, GitHub, blogs