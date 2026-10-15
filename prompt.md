# REA Reverse Engineering Prompt

## Objective

**Reverse engineer the main MCU firmware (GD32F103RET6 ARM Cortex-M3) of the HG-UV98 radio from binary `.bin` files to original C source code.** The ideal outcome is a complete C project that compiles and produces a `.bin`/`.hex` output **byte-for-byte identical** to the original manufacturer firmware.

---

## Context

The HG-UV98 is a Chinese dual-band APRS handheld radio (Lanch/Wuxi Venus Information Technology) with a **dual-MCU architecture**:

| Component | Main MCU (TARGET OF THIS TASK) | APRS Coprocessor |
|-----------|-------------------------------|-------------------|
| MCU | **GD32F103RET6** | STC8A8K64D4 |
| Architecture | **ARM Cortex-M3 (32-bit)** | 8051 (8-bit) |
| Clock | 108 MHz | 22.1184 MHz |
| Flash | 512 KB | 64 KB |
| RAM | 64 KB | 2560 bytes XRAM |
| Firmware | **Binary only (proprietary)** | Source available (this repo) |
| Role | Radio FM, display, menus, RF, channels | APRS, GPS, Bluetooth, sensors |

The source code of the **APRS coprocessor (8051)** is already available in this repository and was released by the manufacturer. We need the same for the **main ARM MCU**.

---

## Available Resources

### 1. Binary Firmware Files (in `firmware/` directory)

- **`firmware/MAIN-MCU-HG-UV98-V1.0.1-190416.bin`** (267 KB) — Original firmware from groups.io attachment, dated 2019-04-16. This is the **earliest publicly available version** and should be the primary target.
- **`firmware/V1.0.1_190619-KT.bin`** (496 KB) — Modified firmware by KT2KT (April 2019). Changes "Sure" → "OK" in UI, improves menu descriptions. Useful as a **diff target** to identify which strings/sections changed between V1.0.1_190416 and V1.0.1_190619-KT.

### 2. Documentation in Repository

- **`docs/ARCHITECTURE.md`** — Dual-MCU system diagram, UART protocol between CPUs, shared EEPROM map
- **`docs/FIRMWARE_INVENTORY.md`** — All known firmware versions, flash procedures, version history
- **`docs/SOURCE_CODE_ANALYSIS.md`** — Analysis of the APRS coprocessor (useful for understanding the communication protocol)
- **`docs/HARDWARE.md`** — Hardware specs, sensor details, telemetry formats
- **`docs/FIRMWARE_VERSIONS.md`** — Version history with changelog details
- **`docs/COMMUNITY.md`** — Community resources, known issues, user reports

### 3. Submodule Repositories

- **`repos/pwnj0hn-HG-UV98/`** — Original firmware packages: binaries, CPS software, APRS configuration software, manual
- **`repos/marrold-HG-UV98/`** — Community repository with issues, firmware discussions, user reports
- **`repos/hg-uv98-firmware/`** — Source code for the APRS coprocessor (8051) — useful for understanding the UART protocol

### 4. External Resources

- **GigaDevice GD32 SDK**: Available at https://www.gd32mcu.com/en/download/ — Contains peripheral libraries, register definitions, and examples for GD32F103 series
- **Keil MDK ARM / GCC ARM**: Standard toolchains for ARM Cortex-M3
- **ST-Link/OpenOCD**: For firmware extraction from physical hardware
- **Chirp driver source**: `hg_uv98.py` (Python) from CHIRP project — reverse-engineered driver showing the clone protocol

### 5. Known Behavioral Information

From changelogs and community reports, the firmware:

- Implements FM radio functions: VHF 136-174MHz, UHF 400-470MHz, 128 memory channels
- Manages color LCD display with day/night mode
- Implements keypad/button interface (2x PF programmable + PTT)
- Handles audio: mic input, speaker output, PL/CTCSS/DCS tones
- Implements menu system with numbered options and sub-menus
- Communicates with APRS coprocessor via UART4 using custom AT-command protocol ($E01, $E02, $E00, $F01, $A09, etc.)
- Reads/writes AT24C512 EEPROM (64KB) — shared configuration storage
- Manages USB programming mode (via CH340)
- Implements battery voltage telemetry
- Changed in V1.0.1_190619-KT: "Sure" → "OK" button text, improved menu descriptions
- V1.2 versions have different hardware identification string (`P3LL` vs `P3107`)

---

## Required REA Tools & Approach

Use **all available REA tools** in this session. The typical REA workflow includes:

### Phase 1: Static Analysis

1. **`rea:file_read`** — Load the `.bin` files and examine raw hex
2. **`rea:strings`** / string extraction — Find ASCII/Unicode strings for menu labels, messages, callsign prompts
3. **`rea:headers`** — Identify embedded headers, magic numbers, version signatures
4. **`rea:disassemble`** — Disassemble the ARM code (Cortex-M3, little-endian, Thumb-2 instruction set)
5. **`rea:decompile`** — Decompile functions to C-like pseudocode
6. **`rea:analyze`** — Cross-reference functions, data segments, imports

### Phase 2: Structure Recovery

7. **`rea:identify_library_code`** — Match against known ARM libraries (GigaDevice CMSIS, standard peripheral library, CMSIS-DSP if present)
8. **`rea:identify_stdlib`** — Identify `printf`, `memcpy`, `strlen`, etc. for reassembly
9. **`rea:identify_framework`** — Detect RTOS (FreeRTOS?), startup code, interrupt vectors, startup assembly
10. **`rea:struct_recover`** — Reconstruct data structures from EEPROM map and known layouts
11. **`rea:constant_recovery`** — Recover magic numbers, register addresses, pin configurations

### Phase 3: Reconstruction

12. **`rea:code_reconstruct`** — Reconstruct C source from decompiled functions
13. **`rea:symbol_recover`** — Recover/assign meaningful function and variable names
14. **`rea:rename`** — Apply context-aware naming (e.g., `SetChannelFreq`, `DisplayMainMenu`, `ReadEEPROM`)
15. **`rea:format_code`** — Clean up and format the generated C code

### Phase 4: Validation

16. **`rea:build`** / compile the reconstructed C to `.bin`
17. **`rea:diff_binary`** — Compare output binary against original (byte-by-byte)
18. **`rea:patch`** — If differences exist, iterate until identical

---

## Specific Technical Guidance

### ARM Cortex-M3 Considerations

- The GD32F103RET6 is Cortex-M3 compatible — expect **Thumb-2** instruction set (16-bit and 32-bit instructions mixed)
- **Little-endian** byte order
- Start with the **vector table** at address `0x00000000` — identify reset handler, NMI, HardFault, SysTick, interrupt handlers
- Look for **startup assembly** (`.s` file equivalent) — stack pointer init, `SystemInit()`, `main()` call
- The GigaDevice CMSIS includes: `core_cm3.h`, `system_gd32f10x.c`, `gd32f10x_peripheral_lib`
- Common peripheral registers: RCC (clock), GPIO, USART, SPI, I2C, ADC, TIM, USB, FLASH, EEPROM (I2C)

### Disassembly Strategy

1. **Entry point**: ARM reset vector at `0x00000000` — first instruction is usually `LDR PC, [PC, #-0xFFC]` (stack pointer load)
2. **Stack pointer**: `0x20000000 + 64KB` = `0x20010000` (top of RAM)
3. **System memory**: `0x1FFFF000` (bootloader area) — firmware typically starts at `0x08000000` (internal Flash)
4. **Identify the binary format**: Check if it's raw binary, ELF, HEX, or has a custom loader/header

### Known Data Structures

From the APRS coprocessor source code, the **shared EEPROM map** (AT24C512, 64KB I2C EEPROM) is accessed by BOTH processors through a 512-byte buffer. The main MCU reads/writes:
- Channel data (frequency, PL/DCS, offset, power, step)
- Menu system settings
- Display configuration
- Audio settings
- USB programming parameters
- Clone protocol data (CHIRP uses `P3107` or `P3LL` identification string)
- Battery calibration values

### Comparison Strategy (V1.0.1_190416 vs V1.0.1_190619-KT)

Since we have TWO versions of the firmware:
1. Disassemble both independently
2. Compare the **text section** (`.rodata`) — string differences reveal what changed
3. The KT2KT modification only changed UI strings and descriptions — the core radio logic should be identical
4. This comparison validates the disassembly by confirming that only expected sections differ

---

## Deliverables

### Primary

1. **Complete C source code** for the main MCU firmware, organized in files matching the original project structure:
   ```
   gd32f103_firmware/
   ├── startup/
   │   ├── startup_gd32f103.s      # Assembly startup + vector table
   │   └── system_gd32f10x.c       # System init, clock config
   ├── core/
   │   ├── main.c                  # Main entry point, system loop
   │   └── gd32f10x_it.c           # Interrupt handlers
   ├── peripherals/
   │   ├── gpio.c                  # GPIO configuration
   │   ├── usart.c                 # UART1-7 (APRS comms via UART4)
   │   ├── spi.c                   # SPI (CMX865A, flash)
   │   ├── i2c.c                   # I2C (EEPROM, sensors)
   │   ├── adc.c                   # Battery voltage, sensors
   │   ├── tim.c                   # Timers, PWM
   │   └── usb.c                   # USB (CH340 emulation)
   ├── radio/
   │   ├── rf.c                    # Frequency synthesis, RF control
   │   ├── audio.c                 # Mic, speaker, PL/DCS
   │   ├── channel.c               # Memory channels, VFO
   │   └── menu.c                  # Menu system, LCD display
   ├── aprs/
   │   └── aprs_bridge.c           # UART bridge to coprocessor
   └── Makefile / Keil project
   ```

2. **Build system** (Makefile or Keil .uvprojx) that compiles the code to a `.bin` file

3. **Binary validation**: A script comparing the output `.bin` against the original `MAIN-MCU-HG-UV98-V1.0.1-190416.bin` with a byte-by-byte diff, achieving **100% match**

### Secondary

4. **Architecture document** describing the firmware structure, module boundaries, and data flow
5. **Protocol document** for the clone interface (the data the radio sends when connected via USB programming cable)
6. **EEPROM layout documentation** mapping all configuration data areas
7. **Diff analysis** between V1.0.1_190416 and V1.0.1_190619-KT showing what the KT2KT modification changed

---

## Constraints & Tips

- The firmware is likely **compiled with Keil MDK-ARM or GCC ARM** (arm-none-eabi-gcc). Try both compilers if needed.
- Use the **GigaDevice GD32 Standard Peripheral Library** as the reference for register addresses and initialization patterns.
- The binary may be **compressed or encrypted** — check for known loaders in the first few KB.
- If full decompilation isn't achievable, aim for **partial reconstruction** with clear markers where assembly/code couldn't be recovered.
- Use the **APRS coprocessor source code** (`repos/hg-uv98-firmware/`) as a reference for the UART protocol between CPUs — this gives us the exact command format the main MCU sends/receives.
- The CHIRP driver (`hg_uv98.py`) shows the clone protocol — this helps identify the USB/serial communication code.

---

## Priority Order

1. **Disassemble V1.0.1_190416.bin** and reconstruct the full C source
2. **Validate by recompilation** — match the output binary exactly
3. **Compare with V1.0.1_190619-KT.bin** — verify the diff matches the documented changes
4. **Document the architecture** based on the reconstructed code

---

*This project aims to achieve what was done for the APRS coprocessor (8051 source was publicly released) but for the main ARM MCU — whose source was never made public. Successful reverse engineering would be a major contribution to the amateur radio community and enable full custom firmware development for this hardware platform.*