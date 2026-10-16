# HG-UV98 main-MCU — Level-3 reconstruction

Original C reconstruction of the GD32F103RET6 main-MCU firmware. This is
**not** a decompiler dump: every function is written against the evidence in
`../docs/RESEARCH_LOG.md`, with original addresses and confidence levels
annotated in each file's header.

## Status

**Builds** (`make` → `build/hguv98.bin`, ~4.1 KB) and is **consistent with
static evidence**. Per the project rules, without emulation or hardware this
is a Level-3 result of the "builds + consistent with static evidence" kind —
no runtime behavior is claimed.

**Recovered architecture (sessions 18-32):** complete screen-state machine
(~145 screen IDs, `ui_state_set` 0x08010530 with band-ring snapshot), the
menu dispatcher (binary-search tree; 116-edge mapping in
`docs/MENU_TREE.md`), the full input chain (scanner / PTT synth /
software-polled keys / programmable IDs → gated `key_event`), battery
gauge (ADC ch14, thresholds 3253/1751, 4 states) and S-meter chains
(AT1846S 0x1B[15:8] RSSI → peak-hold → BCD timestamp history rings →
numeric dB display), beacon windows ('$B%02d'/'$C%02d'), the menu redraw
block layout, and the list scroller. All REA/Ghidra-validated (raw binary
at linked base 0x08004000) and traceable in `docs/RESEARCH_LOG.md`
(sessions 18-32). The protocol layer additionally passes the executed host
conformance tests (`make test-host`).

## Vendored third-party code

| Tree | Source | License |
|---|---|---|
| `ucos2/` | weston-embedded/uC-OS2 V2.93.01 + uC-CPU + uC-LIB headers (the kernel the original runs) | Apache-2.0 (`ucos2/*-LICENSE`) |
| `vendor/gd32f10x/` | GigaDevice GD32F10x Firmware Library **V2.2.1** (2018-era vintage matching the 190416 build; SPL + CMSIS + `system_gd32f10x.c`) | GigaDevice **SLA-GD0001** (`GIGADEVICE-SLA.txt`) — source redistribution permitted with notices, use restricted to GD devices; this reconstruction targets exactly the GD32F103RET6, satisfying the terms |

`SystemInit()` (vendor, 108 MHz HXTAL) is called from `Reset_Handler`; its
constants were located in the original image (see startup.c and the research
log session 10), making the vendor clock init RECOVERED-vendor code rather
than a guess.

## Evidence key (used in per-file annotations)

| Tag | Meaning |
|---|---|
| RECOVERED | Disassembly/protocol evidence is direct (addresses cited) |
| INFERRED  | Strong indirect evidence; details documented in the log |
| SCAFFOLD  | Placeholder pending missing evidence (stripped regions) |
| REPLACEMENT | Clean-room re-implementation of a stripped component with observed semantics |
| VENDOR | Vendored GigaDevice/uC-OS2 source confirmed (or strongly indicated) in the original image |

## Modules

| File | Status | Provenance |
|---|---|---|
| `src/hwregs.h` | original work | register sugar over the vendored GigaDevice header; addresses cross-checked against image literals (ST-style USART numbering per the image) |
| `src/startup.c` | SCAFFOLD + VENDOR clock init | original startup is stripped; vector structure + original handler addresses documented; `SystemInit()` vendored (constants found in image) |
| `src/mem.c` | RECOVERED semantics | counterparts 0x080198E0/0x080198C0/0x0801F610/0x080199A0 |
| `src/rtos.c` | thin wrappers over vendored uC/OS-II | OSMbox*/OSTaskCreateExt/OSTimeDly* mapped to the recovered call sites; task table recovered (7 tasks, priorities, stacks) |
| `src/os_hooks.c` | scaffold hooks | empty uC/OS-II app hooks + CPU_TS_TmrRd stub (never exercised) |
| `src/uart.c` | RECOVERED | USART2 init 0x0801F810, RX ISR 0x0801F920, TX 0x0801FEAA; USART1 init 0x0802932C, RX ISR 0x080292F8 |
| `src/protocol.c` | RECOVERED set / reordered dispatcher | command set cross-verified (ARM image + 8051 UARTX.C + vendor xlsx); original table lives in the stripped tail |
| `src/display.c` | RECOVERED primitives / SCAFFOLD transport | cmd 0x2C ILI9341-family write, GB2312 16 px cells, ASCII 8 px; panel bus wiring open |
| `src/rf.c` | RECOVERED transport + register semantics | AT1846S ×2 on a bit-banged 3-wire bus (PA15/PA9, CS0 PA8/CS1 PA12), 24-bit frames; register map cross-verified vs OpenRTX + AT1846S Programming Guide 1.4; frequency/CSS setters recovered |
| `src/main.c` | RECOVERED structure / SCAFFOLD bodies | main 0x08029120, hw_init 0x080290D0→0x08029080, task table |

## Build

Requires `arm-none-eabi-gcc` (14.x tested). Override `TOOLCHAIN_PATH` if the
toolchain lives in a non-standard prefix, e.g. a `dpkg -x` extracted tree:

    make TOOLCHAIN_PATH=/tmp/opencode/arm-tc/usr/bin/

Targets: `all` (bin + size), `disasm` (objdump for comparison against the
original image), `clean`.

## Explicitly not reconstructed

- Startup/fault-handler code and the 60 IRQ trampolines (stripped in all
  distributed images; only the structure is known).
- Font/bitmap assets (image data region; format analysis pending, log O5).
- RF chip serial transport (PA15 bit-bang pattern recovered; register
  semantics of the target chip not yet identified — log session 6).
- Clone/program-mode handler (stripped; protocol known from CHIRP instead).
- Panel bus wiring, key matrix, screen-by-screen UI semantics.

## Validation performed

- `make` completes with no errors; warnings reviewed (none outstanding).
- Vector table verified from the built `.bin`: MSP 0x20006800, 76 entries,
  all valid Thumb pointers, USART1/USART2 wired at IRQ slots 37/38.
- Disassembly spot-check: PendSV context-switch sequence and the USART2
  TC-poll send loop match the recovered original patterns.
- No runtime/emulation validation (no QEMU model of this board; no
  hardware in scope). Level-4 byte-equivalence is explicitly out of scope.
