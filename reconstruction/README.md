# HG-UV98 main-MCU — Level-3 reconstruction

Original C reconstruction of the GD32F103RET6 main-MCU firmware. This is
**not** a decompiler dump: every function is written against the evidence in
`../docs/RESEARCH_LOG.md`, with original addresses and confidence levels
annotated in each file's header.

## Status

**Builds** (`make` → `build/hguv98.bin`, ~2.4 KB) and is **consistent with
static evidence**. Per the project rules, without emulation or hardware this
is a Level-3 result of the "builds + consistent with static evidence" kind —
no runtime behavior is claimed.

## Evidence key (used in per-file annotations)

| Tag | Meaning |
|---|---|
| RECOVERED | Disassembly/protocol evidence is direct (addresses cited) |
| INFERRED  | Strong indirect evidence; details documented in the log |
| SCAFFOLD  | Placeholder pending missing evidence (stripped regions) |
| REPLACEMENT | Clean-room re-implementation of a stripped component with observed semantics (RTOS kernel) |

## Modules

| File | Status | Provenance |
|---|---|---|
| `src/gd32f10x.h` | original work | public register map, addresses cross-checked against image literals |
| `src/startup.c` | SCAFFOLD | original startup is stripped; vector structure + original handler addresses documented in comments |
| `src/mem.c` | RECOVERED semantics | counterparts 0x080198E0/0x080198C0/0x0801F610/0x080199A0 |
| `src/rtos.c` | REPLACEMENT | uC/OS-II primitives re-implemented; task table recovered (7 tasks, priorities, stacks) |
| `src/uart.c` | RECOVERED | USART2 init 0x0801F810, RX ISR 0x0801F920, TX 0x0801FEAA; USART1 init 0x0802932C, RX ISR 0x080292F8 |
| `src/protocol.c` | RECOVERED set / reordered dispatcher | command set cross-verified (ARM image + 8051 UARTX.C + vendor xlsx); original table lives in the stripped tail |
| `src/display.c` | RECOVERED primitives / SCAFFOLD transport | cmd 0x2C ILI9341-family write, GB2312 16 px cells, ASCII 8 px; panel bus wiring open |
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
