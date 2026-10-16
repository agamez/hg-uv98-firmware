# followup.md — Getting the HG-UV98 reconstruction RUNNING: simulation & differential validation

You are continuing a completed reverse-engineering project. The analysis phase is
**finished** (34+ sessions, REA/Ghidra-validated, everything extractable extracted).
Your mission is different: **make it run** — emulate the firmware, and validate the
C reconstruction against the original binary by differential comparison. Read this
file fully before writing any code.

---

## 0. Inherited rules (from prompt.md — still in force)

- Evidence-driven. Every claim recorded in `docs/RESEARCH_LOG.md` (append a new
  session entry per work session; never rewrite old entries — corrections are
  appended as retractions).
- Git discipline: coherent milestone commits; **never stage the user's `prompt.md`
  or `prompt.md~`**.
- Provenance annotations on all recovered code: `[RECOVERED]`, `[INFERRED]`,
  `[SCAFFOLD]`, `[REPLACEMENT]`, `[VENDOR]`.
- The original firmware binaries are **immutable reference inputs**. Any file you
  produce that derives from them lives under `analysis/` (gitignored) or is a
  clearly-labeled generated artifact.
- **Addressing convention (critical):** all "0x080xxxxx" addresses in the log and
  docs = `file_offset + 0x08000000` (labels). Physical/linked flash = label + 0x4000.
  Pool-pointer *values* inside the image are linked addresses (file + 0x08004000).
  REA/Ghidra sessions open the raw binary with base 0x08004000 so REA addr = label + 0x4000.

## 1. Where things stand

- `docs/RESEARCH_LOG.md` — 37 sessions: full architecture of the main MCU
  (GD32F103RET6, Cortex-M3) recovered from two distributed binaries.
- `docs/MENU_TREE.md` (116 dispatcher edges), `docs/SCREENS.md` (86 pump sites,
  20 list IDs), `analysis/pump_args.json` (screen linkage chain).
- `reconstruction/` — buildable C (`make TOOLCHAIN_PATH=<arm-gcc-dir>`,
  `make test-host` = green protocol conformance tests), tagged `level3-complete`:
  - `src/rf.c/h` — AT1846S ×2 bit-bang transport + full register map
  - `src/ui.c/h` — screen-state machine, input policy, battery/S-meter chains
  - `src/protocol.c` + `tests/test_protocol.c` — the inter-MCU protocol
  - `src/font.c` — external SPI-flash font fetchers (addressing math recovered)
  - `src/display.c`, `src/startup.c`, `src/rtos.c` + vendored uC/OS-II + GD32 SPL
- **Known-absent content (proven, both binaries):** the `.data` initialization image
  (key tables, menu string tables, CSS tone table, screen attribute bytes) and the
  stripped dispatcher tail table. Their RAM addresses and linkage structure are fully
  mapped; their *contents* are unrecoverable from the artifacts.
- The 8051 APRS coprocessor has full source (`UV98_868_220430_D4/`); it is NOT a
  simulation target — model it as traffic (see §5).

## 2. Mission

Build an emulator/simulator harness that runs **both** the original binary and the
reconstructed C on the same modeled hardware, drive them with identical stimuli, and
diff the observable outputs. Success = mechanism-level equivalence demonstrated, with
data-content differences (from the absent `.data`) documented rather than hidden.

## 3. Emulator choice — recommendation

- **Preferred: Renode** (Antmicro). It models STM32F103 (a near-clone of GD32F103:
  same Cortex-M3, same F1 peripheral layout) and allows custom scripted peripherals.
  Define a `gd32f103ret6` platform: CPU + memory map below + peripheral set in §4.
- Alternative: QEMU `mps2-an385` (Cortex-M3) with hand-written peripheral devices at
  the GD32 addresses, or Unicorn Engine + a custom device loop (more manual work).
- The old log note "QEMU can't model GD32F103 peripherals" meant *stock* QEMU —
  with custom devices it is viable; Renode just gets you further faster.

### Memory map to model (from the log)

```
flash  0x08004000 .. 0x08080000 (512K − 16K; bootloader below is NOT distributed)
sram   0x20000000 .. 0x20010000 (64K)
APB1   0x40000000: TIM2 0x40000000, SPI2 0x40003800, USART2 0x40004400 (APRS/BT link)
APB2   0x40010000: GPIOA 0x40010800, GPIOB 0x40010C00, GPIOC 0x40011000,
       GPIOD 0x40011400, USART1 0x40013800 (GPS), ADC1 0x40012400, SPI1 0x40013000
AHB    0x40018000 (SDIO etc. — model as RAZ/WI unless the code touches it)
NVIC/SysTick/PendSV/SCB at the standard CM3 addresses
```

Key wiring recovered in the log (model these behaviors, not just registers):

| Peripheral | Recovered usage |
|---|---|
| GPIOA | RF bit-bang: CLK=PA15, DATA=PA9, CS0=PA8, CS1=PA12 (3-wire, 24-bit frames) |
| GPIOB | ST7735 8080-parallel data bus D0-7=PB8-15, WR=PB0 |
| GPIOC | D/C=PC3, CS=PC1, RESET=PC10; keypad/PTT pins (PC7/PC8/PC11...) |
| GPIOD | PD4 = font-flash CS (SPI2) |
| USART1 | GPS (NMEA in) |
| USART2 | the 8051 link: `$A`/`$B`/`$C` frames (6-byte cmds, 0x216 settings frames) |
| TIM2 | key-scan tick (IRQ 28) |
| ADC1 ch14 | battery divider (thresholds 3253/1751 counts) |
| SysTick | uC/OS-II tick |

## 4. Custom devices to write (in order of boot dependency)

1. **Clock/RCC** — enough for `SystemInit` (constants recovered; PLL to 108 MHz or
   just accept the writes and run the model at a fixed rate).
2. **GPIO** — four ports with output-pin visibility: every PA/PB/PC/PD write must be
   observable as events (the RF and display buses are bit-banged — the comparison
   harness *is* the pin trace).
3. **External SPI font flash** (SPI2, CS=PD4, READ opcode 0x03, 24-bit addr).
   The real content is unavailable — **synthesize** it: HZK16-style CJK glyphs
   (the addressing math in `src/font.c` must be honored: `addr24 = ((hi−0xB0)×94 + lo + 685) << 5`)
   plus ASCII 8×8/8×16 sets, and the asset-table classes 1–9 as stub blocks.
   Use the same synthesized image for both runs.

**UPDATE (sessions 38-42): RESOLVED — nothing needs synthesis here anymore.**
The real table content EXISTS in the firmware as **armlink compressed-RW-data**
(vector → boot stub → init walker → `__scatterload_zeroinit` + an LZ/RLE
decompressor, REA-decompiled in session 38; session 41 proved the image
complete — the packed stream ends exactly at the file end, entry 1 = .bss
zero-init of 0x20001734-0x2000517C). Sessions 39/42 decompressed both builds'
streams; the recovered tables are now **embedded in the reconstruction build**
(`src/tables.S`: `.tables` at RAM 0x20000000, `.uistrings` at flash
0x08040000 = the original's string region; `Reset_Handler` copies the tables
like the original's scatter-load). Load the reconstruction ELF as-is — tables
included. For the ORIGINAL image's run, let it execute its own decompression
(boot chain is self-contained) or preload
`reconstruction/data/ram_tables_recovered.bin`.

**Session 43 correction:** the table at RAM 0x200016D0 once read as a "CSS
tone table" is actually a **frequency sub-band table** (u32[96], 24
initialized; RF frequencies in 10 Hz units: 136.5 … 511.875 MHz) feeding the
reg 0x05 audio-filter config in 0x08023156 — see the research log.

4. **AT1846S ×2** — model the register file from the map in `src/rf.c` (0x1F GPIO,
   0x29/0x2A frequency, 0x30 mode, 0x35 tone, 0x1B RSSI[15:8], 0x1C CSS flag,
   0x7F bank select). Reads return plausible values (configurable RSSI for tests).
   Decode the bit-bang frames on PA15/PA9/PA8/PA12 and log every (chip, reg, value).
5. **ST7735** — decode the 8080-parallel + D/C/CS/RESET pins into a framebuffer
   (cmd 0x2C family per `src/display.c`); export PNG/dumps per frame. This is the
   primary visual comparison channel.
6. **USART2 peer** — a scripted 8051 stand-in: plays the conformance vectors from
   `tests/test_protocol.c` (settings upload `$A01,` 534-byte frames, beacon reads,
   telemetry) and logs every response byte-accurately.
7. **ADC** — ch14 returns a configurable count (test: above/below both thresholds).
8. **NVIC basics** — TIM2 period, SysTick tick, and the 60-trampoline IRQ dispatch
   just works (it is the original's own code).

## 5. The original-binary runner — the one injected change

The distributed images are **neutered**: the reset handler at label 0x08044690
(file 0x40690) is 2× `NOP.W`. To execute the original you must patch the **minimum**:
point the reset vector (or restore a stub) at a reconstructed startup that does what
the original's did — set VTOR=(0x4000&0x1FFFFF80)|0x08000000, init .data/.bss, call
SystemInit, then the app entry at label 0x08029120. Everything else must be the
original's own bytes. Document the exact patch bytes in the log; keep both a patched
and unpatched copy under `analysis/` (never in git).

## 6. Differential comparison harness

Drive both images (original-patched and reconstructed) in identical machines with
identical stimulus scripts; capture and diff:

1. **Protocol trace (USART2, bidirectional)** — byte-exact comparison against the
   conformance vectors. This is the strongest check: the protocol layer is fully
   recovered and already passes the host tests.
2. **AT1846S bus trace** — the ordered (chip, reg, value) write sequence for a fixed
   channel-setup script (power-on init, frequency set to known values, mode
   transitions 0x4006/0x4046/0x40A6). Byte-exact expected for the recovered paths.
3. **Display stream** — the ST7735 write sequence + framebuffer dumps. Expect
   *structural* equality (same layout, same primitives) but NOT byte-equal content:
   menu strings/table-driven content differ because the original `.data` is absent
   (see §7). Diff pixel maps and classify differences as mechanism vs data.
4. **Key-event behavior** — scripted key sequences (GPIOC pin transitions per the
   scanner map) → compare screen-state changes (visible via display stream) and
   emitted events.
5. **Timing** — task-loop heartbeat counts and tick-level ordering (loose equality).

Harness shape: stimulus scripts (JSON/DSL) → runner → per-run trace files
(`analysis/traces/`) → differ (byte-exact for 1-2; classified diffs for 3) → report.

## 7. The equivalence boundary — state it in every report

**REVISED (sessions 42-43):** the `.data` image was recovered and embedded in
the reconstruction — table-driven content NOW matches byte-for-byte: menu
strings, key tables, screen attributes, the frequency sub-band table, the
battery/S-meter calibration ladders. What still CANNOT match byte-exactly:
(a) external SPI font-flash **glyph bitmaps** (the glyphs live on the SPI
flash chip, not in the MCU image — the synthesized font remains, and every
glyph-derived display byte in the diff is a known data-content difference),
(b) ADC/RSSI analog readings (make them configurable in the device models),
(c) timing jitter. Do not "fix" the reconstruction to smooth over those —
fabricating agreement would corrupt the differential signal.

## 8. Suggested checkpoints

1. **CK1** — Renode/QEMU machine boots the reconstruction to the UI task loop
   (heartbeat counter at 0x2000469C+0x166 increments; routers run).
2. **CK2** — protocol conformance parity: both images pass the same USART2 vectors.
3. **CK3** — RF parity: identical AT1846S write sequences for the fixed setup script.
4. **CK4** — display: both images produce framebuffer dumps with identical
   code-driven elements (banners, fixed labels, blinker behavior).
5. **CK5** — full differential report: parity matrix + classified difference report
   (mechanism vs data-content), appended to the research log.
6. **CK6 (stretch)** — the reconstructed `.data` tables tuned so the menu screens
   render structurally identical to the original's screens for the first two levels.

## 9. File map

```
docs/RESEARCH_LOG.md      # the evidence base — read sessions 18-37 first for UI/RF detail
docs/MENU_TREE.md         # 116 dispatcher edges
docs/SCREENS.md           # pump inventory
reconstruction/           # the C tree (build: make TOOLCHAIN_PATH=...)
reconstruction/src/*.c    # recovered mechanisms (start: ui.c, rf.c, protocol.c)
firmware/MAIN-MCU-*.bin   # original 190416 (0x42B97 bytes, neutered reset)
firmware/V1.0.1_190619-KT.bin  # KT variant (0x7C000; APRS-menu redesign; tail to 0x45B57)
firmware/UV98_868_220430_D4.hex # 8051 coprocessor image (45 KB)
analysis/                 # gitignored: dumps, extracted JSON, traces
tools/                    # dis_range.py, vendor_fingerprint.py
```

## 10. Open items that simulation may resolve (secondary)

- The dispatcher's stripped tail-table commands — if the original image, when run,
  reaches the tail table, its behavior can be *observed* even though its bytes are
  absent (the tail table lives in flash the artifacts don't contain — likely
  unreachable; verify where the dispatch actually lands).
- Cross-check the recovered `.data`-structure guesses (linked screen chain, redraw
  block fields) against live behavior.
- S-meter bar thresholds and the `'M'/'N'` blinker source — observable with a
  simulated RF level sweep.

Start by standing up CK1. Keep the log current, and good hunting.
