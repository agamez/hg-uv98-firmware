# CK5 — Differential Equivalence Report

**HG-UV98 firmware reconstruction vs the original image**
`MAIN-MCU-HG-UV98-V1.0.1-190416.bin` (sha256 `73361601…b66b2`)

Status: **living draft** — assembled through session 51. Every claim
traces to a session entry in `RESEARCH_LOG.md`; corrections there are
retractions, not edits. The report cites the checkpoint evidence and
classifies every known difference.

---

## 1. Scope and method

The reconstruction (this repository, `reconstruction/`) is compared
against the original binary by running BOTH under Renode 1.17.0 on the
same platform description (`reconstruction/sim/gd32f103.repl`) with
identical stimuli, then comparing observables at the register/bus level:

- **CK1** boot & OS — the tick, the task cycling, the context switches.
- **CK2** the protocol link — the injected `$`-frames, the dispatch
  routing, the buffer images (matrix v1: the TX handshake, session 49;
  matrix v2: seven RX stimuli, session 50 pt3).
- **CK3** the RF bus — every AT1846S SPI transaction byte-for-byte
  (292/292 events, session 49).
- **CK4** the display & storage buses — the GPIOB stream captured by the
  `pb_displaybus` model (sessions 51-52).

Address-space convention: Renode address = file offset + 0x08004000;
the research-log labels = file offset + 0x08000000 (so REA/Ghidra
addresses = Renode addresses).

## 2. Checkpoint results

### CK1 — boot & RTOS (CLOSED, sessions ≤ 48)

The reconstruction boots, raises a 1 kHz tick, and cycles its tasks.
Canonical runs: `ck1.resc` (recon), `orig-boot.resc` (original).

### CK2 — protocol link (CLOSED at matrix v2, sessions 49–50)

**Open issue R1 is SOLVED** (session 50): the RX starvation was the BRR
value — the reconstruction wrote 0x2580 (an effective 833 baud on the
model's 8 MHz UART clock) while the original writes **0x341** (9604
baud, 1.04 ms/byte). The ISR, the NVIC and the ring were never at fault.

Matrix v2 (session 50 pt3), the dispatch routing of seven stimuli:

| stimulus | original behavior | reconstruction | parity |
|---|---|---|---|
| `$A00,` + 534 B | copies to buffer1 **and** buffer2 (0x216 gate) | `rxBuf` + `rxBuf2`, 532/534 bytes | ✔ (see 3.2) |
| malformed `$A00` (len ≠ 0x216) | no copy, ready flag cleared | no copy | ✔ |
| `$B00\r\n` | copy to buffer2 (0x20002F74) | `rxBuf2` = full frame | ✔ |
| `$C00\r\n` | copy to buffer2 | `rxBuf2` = full frame | ✔ |
| `$A02,…` (15 B) | copy to buffer3 (0x20004B94) | `rxBuf3` = full frame | ✔ |
| `$A08,…` | copy of the **5-byte prefix only** to buffer2 | `rxBuf2` = 5 bytes + residue | ✔ |
| non-`$` frame | ignored (the `$` gate) | ignored | ✔ |

The literal-pool map (the original): EEC = 0x20003A84, EF0 = 0x20002F74,
EFC = 0x20004B94, flags = 0x20005094, app-state = 0x20004E68/0x2000469C.

### CK3 — RF bus (CLOSED, session 49)

292/292 RF bus events (290 `rf_write` + 2 `rf_read`) byte-identical and
in the same order between the images. Traces:
`analysis/traces/ck3_rf_orig.txt`, `ck3_rf_recon_full.txt`. The rf.c
register primitives are decompile-recovered; the boot default-apply
orchestration order is **[DATA-DERIVED]** from the reference trace.

### CK4 — display & storage buses (drivers CLOSED; the app cadence OPEN, sessions 51-52)

- The LCD = an ST7735-class color TFT on an 8-bit parallel bus:
  PB0-7 = D0-D7, PA0 = WR strobe, PC3 = RS/D-CX (inverted), PC0 = reset,
  PC2 = control. Init table (86 ops, 0x0801BB22) recovered.
- **The init phase = 74/74 ODR events byte-identical** between the
  images (the full controller init sequence).
- The I²C EEPROM driver (SDA=PB9, SCL=PA11, 0xA0/0xA1) implemented; the
  direction-switch CRH value set = **exactly** the original's
  {0x44444434, 0x44444444, 0xBBB44434, 0xBBB44444} with the same phase
  ordering (0x4444xx pre-LCD-init, 0xBBB44xx post-init).
- The boot flow recovered (session 52): the vendor PIC boot (r9 =
  0x08040000 static base, the init-table walker at 0x08042900) → the
  app init 0x0802D120 → the main task 0x0802D178 = the peripheral init
  → the task creators → **the boot scene 0x0801B738** (the settings
  load, the factory-reset check, led_write, the full-screen 0xEFBE
  fill, the splash blit, delay(2000), the VFO loads) → the 30 ms loop.
- The screen layer recovered: the fill 0x0801CD32, the image blit
  0x0801D334, the vertical-geometry text 0x0801CB60, the glyph fetch
  0x08028F8C (9 font classes in the internal flash), the cell renderer
  0x0801BFB8 (the per-class color schemes).
- **Parity (3 s runs): the splash blit = byte-exact (the 0x2FF byte
  count 27,940 in BOTH images); the boot fill = byte-exact (40,960
  EFBE).** The recon = 82,090 ODR events vs the original's 160,753.
- OPEN (application layer): the "RESET WAIT..." glyph cells (~3k
  events — the glyph engine decoded, the exact glyph source mapping
  open), the runtime 30 ms UI redraws (~60k events — the UI state
  machine), and the pre-LCD vendor I²C path (2,501 events, no
  direction switches).

## 3. Classified difference list

### 3.1 Timing boundary (accepted, documented)

The original's SysTick LOAD = 7999 against the 108 MHz HCLK (a stale
8 MHz constant) → its tick runs at ~13.5 kHz, not 1 kHz. The
reconstruction programs a true 1 ms tick. **Classified §7(c)**: a
deliberate deviation; every timing-sensitive comparison uses event
sequences, not wall time.

### 3.2 The two-byte settings transform (mechanism identified, effect open)

Under the pattern payload, the original's parse mutates its working
copy in place: frame[28..29] = settings[8..9] become 0x38 0x39 (from
0x08 0x09) — payload-dependent, not a plain `|0x30` ASCII mask; the
neutral payload shows no change. The archive copy never contains the
transform. The reconstruction stores both copies clean → 532/534 byte
parity on the raw copy. The consumer (a lazy field reader) is in the
Ghidra-missed or un-traced region; open item.

### 3.3 The settings-struct theory (RETRACTED)

The 0x20004E40..0x20004F80 region is payload-independent (80/80 words
identical between the two payloads) — it is the radio's own state, not
a copy of the peer's settings. The peer settings = consumed from the
working buffer by lazy per-field reads (23 read / 11 write EEPROM-
wrapper call sites corroborate the same pattern for the local storage).

### 3.4 Application-layer gaps (open)

- The UI scene renderer / draw cadence (CK4, above).
- The keyscan path (TIM2 CC1 can never match — the stimulus side).
- The tone on-branch (RF-adjacent, untested).
- The `$A09` 4-byte-prefix routing and the stripped tail commands
  ($A10/$A11/$A30/$A31/$D00/$F00/$A50/$A51/$E0x/$F0x, the GB2312
  app-text set) — the original's tail table is in a Ghidra-missed
  region; the recon falls through, documented as unknown.
- The exact boot EEPROM call sites (the missed region 0x0802CE00+
  contains them; the recon wires a labeled inferred equivalent).

## 4. Workaround register (§7 restated)

All of these live ONLY in the harness/platform; the original binaries
are immutable and the reconstruction code does not embed them:

1. **SysTick CVR write skipped at ~10 ms** — the model's counter-clear
   timing; the recon's tick derives from LOAD/CLK alone.
2. **r9 = 0x08040000 set on the original's context-restoration hook**
   (0x08042900) — the original's running image expects the register
   bank the bootloader leaves behind.
3. **GPIOB = `pb_displaybus.py`** (a Python peripheral replacing the
   stock STM32F1 port): its IDR reads 0 for every pin, so the I²C ACK
   sample (PB9 direction-switched) always reads "ACKed". Without it the
   original's boot stalls at the first EEPROM transaction (a floating
   PB9 reads 1 = NACK). Everything else about the model is faithful
   register-level GPIOB plus per-write capture logging.
4. **ADC2_SR tagged 0x2** — the model returns 0 without the tag and the
   original's battery sense loop would spin.
5. **SPI2 = `sp2_fontflash.py`** — the external font flash model.
6. **The peer content** — the 8051 tracker's frames are injected from
   the monitor (`WriteChar` bursts), not generated by a second emulated
   MCU.
7. **Injection pacing** — after a WriteChar burst the RunFor must cover
   ~0.6 s of virtual time for a 534-byte frame (the model paces RX at
   BRR/10 bits; R1's root cause, session 50).
8. **Monitor reads of the STM32_UART model** — only 32-bit reads are
   trustworthy (the 16-bit accesses fall outside the model's
   AllowedTranslations and return garbage; session 50).

## 5. Evidence index

- `analysis/traces/ck3_rf_orig.txt`, `ck3_rf_recon_full.txt` — 292/292.
- `analysis/traces/ck4_display_orig.txt.gz` + the summary — the 187,758
  - event original display stream (5 s).
- `/tmp` capture convention: `pb_stream.log` per run (the harness
  recreates it at peripheral init; persistent copies live under
  `/tmp/opencode/pb_stream_*.log`).
- The session entries 1–51 in `RESEARCH_LOG.md` = the complete chain.

## 6. Verdict

The reconstruction is **behaviorally equivalent at the transport layer
across all four checkpointed buses** (the OS boot, the UART protocol
dispatch, the AT1846S SPI, and the display/EEPROM GPIO buses) with the
documented timing boundary and the §7 harness workarounds. The
remaining differences are application-layer (the UI scene renderer, the
keyscan stimulus path, the tone on-branch) and are listed in 3.4 with
their evidence status.
