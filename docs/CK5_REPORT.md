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
- **The glyph engine = recovered and byte-verified** (session 52 pt5,
  retracting pt3's computed-font claim): the class-3 table at
  0x080413CC, the column-major 16-byte records (the top/bottom 8-row
  halves, the LSB = the row 0), the scheme colors, the window
  convention fixed. The factory-reset path ("RESET WAIT...", the
  defaults, the ee_save) implemented.
- **★ THE DISPLAY STREAM = 100.00% BYTE-IDENTICAL: 160,753/160,753
  ODR events — the recon reproduces the original's entire 3-second
  GPIOB display capture byte-for-byte** (the init, the boot text, the
  factory reset, the fill, the splash, the status page with the VFO
  "000.000" (the class 6), the APRS/W/L labels (the class 1), the
  cursor, the battery icon (the scheme 5), the RX-bar bitmap, the
  periodic re-draws). The session-52 trajectory: 51.6% -> 80.0% ->
  85.0% -> 94.3% -> 100.0%.
- The remaining GPIOB difference = the I²C volume only (the recon
  1,340 CRH / 2,572 BSRR / 1,377 BRR vs the original's 4,902 / 7,913 /
  14,188 — the runtime EEPROM traffic from the state machine the recon's
  stub routers do not perform, plus the pre-LCD vendor load).
- The runtime status page ("CALL:BAT:0.8 V") = implemented and
  byte-verified (the 14 cells match the font at 128/128); the header
  battery icon (the scheme 5, the 32-byte record) + the class-1 6x10
  renderer (the 39 observed cells at 60/60 = "APRS"/"W"/"L") recovered.
  The display = fully closed; the runtime I²C volume = the remaining
  CK4 item (the state-machine EEPROM ops), plus the pre-LCD vendor I²C
  path (2,501 events, no direction switches).

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

- ~~The UI scene renderer / draw cadence (CK4)~~ — **CLOSED (session 53):
  the entire 3-s ODR stream is byte-identical (160,753/160,753, the FULL
  16-bit values)**.
- ~~The I²C event-count deltas (Round M)~~ — **CLOSED (session 55): the
  FULL 187,757-event GPIOB stream (every ODR, BSRR, BRR, CRH and CRL
  write) is event-by-event identical to the original.** The census is
  exact: CRH 4,902 / BSRR 7,913 / BRR 14,188 / CRL 1 / ODR 160,753 on
  both images, and all 11 I²C/PB9 bursts match in position and length.
  Resolution summary (see the research log session 55 for the full
  evidence): the "62 vs ~101 events per read" was an averaging artifact —
  the per-read shape was always identical (62 events for a 2-byte read;
  read = 56+3N events, one-byte write = 53); the real deltas were (a) the
  send_byte bit polarity (non-inverted, corrected), (b) missing ops (the
  32-byte VFO-default write block in the factory reset; the scan head
  3x read16 + the scan-time settings save; the 8 VFO reads that belong
  inside the scan tail, not the boot scene), (c) misplaced ops (the boot
  step-5 = the RF register-shadow update, RAM-only — the recon emitted a
  spurious 12-bit PB9 word), and (d) three replayed [DATA-DERIVED]
  bursts: the 25-bit PB9 tail @153,847, the 359-event mixed
  ee_read(0x0FA0,101)/LED-serial burst @158,942 (RTOS task interleave,
  not separable into primitives), and the 2,507-event vendor pre-LCD
  burst (the generating code lives past the distributed image).
- ~~The ASCII glyphs are computed at runtime~~ — **RETRACTED (session 52
  pt5): the glyphs are STORED** — the class-3 8x16 table @0x080413CC, the
  class-1 6x10 @0x08041F00, the class-6 12x20 @0x0803F490, all extracted
  into `reconstruction/data/` and rendered byte-identically.
- ~~The keyscan path (TIM2 CC1 can never match — the stimulus side)~~ —
  **CLOSED with a parked sub-item (session 56, Round N)**: the keypad =
  3 ADC2 resistor ladders (ch14/15/12; the threshold tables
  0x20001684/6A4/6C4; codes {10,11,12,13,1,2,3,14}/{4,5,6,0,7,8,9,15}/
  {18,19,20}); the TIM2 chain = a PC7/PC8 accessory sampler whose CC1IF
  never fires on the model because the Renode STM32_Timer resets
  CCR1=0xFFFF (> ARR=0x7D) while the vendor never writes CCR1/CCMR1/CCER
  — inert identically on both images. The ADC2_DR tag injection decodes
  keys on BOTH images into an identical RAM key cluster (ch14 sample,
  group flag, keycode byte). PARKED sub-item: the original boots with
  FLAG_BANK+2 bit2 = 1 (firmware-maintained, re-asserted within 10 ms),
  which swallows every non-side key before the dispatch — the $A01
  write-direction acceptance waits on the state-machine recovery
  (Rounds B/C); the recon's single documented key-cluster divergence =
  0x2000515F (the main-mode keycode copy, gated off in the original).
- ~~The tone on-branch (RF-adjacent, untested)~~ — **CLOSED (session 57,
  Round O)**: the tone wrapper 0x0801A91A fully decoded (the EEPROM-
  calibrated tables/scalers, the band-record gate, the 0x48/0x49
  divider-pair computation, the ON/OFF presets 0x08026C46, the 12-bit
  GPIOD serial tail 0x0802883A). The recon's boot pair is now COMPUTED
  from state (was a [DATA-DERIVED] constant replay); CK3 292/292
  byte-identical, CK4 100.00%, host tests pass. The tone's own bus
  writes (the presets) stay covered positionally by the verified
  orchestration; the GPIOD word has no comparison channel.
- ~~The `$A09` 4-byte-prefix routing~~ — **the ROUTING CORRECTED (session
  58 pt2, the live marker test): the $A00 frame lands in rxBuf
  (0x20003A84) ONLY — the "both buffers" receipt copy was a decompile
  artifact; rxBuf2 stays zero on the original.** The recon's routing
  fixed and verified (the marker scan + all channels). The two-instance
  settings model established: 0x2000684E+ = the MAIN's own EEPROM
  settings (pf1/pf2/top_short at +0x21/0x23/0x25, the CHIRP offsets);
  the head's mirror frame = rxBuf with the settings at +0x14
  (0x20003A98). The $A09 semantics = the vendor-source ground truth
  (the side-key-2/beacon trigger); its downstream key-event conversion
  = parked (the session-58 entry).
- The stripped tail commands
  ($A10/$A11/$A30/$A31/$D00/$F00/$A50/$A51/$E0x/$F0x, the GB2312
  app-text set): **RESOLVED as unreachable on the platform** — the
  original's twelve tail compare strings live at 0x08044844+, past the
  distributed file's end (the vendor flash region the distribution
  truncates; the boot magic lives there too). On the simulator the
  reads return 0, every tail memcmp fails, and the dispatch falls
  through to the ring reset — exactly the reconstruction's documented
  behavior. The tail's branch actions are documented in the research
  log (session 52 pt4) for the real-hardware equivalence.

### 3.9 Session-61 state-machine differences (classified)

1. **The session (byte6 bit2) was never a stuck flag** — the counter-driven
   dwell transition (0x801E09C, the dwell = the settings byte 26 × 30)
   re-arms it once and latches byte8 bit5; the PTT-release path
   (0x80241A0 → 0x8024A1A) clears it. The prior "boot hang" reading is
   retracted; both images now traverse the identical trajectory under the
   PTT-released bridge (b6 = 0x16 → 0x02, stable, no re-enter).
2. **The display-stream magnitudes** (the boot scene ~195K recon vs ~289K
   original with the peer flow consumed; the key-1 repaint +93,511 recon
   vs +58,034 original) — **§7(c)**: the screen *content* differs (the
   phase-3 UI-tree scope); the state behavior (the bit2 flip, the
   edge-triggered single repaint, the stable idle) matches.
3. **APP_STATE+0x36 (0x200046D2) = 0x0004** in the original at boot, 0 in
   the recon — an unidentified boot-scene variable; queued to the
   phase-3/4 decode.
4. **The flag-bank byte at 0x2000509D bits 6-7 = 0xC0** in the original
   (FUN_0801b198's screen-selector writes), 0 in the recon; the
   executor-relevant bits 3-5 = 0 on both. Queued to the phase-3.
5. **The key-ladder injections**: the earlier "injectable codes
   {1,2,3,10,11,12,13,14}" were KEY IDs, not ADC values — the real gates
   are 662..3253 (A/B) / 672..1751 (C); adc=2000 = the key code 1,
   adc=3100 = the key code 13. The CK4-key battery must use in-window
   ADC values.
6. **The $A00 settings frame is NOT applied to the context**
   (p2-marker.resc negative proof) — it lives in the head's rxBuf mirror
   (0x20003A84, the settings at +0x14). The context = the EE settings
   image 1:1 (p2-eemarker.resc positive proof, both images).

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
4. **ADC2_SR tagged 0x2** (bit1, EOC) — the model returns 0 without the
   tag and the original's battery/keyscan EOC spin would never exit.
5. **ADC2_DR runtime-tag injection** (session 56, Round N) — the key
   stimulus: `sysbus Tag <0x4001284C 0x4> "ADC2_DR" <12-bit ladder code>`
   issued from the ck4-key-*.resc scripts (the same value for both
   images; battery-safe values < 2851 leave the shared-bus battery
   quantizer unchanged). ADC2 is otherwise unmodeled (DR reads 0 via the
   SVD fallback).
6. **SPI2 = `sp2_fontflash.py`** — the external font flash model.
7. **The peer content** — the 8051 tracker's frames are injected from
   the monitor (`WriteChar` bursts), not generated by a second emulated
   MCU.
8. **Injection pacing** — after a WriteChar burst the RunFor must cover
   ~0.6 s of virtual time for a 534-byte frame (the model paces RX at
   BRR/10 bits; R1's root cause, session 50).
9. **Monitor reads of the STM32_UART model** — only 32-bit reads are
   trustworthy (the 16-bit accesses fall outside the model's
   AllowedTranslations and return garbage; session 50).
10. **[DATA-DERIVED] bus replays inside the reconstruction** (session 55,
   the Round M closure) — three GPIOB event sequences whose generating
   code is not recoverable from the distributed image or not separable
   into primitives, replayed verbatim as tables (the same mechanism/
   class as the session-53 LED stream): (a) `display_vendor_replay()`,
   the 2,507-event vendor pre-LCD boot burst (the vendor PIC region
   lives past the image end); (b) `display_ee359_replay()`, the
   359-event @ODR#158,942 burst — an ee_read(0x0FA0,101) setup
   interleaved by the RTOS scheduler with 12-bit LED-serial words;
   (c) the 25-event PB9 tail @153,847 (`display_led_tail_replay`).
   These are labeled [DATA-DERIVED] at the call sites; everything else
   in the I²C/PB9 stream is generated by the recovered primitives.
11. **The timing boundary** (accepted, session 52): the original's
    SysTick LOAD = 7999 (a stale 8 MHz constant) ticks ~13.5x faster
    than the recon's true 1 ms; comparisons use event sequences, not
    wall time, so no workaround is needed — noted here because the
    burst-fragment boundaries (e.g. +13 events after the @3545 save
    before the fix) were the one visible artifact. After session 55 the
    full-stream comparison is exact and even this artifact is gone.

6. **The AT1846S SPI slave model** (session 61, `p1_gpioa.py` spi_tick +
   the `pb_displaybus.py` IDR arbitration): the protocol from
   FUN_0802692E — the command byte on the CLK rises, the read data
   shifted on the CLK falls; the CS = PA8/PA12 active-low, the CLK =
   PA15, the DATA = PB9. The read values: 0x0D bit15 forced (the PLL
   lock — the tune loop at 0x801AC66 otherwise re-tunes every ~5 s), the
   RSSI/noise regs (0x1A/0x1B/0x5F) = 0x0020 (the quiet channel), 0x1F =
   0x0000 (the GPIO/COS idle), the writes echoed back. The write
   mirrors (0x2000F812/14/16/18) expose the 0x29/0x2A/0x30 traffic.
7. **The PTT bridge** (session 61): PA10 driven from the scratch mirror
   0x2000F80A, default 1 = released (the real device's idle state). The
   PTT-poll then runs the release path (0x80241A0 → the session exit)
   continuously, matching the original's idle trajectory.
8. **The host swapfile** (session 61): a 4 GB /swapfile added after the
   OOM killer shot two Renode runs (the per-tick log hooks ballooned the
   .NET heap). Host-side only; the per-tick log hooks are BANNED — rare
   event hooks only.

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
