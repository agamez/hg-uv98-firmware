# PLAN.md — the remaining simulation rounds (session 49+)

Working state at planning time: CK1 closed; CK2 handshake parity achieved
(byte-identical `$A00` traces, peer model v1); the original's OS runs fully
in Renode (3010->90111 context switches around the settings handshake);
CK3 blocked on an RF-label conflict; CK4 blocked on the GPIOB model gap.

## Round A — RF-path label re-validation (CK3 prerequisite) — ✅ CLOSED (session 49)
The rf.c annotations (rf_send_byte 0x08022712, rf_cs_select 0x08022838,
rf_write 0x0802289A, rf_read 0x0802292E) conflict with dynamic hook evidence:
- 0x0802289A fires with (r0=0x1D, r1=0x6A, r2=0x12, r3=0x14) — small values,
  not (chip, reg, val16).
- 0x0802292E fires with (0, 3, 0x23/'#'/0x52/'R') — character-like r2.
- 0x08022712 receives a bl from inside its own annotated body (a self-loop
  => it is a delay/loop helper, not rf_send_byte).
- The events fire through a tail chain: 0x0801CD2A `bl 0x0801D840` (a
  return-0xFF leaf) whose region branches into the RF block; LR at the hook
  is stale (0x0801CD2F).
- bl_graph.json entries for this region contradict the byte-level disassembly
  (a claimed bl site holds an LDR.W) — the graph is unreliable here.
Deliverable: the true function boundaries + ABIs for the RF block
(0x08022712..0x08022930+) and the 0x0801CD20 helper; corrected rf.c label
comments; a validated hook set that captures (chip, reg, value) triples.

## Round B — CK2 broadening (the full parity matrix)
Stimuli to drive through BOTH images with byte-exact TX/RX capture:
1. `$A01,`+512+CRLF (519 B) — the settings WRITE direction.
2. `$B00\r\n` / `$C00\r\n` — beacon list/detail reads.
3. Key events via GPIOC (the keyscan path — verify the scanner first; the
   TIM2-CC1 question may gate it).
4. `$A02,` telemetry and `$A08,` live beacon (the 8051 push direction).
Deliverable: parity matrix v2 (TX/RX byte streams + timing class).

## Round C — settings consumption (the RAM side)
Compare where the received 512 B land and what changes: the original's
$A00-response handler copies the settings into its RAM region; the
reconstruction's handler fills rxBuf + the ready flag. Deliverable: the
RAM-side diff classified (mechanism vs data), and the reconstruction's
consumer wired if the recovered behavior is unambiguous.

## Round D — CK3 execution (the AT1846S trace) — ✅ CLOSED (session 49)
**RESULT: 292/292 bus events (290 writes + 2 status reads) byte-identical,
same order, original vs reconstruction.** The driver family was decompiled
(band+freq+mode dispatch, 0x30 mode writers with 0x40xx/0x70xx band variants,
GPIO 0x1F variants, CTCSS branches, detect/filter path, tone pair) and wired
as at1846s_boot_default_apply() on the dispatcher boot path. Register
primitives = decompile-recovered; the call order = [DATA-DERIVED] from the
reference trace (labeled in rf.c; restate in CK5). Open extensions: pin-level
(ODR) parity, the settings-driven channel path, the 50-entry tone table's
on-branch.

## Round E — CK4 (the display stream)
The current gd32f103.repl REMOVES gpioPortB (the I2C-ACK workaround), which
blinds the display bus. Plan: replace the removed port with a Python
peripheral at 0x40010C00 implementing CRL/CRH/ODR/BSRR/BRR as observable
state with IDR scripted to 0 (fixes the ACK AND restores visibility). Then
capture the display write stream (the serial driver DATA=PB9/CLK=PA11 plus
the 8080-parallel candidates) from both images and diff structurally.
Deliverable: the framebuffer/write-stream diff, classified.

## Round F — CK5 (the full differential report) — ✅ FIRST DRAFT (session 51)
`docs/CK5_REPORT.md` assembled (the checkpoint results, the classified
difference list, the §7 workaround register, the evidence index).
Maintained continuously under Round L below.

## Round R1 — the USART2 ISR starvation — ✅ CLOSED (session 50)
**Root cause: the reconstruction's BRR (0x2580) vs the original's 0x341.**
The Renode model paces WriteChar delivery by platformFrequency/BRR/10 bits:
0x2580 → 833 baud → 12 ms/byte (the observed starvation); 0x341 → 9604 baud
→ 1.04 ms/byte. The reconstruction now writes the original's own BRR.
Result: the complete 534-byte frame lands in rxBuf AND rxBuf2 (mirroring the
original's two-memcpy structure); 532/534 byte parity vs the original's raw
copy — the 2 diffs = the original's parser converting settings[8..9] to
ASCII in its working buffer (a Round C parser finding). 290 RFW events
intact, no faults. Retraction: the session-49 "CR1 RMW corruption" was a
monitor 16-bit-read artifact (only 32-bit reads of the STM32_UART model are
trustworthy).

## Round B — CK2 broadening (after R1)
$A01 write-direction observation (the ARM emits it after a settings change —
needs the keyscan, which is separately broken: TIM2 CC1 can never match),
$B00/$C00 beacon reads with the 8051-side response formats from FUN_B, the
$A02/$A08 telemetry pushes. Parity matrix v2 over all streams.

## Round C — settings consumption (recon side)
Implement the typed-field parser: the original stores the raw frame twice
(0x20003AD0/0x20003BD0) AND a sparse field struct at 0x20004E40..0x20004F80
(6-byte callsign, 4x0x38 field, scattered config). Decompile the parser
call chain (the $A00 branch → FUN_0801D8C0 targets), map the struct
offsets, wire into protocol.c, then re-run the pattern-payload diff on both
images (needs R1 for the recon side).

## Round E — CK4 display driver — ✅ DRIVERS CLOSED (session 51)
**Recovered:** ST7735-class TFT on the 8-bit parallel bus (PB0-7 data,
PA0 WR strobe, PC3 RS/D-CX inverted, PC0 reset, PC2 control); the 86-op
init table (0x0801BB22 — a Ghidra-missed region, recovered by direct
disassembly); the window setter 0x0801B9C8 (0x2A/0x2B); the I2C EEPROM
bit-bang (SDA=PB9, SCL=PA11, 0xA0/0xA1, the inverted SDA polarity, the
double-START ACK retry); the 12-bit LED link (PB9/PD0/PD1, 0x0802883A);
the boot GPIO phase 0x0802BDF4 (PB13-15 open-drain + PD2).
**Verified:** the LCD init = 74/74 ODR events byte-identical; the CRH
value set = exactly the original's with the same phase ordering.
**OPEN (application layer):** the UI draw cadence (~135 window redraws/
3 s in the original, the fill color 0xBEEF; ui.c = scaffold); the
runtime EEPROM op volume; the exact boot EEPROM call sites (the missed
region 0x0802CE00+).

## Round G — boot flow recovery (the missed region) — session 52
The Ghidra-missed code islands (reached only via indirect paths):
disassemble directly (arm-none-eabi-objdump -M force-thumb) and recover
the boot order. Targets: the caller chain up to main (0x0802D080 ←
0x0802D120 ← ?), the early EEPROM probe call site (the 5 pre-LCD CRH
writes in the stream head), and the functions 0x0802D0D0/0x0802D120
call (0x802d200, 0x802bd88, 0x802b3cc, 0x802aac2, 0x8023798, 0x802d5d4,
0x802a9e0, 0x802a9ba, 0x802d5e4). Replace the inferred labels in
main.c/hw_init with the recovered calls; re-verify the CRH/ODR stream
phases stay byte-identical.

## Round H — UI scene renderer — ✅✅ CLOSED 100% (session 52)
THE DISPLAY STREAM = 100.00% BYTE-IDENTICAL (160,753/160,753 ODR
events over the original's full 3-second capture): the init, the boot
text, the factory reset, the fill, the splash, the status page (the
VFO "000.000" the class 6, the APRS/W/L the class 1, the "00" the
class 3, the cursor, the battery icon the scheme 5, the RX-bar bitmap),
and the periodic re-draws. The remaining CK4 item = the runtime I²C
volume only (the state-machine EEPROM ops the stub routers skip).
The boot display = 100% byte-identical (85,487 ODR: the init, the 13
"RESET WAIT..." cells, the fill, the splash); the status-page build
("CALL:BAT:0.8 V" + the fills/lines/borders) pushes the parity to 80.0%
of the original's full stream (128,535/160,753). The font = the class-3
table @0x080413CC, the column-major (RETRACTED: not computed). REMAINING
(phase 3): the header icon glyphs, the class-0 6x10 small font, the
bottom status boxes, the periodic update policy.
The original redraws ~135 windows/3 s (~22 ms cadence; the fill color
0xBEEF dominates, 0xFFFF white second). Recover the draw chain: who
calls the window setter FUN_0801b9c8 and the fill FUN_0801bd3c (the
BL scan found FUN_0801bd3c called from 0x801c954 = inside FUN_0801c904's
function); identify the boot scene (the splash/blank) and the periodic
redraw driver (the UI task tick). Reconstruct the minimal scene
renderer in ui.c at the CADENCE level ([DATA-DERIVED] labels where the
content is synthesized), then census-compare the two streams.

## Round I — keyscan path (TIM2) — after H
The keyscan: TIM2 CC1 can never match on the model (open since session
~40). Recover the original's keyscan mechanism (TIM2 input capture? the
GPIO EXTI?), document the model limitation as a harness boundary, and
wire the key-event stimulus path so the $A01 write-direction observation
becomes testable.

## Round J — tone path — after I
FUN_0801a91a (the tone wrapper, decompiled session 49/51) — the tone
on-branch: the RF-adjacent tone generation, the CTCSS/DTMF dispatch
(0x080276D4, 0x08027474 already decompiled). Trace the tone enable
condition and reproduce.

## Round K — the protocol tail — ✅ RESOLVED (session 52 pt4)
The twelve tail compare strings live at 0x08044844+ = BEYOND the
distributed image (the vendor flash the distribution truncates). On the
simulator every tail memcmp fails and the dispatch falls through to the
ring reset — exactly the recon's behavior. No code change needed.

## Round K-old — the protocol tail (the missed region) — after J
The ISR's tail table: $A09 (the 4-byte prefix) routing + the stripped
commands ($A10/$A11/$A30/$A31/$D00/$F00/$A50/$A51/$E0x/$F0x + the GB2312
app-text set). Disassemble the dispatch tail from the missed region,
extend protocol.c, and extend the matrix to the tail commands.

## Round M — runtime I²C volume (the last CK4 item) — session 53/54, IN PROGRESS
The display ODR stream = 100.00% CLOSED (160,753/160,753, the FULL
16-bit values — session 53 pt2, re-verified session 54). What differs =
the I²C event counts: the recon 3,392 CRH / 7,432 BSRR / 6,860 BRR vs
the original 4,902 / 7,913 / 14,188 (CRL 0 vs 1). The I²C primitives
are corrected and committed (session 54). The full burst map + the
ordered open items live in followup.md §3: (1) the per-read delta
(62 vs ~101 events/2-byte read), (2) the 104-event write/ACK-poll
bursts vs FUN_080286C0, (3) the save region (4,198 vs 5,881), (4) the
208/359/25-event bursts, (5) the pre-LCD vendor burst (63 vs 2,507 —
possibly a harness boundary).

## Round N — keyscan path (TIM2) — after M
TIM2 CC1 can never match on the model (open since ~session 40).
Recover the original's keyscan mechanism (the input capture? the EXTI?
the polled IDR?), document the model boundary, wire the key-event
stimulus path so the $A01 write-direction observation becomes testable.

## Round O — tone path — after N
FUN_0801a91a (the tone wrapper): the tone on-branch, the CTCSS/DTMF
dispatch (0x080276D4, 0x08027474 already decompiled). Trace the enable
condition, reproduce.

## Round L — CK5 report maintenance — continuous
Update docs/CK5_REPORT.md as each round closes: reclassify the
differences, refresh the parity numbers, and keep the §7 workaround
register in sync with the platform files.

## Standing rules (in force)
- The original binaries = immutable; every derived artifact lives under
  analysis/ or is a labeled generated artifact.
- Workarounds live in the harness/platform only; restate them in every
  equivalence report (CVR=0 at t=10 ms; r9=0x08040000 at the walker entry;
  gpioPortB removed/Python-substituted; ADC2_SR=0x2; SPI2 = sp2_fontflash.py;
  the peer content = neutral defaults, identical for both images).
- Log discipline: append-only sessions; retractions as new entries.
- Re-derive all reconstruction symbols with nm after every rebuild.
