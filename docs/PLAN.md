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

## Round F — CK5 (the full differential report)
Assemble: the CK1 boot parity, the CK2 matrix (all stimuli), the CK3 RF
traces, the CK4 display streams, the classified difference list (mechanism
vs data-content vs timing), and the workaround register (CVR, r9, GPIOB,
ADC2, SPI2, the peer content). Append to the research log as the CK5 report.

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

## Round E — CK4 display driver
Decompile 0x080284F2/0x080285B6 (the serial display functions, PB13-15
control), the controller init sequence, and the PB0-7 streaming path;
implement in display.c against the pb_displaybus.py observable; diff the
two streams structurally (the init phase byte-exact; the streaming phase
census-level).

## Round F — CK5 full differential report
Boot parity (CK1), the matrix (CK2), the RF traces (CK3, 292/292), the
display streams (CK4), the classified difference list, the workaround
register (CVR, r9, GPIOB model, ADC2, SPI2, peer content, injection
pacing), and the open items (R1, the keyscan/TIM2, the tone on-branch).

## Standing rules (in force)
- The original binaries = immutable; every derived artifact lives under
  analysis/ or is a labeled generated artifact.
- Workarounds live in the harness/platform only; restate them in every
  equivalence report (CVR=0 at t=10 ms; r9=0x08040000 at the walker entry;
  gpioPortB removed/Python-substituted; ADC2_SR=0x2; SPI2 = sp2_fontflash.py;
  the peer content = neutral defaults, identical for both images).
- Log discipline: append-only sessions; retractions as new entries.
- Re-derive all reconstruction symbols with nm after every rebuild.
