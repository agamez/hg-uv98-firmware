# GOAL.md — HG-UV98 firmware recovery: the one-shot completion directive

You are an autonomous agent. This file is your mission specification. Work it
top to bottom, loop until §1's DONE gate passes with fresh evidence, and do not
stop, ask, or wait for a human at any decision point. Every decision rule you
need is in this file. The repo contains the full evidence base
(`docs/RESEARCH_LOG.md`, 69+ sessions) and a prior incremental handoff
(`followup.md`) — this file supersedes followup.md's KICKOFF as the driver, but
followup.md §2/§6/§7 remain valid reference (the harness recipes and the Renode
survival guide).

---

## 0. The mission and the REA-first doctrine

The vendored firmware of the HG-UV98 handheld (main MCU, distributed image
`firmware/MAIN-MCU-HG-UV98-V1.0.1-190416.bin`, loaded at 0x08004000) has been
reverse-engineered into a C reconstruction (`reconstruction/src/`, builds to
`reconstruction/build/hguv98.elf`). Both binaries run in Renode. The
reconstruction already passes several byte-exact differential channels.

**Your mission: finish the recovery — drive the reconstruction to full
behavioral equivalence with the original on this platform, close every
remaining gap, and prove completeness.** One shot, no human in the loop.

**THE METHOD PRIORITY (in force since session 69, superseding the earlier
"stream-replay-over-REA" preference):**

1. **REA is king.** The REA toolchain (Ghidra-class decompilation over the
   original image) is the PRIMARY instrument for every behavioral question:
   control flow, call chains, state gates, timing loops, register semantics.
   REA produces **C pseudocode**; the reconstruction's C is the curated,
   compiled form of REA's output. You are NOT to hand-decode assembly,
   literal pools, or event streams to answer a question REA can answer.
2. **Replay is the judge, not the detective.** The differential harness
   (`reconstruction/sim/p3diff.sh` + `p3diff_cmp.py`, the CK1–CK4 channels)
   exists to COMPARE, TEST, and ASSERT — to prove that a reconstruction built
   from REA's C reproduces the original's observable behavior. It is not a
   discovery instrument. Do not guess control flow from divergence patterns.
3. **When in doubt → REA.** If a differential diverges and the cause is not
   already established: REA-decompile the handler(s) involved, read the C,
   fix the recon. Budget: at most 2 replay calibration runs per slice; a
   third run on the same question means you skipped REA.
4. **Current phase:** a large part of the existing recon C was recovered
   piecemeal (stream archaeology, marker experiments, partial decompiles).
   **The standing task is to VALIDATE the reconstruction's C against fresh
   REA decompilation, function by function, and fix every divergence** —
   both behavioral bugs (wrong gate, wrong order, wrong constant) and
   provenance gaps (functions whose recon C was inferred without a full
   decompile). §5.0 is that sweep.

## 1. The termination condition — what DONE means (the only exit)

DONE = ALL of the following, verified FRESH in the session that closes the
goal (re-run everything that session; never cite stale numbers):

1. **Build & tests:** `reconstruction` builds green; `make test-host` — all
   pass, with NO test weakened or deleted since `1f790a8` (verify with
   `git diff 1f790a8 -- reconstruction/tests/`; any test change must be a
   strictly-added coverage, never a loosened assertion).
2. **CK4 (display/I²C):** the FULL GPIOB stream (every ODR/BSRR/BRR/CRH/CRL
   write, boot + 3 s) EVENT-IDENTICAL between the images; ODR 100.00%; the
   event census exact. The acceptance bar is the FULL 16-bit value — never
   regress to masked comparison.
3. **CK3 (AT1846S SPI):** the bus-event trace byte-identical (292/292 at
   minimum; extended tone/RF stimulus must also match — see §5.4).
4. **CK2 (peer UART):** the `$A00` handshake byte-identical AND the expanded
   command matrix (§5.6) shows identical observable responses on both images.
5. **CK4-key (keyscan):** every injectable key code (§5.6) produces identical
   RAM state clusters AND identical downstream display stream deltas
   (render-paced slices: the recon must reproduce the original's stream as a
   prefix covering the full original stream; wall-pacing residuals are
   documented per slice).
6. **The completeness ledger (`docs/COVERAGE.md`):** 100% of the distributed
   image's code paths classified — each function/region is one of:
   `[RECOVERED]` / `[DATA-DERIVED]` / `[INFERRED]` (with the recon source
   cite), `UNREACHABLE-PLATFORM` (with the proof), or `DATA` (with the
   layout). Zero `UNCLASSIFIED` rows. Method in §5.5.
7. **The gap ledger = empty except proven-unreachable items:** every item
   parked by prior sessions is either CLOSED, or has become
   `UNREACHABLE-PLATFORM` with a proof, or was reconstructed.
8. **The REA-validation ledger (§5.0):** every recovered function carries the
   `[REA-C-VALIDATED]` mark (the REA pseudocode compared against the recon C,
   divergences fixed or justified) or an explicit `REA-PENDING` note with the
   reason. No recovered function may be silently unvalidated.

If any channel differs or any ledger row is unclassified, you are NOT done.
Do not stop while that is true; do not declare DONE on stale evidence.

## 2. The working rules (in force)

- **REA-first evidence chain.** A recovered function's C must trace to REA
  pseudocode (the Evidence ID / the decompiled function cite) OR carry an
  explicit legacy-provenance note and a §5.0 validation slot. Hand-reading
  assembly to establish behavior is a last resort, used only where REA's
  decompiler output is ambiguous, and the resulting claim gets re-checked in
  REA terms.
- **Evidence-driven, append-only.** Every claim → `docs/RESEARCH_LOG.md` as a
  new dated entry. Never rewrite old entries; corrections are new entries
  marked RETRACTION/CORRECTION with what superseded them.
- **Item loop:** REA-decompile → reconstruct/fix C → rebuild → replay-assert
  (p3diff) → log → commit → push to main. Coherent milestone commits. NEVER
  stage `prompt.md` or `prompt.md~`.
- **Two replay runs per question, then REA.** If a differential does not
  converge after two calibration runs of a slice, STOP replaying: REA-decompile
  the handler involved and fix the model from the C. Parking requires the
  decompile-cite for what remains unexplained.
- **Immutability:** the original binaries are immutable reference inputs.
  Workarounds live ONLY in `reconstruction/sim/` (harness/platform) and are
  restated in `docs/CK5_REPORT.md` §4. The reconstruction source may only
  change to match observed original behavior.
- **Provenance or it didn't happen:** recovered code gets
  `[RECOVERED]`/`[DATA-DERIVED]`/`[INFERRED]`/`[SCAFFOLD]`/`[REPLACEMENT]`
  annotations with the original address cite. A slice without a decompile
  citation AND a behavioral assert is not recovered.
- **Fidelity over elegance:** when the original's behavior looks like a bug,
  reproduce the bug. The north star is differential equivalence, not good
  design.
- **No test-weakening, no channel-faking:** the harness may grow stimulus
  scripts; the comparison code and the host tests may only grow stricter.
- **Watch the -O2 dead-code trap:** if the recon build's size/symbols do not
  move after an edit, or a behavior contradicts the source, verify the build
  actually contains the code (`arm-none-eabi-nm`, the .rodata sections) and
  that no earlier `if` swallows the branch. A C-level control-flow mistake
  will be silently dead-code-eliminated — the ELF is the ground truth of what
  runs, and REA on the recon ELF is a legitimate cross-check.

## 3. The current verified state (trust the log, but re-verify before building on it)

Verified through session 69 (commits through `7fc6455`):
- CK1 boot ✅; CK2 handshake ✅; CK3 292/292 ✅; CK4 FULL event-identical,
  census exact, ODR 100.00% ✅; host tests pass ✅.
- Keys (the p3diff prefix bar): key-1, key-6, key-10, key-13 = 100% exact;
  key-2/key-3 = 100% (the digit-select family); **key-14 = the orig's full
  stream reproduced as a prefix (362,176/362,176 short; 517,228/517,228
  long)** — the scan slice content is exact, the wall pacing is calibrated
  (the PB9 bit-bang delay) with the held-era scheduler residual documented;
  key-11/12 = 89.77% (51 display slices exact; the release/end-frame trigger
  still off — the §5.0 REA validation target).
- The PB9 head-serial protocol DECODED: the tune frame = 1,996 events (the
  996-bit burst + the CRH flips + the 996-bit burst + the flips); the two
  11-bit PLL fields (922..932, 1920..1930) = `f_kHz − 135168` (the slope 1 per
  kHz, verified linear on 25 key-14 frames + the key-11 end frame = the same
  encoding); the mode bits constant; the key-13 settings-save = the same
  transport (6,077 events).
- Key recovered machinery: the RTOS boot graph (144-pass boot anchor, 720 Hz
  tick), the display pipeline (100%), the I²C EEPROM layer, the AT1846S SPI
  layer + the tone wrapper, the keyscan = 3 ADC2 resistor ladders (the
  injectable codes {1,2,3,10,11,12,13,14} via the group-A windows), the
  protocol routing, the battery quantizer chain, the S-meter pipeline.
- Key structures: FLAG_BANK = 0x20005094 (bit2 of +2 = 0x20005096); the screen
  byte = 0x200050E8; the radio state = 0x20004E68; the key poll = 0x08019DCC
  (`key_scan` 0x080281A6, `key_get_code` 0x080282FC, the A/B/C ladder tables
  0x20001684/0xA4/0xC4, the maps {10,11,12,13,1,2,3,14}/{4,5,6,0,7,8,9,15}/
  {18,19,20}); the vtable[10].press = 0x08018ED4, vtable[10].release =
  0x08018F1A; the key-13 executor 0x08022CB4; the S-meter chain per session 58.

**Addressing trap (critical):** the log entries through ~session 28 use a base
0x4000 LOWER than the current disassembly (add 0x4000 to map them). Sessions
52+ use the current base. Comments in `reconstruction/src/ui.c` and `ui.h` are
MIXED (mostly current, some stale). **Before hooking or citing any address,
re-derive it with REA (`analyze_function` on the cited symbol/address) or from
the current disassembly.** A hook that never fires: first check ±0x4000, then
check whether the code path is gated.

## 4. Environment survival (after any machine reboot)

```
# Renode: ~/tools/renode_1.17.0-portable (persistent; verify ~/tools/renode_1.17.0-portable/renode exists;
#   if lost: download https://github.com/renode/renode/releases/download/v1.17.0/renode-1.17.0.linux-portable.tar.gz
#   and extract to ~/tools/renode_1.17.0-portable)
# Toolchain: /usr/bin/arm-none-eabi-gcc (the system package; the old
#   /tmp/opencode/arm-tc path is dead — use TOOLCHAIN_PATH=/usr/bin/)
# Disassembly (only as a cross-check; REA is the primary):
cp firmware/MAIN-MCU-HG-UV98-V1.0.1-190416.bin /tmp/opencode/lcd.bin
arm-none-eabi-objdump -D -b binary -m arm -M force-thumb --adjust-vma=0x08004000 \
  /tmp/opencode/lcd.bin > /tmp/opencode/lcd.asm
```
- Build: `cd reconstruction && make TOOLCHAIN_PATH=/usr/bin/`; host tests:
  `make test-host`.
- **Re-derive recon symbols after EVERY rebuild:**
  `arm-none-eabi-nm reconstruction/build/hguv98.elf | grep <sym>` — and
  remember -O2 inlines static single-caller functions (the missing symbol does
  NOT mean the missing code; check the size delta and the .rodata).
- Runs are SEQUENTIAL: the driver truncates `/home/sslayer/hguv98-sim/pb_stream.log`
  at init; `p3diff.sh NAME ORIG.resc RECON.resc` handles the fresh-EE delete,
  both runs, the markers, and the prefix comparison.
- Monitor python is single-line; use
  `m = emulationManager.CurrentEmulation.Machines[0]`; **file writes must
  close**: `f = open(p,"wb"); f.write(bytes(...)); f.close()` (without close
  the buffer flushes only via GC luck — 0-byte files result).
- The ADC2 injection = runtime `sysbus Tag <0x4001284C 0x4> "ADC2_DR" <value>`
  (a SPACE inside the angle brackets, not a comma). The tag value is
  persistent until re-tagged; the dispatch edge = the first sighting, and
  ADC2_DR reads must not be used as the held-state source without REA
  confirming the original's gate.
- Renode `cpu AddHook` register hooks: read via
  `self.GetRegister(n).RawValue` (strip the trailing `L`); write via
  `from Antmicro.Renode.Peripherals.CPU import RegisterValue;
  self.SetRegister(n, RegisterValue.Create(v, 32))`. **PC writes (r15) do
  NOT stick** — steer execution by feeding the register a branch consumes
  (§9). GDB is available when needed: `machine StartGdbServer 3333` (machine
  scope; `gdb-multiarch -ex 'target remote :3333'`), but prefer REA static
  pseudocode over live debugging.

## 5. The work queue (ordered)

### 5.0 THE REA VALIDATION SWEEP — the standing task (interleave with everything below)

Every function whose recon C exists but was recovered without a full REA
decompile (the stream-derived, the marker-derived, the partial-decompile
slices) gets: REA `analyze_function`/`batch_decompile` on the original
address → read the pseudocode C → diff against the recon C → fix every
divergence (the gates, the order, the constants, the loop shapes) → replay-
assert through the committed driver → mark `[REA-C-VALIDATED]` in
COVERAGE.md + the log.

Priority order (the highest-divergence-risk first):
1. The keyscan family: the dispatch chain 0x08019DCC, `key_scan`/`key_get_code`,
   the vtable press/release handlers (the vtable base = findable via the
   0x08018ED4 pointer), the key-11/12 scan handler (the held cadence, the
   release gate, the end-frame trigger — this VALIDATES the parked 89.77%
   key-11 residual), the key-14 scan handler (validate the 8-pass-era
   assumptions: the free-run vs the timer cadence).
2. The AT1846S tune/PB9 frame senders (validate the PLL field formula and the
   frame assembly against the decompiled writer).
3. The RSSI task + the S-meter chain (the integrator modes, the clamps).
4. The settings consumers (0x08021238/0x080214E4/0x0802166C).
5. Everything else in COVERAGE.md marked RECOVERED-without-pseudocode.

Rules: batch `batch_decompile` calls per family; keep the Evidence IDs in the
log entries; a divergence found here beats any number of replay runs.

### 5.1 The UI surface — the full screen tree (REA-driven now)

Drive every screen/flow: REA-decompile the per-key vtable handlers and the
screen machine, implement from the C, then the CK4-key-style differential.
The known anchors: the screen byte 0x200050E8 (the printable-character state
enum), ui_state_set, the per-UI-state progress driver, the memory-mode entries,
the blinking mode-indicator driver. The menu tree = the key-driven screen
transitions; the stimulus matrix = every key code × short/long press. When a
screen's stream diverges, REA the renderer before touching the recon.

### 5.2 The settings layer (the two-instance model)

Validate the recon's settings shadow (0x2000684E), the head's mirror frame in
rxBuf (0x20003A84, settings at +0x14), and the consumers against REA
decompilation. Provenance: the CHIRP driver reference
`analysis/hg_uv98_chirp_driver_reference.py` = the EEPROM 0x0A00 settings
layout ground truth. Verify each consumed field via the decompile + a
replay assert.

### 5.3 The remaining decode items

- The `$A09` downstream consumer (the frame-ready pump) — REA the
  FLAG_BANK+9 bit0 readers, the key_event sink chain.
- The S-meter mode scheduler: REA the writers of [0x2000469C+0x76]; then the
  RSSI sweep difference-matching.
- The `0x20004E40` radio-state region vs the lazy per-field settings reads —
  classify every field.
- The RX/TX signal path: the squelch/RSSI poll task (0x0801B704, 500 ms), the
  PTT synthesizer 0x08019F70, the TX keying — REA-decompile, reconstruct,
  differential-test what the platform can express.
- The EEPROM write path parity (ee_save/ee_write cycles) on the I²C bus —
  and the PB9 head-serial save (the key-13 6,077-event frame) validation
  against the decompiled sender.

### 5.4 The completeness ledger (`docs/COVERAGE.md`)

Build the ledger once, maintain it forever after:
1. From the reset vector + the exception table + every `bl` target reachable
   from them, compute the call-target closure (REA callers/callees graphs are
   the tool); add the task entry points from the RTOS task creations; add a
   prologue scan for never-called functions (callback-only handlers).
2. Classify each function: the recon source cite (with the provenance tag +
   the original address), `UNREACHABLE-PLATFORM` (with the existing proofs),
   or `DATA` (with the layout).
3. The image regions with no function: bound and classify as data.
4. A row is closed only with the cite or the proof. DONE requires zero
   unclassified rows. The ledger anchors to the ORIGINAL's addresses, which
   never shift.

### 5.5 The stimulus matrix (the DONE-gate breadth)

The differential battery must cover, at minimum:
- **UART RX** (the peer injection, both images): `$A00` (the neutral, the
  marker, and the per-field settings variants), `$A01` (the length-gate
  question resolved first), `$A02`, `$A08`, `$A09`, and every head command
  whose compare string is inside the distributed image — the tail commands
  are proven unreachable (do NOT chase; cite the existing proof).
- **Keys:** the ADC2_DR Tag injection for every injectable code
  {1,2,3,10,11,12,13,14}; short-press AND long-press (the auto-repeat paths).
- **Battery:** the ADC2 values straddling 3429/3357/3155/2851 (the quantizer
  tree) — the level transitions must match.
- **Tone/RF:** the AT1846S stimulus via the CK3 harness — the tone wrapper
  matrix (chip × type × code over the CTCSS/DCS tables) and the boot pair.
- **Boot:** the default path and the factory-reset path.

## 6. The verification battery (run fresh, same session, when claiming DONE)

```
cd reconstruction && make TOOLCHAIN_PATH=/usr/bin/ && make test-host
git diff 1f790a8 -- reconstruction/tests/   # must show only added coverage
bash reconstruction/sim/p3diff.sh p3-key1 ... # (the driver pattern for every channel)
#   - the CK4 harness pair (ck4-orig/ck4-recon) + the FULL-stream identity,
#     the ODR 100.00% + the census comparison (followup.md §2)
#   - the CK3 harness pair (ck3-tonediag/ck3-recon + the original-side twin)
#   - the CK2 peer pair (peer-8051.resc / peer-recon.resc)
#   - the ck4-key matrix scripts
#   - the §5.5 matrix additions
# Log the fresh numbers in docs/RESEARCH_LOG.md, write docs/COVERAGE.md to 100%,
# commit, push — and only then close the goal with the evidence summary.
```

## 7. Anti-stall and anti-premature-done

- If a differential diverges: REA FIRST. Two replay runs max, then the
  decompiler. Never idle on parked items; never abandon the goal.
- If DONE seems within reach but one channel differs: that is not DONE. Fix
  the difference. Never weaken a comparison, a test, or a harness to pass.
- If you find the original doing something the recon cannot express on this
  platform, prove the boundary (the unreachable evidence style of the tail
  commands) and classify it `UNREACHABLE-PLATFORM` in COVERAGE.md.
- The goal closes ONLY with: the fresh battery numbers in the log, COVERAGE.md
  at 100% classified, the gap ledger empty-but-for-proofs, and the §5.0
  validation ledger complete.

## 8. Known-proven boundaries (do NOT re-litigate, do cite)

- The twelve tail compare strings ($A10/$A11/$A30/$A31/$D00/$F00/$A50/$A51/
  $E0x/$F0x) and the boot magic word live past the distributed image's end
  (the vendor flash the distribution truncates): reads return 0 on the
  simulator, the dispatches fall through identically on both images. The
  branch actions are documented (the session-52 pt4 log entry) for
  real-hardware equivalence.
- The clone-handler strings (SG0NNiNH/SG0NP3107) = inside the stripped boot
  region, not recoverable from this image; the protocol is known from CHIRP.
- TIM2 CH1/PC7-PC8 accessory sampler: dead on the model (the Renode STM32
  Timer resets CCR1 > ARR; the vendor never writes CCR1/CCMR1/CCER) — both
  images behave identically (the session-56 evidence).

## 9. The technique library

### 9.0 REA (the primary)
- **Open the target:** `open_binary` on
  `firmware/MAIN-MCU-HG-UV98-V1.0.1-190416.bin` (raw binary, ARM Cortex-M,
  Thumb, load base 0x08004000). Keep one native session per session; reuse it
  for every query (do not re-open per question).
- **Decompile:** `analyze_function` for one function (pseudocode + the
  callers/callees + the strings + the CFG); `batch_decompile` for a family
  (the vtable handlers, the task bodies). Read the C, not the assembly.
- **Call-chain questions:** the callers/incoming_references facets answer
  "who calls this sender"; the vtable = the pointer search (find 0x08018ED4
  in flash to locate the table base, then entries 11/12/13/14).
- **Cross-checks:** REA on the RECON ELF is legitimate for verifying what the
  compiler actually built (the -O2 inlining/dead-code traps) — cite both
  sides when a build-output question arises.
- **Log discipline:** every REA-derived fix gets the Evidence ID in
  docs/RESEARCH_LOG.md and the `[REA-C-VALIDATED]` mark in COVERAGE.md.

### 9.1 Marker experiments
Inject distinguishable bytes through a legitimate frame (e.g. the `$A00`
settings block), then scan the 64 KB SRAM dump for the pattern to find where
the firmware stores/parses it. Proved rxBuf = 0x20003A84 and killed the
double-copy myth.

### 9.2 Renode instruments (the harness side)
- **CPU-hook register steering:** to force a code path, feed the register a
  conditional branch consumes — NOT the PC (r15 writes do not stick).
- **Return-site value injection:** hook the instruction after a `bl` and
  override r0 (the RSSI override at 0x08020468).
- **Flag pokes are futile** against firmware-recomputed RAM — patch at the
  CPU-hook level or unlock through the state machine.
- **Runtime sysbus Tag** for the ADC2_DR key/battery injection.
- **showAnalyzer sysbus.usart2** for the UART traces; grep the `$` frames.
- **The prologue test:** a hook address is only valid if the disassembly shows
  a function prologue there; a hook that never fires means either the wrong
  address (check ±0x4000) or a gated path (find the gate — with REA).
- **The segmenter** `/tmp/opencode/segment.py` = the stream cell viewer for
  the p3diff comparisons (display cells, fills, PB9 bursts).

## 10. The resume protocol (if the session ever restarts)

1. `git log --oneline -15` + the tail of `docs/RESEARCH_LOG.md` = where you
   are. GOAL.md §0 = the method priority (REA first; replay = verify).
2. Re-verify §4's environment; rebuild with `TOOLCHAIN_PATH=/usr/bin/`;
   re-derive the recon symbols; check the -O2 size deltas.
3. Check the §5.0 validation ledger + the gap ledger (the parked items in the
   log).
4. Continue the work queue from the first unclosed item — REA-decompile before
   reconstructing, replay-assert after. Repeat until §1's DONE gate passes
   fresh, then close.
