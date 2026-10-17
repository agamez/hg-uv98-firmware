# GOAL.md — HG-UV98 firmware recovery: the one-shot completion directive

You are an autonomous agent. This file is your mission specification. Work it
top to bottom, loop until §1's DONE gate passes with fresh evidence, and do not
stop, ask, or wait for a human at any decision point. Every decision rule you
need is in this file. The repo contains the full evidence base
(`docs/RESEARCH_LOG.md`, 58+ sessions) and a prior incremental handoff
(`followup.md`) — this file supersedes followup.md's KICKOFF as the driver, but
followup.md §2/§6/§7 remain valid reference (the harness recipes and the Renode
survival guide).

---

## 0. The mission

The vendored firmware of the HG-UV98 handheld (main MCU, distributed image
`firmware/MAIN-MCU-HG-UV98-V1.0.1-190416.bin`, loaded at 0x08004000) has been
reverse-engineered into a C reconstruction (`reconstruction/src/`, builds to
`reconstruction/build/hguv98.elf`). Both binaries run in Renode. The
reconstruction already passes several byte-exact differential channels.

**Your mission: finish the recovery — drive the reconstruction to full
behavioral equivalence with the original on this platform, close every
remaining gap, and prove completeness.** One shot, no human in the loop.

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
   RAM state clusters AND identical downstream display stream deltas.
6. **The completeness ledger (`docs/COVERAGE.md`):** 100% of the distributed
   image's code paths classified — each function/region is one of:
   `[RECOVERED]` / `[DATA-DERIVED]` / `[INFERRED]` (with the recon source
   cite), `UNREACHABLE-PLATFORM` (with the proof), or `DATA` (with the
   layout). Zero `UNCLASSIFIED` rows. Method in §5.5.
7. **The gap ledger = empty except proven-unreachable items:** every item
   parked by prior sessions is either CLOSED, or has become
   `UNREACHABLE-PLATFORM` with a proof, or was reconstructed.

If any channel differs or any ledger row is unclassified, you are NOT done.
Do not stop while that is true; do not declare DONE on stale evidence.

## 2. The working rules (in force)

- **Evidence-driven, append-only.** Every claim → `docs/RESEARCH_LOG.md` as a
  new dated entry. Never rewrite old entries; corrections are new entries
  marked RETRACTION/CORRECTION with what superseded them.
- **Item loop:** decode/change → rebuild → run → compare → log → commit →
  push to main. Coherent milestone commits. NEVER stage `prompt.md` or
  `prompt.md~`.
- **Three genuine attempts per item, then park with evidence** — but parking
  is never terminal: after parking, either (a) attack the ROOT blocker (most
  parkings share the boot-state root — see §5.1), or (b) route around with a
  CPU-hook decode instrument (§9), or (c) move to the next item and revisit
  after new findings. Return to every parked item after each root finding.
- **Immutability:** the original binaries are immutable reference inputs.
  Workarounds live ONLY in `reconstruction/sim/` (harness/platform) and are
  restated in `docs/CK5_REPORT.md` §4. The reconstruction source may only
  change to match observed original behavior.
- **Provenance or it didn't happen:** recovered code gets
  `[RECOVERED]`/`[DATA-DERIVED]`/`[INFERRED]`/`[SCAFFOLD]`/`[REPLACEMENT]`
  annotations with the original address cite. A slice without both a decompile
  citation AND a behavioral test is not recovered.
- **Fidelity over elegance:** when the original's behavior looks like a bug,
  reproduce the bug. The north star is differential equivalence, not good
  design.
- **No test-weakening, no channel-faking:** the harness may grow stimulus
  scripts; the comparison code and the host tests may only grow stricter.

## 3. The current verified state (trust the log, but re-verify before building on it)

Verified through `1f790a8` (sessions 49–58):
- CK1 boot ✅; CK2 handshake ✅; CK3 292/292 ✅; CK4 FULL 187,757 events
  event-identical, census exact, ODR 160,753/160,753 = 100.00% ✅;
  host tests pass ✅.
- Closed rounds: A/R1/D/K/G/H/E/F/M/N/O/P-item1/B-C-items-1&3 (details in
  `docs/PLAN.md` + the log).
- Key recovered machinery: the RTOS boot graph, the display pipeline (100%),
  the I²C EEPROM layer, the AT1846S SPI layer + the tone wrapper 0x0801A91A
  (the boot pair computed from state), the keyscan = 3 ADC2 resistor ladders
  (the stimulus = the ADC2_DR runtime Tag, values < 2851 to stay battery-safe;
  the injectable codes {1,2,3,10,11,12,13,14} via the group-A windows), the
  protocol routing (the marker-verified parse surface rxBuf = 0x20003A84, the
  settings at +0x14; `$A00` → rxBuf ONLY), the battery quantizer chain, the
  S-meter bar pipeline (the integrator + the step table, below).
- Key structures: FLAG_BANK = 0x20005094 (bit2 of +2 = 0x20005096 = the
  app-state "menu" gate — the ROOT blocker; bits 6/7 of +0 = the S-meter
  path enables, firmware-recomputed every few ms); the screen byte =
  0x200050E8; the radio state = 0x20004E68 (the band byte at +0x40 =
  0x20004EA8); the main's own settings shadow = 0x2000684E (pf1/pf2/top_short
  at +0x21/0x23/0x25, the CHIRP offsets); the 4×30-byte per-band ring tables
  = 0x20004F94; the S-meter context = 0x2000469C.
- S-meter (session 58 pt3, resolved): the read = rf_read(chip,0x1B)[15:8] at
  0x08020464 inside the updater 0x080203D0; the integrator 0x080201AA (mode
  11 = += the step [ctx+0x2E+band*2] or += 100000 squelch-open; mode 12 =
  += 0xfffe7960 i.e. −100000); the clamps [ctx+0x62+band*4]=0 ..
  [ctx+0x5A+band*4]=99500; the step table 0x0801F1C4 = {500,625,1000,1250,
  2500,5000,10000} selected by a settings byte; the ring = the BCD bar-value
  history rendered by 0x0801AB36/0x0801A7AA/0x0801E296.

**Addressing trap (critical):** the log entries through ~session 28 use a base
0x4000 LOWER than the current disassembly (add 0x4000 to map them). Sessions
52+ use the current base. Comments in `reconstruction/src/ui.c` and `ui.h` are
MIXED (mostly current, some stale). **Before hooking or citing any address,
re-derive it from the current `/tmp/opencode/lcd.asm`.** A hook that never
fires: first check ±0x4000, then check whether the code path is gated.

## 4. Environment survival (after any machine reboot)

```
# Renode: ~/tools/renode_1.17.0-portable (persistent; verify ~/tools/renode_1.17.0-portable/renode exists;
#   if lost: download https://github.com/renode/renode/releases/download/v1.17.0/renode-1.17.0.linux-portable.tar.gz
#   and extract to ~/tools/renode_1.17.0-portable)
# Toolchain: if /tmp/opencode/arm-tc/usr/bin/arm-none-eabi-gcc is missing:
mkdir -p /tmp/opencode/arm-tc && for d in /tmp/opencode/debs/*.deb; do dpkg -x "$d" /tmp/opencode/arm-tc/; done
#   (if /tmp/opencode/debs is also gone: apt-get download gcc-arm-none-eabi
#    binutils-arm-none-eabi libnewlib-arm-none-eabi libstdc++-arm-none-eabi-newlib
#    into /tmp/opencode/debs, then the dpkg -x loop)
# Disassembly: regenerate after every reboot
cp firmware/MAIN-MCU-HG-UV98-V1.0.1-190416.bin /tmp/opencode/lcd.bin
arm-none-eabi-objdump -D -b binary -m arm -M force-thumb --adjust-vma=0x08004000 \
  /tmp/opencode/lcd.bin > /tmp/opencode/lcd.asm
```
- Build: `cd reconstruction && make TOOLCHAIN_PATH=/tmp/opencode/arm-tc/usr/bin/`;
  host tests: `make test-host`.
- **Re-derive recon symbols after EVERY rebuild:**
  `/tmp/opencode/arm-tc/usr/bin/arm-none-eabi-nm reconstruction/build/hguv98.elf | grep <sym>`
  (hook addresses in every `reconstruction/sim/*.resc` that hooks the recon
  shift on rebuild — ck3-recon.resc's rf hooks are the known example).
- Runs are SEQUENTIAL: `pb_displaybus.py` truncates `/tmp/opencode/pb_stream.log`
  at init; copy to a persistent name after each run.
- Monitor python is single-line; use
  `m = emulationManager.CurrentEmulation.Machines[0]`; **file writes must
  close**: `f = open(p,"wb"); f.write(bytes(...)); f.close()` (without close
  the buffer flushes only via GC luck — 0-byte files result).
- The ADC2 injection = runtime `sysbus Tag <0x4001284C 0x4> "ADC2_DR" <value>`
  (a SPACE inside the angle brackets, not a comma).
- Renode `cpu AddHook` register hooks: read via
  `self.GetRegister(n).RawValue` (strip the trailing `L`); write via
  `from Antmicro.Renode.Peripherals.CPU import RegisterValue;
  self.SetRegister(n, RegisterValue.Create(v, 32))`. **PC writes (r15) do
  NOT stick** — steer execution by feeding the register a branch consumes
  instead (§9).

## 5. The work queue (ordered)

### 5.1 THE MASTER KEY — the boot/app state machine (do this first; most parkings share this root)

The firmware boots into a settings/menu app-state: FLAG_BANK+2 bit2
(0x20005096) = 1 from boot and is re-asserted ≤10 ms by the firmware. Behind
this gate sit: the key-driven UI (only the side keys act), the `$A01` write
acceptance, the `$A09` frame consumer, the S-meter mode scheduler
([0x2000469C+0x76] = stuck 0), and the full screen tree. Known threads:
- The boot runs a factory-reset path because the app-state magic check
  (app+0x44 vs a ROM word beyond the distributed image) fails on the
  simulator. Decode what the reset path sets and what would transition the
  state afterward.
- The state machine's writers: trace every write to 0x20005096 (and to the
  flag-bank byte 0 recomputation) — find the transition function and its
  conditions. The `$A01` settings-write cycle (the 8051 programming the
  radio) is the prime candidate for the natural exit: decode the full $A01
  acceptance path (frame length: the recon gates 0x216 = 534 B but the vendor
  source says `$A01,<512B>\r\n` = 519 B — re-verify the original's length
  gate against the disassembly and the live behavior; one of them is wrong).
- The peer handshake may need to COMPLETE (a $A01 write + the ACK + a
  $A02/$A08 cadence?) before the state exits. Drive it with peer-8051.resc
  variants until the transition happens naturally, then lock the sequence
  into the harness.

**Exit criterion:** on both images, a scripted stimulus sequence flips
FLAG_BANK+2 bit2 1→0 and the post-transition behavior (keys, S-meter enables,
$A01) matches. Then the gated items below unblock naturally.

### 5.2 The settings layer (the two-instance model)

Implement in the recon: the main's own settings shadow at 0x2000684E (the
boot EEPROM load + the runtime writers, e.g. 0x0801685A for the S-meter
steps), the head's mirror frame in rxBuf (0x20003A84, settings at +0x14), and
the consumers discovered so far (the side-key executors 0x08021238/0x080214E4/
0x0802166C reading +0x21/0x23/0x25; the S-meter step selection 0x0801F1C4).
Provenance: the CHIRP driver reference `analysis/hg_uv98_chirp_driver_reference.py`
= the EEPROM 0x0A00 settings layout ground truth. Verify each consumed field
with a marker experiment (§9) before wiring it.

### 5.3 The UI surface — the full screen tree

Once §5.1 exits the boot state: drive every screen/flow via the key stimulus
+ the UART and make the display streams match. Work screen by screen (each =
decode → implement → the CK4-key-style differential → commit). The known
anchors: the screen byte 0x200050E8 (the printable-character state enum:
0x00, 0x02, 0x04, 0x2F, 0x36, 0x3F, 0x40 '@', 0x45 'E', 0x47 'G', 0x4D 'M',
0x50 'P', 0x58 'X', 0x67 'g', 0x68 'h', 0x69 'i', 0x78 'x', 0x7B, 0x86);
ui_state_set = the guarded write to 0x200050E8 (re-derive the current address
from lcd.asm — the logged 0x08010530 is the OLD base); the per-UI-state
progress driver 0x08010302+0x4000; the memory-mode entries 'M'/'N' at
0x08011DC8/0x08011DE8+0x4000; the blinking mode-indicator driver
0x0800ED24+0x4000 (the 17th-pass toggle). The menu tree = the key-driven
screen transitions; the stimulus matrix = every key code × short/long press.

### 5.4 The remaining decode items

- The `$A09` downstream consumer (the frame-ready pump that converts the
  frame into a key event) — revisit after §5.1; the hooks to start from:
  FLAG_BANK+9 bit0 readers, the key_event sink chain.
- The S-meter mode scheduler: the writers of [0x2000469C+0x76] (the mode
  11/12 toggler) and the FLAG_BANK recompute; then re-run the RSSI sweep
  (ck2-smeter-db*.resc) and require the stream DIFFERENCES to match between
  the images (this is the live dB→bar verification).
- The `0x20004E40` radio-state region vs the lazy per-field settings reads
  (CK5 §3.2/§3.3) — classify every field.
- The RX/TX signal path: the squelch/RSSI poll task (0x0801B704, 500 ms),
  the PTT synthesizer 0x08019F70, the TX keying — decode and differential-
  test what the platform can express.
- The EEPROM write path parity (ee_save/ee_write cycles) on the I²C bus.

### 5.5 The completeness ledger (`docs/COVERAGE.md`)

Build the ledger once, maintain it forever after:
1. From the reset vector + the exception table + every `bl` target reachable
   from them, compute the call-target closure; add the task entry points from
   the RTOS task creations; add a prologue scan
   (`push {..., lr}` / `stmdb sp!, {..., lr}`) over the whole image for
   never-called functions (callback-only handlers).
2. Classify each function: the recon source cite (with the provenance tag +
   the original address), `UNREACHABLE-PLATFORM` (the tail-command region
   past the distributed end, the vendor-flash words — with the existing
   proofs), or `DATA` (the tables/strings/pools — with the layout).
3. The image regions with no function: bound and classify as data.
4. A row is closed only with the cite or the proof. DONE requires zero
   unclassified rows. Re-check after every recon change (the addresses shift
   only in the recon; the ledger anchors to the ORIGINAL's addresses, which
   never shift).

### 5.6 The stimulus matrix (the DONE-gate breadth)

The differential battery must cover, at minimum:
- **UART RX** (the peer injection, both images): `$A00` (the neutral, the
  marker, and the per-field settings variants), `$A01` (after §5.1; the
  length-gate question resolved first), `$A02`, `$A08`, `$A09`, and every
  head command whose compare string is inside the distributed image ($B/$C/
  $E/$F/$G/$H/$J/$L/$M families) — the tail commands are proven unreachable
  (do NOT chase; cite the existing proof).
- **Keys:** the ADC2_DR Tag injection for every injectable code
  {1,2,3,10,11,12,13,14} and every code the other group windows yield once
  decoded; short-press AND long-press (the auto-repeat paths).
- **Battery:** the ADC2 values straddling 3429/3357/3155/2851 (the quantizer
  tree) — the level transitions must match.
- **Tone/RF:** the AT1846S stimulus via the CK3 harness — the tone wrapper
  matrix (chip × type × code over the CTCSS/DCS tables) and the boot pair.
- **Boot:** the default path and the factory-reset path.

## 6. The verification battery (run fresh, same session, when claiming DONE)

```
cd reconstruction && make TOOLCHAIN_PATH=/tmp/opencode/arm-tc/usr/bin/ && make test-host
git diff 1f790a8 -- reconstruction/tests/   # must show only added coverage
cd ~/tools/renode_1.17.0-portable
timeout 280 ./renode --console --disable-xwt <repo>/reconstruction/sim/ck4-orig.resc
cp /tmp/opencode/pb_stream.log /tmp/opencode/pb_stream_orig.log
timeout 280 ./renode --console --disable-xwt <repo>/reconstruction/sim/ck4-recon.resc
cp /tmp/opencode/pb_stream.log /tmp/opencode/pb_stream_recon_final.log
# then the FULL-stream identity + the ODR 100.00% + the census comparison
# (the python snippets in followup.md §2), plus:
#   - the CK3 harness pair (ck3-tonediag/ck3-recon + the original-side twin)
#   - the CK2 peer pair (peer-8051.resc / peer-recon.resc)
#   - the ck4-key matrix scripts
#   - the §5.6 matrix additions
# Log the fresh numbers in docs/RESEARCH_LOG.md, write docs/COVERAGE.md to 100%,
# commit, push — and only then close the goal with the evidence summary.
```

## 7. Anti-stall and anti-premature-done

- If an item stalls 3 attempts: PARK it in the log with the evidence, then
  IMMEDIATELY pick the root-unblocking item (§5.1) or the next queue item.
  Never idle on a parked item; never abandon the goal.
- If DONE seems within reach but one channel differs: that is not DONE. Fix
  the difference. Never weaken a comparison, a test, or a harness to pass.
- If you find the original doing something the recon cannot express on this
  platform, prove the boundary (the unreachable evidence style of the tail
  commands) and classify it `UNREACHABLE-PLATFORM` in COVERAGE.md.
- The goal closes ONLY with: the fresh battery numbers in the log, COVERAGE.md
  at 100% classified, and the gap ledger empty-but-for-proofs.

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

## 9. The technique library (the decode instruments that already worked)

- **Marker experiments:** inject distinguishable bytes through a legitimate
  frame (e.g. the `$A00` settings block), then scan the 64 KB SRAM dump for
  the pattern to find where the firmware stores/parses it. Proved rxBuf =
  0x20003A84 and killed the double-copy myth.
- **CPU-hook register steering:** to force a code path, feed the register a
  conditional branch consumes (`self.SetRegister(0, 1)` right before the
  `lsls/bmi` pair) — NOT the PC (r15 writes do not stick). Proven: the
  S-meter pipeline went live end-to-end on the original this way
  (ck2-smeter-poke8.resc).
- **Return-site value injection:** hook the instruction after a `bl` and
  override r0 to fake a return value (the RSSI override at 0x08020468:
  `SetRegister(0, db<<8)`). Used for the sweep.
- **Flag pokes are futile** against firmware-recomputed RAM (the flag bank,
  the radio state) — anything rewritten every few ms must be patched at the
  CPU-hook level or unlocked through the state machine.
- **Runtime sysbus Tag** for the ADC2_DR key/battery injection.
- **showAnalyzer sysbus.usart2** for the UART traces; grep the `$` frames.
- **The ring-write helper 0x0801A8DA has 14 call sites** — always
  disambiguate generic helpers with a caller-identifying hook (log lr — note
  lr reads as 0 in this Renode's hook context; instead hook the distinct
  call sites).
- **The prologue test:** a hook address is only valid if lcd.asm shows a
  function prologue (push {..., lr}) there; a hook that never fires means
  either the wrong address (check ±0x4000) or a gated path (find the gate).

## 10. The resume protocol (if the session ever restarts)

1. `git log --oneline -15` + the tail of `docs/RESEARCH_LOG.md` + the current
   followup.md §3 = where you are.
2. Re-verify §4's environment; rebuild; re-derive the recon symbols.
3. Check the gap ledger (the parked items in the log) against any new root
   findings from §5.1.
4. Continue the work queue from the first unclosed item. Repeat until §1's
   DONE gate passes fresh, then close.
