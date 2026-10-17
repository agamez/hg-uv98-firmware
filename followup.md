# followup.md — the HG-UV98 reconstruction: simulation & differential validation

## KICKOFF — the prompt to give the agent (paste verbatim; keep this block current)

```
Read followup.md in the repo root and execute it from the top.
§1 = the current state and round map; §3 = THE ACTIVE ROUND with its
ordered open items; §4 = the working rules (three attempts max per
item, then park it with evidence and move on — do not loop). Before
your first change, reproduce §2's three verification recipes and
confirm 160,753/160,753 ODR events = 100.00% AND the full 187,757-
event GPIOB stream = event-identical. Work item by item: change ->
rebuild -> run -> compare -> log -> commit. Log every claim in
docs/RESEARCH_LOG.md (append-only), commit at milestones, push to
main, never stage prompt.md or prompt.md~. When the active round
closes (or is parked), update THIS kickoff block and §3 before moving
to the next round. ACTIVE: Round B/C (protocol broadening — the $A09
4-byte-prefix routing first; the key-driven $A01 stays parked on the
state gate). Rounds N and O are CLOSED; Round P item 1 (host tests)
is DONE, all conformance tests pass.
```

You are continuing a completed reverse-engineering project. The analysis phase is
**finished**; your mission is the simulation/differential phase: run both the
original binary and the C reconstruction in Renode, drive them identically, diff
the observable outputs, and close the remaining round items.
**Start at §1 (the session-54 handoff).** The previous handoffs (session-44
CK1 bring-up, sessions 45-53) are fully recorded in `docs/RESEARCH_LOG.md`;
nothing in them needs re-deriving.

---

## 0. Inherited rules (from prompt.md — still in force)

- Evidence-driven. Every claim recorded in `docs/RESEARCH_LOG.md` (append a new
  session entry per work session; never rewrite old entries — corrections are
  appended as retractions).
- Git discipline: coherent milestone commits; **never stage the user's `prompt.md`
  or `prompt.md~`**. Push to main at clean states.
- Provenance annotations on all recovered code: `[RECOVERED]`, `[INFERRED]`,
  `[SCAFFOLD]`, `[REPLACEMENT]`, `[VENDOR]`, `[DATA-DERIVED]` (replayed tables
  extracted from a reference capture).
- The original firmware binaries are **immutable reference inputs**. Any file you
  produce that derives from them lives under `analysis/` (gitignored) or is a
  clearly-labeled generated artifact. **Workarounds live only in the
  harness/platform** (`reconstruction/sim/`) — never in the reconstruction — and
  are restated in `docs/CK5_REPORT.md` §4.
- **Addressing convention (critical):** all "0x080xxxxx" addresses in the log and
  docs = `file_offset + 0x08000000` (labels). Physical/linked flash = label + 0x4000.
  Pool-pointer *values* inside the image are linked addresses (file + 0x08004000).
  REA/Ghidra sessions open the raw binary with base 0x08004000 so REA addr = label + 0x4000.

## 1. Where things stand — session-54 handoff (READ THIS FIRST)

**Checkpoint status:**
- **CK1 (OS boot) ✅** — both images boot; the OS runs fully (context switches,
  SysTick tick, all 7 tasks).
- **CK2 (protocol) ✅** — byte-identical `$A00` handshake traces; the peer model
  v1 works (see §7 workarounds 6-8). The settings-write direction awaits the
  keyscan stimulus (Round N) → **now stimulus-proven but state-gated (Round N
  closed with a parked sub-item — see §3)**.
- **CK3 (AT1846S SPI) ✅** — 292/292 bus events byte-identical, same order
  (session 49).
- **CK4 (display + I²C) ✅✅✅ CLOSED — ★★ the FULL 187,757-event GPIOB stream
  (every ODR, BSRR, BRR, CRH and CRL write) = EVENT-BY-EVENT IDENTICAL,
  census exact on both images (CRH 4,902 / BSRR 7,913 / BRR 14,188 / CRL 1 /
  ODR 160,753); all 11 bursts match in position and length** (session 55).
  The acceptance bar remains the FULL 16-bit ODR value — do not regress to
  the masked comparison.
- **CK5 (the report)** — `docs/CK5_REPORT.md` maintained (§3.4 closed items
  updated, §4 workaround register current through session 56).

**Round map (details + statuses in `docs/PLAN.md`):**
A ✅ | R1 ✅ | D ✅ | K ✅ (tail commands unreachable — platform boundary) |
G ✅ (boot call graph) | H ✅✅ (display 100%) | E ✅ (drivers) | F ✅ (report draft)
| M ✅✅ (runtime I²C volume — full bus parity, session 55) |
| **N ✅ CLOSED session 56 (keyscan = 3 ADC2 ladders; the TIM2 boundary
re-diagnosed = CCR1 model reset; the stimulus works on both images; the $A01
acceptance parked-blocked on the app-state gate — Rounds B/C)** |
| **P ✅ item 1 CLOSED session 56 (the 4 host-test failures re-baselined to
the session-50 routing, all pass)** |
| **O ✅ CLOSED session 57 (the tone wrapper fully decoded; the recon boot
pair computed from state; CK3 292/292, CK4 100%)** |
| **B/C — ACTIVE (the $A09 4-byte-prefix routing first; the key-driven
$A01 parked on the state gate)** |
L (report maintenance, continuous) | B/C (protocol broadening — now ALSO
unblocked on the state gate: what clears FLAG_BANK+2 bit2 naturally).

**Repo state:** all work committed & pushed through the session-56 commits
(log = 56 sessions). `reconstruction/` builds green
(`make TOOLCHAIN_PATH=/tmp/opencode/arm-tc/usr/bin/` — the toolchain and Renode
live in /tmp and ~/tools, re-extract after a reboot; recipes in §2).

**Session-56 changes (these commits):** Round N closed — the keyscan
mechanism recovered (3 ADC2 resistor ladders + the code maps + the threshold
tables; the TIM2/PC7-PC8 pair = an accessory sampler, GPIOC not GPIOA), the
model boundary evidenced live (CCR1=0xFFFF reset, CC1IF never sets, the
sampler chain dead on both images), the ADC2_DR runtime-tag stimulus
implemented and proven on BOTH images (identical RAM key clusters), the
recon keyscan chain implemented [RECOVERED] (key_adc_read/key_scan/
key_get_code + the ui_task poll slice), the app-state gate found and parked
(FLAG_BANK+2 bit2 = 0x14 from boot, firmware-maintained ≤10 ms), and 4
pre-existing host-test failures discovered (Round P opened). **No ODR
regression**: 160,753/160,753 = 100.00% throughout.

## 2. The canonical run harness — reproduce any number in this file

**Build:** `cd reconstruction && make TOOLCHAIN_PATH=/tmp/opencode/arm-tc/usr/bin/`
(host tests: `make test-host`).

**Re-derive symbols after EVERY rebuild** (addresses shift):
`arm-none-eabi-nm reconstruction/build/hguv98.elf | grep <symbol>`.

**Run the original (3-s display capture):**
```
cd ~/tools/renode_1.17.0-portable
timeout 280 ./renode --console --disable-xwt reconstruction/sim/ck4-orig.resc
cp /tmp/opencode/pb_stream.log /tmp/opencode/pb_stream_orig.log   # canonical
```
**Run the reconstruction:**
```
timeout 280 ./renode --console --disable-xwt reconstruction/sim/ck4-recon.resc
cp /tmp/opencode/pb_stream.log /tmp/opencode/pb_stream_reconNN.log
```
(`pb_displaybus.py` truncates `/tmp/opencode/pb_stream.log` at peripheral init;
always copy it to a persistent name after the run. The scripts above load
`firmware/MAIN-MCU-HG-UV98-V1.0.1-190416.bin` @0x08004000 with the r9 hook, or
the recon ELF, run 10 ms, clear SysTick CVR, then run 3 s.)

**The comparison (the FULL GPIOB stream — the acceptance bar since session 55):**
```bash
python3 - <<'EOF'
def seq(p):
    return [(l.split()[1], l.split()[2]) for l in open(p)
            if len(l.split())>2 and l.split()[1] in
            ('ODR','BSRR','BRR','CRH','CRL')]
so=seq('/tmp/opencode/pb_stream_orig.log')
sr=seq('/tmp/opencode/pb_stream_recon34.log')
print('FULL stream:', 'EVENT-IDENTICAL' if so==sr else 'DIFFER',
      f'({len(so)} vs {len(sr)} events)')
o=[v for t,v in so if t=='ODR']; r=[v for t,v in sr if t=='ODR']
n=min(len(o),len(r)); s=sum(a==b for a,b in zip(o,r))
print(f'ODR: {s}/{n} = {100*s/n:.2f}%')
EOF
```
**The event census (the Round M accounting):**
```bash
python3 - <<'EOF'
from collections import Counter
for p in ('/tmp/opencode/pb_stream_orig.log',
          '/tmp/opencode/pb_stream_recon34.log'):
    c=Counter(l.split()[1] for l in open(p) if l.split()[1:2][0] in
              ('ODR','BSRR','BRR','CRH','CRL'))
    print(p, dict(c))
EOF
```
**The disassembly of the original:** `/tmp/opencode/lcd.asm` — regenerate after
a reboot with:
`arm-none-eabi-objdump -D -b binary -m arm -M force-thumb --adjust-vma=0x08004000 lcd.bin > /tmp/opencode/lcd.asm`
(lcd.bin = the distributed original; REA MCP works too — reopen the session per
burst: `open_binary` raw-binary, entry 0x08004000 — `batch_decompile` needs it).

## 3. Round B/C — protocol broadening (THE active work)

Round O CLOSED in session 57: the tone wrapper 0x0801A91A fully decoded
— the EEPROM-calibrated state (tick at APP_STATE+0x109, base/scaler at
+0xEC/+0xEA/+0xEF/+0xEE from EEPROM 0x0FCD/0x0FCF/0x1FCD/0x1FCF, tables
A/B at 0x20005010/0x20005030 from EEPROM 0x0FB0/0x1FB0/0x0F90/0x1F90,
the band-record gate = bit1 of byte 12 at 0x20004F94+chip*30), the
0x48/0x49 divider-pair computation, the ON/OFF presets 0x08026C46, the
12-bit GPIOD serial tail 0x0802883A. The recon's boot pair now
COMPUTED (was a constant replay); CK3 292/292 byte-identical (harness
= reconstruction/sim/ck3-recon.resc, hooks nm-derived after every
rebuild), CK4 100.00%, host tests pass. Full evidence: the research
log session 57.

**Round B/C items, in order:**

1. **The `$A09` 4-byte-prefix routing mechanics** (§9 item 1): the RX
   command `$A09\r\n` (the side-key beacon trigger, len>=4 per the
   protocol notes) — decompile the routing site, drive it via the peer
   injection (the RX direction is NOT blocked by the keyscan state
   gate), compare both images (the UART + the beacon effect).
2. **The key-driven `$A01` write direction** — parked-blocked on what
   clears FLAG_BANK+2 bit2 naturally (session 56); revisit only if a
   B/C finding touches the state machine.
3. **Cross-check the recovered `.data`-structure guesses against live
   behavior** (§9 item 2) as the B/C passes touch them.
4. **Round L maintenance** at the closure.

Ground truth for the tone path (do NOT re-derive): the session-57 log
entry. For the keyscan/state-gate/session-56 ground truth: the
session-56 pt1/pt2 entries. The addressing convention: REA addrs =
lcd.asm addrs (the binary @0x08004000).

**When Round B/C closes:** update CK5 §3/§4, close the rounds in
PLAN.md, refresh the KICKOFF block at the top of this file, commit,
then the remaining secondary items (§9: the S-meter sweep, the
0x20004E40 radio-state region) with **Round L** maintenance after
every closure.

## 4. Working rules (added session 54 — anti-loop)

- Work in **small verified increments**: one change → rebuild → run → compare →
  log → commit. Never stack unverified changes.
- **Three-attempt rule:** if an item's comparison does not converge after 3
  genuine attempts, STOP that item — append the current hypothesis + raw
  evidence to the log, mark it parked-with-evidence, and move to the next item.
- Never re-run an experiment that already produced a logged number; §2's
  recipes reproduce any of them.
- The Renode monitor's `python` is **single-line only** (multi-line scripts fail
  to tokenize); RAM dumps work as a one-liner
  `python 'open("/tmp/opencode/x.bin","wb").write(bytes(machine["sysbus"].ReadBytes(0x20000000, 0x10000)))'`.
- Scripts abort at the FIRST failing command — a bad mid-script command
  silently truncates the run.

## 5. Hard-won ground truth (do NOT re-derive; verified sessions 51-54)

- **The acceptance bar = the FULL 16-bit ODR value.** The bit9 of every display
  ODR is read-modify-write preserved from the PB9 latch (the SDA state leaks
  into every display event).
- **I²C writer mapping:** 0x08023780 = BSRR (+0x10), 0x08023788 = BRR (+0x14),
  0x0802375E = IDR (+8). (The session-51 reversed note was retracted.)
- **The I²C primitives (as committed in eeprom.c, session-55 state):**
  start (0x080284F2) = SDA high, SCL high, SDA low, SCL low;
  stop (0x08028532) = SDA low, SCL high, SDA high;
  master_ack (0x08028566) = val==0 → BRR (the ACK = SDA LOW), val≠0 → BSRR
  (the NAK = SDA HIGH);
  send_byte (0x080285B6) = MSB-first with NON-INVERTED SDA polarity
  (bit=1 → BSRR = SDA HIGH, bit=0 → BRR; the ack tail = BSRR) — the
  session-51/54 "inverted" note RETRACTED (four bl targets were misread;
  the captured 0xA0 byte proves the non-inverted mapping).
  Per-op event budgets: read(count=N) = 56+3N events; 1-byte write = 53;
  the read_byte emits 1 CRH (SCL=PA11 is uncaptured).
- **The boot step-5 = the RF register-shadow write** (0x08024CF8 →
  0x0802BC28(0,v) → shadow[8]=v, RAM-only — NO bus traffic). Do not
  wire it to the LED serial writer again.
- **The boot call graph:** the vendor PIC walker @0x08042900 → FUN_0802D120 →
  main task FUN_0802D178 → FUN_0802D0D0 → FUN_0801B5A8 → the boot scene
  FUN_0801B738 → the 30 ms loop. The r9 = 0x08040000 hook (the PIC static
  base) is REQUIRED at 0x08042900 for the original.
- **The channel scan** 0x0801FDDC `[RECOVERED]` = 128 × ee_read(0x02+0x10k, 2),
  sits between the status text grid and the content fill.
- **The fonts/assets** (reconstruction/data/, incbin via splash_data.S):
  the class-3 8×16 @0x080413CC, the class-1 6×10 @0x08041F00, the class-6
  12×20 @0x0803F490, the splash @file 0x130 (0xA000 B),
  led_stream.bin (4,458 B, replayed [DATA-DERIVED] between the (78,13)
  space cell and the (97,30) 90×10 fill — the 2 swapped CRH pairs fixed
  in session 55), ee359_stream.bin (359 B) + vendor_stream.bin (2,507 B)
  [DATA-DERIVED, session 55].
- **The wire frame ground truth:** buf = {u8 reg; u8 pad; u16 val LE};
  wire = [reg, val_hi, val_lo]; the mode writers emit their own leading standby.
- **The protocol tail commands** ($A10/$A11/$A30/$A31/$D00/$F00/$A50/$A51/
  $E0x/$F0x) = **unreachable in simulation**: their compare strings live at
  0x08044844+, past the distributed file's end; the recon's fall-through is
  behaviorally correct (session 52 pt4).
- **The timing boundary:** the original's LOAD = 7999 (a stale 8 MHz constant)
  → ~13.5 kHz tick at 108 MHz vs the recon's true 1 ms — §7(c)/§3.1, accepted;
  all timing-sensitive comparisons use event sequences, not wall time.
- **The original's I²C per-byte read pattern** = CRH(44 input), CRH(34 output),
  BSRR — 3 events/byte; the read-byte emits 0 PB9 events (SCL = PA11 is
  uncaptured by the PB model). The pre-LCD vendor ops leave the PB9 latch LOW
  (the recon's probe restores SDA low after the read — [DATA-DERIVED] tail in
  main.c); the app ops leave it HIGH.

## 6. Renode survival guide (patterns that work / traps that waste time)

- Run: `cd ~/tools/renode_1.17.0-portable && timeout N ./renode --console
  --disable-xwt <script.resc>`; background + stdin: `(tail -f /dev/null |
  timeout N ./renode ... &)`.
- `emulation RunFor "00:00:00.050"` = run then pause. Do NOT `start` first.
- State reads: `sysbus ReadDoubleWord <addr>`; CPU registers via properties
  (`sysbus.cpu PC`, `SP`, `PRIMASK` — bare `cpu` dumps the API list).
- SysTick VAL monitor reads are unreliable; only 32-bit reads of the STM32_UART
  model are trustworthy (the 16-bit ones return garbage).
- Hooks: `cpu AddHook <addr> "self.Log(LogLevel.Info, '...')"`,
  `AddHookAtInterruptBegin/End` — re-derive addresses with nm after every rebuild.
- GDB software breakpoints never fire (don't retry; hbreak untried).

## 7. Workaround register (restated in full in docs/CK5_REPORT.md §4)

All live ONLY in the harness/platform: (1) the SysTick CVR write skipped at
~10 ms; (2) r9 = 0x08040000 set at 0x08042900; (3) GPIOB = `pb_displaybus.py`
(the IDR reads 0 → the I²C ACK always "ACKed"; the per-write capture IS the
comparison channel); (4) ADC2_SR tagged 0x2 (EOC, bit1); (5) the ADC2_DR
runtime-tag key injection (session 56); (6) SPI2 = `sp2_fontflash.py`;
(7) the 8051 peer = monitor-injected frames; (8) the injection pacing (~0.6 s
virtual time per 534-B frame); (9) the neutral peer content, identical for both
images. Restated in full (incl. the session-55 [DATA-DERIVED] bus replays and
the accepted timing boundary) in `docs/CK5_REPORT.md` §4.

## 8. File map

```
followup.md                # THIS handoff — start here
docs/RESEARCH_LOG.md       # the evidence base — sessions 1-54 (51-54 = display/I2C)
docs/PLAN.md               # the round map with statuses
docs/CK5_REPORT.md         # the consolidated differential report (§3.4 open, §4 workarounds)
reconstruction/            # the C tree (build: §2)
reconstruction/src/eeprom.c  # the corrected I²C primitives + eeprom_read/write
reconstruction/src/display.c # all display primitives + display_led_stream_replay
reconstruction/src/ui.c      # ui_boot_scene, ui_screen0_build, ui_task
reconstruction/sim/ck4-{orig,recon}.resc  # the canonical capture scripts
reconstruction/data/         # splash + 3 fonts + led_stream.bin +
                             # ee359_stream.bin + vendor_stream.bin (+ .incbin sources)
firmware/MAIN-MCU-*.bin      # the original (immutable)
/tmp/opencode/pb_stream_{orig,recon34}.log  # the canonical captures (re-copy after a reboot)
/tmp/opencode/lcd.asm        # the original's full disassembly (§2 regenerates it)
/tmp/opencode/m_ops.py       # the burst op-map decoder (session 55; usage:
                             #   python3 m_ops.py <capture.log> <odr_pos>)
analysis/                    # gitignored dumps/traces
```

## 9. Open items that simulation may resolve (secondary)

- The `$A09` 4-byte-prefix routing mechanics (the tail commands themselves are
  resolved-unreachable).
- Cross-check the recovered `.data`-structure guesses against live behavior.
- The S-meter bar thresholds and the `'M'/'N'` blinker source (a simulated RF
  level sweep).
- The `0x20004E40` radio-state region vs the lazy per-field settings reads
  (CK5 §3.2/§3.3).

Keep the log current, commit at milestones, and good hunting.
