# followup.md — the HG-UV98 reconstruction: simulation & differential validation

## KICKOFF — the prompt to give the agent (paste verbatim; keep this block current)

```
Read followup.md in the repo root and execute it from the top.
§1 = the current state and round map; §3 = THE ACTIVE ROUND with its
ordered open items; §4 = the working rules (three attempts max per
item, then park it with evidence and move on — do not loop). Before
your first change, reproduce §2's two verification recipes and confirm
160,753/160,753 ODR events = 100.00%. Work item by item: change ->
rebuild -> run -> compare -> log -> commit. Log every claim in
docs/RESEARCH_LOG.md (append-only), commit at milestones, push to
main, never stage prompt.md or prompt.md~. When the active round
closes (or is parked), update THIS kickoff block and §3 before moving
to the next round.
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
  keyscan stimulus (Round N).
- **CK3 (AT1846S SPI) ✅** — 292/292 bus events byte-identical, same order
  (session 49).
- **CK4 (display) ✅✅ CLOSED — ★ the FULL 16-bit ODR stream = 100.00%
  byte-identical: 160,753/160,753 events** (session 53 pt2, independently
  re-verified in session 54). The acceptance bar is the FULL 16-bit ODR value —
  earlier rounds masked `& 0xFF` and hid the PB9/SDA-latch bit; **do not regress
  to the masked comparison**. The remaining CK4 delta = the I²C event *counts*
  only (the GPIOB writes that are not ODR events) = **Round M, in progress**.
- **CK5 (the report)** — `docs/CK5_REPORT.md` drafted (session 51), maintained
  under Round L as items close.

**Round map (details + statuses in `docs/PLAN.md`):**
A ✅ | R1 ✅ | D ✅ | K ✅ (tail commands unreachable — platform boundary) |
G ✅ (boot call graph) | H ✅✅ (display 100%) | E ✅ (drivers) | F ✅ (report draft)
| **M — IN PROGRESS (runtime I²C volume)** | N (keyscan/TIM2) | O (tone) |
L (report maintenance, continuous) | B/C (protocol broadening, blocked on N's
stimulus path).

**Repo state:** all work committed & pushed through the session-54 commit
(log = 54 sessions). `reconstruction/` builds green
(`make TOOLCHAIN_PATH=/tmp/opencode/arm-tc/usr/bin/` — the toolchain and Renode
live in /tmp and ~/tools, re-extract after a reboot; recipes in §2).

**Session-54 changes (this commit):** the I²C primitives re-verified against the
disassembly and corrected (start/stop/ack polarity, the LED bit) — the final
step to the 100.00% ODR parity; the CK4 capture scripts committed to `sim/`;
this handoff rewritten. **No numbers changed**: the ODR stream was already
100%; the corrections closed the PB9-latch tail and fixed the future event
accounting (the census is in §3).

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

**The comparison (the ODR-only sequence — the acceptance bar):**
```bash
python3 - <<'EOF'
def odrs(p):
    return [l.split()[2] for l in open(p)
            if len(l.split())>2 and l.split()[1]=='ODR']
o=odrs('/tmp/opencode/pb_stream_orig.log')
r=odrs('/tmp/opencode/pb_stream_recon26.log')
n=min(len(o),len(r)); s=sum(a==b for a,b in zip(o,r))
print(f'{s}/{n} = {100*s/n:.2f}%')
EOF
```
**The event census (the Round M accounting):**
```bash
python3 - <<'EOF'
from collections import Counter
for p in ('/tmp/opencode/pb_stream_orig.log',
          '/tmp/opencode/pb_stream_recon26.log'):
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

## 3. Round M — the runtime I²C event accounting (THE active work)

The display ODR stream is closed; what differs is the **count and placement of
the I²C (non-ODR) GPIOB events**. Current census (recon vs original):
CRH 3,392/4,902 · BSRR 7,432/7,913 · BRR 6,860/14,188 · CRL 0/1.

**The burst map** (segments delimited by CRH direction-switch pairs; positions
in ODR-event index, counts in events; from the canonical captures):

| position | orig | recon | identification |
|---|---|---|---|
| pre-LCD | 2,507 | 63 | the vendor PIC boot region (missed code) |
| @74 (ee_load) | 288 | 288 | ✓ exact |
| @3,545 (0x0FD0 read + ee_save) | 5,881 | 4,198 | the page-write accounting |
| @85,487 | 83 | 741 | the recon's callsign+VFO reads sit here; orig smaller |
| @89,225 (channel scan 0x0801FDDC) | 12,987 | 7,936 | 128 × ee_read(0x02+0x10k,2); ~101 vs 62 events/read |
| @118,308 | 208 | — | orig-only |
| @137,739 | 104 | — | orig-only (write/ACK-poll shape) |
| @145,359 | 104 | — | orig-only (identical to the above) |
| @152,036 (LED stream) | 4,458 | 4,458 | ✓ replayed `[DATA-DERIVED]` (data/led_stream.bin) |
| @153,847 | 25 | ? | small tail burst |
| @158,942 | 359 | ? | orig-only |

**Open items, in the order to attack them** (each has its evidence pointer;
session-53 pt3 banked the raw 104-event burst decode in the log):

1. **The per-read delta (62 vs ~101 events per 2-byte read)** — the biggest
   multiplier (×128 over the scan). Diff `eeprom_read`'s event sequence against
   the original's 2-byte reads in the capture, byte-by-byte: check the ACK-wait/
   retry loop's event emission and the per-bit SDA writes (the original's
   per-byte read pattern = CRH(44 input), CRH(34 output), BSRR = 3 events/byte;
   the read-byte itself emits 0 PB9 events — SCL=PA11 is uncaptured).
2. **The 104-event bursts (@137,739/@145,359)** — match event-by-event against
   FUN_080286C0 (the write path: per byte = start + `do{start;send(0xA0)}while(ack!=1)`
   + hi + lo + data + stop). Identify the call sites via the main-task loop body
   after the routers (0x0800FE58/0x08010302/0x08015DCC) and FUN_0801FDDC's callers.
3. **The save region (4,198 vs 5,881)** — the ee_save page-write accounting
   (the 0x4D write crosses 3 pages).
4. **The 208 (@118,308) and 359 (@158,942) bursts** + the small 25 (@153,847);
   also reconcile @85,487 (the recon emits MORE here: the callsign+VFO reads may
   belong elsewhere in the original).
5. **The pre-LCD vendor burst (63 vs 2,507)** — the vendor PIC region past the
   distributed image (the strings region); possibly a permanent harness
   boundary. Do not sink more than one attempt before classifying it.

**When Round M closes:** update CK5 §3.4/§4, close the round in PLAN.md,
refresh the KICKOFF block at the top of this file, commit, then move to
**Round N** (the keyscan/TIM2: TIM2 CC1 can never match on the
model — recover the original's mechanism, document the model boundary, wire the
key-event stimulus so the `$A01` write-direction observation becomes testable),
then **Round O** (tone FUN_0801A91A on-branch, CTCSS/DTMF dispatch
0x080276D4/0x08027474), with **Round L** maintenance after every closure.

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
- **The I²C primitives (as committed in eeprom.c):**
  start (0x080284F2) = SDA high, SCL high, SDA low, SCL low;
  stop (0x08028532) = SDA low, SCL high, SDA high;
  master_ack (0x08028566) = val==0 → BRR (the ACK = SDA LOW), val≠0 → BSRR
  (the NAK = SDA HIGH); send = MSB-first with the inverted SDA polarity;
  the LED bit=1 → BSRR (PB9 HIGH — not inverted).
- **The boot call graph:** the vendor PIC walker @0x08042900 → FUN_0802D120 →
  main task FUN_0802D178 → FUN_0802D0D0 → FUN_0801B5A8 → the boot scene
  FUN_0801B738 → the 30 ms loop. The r9 = 0x08040000 hook (the PIC static
  base) is REQUIRED at 0x08042900 for the original.
- **The channel scan** 0x0801FDDC `[RECOVERED]` = 128 × ee_read(0x02+0x10k, 2),
  sits between the status text grid and the content fill.
- **The fonts/assets** (reconstruction/data/, incbin via splash_data.S):
  the class-3 8×16 @0x080413CC, the class-1 6×10 @0x08041F00, the class-6
  12×20 @0x0803F490, the splash @file 0x130 (0xA000 B), led_stream.bin
  (4,458 B = 2,966 BRR + 1,488 BSRR + 4 CRH, replayed [DATA-DERIVED] between
  the (78,13) space cell and the (97,30) 90×10 fill).
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
comparison channel); (4) ADC2_SR tagged 0x2; (5) SPI2 = `sp2_fontflash.py`;
(6) the 8051 peer = monitor-injected frames; (7) the injection pacing (~0.6 s
virtual time per 534-B frame); (8) the neutral peer content, identical for both
images.

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
reconstruction/data/         # splash + 3 fonts + led_stream.bin (+ .incbin sources)
firmware/MAIN-MCU-*.bin      # the original (immutable)
/tmp/opencode/pb_stream_{orig,recon26}.log  # the canonical captures (re-copy after a reboot)
/tmp/opencode/lcd.asm        # the original's full disassembly (§2 regenerates it)
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
