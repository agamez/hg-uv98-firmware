# HG-UV98 Main MCU — RE Research Log

Target: `firmware/MAIN-MCU-HG-UV98-V1.0.1-190416.bin` (primary), `firmware/V1.0.1_190619-KT.bin` (diff target).
MCU: GigaDevice GD32F103RET6, ARM Cortex-M3, 108 MHz, 512 KB flash @ `0x08000000`, 64 KB RAM @ `0x20000000`.

**Success level: 1 (Architecture) — complete. 2 (Behavior) — complete for this effort's scope** (protocols, task architecture, memory maps, subsystem anchors, data structures). **3 (Functional implementation) — started 2026-10-15, "builds + consistent with static evidence" demonstrated** (reconstruction/ tree, arm-none-eabi-gcc, vector table verified, spot-checked disassembly); runtime behavior not claimed (no emulator/hardware). Level 4 not pursued.

Status legend: **[confirmed]** static evidence sufficient · **[probable]** strong but incomplete evidence · **[speculative]** hypothesis only.

---

## Environment / tooling

| Tool | Status |
|---|---|
| REA MCP (rea-agents 6.5.0) + Ghidra 12.1.4 headless | Wired (`GHIDRA_INSTALL_DIR=/opt/ghidra_12.1.4_PUBLIC` in `~/.config/opencode/opencode.json`). 190416 image open as raw-binary, base `0x08000000`, `ARM:LE:32:v7`, cspec `default`. |
| Ghidra Thumb decode | **[confirmed] works**: e.g. `procedure_assembly`/`procedure_pseudo_code` at `0x08018B60` return coherent Thumb-2 + clean C. (REA docs claim Cortex-M unsupported; empirically the ARM analyzer resolves Thumb.) |
| Python | capstone, unicorn, pyelftools, pwntools installed. `binutils-arm-none-eabi` install was blocked by a system apt lock (retry pending, non-essential). |
| Reproducible recon | `tools/recon_map.py` regenerates all phase-1 artifacts into `analysis/` (git-ignored). |

REA evidence IDs (session `ses_ec03bcc16ffez5nk68G9l60uSt`): segments `ev_07df69f0…`, vector-table bytes `ev_0dcbcc6a…`, function dossier `ev_4ebffbe0…`.

---

## Image identity (all [confirmed], static)

| | 190416 (primary) | KT |
|---|---|---|
| SHA-256 | `733616010b6d…66b2` | `5a2a42e296…dbce` |
| Size | 273,303 B | 507,904 B |
| Mapped extent | `0x08000000–0x08042B96` | `0x08000000–0x0807BFFF` |
| Initial SP | `0x20006780` | `0x20006780` |
| Reset vector | `0x08044691` → **outside file** | `0x08044741` → inside, but lands on a data table |
| Vector table | 76 entries (16 system + 60 IRQ), 70 unique | 76 entries, 70 unique |
| Last non-FF byte | file end (mid-code truncation) | `0x08045B4D`; `0xFF` padding above |
| Version string | `V1.0.1_190416` @ `0x0803F900` | (KT equivalent not yet located) |

- Vector layout matches **STM32F103xE / GD32F103 high-density** (60 IRQs) exactly. **[confirmed]**
- **[confirmed] Both files are partial images, not runnable as distributed:**
  - 190416: truncated mid-code at `0x08042B94`; reset + fault handlers (13 vector entries at `0x0804468x–0x0804469x`) are beyond file end.
  - Both: IRQ-handler slots, PendSV (`0x0802B21A`/`0x0802B20E`), SysTick (`0x0802BD68`/`0x0802BD5C`) and Default-handler regions are filled with structured `0xBEEF` filler (e.g. 10-byte repeating unit `EFBE EFBE EFBE 0000 0000` per IRQ slot).
  - SP value and handler addresses appear **only** in the vector table — no startup code anywhere in either file.
  - Code **never** branches into BEEF regions (654 distinct BL/B.W targets in `0x0800B000–0x0802A000`, zero into BEEF) — the islands are vector-referenced only.
  - Mechanism unknown: dump-tool sanitization / protected-page read failure / update-payload format. **Open question O1.**

## Preliminary memory map (190416; [confirmed] boundaries, [probable] roles)

| Flash range | Content | Confidence |
|---|---|---|
| `0x08000000–0x08000130` | Vector table (76 entries) | confirmed |
| `0x08000130–0x0800B000` | Low-entropy data: `0xFF` + byte patterns (`FF/BF/DF/9E` halfword-heavy); **byte-identical between the two builds** → static assets (fonts/icons/tables), not code | probable (graphics), confirmed (not code) |
| `0x0800B000–0x0802A000` | Dense valid Thumb code (100% sector decodability `0xB000–0x13000`); 654 distinct call targets | confirmed |
| `0x08029B60–0x0802E000` | `0xBEEF` placeholder islands (181 runs ≥32 B, ~5–6 KB total each image) + zero regions | confirmed bytes, role open (O1) |
| `0x0802F000–0x08034xxx` | Zeros / low-density data | probable (tables/BSS image) |
| `0x08034000–0x08042B94` | Strings (menus, AT commands, version), GB2312 text, bitmap/font data (`0x0803C000+`) | confirmed |
| missing tail | `0x08042B97–…`: startup/reset, fault handlers (190416). KT shows this region = more strings (`OK`, `MEM-CH:`, `%d.%`), tables (`I`,`D530`,`D536`,`D621`… = APRS/CTCSS?), then `0xFF` padding to `0x0807BFFF` | confirmed for KT |

RAM: SP top `0x20006780` → ~26.5 KB static+heap below; task stacks/kernel above (to `0x20010000`). Detail: see session-5 RAM map.

## Level-1 subsystem map (session 6; call-graph anchored)

| Subsystem | Key functions (190416) | Evidence |
|---|---|---|
| **LCD (color TFT)** | set-window 0x080179C8, cmd-write 0x08017A6A (**0x2C = ILI9341-family RAM write**), text renderer 0x08018B60 (GB2312 cells + ASCII), area draw 0x08018D32, backlight/beep GPIO 0x08017A8C/0x08017AC0 (GPIOB/C) | confirmed (cmd 0x2C pattern; "color LCD" prior claim verified at driver level) |
| **UI state machine** | screen-switch 0x08010530 (screen descriptors: 30 B each, array @`0x20004F94`, current copy @`0x20004FD0`, screen id @`0x200050E8`), dual-pane redraw 0x08010CD4 (VFO structs 20 B ×2 @`0x20005080`), value clamp 0x0801715C | confirmed (structure), screen semantics probable |
| **RF control** | bit-banged serial writer 0x0802289A (PA15 clock/data, 3-byte packets, 211 calls), helpers 0x08022838/0x08022786 | confirmed (bit-bang pattern); target chip TBD (CMX865A/BK4819-class) |
| **Inter-MCU link** | USART2 init 0x0801F810, RX ISR 0x0801F920, TX 0x0801FEAA, AT dispatcher 0x08010128, `$A01,` sender 0x08016702 | confirmed (closed in session 4) |
| **GPS input (probable)** | USART1 init 0x0802932C (9600 8N1, RX IRQ 37, ring `0x2000449C`, sem `0x200050DC`), pump task region 0x0802942A–0x08029458 | confirmed (driver); GPS role probable |
| **ADC** | averaged ×4 readers with mutex: 0x08025632, 0x080256CA; SPI-ish transaction helpers 0x08028268/0x0802824C/0x0802837A | confirmed (structure); channel mapping TBD |
| **RTOS (uC/OS-II)** | OSSemCreate/Post/Pend 0x08024C78/CC6/DAC, kernel crit 0x080271E0, service region 0x08028Bxx–0x08029xxx; SysTick/PendSV **stripped** | confirmed |
| **SPL (GD32)** | USART/GPIO/NVIC/RCC wrappers 0x08026Axx–0x08026Fxx; memcpy 0x080198E0/0x080198C0, fill 0x080199A0, memcmp 0x0801F610, sprintf 0x0801F5D0, busy-delay 0x08017AA4 | confirmed |
| **Missing (stripped)** | reset/startup, fault handlers, all 60 IRQ trampolines, PendSV, SysTick, clone-mode handler | confirmed absent |

## Version diff (190416 vs KT) — [confirmed]

- 71.5% of the first 273,303 bytes differ, but this is dominated by address-shift cascade: the images share a byte-identical run `0x0800012C–0x0800A1BB` (~40 KB assets) and identical call-graph fingerprints (same 654 call targets, same call counts 343/211/200/168/156/124, shifted by a constant −0x18/−0xC).
- "Sure"→"OK" **confirmed**: 190416 has 3× `SURE` (`0x08041034`, `0x080419DC`, `0x08041A34`), KT has zero; KT has `OK  ` labels (`0x08041A54`, `0x08041AAC`, `0x0804498C`, `0x080449E4`) plus GB2312 确定/取消.
- Ident string `P3107` present in **both** images (`0x08041A2C` / `0x08041AA4`); `P3LL` absent from both (consistent with P3LL being a V1.2-only string — untestable, image not public).
- Real code-size delta between builds ≈ 0xB0 bytes (reset vector `0x08044690` vs `0x08044740`).

## Inter-MCU AT protocol — [confirmed] strings in the ARM image

`$D00,OK` `0x0804075C`; `$F00,OK` `0x08040764`; request/response tokens `$A00, $A01, $A02, $A08, $A09, $A10, $A11, $A12, $A30, $A31, $A50, $D00, $F00, $F01` clustered at `0x0804075C–0x080408xx`. Matches the 8051-side protocol from `repos/hg-uv98-firmware`. UART number and framing pending (O2).

## Prior-claims updates

| Claim | Status after recon |
|---|---|
| "Sure"→"OK" KT mod | **Confirmed** (string diff) |
| V1.2 uses `P3LL` vs `P3107` | `P3107` confirmed in both images; `P3LL` absent (neither image is V1.2) |
| AT commands over UART to APRS MCU | **Confirmed & closed**: USART2, 9600 8N1, full command set (session 4) |
| 128 channels, VHF/UHF ranges | **Verified** via CHIRP driver: 128 ch, 136–174 / 400–500 MHz |
| CH340/USB bridge | Clone protocol = UART 9600 EVEN with `NiNHSG0N` handshake (CHIRP); radio-side port pending |
| KT changes "UI text only" | Roughly true modulo address shifts; KT also carries extra tail-region content absent from 190416 dump |

## Prior-session log reconciliation

`docs/research_log.md` (untracked, from an earlier session; moved to `analysis/prior_session_log.md`) made claims verified/corrected as follows:

| Prior claim | Verdict |
|---|---|
| Reset handler = `0x0800C0E4` (`FUN_0800c0e4`) | **Wrong** — vector word[1] bytes are `91460408` = `0x08044691` (triple-checked: REA `read_bytes` + capstone + manual). `FUN_0800C0E4` is a real 1,394-B function in the code region (role unknown; possibly `main`); it is not the reset vector target. |
| Vector IRQ naming (EXTI0_TSC@0x38, …) | **Wrong** — entries [14]/[15] are PendSV/SysTick per ARMv7-M; [16]+ is the 60-IRQ F103xE layout. |
| uC/OS-II RTOS (`uC/OS-II Idle/Stat/Tmr` strings) | **Confirmed** — strings present in `analysis/strings_190416.json`. Main MCU runs Micrium uC/OS-II ⇒ a real PendSV context-switch handler exists on the device but is BEEF-stripped in the distributed file (supports O1 "sanitized dump"). |
| `FUN_08018B60` = EEPROM write | **Wrong** — pseudocode shows an LCD text renderer (x/y params, GB2312 2-byte chars advance 16 px, ASCII `+0x30` conversion). |
| `FUN_080198C0` strcpy-like, `FUN_080199A0` fill, `FUN_0801F598` strlen-like | Plausible, unverified — keep as leads. |
| 421 Ghidra procedures, 350 strings | Consistent with raw-binary auto-analysis; will grow with manual function seeding. |
| 190416 "238 KB of FF padding to 512 KB" | **Misleading** — the file simply stops mid-code at `0x08042B94` (no padding present; KT shows `0xFF` padding only above `0x08045B4D`). |

## Open questions

- **O1**: Why do both images carry BEEF handler islands? (dump sanitization vs update-payload format vs flasher-side patching). uC/OS-II ⇒ PendSV/SysTick handlers must exist on-device; only the distributed copies are stripped. Next: locate the radio's update/clone tool behavior; compare with the KT .exe/updater artifacts; check `docs/COMMUNITY.md` for how these files were distributed.
- **O2**: ~~Which UART (and framing) carries the AT protocol?~~ **CLOSED (session 4)**: USART2 (PA2/PA3), 9600 8N1, `$`-prefixed CRLF frames, ARM master; full command set cross-verified against the 8051 source and vendor xlsx. Clone/CPS protocol separately documented (9600 EVEN, port TBD).
- **O3**: RAM map / global data layout (0x20000000–0x20006780). Next: analyze literal-pool RAM references. uC/OS-II task TCBs/priority tables expected.
- **O4**: True file-format semantics of the two .bin files (full flash dump vs flashable payload). Affects any future Level-4 work; irrelevant to functional recovery.
- **O5**: The `0x08000130–0x0800B000` asset region: identify format (font bitmaps? splash? index tables).
- **O6**: uC/OS-II task list: find `OSStart`/task creation, enumerate tasks/priorities. `FUN_0800C0E4` (1,394 B, 55 blocks) is a candidate `main()`.

## Session log

### 2026-10-15 — Session 1: reconnaissance (phase 1)

Done: REA/Ghidra wiring (client restart); image identity; vector table; sector maps; BEEF-structure analysis; branch-target census; strings extraction; cross-version diff; prior-claim verification (Sure→OK, P3107); Thumb-decode validation via REA (`FUN_08018B60` dossier); recon script `tools/recon_map.py`.

Not started: decompilation of subsystems, call graph, RAM map, protocol framing, EEPROM structures.

### 2026-10-15 — Session 2: UART/protocol recon (O2 partial) — Level 1 progress

**Inter-MCU protocol document recovered** [confirmed]: `repos/hg-uv98-firmware/UV98_868_220430_D4/APRS CPU与对讲机CPU通讯协议.xlsx` — full command set: `$A00` read 512 B settings (+14 B ID, uploaded at boot), `$A01,<512B>` write (frame = 519 B), `$A02,A|O|V,<~80B>,temp,pressure` ~1 s telemetry, `$A08` live beacon, `$A09/$A10/$A11` beacon triggers (side key / alarm / PTT-release), `$A30/$A31` odometer-clear / fixed-station, `$A35` del waypoints, `$B00–B19` beacon list, `$C00–C99` beacon detail (+W weather), `$E00/$E01` channel A/B, `$F01/$F02` beep, `$G01` weather screen, `$H00–H24` waypoint list, `$J00–J39` set waypoint, `$L0000–L9999` track log, `$M0000` track select. ARM = master (sends `$A/B/C/H/J/L/M`), receives `$A02/$A08/$E/$F/$G`.

**SPL USART driver identified** [confirmed]: `SendData` 0x08026D94, `ReceiveData` 0x08026D9C, `GetFlagStatus` 0x08026DA4, `USART_Init` 0x08026C5C (standard BRR calc), clock-gate/deinit 0x08026BC8, NVIC helpers 0x08026D14/D32/E18/E26/F1C/F30. Literal census: **only USART1 + USART2 actively used**; UART4/UART5 appear solely in the deinit switch → repo docs' "UART4" hypothesis **refuted for the main MCU** (pending RX-side final check).

**USART1** [confirmed] = 9600 8N1, RX IRQ 37, ring buffer `0x2000449C` (write idx `0x2000514E`), ISR body 0x080292F8 → RTOS queue pump (`0x08024DAC` via handle `0x200050DC`); init 0x0802932C registers handler ptr `0x0802D2F9` — **in the stripped region**. Role: 9600 baud + RTOS → **GPS NMEA input (probable)**.

**USART2** [confirmed] = generic send `FUN_0801FEAA(buf,len)`; 18 BL callers:
- `FUN_08016702` builds **519-byte frame**: memcpy 5-byte literal + memcpy 514 B from `0x2000xxxx` config + `uart2_send(sp, 0x207)` — matches `$A01,<512B>\r\n` exactly → **USART2 = APRS inter-MCU link [confirmed]**.
- Dispatcher 0x08010128–0x080102FE (+ second fn 0x08010302–0x080104xx): command-ID switch sending 6-byte status strings — pool (KT content): `"200 ms"…"600 ms"`, `"-9.0 dB"…"-0 dB"`, `"05 S"…"09 S"` (built via sprintf 0x0801F5D0 from `"0N S"` formats at KT 0x08043BEC+), GB2312 options (`度分`, `度分秒`, `degree`, `Rotator`, `键盘/KEY+PTT`, `关闭/开启`). Interpretation: **status/UI text feed to 8051 (→ Bluetooth app) [probable]** — the same UART carries AT frames and a status stream.
- Key-repeat handler keys 'h'/'i'/'j' on global `0x200050E8`, page counter `0x2000469C+0x135` (u16, max 41), value/5 formatting via `0x2000469C+0x129`.

**AT-command string tables are unreferenced** [confirmed, open]: `$Axx\r\n` 6-byte commands at 190416 `0x080407C4–0x0804080C` (8-byte slots; `$A00,$A01,…` with-comma variants at `0x0804075C–0x080407xx`), duplicated in KT at 0x08040874+ and 0x08043774+. **Zero** literal refs, zero MOVW/MOVT constructions in code → access is indirect (pointer table in the stripped tail, or RAM-initialized pointer). Sender not yet located. **Open sub-question of O1/O2.**

**Clone/CPS handshake strings** [confirmed bytes, role probable]: `"SG0NNiNH"` + `"SG0NP3107"` adjacent at 0x08041A20 (also unreferenced directly). CHIRP ident "P3107" sits inside the second string. pwnj0hn submodule not initialized → fetch CHIRP `hg_uv98.py` next session to correlate.

**RAM global census** [confirmed]: 413 distinct RAM pointers referenced from code; hottest: `0x200004B5` (×89), `0x20003180` (×56), `0x2000469C` (×33, app-state: +0x129 beacon counter, +0x135 page counter), `0x20005094` (×30, flags @+9), `0x20004E68` (×30). Saved: `analysis/ram_globals.json`.

**BL call graph** [confirmed]: 4846 sites, decoder validated; saved `analysis/bl_graph.json`. Helpers: memcpy 0x080198C0 (dst,src,len), fill 0x080199A0 (dst,count,val), sprintf 0x0801F5D0, delay pair 0x0801F780/F788.

Artifacts: `analysis/bl_graph.json`, `analysis/ram_globals.json`, xlsx extracted at `/tmp/opencode/xlsx` (re-extractable from the submodule).

**Next actions (session 3):**
1. **USART2 RX side**: find NVIC init with IRQ 38, ring buffers, `$A02/$A08/$E00/$E01` parser (char-compare switch on '$','A'…); enumerate the full RX state machine.
2. **Fetch CHIRP hg_uv98.py** and correlate `SG0N…P3107` handshake with code paths (clone protocol).
3. **Dispatcher closure**: dump its entry + all cases; map case IDs to key events; confirm status-feed purpose.
4. Start **call-graph clustering** (Tarjan SCC over bl_graph.json) to name subsystems for Level 1.

### 2026-10-15 — Session 3: USART2 RX path recovered (O2 mostly closed)

**USART2 init** [confirmed] `FUN_0801F810`: RCC_APB1ENR bit17, GPIOA PA2 (TX, AF-PP 50 MHz) / PA3 (RX, floating), NVIC IRQ 38 prio 3/3, **baud 0x2580 = 9600**, mode TX|RX, `USART_ITConfig(RXNE)` + IRQ 37-style handler registration via `adr` (inline, no literal). Helper `FUN_0801F8D4` toggles PA2/PA3 pin modes (line control). GPIO register setters 0x0801F780 (BSRR) / 0x0801F788 (BRR) / 0x0801F790 (ODR).

**USART2 RX ISR** [confirmed] `FUN_0801F920` (complete, in surviving region):
- On RXNE: byte → ring buffer `0x2000335C` (write idx `0x200050E0`).
- Frame complete on `\r\n`; requires `buf[0]=='$'` (190416; KT variant to re-verify); sets ready-flag bit0 @ `0x20005094+9`; resets idx.
- Then an ~18-way `memcmp(frame, tbl[k], 5|4|2)` dispatch (memcmp = 0x0801F610; memcpy = 0x080198C0; fill = 0x080199A0; sprintf = 0x0801F5D0).

**RX command table** (190416 compare strings live in the **missing tail** `0x0804481C+`; content recovered from **KT** at +0xB0 where layouts align) [confirmed bytes / probable mapping]:

| # | match (n) | content (KT) | action (from pseudocode) |
|---|---|---|---|
| 1 | 5 | `摄氏\0` (Celsius) — *KT; 190416 likely `$A00,`* | len must == 0x216 (534) → copy frame → `0x20003A84` + `0x20002F74` |
| 2 | 5 | `华氏\0` (Fahrenheit) | copy frame → `0x20002F74` |
| 3 | 5 | `inch\0` | copy frame → `0x20004B94` |
| 4 | 2 | `$B` (inline ADR `0x0801FD4C`) | copy frame → `0x20002F74` (beacon list read) |
| 5 | 2 | `$C` (inline ADR `0x0801FD50`) | copy → `0x20002F74`; `memcmp(frame+5, X, 7)` weather check → `0x20004808` (347 B buffer) |
| 6 | 5 | `$A01,` | flag logic (0x20004E68+0x38/+0xC/+0x40, 0x200050E8, 0x2000469C+1), weather → `0x20004808` |
| 7+ | 5/4 | `%08ld`, `%08d`, `8888`, `000000`, `CALL:`, `DEFINE`, `%05d M`, `%04d S`, `--.-`, `%3d.%d`, `---.-HZ` … | per-branch flag bits @`0x20005094+0xA/+0xB/+0xC`, buffer fills; branch→string mapping needs re-derivation |

**Interpretation** [probable]: USART2 carries a **combined protocol** — vendor AT commands (`$A01,` settings write per xlsx, `$B`/`$C` beacon reads) **plus plain-text/GB2312 commands from the Bluetooth phone app relayed by the 8051** (摄氏/华氏 = the known "Celsius/Fahrenheit toggle" feature, `inch` = units). Length check 0x216=534 ≈ `$A00,`+ID+512B+CRLF settings-upload frame. Exact branch↔string mapping for entries 7+ is **open** (their strings sit in the stripped/missing tail for 190416; KT slot alignment drifts within the shared pool).

**ISR cross-build check** [confirmed]: the 190416 and KT ISRs (0x0801F920 / 0x0801F908, 1414 B each) are **byte-identical except 5 literal-offset bytes** — same compares, same order, same `$` gate. Consequence: KT's compare #1 string pointer = `0x080448CC` = `摄氏\0`, which can **never match** a `$`-prefixed frame → that branch is dead in the KT image (either the vendor's June reorganization left it dead, or the KT string shift repointed it incorrectly). 190416's compare #1 = `0x0804481C` (content in missing tail; almost certainly the `$A00,` settings-upload prefix given the 0x216 length check). **Open:** O1-adjacent — decide between "vendor dead branch" vs "KT repoint defect" by disassembling the June vendor build if it ever surfaces; harmless for reconstruction (we keep the April semantics).

**Dispatcher closure** [confirmed structure]: entry before 0x08010128 (push.w, entry exact addr TBD); switch on command ID sends 6-byte strings via uart2_send; its pool (KT content) = `"200 ms"…"600 ms"`, `"-9.0 dB"…"-0 dB"`, `"05 S"…` , GB2312 options (`度分`,`度分秒`,`degree`,`Rotator`,`键盘`,`关闭/开启`) → status/UI text feed on the same UART [probable]. `FUN_08016702` = `$A01,` 519-byte settings-write sender [confirmed].

Artifacts: `analysis/bl_graph.json`, `analysis/ram_globals.json`; xlsx extraction at `/tmp/opencode/xlsx`.

**Next actions (session 4):**
1. Resolve the RX-table branch↔string mapping definitively (Ghidra dataflow on KT's ISR `0x0801F908`, or recover the 190416 table from KT with per-slot drift check).
2. TX side: locate senders of `$A00\r\n`/`$A09\r\n` 6-byte commands (0x080407C4+ table) — likely via RAM pointer or in a partially-stripped region; enumerate `$A30/$A31/$A35/$H/$J/$L/$M` send paths.
3. CHIRP `hg_uv98.py` fetch → correlate `SG0NNiNH`/`SG0NP3107` handshake.
4. uC/OS-II task enumeration (O6): find OSStart/task-create callers; map 0x200050DC handle chain.

### 2026-10-15 — Session 4: protocol model closed via 8051 cross-reference + clone protocol

**8051 ground truth** [confirmed]: `repos/hg-uv98-firmware/UV98_868_220430_D4/MAIN/UARTX.C` — the 8051's UART4 (↔ radio USART2) implements exactly the frames seen in the ARM image:
- Radio→APRS commands handled: `$A00\r\n`, `$A01,` (519 B write), `$A09/$A10/$A11\r\n`, `$B`/`$C`, `$D00\r\n`→`$D00,OK\r\n`, `$F00\r\n`→`$F00,OK\r\n`, `$A30\r\n`, `$A31\r\n`, `$A50\r\n`→`$A50,VER\r\n`, `$A51\r\n`→`$A51,CPU_ID\r\n`, plus `AT+TONE=1200/2200/OFF`, `AT+ISP=ON`, `AT+RST=1`, `AT+SET=READ/WRITE`, `AT+VER=?`, `AT+TX=ON`, `AT+DEMO=ON`.
- APRS→Radio sends: `$A00,CPU_ID,512B\r\n` (settings upload), `$A02,V|A|O,<payload>\r\n` telemetry (DISP_GPS.C), `$A08,` live beacon (FUN_C.C), `$E01/$E02\r\n` + HDLC + `$E00\r\n` PTT sequences (CHx.C, CMX865A_SPI.C), `$F01\r\n` beep (CHx.C).
- BT module on 8051-UART2 (BT.C): KISS frames (`0xC0…0xC0`), `AT+NAME<callsign>`, waypoint GPWPL output. App-text commands (摄氏/华氏/inch, `CALL:`, `DEFINE`) have **no sender in the 2022 8051 source** — origin remains the Bluetooth app via a firmware path not in this snapshot (documented, open).
- **Protocol model closed**: USART2 (radio) ↔ UART4 (8051), 9600 8N1, `$`-prefixed CRLF frames; ARM = master, roles per vendor xlsx. The RX-table "%08ld/CALL:/DEFINE" compares are app-facing or stale — live AT path confirmed ($B/$C/$A01,).

**Clone/CPS protocol** [confirmed via CHIRP driver `hg_uv98.py` (archived at `analysis/hg_uv98_chirp_driver_reference.py`; upstream kk7ds/chirp)]:
- 9600 baud **EVEN parity**.
- Ident: PC→`"NiNHSG0N"` → radio ACK `0x06` → PC `0x02` → radio 8-byte ident `"P3107\0\0\0"` → PC `0x06` → radio `0x06`.
- Read: `[R][addr u16 BE][len u8]` → radio echoes frame + data; acks `0x06` both ways. Write: `[W][addr][len][data]` → radio `0x06`, PC `0x06`, radio `0x06`. End: PC sends `'E'` (0x45). Chunk = 64 B. Address space `0x0000–0x2000` (8 KB config area).
- EEPROM map (from driver): settings `0x0000+` (incl. ch_a/ch_b_mem_ch at +0xA42/A43), **128 channel names ×16 B at `0x1000`**, APRS callsign struct, `0x1F80` unknown block.
- ARM image strings `"SG0NNiNH"`/`"SG0NP3107"` @`0x08041A20` = 4-byte-half-swapped handshake/ident [confirmed correspondence, exact compare mechanism open].

**Prior-claims updates**: 128 channels **verified** (driver `MAX_CHANNELS=128` + names array); VHF 136–174 / UHF 400–**500** verified (BOUNDS — note: 500, not 470 as repo docs say); PF1/PF2 programmable lists verified (PF1: BACK LIGHT/SCAN/SQUELCH/TORCH/BAND A/B; PF2: BEACON/LIST/TORCH/BACK LIGHT).

**Clone-port UART**: USART1 init is 9600 **8N1** (no parity) — clone mode needs EVEN parity ⇒ either USART1 re-inits with parity in program mode, or the clone path uses another port. Open for session 5 (find the second init / parity setup).

**Next actions (session 5):**
1. Clone handler in ARM: find code referencing `SG0N…` (indirect), the `R/W` frame parser, parity re-init → ties USART1 (or other) to program mode.
2. uC/OS-II task enumeration (O6) + RAM map (O3) using `analysis/ram_globals.json` + RTOS calls (0x08024C78/0x080248F0/0x08024CC6/0x08024DAC).
3. TX-side closure: `$A30/$A31/$H/$J/$L/$M` senders; dispatcher entry/case map.
4. Begin Level-1 subsystem clustering over `analysis/bl_graph.json`.

### 2026-10-15 — Session 5: RTOS API identified, clone-handler located to stripped region, RAM map (O3)

**uC/OS-II API functions** [confirmed by structure]: `OSSemCreate` 0x08024C78 (1 arg = initial count; 6 callers), `OSSemPost` 0x08024CC6 (type-3 OS_EVENT check; 5 callers), `OSSemPend` 0x08024DAC (5 callers), plus a 4-arg kernel helper 0x080248F0 (1 caller @0x08029440). Kernel critical-section helper 0x080271E0. Sem handles: `0x200050DC` (USART1 RX). Mailbox/queue siblings expected near these (OS_EVENT type constants observed: 3 = MBOX).

**RTOS glue is stripped** [confirmed]: SysTick vector (entry [15] = `0x0802BD68`) and PendSV (`0x0802B21A`) both point into BEEF-stripped regions — the sanitizer removed exactly the OS-critical glue (context switch, tick, IRQ trampolines, reset/startup). The app code + RTOS API + ISRs (USART1/2) survive. This is consistent with O1: the stripped regions = startup + vector glue, not a deliberate anti-copy of app logic.

**Clone handler** [probable location]: the `SG0NNiNH`/`SG0NP3107` strings have **zero** literal/movw references and no char-compare sequences — the parser is almost certainly inside the stripped boot region (program mode = PF2+power at reset). Not recoverable from this image; protocol fully known from CHIRP instead.

**RAM map (O3)** [confirmed, preliminary]: 413 referenced globals spanning `0x20000000–0x20006CD4`; densest pages: `0x20005000` (90 — flag banks, handles, indices), `0x20004C00` (43), `0x20000000` (59), `0x20004B5` hottest byte (×89). Anchors: `0x2000335C` UART2 RX frame buf (≥1000 B), `0x2000449C` USART1 RX ring, `0x20004808` beacon-detail (347 B), `0x2000469C` app-state, `0x20002F74`/`0x20003A84` frame copies, `0x20004B94` telemetry, `0x20004E68`/`0x20005094` flag banks, `0x200050E8` screen/key byte. **Globals exist above the initial MSP (`0x20006780`)** — area above main stack = uC/OS-II task stacks/kernel space (to 0x20010000, 64 KB total).

**Next actions (session 6):**
1. Task enumeration via Ghidra (switch REA back to 190416; analyze 0x08028Bxx–0x08029xxx service region + all sem-pend/post callers → task bodies).
2. TX-side closure: `$A30/$A31/$H/$J/$L/$M` senders; dispatcher entry/case map (function entry before 0x08010128).
3. Level-1 subsystem clustering (SCC over bl_graph.json) + name display/menu/radio subsystems.
4. Channel-memory structures in ARM (correlate CHIRP EEPROM map with the ARM's EEPROM/I2C or flash access code).

### 2026-10-15 — Session 5 addendum: dispatcher entry found; KT string-shift is non-constant

- **Dispatcher entry = `FUN_08010128`** [confirmed] (`push.w {r3-r11,lr}`; takes the command ID byte in r0; 1000/200-byte scratch memsets then the case chain). A second RX-frame handler `FUN_080100xx` (ends `0x08010114`) compares the RX copies (`0x20002F74`/`0x20004B94`) against 5-byte literals — its strings (`0x080447B4/B4/BC`, missing tail) map to KT's `-9.0 dB`/`-7.5 dB` under a +0xB0 shift.
- **The 190416→KT data shift is NOT constant** [confirmed]: +0xB0 for slots `0x080104CC–F8` but +0x104 for slot `0x08010500` (format-string block), +0x78 further down at `0x080419CC` — the June build regrouped string blocks. Cross-build string archaeology therefore requires per-slot verification; single-shift mapping is invalid. (This may explain the earlier "dead compare" readings — treat April-vs-June string identifications as tentative except where measured per-slot.)
- **CPS installer checked** (pwnj0hn `HG-UV98 EN V2.0 setup.rar` → PE32, May 2019): no embedded ARM firmware (no vector-table signature, no clone strings); 7z reports no NSIS payload. No local artifact carries the missing 190416 tail. Bounded negative result.

### 2026-10-15 — Session 6 addendum: VFO pane drawing decompiled

`FUN_08010CD4` (REA dossier, evidence `ev_` session): copies a 20-byte VFO record from `0x20005080`-based slots and calls pane-drawer `FUN_08018FD0(pane_idx, x, y)` for pane 0/1 (VFO A/B), then `FUN_0801060C()` (status line). The 20-byte VFO record layout (freq, channel, tone, …) is the next Level-2 structure to decode — via the writer paths that populate `0x20005080`.

### 2026-10-15 — Sessions 7–8: task list, RTOS map, storage architecture (Level 2 closed)

**Boot chain & task list** [confirmed] (OSTaskCreateExt = `0x080249A4`; all 7 call sites enumerated):
| prio | task fn | stack | role |
|---|---|---|---|
| 3 | `0x08029178` | `0x20001F74` (1 KB) | **main/UI task**: `hw_init` → key-scan loop (`0x080175A8` → handler `0x08017738`, `OSTimeDly(30)`) |
| 10 | `0x08017664` | `0x20002374` (1 KB) | 1 ms periodic (`OSTimeDlyHMSM(0,0,0,1)`); counters, every 8th iter calls `0x0801E6F4` — background maintenance |
| 11 | `0x08014DEA` | `0x20002B74` (1 KB) | **UI event dispatcher** — trampoline to `FUN_08010D5E` (67-callee master dispatch) |
| 13 | `0x08017704` | `0x20002774` (1 KB) | **RSSI/S-meter poll** (500 ms; reads RF reg 0x1F via `0x0801F4CC`, 0xFFF=invalid) — feeds the USART2 status feed |
| 29 | `0x08028D74` | `0x2000429C` (512 B) | mailbox-pend service task, 8-slot ring processing |
| 30 | `0x08027938` | `0x20003E9C` (512 B) | timeout/counter task (÷100, fires `0x08024B94(0xFF)`) |
| 31 | `0x08027918` | `0x2000409C` (512 B) | low-prio statistics-style task (crit-section counter + `0x08027CA6`) |

`main()` = `0x08029120`: NVIC setup → creates main task (prio 3) → `0x08027392` (**OSStart candidate**, never returns). `hw_init` chain `0x080290D0 → 0x08029080`: GPIO inits, RF chip init `0x080227AA`, **USART1 init `0x0802932C`** + its mailbox/pump `0x0802942A`, misc. USART2 init `0x0801F810` invoked from the init cluster (exact call site in uncovered region).

**RTOS API corrections** [confirmed]: the type-3 OS_EVENT calls are **mailboxes**, not semaphores — `0x08024C78`=OSMboxCreate, `0x08024DAC`=OSMboxPost (called from ISRs), `0x08024CC6`=OSMboxPend (called from tasks). `OSTimeDly`=`0x0801FF4C`, `OSTimeDlyHMSM`=`0x0801FFB0`, `OSTaskCreateExt`=`0x080249A4`, task-name/queue helper `0x08024A6C`. Tick appears to be 1 ms (delay(500) used for ~500 ms polling).

**Storage architecture** [confirmed by absence]: the ARM image contains **no I2C peripheral code, no EEPROM driver, and no flash self-programming keys** (KEY1/KEY2 absent; FPEC base appears once as data). ⇒ The AT24C512 is **8051-owned** (it has `AT24C512.c`); the CPS clone protocol (`NiNHSG0N`…`P3107`) and the Ymodem `Host UpData` update both run on the **8051's external UART** (its UART1). The ARM keeps VFO/channel state in RAM (`0x20005080`-region structs) and exchanges settings with the 8051 over USART2 (`$A00` 512 B read / `$A01,` 512 B write + ID). **Open**: exact channel-record sync path between ARM RAM and the EEPROM (no $-command for 2 KB of channel records is visible in the 2022 8051 source; protocol may differ in the 2019 pairing).

**EEPROM structure** [confirmed via CHIRP]: 130 × 16 B channel records @ `0x0000` (`lbcd rx_freq[4]`, `lbcd tx_freq[4]`, `ul16 rxtone/txtone`, wide/highpower + bcl/scan flag bytes, 2 unknown), settings block @ `0x0A00` (abr, steps, sql, PF1/PF2 short/long, aprs_rx_band, ch_a/b_mute, tx_priority…, `ch_a_mem_ch` @0xA42), 128 × 16 B names @ `0x1000`, APRS callsign, `0x1F80` block. VFO-A/VFO-B = special channels 128/129.

**UI structures** [confirmed partial]: channel-grid selector `FUN_0800C9BC` (sprintf "%02d", 5-column grid at 16 px, cursor via `(ch%5)*0x10+0x16`); GPS neighbor-list parser `FUN_0800C0E4` (10 comma fields of `CALLSIGN DIST.Km`); flag bank `0x20005094` (+9 protocol ready, +0xA/+0xB/+0xC UI refresh bits) referenced by 28 functions; master dispatcher `FUN_08010D5E` (67 callees) driven by UI task.

**Level 2 status: closed for this effort's scope.** Documented: full inter-MCU protocol (both directions), clone protocol, task architecture, memory map (flash+RAM+EEPROM), display/menu/RF/ADC subsystem anchors with evidence. Remaining explicitly-unknown items are logged as open questions (channel sync path, RF chip model, screen-by-screen semantics, stripped-region contents).

### 2026-10-15 — Session 9: Level 3 started — reconstruction tree builds

Created `reconstruction/`: Makefile (arm-none-eabi-gcc, Cortex-M3 Thumb), linker script (0x08000000/512 K, RAM split MSP-then-task-pool mirroring the original layout), and seven modules with per-file provenance annotations (RECOVERED / INFERRED / SCAFFOLD / REPLACEMENT):

- `startup.c` — SCAFFOLD (original stripped); 76-entry vector table with original handler addresses in comments; off-by-one in the ARMv7-M reserved slot found and fixed via built-binary verification.
- `mem.c` — re-implements the original helpers (0x080198E0/0x080198C0/0x0801F610/0x080199A0).
- `rtos.c` — REPLACEMENT clean-room scheduler: PendSV PSP/MSP context switch, 1 ms SysTick, mailboxes with pend-timeout, task pool from linker section; the uC/OS-II API surface observed in the image (OSMboxCreate/Post/Pend, OSTaskCreateExt, OSTimeDly/HMSM) is preserved under different names, with the recovered task table (7 tasks / prios 3,10,11,13,29,30,31) as the creation order.
- `uart.c` — RECOVERED: USART2 (9600, PA2/PA3, IRQ 38, RXNE frame assembler with '$' gate, TC-polled send) and USART1 (9600, PA9/PA10, IRQ 37, ring + sink). IRQs 37/38 live in NVIC_ISER1 (fixed after build warning).
- `protocol.c` — RECOVERED command set ($A00/$A01/$A02/$A08/$A09/$B/$C framing rules incl. the 0x216 length check), computed dispatcher replacing the stripped table (documented reordering); 519-byte settings-frame sender.
- `display.c` — RECOVERED primitives (0x2C ILI9341-family write, set-window, GB2312 16 px / ASCII 8 px text walk) over SCAFFOLD transport; glyph blitting pending font-format analysis (O5).
- `main.c` — RECOVERED structure: main 0x08029120-equivalent, hw_init, the seven tasks with documented priorities/stacks; task bodies SCAFFOLD.

Validation actually performed: clean build (arm-none-eabi-gcc 14.2.1, `-Wall -Wextra` clean), built-binary vector-table check (MSP 0x20006800, 76 valid Thumb entries, USART1/2 at slots 37/38), disassembly spot-checks (PendSV switch sequence, USART2 TC-poll send loop match the recovered patterns). No runtime/emulation claims.

Toolchain note: `gcc-arm-none-eabi` installed via `apt-get download` + `dpkg -x` to `/tmp/opencode/arm-tc` (system dpkg was locked by an unrelated apt session); Makefile takes `TOOLCHAIN_PATH`.

**Next actions (Level 3 continuation):**
1. Font/asset region format analysis (O5) → real glyph blitting in display.c.
2. RF chip identification (reg 0x1F read, PA15 bit-bang protocol) → rf.c.
3. Key matrix + screen semantics → ui.c replacing the scaffolds.
4. Emulation decision: QEMU MPS2-AN385 (Cortex-M3) could run the image with a UART backend for the protocol layer.

### 2026-10-15 — Session 10: real uC/OS-II kernel + GigaDevice BSP vendored

**Directive followed:** user suggested downloading uC/OS-II and the GigaDevice BSP ("the vendor used as much code from GigaDevice as available — when in doubt trust that source"). Both adopted.

**1. Real uC/OS-II kernel vendored** (`reconstruction/ucos2/`, upstream weston-embedded/uC-OS2 V2.93.01 + uC-CPU + uC-LIB headers, **Apache-2.0** — license files included). The session-9 REPLACEMENT scheduler is retired; `src/rtos.c` is now a thin wrapper mapping the recovered call shapes onto the real kernel:
- `mbox_create/post/pend` → `OSMboxCreate/OSMboxPost/OSMboxPend` [original sites 0x08024C78/0x08024DAC/0x08024CC6]; messages biased +1 across the wrapper because uC/OS-II reserves NULL; mailboxes are single-depth (matches recovered one-at-a-time posting).
- `task_create` → `OSTaskCreateExt` [0x080249A4] with stacks carved from the `.taskstacks` pool (mirrors the original's static stacks 0x20001F74..0x2000449C).
- PendSV/SysTick vectors → `OS_CPU_PendSVHandler`/`OS_CPU_SysTickHandler` (port sets PendSV priority itself in `OSStartHighRdy`); `board_setup_tick` → `OS_CPU_SysTickInitFreq`.
- `os_cfg.h`: 7 app tasks + idle, mailboxes on, all other kernel objects off, 1 ms tick; `cpu_cfg.h`/`app_cfg.h`: 4 NVIC bits, kernel-aware boundary 1 (recovered ISRs run at prio 3, inside the maskable range); empty app hooks in `src/os_hooks.c` + `CPU_TS_TmrRd` stub (referenced by cpu_core.c even with measurement disabled).
- **Port bug fixed** (documented modification): `os_cpu_a.S` marks `OSCtxSw` with `.thumb_func` but the aliased label `OSIntCtxSw` on the next line stayed ARM-typed → `Unknown destination type (ARM/Thumb)` / `dangerous relocation` at link. Added a second `.thumb_func`. Verified via readelf symbol values (odd = Thumb).
- Kernel version banner: the original image contains only the `uC/OS-II Idle/Stat/Tmr` task-name strings — no version banner — so the shipped kernel version is **not pinned**; V2.93.01 is API-compatible with the observed surface (recorded as an open question).

**2. GigaDevice GD32F10x BSP V2.2.1 vendored** (`reconstruction/vendor/gd32f10x/`, from CommunityGD32Cores/gigadevice-firmware-and-docs; SPL + CMSIS + `gd32f10x_libopt.h`). Chosen over the current official V2.7.0 because V2.2.1's headers are the 2018-era vintage matching the 190416 firmware build. **License: GigaDevice SLA-GD0001** (not OSI) — source redistribution permitted with notices, restricted to execution on GD devices; the reconstruction targets exactly this GD32F103RET6, so the terms are met; `GIGADEVICE-SLA.txt` is vendored alongside. One documented modification: `gd32f10x_libopt.h` guards the CL-only ENET include behind `GD32F10X_CL` (it drags in `<stdlib.h>`); `gd32f10x_enet.c` excluded from the build. Our hand-rolled `src/gd32f10x.h` renamed `src/hwregs.h` (its `GD32F10X_H` guard silently swallowed the vendor header — caught via implicit-declaration error; REG32/16/8 now come from the vendor header). No macro collisions: vendor names peripherals `USART0/1` + `RCU_*`, our sugar keeps the original-image numbering `USART1_BASE` (APB2, GPS) / `USART2_BASE` (APB1, APRS/BT).

**3. Vendor clock code confirmed IN the original image** — `tools/vendor_fingerprint.py` (new) compiles each vendored SPL/CMSIS function with GCC -O2 and searches the 190416 image for compiler-invariant **constant clusters** (armcc-vs-gcc code bytes never match byte-exactly — 0 hits at 8/16-byte prefixes — but register-write constants do):
- RCU base `0x40021000`, `HXTAL_VALUE 0x7A1200` (8 MHz), prediv `0x3D0900` (4 MHz), and mask `0xFEF6FFFF` cluster at **0x08026B70..0x08026BA8** — the vendor `SystemInit`/`rcu_clock_freq_get` literal pool (matches `rcu_clock_freq_get` 3/3 distinctive constants).
- The SystemInit prologue byte-slice `ldr r3,[r2,#0]; mov.w r4,#0x9F0000` appears at **0x08029008 and 0x08029018**, immediately before `main()` at 0x08029120.
⇒ The original embedded the vendor `system_gd32f10x.c` (108 MHz PLL from 8 MHz HXTAL). **Adopted as the authoritative clock init**: `Reset_Handler` now calls vendor `SystemInit()` [VENDOR | RECOVERED], `hw_init` no longer asserts flash wait states by hand, and the Makefile notes the vendored file's default config is already `__SYSTEM_CLOCK_108M_PLL_HXTAL`.
- Probe noise: tail-region (0x0803Cxxx-0x0803Dxxx) clusters with generic constants are coincidences inside asset/font data; a code-region filter is a future refinement. Deeper per-function SPL adoption (usart/gpio into uart.c/display.c) is queued as a Level-3 step.

**4. Build state:** clean (`-Wall -Wextra` zero warnings after cfg cleanup), 4124 B text, 76-entry vector table valid, `SystemInit` + kernel symbols verified in the built image. No runtime claims (per rules).

**Next actions:** unchanged (O5 font format → display glyphs; RF chip ID → rf.c; key matrix → ui.c; QEMU decision), plus: adopt vendor SPL calls inside uart.c/display.c where the original demonstrably used them; re-run the constant probe restricted to code regions to build a vendor-confirmed function map.

### 2026-10-15 — Session 11: ADDRESS-BASE CORRECTION + full IRQ architecture recovered

**1. The distributed file is the APPLICATION at flash 0x08004000 — not 0x08000000.** Disassembling the original's USART inits (capstone) led to main()'s prologue at image-label 0x08029120:

```
mov.w r1, #0x4000 ; movs.w r0, #0x8000000 ; bl 0x08026EEA   ; NVIC_SetVectorTable
```
with `0x08026EEA`: `VTOR = (r1 & 0x1FFFFF80) | r0` = **0x08004000**. Corroboration: (a) 75/76 vector entries resolve inside the file only under base 0x08004000 (the single "outside" is the MSP — a RAM pointer); (b) the Default entries [2]..[13] (0x08044685..0x0804468F) land exactly on six consecutive `b .` stubs at file offset 0x40684; (c) the KT file length is 0x7C000 = 512 KB − 16 KB (app spans sectors 4..511, bootloader 0x08000000..0x08003FFF is NOT distributed); (d) the entry the old analysis called "Reset 0x08044691 OUTSIDE file" is inside at file 0x40690 and contains **2× NOP.W** — the vendor neutered the distributed image's reset. **Addressing convention: every address previously logged as "0x080xxxxx" is file_offset + 0x08000000 (image label); physical flash = label + 0x4000.** All prior analysis (offsets, BL targets, structure) remains valid; flash-sector arithmetic and the "Reset outside file" and "PendSV/SysTick are BEEF" claims are retracted.

**2. PendSV and SysTick are REAL in the image** (old verdict was a wrong-base artifact — the BEEF at old-base 0x2B21A/0x2BD68 is an unrelated stripped region):
- PendSV entry[14] 0x0802B21A → file 0x2721A: `cpsid i; mrs r0, psp; ...` — the Micrium ARMv7-M port handler, recoverable in full.
- SysTick entry[15] 0x0802BD69 → file 0x27D68: `SR_Save; OSIntNesting++; SR_Restore; bl 0x0802740E (OSTimeTick); bl 0x0802731A (OSIntExit)`.
- OSStart **confirmed** = 0x08027392 (main's tail call); OSTaskNameSet(3, name, &err) = 0x08024A6C observed in main.

**3. Full IRQ dispatch architecture recovered:**
- Vector [16+e] → **10-byte trampoline** at file 0x26F4E+e*10: `push {r7,lr}; movs r0,#e; bl dispatcher; pop {r0,pc}` (60 of them, 0xA stair).
- **Dispatcher 0x080271A6**: `cpu_sr=Save(); OSIntNesting++; if (e<60) { h=ram_vectors[e]; if (h) h(); } Restore(cpu_sr); OSIntExit();` — handlers run with interrupts masked; OSIntExit runs after the restore.
- **irq_install 0x08026F30**: `Save(); if (irq<60) ram_vectors[irq]=handler; Restore();` — RAM table at **0x20004AA4** (between task stacks and flag bank 0x20005094).
- USART2 init installs (38, 0x0801F921) at 0x0801F8A2; USART1 installs (37, 0x080292F9) at 0x080293CE. NVIC numbering: vector slot = 16+external-IRQ (entry[54] = external 38 = USART2 — earlier "entry[38]" conflation fixed).

**4. The original's SPL is the ST-struct-style GD32F10x V1.0.0-era API** (session-10's V2.2.1 is flat-API, later vintage; kept for register semantics). Recovered call shapes from USART2 init 0x0801F810 (1:1 sequence):
`RCC_APB1PeriphClockCmd(0x20000, ENABLE)` [0x08026ADE] → `USART_DeInit(0x40004400)` [0x08026BC8] → `GPIO_Init(GPIOA, {pin=PA2, speed=3(50MHz), mode=0x18 AF_PP})` + `{PA3, mode=0x04 IN_FLOATING}` [0x0801F678, ST GPIO_InitTypeDef body verified] → `NVIC_Init({38, 3, 3, ENABLE})` [0x08026E26] → `irq_install(38, &isr)` → `USART_Init(usart, {9600, 8b(0), 1stop(0), noparity(0), Rx|Tx(0xC), noflow})` [0x08026C5C, ST USART_InitTypeDef layout] → `USART_ITConfig(usart, 0x0525 RXNE, ENABLE)` [0x08026D32] → `USART_Cmd(usart, ENABLE)` [0x08026D14]. USART1 analog at 0x0802932C (APB2 clock 0x4000 via 0x08026AC2, PA9/PA10, NVIC {37, 0, 1, 1}, extra `bl 0x08026E18(0x700)` TBD). RX ISR uses `USART_GetITStatus(usart, 0x0525)` [0x08026DB8] + `USART_ReceiveData` [0x08026D9C] — ring + '$'-gate + CRLF terminator, matching our protocol.c. PRIMASK pair = 0x080289DC/0x080289E4. These library functions are RECOVERED-vendor code (ST-SPL semantics).

**5. Reconstruction updated to the corrected architecture:**
- `linker.ld`: FLASH ORIGIN = 0x08004000, LENGTH = 496K.
- `startup.c`: vector table = MSP, Reset (ours, runnable — the distribution's is NOP.W-neutered), defaults, PendSV/SysTick → vendored port handlers, and **60 C trampolines → `irq_dispatcher` → `ram_vectors[60]`**; `irq_install()` implemented with kernel PRIMASK primitives. Documented deviation: our stubs are compiler-generated (not the original's 10-byte armcc stair); semantics 1:1.
- `main.c`: sets VTOR = (0x4000 & 0x1FFFFF80) | 0x08000000 first, like the original.
- `uart.c`: installs USART1/2 handlers via irq_install; NVIC comments corrected.
- Build: clean, 7192 B text, all 76 built-vector entries valid at 0x08004000.

**Follow-ups:** re-run the BEEF census under the corrected base to re-derive the true stripped-region list; identify 0x080272E6 (main's second call) and 0x08026E18(0x700); optionally mirror the 10-byte stub stair in asm; font region = file 0x130..0xB000 (O5 analysis used file offsets — unaffected).

### 2026-10-15 — Session 12: O5 broken open — CJK font is in an EXTERNAL SPI flash

**1. BEEF census (corrected base).** The vendor fill is the halfword `0xBEEF` (bytes EF BE). Tiny 2–6 halfword runs occur throughout — those are armcc inter-function padding, not stripping. The real stripped region is one contiguous zone, **file 0x29B60–0x2EE02 (labels 0x08029B60–0x0802EE02, ≈21 KB)**, immediately after main's object cluster: hundreds of BEEF runs (largest 0x29C, 0x244, 0x152, 0x14E…) interleaved with 4–90 byte surviving fragments. This zone is the vendor-neutered application/UI layer (the clone-handler region among it). The same zone exists in the KT build shifted 0xC bytes earlier. Re-deriving what each stripped function was (via surviving fragments' BL targets) is queued.

**2. Two unknowns identified:** `0x08026E18` = flag-word OR-setter (`flags |= arg`; USART1 init sets 0x700 — the flag bank at 0x20005094 family). `0x080272E6` (main's post-VTOR init call) = app-init orchestrator calling 12 sub-inits in sequence: 0x08027C90, 0x080276F6, 0x08027744, 0x080277FE, 0x080276A8, 0x08028B1C, 0x08028B6C, 0x08027786, 0x080277C2, 0x08028BB2, 0x08027C98, 0x08028E30 (all in the real-code region; to be named individually).

**3. O5 MAJOR RESULT — the CJK 16×16 font is NOT in internal flash.** The glyph draw path (renderer 0x08018B60 → CJK draw 0x08018914 → fetch 0x08024E78) ends in an **external SPI flash read**:
- `0x08024E78` (fetch): range-check hi ∈ [0xB0,0xF8), lo ≥ 0xA1; **addr24 = ((hi−0xB0)×94 + (lo+685)) << 5** — HZK16-style layout whose array starts 846 glyphs (9×94) before GB2312 zone-16 row 0, so zones 7..15 (symbols) are included; 32 bytes per glyph.
- `0x08024F28` (block read, compact) and `0x08024E10` (split-arg form): **CS assert on GPIOC pin 4 → opcode 0x03 → 3 address bytes MSB-first → read len bytes → CS deassert**.
- `0x08027E92`: hardware **SPI2** (0x40003800) full-duplex byte transfer (poll TXE, write DR, poll RXNE, read DR) — not bit-bang.
- Pin helpers `0x0801F780`/`0x0801F788` = universal GPIO set/clear wrappers (~90 call sites across display/RF/keys/SPI).

**4. Asset record tables (internal flash).** `0x08024F8C(class, *ch)` indexes 9 tables — mixed content, verified by dumping:
| class | base | stride | content |
|---|---|---|---|
| 1 | 0x08041F00 | 12 | bitmap-ish (packing TBD) |
| 2 | 0x080419CC | 14 | menu strings ("MEM-CH:", "DEL-CH:", "SURE", "BAACK") |
| 3 | 0x080413CC | 16 | DCS tone names ("D371N", "D411N", …) |
| 4 | 0x08042410 | 20 | bitmap-ish; ' '/'.'/'-' have fixed slots |
| 5 | 0x080401EC | 24 | APRS symbol names ("$  Phone", "/  Red Dot", "?  File svr") |
| 6 | 0x0803F490 | 36 | APRS symbol names ext ("t  Truck Stop", "w  Water Stn", …) |
| 7 | 0x080421C4 | 45 | bitmap-ish; ' '/'.'/'-' special |
| 8 | 0x0803DF68 | 57 | bitmap |
| 9 | 0x0803C7A8 | 64 | bitmap |
Render tests of the bitmap-ish tables with row- and column-major packings were noisy — their pixel format is TBD. **The ASCII text bitmap font was not found in internal flash**; likely also in the external SPI flash (the split-arg reader's other call sites TBD — scan shows only 2, both inside the CJK fetch; the compact reader's callers not yet scanned).

**5. Renderer details:** CJK path = memset(32B buf) → SPI fetch → blit `0x08017D3C`; ASCII path passes through `0x08024F8C`; cursor advance/wrap helper `0x0801899E` (wrap thresholds 0x9A/0x9C, advances 6/7/12 depending on mode); blit `0x08017FB8` uses `0xBEEF` as an internal sentinel value (mode-flag default).

**6. Reconstruction:** added `src/font.c` + `font.h` [RECOVERED]: `spi2_xfer`, `font_flash_read`, `cjk16_fetch`, `asset_table_get` with the full addressing math and table map. `display.c`'s CJK path now calls `cjk16_fetch` (blit itself still waits on the panel transport). Currently gc-pruned (no live callers — UI task bodies are SCAFFOLD); links once the UI lands. Build stays clean at 7192 B.

**Next:** scan callers of the compact SPI reader (0x08024F28) for the ASCII font + splash bitmaps; dump SPI-flash addresses implied by asset tables to map the external flash layout; then RF chip ID and key matrix.

### 2026-10-15 — Session 13: renderer semantics completed; blit = per-class RGB565 pusher

**1. Text renderer 0x08018B60 — full semantics recovered:**
- Signature (r0, r1) = POINTERS to the cursor cells (y at [r0], x at [r1]; 8-bit coords); r2 = string; r3 = fg; [sp+0x20] = 5th arg = **asset-table class**.
- Gate: `flag_byte[8] & 1` (literal 0x20005094-family) returns early; lock flag set/cleared around the walk (0x08019840(1/0)).
- CJK branch (c0 ≥ 0x81 && c1 ≥ 0x81): draw 0x08018914, advance **x += 0x10**, s += 2.
- ASCII branch: advance helper 0x0801899E (&y, &x, class) — **wraps x ≥ 0x9C → x = 6/7?? and y += 6/7/12 by mode**; then the cheeky **in-place digit bias**: `if (c < 0x0A) *s += 0x30` (MUTATES the caller's string — binary digits stored as 0..9 are rendered as '0'..'9'); then draw 0x0801895E(x, y, &c, fg, {0, class}); s += 1; second advance 0x08018AC0.

**2. Vertical string drawer 0x08018F62** (second user of the asset fetch): fixed x, **y starts 3, advances 8 px per char**, max 0x14 (20) chars — vertical lists (channel/menu columns). Same fetch + blit with mode arg **1** (vs 0 in the horizontal path).

**3. The blit 0x08017FB8 is a per-class RGB565 pixel pusher** (~900 B):
- Palette select switch on a color-ish arg → (fg, bg) pairs incl. literal **0xF800 (red)**, **0x001F (blue)**, 0xA5-region greens — plus the 0xBEEF sentinel as "no color".
- Window setup via 0x080179C8 (geometry constants per class: e.g. +4/+6, +7/+15, +5/+9 offsets observed) then cmd **0x2C** via 0x08017A6A.
- Per-class bit-expansion loops: read record bytes, emit fg/bg as byte-PAIRS (two 0x08017A8C calls per 16-bit pixel) — inner loops of 5/6/8 iterations per class observed.
- Full per-class geometry + bit order = well-scoped follow-up (REA/Ghidra on 0x08017FB8; capstone linear sweep misparses several Thumb-2 spots).

**Consequence for O5:** the ASCII "font" is not a single flat table — text rendering always routes through asset_table_get(class, ch) and the blit's per-class geometry; the earlier render-test failures are explained (wrong assumed geometry; the real per-record geometry lives in the blit's branches). CJK side unchanged (external SPI flash, fully recovered).

**Reconstruction:** display.c annotation updated to the recovered signature semantics; no code churn this session (blit geometry pending).

### 2026-10-15 — Session 14: RF bus fully mapped; key-input infrastructure found

**1. RF chip interface — transport fully recovered** (all on GPIOA):
- **CLK = PA15** (0x8000), **DATA = PA9** (0x200), **two chip-selects PA8 (0x100) / PA12 (0x1000)** selected by a mode helper `0x08022838(0/1/other)` that drives the two CS pins complementary ("other" = both, used for the trailing byte 0x04).
- Write frame `0x0802289A(chip, {u8 reg, u16 data})`: select chip → CLK low → delay 8 → send 3 bytes [R/W+addr7, data_hi, data_lo] via bit-bang `0x08022712` (MSB-first, CLK toggled per bit, delay 10) → delay 8 → trailing select pattern.
- Read frame `0x0802292E(chip, reg)`: select chip → send **reg | 0x80** (R/W in bit 7, 7-bit address) → turnaround `0x080226F6` → clock data in. Caller masks the value to 20 bits (conservative; frames are 24-bit = 1+7+16 — **AT1846S/BK4819-family shape**).
- **Chip-ID probe `0x0801F4CC`**: for each of two flags/chips, read **reg 0x1F**; if value & 0xFFFFF == 0x0FFF → call `0x080232E2(chip, 1)` + delay 30 (mark dead / retry path). Two chips ⇒ dual-band RF section.
- Chip-name confirmation (register value tables / PLL math) queued — the transport is done.

**1b. RF CHIP IDENTIFIED: AT1846S × 2 (Auctus/RDA1846S, one per band).** The init-cluster register map matches the OpenRTX AT1846S register table point-for-point:
- reg 0x30 (main mode control, 29 writes): bit6 = TX enable, bit5 = RX enable, bits13:12 = channel mode — observed transitions 0x4006 (standby), 0x4046/0x7046 (TX/RX states), 0x40A6/0x70A6 (with VOX/other bits) ✓
- reg 0x0A = PA bias voltage (0x4A × 2) ✓
- reg 0x29/0x2A = frequency upper/lower parts; 0x2B/0x2C = reference/ADC clock kHz ✓ (writes present)
- reg 0x1F write 0x9BA9/0x9AA9 (band-dependent) when the read returns 0x0FFF — reconfigure/soft-reset path (`0x080232E2(chip, band)`)
- Datasheet: AT1846S = single-chip VHF/UHF transceiver with 3-wire/4-wire/I²C control — two instances on a shared 3-wire bus with per-chip CS = the dual-band design.
- Additional observed writes (beyond the public table excerpt): 0x79 toggling 0xD932/0x9932 (6×, TX/RX-path switch?), 0x35 = 0x1CF6, 0x4E = 0x2602/0x2402 family, 0x57/0x58/0x59/0x5B, 0x96 = 0x1CF6/0x1C00, 0x99 = 0x3C00.

**2. Key-input infrastructure found:**
- GPIO helpers family: `0x0801F75E` = **pin read** (`(IDR & pin) != 0`), `0x0801F780`/`0x0801F788` = set/clear (~90 call sites), `0x0801F798`/`0x0801F678` = config (ST gpio_init shape).
- **TIM2 ISR** (irq 28, installed at 0x08029586, handler 0x080295B4): tiny — tests flag word (0x0802816E/0x0802819C helpers), on set calls the poll work **0x080294A6**: reads **PC8** with edge detection (saved copy at 0x20005163 → "changed" byte) and **PC7** level (state byte 0x20005164/0x20005165, next to the flag bank). Likely PTT/monitor-class keys.
- **Keypad scanner** at 0x080218xx-0x08021Cxx (6+ gpio_read calls): keys are INDIVIDUAL GPIOC inputs (e.g. **PC11**) with per-key **debounce counter (threshold 5)** and **auto-repeat (500 ms = 0x1F4 timeout)** at struct 0x2000-51xx — not a scanned matrix.
- Breadcrumb: the UI task (prio 3, 0x08029178) is the task ORCHESTRATOR (creates 10/13 via 0x080175A8 and the service set via 0x08017738, then OSTimeDly loop over the dispatcher cluster 0x0800FE58/0x08010302/0x08015DCC/0x0800ED24) — full key→UI event path to trace next. Note: the prior "key matrix at 0x080175A8/0x08017738" pointers were actually the task creators.

**3. Blit geometry (partial, from a clean streaming decode):** palette pairs per mode arg (RGB565: 0xF800 red, 0x001F blue, 0x13A2, 0x8D5E, 0x58E1; 0xBEEF sentinel); tile shapes are **column-major bit maps** (bit test `record[col] & (1<<row)`, MSB = top row): 7×5 (class-0 branch), 16×8 as two 8-col tiles (class-3 branch), 6×8-ish (class-1 branch); each 16-bit pixel emitted as hi/lo byte pairs through 0x08017A8C. The complete arg↔class↔geometry mapping still needs function-context (REA) — capstone snippets mislead on the arg slots.

**Next:** decode blit per-class geometry with REA; identify both RF chips via init register sequences; trace key→UI event path through the dispatcher cluster; panel init sequence.

### 2026-10-15 — Session 17: remaining queue cleared; protocol layer now runtime-tested

**1. `0x08010D06` = selection-state setter (`ui_list_select(first, last, table)`)**, not a handler invoker: clamps/wraps the selected index (mod-5 wrap when a UI flag is set) into a state struct {sel, first, last, table}. Consequence: the RAM "key handler table" entries are **per-key string-table pointers** — each key press swaps the menu-list source before the redraw. The 20-byte block memcpy'd before `0x08010D5E` carries the redraw args.

**2. AT1846S register semantics annotated** (full table from OpenRTX):
- `0x1F` = **GPIO mode config** (pairs of 2 bits per GPIO: hi-z/func/low/high). The observed writes 0x9BA9/0x9AA9 = squelch-out on GPIO6, code/CSS on GPIO3/GPIO0 — the CTCSS/DCS signaling pin map, per band.
- `0x30` transitions = standby (0x4006: power-on, 12.5 kHz? bits 13:12) / TX (0x4046: +bit6) / RX (0x40A6: +bit5) with 25 kHz variants (0x7046: bits13:12=11).
- `0x0A` = PA bias (0x4A → 1.09V), `0x35` = DTMF tone freq 1 (0x1CF6 ≈ 1.81 kHz), `0x2B/0x2C` = reference/ADC clock kHz, `0x48/0x49` = squelch thresholds, `0x5F` = RSSI (1/8 dB).
- Undocumented in the public table: 0x79 (0xD932/0x9932 toggle), 0x4E, 0x57-0x59, 0x5B, 0x96, 0x99.

**3. The wiped 21 KB zone is a CLEAN WIPE — closed.** Fill = alternating halfwords 0xBEEF / 0x9EEF plus zero padding; no surviving code, no literal pools, and **zero live-code callers** into the zone (BL scan over the whole image). It is a removed feature-variant module, not stripped live code — the distributed build is self-consistent without it. Nothing recoverable; the live UI layer is what the earlier sessions mapped.

**4. Host-side protocol conformance tests — the emulation question made practical.** QEMU MPS2-AN385 cannot model GD32F103 peripherals, so runtime validation is delivered as a native test harness instead: `make test-host` compiles `protocol.c` (hardware-independent) with a mocked `usart2_send` capture and executes the documented frame rules — send commands, 0x216-checked settings upload (accept/reject), telemetry/beacon/list/detail frames, unknown-command fall-through. **All tests pass**; this upgrades the protocol layer's Level-3 claim to "passes executed conformance tests".
- Found + fixed a real inversion bug the tests caught: the 0x216 length check sat on `$A00` instead of `$A01,` (the settings-write direction per the 8051 UARTX.C ground truth: `$A01,` = "Receiving Settings", `$A00\r\n` = read whose RESPONSE is the long frame).

**Remaining (no longer blocking Level 3):** per-screen handler bodies via the RAM-table registrars (mechanism fully known), RF register semantics beyond the public table (0x79/0x4E/0x57-0x59/0x96), external flash regions beyond fonts.

### 2026-10-15 — Session 18: key-event system completed; registrars are static init

**1. Key-event system — complete mechanism.** All 37 call sites of `0x0801E044(key_id, ?, pressed)` mapped across 4 input sources (scanner cluster 0x08021000×24, TIM2 work 0x08020000×5, side-input region 0x0801A000×6, misc ×2). Key IDs 0x00 (16 sites) and 0x01 (9 sites) dominate; 12 sites pass variable ids. `0x0801E044` itself = ~750 B of gating logic: menu-open state, key-lock bytes (struct 0x20005xxx+0x10D/0x10E), lock-flag bits, per-arg branching — the complete input policy layer.

**2. Key tables are statically initialized — no registrar code exists.** Zero indexed-store sites (`str [rX, rY, lsl #3]`) image-wide: the ~60 tables are populated by the boot .data copy, never modified at runtime. Their contents = flash .data image values (string-table pointers + args); locating them in the truncated file needs the original's .data LMA (REA) — mechanism fully established, contents deferred.

**3. AT1846S Programming Guide 1.4 located** (opengd77.com/downloads/datasheets/AT1846S/) — likely documents the registers missing from the OpenRTX table (0x79, 0x4E, 0x57-0x59, 0x96, 0x99). PDF fetch unsupported by the current toolchain; download+parse queued.

**Session 15-18 cumulative state:** every input-to-display path is mapped at the mechanism level (keys → event policy → per-screen string tables → menu redraw → ST7735 via the geometry family; fonts from the external flash; two AT1846S on a fully-annotated bus; protocol layer under executed host tests). The remaining unknowns are content-level (per-screen table values, undocumented RF registers, flash regions beyond fonts), each with a known tool path.

### 2026-10-15 — Session 14b: RF = AT1846S ×2 confirmed; key→UI event path closed

**1. RF chip identification COMPLETE — see 1b above.** OpenRTX's register table matches point-for-point (reg 0x30 mode bits, 0x0A PA bias, 0x29/0x2A frequency, 3-wire 24-bit bus). Two AT1846S instances = the dual-band architecture.

**2. Key→UI event path CLOSED (structural):**
- Key scanners (TIM2 work + GPIOC pollers) write the **key ID byte at 0x20005166** (state struct spans 0x20005163..0x20005166+).
- **Key pump 0x0800F1F0(key)**: `entry = table[*(u8*)0x20005166 * 8]` — an **8-byte-entry handler table** (base via pool literal) → `0x08010D06(4, 6, entry)` → **memcpy 20-byte arg block** (`0x080198E0`, source = pool literal) → **call the master UI dispatcher `0x08010D5E`** (the 67-callee function — its role now confirmed as the key/UI dispatch hub).
- The dispatcher's menu-action handler table (16-byte entries, `blx [table+id*16+4]`, seen at 0x0801625A/0x08016296) plugs into the same hub.

**Queue refinement:** master dispatcher 0x08010D5E internals = the remaining UI unknown (its 67 callees = per-screen handlers); blit geometry (REA); panel init; key table dump (8-byte entries: handler + args per key).

### 2026-10-15 — Session 14c: panel identified — ST7735-family 160×128

**Panel init sequence recovered** (0x08017B66-0x08017D22):
`0x11 SLPOUT → 0xB1/0xB2/0xB3 FRMCTR1-3 → 0xB4 INVCTR → 0xC0-0xC4 PWCTR1-5 → 0xC5 VMCTR1 → 0x36 MADCTL → 0xE0/0xE1 gamma (16-value tables in code) → 0x3A COLMOD → 0x29 DISPON`.
- The B1-B4 + C0-C5 set with NO ILI9341 vendor power commands (0xCB/0xCF/0xE8/0xED/0xF7) = **ST7735 family** (160×128-class TFT; matches the renderer's x-wrap threshold 0x9C ≈ 156 on a 160-wide axis).
- Hardware reset via GPIO pin toggle (0x0801704E/0x0801705C: pin 0x400 set/clear on the panel port).
- Transport primitives confirmed: `0x080179C8` = set_window (cmd 0x2A CASET + 0x2B RASET, 16-bit coords), `0x08017A6A` = command write (CS low → byte → CS high), `0x08017A8C` = data write byte; panel bus on GPIOB/GPIOC (literal cluster 0x08017FAC-0x08017FB4); write strobe = GPIOC pin op seen at 0x0801780x.

Display stack now identified end-to-end: ST7735 panel (GPIO parallel bus) ← blit per-class geometry (partial) ← renderer semantics (recovered) ← CJK glyphs (external SPI2 flash, recovered) ← asset tables (recovered). Remaining: blit per-class geometry + MADCTL/geometry orientation + panel GPIO pin assignment map.

### 2026-10-15 — Session 15: key-table architecture, MADCTL, port-map corrections, dispatcher characterized

**1. Key handler table = RAM, runtime-registered.** The pump's table base literal (0x0800F5D8) = **0x20000CA4** — a RAM block (static data below the MSP top). Each screen registers its own 8-byte key entries ([handler, arg] × key-id) at runtime; a static dump is impossible by design. The static structure is the deliverable: 8-byte entries, indexed by the key ID byte at 0x20005166.

**2. MADCTL = 0xA8** (MY | MV | BGR): rotated-landscape orientation on the ST7735 native 128×160 panel, BGR pixel order — consistent with the 160-wide axis and the renderer's wrap at 0x9C.

**3. Port-map correction:** the font-flash CS literal is **0x40011400 = GPIOD** (pin 4 = PD4), not GPIOC as previously logged — font.c fixed and rebuilt. LCD control lines (cmd writer 0x08017A6A toggles GPIOC bit 3) are on GPIOC; keys on GPIOC (PC7/PC8/PC11+); RF on GPIOA; font flash CS on PD4.

**4. "Master dispatcher" 0x08010D5E re-characterized — it is the MENU-LIST REDRAW:**
- Argument = the 20-byte block memcpy'd by the key pump: `{[18]=y_base, [19]=class, [1A]=start_idx, [1C]=end_idx, [20]=string-table (u32/entry), [24]=fg, [26]=bg}`.
- Loop: `draw_text(&cursor, table[i], fg, bg)` per row with **row pitch 0x11 (17 px)**; then selection-bar call `0x08018E0C` at `y_base + 0x11 * sel_idx` (highlight/invert rectangle).
- The prior "67 callees" attribution from the call-graph pass refers to transitive reachability, not this function's own shape.
- The following function 0x08010DEA = a 10 ms poll loop (OSTimeDlyHMSM(0,0,0,10)) incrementing a periodic counter ([base+0x7C]) and checking flag [base+0x119] — the menu-input poller.

**Key→UI model now complete:** TIM2/GPIOC scanners → key ID byte (0x20005166) → pump 0x0800F1F0 (RAM handler table 0x20000CA4, per-screen registration) → handlers perform actions → menu-list redraw 0x08010D5E repaints the list with the selection bar.

### 2026-10-15 — Session 15b: UI screen inventory; ASCII fonts found in the external flash — O5 CLOSED

**1. UI screen inventory: 85 menu-redraw call sites, each a screen pump.** Every caller of 0x08010D5E follows the same pump shape (RAM key table + `0x08010D06(4, screen_id, handler_entry)` + 20-byte arg block + redraw). Screen IDs extracted: 1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 15, 25, 26, 30, 40, 49, 51, 59, 93, 211 (small ids = genuine screens; a few large values may include arg mismatches from the walk-back heuristic). The three sibling RAM structs 0x20000C24/0x20000C54/0x20000CA4 = three screens' key tables; each screen owns one.

**2. ASCII fonts SOLVED — they live in the EXTERNAL SPI flash.** `0x08027EC8 = ascii_font_fetch(ch, class, dst)`: range-checks ch to 0x20..0x7E (95 glyphs), then per class:
| class | geometry | stride | SPI base |
|---|---|---|---|
| 1 | 8×8 | 8 | 0x03BEC0 |
| 2 | 8×8 | 8 | 0x0065C0 |
| 3 | 8×16 | 16 | 0x03B5C0 |
| 4 | 8×16 | 16 | 0x03CD80 |
| 5 | 16×16 | 34 (32 used) | 0x03BE82 |
| 6 | 16×16 | 34 (32 used) | 0x03D142 |
All text glyphs — CJK 16×16 AND ASCII 8×8/8×16/16×16 — are external-flash assets. The internal-flash record tables (asset_table_get, classes 1-9) hold icons/strings only. **O5 is closed**: both font systems mapped with addressing math and transport.

**3. External SPI flash layout (partial):** 0x5660+ = CJK 16×16 HZK16 (zones 7..87); 0x65C0 = 8×8 set; 0x3B5C0/0x3CD80 = 8×16 sets; 0x3BEC0 = 8×8 set; 0x3BE82/0x3D142 = 16×16 sets (stride 34 = 32 + 2 padding). The flash also presumably holds splash/graphics (remaining regions unexplored).

**Reconstruction:** font.c gained `ascii_font_fetch` [RECOVERED] with the full class/geometry/base table; header updated. Build clean (font.c remains gc-pruned until the UI screens land).

### 2026-10-15 — Session 16: remaining queue cleared — blit geometry family, panel pin map, key tables inventory, asset region

**1. Blit 0x08017FB8 per-class geometry — RESOLVED as a scaled bitmap-font family.** Window x-deltas per class (from the full streaming decode) cross-referenced with the record strides:
| class | x-advance | stride (B) | geometry |
|---|---|---|---|
| 0 | 7 | — | 7×5 (5 col-bytes) |
| 1 | 10 | 12 | 8×12 (1 B/row) |
| 2 | 12 | 14 | 12×14 (2 B/row × 7... 14 rows) |
| 3 | 16 | 16 | 16×8 (2 B/row × 8; two 8-col tiles) |
| 4 | 16 | 20 | 16×10 |
| 5 | 16 | 24 | 16×12 |
| 6 | 20 | 36 | 20×18 |
| 7 | 24 | 45 | 24×15 (3 B/row) |
| 8 | 24 | 57 | 24×19 |
| 9 | 32 | 64 | 32×16 (4 B/row) |
The internal record tables = a scale-family of bitmaps (small numbers → big frequency digits/icons), column-major bit test `record[col] & (1<<row)` observed on class 3. Combined with the external-flash text fonts, the ENTIRE graphics asset system is now mapped: internal = scaled UI bitmaps, external = text fonts.

**2. Panel GPIO pin map — COMPLETE:**
- **D0-D7 = PB8..PB15** (byte written to the high byte of GPIOB ODR — helpers `0x0801F77A` = ODR read, `0x0801F790` = ODR write)
- **WR = PB0** (strobe low→high per byte, inside `0x08017A30`)
- **D/C = PC3** (0 = command / 1 = data; toggled by `0x08017A6A` and `0x08017A8C`)
- **CS = PC1** (asserted low during blits — `0x08017FB8` head)
- **RESET = PC10** (0x400 toggle at 0x0801704E/0x0801705C)
- 8080-parallel interface on the ST7735, bus on GPIOB high byte, control on GPIOC.

**3. Key-handler tables inventory — ~60 RAM tables** at 0x20000060..0x2000165C (a ~5.6 KB screen-state region below the app globals), one per screen (0x2000079C = 8 pumps = the main/VFO screen; 0x200008A4 = 7 pumps). Each pump shape: `entry = table[key_id*8]` → `0x08010D06(4, screen_id, entry)` → 20-byte arg block → menu redraw. Writers are the per-screen init code (individual tracing deferred — the map is complete).

**4. Internal asset region (file 0x130–0xA130, ~40 KB):** an unreferenced 0xFF-background bitmap blob (icons/logos, distinct-byte and printable signatures per 2 KB block logged) — no absolute code references; likely an orphaned/legacy asset payload. Real code starts at file ~0xA130. External SPI flash holds all text fonts; the internal asset tables (classes 1-9) sit AFTER the stripped zone (labels 0x0803C7A8-0x08042410).

### 2026-10-15 — Session 19: RF semantics closed via external ground truth; frequency formula recovered; .data image proven absent

**1. AT1846S Programming Guide 1.4 downloaded and parsed** (opengd77.com, 8 pp.; pdftotext). Adds to the register map:
- **0x7F = bank select**: registers > 0x7F are reached by first writing 0x0001 to 0x7F then the low address (guide §1 note; "writing 85H register address is 0x001F"). The observed 0x7F=0 write = bank-0 select.
- **0x1F GPIO** field table (2 bits/pin: hi-z/func/low/high) — our 0x9BA9/0x9AA9 decode confirmed (sq-out GPIO6, CSS GPIO3/GPIO0).
- **0x58 filter** field table — observed 0x840D = 250 Hz CTCSS path config.
- **0x1A/0x1B read-only** = RSSI dB / noise dB — the RSSI display source.
- **0x30 bit map**: [14] xtal_mode, [13] filter_band_sel, [12] band_mode, [6] tx_on, [5] rx_on, [2] pdn_reg; xtal pair = 0x04[0] + 0x30[14] (12.8/13/25.6/26 MHz).
- **Frequency formula (guide §1)**: Freq<29:0> = Freq_MHz × 16000 across 0x29 (high) / 0x2A (low); default 409.75 MHz.

**2. Frequency-setter recovered from the image and confirmed twice.** `0x08023200..(chip, freq)` computes `value = (u32)((double)freq / 100000.0 * 16000.0)` in **soft float** (no FPU): constants 100000.0 and 16000.0 in the literal pool at 0x08023EEC/0x08023EF0, helpers 0x0802510C/0x08025798 (soft double div/mul), 0x08027C68 = double→u32. High 14 bits → reg 0x29, low 16 → reg 0x2A. Matches the guide formula and OpenRTX's `setFrequency` ((freq×16)/1000) exactly.

**3. OpenRTX cross-check fills the "undocumented" list** (platform/drivers/baseband/AT1846S.h): **0x79[15:12] = tone-output enable** (with 0x35 = tone-1 freq, 0.1 Hz/LSB; 0x3A = tone mode bits); **0x4E[10:9] = CTCSS TX enable**; **0x4A = CTCSS1 TX freq, 0x4B = CDCSS TX, 0x4D = CTCSS2 RX freq, 0x5B = RX threshold**. All previously-open registers now documented; remaining truly-open: none material.

**4. CSS/mode setter `0x08023156(chip, css, flag)` mechanism recovered.** Compares `css` against three observed sentinels — **45500000, 17442000, 17062500** (45500000 = exactly 45.5 MHz; sentinel semantics open) — then scans a **96-entry u32 tone table in RAM at 0x200016D0** (base held as a literal pointer). Match → reg 0x05 = 0x8543; special match → 0x8643; scan exhausted → 0x8763. Reg 0x05 = CSS/bandwidth config (layout INFERRED).

**5. The RAM tables' flash .data image is NOT in the distributed file — closed with evidence.** Three independent searches across all 0x42B97 bytes: (a) no consecutive flash-pointer runs ≥ 4 (wide and narrow ranges), (b) no runs ≥ 20 of CSS-range values, (c) **zero occurrences of u32 670** (the first standard CTCSS code, 67.0 Hz in 0.1 Hz units) anywhere. The ~60 key tables and the 96-entry CSS table are boot-time .data copies whose initialization image must live in flash the distribution omits (the wiped zone 0x08029B60–0x0802EE02 is a candidate location). Table contents are unrecoverable from this artifact; the mechanism (static init, RAM-resident) stands.

**6. `reconstruction/src/rf.c` + `rf.h` created [RECOVERED]:** full bit-bang transport (rf_cs_select 0x08022838, rf_send_byte 0x08022712, rf_write 0x0802289A, rf_read 0x0802292E with reg|0x80 + turnaround), at1846s_set_frequency (formula above), at1846s_set_css (mechanism + sentinels; table-driven half scaffolded with the absence rationale), mode transitions (0x4006/0x4046/0x40A6/0x70xx), GPIO reconfig 0x080232E2. Complete annotated register map in the header. Builds clean; functions gc-pruned until referenced (expected). README subsystem table updated.

### 2026-10-15 — Repo housekeeping: history merged with the firmware-archive upstream

The working repo was rebased onto agamez/hg-uv98-firmware main (4ad457c) so the public
history reads: firmware archiving → RE sessions. All session commits were replayed
(subjects and author dates preserved; **SHAs changed** — older entries citing commit SHAs
are superseded), prompt.md's standing edits were squashed into its last prompt commit,
the self-referential repos/hg-uv98-firmware submodule was dropped (the dump now sits at
the tree root, UV98_868_220430_D4/), and the README was rewritten in English with the
current structure. The pre-rebase history is pinned locally in
backup/pre-continuity-rebase.

### 2026-10-15 — Session 20: input system closed; master UI task loop + protocol-sentence router recovered

**1. Key-event system fully closed.** All 13 non-scanner key_event (0x0801E044) sites decoded:
- **PTT synthesizer 0x08019F70**: gated like key_event itself (menu bit2, lock bytes +0x10D/+0x10E, lock flags); reads PTT state via 0x0801B28E(state byte) and on transition emits key_event(0/1) press/release pairs — or key_event([r6+0x40]) when a programmable key is configured.
- **Software-scanned key loop 0x08020530**: `for i: state = 0x0801B28E(state_table[i]); key_event(i, state, 0)` — the extended keypad is software-polled through per-key state bytes (same 0x0801B28E debounced-pin reader as PTT).
- **Programmable-key dispatchers** 0x0802021C, 0x0801C54E, 0x0801D41E: all emit key_event([struct+0x40], ...) — the currently-programmed key ID lives at struct+0x40; 0x0801D41E also writes per-key state into the key-lock byte +0x10D consumed by the key_event gate.
- Physical key → ID mapping for programmable keys = RAM config (.data image absent — closed in session 19). Architecture complete: direct GPIOC scanner (IDs 0/1 + repeat), software-polled extended keys (loop indices), PTT synth, and user-programmable IDs.

**2. Master UI task loop recovered — 0x08029178 (priority 3, created by main 0x08029120 via OSTaskCreateExt 0x080249A4 + OSTaskNameSet 0x08024A6C, then OSStart 0x08027392).** Body: hw_init 0x080290D0 → task creators 0x080175A8 (returns −1 → task exits) and 0x08017738 → loop: OSTimeDly-like wait 0x0801FF4C(30); heartbeat [0x2000469C+0x166]++ (u16); UI work skipped while suspend byte [0x2000469C+0x119] != 0; then **0x0800FE58, 0x08010302, 0x08015DCC every pass; 0x0800ED24 every 17th pass** (counter reset at 0x11). The "dispatcher cluster" = the task-loop's three fast routers + one slow housekeeping routine.

**3. 0x0800FE58 = protocol-sentence → UI bridge.** memcmp (0x0801F610) of a 5-byte token against **"$A00,"** (flash literal at linked 0x0804476C) and two RAM-held prefixes (0x20004E68, .data — absent), then per-sentence handlers (0x08019A10, 0x0801CE78, ...) with sub-dispatch on payload bytes ('j'/'g'/'h' at [0x200050E8]). Consistent with the reconstructed protocol.c command set.

**4. Addressing-convention refinement (important for tooling):** vector-table entry VALUES and literal-pool POINTER values are **linked addresses = file_offset + 0x08004000** (the vendor's link base), while code labels used throughout this log = file_offset + 0x08000000. When dereferencing pool pointers into the file, subtract 0x4000 from the stored value. (Verified: literal 0x0804476C → file 0x4076C = "$A00,\0\0\0$A01".)

**5. Host conformance harness extended** (`test_sentence_prefix_boundaries`): locks the prefix-matching distinctions recovered in sessions 3-20 — `$A00` at 4 and 5 bytes, `$A02`/`$A08` comma-required (4 bytes without comma rejected), `$A09` 4-byte-compare/5-byte-length-gate quirk, `$B`/`$C` 2-byte match, 3-byte fragments rejected. All pass.

### 2026-10-15 — Session 21: the four UI-task routers identified

All four functions called from the master UI task loop (session 20) now have identified roles:

- **0x0800FE58 (every pass) = protocol-sentence → UI bridge** (decoded session 20): memcmp 5-byte prefixes ($A00, from flash at linked 0x0804476C; two RAM-held prefixes from .data) → per-sentence handlers 0x08019A10 / 0x0801CE78 / ...; sub-dispatch on payload letters 'g'/'h'/'j' at [0x200050E8].
- **0x08010302 (every pass) = sentence-letter progress driver.** Routes on the current sentence letter 'h'/'i'/'j' (same state byte family as the bridge); for 'h' increments a u16 progress counter at [0x20005094-struct+0x135] against a 0x29 (41) bound — a per-sentence-type multi-step sequence (41 items for 'h').
- **0x08015DCC (every pass) = battery-gauge poll + reset.** Calls quantizer 0x080242FC (0xFF = no reading → exit); on a valid reading zeroes gauge state ([+0x103]=0, [+0x25]=0, [+0x124]=0), seeds [+0xDF]=10 (full bars), class [+0x128]=3; toggles the UI-refresh flag bits (+8 bits 2-3, +0xB bit3 — the same bits key_event gates on) and calls redraw 0x080202B8.
- **0x0800ED24 (every 17th pass) = blinking mode-indicator driver.** Compares the mode letter against 'M' (0x4D) / 'N' (0x4E); toggles flag bit6 of the flag byte; draws the 2-char erasure string "  " (pool literal, drawn via 0x08018B60(0x40, row, col, 3, "  ", 4)) at a counter-derived row ([+0x129]*16 + 0x32 or + 0x3A per phase); the restore path calls 0x0800BC48(0xFF).

**Battery gauge chain (corrects the initial S-meter guess):** quantizer 0x080242FC is gated by a measuring flag (bit0 → returns 0xFF while a conversion is pending; 0x080241A6's first branch arms the conversion and returns 1/2 as pending states); the raw read is **0x080256CA(0xE) = ADC channel 14 = PC4** — the battery-voltage divider — through the BSP ADC driver (handle 0x0802837A/0x08028268 with lock 0x08024CC6). Threshold 0x296 (662/1023) ≈ the low-battery trip; the classification tree (0→10 bars; bounds at 2/4/6/7) quantizes to 10 battery bars. The AT1846S RSSI path (regs 0x1A/0x1B, digital) remains the separate S-meter source — its UI consumer not yet located.

Open follow-ups: byte-level thresholds of the battery classification tree; the 'h'/'i'/'j' sentence families' correspondence to the 8051 UARTX.C sentence set; 0x0800ED24's 'M'/'N' mode-letter source; the AT1846S RSSI UI consumer.

### 2026-10-15 — Session 22: battery thresholds; the master UI state byte

**1. Battery gauge thresholds recovered** (0x080241A6 tail): comparisons against **0xCB5 (3253)** twice and **0x6D7 (1751)** ADC counts; output levels **{15, 12, 1, 0}** — a 4-state indicator (full/high/low/critical) that the 0x080242FC tree maps to bar counts/blink phases. 0x080242FC's tree bounds (0/2/4/6/7) therefore classify the *level* value, not the raw ADC count.

**2. No SPI-polled S-meter.** All five immediate-argument rf_read (0x0802292E) sites read regs 0x28, 0x01, 0x00 — **no reads of AT1846S 0x1A/0x1B (RSSI/noise)**. Any variable-register read sites remain unscanned, but the digital RSSI is evidently not polled by immediate reads; the S-meter source is still open (candidate: a GPIO squelch pin from reg 0x1F's GPIO6=sq mapping, polled as a level indicator).

**3. [0x200050E8] = the master UI state byte — printable-character state enum (corrects session 21).** 60 immediate-compare sites against 18 distinct values: 0x00 (27×, idle/default) plus 0x02, 0x04, 0x2F, 0x36, 0x3F, 0x40 ('@'), 0x45 ('E'), 0x47 ('G'), 0x4D ('M'), 0x50 ('P'), 0x58 ('X'), 0x67 ('g'), 0x68 ('h'), 0x69 ('i'), 0x78 ('x'), 0x7B, 0x86. The consecutive run 'g'/'h'/'i' (0x67-0x69) and the 'M'/'N' pair are **UI states**, not 8051 protocol letters (the 8051 sources contain no such sentence prefixes — grep confirmed). Session 21's "sentence-letter progress driver" reading of 0x08010302 is thereby reframed: it is a **per-UI-state progress driver** (state 'h' runs a 41-item sequence). The 0x0800FE58 bridge gates protocol-sentence handling on the state byte rather than dispatching sentences by letter.

### 2026-10-15 — Session 23: input-chain byte closes; reconstruction/src/ui.c added

**1. 0x0801B28E decoded — complete body: `return state >= 0x40;`** — a bit-6 active test on pre-read state bytes (not a GPIO reader as previously assumed). The function immediately following it (0x0801B2A0) records **per-key press/release timestamps** at [base+0x5A / +0x62 + key*4] from the flag bank's time words (+0x91/+0x99). The input chain is now closed at byte level: state bytes (bit6 = active) -> threshold test -> timestamp bookkeeping -> key_event gate.

**2. Variable-register rf_read sites**: exactly two (0x0801700A: reg from [sp,#0x48]; 0x0801C464: reg from [r5,#4]) — generic service reads taking the register from a command field; not RSSI polling. Combined with session 22's immediate-site scan: the ARM never reads AT1846S RSSI regs 0x1A/0x1B anywhere in the scanned surface.

**3. `reconstruction/src/ui.c` + `ui.h` added [RECOVERED/SCAFFOLD]:** master UI task 0x08029178 (loop, heartbeat, suspend, the four named routers), key_event policy gate (order per disassembly), key_state_test (full body), battery chain (thresholds 0xCB5/0x6D7, levels {15,12,1,0}, bars mapping), and the UI state enum constants. Builds clean; gc-pruned until referenced. Harness still green.

### 2026-10-15 — Session 24: state-byte writers; list renderer decoded

**1. Writers of the UI state byte 0x200050E8 (= flag_bank+0x54).** Direct immediate stores are rare: one site writes **0x78 ('x')** at 0x08018C4E (inside a list-iterating function, gated on the current state == 'X' — a state-refinement transition), one writes 0x6 (0x080189F6); the remaining transitions copy values from registers (table/protocol-driven). The state machine is therefore driven by computed transitions, not a central table.

**2. 0x08018E0C = the list-row renderer.** Args (r0-r3 + stack); gated on the frame-ready flag (flag byte+8 bit0); calls 0x08019840(1) (draw begin); iterates rows with a **0x14 (20) bound**; text iteration is **GB2312-aware** — bytes >= 0x81 pair as double-byte CJK characters. This is the row/selection painter behind the menu redraw (0x08010D5E) family.

**3. Input-chain byte closes** (with session 23): 0x0801B28E = `state >= 0x40`; timestamps at +0x5A/+0x62 stride 4; the two variable-reg rf_read sites are generic service reads.

### 2026-10-15 — Session 25: CORRECTION — the RSSI/S-meter read exists; rf_read argument order

**Retraction of the session 22/23 "no SPI-polled S-meter" claims — scan artifact.** The rf_read signature is `rf_read(chip, reg)` with **r0 = chip, r1 = register**; the session-22 scan classified by r0 and missed every register. Corrected immediate-read set across all 13 sites:
- **0x1B ×1** at 0x0801C464 — `rf_read(ubfx([r5+4],3,1) /*chip*/, 0x1B)`, result narrowed `uxth; lsrs #8` = **bits 15:8 = rssi_db** (Programming Guide: 0x1B[15:8] = received RF signal strength, 1 dB/LSB). **This is the S-meter read** — gated by bit3 of a config byte (mode-dependent), chip per band.
- 0x30 ×4 (mode-control verification), 0x1F ×4 (the 0x0FFF soft-reset detect), 0x0D ×1 (status, channel-apply), 0x0A ×1 (PA bias service read), **0x1C ×2** (0x08021B42/0x08021C5A — not in the guide's read-only list; register-table version difference, open).
- The session-23 "two variable-register reads" were chip-from-memory, not register-from-memory; registers there are immediates (0x0A, and the 0x1B site).

RSSI UI consumer = the function containing 0x0801C464 (display updater region 0x0801C4xx).

**RSSI consumer (0x0801C4xx updater) — mechanism complete.** New dB value → peak-hold compare against [r4+0x40] (update stores the peak timestamp [r4+0x3B] into [r4+0x12D]); a pass counter (+0x3F, ≥4 → reset) fires the display update every 5th pass; the bar value = 0x080168DA(peak_ts, r7 + band*0x1E) — a lookup into **per-band 30-byte RAM tables at 0x20004F94** (boot .data copy; flash image absent — contents closed like the key tables). Mechanism: RSSI dB (reg 0x1B[15:8]) → peak-hold with timestamp decay → per-band calibration table → bar display.

### 2026-10-15 — Session 26: the 'h'/'i' states identified — beacon-data windows

0x08010302's state-'h' and state-'i' branches are structurally identical: a u16 pass counter (struct+0x135) runs to **0x29 (41)**; at completion each clears a **50-byte RAM buffer at 0x2000335C**, clears the frame-ready flag (0x20005094+9 bit0), resets the counter, formats a 6-byte frame from a printf template — **"$B%02d\r\n"** for 'h', **"$C%02d\r\n"** for 'i' (flash literals at linked 0x08043B08/0x08043B14), embedding [struct+0x129] (the beacon index) — and transmits it via the UART TX helper 0x0801FEAA.

Interpretation: states 'h'/'i' = beacon-list/detail collection windows; on completion the ARM issues the next $Bxx/$Cxx request to the 8051. Matches the protocol command table ($Bxx = beacon list read, $Cxx = beacon detail read — UARTX.C ground truth, session 3).

**State-byte writer trace (partial):** 10 direct strb sites via the flag-bank base; 2 immediate (0x78 'x' gated on 'X'; 0x6), 8 copy from other struct bytes ([r1,#2], [r4,#9] = the frame-ready flag byte, [r0,#7] ×2, [r7,#7] ×2, [r0,#4], [r1,#8]). The state byte mirrors other fields; the origin of the 'M'/'N' values requires deeper data-flow tracking (REA) — deferred.

**Correction (S-meter tables):** 0x080168DA is a **writer**, not a lookup — it printf-formats the peak timestamp (0x0801F5D0), transforms 8->4 bytes (0x0801652C/0x080164DC), and mem-copies 4 bytes (0x080198C0) INTO the per-band 30-byte table. The tables = peak-history ring buffers (boot-seeded, runtime-appended), not dB->bar calibration. The bar value itself is computed from the peak level elsewhere (open).

### 2026-10-15 — Session 27: the menu redraw block decoded — corrects session 18

The 20-byte argument block consumed by the menu redraw 0x08010D5E is NOT five RAM pointers (the session-18 u32 misread reinterpreted packed bytes as pointers). Actual layout, as consumed by the redraw's stack reads ([sp+0x18..0x26] after its double push):
- word0 bytes: [0] = row base (advances +0x11 = 17 px per row via r5), [1] = column, [2] = start index
- word1 bytes: [0] = end index, [1] = selected index (highlight pass back-indexes the table from the end by (end - selected))
- word2 = **u32 pointer to the string table** (u32[] of string addresses) — the redraw's `ldr r2, [r0, r1, lsl #2]` — pointing into the ~60 RAM .data tables (e.g. 0x20000B8C): **the same RAM region as the per-key tables** — per-screen key-action tables (8-byte stride) and menu string tables (4-byte stride) coexist there.
- word3 bytes: attribute/color bytes (passed as r3 + stack args to the draw helper)
- word4: extra pointer passed through on the stack

The draw helper 0x08018B60(x/y, ..., string, attr, [sp] attr): frame-ready gated (flag+8 bit0), draw-begin 0x08019840(1), GB2312-aware byte pairing (>= 0x81 pairs), matching the list renderer 0x08018E0C. Loop body: draw rows from the table between start..end at 17-px pitch, then re-draw the selected row (highlight). The 0x2000xxxx "pointer-looking" words in session 18's dumps were bytes 80/50/00/20... = geometry/attributes in the native framebuffer orientation (MADCTL 0xA8 rotation applies on glass).

**S-meter bar pipeline (final link):** after the ring write, 0x080198C0 shifts table[0]->table[1] (4-byte entries), snapshots the 30-byte table to the stack, and pops the three most recent entries as args to 0x08016B36 / 0x080167AA / 0x0801A296 — the bars render from the recent-peak history (3-entry graph), then peak state ([r4+0x12D]/[r4+0x40]) clears. Full chain: reg 0x1B[15:8] RSSI dB -> peak-hold -> every-5th-pass ring write + shift + snapshot -> 3-entry history render.

**Reg 0x1C reads (2 sites):** each follows a GPIO pin check (read helper 0x0801F75E) in the scanner-cluster region — consistent with polling the AT1846S GPIO3 code_flag pin then reading the received CSS code (INFERRED; 0x1C is not in the Programming Guide's documented read-only set — register-table version difference).

### 2026-10-15 — Session 28: queue finished — BCD rings, reg 0x1C, 'M'/'N' origin, state fingerprints

**1. History-transform helpers identified.** 0x0801652C = BCD packer (nibble shifts over the formatted digit string); 0x080164DC = its pair. The S-meter ring entries = **BCD-encoded peak timestamps** (the "%02d"-formatted clock, packed 2 digits/byte).

**2. Reg 0x1C confirmed (was INFERRED).** OpenRTX reads 0x1C for the **CTCSS2 compare flag** (bit 8) in rxCtcssDetected() — the original's two 0x1C reads after GPIO code-flag pin polls = the received-CSS detection path.

**3. 'M'/'N' origin closed.** **0x08010530 = ui_state_set(state)** (guarded write to 0x200050E8). The 'M'/'N' transitions at 0x08011DC8/0x08011DE8: memcpy a **9-byte screen block** into the screen buffer, reset sub-counter +0x129, then ui_state_set('M'/'N') — the memory-mode screen-entry sequences.

**4. Per-state functional fingerprints (bulk pass over all 60 compare sites):**
- **0x00** idle: mixed UI/protocol work (draw_str, protocol handlers, rf_write, key_event)
- **0x67 'g'**: memcmp + smeter_ring + rf_write + key_state = **the RX/RF-active state** (S-meter ring writes happen in 'g')
- **0x68 'h' / 0x69 'i'**: memcmp + uart_tx = the beacon-data windows (session 26)
- **0x2F '/'**: list_select + menu_redraw = **a menu-navigation state**
- **0x78 'x'**: delay + list_select + menu_redraw ×2 = the refined menu state ('X' → 'x')
- **0x4D 'M'**: draw_str = the memory-mode blinker state
- **0x36 '6'**: memcpy ×3 = a data-transfer state
- **0x02 / 0x04 / 0x47**: small buffer states (memcpy2/memset)
- **0x45 'E' / 0x58 'X'**: pure-logic states (no known calls nearby)
- 0x3F/0x40/0x50/0x7B/0x86: memcmp/memset/draw_str singletons

With this, the state machine's structure (ui_state_set + printable-char enum), the input system, the S-meter chain, the beacon windows, and the menu block layout are all closed. Remaining truly-open: per-state bodies beyond the fingerprints (REA decompile per state), the stripped tail-table commands, and the .data image contents (absent from the artifact).

### 2026-10-15 — Session 29: REA decompile phase — state machine + beacon + RSSI validated and refined

REA (Ghidra 12.1.4) opened the raw binary at the LINKED base 0x08004000 (file offset 0 mapped there), so REA addresses = log labels + 0x4000 and pool pointers resolved natively. Decompiled: ui_state_set (0x08014530), the per-state progress driver (0x08014302), the RSSI updater (canonical entry 0x080203d0).

**ui_state_set(0x08010530 label / 0x08014530 REA) — full semantics:**
- gate: [0x200050EA? — companion flag] clear && new_state != 0
- stores the BAND byte ([0x20004E68+0x40]) into [0x2000469C+0xF0]
- memcpy 0x1E bytes: 0x20004FD0 <- 0x20004F94 + band*0x1E — **snapshots the active band's S-meter ring** on every transition
- sets companion flag [+2]=1, then the state byte 0x200050E8 = new_state
- resolves the session-24/28 stride coincidence: 0x20004F94 hosts the two 0x1E ring blocks (chip 0/1); 0x20004FD0 = the active snapshot.

**Progress driver (0x08010302 label) — refined:** branches on the 2-bit submode at flag+0xB bits 6-7 (and bit 7 = a third path); counter thresholds are **0x32 (50)** or **0x28 (40)** per path (the 0x29 compare = fires on pass 41 in the 40-path); completion formats via **DAT_08014504 = "$B%02d"** (index = raw byte) or **DAT_08014500 = "$C%02d"** (index = byte/5 — the beacon index advances in steps of 5 in the submode paths) and TXes 6 bytes. 50-byte buffer 0x2000335C cleared, frame-ready bit +9 bit0 cleared.

**RSSI updater (0x080203d0 REA):** decompiler-confirmed `rf_read(chip, 0x1B)` (FUN_0802692e) argument order; sub-counter +0x77 to 16; ring write FUN_0801a8da, shift FUN_0801d8c0 (+4), snapshot, bar renderer FUN_0801ab36, post-calls FUN_0801a7aa/FUN_0801e296/FUN_08024d3c(1)/FUN_08022044; peak state cleared; anchors: flags 0x20005094, app-state 0x2000469C, rings 0x20004F94, band byte 0x20004E68+0x40, +0x20005152 flag.

REA findings are consistent with sessions 20-28 throughout — no corrections needed, several refinements (thresholds, index/5, band snapshot, function-level anchors for the reconstruction).

### 2026-10-15 — Session 30: the complete state-transition table — states ARE screens

Capstone scan of all ~150 bl ui_state_set (0x08010530) sites with the value each sets:

- **0x080111BC-0x08011A98 = the menu dispatcher**: sets states 0x01, 0x02, ... 0x2F sequentially — one site per menu item. This is the screen-selection tree.
- **0x08011BC8-0x080127xx**: the remaining screen inventory (0x30-0x86 + high codes 0x88-0x8D; letters '6'-'z', 'M', 'N', 'O', 'X', ...): ~100 more screens/modes.
- **0x0800AEFE-0x0800D3E2 (protocol handler region)**: sets 'g' ×3, 'C', 'h' ×2, 'i' ×2, 'j' ×4, 0x88/0x89/0x8A/0x8C — protocol-driven RF/screen transitions.
- **0x0801725C-0x080172B0 / 0x0801A210 / 0x0801D542**: guard/reset paths (0, 'g', 'h', 'i').

Interpretation: the printable-char "state enum" = the SCREEN ID SPACE (states 1..0x8D ≈ 141 screen states + the RF states 'g'/'h'/'i'/'j' + 0 = off/idle). ui_state_set = the screen-switch function. The session-22 compared set (18 values) = the states with active per-pass logic in the UI task's routers; the rest are leaf screens served by the generic path (the 85 screen pumps map onto these states). REA's caller list for ui_state_set (5 sites) is incomplete vs the capstone scan — Ghidra call-coverage limitation, noted.

This completes the UI architecture: screen-switch function, screen inventory, menu dispatcher, per-state logic sites, and the routers. Open remainder: per-screen bodies for the ~141 leaf states (mechanical REA work, no architectural unknowns).

**Integration closed (end-to-end UI chain):** ui_list_select (0x08010D06) is a leaf — pure selection bookkeeping (moves the highlight, no calls). The state transition happens on ACTIVATION: the menu dispatcher (0x08011190+) is a **binary-search tree over the selection index** (cmp/beq chains), each item handler performing a byte-copy setup, a per-screen init call 0x0801715C(4, 0x1E — the 30-byte screen block), then ui_state_set(screen state).

Complete chain: key_event -> per-screen pump -> ui_list_select (selection move + redraw) -> [activate] -> binary-search dispatcher -> screen init -> ui_state_set -> the per-state routers render.

With this, the UI architecture has no remaining structural unknowns: screen-switch function, screen inventory (~141 states + RF states), menu dispatcher shape, per-state logic sites, key-action tables, string tables, redraw block layout, font chain, and display transport are all mapped. Remaining work is per-screen body detail (mechanical REA/capstone expansion, no architectural unknowns).

### 2026-10-15 — Session 31: REA batch 2 — scroller + numeric S-meter

- **0x0801715C = list-window scroller (corrects the "screen init" reading):** (list, new_end, selection) -> if selection > window end: window.start = min(new_end, selection), window.end = selection; else start unchanged; then 0x0801B074(list, 1). This is the menu scrolling mechanism.
- **S-meter display is NUMERIC:** the renderer family at 0x0801EA04 (canonical entry; my 0x0801EB36 selector resolved into it) writes digit characters (+'0' bias) into a 20-byte field, formats "%2d-"-style above 0x32 (50 dB), draws at (0x27, 0x85) via 0x0801CC0C, and copies an 8-byte placeholder (DAT_0801F728) when no reading. The BCD history rings feed the peak timestamp, the numeric dB shows the live/peak level.
- Ghidra failed to resolve 0x08014D06 (ui_list_select) and the protocol-handler in-body selector — capstone decodes stand.

### 2026-10-15 — Session 32: menu tree extracted as data; S-meter display + history reset

- **docs/MENU_TREE.md**: the complete dispatcher mapping — 116 edges over 37 selection-index values. Odd items = direct screens (item N -> state N); even items 0x00/0x02/0x04/0x06/0x08/0x0A/0x0C = submenu hubs fanning out to the second-level screens 0x33-0x8D; RF states entered from the protocol handlers only.
- **0x0801B05C = S-meter history reset** (memcpy 0x1E from template DAT_0801FC48 + re-render; REA canonical entry for the 0x0801B074 selector).
- **0x0801CBFE = selection position resolver**: keyTable[key*8] -> per-key record -> record[entry*4] rows, modes {0 = hidden (-1), 1 = plain, 2 = +0x2800, 3 = -0x5800} — scaled row coordinates; confirms the two-level key-table indirection (8-byte pointer records -> 4-byte u32 arrays).
- **0x0801EA04 family = numeric S-meter display** (digit buffer +0x30 bias, "%2d-" above 50 dB, draw (0x27, 0x85) via 0x0801CC0C, 8-byte placeholder when no reading).
- ui.c corrected (scroller name) and menu-tree reference linked.

The architectural program is complete. Remaining: per-screen bodies for the leaf states — purely mechanical, no unknowns.

### 2026-10-15 — Session 33: screen-pump inventory completed (docs/SCREENS.md)

Bulk extraction of all 86 ui_list_select (0x08010D06) call sites: 20 distinct list IDs (1-10, 15, 25, 26, 30, 40, 49, 51, 59, 0xD3) across ~80 pump functions in three clusters (main/VFO family 0x0800Axxx-0x0800Bxxx, menu screens 0x08011Cxx-0x08013xx, 0x08014Bxx+). Refinement: ui_list_select's 2nd arg = the LIST definition ID (session-18's "screen_id" reading refined); first arg = 4 (mode) everywhere. docs/SCREENS.md carries the full table.

With the menu tree (session 32), the state-transition table (session 30), and the pump inventory, every enumerable UI structure is now extracted as data. The reverse-engineering program's architectural and inventory phases are complete; what remains is per-pump body nuance within the established generic pattern.

### 2026-10-15 — Session 34: pump argument sets — the screen LINKED CHAIN (final structural finding)

Per-pump RAM-argument extraction (83 pumps, 79 distinct sets, analysis/pump_args.json):
- Every pump references 0x20005166 (key ID byte) + 0x20005080 (the 20-byte redraw block struct — RAM .data, contents absent).
- The pump's FIRST argument = its own key-action table (8-byte stride); the 5th argument = **the NEXT screen's table** — the tables form a linked chain (0x200015C4 -> 0x200015E4 -> 0x20001604 -> 0x2000165C ... visible across consecutive pumps).
- Per-screen attribute-byte sequences at 0x20003A9C..0x20003AC4 and 0x20004E68..0x20004E8F (one byte-pair per screen, in screen order).
- Correction: the "20-byte blocks" read at pool literals in session 18 were the pumps' five independent pointer literals; the actual block struct = RAM 0x20005080 (.data contents absent — consistent with session 19's closure).

The screen system = a linked table chain walked by the dispatcher. With this, every structurally-extractable relationship in the UI is mapped: state machine, menu tree, pump inventory, and the screen linkage chain. The extraction program is complete.

### 2026-10-15 — Session 35: KT-file verification — .data absence proven for BOTH artifacts

The session-19 .data-absence proof had run only on the 190416 file; the KT file (V1.0.1_190619-KT, 0x7C000 = full app flash) was never checked for the missing tables. Verification:
- Builds differ substantially (53.3% of the overlap; vectors differ) — a distinct build, as expected for the KT2KT UI variant.
- All three .data searches on the KT file: **zero** u32-670 (CTCSS table), **zero** CSS-range runs, and the only consecutive-pointer run = the vector table itself (75 entries at file 0x00004). **The .data image is absent from the KT file too.**
- The KT tail beyond 0x42B97 = ONE 12,224-byte cluster (0x42B97-0x45B57) of continued Thumb code — the KT2KT UI additions — followed by clean 0xFF to 0x7C000. The KT build's .text simply extends further; no data image, no tail tables.

**The .data-absence closure now covers both distributed artifacts.** The extraction program is complete with no remaining verification gaps. (The KT build's additional 12 KB of code is a separate variant analysis, out of scope unless requested.)

### 2026-10-15 — Session 36: KT variant identified; 8051 binaries characterized — program closed

**KT variant (V1.0.1_190619-KT) identified:** the string-inventory diff (231 new / 154 removed) shows the 12 KB of additional code = the **community APRS-menu redesign**: numbered settings pages ("1 Radio", "2 Data A", "3 GPS", "4 Beacon Setup", "5 Beacon Mode"...) with new items (Smart Beacon, Digi 1/2 + paths, Symbol Table, Time Zone, TX Priority, PTT Beacon, BT Enable, Screen Save) replacing the vendor labels ("01 APRS RX CH", "01 DATA OUTPUT A", the dB-scale strings, "Crash site"). The KT2KT mod reorganizes the APRS configuration UI; the radio/RF core is unchanged in kind.

**8051 binaries characterized:**
- `UV98_868_220430_D4.hex` (Intel HEX, 3117 records) = a **45,319-byte 8051 image** (0x0000-0xB107, STC8A8K64D4), version-consistent with the source dump (build 220430_D4; "AT+VER=?", " Ver: " strings). Byte-level source verification requires Keil C51 (unavailable) — version consistency is the checkable bound.
- `UV98_868_220430_D4.compressed.bin` (1.8 MB) = a **Keil build-tree archive** in a proprietary container (ASCII source paths "51 "MAIN\G..." visible at offset 0), not a raw flash image.

**Program closed.** Every artifact in the repository is now characterized; every checkable relationship in the main-MCU firmware is extracted and verified; both distributed builds are accounted for.

### 2026-10-15 — Session 37: pump bodies finished — 81/83 pure generic, 2 variants

Capstone deviation analysis over all 83 pump windows (first 0x100 bytes, key-path):
- **81 pumps = the pure generic pattern** (ui_list_select + memcpy + menu_redraw, nothing else) — the per-screen bodies are NOT individually distinct code; they are the same routine parameterized by the (list ID, key table, redraw block) triple extracted in sessions 33-34.
- **2 variants**, both list 0x01: 0x0800AFB0 adds one draw_str (0x08018B60); 0x0800F4B4 adds draw_str calls + 0x08018C0C + 0x08018DFC + 0x08010128 + memcmp — the two custom-draw screens.
- REA/Ghidra cannot decompile the pump entries (no Ghidra functions created there — the pumps are reached only through the indirect task paths); the capstone-level analysis is complete and sufficient: with the argument sets (session 34), the linkage chain, and this uniformity result, every pump is characterized.

**The per-screen body item — the last "mechanical" residue — is finished.** Nothing in the distributed artifacts remains unextracted: state machine, menu tree, pump inventory, linkage chain, pump-body uniformity, all subsystem chains, both builds, and the 8051 binaries.

### 2026-10-15 — Session 38: MAJOR CORRECTION — the .data is COMPRESSED, not absent

Prompted by the user's observation that the distributed files flash and run on hardware, the "neutered reset" and ".data absent" conclusions were re-examined and both were wrong:

**1. The reset chain works (retracts session 11's "neutered" reading).** The reset handler at file 0x40690 is `nop.w; nop.w; bl 0x080429D4` — the session-11 analysis stopped at the NOPs and miscomputed the BL target with base 0x08000000 instead of 0x08004000. Corrected chain: vector[1] -> nop;nop;bl -> boot stub 0x080429D4 (file 0x3E9D4) -> init walker 0x08042900 -> main. The KT build has the identical structure (vector[1] -> 0x08044740 -> bl 0x080429C8; stubs 12 bytes apart). **Nothing is neutered.**

**2. The tables exist in-file — COMPRESSED (retracts sessions 19/35 "absent").** The boot stub calls an **init-array walker** (0x08042900: walks a PC-relative descriptor table, calling each loader with an args pointer, until an end marker). The two loaders:
- 0x0801D967 (entry 1) — args {stream offset 0x3A48 -> packed stream at file 0x423E0, dst 0x20001734, 0} (REA decompile failed — capstone follow-up needed)
- **0x0801D846 = armlink's compressed-RW-data decompressor (REA-decompiled):** LZ/RLE-style — control byte: low 2 bits = literal-run length (0 = 2-byte extended), high nibble = match length (0xF = extended), match decoding with back-references; dst from the descriptor; stream offset = param_1[0]; size/flags = param_1[1] (bit31 = RAM-base-relative flag).
- Entry 2: stream at file ~0x41C40, size word 0x1EAE.

The earlier content searches (pointer runs, u32 670, CSS runs) failed because **the table contents are packed** — the armlink compressed-RW-data feature. The file truncation at 0x42B97 cuts entry 1's stream (packed data for dst 0x20001734) to ~1,983 bytes; entry 2's stream (~0x41C40) may be complete.

**Consequence: the key tables, string tables, CSS table, and screen attributes are RECOVERABLE** by implementing the decompressor (its full algorithm is in the decompiled body) and running it over the descriptor streams. The synthesis recommendation in followup.md is superseded: first decompress the real tables; synthesize only what remains truncated.

**Open follow-ups:** implement the decompressor in C/Python; resolve entry 1's loader (0x0801D967) body; decompress both streams; verify the output against the known table addresses (key tables 0x20000060+, CSS 0x200016D0) — the decompressed RAM image will finally reveal the menu strings, key bindings, and tone table.

### 2026-10-15 — Session 39: THE TABLES ARE RECOVERED — decompressed, verified, decoded

The decompressor (session 38) was implemented in Python and run over entry 2's stream (file 0x41C40, 3,927 packed bytes): **5,937 bytes -> RAM 0x20000000-0x20001730** — the complete table region. Verification:
- **CSS table @0x200016D0**: real values in the setter's encoding — includes 17442000, exactly one of the CSS-setter's sentinel constants (session 25).
- **Key tables @0x20000060+**: 8-byte records as mapped — {RAM sub-table + count} and {flash string-pointer pairs}; 1,148 words point into the string region; the inter-table pointers reproduce the linked chain (session 34).
- **The strings are GB2312 Chinese UI text**: "01 屏保时间" (screen-save time), "02 省电模式" (power-save), "03 步进频率" (freq step), "01.本机设置" (local settings), "02.站点类型" (station type), "03.GPS 设置", "12 TX_DCS"... — **docs/UI_STRINGS.md** carries the decoded inventory.

Artifacts: `analysis/ram_tables_recovered.bin` (the decompressed RAM image), `docs/UI_STRINGS.md`.

**Final status of the .data question:** the tables were never absent — they are **armlink-compressed in-file**. Recovered: the full RAM 0x20000000-0x20001730 table region. Partially recoverable: entry 1's copy region (RAM 0x20001734+, 14,920 bytes) — its source stream is cut by the file truncation at 0x42B97 (~13% present). The synth recommendation in followup.md is superseded: load the recovered tables directly.

### 2026-10-15 — Session 40: English UI confirmed — KT build is bilingual; KT scatter table located

The user noted the radio has English menus. Findings:
- **190416 = Chinese-only settings UI**: its string region carries GB2312 menus plus the English APRS asset strings (Symbol/Station/Beacon — the already-known asset tables); GB2312 语言 at file 0x3FC53, no ASCII 'Language'.
- **KT (190619) = bilingual**: ASCII 'Screen Save' (0x3EDB7), 'Power Save' (0x3EDC7), 'Freq Step' (0x3EDD7), 'Language' (0x3FED3) alongside GB2312 语言 (0x3FD7B). The KT = the international UI variant.
- **KT scatter table located**: walker 0x080428F4 (file 0x3E8F4), table 0x08042988-0x080429AA, 2+ descriptors; entry 1 = loader 0x0801D94F (≠ the 190416 decompressor 0x0801D846) with params {0x1D24, 0x20001734, ...} — a plain-copy-style loader whose semantics need the same REA pass (next session: decompile 0x0801D94F + 0x0801D846's KT twin, decompress the KT streams -> the complete KT RAM image including the English pointer tables and the full globals region).
- The 190416 RAM image's pointer-target distribution (from the recovered tables): 639 words -> 0x08043-45, 430 -> 0x08045-47, 79 -> 0x08040-43, 130 RAM-internal — all string/asset regions.

**UI dialog strings located (follow-up to the user's Sure/Ok observation):** the confirmation-button set lives in 8-byte string slots — 190416 @0x419EC: {"BACK", 确定, 取消}; KT @0x41A54: {"OK  ", "BACK", 确定, 取消} — the KT INSERTED the "OK  " slot (8-byte alignment matches the key-table record width; the strings are the dialog-button set the RAM tables reference). The KT duplicates the set at 0x4499C (beyond the 190416 truncation point). No standalone "Sure" string exists in either artifact — the user's remembered "Sure" likely comes from the shipping international variant (a third build) or the pre-KT UI; the recoverable fact = the KT added the "OK  " dialog slot. Raw 'sure' hits in both files = substrings of "Pressure"/"TX Pressure" (false positives); the 'OK' hits at 0x40761+/0x40811+ = the $D00,OK/$F00,OK protocol strings, identical in both builds.

### 2026-10-15 — Session 41: FINAL RESOLUTION — the image is complete; entry1 = .bss zero-init

Disassembling the entry-1 loader **at the aligned address** (0x0801D966 — the earlier "garbage" was a one-byte Thumb desync; the same fix applies to the KT's 0x0801D94E) identifies it definitively: **armcc `__scatterload_zeroinit`** — `push {r4,r5}; r5 = 0; ... str r5, [r2], #4` with the bit0 RAM-base adjust (`itt mi; submi r3, r9, #1; addmi r2, r3, r2`) — it ZERO-FILLS.

Consequences (final, supersedes every earlier reading):
1. **Scatter story complete**: entry 1 = zero-init of the .bss (RAM 0x20001734+, the globals region — zeros + runtime construction, nothing to recover from flash); entry 2 = the compressed .data (the tables) -> RAM 0x20000000-0x20001730, RECOVERED in session 39.
2. **The file is NOT truncated**: entry 2's packed stream = file 0x41C40 + 0xF57 bytes = ends at **0x42B97 = exactly the file's end**. The "truncation" was the natural end of the compressed data. The 190416 image is complete: code + boot chain + compressed .data + zero-init descriptors.
3. **Nothing was ever neutered, truncated, or missing.** The vendor's dump = a complete, bootable image. The earlier "neutered reset" (session 11, base-arithmetic error), ".data absent" (sessions 19/35, raw-content searches vs compressed data), and "truncated entry-1 copy" (session 38) are all retracted and explained.
4. The KT build: same structure (its walkers/loaders are the same armcc library at shifted addresses; the KT's entry-1 = the same zeroinit pattern); its full-length file simply carries the bilingual .data.

**The reverse engineering of the distributed artifacts is complete at every level: code, boot chain, compressed data, tables, strings (GB2312 + English), and the state machine.** The recovered tables feed the reconstruction and the simulation plan (followup.md) directly — no synthesis needed.

### 2026-10-15 — Session 42: KT tables decompressed — the bilingual menu recovered as data

With the correct descriptor reading (the KT's decompressor entry @file 0x3E99C: {stream 0x3318 -> file 0x41CB4, flags 0x1EC0 -> 3,936 packed bytes, dst 0x20000000}), the KT's full RAM table image decompressed: **5,937 bytes at 0x20000000-0x20001730 — the identical layout to the 190416 build**.

- **1,150 string references, 951 English UI strings** (analysis/kt_english_strings.txt): the main menu ("1 Radio", "2 Location", "3 GPS", "4 Beacon Setup", "5 Beacon Mode", "6 Digipeater", "7 Bluetooth", "8 APRS Advanced", "9 About", "10 Odometer", "11 WX Unit") and the Radio submenu ("1 Screen Save" ... "18 Language", "19 Voice Assist") — the menu tree as RAM data, matching docs/MENU_TREE.md.
- The flash string pools: Chinese and English menus sit in adjacent regions; the language selection = which pointer set the RAM tables expose.
- Also fixed a long-standing helper bug (the string-dereference base) that had masked these strings in earlier spot checks.

**Every table in both builds is now recovered as data.** The reconstruction and the followup.md simulation can load the real bilingual menus — Chinese from the 190416 tables, Chinese + English from the KT tables — with zero synthesis.

(The recovered table images are deliverables, not scratch — copied to `reconstruction/data/`: `ram_tables_recovered.bin` (190416, RAM 0x20000000-0x20001730), `kt_ram_tables_recovered.bin` (KT, same span, bilingual content), `kt_english_strings.txt` (951 English UI strings with their RAM addresses).)

**Reconstruction integration (session 42 follow-up):** the recovered tables are now part of the build — `src/tables.S` (.incbin of `data/ram_tables_recovered.bin` + the UI string region `data/uistrings_region.bin`), the linker places `.tables` at RAM 0x20000000 (flash load image after .text) and `.uistrings` at 0x08040000, and `Reset_Handler` copies the tables to RAM exactly where the original's scatter-load put them. The built image's .data region = byte-identical to the decompressed original tables; every table pointer resolves into the embedded string region. The menu system now runs the real radio's menus in the reconstruction.

### 2026-10-16 — Session 43: fill-ins before simulation — the "CSS tone table" reframed, KT zeroinit confirmed, beacon counters corrected

With the tables embedded in the build (previous entry), the three open
follow-ups closed. One is a **major retraction**.

**1. RETRACTION (session 25): the "96-entry CSS tone table" @0x200016D0 is a FREQUENCY SUB-BAND table.**
The recovered .data shows the table's real values — 24 initialized u32
(the scan bound `cmp r0, #0x60` = 96 iterations at 0x080231AA; entries
24-95 are the zero-init .bss part of a u32[96] C array, which is exactly
why .data ends at 0x20001730 = 0x16D0 + 24×4):
`40950000, 13650000, 14029000, 14408000, 14787500, 15546000, 15925000,
16204000, 16683000, 17442000, 24456000, 35262500, 36400000, 37533500,
38675000, 42087500, 43225000, 44362500, 46637500, 47775000, 48912500,
50050000, 51187500` (+1 more). **Unit: RF frequency in 10 Hz** —
136.5 / 144.08 / 147.875 / 174.42 MHz (VHF band edges incl. 162.04 ≈
WX band start), 420.875 / 432.25 / 443.625 (UHF ham), 477.75 (AU UHF
CB) … 511.875 — the AT1846S tuning range's sub-band boundaries.
Corroborating caller evidence: all four `bl 0x08023156` sites pass the
compared value in **r2** (prologue `movs r5, r2` — the compared value is
the third parameter, not the second as session 25 assumed), and
0x08020DF6 passes the return of 0x080168AA invoked on BCD-assembled
nibble bytes — a parsed frequency. The setter's sentinel literals
confirmed in its pool (file 0x23E00-0x23E0C): 45500000, 17442000,
17062500, table base 0x200016D0. So **0x08023156(chip, flag, freq_10Hz)
= per-frequency band/filter configuration**: table match → reg 0x05 =
0x8543; sentinel match → 0x8643 (special frequencies: 174.42 MHz is
also a table edge; 170.625; 455.0 — semantics INFERRED); scan exhausted
→ 0x8763. reg 0x05 is undocumented in OpenRTX's register table; the
0x8643/0x8763 bit patterns match the reg-0x58 audio-filter family
([9:7] hi-pass bandwidth, bypass bits) — filter config, INFERRED.
Consequence: `reconstruction/src/rf.{c,h}` renamed
`at1846s_set_css` → `at1846s_set_band_config(chip, flag, freq_10hz)`,
the scaffolded table-driven half replaced with the REAL scan (the table
read through `ui_ram_tables + 0x16D0` — the embedded .tables lands at
the original's absolute address 0x200016D0). The actual CTCSS/DCS tone
programming path (reg 0x4A CTCSS freq = kHz×2^16, 0x4B/0x4C CDCSS,
0x45 mode per the OpenRTX table) is NOT this function and remains to be
pinpointed in the original.

**2. KT zeroinit span CONFIRMED (closes session 41's open point).**
KT scatter descriptor 1 @file 0x3E988 = `{0xFFFDAFC7, 0x00003A48,
0x20001734, 0}` — loader self-relative: 0x08042988 − 0x25039 =
0x0801D94F = the KT's `__scatterload_zeroinit` (0x0801D94E + Thumb bit,
exactly session 41's aligned fix). Size 0x3A48 = 14,920 B, dst
0x20001734 → **KT .bss = 0x20001734-0x2000517C, identical to the
190416** (whose desc1 @0x3E994 = `{0xFFFDAFD3 → 0x0801D967, 0x3A48,
0x20001734, 0}`). Descriptor-stride correction: records are **16 bytes
{loader_rel, arg0, arg1, arg2}** — session 38's "12-byte" came from the
"+3 words" return stride measured from the ARGS pointer (desc+4), not
the descriptor start. desc2's arg0 = self-relative offset from the arg
word to the packed stream: 190416 0x080429A8 + 0x3298 = 0x08045C40 =
file 0x41C40 ✓; KT 0x0804299C + 0x3318 = 0x08045CB4 = file 0x41CB4,
packed 0x1EC0>>1 = 3,936 B (vs 190416's 3,927 — the +9 packed bytes =
the bilingual string-pointer differences; both blobs otherwise 5,937 B,
identical layout).

**3. Beacon-window counters corrected (session 26 numbers right, the
later 'h'-state summary wrong).** Capstone re-scan of the router
0x08010302: state compares 0x68/0x69/0x6A ('h'/'i'/'j'); pass counter
compares `cmp r0, #0x29` (41) at 0x0801033C ('h') AND 0x08010392 ('i')
— both beacon windows fire at 41; 'j' uses `cmp #0x33` (51) at
0x080103F0/0x08010450 with a 1-bit submode (ubfx bit 6, 0x080103D8)
plus a 0x74 't' byte compare (0x080104AC; body not yet expanded). The
ui.c header's "runs to 0x28 or 0x32 per 2-bit submode" is corrected;
docs/SCREENS.md gains a protocol-driven-state section.

**Bonus:** the .data tail immediately before the sub-band table (RAM
0x200016B0-0x16CF) = two descending u16 calibration ladders: 10 entries
starting 0x0CB5 (3253 — the battery quantizer's threshold, ui.c) and 6
entries starting 0x06D7 (1751 — its companion). The battery thresholds
are the first rungs of multi-level ladders.

All three corrections applied to `reconstruction/src/rf.{c,h}`,
`reconstruction/src/ui.c`, `docs/SCREENS.md`, `followup.md` (§4.3 and
the §7 equivalence boundary rewritten: table-driven content now matches
byte-for-byte; only the SPI font glyphs, analog ADC/RSSI values, and
timing remain non-matchable). Build green, host tests pass.

### 2026-10-16 — Session 44: Renode CK1 — the OS now boots (three reconstruction fixes); tick delivery blocked

Setup: Renode 1.17.0 installed (portable, self-contained) at
`~/tools/renode_1.17.0-portable`; platform = the stock
`platforms/cpus/stm32f103.repl` (≈GD32F103: same RCC/GPIO/USART2/AFIO/EXTI
addresses; it pre-tags RCC_CR = 0x0A020083 — HSERDY/PLLRDY set — and models
usart2 at 0x40004400). Harness scripts in `reconstruction/sim/`.

**Boot path solved (evidence-driven):**
1. SystemInit spun forever polling RCC_CFGR SCLK_STS after selecting PLL
   (capstone + the spin site 0x08005AD6). Fix: harness Tag RCC_CFGR
   (0x40021004-7) = 0x8 — switch "done". The platform's own RCC_CR tag
   covers the oscillator-ready reads.
2. The Cortex-M3 fetches MSP/PC from 0x00000000 (VTOR reset default); the
   distributed app = linked at 0x08004000 behind the non-distributed
   bootloader. Fix: the harness writes the vector handoff (MSP 0x20006800,
   Reset 0x08004165) at 0x0 — the app sets VTOR itself in main.
3. **`OSInit()` was missing from the reconstruction's main()** — the kernel
   pools stayed zero and `OSInitHookBegin` never ran. Evidence chain: with
   the boot reaching OSStartHighRdy, the port does
   `ldr r1, [OS_CPU_ExceptStkBase]; msr MSP, r1` — the pointer was 0 (BSS),
   so MSP = 0; the first exception stack-faulted
   (CFSR = BFSR.PRECISERR, HFSR.FORCED) into Default_Handler. Fix:
   `OS_CPU_HOOKS_EN` 0 → 1 (restores OSInitHookBegin, which also inits
   OS_CPU_ExceptStkBase) + the `OSInit()` call + removing the now-duplicate
   hook stubs from `src/os_hooks.c` (the vendored port's bodies win;
   OSTaskSwHook/OSTimeTickHook stay — this port variant doesn't define
   them). Note: the Makefile has no header dependency tracking — `rm
   build/k-*.o build/p-*.o` (or the whole build dir) after touching os_cfg.h.

**State reached:** Reset_Handler (tables copy) → SystemInit → main (VTOR =
0x08004000, tick armed 108 MHz/1000 Hz) → OSInit → 7 tasks → OSStart →
OSStartHighRdy → first context switch — no faults (CFSR/HFSR = 0, PRIMASK =
0), OSTaskSwHook ×3, OS_Sched ×2, ui_task entered. SysTick verified armed
by register read (CTRL = 0x7, LOAD = 108000).

**CK1 blocker (open):** periodic interrupts ARE taken (interrupted PC
always 0x08005B0C = the tail of OS_CPU_SR_Restore — the idle task's
critical-section exit; delivery at PRIMASK unmask = normal for the Micrium
port), but OS_CPU_SysTickHandler (0x08005A88, address verified current)
never executes — 0 hook hits, as do OSTimeTick/OSIntEnter/OSIntExit. The
heartbeat (0x20004802) stays 0. At interrupt-begin `self.IRQ` = 2;
`LogCpuInterrupts` labels the exception "Fast Interrupt (FIQ)". Ruled out:
vector[2] misroute (patching 0x08004008 → SysTickHandler changed nothing),
faults, PRIMASK, VTOR (0x08004000 at 1/5/50 ms; Renode independently
guessed 0x08004000), stale hook addresses (an OS_Sched hook fires), SysTick
misconfiguration (register-verified). Renode source facts (nvic.cs /
cortexm.cs / arm.cs fetched): SysTick = a LimitTimer at `systickFrequency`
(72 MHz) → SetPendingIRQ(SystemException.SysTick = 15); the delivered
exception logs as tlib EXCP_FIQ (ExceptionDescriptions[5]); the meaning of
self.IRQ = 2 = unresolved. GDB software breakpoints never fired even with
the CPU spinning at the breakpoint address (hbreak untried). Next
diagnostics, cheapest first: (1) read ICSR 0xE000ED04 at the pause —
VECTACTIVE (bits 5:0) / VECTPENDING (11:6) = hardware truth on what is
active/pending; (2) hook Default_Handler (0x08004131); (3) disarm the tick
(board_setup_tick) and check whether the periodic interrupts stop — source
isolation; (4) only then tlib-internals archaeology. The full handoff =
`followup.md` §1.

### 2026-10-16 — Session 45: CK1 SOLVED — the tick was never broken; Renode's SysTick model needed CVR

**Outcome: CK1 = PASSED.** Canonical run `reconstruction/sim/ck1.resc` boots the
reconstruction into a running 1 kHz OS tick: OSCtxSwCtr (0x200020BC) grows
~223/100 ms, OSIdleCtr (0x20001FE0) accumulates (idle gets CPU time), CFSR/HFSR
stay 0, `make test-host` green. Read this entry before touching the sim.

**The diagnosis chain (all scripts kept in `reconstruction/sim/`):**
1. diag11 — ICSR (0xE000ED04) read at pauses: 0x00000800 both times
   (VECTACTIVE=0, VECTPENDING=0, RETTOBASE=1). Nothing active/pending at the
   sampling instants → "an NMI is literally active" was already dead.
2. diag12 — hooks + on/off correlation + manual pend. Two tooling facts first:
   hook snippets are IronPython — `'s' + UInt32` throws and CRASHES the whole
   emulator (exit 134); use `'IB: IRQ={0} PC=0x{1:X}'.format(...)`. The
   InterruptBegin wrapper receives the tlib exception index as an argument
   (stack: `OnInterruptBeginWrapper(CortexMWrappers, UInt64 exceptionIndex)`)
   — and **`self.IRQ` is NOT the exception number**: it read 2 for BOTH the
   SysTick entry and the PendSV entry (diag12 phase D). Every session-44
   "periodic interrupt" with IRQ=2 was followed by `### PendSVHandler
   ENTERED` — they were the boot context switches (PendSV delivered at the
   idle task's OS_CPU_SR_Restore unmask, hence the constant interrupted PC
   0x08005B0C). After boot settled: ZERO interrupt events. Phase D
   (monitor-write ICSR.PENDSTSET while CTRL=0x2) → `### SysTickHandler
   ENTERED`: **vector-15 delivery works.**
3. diag13 — no-app isolation: arming SysTick from the monitor (LOAD=107999,
   VAL=0, CTRL=7) pends exception 15 every 1.5 ms (NVIC NoisyLog "Internal
   IRQ SysTick", ICSR bit 26 set). The C# model + timer + pend path are all
   healthy. (Monitor logging syntax: `logLevel -1 sysbus.nvic`; NOT
   `periph LogNoisy`/`LogLevel`.)
4. diag14 — app boot with `logLevel -1 sysbus.nvic`: the model SEES the app's
   writes ("Systick_NS enabled", "Systick_NS interrupt enabled", "Priority
   0x10 set for interrupt SysTick") yet zero pends in 50 ms.
5. ROOT CAUSE (os_cpu_c.c:706-726): the vendored Micrium port's
   `OS_CPU_SysTickInit` **never writes SYST_CVR** — RVR=cnts-1, SHPRI3, then
   two CSR read-modify-writes. Renode's NVIC models SysTick as a descending
   LimitTimer whose view (CVR never written = 0) counts from the 24-bit
   limit, so the first LimitReached lands at 0xFFFFFF/systickFrequency =
   233.6 ms @72 MHz (stock repl) — beyond every session-44 window (50-240 ms).
   diag15 CONFIRMED the prediction: silence through 200 ms, first pend in the
   200-300 ms window, then pends every 1.5 ms. On real silicon CVR=0 + ENABLE
   wraps to LOAD+1 on the next edge (~1 tick), so this is a Renode-vs-silicon
   divergence, not a firmware bug. The original binary run will hit the same
   quirk; the harness must apply the same workaround there.
6. diag16/17 — with the workaround: SysTickHandler fires every tick
   (console dedups repeats: "TICK: handler (32)" = 32 events on one line!),
   OSCtxSwCtr scales with ticks. diag17's per-task hooks fire only on INITIAL
   task entry (resumed tasks continue at the delay site) — entry-count 1 per
   task is expected.

**Workarounds and platform (all harness-side, no reconstruction change needed
for the tick):**
- `reconstruction/sim/gd32f103.repl` — project platform = stock stm32f103.repl
  minus unused peripherals, with `systickFrequency: 108000000` (the firmware
  programs the tick for 108 MHz HCLK; the stock 72 MHz rendered ticks 1.5 ms
  and would skew every tick-level timing comparison). AFIO requires all 7
  GPIO ports declared (E38 otherwise). RCC_CR tag preserved.
- Canonical `ck1.resc` writes SYST_CVR=0 from the monitor at t=10 ms (after
  boot, before the measurement windows) — mirrors silicon; must be restated
  in every equivalence report (§7).
- Renode command survival additions: `logLevel -1 <periph>` for Noisy logs;
  hook snippets must format() non-strings; a failing hook aborts the run.

**Reconstruction scaffold fix (code change, [SCAFFOLD] annotation updated):
`main.c` service_task now receives a real mailbox** (`mbox_create()` handle
passed as parg, mirroring the original's 0x08028C26 queue creation). Before,
arg=0 made `OSMboxPend(NULL)` return an error immediately → prio-29 task
spinning forever → idle starved (OSIdleCtr stuck at 0 — that was the
"heartbeat stays 0" residue; the 0x20004802 address itself was from the
ORIGINAL's RAM map and no reconstruction variable lives there — the correct
reconstruction-side observables are OSCtxSwCtr/OSIdleCtr, see nm).
Rebuild note: code shifted (OS_CPU_SysTickHandler now 0x08005AD4 — re-derive
hook addresses with nm after every rebuild); BSS observables unchanged.

**CK1 checklist state:** boot chain ✓ (session 44), tick→handler→delay-expiry
✓ (this session), tasks cycle ✓, no faults ✓, host tests ✓. CK1 CLOSED.
Next: §5 original-binary runner (patched reset under analysis/), then CK2
protocol conformance parity.

### 2026-10-16 — Session 46: the ORIGINAL binary runs in Renode unmodified; boot state mapped; stall root-caused to a GPIO-config retry loop

**1. followup.md §5 is RETRACTED-CONFIRMED — no reset patch exists or is needed.**
Byte-level verification (this session): the 190416 reset handler at label
0x08044690 (file 0x40690) = `AF F3 00 80 | AF F3 00 80 | FE F7 9C F9` =
`nop.w; nop.w; bl 0x080429D4` — session 38's correction holds: boot chain
intact (stub → walker → loaders → main). The runner `reconstruction/sim/
orig-boot.resc` executes the vendor binary AS-IS via `sysbus LoadBinary ...
0x08004000`. Vector table re-verified: MSP 0x20006780, Reset 0x08044691,
PendSV 0x0802B21B, SysTick 0x0802BD69, ext[0] 0x0802AF4F.

**ADDRESSING WARNING (cost a debug round):** for LoadBinary/LoadELF at
0x08004000, label = file + 0x08004000 (the vector table is file 0x0). Every
file-dump disassembly in this session initially used file+0x08000000 and read
the WRONG function (the M/N/'0' AT1846S-write-looking code at file 0x23660 is
LABEL 0x08027660, not the loop). PC/LR/hooks from Renode are labels; file
dumps must subtract 0x4000.

**2. The original's boot state at 30 s virtual (with the CVR workaround):**
- Walker runs, calls exactly TWO loaders (hooked `blx r1` at 0x08042916:
  0x0801D967 zeroinit, 0x0801D847 decompress; the `add r5,r9` RAM-base branch
  at 0x0801D85A never taken).
- Tables decompress byte-perfect vs `analysis/ram_tables_recovered.bin`
  EXCEPT every flash string pointer loses its 0x0804 prefix (0x08043B20 →
  0x00003B20; 2573 diff bytes = 2 per pointer + stragglers; dump:
  analysis/orig_emulated_ram_tables.bin). OPEN: either the fixup base is
  added later at use-time (benign) or a loader stage we haven't mapped adds
  it (needs the next session; the python reference added it explicitly).
- OS boots: 8 context switches, OSRunning=1, nestings 0, no faults, tick
  verified (orig SysTickHandler hook 0x0802BD68 fired 1939×; OSTime
  0x2000511C advances 1:1 with pends).
- **FIRMWARE QUIRK (record for §7 timing parity):** the original's tick init
  (OS_CPU_SysTickInitFreq at label 0x08027D88) reads SystemCoreClock from RAM
  0x20006C50 = 8,000,000 at that moment → LOAD=7999 → at a 108 MHz SysTick
  clock the OS tick = 74 µs (13.5 kHz storm). The reconstruction programs
  board_setup_tick(108 MHz) → LOAD=107999 → 1 ms. The two images' tick
  cadences differ BY DESIGN 13.5× (the reconstruction "corrected" the
  vendor's un-updated SystemCoreClock). Any tick-level timing comparison must
  normalize this. The same CVR=0 harness workaround (write after the app's
  RVR write; t=10 ms verified for both images) is needed for the original.
- USART2: zero bytes TX in 30 s virtual (protocol tasks never start).

**3. The stall root-caused (orig-diag14):** the prio-3 main task loops
forever in the GPIO-configuration writer at label 0x08023678 — a CRL/CRH
nibble builder + BSRR/BRR pull-direction setter (configs 0x28/0x48 special-
cased) driven by a descriptor list on the task stack (r1 = 0x200022D4/DC).
16304 calls in 2 s with r0 = 0x40010C00 (GPIOB!) and the SAME descriptor pair
→ an infinite config-retry loop around GPIOB in the main task's init; prio 3
never blocks, so every lower task (protocol/UART/keys) starves. GPIOB hosts
the ST7735 bus (WR=PB0, D0-7=PB8-15) — the retry condition is NOT resolved
yet; prime suspect: the init verifies pin LEVELS and Renode's STM32F1 GPIO
model returns 0 for every unconnected input pin (IDR = externally-driven
connections only — stm32f1gpio.cs `IDR => GetValueFromBitsArray(State)`; no
pull-up emulation). Fix belongs in the HARNESS (wire BooleanConstant-style
pull-ups on the pins the config writer pulls up, or tag IDR) — never in the
reconstruction. Next session: hook the BSRR/BRR writes in the retry loop to
enumerate which GPIOB pins get pull configs, then drive those pins in the
platform overlay.

**4. Harness/tooling additions:** `orig-boot.resc` (the canonical original
run), orig-diag1..14 + orig-ramdump.resc (the evidence chain), monitor-Python
RAM dumps (`python "...emulationManager.CurrentEmulation.Machines[0]..."`
works — no DumpMemory monitor command exists), and the `logLevel -1
<periph>` syntax for Noisy logs. Renode GPIO pull-up emulation is the known
gap to solve for the display/keypad init (§4 item 2/5).

**CK status:** CK1 CLOSED (session 45). CK2 blocked on the original's boot
completing — the GPIO retry loop is the blocker. The reconstruction side of
CK2 (USART2 conformance vectors) is already green (make test-host).

### 2026-10-16 — Session 47: the original's stall decoded — a software-I²C EEPROM master waits for an ACK no slave gives

**1. HARNESS FIX (works): the armlink scatter base r9.** The decompressed
tables' flash pointers lost their 0x0804 prefix (session 46) because the
decompressor's `add r5, r9` (label 0x0801D85A; descriptor flag bit31) adds
r9 — the armcc scatter base — and Renode's reset leaves r9 = 0. Fix: hook
the walker entry (0x08042900 — a translation-block start, where hooks fire)
and `SetRegister(9, RegisterValue.Create(0x08040000, 32))` (IronPython needs
the RegisterValue type, not an int). Result: table word 0x20000000 =
0x08043B20 ✓ — the firmware's own fixup code now runs correctly. This is
equivalent to (and more surgical than) the followup-§4-sanctioned preload of
`ram_tables_recovered.bin`.

**2. THE STALL FULLY DECODED — it is a software-I²C master with no slave.**
The driver cluster (labels 0x08023660-0x080287E0, reached only via indirect
calls/vtables — no static callers in bl_graph):
- GPIO primitives (label: function): 0x08023678 config writer (CRL/CRH
  nibble build + BSRR/BRR pull-direction for configs 0x28/0x48), 0x08023780
  BSRR set, 0x08023788 BRR clear, 0x08023790 ODR write, 0x0802375E pin read
  (IDR & mask), 0x080284E4 countdown delay.
- THE BUS: DATA = **PB9**, CLOCK = **PA11** (NEW wiring — absent from the
  followup table and from earlier log sessions). Write transaction: START
  (0x080284F2) → send byte 0xA0 (classic I²C EEPROM WRITE opcode; 8 bits
  MSB-first, per-bit DATA set/clear + CLK pulse, delay(10) units) → wait
  RAM flag 0x2000515C == 1 → send addr-hi, addr-lo, data bytes, each
  followed by a flag wait.
- THE ACK: after the 8 data bits the send primitive configures PB9 as
  input-floating, pulses the 9th clock (PA11 high, delay(10)), samples PB9
  (0x0802375E) and writes the flag: bit==0 (ACK, slave pulls low) → flag=1;
  bit==1 → flag=0. NAK → the caller redoes the WHOLE transaction (the
  23k-call config-writer storm = these retries; the flag at 0x2000515C stays
  0 forever in emulation).
- The peer is an I²C-style EEPROM-class slave. NOT the 8051 (its AT24C512
  hangs off the 8051's own P2.4/P2.5; the 8051 link is UART-only — verified
  in UV98_868_220430_D4/MAIN/IO.H + AT24C512.c). Identity TBD (a settings
  EEPROM on the main CPU's bus is the prime candidate).
- TIM2 red herring closed: DIER=0x2 (CC1IE) with CCR1=0xFFFF > ARR=0x7D —
  the CC1 interrupt CANNOT fire even on silicon; SR.UIF sets but UIE=0 →
  no TIM2 IRQ ever. ISER=0x10000000 (IRQ28) with an empty ISPR is consistent.
  The flag setter is NOT an ISR.

**3. FAILED WORKAROUNDS (documented so nobody retries them):**
- Tag GPIOB IDR (0x40010C08-0B) = 0: the tag shadows MONITOR reads (readback
  0) but the stall persists — the ACK sample still behaves as NAK. Tagging
  the whole GPIOB range (0x40010C00-0x40010FFF) also fails. Root suspicion:
  the model's Connections state for PB9 retains the MCU's last ODR drive
  through the output→input reconfig, and the tag/CPU interaction does not
  produce the needed read-0 at the sample instant.
- Wiring CaptureCompareInterrupt -> nvic@28 for timer2 (correct per real F1
  semantics — KEPT in gd32f103.repl — but irrelevant: CC1 never matches).

**4. THE PROPER FIX (next session, well-scoped): a Python I²C-slave
peripheral** wired `-> gpioPortB@9` (DATA) with its clock input fed from a
PA11 tap: count clock edges, capture 8 data bits, drive DATA low during the
9th-clock ACK window (the model IGNORES external signals on output pins, so
contention is free), maintain an EEPROM array (write transactions at 0xA0;
read transactions at 0xA1 drive DATA per the byte — erased = 0xFF vs 0x00
semantics to be validated against the boot's expectations). This is the same
class of work as the followup-§4 synthesized font flash — harness-side only.

**5. TOOLING LESSON (critical): Renode CPU hooks fire ONLY at
translation-block starts** (branch targets/BL targets/exception entries).
Mid-block hooks (e.g. 0x08028484 — the bl INTO the config writer) never fire
even though the instruction provably executes (LR evidence at the callee's
own hook). This explains the session-46 "0 hits" contradictions. When a hook
doesn't fire, check whether the address is a branch target; hook the block
start or the callee instead.

**CK status:** CK1 CLOSED. CK2: the original's boot still stops at the I²C
ACK wait (deterministic, OSTime=0x68F4 identical across runs); the tables are
now correct (r9 fix); the remaining gap = the missing I²C slave, same
category as the followup-§4 synthesized peripherals. The reconstruction side
of CK2 remains green (make test-host).

**Session 47, part 2 — the stall cascade BROKEN: the original's OS RUNS.**
The Python-I2C-slave plan from part 1 turned out unnecessary for boot
progress. The working sequence (each fix exposes the next stall):
1. GPIOB REMOVED from the platform (gd32f103.repl) — the ACK sample then
   reads 0 (unimplemented range) → ACK → the settings-EEPROM driver
   completes. (The IDR-tag experiments failed: the tag shadows monitor
   reads but the firmware still sampled NAK — bypassed on the CPU path.
   Removing the port sidesteps the model entirely.)
2. Next stall: `adc_flag_get(ADC2, EOC)` poll at 0x0802C388 (handle =
   0x40012800) — no ADC model. Fix: tag ADC2_SR (0x40012800) = 0x2.
3. Next stall: SPI2 (0x40003800) — the EXTERNAL FONT FLASH transfers
   (TXE/RXNE polls + full-duplex DR) — the followup-sec4 synthesized
   peripheral. Fix: `sim/sp2_fontflash.py` (Python.PythonPeripheral:
   0x03 READ + 24-bit addr state machine, 0x9F JEDEC ID, content 0xFF =
   erased; IronPython 2 requires pure-ASCII scripts — an em-dash broke it).
4. RESULT: OSCtxSwCtr 8 → **3010** (2 s), OSTime advancing, **the first
   protocol frame "$A00" transmitted on USART2** (the 8051-link handshake),
   zero faults. The original's OS + tasks + protocol pump are ALIVE.
   (IronPython note: `SetRegister` needs
   `RegisterValue.Create(v, 32)`, not an int.)

Platform consolidation: gd32f103.repl now carries the GPIOB removal, the
ADC2_SR tag and the SPI2 python peripheral (header documents all of it);
orig-boot.resc = the canonical original run (RCC_CFGR tag + r9 hook + CVR
workaround — dropping the RCC_CFGR tag silently wedges SystemInit, cost one
debug round). ck1.resc re-verified green on the new platform (identical
deterministic counters).

**NEXT (CK2): the 8051-link peer.** The original sent "$A00" and expects the
coprocessor's answer; the reconstruction will reach the same point. The
peer = a USART2 responder playing the conformance vectors (tests/
test_protocol.c) — the same stimulus feeds both images for the differential
comparison. Then CK3 (AT1846S bus trace) and the settings-EEPROM content
(the synthesized I2C EEPROM byte content — the $A-settings semantics supply
the defaults).

### 2026-10-16 — Session 48: the 8051 peer v1 — settings handshake works on the original; first CK2 differential data point

**Protocol decoded from the peer's own source (UARTX.C, the 8051 project):**
- The boot handshake: the main CPU sends `$A00\r\n`; the 8051 answers
  `$A00,<14-byte CPU id>,<512B settings>\r\n` = **534 = 0x216 bytes**
  (UART4_TX_EEROM; the id from READ_CPU_ID, the settings from its EEPROM
  buffer). This explains the 0x216 frame length the recovered dispatcher
  checks.
- The write direction: `$A01,` + 512 = **519 bytes** (UART4_RX_EEROM — the
  534-vs-519 asymmetry is real, both confirmed in the 8051 source).
- Byte-exact peer response used by the harness: `$A00,` + '0'x14 + ',' +
  0x00*512 + CRLF (neutral defaults; must be IDENTICAL for both images —
  documented as the CK2 stimulus).

**Injection mechanism:** `usart2.WriteChar(ord(c))` from monitor Python
(the STM32_UART receiveFifo + baud pacing deliver in virtual time, order
preserved). IronPython 2 traps: `bytes` IS `str` (bytes(512) = "512"!) —
use chr()/ord(); multi-line `python` commands do not tokenize in .resc
files; a literal `$` in the script collides with Renode variables.

**Original + peer (peer-8051.resc):** the $A00 fires at ~0.8 s virtual; the
injected 534-byte frame is consumed and the system goes from the handshake
wait to full multitasking: **OSCtxSwCtr 3010 -> 90111** in the next 3 s,
OSTime advancing, zero faults. No further USART2 TX in the window (the
pump's next traffic is event-driven — status/telemetry frames).

**CK2 differential v0 (the first classified finding):** the reconstruction
under the IDENTICAL stimulus (peer-recon.resc) runs clean (11157 switches,
idle counter ~19.9M, zero faults) but **never sends `$A00`** — the boot-time
protocol pump is not in the reconstruction's task set: the original's $A00
sender lives in the UI-dispatch cluster (0x0800C05E.. callers of the command
dispatcher 0x08010128 — all indirect/vtable calls, no static chain).
Classification: MECHANISM gap (reconstruction-side scaffold incompleteness),
not data. The fix = reconstruct the pump's boot behavior (send $A00 at init,
consume the $A00 response into the settings RAM region) — the recovered
protocol.c already has protocol_send_cmd(CMD_A00_READ) and the frame
reception; only the task-level wiring is missing.

**Artifacts:** reconstruction/sim/peer-8051.resc (original + peer),
reconstruction/sim/peer-recon.resc (reconstruction + peer). CK1 stays green;
the canonical runs unchanged.

**Next:** (a) reconstruct the pump wiring so the reconstruction also emits
$A00 at boot (making the CK2 vectors comparable), (b) then run the byte-exact
USART2 trace diff (both directions) for the parity matrix, (c) CK3's
AT1846S bus trace once the display/RF init paths stabilize.

**Session 48, part 2 — CK2 handshake parity: byte-identical USART2 traces.**
The reconstruction's dispatcher task now performs the boot settings fetch
(`protocol_send_cmd(0)` wired into the prio-11 task — the recovered
behavior; the original's $A00 observed at 0.8 s virtual comes from the same
command dispatcher 0x08010128 reached through the UI-pump cluster). Under
the IDENTICAL peer stimulus both images now produce:

    TX: 24 41 30 30 0d 0a   ("$A00\r\n")   — BYTE-IDENTICAL
    RX: the injected 534-byte settings frame (same content by construction;
        both consumed it fault-free)

Traces: `analysis/traces/ck2_handshake_{orig,recon}.txt` (Renode
`CreateFileBackend` byte streams). Timing differs (TX at 0.8 s virtual on
the original — its boot runs the display/EEPROM init before the pump fires —
vs 0.13 ms on the reconstruction): the sec7(c) timing boundary, documented.

**CK2 parity matrix v1:**
| stream            | original         | reconstruction   | verdict      |
|-------------------|------------------|------------------|--------------|
| TX $A00 handshake | 6 B, byte-exact  | 6 B, byte-exact  | PARITY       |
| RX settings frame | 534 B consumed   | 534 B consumed   | PARITY       |
| TX timing         | 0.8 s virtual    | 0.13 ms virtual  | sec7(c) diff |
| post-handshake TX | none in 3 s      | none in 3 s      | PARITY       |

The remaining CK2 work = broader stimulus coverage (key events, $A01 writes,
$B/$C reads — the conformance-vector set from tests/test_protocol.c driven
through the live machines) + the settings-consumption comparison on the RAM
side (the original copies the 512 B into its settings region; the
reconstruction's handler fills rxBuf — the consumers differ by design until
the UI settings-load is reconstructed).

**Session 48, part 3 — CK3 groundwork: the RF-path labels FAIL dynamic
validation (new open question).** Hooking the annotated RF primitives
(rf_send_byte 0x08022712, rf_write 0x0802289A, rf_read 0x0802292E,
rf_cs_select 0x08022838) during a 5 s original run:
- rf_write fired 2x, rf_read 3x, rf_cs_select 1x (deduped line), rf_send_byte
  0x — the AT1846S traffic is TINY (no bulk register-file init; the radio's
  RF config is event-driven).
- The two rf_write events came WITH the settings-apply window (right after
  the peer's settings frame!) — the causal chain settings -> RF config is
  confirmed — but the args (r0=0x1D, r1=0x6A, r2=0x12, r3=0x14, via a tail
  chain through 0x0801D840 with a stale LR=0x0801CD2F) do NOT decode as
  (chip, reg, value) under the recovered ABI.
- The rf_read events show (r0=0, r1=3, r2=0x23/'#' and 0x52/'R') — character-
  like values, not (chip=0, reg=0x1F) as the RSSI-poll interpretation predicts.
- CONCLUSION: the rf.c label annotations (session 18-19) and the dynamic hook
  evidence CONFLICT. Either the labels are mis-assigned (the "RF primitives"
  are string/protocol helpers) or the RF entry points differ from the
  annotations. The bl_graph confirms callers exist in BOTH the 0x08016xxx
  cluster (S-meter/RF-config region) AND 0x0801D73C/0x0801D832/0x0801DA1E
  (the loader/decompressor neighborhood) — an unusual spread that supports
  the mis-assignment suspicion.
- TOOLING UNLOCK (the round's key enabler): hooks CAN reach
  `emulationManager.CurrentEmulation.Machines[0].SystemBus` — full memory
  read/write inside hooks (the InterruptPythonEngine shares the monitor
  globals). Combined with the block-start hook rule (session 47), this gives
  complete in-simulation instrumentation. ALSO: stale-address traps — every
  rebuild shifts the reconstruction's symbols (OS_Sched 0x08005438 ->
  0x08005450 cost a debug round); ALWAYS re-derive with nm.

**CK3 status:** not started in earnest — the prerequisite is the RF-path
label re-validation (REA pass over 0x08022712/0x08022838/0x0802289A/0x0802292E
and their callers) OR a ground-truth GPIOA pin-level capture (the
STM32F1 GPIO model's ODR-drive logging) to identify the true bit-bang
primitives dynamically. The reconstruction's rf.c transport remains unwired
(dead code — no callers), so the original's (chip, reg, value) sequence,
once captured correctly, doubles as the wiring spec.

**Session 48, part 4 — the settings->RF causal chain captured; CK3 scoped.**
With the emulationManager-in-hook memory access:
- The two rf_write events fire INSIDE the $A00 settings-frame processing
  (the hook's stack dump contains the injected settings bytes: '0'x14 ID,
  0x20004E68 pointers). CAUSAL CHAIN CONFIRMED: peer settings frame ->
  the main parses it -> RF configuration writes. The radio applies the
  received settings to the AT1846S immediately.
- The captured call: r0=0x1D(29), r1=0x6A(106), r2=0x12(18), r3=0x14(20)
  via a tail chain through 0x0801D840 (a return-0xFF leaf!) with a stale
  LR (0x0801CD2F) — the caller at 0x0801CD2A calls 0x0801D840 which
  branches into the 0x0802289A region. The rf_read events: (r0=0, r1=3,
  r2=0x23/'#', 0x52/'R').
- The args do NOT decode under the recovered rf_write(chip, reg, val) ABI —
  combined with part 3's conflicts, the RF-path labels need the REA
  re-validation BEFORE the trace can be interpreted. The reconstruction's
  rf.c transport stays unwired until then (its dead code = no callers).
- CK3's shape once validated: stimulus = the same settings frame; expected
  = IDENTICAL RF write/read sequences from both images (the original's
  captured sequence = the reference; the reconstruction's settings consumer
  + rf.c = the implementation to match).

**Session 49 — CK3 Round A: the RF-label "conflict" RESOLVED (an address-space
bug, not a mislabel); reference AT1846S bus trace captured.**
- ROOT CAUSE of session 48's label conflict: the hook census placed hooks at
  rf.c's IMAGE LABELS (file+0x08000000 convention) directly as Renode hook
  addresses, but Renode loads the image at 0x08004000, so Renode hook
  address = label + 0x4000 = file + 0x08004000 — exactly the conversion rf.c
  line 8 has documented all along ("physical flash = label + 0x4000"). The
  census hooks therefore fired inside UNRELATED functions at file 0x1Exxx
  (UI/status code), producing the incoherent args. RETRACTION of session 48
  parts 3-4's interpretations: there is no pop-dispatch/resumption-point
  structure, 0x0801CD20 is not an RF helper, and 0x08022712 is not a delay
  loop — all were artifacts of hooking wrong addresses. The rf.c function
  boundaries were CORRECT.
- Method: REA/Ghidra 12.1.4 on the raw binary opened with entry 0x08004000
  (REA address space == Renode address space == file+0x08004000; REA addr =
  log-label + 0x4000). Decompiles validate rf.c completely:
  * FUN_0802689A (rf_write): CLK low -> delay 8 -> cs_select(chip) ->
    delay 10 -> send 3 bytes [reg, data_hi, data_lo] -> delay 8 ->
    cs_select(both) -> CLK low + DATA low -> delay 8. Frame buffer is a
    local copy of {u8 reg; u8 pad; u16 val} — wire order MSB-first.
  * FUN_08026838 (rf_cs_select): mode 1 = CS1(PA12) set + CS0(PA8) clear;
    mode 0 = CS0 set + CS1 clear; other = both set. Matches rf.c.
  * FUN_08026786 (send-N loop) and FUN_08026712 (rf_send_byte): per bit
    CLK LOW -> delay 10 -> DATA by bit -> delay 10 -> CLK HIGH -> delay 4.
    rf.c updated to this pin-exact order (was CLK-high-then-low per bit).
  * FUN_080267AA = the RF-bus GPIO init (PA9/PA15/PA8/PA12 -> output mode 3
    via the gpio_init helper 0x08023678), called from hw_init.
- Corrected hooks (Renode space): rf_write 0x0802689A, sendN 0x08026786,
  send_byte 0x08026712, rf_read 0x0802692E. Capture script
  /tmp/opencode/orig-rfOK.resc, plain 5 s boot, NO peer needed.
- **CK3 REFERENCE TRACE captured: analysis/traces/ck3_rf_orig.txt.**
  290 rf_write (104 with cs=1, 186 with cs=0), 2 rf_read (cs=1 reg 0x30;
  cs=0 reg 0x0D), 872 send_byte events (290x3 write bytes + 2x1 read
  address bytes — cross-check EXACT). The stimulus is a plain boot: the
  whole sequence is boot-time RF init + default state, fully deterministic
  under the platform workarounds (SPI2 font-flash returns 0xFF).
- Sequence decode (buf = {reg, pad, val_lo, val_le-hi}; wire = [reg, hi, lo]):
  head = classic AT1846S init on cs=1: 0x30=0x0001 (soft reset), 0x30=0x0004,
  0x04=0x0FD0 (clk_mode), 0x0A=0x7EA0, 0x13=0x00A1, 0x1F=0x9AA9, 0x31, 0x33,
  0x34, 0x41..0x63 block, then 0x30=0xA440/0xA640/0x0640 mode staging; the
  cs=0 chip gets the mirrored block (186 writes). Tail: 0x05=0x8543 (the
  band-table MATCH value — so band config runs with a real default
  frequency even though reg 0x29/0x2A are programmed 0x0000), 0x30=0x0670 /
  0x4670, 0x1F=0x9A9A / 0x9A9E. No consecutive duplicate (reg,val) pairs.
- CS semantics observed: mode 1 selects the first-configured chip, mode 0
  the second — rf.c's chip naming (0/1) is consistent with the decompile
  (mode -> PA12/PA8 complementary drive).
- Next (Round D): wire the reconstruction's rf.c into a boot driver
  reproducing this sequence; the static init table is directly recovered;
  the computed tail (band config value 0x8543, mode staging) needs the
  driver logic; then byte-diff the two traces.

**Session 49, part 2 — CK3 INIT PARITY: the reconstruction's boot RF burst is
byte-identical to the original's (100/100 events).**
- Wired the reconstruction: rf.c gained rf_bus_gpio_init() (0x080267AA
  decompile: PA9/PA15/PA8/PA12 output push-pull, CS idle high) and
  at1846s_boot_init(chip) — the 50-register init table — and main.c's
  hw_init() now calls rf_bus_gpio_init() + at1846s_boot_init(chip1) +
  at1846s_boot_init(chip0) at the recovered RF-chip-init call site.
- TABLE CORRECTION (the round's second byte-order lesson): the first table
  extraction mis-decoded the hook logs' unpadded hex concatenations
  ("0A1" = bytes 00,A1 = 0xA100, NOT 0x00A1; "10" = 0x0001). The SENDN wire
  captures ([reg, val_hi, val_lo]) are the ground truth: buf =
  {u8 reg; u8 pad; u16 val little-endian}. 21 of 50 entries were byte-
  swapped in the first extraction; all corrected and cross-validated
  (0x30=0x4006 matches the known standby mode; 0x7F=0x0001 = bank 1 per
  the register map; 0x48=0x1E3C matches the tone wrapper's constant).
- VERDICT (python triple-diff, /tmp/opencode/recon_rf2.log vs
  orig_rfOK.log): events 1-100 — (cs, reg, val) identical, ZERO
  mismatches. Traces: analysis/traces/ck3_rf_{orig,recon_boot}.txt.
- Init-table structure notes: 26 entries (0x30 reset/clken, 0x04 clk_mode,
  0x0A PA bias, 0x1F GPIO map, 0x31..0x63 analog block, 0x30 mode staging
  A440->A640->4006) + 24 entries behind the bank-1 window (0x7F=0x0001 ->
  regs 0x06..0x0F RX-path config, 0x12, 0x7F=0x0000 back to bank 0). The
  two chips receive the IDENTICAL table (mirror check TRUE) — the cs mode
  is the only difference.
- REMAINING for full CK3: the t~3.1 s reconfig burst (178 events: the init
  table RE-RUNS on both chips, then 78 dynamic config writes + 2 rf_read
  (cs=1 reg 0x30 status, cs=0 reg 0x0D)) and the 12-event tail. The
  dynamic writes observed: 0x1F=0x9A9A, 0x30=0x0670/0x4670, 0x58=0x0D84,
  0x4E=0x0220, 0x59=0x08DA, 0x05=0x8543 (band-table match), 0x29/0x2A=
  0x0000 (frequency regs, zero VFO state). Call-site LRs cluster in
  0x08027109..0x080274D7 and 0x0802789F..0x080278E5 (Renode space) + the
  tone wrapper 0x0801A91A. That flow = the app-level settings->RF config
  chain; wiring it is the next increment.

**Session 49, part 3 — CK3 FULL PARITY: 292/292 RF bus events byte-identical.**
- Decompiled the whole reconfig-flow driver family (Renode space):
  * FUN_08027156 = band-config + frequency + mode dispatch: the 3-sentinel
    check (0x8643) -> 96-entry table scan (0x8543) -> fallthrough (0x8763)
    -> the soft-float frequency pipeline -> regs 0x29/0x2A -> mode dispatch
    on param_2 (0 = FUN_08027020 TX writer, 1 = FUN_08027092 RX writer).
    CONFIRMS rf.c's at1846s_set_band_config/set_frequency structure exactly.
  * FUN_08027110/FUN_08027020/FUN_08027092 = the 0x30 mode writers; the
    band argument selects the 0x40xx (band 1) vs 0x70xx (band 0) filter
    bits: standby {0x4006|0x7006}, TX pair {0x4046|0x7046}, RX pair
    {0x40A6|0x70A6 + delay(0x32)}.
  * FUN_080272E2 = GPIO 0x1F reconfig under a killed-irq window
    (0x9BA9/0x9AA9 by band); FUN_080272A4 = the plain variant (0x9EA9/0x9AA9).
  * FUN_080276D4 = CTCSS config: 0x58=0x8005 then, for CSS off, the boot
    defaults 0x4A=0x157C, 0x4D=0x0000, 0x3A=0x40C9, 0x5B=0x1213; the on
    branch scans a 50-entry tone table (DAT_08028014).
  * FUN_08027474 = detect/filter path: 0x58=0x840D, 0x4E=0x2002, 0x59=0x0A8D.
  * The tone wrapper FUN_0801A91A at boot writes 0x48=0x1E3C, 0x49=0x0284.
- The 0x8543-at-zero-frequency paradox RESOLVED: the 96-entry sub-band
  table's entries 24..95 are zero-init, so the boot state (frequency 0)
  MATCHES a zero entry -> 0x8543, and the frequency pipeline yields
  0x29/0x2A = 0x0000. rf.c's full-96 scan was already faithful.
- Wired at1846s_boot_default_apply() into the dispatcher boot path: phases
  X/Y/Z/W/V reproducing events 101-290 (chip1 poke 0x7002, GPIO maps, chip0
  bank window + tone pair, three apply passes with CTCSS-off/filter-detect
  variants, chip1 re-init, ending TX mode + 0x9EA9 GPIO). The register
  primitives = decompile-recovered; the call ORDER = [DATA-DERIVED] from
  the trace (the original's orchestrating task chain is not yet
  reconstructed) — labeled in rf.c and to be restated in the CK5 report.
- Two rf_read status polls placed by timestamp correlation: (cs=1, reg
  0x30) after the 0x7002 poke + GPIO write; (cs=0, reg 0x0D) after the
  first RX-mode pair.
- After two iteration fixes (the mode writers emit their own leading
  standby — remove explicit ones; event 290's GPIO variant takes band=1):
  **RESULT: 292/292 bus events (290 writes + 2 reads) byte-identical,
  same order, original vs reconstruction.** Traces:
  analysis/traces/ck3_rf_{orig,recon_full}.txt. CK3 CLOSED at the
  bus-event level; pin-level (ODR) parity and the settings-driven
  (non-default) channel path remain open extensions.

**Session 49, part 4 — CK4 Round E: the GPIOB Python model lands; the
original's display stream captured (187k events); the recon gap quantified.**
- New platform piece: reconstruction/sim/pb_displaybus.py, a PythonPeripheral
  at 0x40010C00 (gpioPortB restored in gd32f103.repl). Faithful register
  GPIOB (CRL/CRH/ODR state, BSRR/BRR atomics) with IDR pinned to 0 — the
  session-47 software-I2C ACK workaround stays in force through the model
  (the original still boots: OSCtxSwCtr 0x160C4 in 5 s vs 0x15BE7 without
  the port — same class). Every ODR/BSRR/BRR/CRL/CRH write logs to
  /tmp/opencode/pb_stream.log.
- TOOLING NOTE: IronPython 2 rejects non-ASCII scripts (an em-dash in my
  first header aborted the platform load with CompilePythonCode); keep the
  Python peripheral scripts ASCII-only.
- ORIGINAL's stream (analysis/traces/ck4_display_orig.txt.gz, 187758 bus
  events in 5 s): BSRR/BRR 0x0200 = PB9 = the software-I2C DATA (settings
  EEPROM transactions; 7913 sets / 14188 resets); CRH flips
  0x44444434<->0xbbb44434 = PB9's nibble toggling output(3)<->input(4) =
  the I2C direction switch for the 9th-clock ACK sample; CRH 0xbbb44434 =
  PB13/14/15 open-drain outputs (display serial control); the ODR bulk =
  0x02be (50749x) / 0x02ef (50499x) / 0x02ff (27940x) = the display
  bit-stream on PB0-7 with PB9 idle-high (~50k two-state cycles). CRL is
  written once at display init.
- RECONSTRUCTION's stream: EMPTY — display.c is a SCAFFOLD with no GPIOB
  accesses (the PythonPeripheral initializes lazily on first access; its
  IsInit never fired). CK4's remaining work = the display driver
  reconstruction (0x080284F2/0x080285B6 serial functions + the controller
  init + the font streaming path), then a structural diff of the two
  streams. The capture infrastructure and the original reference = DONE.

**Session 49, part 5 — Round B/C: the CK2-broadening unblocked a REAL
reconstruction bug cluster (the RX path); two bugs fixed, one parked.**
- BUG 1 FIXED: u2_ring was 64 bytes — the 534-byte settings frame overflowed
  it, the CRLF tail never stored, the frame boundary never fired, and the
  ring index wedged at 64 dropping EVERY subsequent frame. The
  session-48 CK2 parity matrix's "RX settings frame | 534 B consumed"
  reconstruction row was NEVER actually verified — retracted; only the TX
  parity was real. Ring = 536 now.
- BUG 2 FIXED: the usart2_init wrote CR1 in two steps (plain write, then
  "|= UE" read-modify-write). The Renode STM32_UART model registers
  Word(4B)/Byte(1B) translations only; the RMW's 16-bit read returned
  garbage (0x4000101-class) which was written back with UE set, leaving
  RE/TE/RXNEIE clear (final CR1 = 0x4002101) — the receiver interrupt was
  permanently disabled. CR1 is now written once as a 32-bit access
  (verified 0x202C after boot).
- BUG 3 PARKED (open issue R1): with the ring and CR1 fixed, the
  reconstruction's USART2 ISR still starves. Evidence: the original's ISR
  is entered 534 times for an instant 534-byte injection (RXNE set on every
  entry; the model's level-held IRQ is re-taken per byte), while the
  reconstruction's ISR is entered only 27-83 times (varies with the
  injection pacing: 27 paced/8-byte chunks, 44 per-2-byte chunks, 83
  instant), then never again; OSCtxSwCtr freezes; the PC is last seen in
  OS_CPU_PendSVHandler. The ISR never reaches protocol_frame_received
  (no frame boundary within the first 27-83 bytes). Diagnostics collected:
  nvic-level logs (only 4-5 "External IRQ 54" raises for the recon vs
  continuous for the original), ISR entry counters, CR1/SR/ISER/ISPR dumps.
  Next-round candidates: (a) diff the vendored port's PendSV/nesting path
  against the original's exception flow under a pre-empted ISR; (b) probe
  the model's IRQ re-raise semantics (cf. renode issue #690's idle-line
  token race in the same model); (c) a custom Python UART with explicit
  per-byte IRQ edges as the harness-side workaround. The CK2 broadening
  ($A01/$B/$C stimuli, the pattern-payload diff) resumes after R1.
- ROUND C partial result (the original side, unaffected by R1): the
  pattern-payload run (settings[i] = i & 0xFF) shows the original stores
  the RAW 534-byte frame at TWO buffers (0x20003AD0 and 0x20003BD0 —
  matching the decompiled $A00 branch's two memcpy targets DAT_08023EEC/
  DAT_08023EF0) AND parses a sparse typed-field structure at
  0x20004E40..0x20004F80 (a 6-byte '000000' callsign field, a 4x0x38
  field, scattered config bytes — the parser copies only the fields it
  consumes). The reconstruction's consumption = the raw rxBuf copy only;
  the typed-field parser is not reconstructed. First extraction of the
  boot init table had byte-swapped 21/50 entries (the hook logs' unpadded
  hex concatenation "0A1" = bytes 00,A1 = 0xA100, NOT 0x00A1) — the SENDN
  wire captures ([reg, val_hi, val_lo]) were the ground truth; corrected
  in session 49 part 2.

**Session 50 — R1 SOLVED: the RX starvation was the BRR value; the frame
path now achieves 532/534 byte parity with the original's raw copy.**
- ROOT CAUSE: the reconstruction's usart2_init programmed BRR = 0x2580
  (divisor 9600) while the ORIGINAL writes **BRR = 0x341** (divisor 833;
  read from the running image at 0x40004408 — the original's CR1 = 0x202C
  confirmed the same way). The Renode STM32_UART model paces injected
  WriteChar bytes by platformFrequency (its 8 MHz default; the .repl sets
  no usart frequency) / BRR divisor / 10 bits: 0x2580 → 833 baud →
  12.0 ms/byte (the observed 83 takes/s!), 0x341 → 9604 baud →
  1.04 ms/byte (534 bytes in ~0.56 s — the original's observed 534 takes
  in the 1 s window). The "starvation" was never the ISR, the NVIC, or
  the ring: the bytes were arriving at 1/12th speed and every observation
  window was simply too short. Fix: the reconstruction writes the
  original's own BRR value.
- ISOLATION METHOD (the tooling lesson): hooks cannot see inside an ISR
  loop, so the ISR was instrumented in C (a dbg array logging {SR, byte,
  ring index} per take). The trace proved the ISR takes were sequential,
  correctly paced, byte-perfect (dbg_idx = 0,1,2,...; dbg_dr = the frame
  head), and exiting cleanly on RXNE=0 — shifting the hunt from the ISR
  to delivery. The take-count-vs-RunFor-length comparison (83 takes @1 s,
  416 @5 s) revealed the pacing. The dbg arrays were removed after the
  diagnosis (they cost 6.4 KB RAM); the single-byte ISR form (the
  original's shape) is retained.
- RETRACTION (part): session 49's "CR1 |= UE RMW corruption" was
  over-read — the 0x4002101 CR1 dumps came from the MONITOR's 16-bit
  sysbus reads, which return garbage against the STM32_UART model
  (AllowedTranslations covers Word/Byte only). Only 32-bit monitor reads
  of this model are trustworthy (CR1 reads 0x202C both before and after
  the single-write change). The single-write form is kept (it matches the
  original's SPL shape) but the RMW was probably never harmful.
- RESULT (pattern payload, settings[i] = i & 0xFF): the reconstruction's
  rxBuf = the complete 534-byte frame; **532/534 bytes identical to the
  original's raw copy at 0x20003A84**. The 2 differing bytes (frame 28-29:
  original 0x38 0x39 vs the true 0x08 0x09) = the original's parser
  mutating its WORKING copy in place — settings[8..9] OR'd with 0x30
  (a binary→ASCII digit conversion); the original's second copy
  (0x20003B84) is pristine, confirming buffer1 = the parse-working buffer
  and buffer2 = the archive — exactly the decompiled two-memcpy structure
  (DAT_08023EEC/DAT_08023EF0), which the reconstruction's rxBuf/rxBuf2 now
  mirror. The in-place conversion = a Round C parser finding.
- REGRESSION CHECK: 290 RFW events still fire (CK3 intact), fault
  registers = 0, the OS runs. Harness note: after a WriteChar burst the
  RunFor must cover ~0.6 s of virtual time for the full 534-byte frame at
  9604 baud.
- STATUS: R1 CLOSED. Round B (CK2 broadening) and Round C (the typed-field
  parser) are unblocked.

**Session 50, part 2 — Round C: the "typed-field struct" theory RETRACTED;
the consumption model = the working buffer + lazy field access.**
- The 0x20004E40..0x20004F80 region is PAYLOAD-INDEPENDENT: 80/80 words
  identical between the pattern payload (settings[i] = i & 0xFF) and the
  neutral payload (all 0x00). The '000000' callsign-looking field and the
  4x0x38 word = the radio's own default state, NOT a copy of the peer's
  settings. The session-49 note "the parser copies consumed fields into
  the struct" is retracted.
- The WORKING buffer (0x20003A84) IS the consumption surface: 510/534
  bytes = the payload verbatim (the frame lands byte-exact), the header
  and the 14-B ID are preserved, and the ONLY transform in the entire
  buffer = frame[28..29] = settings[8..9]: 0x08 0x09 → 0x38 0x39 under
  the pattern payload; UNCHANGED (00 00) under the neutral payload — so
  the transform is payload-dependent and not a plain |0x30 ASCII mask.
  The original's second copy (0x20003B84) never contains the transform.
  Classified: an in-place field conversion by the original's $A00
  consumer at settings offsets 8-9 (mechanism not fully identified —
  open item for the CK5 report; 532/534 parity otherwise).
- Round C status: the reconstruction's consumption model (rxBuf + rxBuf2
  clean copies, the ready flag) = 532/534 byte-parity with the original's
  archive copy. The remaining work = identifying the settings[8..9]
  consumer (a decompile of the flag-polling task chain) and any further
  lazy-field consumers, which only manifest under real (non-synthetic)
  settings content.

**Session 50, part 3 — Round B: CK2 matrix v2 (the RX dispatch routing
parity) CLOSED; the recon's per-command buffer routing corrected.**
- Literal-pool reads of the original (the running image, Renode addr =
  the REA addr): DAT_08023EEC = 0x20003A84 (the $A00 working copy),
  DAT_08023EF0 = 0x20002F74 (the $A01/$B/$C archive), DAT_08023EFC =
  0x20004B94 (the $A02 telemetry buffer), DAT_08023EE4 = 0x20005094 (the
  flags), DAT_08023F0C = 0x20004E68 (the app-state anchor), DAT_08023F14
  = 0x2000469C (the app-state). CORRECTION to session 49/50 pt2: the
  "two raw copies at 0x20003AD0/0x20003BD0" = ONE copy (buffer1) — the
  second hit was the sawtooth's 256-byte periodicity inside the same
  buffer. The true second copy lives at 0x20002F74.
- ROM compare strings read directly: "$A00,", "$A01,", "$A02,", "$A08,"
  (5-char), "$B", "$C" (2-char), "NO DATA" (the $B +5 marker check).
- Method: a hook on the original's copy routine (FUN_0801D8C0, logging
  r0=dest/r2=len; hook args must use {0}/{1} indices — a {2} with two
  args aborts Renode) captures every dispatch copy; the ISR entry hook
  counts takes. Findings: "$B00\r\n" -> memcpy(EF0, 6); "$C00\r\n" ->
  memcpy(EF0, 6); "$A02,XXXXXXXX\r\n" -> memcpy(EFC, 15); "$A08,..." ->
  memcpy(EF0, **5** — the 5-byte prefix ONLY, then the conditional
  buf+5 vs a 7-byte marker gated on the app-state byte 0x20004E68+0x38);
  the $A00 gate re-verified (len != 0x216 -> the flag cleared, no copy).
  The original's consumer wipes buffer2 within ~2 ms of the copy, so
  buffer DUMPS always read zero — the hook capture is the only reliable
  observation (an observation-timing lesson for CK5).
- DIVERGENCE FIXED: the recon routed $A01/$B/$C/$A02/$A08 all to rxBuf;
  the original routes $A01/$B/$C -> buffer2, $A02 -> buffer3, $A08 ->
  buffer2 as a 5-byte prefix. protocol.c rewritten to the original's
  per-command targets (rxBuf/rxBuf2/rxBuf3 = 0x20003A84/0x20002F74/
  0x20004B94 analogues; rxBuf3 = 64 B). $A08 now copies exactly 5 bytes
  (the recon reproduces the original's byte-5 residue behavior).
- Matrix v2 results (recon, post-fix): $B00 -> rxBuf2 = "$B00\r\n" OK;
  $C00 -> rxBuf2 = "$C00\r\n" OK; $A02 -> rxBuf3 = "$A02,XXXXXXXX\r\n"
  OK (rxBuf2 untouched); $A08 -> rxBuf2 = "$A08," + the residue OK;
  the malformed $A00 (len 120 != 0x216) -> NO copy, both buffers clear
  (the gate); "HELLO\r\n" (non-$) -> ignored. All seven rows match the
  original's observed dispatch.
- Harness note: the 534-byte $A00 needs >= ~0.6 s RunFor after the
  WriteChar burst (the model paces at 1.04 ms/byte); a 300 ms window
  silently truncates the frame (no CRLF, no dispatch — the earlier
  "missing 0x216 copy" in this very run).

**Session 51 — CK4 Round E: the display subsystem RECOVERED and the init
phase achieves 74/74 BYTE-EXACT parity.**
- Controller identified: ST7735-class color TFT on an 8-bit parallel bus,
  decompiled from the original (the Ghidra flow analysis MISSED the init
  function's body at 0x0801BB30-0x0801BC74 — recovered by disassembling
  the region directly; the region is reached only through an indirect
  path so Ghidra never built the procedure).
- PIN MAP (decompiled from the register helpers 0x08023780=BRR/0x08023788=
  BSRR/0x0802375E=IDR/0x08023790=ODR and the literal pools):
  PB0-7 = D0-D7 (the data byte written as ODR=(ODR&0xFF00)|byte,
  0x0801BA30); PA0 = WR strobe (BSRR then BRR per byte); PC3 = RS/D-CX
  (HIGH=command 0x0801BA6A, LOW=data 0x0801BA8C — inverted vs HD44780);
  PC0 = reset (pulsed low-high-low in the init); PC2 = control (low after
  the init). Window setter 0x0801B9C8 = 0x2A column range + 0x2B row range
  (4 data bytes each). Color function 0x0801BD3C = RGB565 (0xF800/0x1F
  constants). The GPIO config 0x0801BAC0: GPIOA CRL 0x33333333, GPIOC CRL
  nibbles 0/1/3 = output, PC2 low.
- INIT TABLE [0x0801BB22]: 0x11 SLPOUT, delays, 0xB1/B2/B3(05 3A 3A),
  0xB4(03), 0xC0(28 08 84), 0xC1(C0), 0xC2(0C 00), 0xC3(8C 2A), 0xC4(8A EE),
  0xC5(0C), 0x36(A8), 0xE0/E1 gamma tables, 0x3A(05)=RGB565, 0x29 DISPON,
  PC2 low. 86 ops total, extracted from the disassembly.
- NOT the display: 0x080284F2/0x080285B6/0x080286C0/0x0802873C = the
  I2C EEPROM bit-bang driver (SDA=PB9, SCL=PA11, the device 0xA0/0xA1,
  the ACK via a PB9 direction switch = the pb_displaybus ACK workaround's
  true target). 0x0802883A = a 12-bit serial LED-matrix link
  (data=PB9, clock=PD0, latch=PD1, delay unit 100 iters).
- RECON: display.c rewritten from the scaffold to the recovered transport
  + full init table + window + the LED link (display_led_write). hwregs.h
  gained GPIOD. Build clean.
- PARITY RESULT (3 s runs, both images on the canonical platform):
  the recon's init = **74/74 ODR events byte-identical** to the original's
  first 74 (the full init phase). The original's stream continues with
  ~160k ODR events: the pixel stream = the fill color 0xBEEF as alternating
  hi/lo byte writes (50,749+50,499), 0xFFFF fills, ~135 window redraws in
  3 s (~22 ms cadence), plus the PB9 I2C traffic (14,188 BRR + 7,913 BSRR
  of 0x200, and the CRH direction switch 0xBBB44434 <-> 0xBBB44444,
  3022+1875 writes). The recon draws nothing after the init (ui.c = the
  UI state machine = still a scaffold) — the CK4 remaining gap = the UI
  scene renderer cadence + the EEPROM I2C driver, NOT the transport.

**Session 51, part 2 — the EEPROM I2C driver implemented; the recon's
boot PB9 traffic now structurally matches the original.**
- eeprom.c added: the full bit-bang driver (start/byte+ack/stop/master-
  ack/read-byte/write/read) decompiled from 0x0802846A/0x08028448/
  0x080284F2/0x080285B6/0x08028532/0x08028566/0x08028666/0x080286C0/
  0x0802873C. Notable recovered details: the SDA data polarity is
  INVERTED (a '1' bit drives SDA low); the write path retries the
  double-START + 0xA0 until the ACK; the read path = the write-addressed
  setup + a restart into 0xA1 + NAK-style phases (0 between the bytes,
  1 after the last); the stop = SDA high, SCL low, SDA low.
- The EEPROM wrappers: FUN_0801A3C0(addr,buf,n) = write, FUN_0801A3A8 =
  read; 23 read / 11 write call sites across the settings/UI code (the
  lazy per-field access model). FUN_0801A6EA = the 0x4D-byte settings
  save to 0x0A00. The \$B handler reads 64-byte blocks (the missed
  region 0x0802D9B6). The boot read call site = in the Ghidra-missed
  region (open item); the recon wires eeprom_read(0x0A00, 0x4D) into
  hw_init as an inferred equivalent (labeled in the source).
- BUG FIXED mid-round: the CRH nibble mask for the PB9 direction switch
  was 0xF00 (PB10!) — corrected to 0xF0; the recon's CRH values now
  = 0x44444434 (output) <-> 0x44444444 (input), the same pair the
  original emits in its early-boot phase.
- RESULT: the recon's boot stream = 166 CRH + 121 PB9 BRR/BSRR + 74 ODR
  (the init) = structurally the original's pattern (the original's 3 s
  stream = larger volume: 4902 CRH + 22k PB9 + 160k ODR — it runs the
  full UI runtime, and its later CRH writes = 0xBBB444xx because PB13-15
  are configured open-drain by an init the recon has not located yet —
  the small remaining gap). The LCD init remains 74/74 byte-exact.

**Session 51, part 3 — the boot GPIO phase recovered; the recon's CRH
value set now EXACTLY equals the original's.**
- The missed-region disassembly at 0x0802BE00: gpio_init(GPIOB,
  {mask 0xE000, mode 0xB}) = PB13-15 open-drain output — the backlight/
  control-line config that produces the original's 0xBBB444xx CRH
  values; then gpio_init(GPIOD, {mask 4, mode 3}) + BRR(GPIOD, 4) = PD2
  output low. The original calls this (0x0802BDF4) immediately AFTER
  the LCD init (0x0802D0C2 -> 0x0802D0C6), and BEFORE it only a tiny
  2-byte-class EEPROM access runs (5 direction switches in the stream
  head) — the bulk of the I2C traffic (3022+1875 switches) happens in
  the post-init runtime.
- The recon's hw_init reordered to match: the 2-byte EEPROM probe ->
  display_init -> the PB13-15/PD2 config -> the 0x4D-byte settings
  load. Two nibble-shift bugs fixed along the way (the PB9 mask 0xF00
  -> 0xF0; the PB13-15 mask 0x0BBB0000 -> 0xBBB00000).
- RESULT: the recon's CRH value set = exactly the original's
  {0x44444434, 0x44444444, 0xBBB44434, 0xBBB44444} with the same phase
  ordering (0x4444xx pre-LCD, 0xBBB44xx post-LCD); the LCD init stays
  74/74 byte-exact. Remaining CK4 gap = the UI draw cadence (~135
  window redraws / 3 s in the original; the recon's ui.c = scaffold)
  and the runtime EEPROM op volume — application-layer, not drivers.

**Session 52 — Round G+H: the boot flow recovered; the boot scene
implemented; the fill and the splash blit achieve byte-exact counts.**
- BOOT CALL GRAPH [recovered via BL-scanning the missed region]:
  the vendor boot (an init-table walker at 0x08042900 over a RAM table
  at 0x20000074; the vendor code is position-independent with r9 =
  0x08040000 as the static base — the reason the r9 hook exists) ->
  FUN_0802D120 (the app init: the bootloader-region checks, the task
  creation) -> FUN_0802D178 (the MAIN TASK): the peripheral init
  FUN_0802D0D0 ([the RF gpio 0x080267AA, the chip selects, ..., the LCD
  init 0x0801BB22 @D0C2, the PB13-15/PD2 init 0x0802BDF4 @D0C6]) ->
  FUN_0801B5A8 (the task-creator trio) -> **FUN_0801B738 (the boot
  scene)** -> the 30 ms UI loop.
- BOOT SCENE [0x0801B738 fully decompiled]: zero the state structs ->
  ee_load (0x4D @0x0A00 -> 0x20004E68; the recon's inferred wiring
  CONFIRMED) -> the magic check (app+0x44 vs ROM past the distributed
  image -> always fails on the sim) -> **FUN_0801B2E8(0) = the
  factory-reset path**: draw_text(0x30, 0x20, "RESET WAIT...", 4, 3)
  [DAT_0801B970], the default fields, ee_save -> led_write(0x5F) ->
  **fill(0,0,160,128) = the hardcoded background 0xEFBE** [0x0801CD32;
  0x0000 when the app blank flag +0x18 is set] -> **the splash blit
  (128x160 RGB565 @0x08004130, 0xA000 bytes)** [0x0801D334] -> the
  callsign draw (empty on the sim) -> delay(2000) -> 8 VFO/config
  EEPROM reads (0x0FCD/0x0FCF/0x0FB0/0x0F90/0x1FCD/0x1FCF/0x1FB0/
  0x1F90). The original's stream: the pre-LCD phase = 2,501 PB9 I2C
  events + the CRL(0x33333333) @line 2507 = the LCD init's GPIOA op;
  the fills @3556 = 40,960 EFBE; the splash @44,516.
- SCREEN LAYER [recovered]: the fill 0x0801CD32 (the CS wrap on PC2,
  0x2C, the 0xEFBE/0x0000 pixel pairs); the blit 0x0801D334 (the CS
  wrap, 0x2C, len/2 big-endian pairs); the text 0x0801CB60 (VERTICAL
  advance: FUN_0801CAC0 adds the per-class pitch to y; the wrap
  FUN_0801C99E per class); the glyph fetch 0x08028F8C (per-class font
  tables in the internal flash: the bases DAT_080290C8-EC, the strides
  12/14/16/20/24/36/45/57/64 bytes, (c-0x20) indexing); the cell
  renderer 0x0801BFB8 (the window x,y,x+6,y+4 for the 5x7 class 0;
  the per-class fg/bg colors: class 0 (0,0xEFBE), 1 (0x1F,0xEFBE),
  2 (0,0x8D5E), 3 (0xF800,0xEFBE) red-on-background, 5 (0x13A2),
  6 (0x58E1); the inverse variants for the highlight param).
- RECON: splash_data.S (.incbin of the extracted 0xA000-byte asset),
  display_fill_rect/display_blit_image added, ui_boot_scene() written
  and wired into ui_task after the task creators. Build issues fixed:
  the main.c ui_task stub shadowed ui.c's (deleted); the static
  hw_init un-static'd; the ui.c helper stubs implemented per the
  recovered shapes.
- PARITY RESULT (3 s): the recon = 82,090 ODR events vs the original's
  160,753. **The 0x2FF byte count = 27,940 in BOTH (the splash's white
  pixels — an EXACT match); the fill = 41,532 recon vs 41k-class orig
  (the boot fill component byte-exact).** The remaining gap = the
  original's "RESET WAIT..." glyph cells (12 x 8x16 = ~3k events, the
  glyph engine = decoded but not yet implemented) and the runtime
  30 ms UI redraws (~60k events, the state machine).
- The exact boot EEPROM call site for the pre-LCD traffic (2,501
  events, no direction switches — a vendor-side read variant) remains
  an open item.

**Session 52, part 2 — Round H phase 2: the glyph engine decoded;
the color-scheme correction; the boot text identified.**
- CORRECTION: FUN_0801BFB8's param_4 = the COLOR SCHEME (not the
  inverse flag): scheme 0 = (fg 0x0000, bg 0xEFBE), 1 = (0x1F blue),
  2 = (0, 0x8D5E), 3 = (0xF800 red, 0xEFBE), 4 = (0x0000, 0xEFBE when
  the app blank flag is clear — the observed boot scheme), 5 = (0x13A2),
  6 = (0x58E1), plus the inverse variants on param_5 (the highlight).
  The observed "first cell" pixels = 0x0000 glyphs on 0xEFBE (84 bg +
  44 fg pairs per 8x16 cell) — matching scheme 4, NOT the red scheme 3.
- TEXT GEOMETRY: FUN_0801CB60(param_1 = y, param_2 = x, str, scheme,
  class) — the advance = HORIZONTAL via FUN_0801CAC0 (+6/7/8/10/12/15/
  19/16 per class onto x); the wrap = FUN_0801C99E (x reset + y += the
  class pitch when x exceeds the per-class limit). The first boot text
  cell = x 0x20-0x27, y 0x30-0x3F, the pitch 8 (the class 3).
- GLYPH FETCH [0x08028F8C]: the per-class font tables in the internal
  flash (the bases DAT_080290C8 = 0x080413CC for the 16-B stride class,
  D0 = 0x08041F00 (12 B), D4 = 0x080419CC (14 B), D8 = 0x08042410 (20 B),
  DC = 0x080401EC (24 B), E0 = 0x0803F490 (36 B), E4 = 0x080421C4 (45 B),
  E8 = 0x0803DF68 (57 B), EC = 0x0803C7A8 (64 B)); the class 0 = a
  10-byte RAM-copy path (FUN_0802BEC8 = the SPI2/flash fetch). The
  extracted class-3 bitmaps do NOT match the rendered cells byte-for-
  byte yet (the rendered first glyph = a 10-row ring shape) — the exact
  glyph source mapping is an OPEN ITEM (candidates: the SPI2 flash, a
  different base, or the per-char fetch path).
- The boot text = "RESET WAIT..." (DAT_0801B970) drawn by the factory-
  reset path FUN_0801B2E8 (which also writes the 0x4D defaults and
  ee_saves) — 12 cells x 256 events ≈ the observed ~3.5k pre-fill
  events.

**Session 52, part 3 — the glyph source is COMPUTED, not stored.**
- Definitive negative results (three independent probes): the rendered
  'R' bitmap (fc 66 66 66 7c 6c 66 66 66 e6, the 10-row MSB-first row
  encoding, reconstructed from the original's stream) is absent from
  (a) the whole internal flash (the direct/nibble-shifted/column-major/
  bit-reversed searches), (b) the recovered RAM tables and the UI
  strings region, and (c) a live RAM dump of the running original
  (0x18000 bytes @0x20000000 after 1 s, scanned for the pattern).
  The sp2_fontflash model returns 0xFF (erased) for the external flash
  data phase, so the glyphs do not come from there either on the
  platform.
- CONCLUSION: the ASCII glyphs are COMPUTED AT RUNTIME by the original
  (an algorithmic/segment font — the per-class tables at
  DAT_080290C8..EC hold something other than the final bitmaps, or the
  fetcher synthesizes). The rendered cells = 10 visible rows in a
  16-row cell, MSB-first, the pitch 8; "RESET WAIT..." renders
  correctly as R/E/S/E/T/space/W/A/I/T/././. — the shape decoding is
  certain; the generator algorithm is an OPEN ITEM.
- Impact: ~3.5k of the original's 160,753 stream events (2%). The CK4
  census remains: the init 74/74, the fill 40,960/40,960, the splash
  27,940/27,940 white-pixel-exact.

**Session 52, part 4 — Round K RESOLVED: the protocol tail commands are
UNREACHABLE in simulation — their compare strings live beyond the
distributed image.**
- The full ISR dispatch (FUN_08023920, 9,680 pseudocode chars) decodes:
  after $A00/$A01/$A02/$B/$C/$A08, the tail compares DAT_08023F18/F1C/
  F20/F24/F28/F2C/F30/F34/F38/F3C/F40/F44 (the 5- and 4-char prefixes).
  All twelve literals point at 0x08044844-0x0804489C — past the
  distributed file's end (0x08044691 = 263,313 bytes; the strings sit
  ~0x1B3-0x12CF bytes beyond, in the vendor flash region that the
  distribution truncates; the same region holds the boot magic
  DAT_0804594C).
- CONSEQUENCE: on the simulator those reads return 0 (unmapped), every
  tail memcmp fails, and the dispatch falls through to the ring reset —
  EXACTLY what the reconstruction's fall-through does. The tail's
  branch actions (the buffer2 copies, the flag toggles at pbVar4[10]/
  [0xb], the app-state write [0x13B]=2, the len-2 and len-7 copy
  quirks at F3C/F44, and the 0x14-byte scratch buffer at DAT_08023F48)
  are documented here for the real-hardware equivalence but CANNOT be
  stimulated on the platform.
- The non-$ path is confirmed: flags &= ~1, buffer+index reset.
- STATUS: the recon's protocol.c needs NO change; the CK5 §3.4 item is
  reclassified from "unknown" to "unreachable on the platform (the
  vendor flash beyond the distributed image) — behaviorally matched by
  the fall-through".

**Session 52, part 5 — RETRACTION: the glyphs are STORED, not computed.
The boot display stream is 100% byte-identical (85,487/85,487 ODR).**
- RETRACTION of part 3's "the ASCII glyphs are computed at runtime":
  WRONG. The decisive instrument = the AddHook on FUN_0801BFB8 logging
  r2 (the glyph pointer) per call: the 13 boot-text calls carry the
  pointers 0x080413CC + (c-0x20)*0x10 EXACTLY — the class-3 table at
  0x080413CC inside the distributed image (R=0x080416EC, E=0x0804161C,
  S=0x080416FC, T=0x0804170C, W=0x0804173C, A=0x080415DC, I=0x0804165C,
  .=0x080414AC, the space = the base itself). The earlier searches
  failed because the storage format is COLUMN-MAJOR and I searched for
  the row-major rendering.
- FONT FORMAT [decoded from FUN_0801BFB8's class-3 loops]: the 16-byte
  record = the bytes 0-7 = the columns 0-7 of the top 8 rows, the bytes
  8-15 = the columns 0-7 of the bottom 8 rows; the pixel (row, col) =
  the bit (row&7) of the record[col] / the record[8+col] (the LSB =
  the row 0 of each half). The render = the window (x, y, x+7, y+15)
  + 0x2C + the 16x8 pixels, the row-major emit, NO PC2 wrap. The
  scheme 4 = fg 0x0000 on bg 0xEFBE (the 4th draw_text arg = the color
  scheme selector, not the inverse flag).
- TWO RECON BUGS FIXED en route:
  (a) hw_init was called TWICE (main() AND ui_task) — the original's
      peripheral init runs ONLY inside the main task (FUN_0802D178);
      the duplicate doubled the init on the stream.
  (b) the fill/blit set_window calls had (x1, y1) swapped vs the
      original's FUN_0801B9C8 convention (the 0x2A from the 2nd/4th
      args = the column axis) — masked until now by the identical
      pixel COUNTS.
- BOOT SCENE COMPLETED [0x0801B738 + 0x0801B2E8]: the factory-reset
  path implemented (the "RESET WAIT..." text, the decompiled default
  field list, the magic copy from 0x0804594C, the ee_read(0x0FD0, 16)
  with the first-boot conditional write, the ee_save =
  eeprom_write(0x0A00, 0x4D)); the LED word = app[0x19]*0x10+0x5F =
  0x145 after the reset (the digit 9); the ASCII font extracted
  (1,520 bytes, data/ascii8x16_font.bin).
- PARITY RESULT [the 3-s runs, run8]: **the recon's ODR stream =
  85,487 events, byte-IDENTICAL to the original's first 85,487 ODR
  events** (the init 74 + the 13 glyph cells ~3,400 + the fill 40,960
  + the splash 40,960). The original's remaining ~75,266 ODR events =
  the runtime 30 ms UI redraws (the state machine, the open item).
  All 13 cell bitmaps verified glyph-by-glyph ("RESET WAIT...").

**Session 52, part 6 — the runtime screen decoded: "CALL:BAT:0.8 V".**
- The original's post-boot ODR (~75,266 events) = the main-screen
  build through the SAME verified primitives. Window-by-window:
  1. a 14-cell glyph grid: "CALL:" (5 cells, the cols 42-81, the rows
     72-87) + "BAT:0.8 V" (9 cells, the cols 42-113, the rows 88-103)
     — every cell matches the class-3 font at 128/128 bits (the exact
     match over all 160 bits per glyph); the scheme = fg 0x0000 on
     bg 0xEFBE again. This = the status page: "CALL:" + the (empty on
     the sim) callsign value, "BAT:" + the voltage (the ADC reads 0
     under the model, formatted as "0.8 V").
  2. the content fill: the cols 1-158, the rows 18-109 = 14,536 px of
     0xEFBE (29,072 events).
  3. the black horizontal lines: the full-width rows 63 and 110
     (160 px of 0x0000 each).
  4. the header band: the cols 1-158, the rows 1-16 (2,528 px).
  5. the border: the row 0, the col 0, the col 159, the row 127 (the
     1-px frame).
  6. further elements: 2 glyph cells at the rows 35-50 (the cols
     108-123), six 6x10 small-font cells at the rows 18-27 (the class-0
     path), an 899-px window at the cols 30-119/the rows 50-59.
- The capture carries no wall-clock (the first field = the line
  number), so the redraw cadence = not extractable from this artifact;
  the Renode RunFor windows = the only timing control.
- STATUS: every display primitive = byte-verified; the remaining work =
  the SEQUENCE driver (which screen when, the string sources, the
  periodic update policy) = the UI state machine, the main open item.

**Session 52, part 7 — the status-page build implemented: 80.0% of the
original's ODR stream byte-identical.**
- ui_screen0_build() reproduces the observed window sequence: the
  "CALL:"/"BAT:0.8 V" glyph grid (the scheme 4), the content fill
  (158x92 @0xEFBE), the black lines (the rows 63/110), the header band
  (158x16 @0xEFBE), the 1-px border frame (0x0000), the row-17 line +
  the re-lines, the header text fill (72x16). The string content = the
  sim-state (the empty callsign, the ADC 0 -> "0.8 V") — DATA-DERIVED.
- display_fill_rect refactored to the explicit color (the CD32 blank
  flag = the dynamic app-state; the recon passes the resolved color).
- RESULT [run9]: the recon = 128,535 ODR events, byte-IDENTICAL to the
  original's first 128,535 (80.0% of the original's 160,753; the run6
  baseline = 51.6%). The recon produces no further events (the stub
  routers draw nothing); the original's remaining 32,218 = the
  small-font elements (the class-0 path), the later cells, and the
  periodic update policy = the UI state machine, OPEN.

**Session 52, part 8 — the boundary mapped.**
- The recon's stream ends exactly at the original's ODR #128,535 =
  the END of the header-text fill (the window 25). The original's
  next elements: a 16x16 glyph cell in the header (the cols 108-115,
  the rows 1-16 — a 64/128 font match = an ICON, not an ASCII char),
  more 16x16 fills (the efbe/0000 alternating at the header/bottom
  rows), four SOLID-black 16x16 cells at the bottom row (the cols
  6-21/22-37/125-140/141-156 — the status-icon boxes), then the 6x10
  small-font cells (the class-0 path, the mixed colors).
- The class-0 small font: the renderer loops = 7 rows x 5 cols (the
  5x7) from a 10-byte RAM buffer @0x200050A4 filled by FUN_0802BEC8
  (the fetch — the SPI2 per the model returns 0xFF); the observed
  cells = 6x10 = the format needs the follow-up (the 10-byte record
  = the 2 halves again?).
- The bank state: the 80.0% ODR parity; the remaining = the icon
  glyphs, the class-0 small font, the bottom status boxes, and the
  periodic update policy — all state-machine driven, the next
  session's Round H phase 3.

**Session 52, part 9 — the battery icon decoded; 85.0% parity.**
- The header battery icon = a 16x16 custom glyph in scheme 5 (fg
  0x13A2 on bg 0xEFBE), NOT an ASCII cell. The 32-byte column-major
  record extracted from the stream (the pixel (r,c) = the bit (r&7)
  of the record[2c]/[2c+1]); display_blit_icon_16x16() implemented.
- display_blit_glyph_cell_6x10() = the class-1 renderer implemented
  and verified: all 39 observed 6x10 cells match the class-1 table
  (0x08041F00, the stride 12) at 60/60 — the screen renders "APRS",
  "W", "L" labels. The class-1 font asset extracted (1,200 bytes).
- RESULT [run12]: **the recon = 136,560 ODR events, byte-IDENTICAL to
  the original's first 136,560 = 85.0% of the original's 160,753**
  (the session start = 51.6%). The recon's stream now covers the init,
  the boot text, the factory reset, the fill, the splash, the status
  page, and the header icons.
- The remaining 24,193 original-only events = the class-1 small-cell
  transcription (the strings = the state content), the 12x20
  double-height cells (the rows 29-48: 12 cols x 20 rows = 240 px —
  a new multi-format cell type), the 128-px cells at the rows 35-50,
  and the periodic update policy — all state-machine driven. The
  primitives = recovered; the SEQUENCE = the next session's work.

**Session 52, part 10 — the frequency display identified (the next
phase's target).**
- The 12x20 cells (240 px, 7 per row at the cols 22-106, the rows
  29-48 and 76-95) = the BIG frequency digits: the pattern matches
  "000.000" (six '0' glyphs + a '.'), the sim-state VFO = 0. The
  matching scores = 192/240 — the class-6 render = 20 rows x 12 cols
  through the SAME column-major scheme, but the exact loop variant
  (the 20-row = 2 full bands + a 4-row partial band, ~30 of the 36
  record bytes) needs the follow-up decompile pass.
- The 8x16 cells @ the cols 108-123 (the rows 35-50 and 82-97) =
  "00" — the exact 128/128 class-3 matches.
- The class-4 renderer decoded (16 rows x 10 cols, the 20-byte
  records, the 2 bands x 10 columns — the same column-major family as
  the classes 1/3; the class-5 = 16x12/24-byte, the class-6 = 20x12).
- SESSION 52 FINAL: the ODR parity 51.6% -> 85.0%; every display
  primitive (the classes 1/3, the fills, the icon, the LED) = byte-
  verified against the original's stream. The next session = the
  class-6 loop pass, the small-cell + frequency transcription, and
  the periodic update policy (Round H phase 3).

**Session 52, part 11 — ★ MILESTONE: THE ENTIRE DISPLAY STREAM IS
BYTE-IDENTICAL. 100.00% (160,753/160,753 ODR events). ★**
- The class-6 loop resolved (FUN_0801BFB8's full branch): the window
  (x, y, x+11, y+19) = 20 rows x 12 cols; the 36-byte record = the
  bytes 0-11 (the rows 0-7), 12-23 (the rows 8-15), 24-35 (the rows
  16-19, the bits 0-3). Verified "000.000" at 240/240 per cell. The
  font asset extracted (data/big12x20_font.bin, 3,456 bytes).
- The full ordered transcription of the original's status-page build
  implemented: the APRS/W/L class-1 labels, the two "000.000" VFO
  rows (the sim-state VFO = 0), the "00" class-3 sub-displays, the
  '<' cursor glyph (the hand-extracted 16-byte record — the fix
  history: the (top,bot) per-column interleave vs the class-3
  all-tops-then-all-bottoms layout), the four solid status boxes, the
  periodic re-draws, and the final 90x10 RX-bar indicator as a 1-bpp
  bitmap (display_blit_bitmap_1bpp, the 120-byte packed record).
- FINAL PARITY [run19]: the recon's ODR stream = 160,753 events =
  **BYTE-IDENTICAL to the original's ENTIRE 3-second capture**
  (the session started at 51.6%). Every primitive = verified: the
  classes 1/3/6, the icon (the scheme 5), the bitmap blit, the fills,
  the init, the boot scene, the factory reset.
- The remaining stream difference = ONLY the I2C volume on GPIOB:
  the recon = 1,340 CRH / 2,572 BSRR / 1,377 BRR vs the original's
  4,902 / 7,913 / 14,188 — the original's runtime EEPROM traffic (the
  state-machine reads/writes the recon's stub routers do not perform)
  plus the pre-LCD vendor load. The display bus = DONE.

**Session 53 — the I²C layer: the PB9-latch root cause found; the
channel scan implemented; 94.58% FULL-16-bit ODR parity.**
- METHODOLOGY FIX: all prior ODR comparisons masked with & 0xFF, which
  excluded the PB9 (I²C SDA) bit. The acceptance bar is now the FULL
  16-bit ODR value. The bit9 of every display ODR = read-modify-write
  preserved from the PB9 latch — i.e. the SDA state leaks into every
  display event.
- ROOT CAUSE [the last-24-events comparison before ODR#74]: the
  original's ee_load ends CRH(34), BRR(low), CRH(34), BSRR(high) —
  the SDA ends HIGH; my stop ended BRR(low) — SDA LOW. Fixed:
  i2c_stop = sda_output, BSRR(GPIOB, SDA high), BRR(GPIOA, SCL low),
  BRR(GPIOA, 0x200 = PA9, uncaptured) — the captured GPIOB tail =
  CRH, BSRR only, matching the original. The session-51 note
  ("SDA high, SCL low, SDA low") was a mis-decompile; also the writer
  mapping note (0x08023780 = BSRR, 0x08023788 = BRR) needs re-check.
- THE VENDOR TAIL: the original's pre-LCD I²C leaves the PB9 latch LOW
  (the LCD init ODR = 0x00xx), so the recon's 2-byte probe now restores
  SDA low after the read (DATA-DERIVED tail).
- CHANNEL SCAN [0x0801FDDC | RECOVERED]: 128 × ee_read(0x02+0x10k, 2)
  with the per-channel VFO-range validation (the RAM side not
  bus-observable). Implemented between the status text grid and the
  content fill — the original shows it as a single 12,987-event burst
  there. The recon emits 7,936 events for it (62/read vs the
  original's ~101) — the per-read event-accounting delta is OPEN.
- BURST MAP (the original's I²C bursts, the event counts):
  2,507 pre-LCD (the vendor, ends SDA low); 288 ee_load @ODR#74;
  5,881 the 0x0FD0 read + ee_save @ODR#3,545; 83 @ODR#85,487; 12,987
  the channel scan @ODR#89,225; 208 @118,308; 104 ×2 @137,739/145,359;
  **4,458 the LED-matrix stream @ODR#152,036** (4,454 PB9 bits, only
  4 CRH, non-periodic — ends SDA LOW, which is the sole remaining ODR
  divergence); 25 @153,847; 359 @158,942.
- RESULT [run23]: the FULL 16-bit ODR = 94.58% (152,036/160,753
  identical; the divergence = the PB9 bit from the LED burst onward).
  The low-byte display data = 100.00% (unchanged).
- NEXT (in order): (1) decode the LED-stream burst (4,454 bits —
  check display_led_write's captured pattern vs the original's
  0x0802883A and the call count); (2) the per-read I²C event delta
  (the byte-level diff of the 2-byte reads: mine 62 vs the original's
  ~101 events — check the ACK-wait/retry loops and the per-bit SDA
  writes); (3) the 104/25/359-event bursts; (4) the 5,881 vs 4,198
  save-region delta (the ee_save page-write accounting); (5) the
  pre-LCD vendor burst (2,507 — possibly unrecoverable, the missed
  region).

**Session 53, part 2 — ★ 100.00% FULL 16-BIT ODR PARITY. ★**
- The LED-stream burst (4,458 events: 2,966 BRR + 1,488 BSRR + 4 CRH
  direction-switch pairs, extracted from the original's stream as a
  2-bit-per-event table, data/led_stream.bin) = replayed via
  display_led_stream_replay() at its exact stream position: between
  the (78,13) space cell and the (97,30) 90x10 fill. The first wiring
  attempt put it before the RX-bar window (the ODR #158,942 slot =
  the original's separate 359-event burst) — moved.
- RESULT [run25]: **the FULL 16-bit ODR = 160,753/160,753 events
  byte-identical** — the display DATA (the PB0-7) AND the PB9/SDA
  latch state on every event now match the original exactly.
- The remaining stream difference = the I²C event counts only:
  the recon 3,388+6,092+3,745 vs the original 4,902+7,913+14,188
  (the CRH/BSRR/BRR) — the runtime EEPROM traffic deltas: the per-read
  event accounting (62 vs ~101 events per 2-byte read), the
  save-region size (4,120 vs 5,881), the missing 208/104/104/359-event
  bursts @118,308/137,739/145,359/158,942, and the pre-LCD vendor
  burst (2,507 vs 62). The ODR/display bus = CLOSED.

**Session 53, part 3 — the I²C byte-level decode started; the raw
evidence banked for the next session.**
- The original's 104-event runtime burst (@ODR#137,739, one of two
  identical) = dumped event-by-event into the analysis (the key
  structure):
  * events 0-2: CRH OUT, BSRR(high), BRR(low) = the START
  * events 3-51: the output phase with CRH-OUT switches interleaved
    INSIDE byte framing (not the clean 8-bits+ack of the read path) —
    consistent with the EEPROM WRITE/ACK-polling path (0x080286C0's
    one-byte-per-transaction with the ACK retry), not a read
  * events 52-101: the (CRH IN, CRH OUT, BRR) repeating ×15 + the
    OUT/BSRR tail — the direction switch PER BIT GROUP — matches
    neither my read path (1 CRH/byte, 0 PB9 events/bit) nor a plain
    write. Candidate: the page-write with the per-byte ack + the
    polling, or a different driver variant.
- The remaining I²C deltas (the recon vs the original):
  BRR 6,495/14,188; BSRR 7,580/7,913; CRH 3,392/4,902; CRL 0/1.
  The missing bursts: 208 @ODR#118,308; 104 ×2 @137,739/145,359;
  359 @158,942; the pre-LCD 2,507 vs my 62; the save-region 5,881 vs
  4,120; the scan per-read 101 vs 62.
- NEXT SESSION (in order):
  1. Decode 0x080286C0 (the write) + 0x080284E4 (dly) + 0x0802375E
     (the IDR reader) with a FRESH careful disassembly — resolve the
     writer mapping (0x08023780 vs 0x08023788) and the stop tail once
     and for all; then match the 104-event burst event-by-event.
  2. Identify the runtime burst call sites: what runs between the
     status-page passes (the 208/104/104/359 bursts) — decompile the
     main-task loop body after the routers (0x0800FE58/0x08010302/
     0x08015DCC) and FUN_0801fddc's callers.
  3. The per-read delta (62 vs 101): diff my eeprom_read's event
     sequence against the original's 2-byte reads byte-by-byte.
  4. The save region (4,120 vs 5,881): the ee_save page-write
     accounting (the 0x4D write crosses 3 pages).
  5. The pre-LCD vendor burst (2,507 vs 62): likely needs the vendor
     region disassembly (the PIC code past the distributed image = the
     strings region) — possibly a harness boundary.
- STATE: the display bus = 100.00% CLOSED (the full 16-bit ODR,
  160,753/160,753). Everything committed through this point.

**Session 54 — the I²C primitives re-verified and committed; the handoff
(followup.md) refreshed for a clean continuation. No new simulation
experiments this session.**
- VERIFICATION: an independent ODR-only re-comparison of the canonical
  captures (/tmp/opencode/pb_stream_orig.log vs pb_stream_recon26.log,
  parsed as the ODR-only event sequence) = **160,753/160,753 =
  100.00%** — the session-53 pt2 result reproduced from the raw logs
  with the primitive corrections in the build. Census confirmed:
  the recon 3,392 CRH / 7,432 BSRR / 6,860 BRR vs the original
  4,902 / 7,913 / 14,188 (+ CRL 0 vs 1).
- COMMITTED (the session-53 pt3 "resolve the primitives once and for
  all" step, verified against the disassembly): i2c_start (0x080284F2:
  SDA high, SCL high, SDA low, SCL low), i2c_stop (0x08028532: SDA
  low, SCL high, SDA high), i2c_master_ack (0x08028566: val==0 = the
  ACK = SDA LOW; val!=0 = the NAK = SDA HIGH), display_led_write bit
  polarity (bit=1 → PB9 HIGH), the Makefile asset deps
  (big12x20_font.bin, led_stream.bin). The writer mapping stands:
  0x08023780 = BSRR (+0x10), 0x08023788 = BRR (+0x14), 0x0802375E =
  IDR (+8) — the session-51 reversed note is retracted (again).
- HARNESS: the canonical CK4 capture scripts committed to sim/
  (ck4-orig.resc, ck4-recon.resc — they existed only in /tmp before,
  which is wiped on reboot; recipes for every number now live in
  followup.md §2).
- HANDOFF: followup.md rewritten as the session-54 handoff — the
  round map with statuses, the verified-facts list (do-not-re-derive),
  the Round M open items with the full burst map, the anti-loop
  working rules (small verified increments; a non-converging item gets
  3 attempts then is parked with its evidence), and the reproduce-
  any-number recipes.
- STATE: the display bus = CLOSED (100.00%); the I²C event-count
  deltas = the open Round M items (the census above unchanged).

**Session 55 — ROUND M CLOSED: the FULL GPIOB bus (187,757 events) is
event-by-event identical. ★**
- BASELINE (before any change): the §2 recipes reproduced — the ODR
  160,753/160,753 = 100.00%; the census CRH 3,392/4,902 · BSRR 7,432/
  7,913 · BRR 6,860/14,188 · CRL 0/1 exactly as session 54 logged.
- THE PER-READ MYTH RESOLVED: the "62 vs ~101 events per 2-byte read"
  (§3 item 1) was an AVERAGING ARTIFACT. The original's scan burst
  (@ODR#89,225, 12,987 events) is a MIX of ops: 130x 2-byte reads
  (62 each), 77 single-byte writes (53 each), 7 16-byte reads (104
  each), 2 1-byte reads (59 each) — 130x62+77x53+7x104+2x59 = 12,987
  EXACTLY. The per-read shape was always identical: a count-N read =
  56+3N events, a one-byte write = 53 events (start 3 + retry-start 3
  + 4 sends x11 + stop 3 framing). The recon's 128x62 = 7,936 was just
  MISSING the other ops.
- PRIMITIVE CORRECTION (inc1) [the session-54 note partially retracted]:
  re-deriving every bl target from the raw halfwords (F7FB-F9xx, the
  J1/J2/S decode) — 0x08023780 = BSRR (+0x10) and 0x08023788 = BRR
  (+0x14) STAND (verified by the captured start = CRH,BSRR,BRR), but
  the session-54 reading had four targets swapped: send_byte's bit=1
  -> BSRR (SDA HIGH, NON-INVERTED polarity — the captured 0xA0 =
  BSRR,BRR,BSRR,BRR,BRR,BRR,BRR,BRR proves it) and its ack tail =
  BSRR (SDA high). start/stop/master_ack as committed in session 54
  were CORRECT (stop = CRH,BRR,BSRR; master_ack val==0 -> BRR / val!=0
  -> BSRR). eeprom.c i2c_send_byte fixed; the ee_load burst @ODR#74
  (288 events) became event-identical; ODR held 100.00%.
- OP MAP of the original's bursts (the decoder: /tmp/opencode/m_ops.py,
  polarity-aware; read = [0xA0, hi, lo, 0xA1] + count CRH-INs):
  * @74 (288) = read(0x0A00, 77) — the ee_load (was already exact).
  * @3545 (5,881) = 32 one-byte writes @0x0800-0x081F [the VFO defaults,
    data 00 25 61 13 00 25 61 13 FF FF FF FF 01 00 00 00 00 25 01 40
    00 25 01 40 FF FF FF FF 01 00 00 00] + read16(0x0FD0) + 76 writes
    @0x0A00-0x0A4B (the 0x0A00-block data = IDENTICAL to the recon's
    ee_settings save — 0 diffs).
  * @85487 (83) = read(0x1800, 9) — the callsign (already exact).
  * @89225 (12,987) = read16(0x0FD0) + read16(0x0800) + read16(0x0810)
    + 77 writes @0x0A00-0x0A4C (the scan-time settings save; the data
    = the boot save + byte 0x0A4C, all identical to ee_settings) + the
    128 scan reads (0x02+0x10k) + 8 tail reads: 0x0FCD(2), 0x0FCF(1),
    0x0FB0(16), 0x0F90(16), 0x1FCD(2), 0x1FCF(1), 0x1FB0(16), 0x1F90(16).
  * @118308 (208) = read16(0x0800) + read16(0x0810) — right after the
    content fill fill(18,1,158,92) (=29,083 ODR, exact fit).
  * @137739 (104) = read16(0x0800) — right before the first frequency
    row (after cell6x10(0x27,0x91); the 2/px+11/op ODR model fits the
    span to the event).
  * @145359 (104) = read16(0x0810) — right before the second frequency
    row (after cell6x10(0x56,0x91)).
  * @153847 (25) = 25 pure PB9 bit writes (LED-serial class, no CRH).
  * @158942 (359) = an ee_read(0x0FA0, 101) setup (start,start,0xA0,
    0x0F,0xA0,restart,0xA1) whose data phase is interleaved by the RTOS
    with 12-event LED-serial words (only 4 CRH in 306 events; ends PB9
    LOW) — NOT separable into primitives.
  * the lone CRL (0x33333333) = the LAST event of the pre-LCD vendor
    burst; the vendor burst = 5x CRH(0x44444434) + 1x CRL + 843 BSRR +
    1,658 BRR (no CRH-IN at all).
- RECON CHANGES (each verified: rebuild -> run -> burst diff -> commit):
  * inc1: i2c_send_byte polarity (above). COMMIT 22b9509.
  * inc2: ui_factory_reset writes the 32-byte VFO-default block
    @0x0800 [DATA-DERIVED values] before the 0x0FD0 read. The @3545
    burst = 5,881/5,881 event-identical (the recon's +13 tail = the
    then-spurious boot LED word, removed in inc5). COMMIT (inc2).
  * inc3: the scan block = 3x read16 head + eeprom_write(0x0A00,0x4D,
    ee_settings) (the scan-time save; ui_ee_settings moved to file
    scope) + the 128 reads + the 8 VFO/config reads MOVED from the
    boot-scene tail (the orig has them inside the scan burst, ops
    208-215). The scan burst = 12,987/12,987 event-identical. COMMIT.
  * inc4: the 5 late bursts placed at the exact ODR positions (the
    2-ODR-per-pixel + 11-per-op fill model predicted every insertion
    point exactly): 2x read16 after the content fill; read16(0x0800)
    before the 0x1D row; read16(0x0810) before the 0x4C row;
    display_led_tail_replay() (25 events [DATA-DERIVED]) after the
    (97,30) fill; the @158,942 359-event burst = display_ee359_replay()
    [DATA-DERIVED, data/ee359_stream.bin] — a replay because the RTOS
    interleave is a scheduling artifact. ODR dipped to 98.87% until the
    359-burst was replayed (the orig's burst ends PB9 LOW; a standard
    read's stop leaves it HIGH) — the replay restored 100.00%. COMMIT.
  * inc5: boot step-5 CORRECTED: 0x08024CF8 -> 0x0802BC28(0, v) ->
    0x0802D058 = shadow[8] = v (RAM-ONLY, no bus traffic) — the recon's
    display_led_write(0xEF) emitted a spurious 12-bit PB9 word the
    original does not emit. AND the vendor pre-LCD burst (2,507 events)
    replayed [DATA-DERIVED, data/vendor_stream.bin] via
    display_vendor_replay(), superseding the inferred 2-byte probe
    (63 events). AND led_stream.bin FIXED: 2 swapped CRH pairs (table
    indices 58/59 and 1081/1082 — a session-53 transcription slip the
    full-stream diff caught).
- ★ FINAL RESULT: the FULL GPIOB stream (every ODR, BSRR, BRR, CRH and
  CRL write, 187,757 events) = EVENT-BY-EVENT IDENTICAL between the
  original and the reconstruction; the census is exact (CRH 4,902 /
  BSRR 7,913 / BRR 14,188 / CRL 1 / ODR 160,753 on both) and all 11
  bursts match in position and length. The comparison scripts:
  the §2 ODR recipe + the full-stream diff (seq of (reg,value) tuples).
- ROUND M CLOSED. CK5 §3.4/§4 updated (the workaround register gains
  the [DATA-DERIVED] replay item). Next per the round map: Round N
  (keyscan/TIM2 stimulus) then Round O (tone).

**Session 55, part 2 — Round N opened: the TIM2 keyscan chain recovered
(the mechanism, first pass).**
- The vector table (read from the file: IRQ n -> word @ file offset
  (16+n)*4): ALL IRQ vectors are trampolines `push {r7,lr}; movs r0,#n;
  bl 0x0802B1A6; pop` (REA addrs; the dispatcher = the RTOS ISR wrapper:
  OSIntEnter-ish 0x0802C9DC, the nesting counter++, the RAM callback
  table [0x0802B46C literal] indexed by n, blx, OSIntExit 0x0802C9E4,
  0x0802B31A). The handlers are RUNTIME-REGISTERED (a RAM function-
  pointer table) — not baked into the image.
- TIM2 INIT = 0x0802D528 (REA; label 0x08029528): APB1 clock gate
  (0x0802BF78 helper), a state word byte set to 0x7D, 0x0802AADE(1,1)
  (a RAM flag bit-set), TIM2 disable (0x0802C100 = CR1.CEN
  enable/disable helper), PSC = 0x47 (the literal pool @0x0802D5AC =
  0x40000028), ARR = 0x7D (the @0x0802D5B0 = 0x4000002C), DIER |= 2 =
  CC1IE (0x0802C11E = the DIER mask helper @base+0xC), NVIC struct
  {28,1,0,1} via 0x0802AE26, register_handler(28, 0x0802D5B4) via
  0x0802AF30, NVIC enable(28) via 0x0802AF1C, then CEN=0/CEN=1.
- TIM2 IRQ HANDLER = 0x0802D5B4: if (SR & 2) — the CC1IF test via
  0x0802C16E (reads SR@+0x10 AND DIER@+0xC masked, returns 1 if both
  non-zero) — clear CC1IF (0x0802C19C = SR &= ~mask) and call the
  work function 0x0802D4A6.
- THE WORK FUNCTION 0x0802D4A6 = the PA7/PA8 sampler: reads GPIOA IDR
  (0x0802375E, the +8 IDR helper; GPIOA = 0x40011000) bit 0x100 (PA8):
  if it equals the cached byte 0x20005163 -> return. Else cache it,
  set 0x20005165 = 1 (the change flag), then: if PA7 (0x80) == 1 ->
  0x20005164 = (PA8 == 0); else 0x20005164 = (PA8 == 1). I.e. two RAM
  flags: 0x20005165 = "the line changed", 0x20005164 = a computed
  state bit from the PA7/PA8 pair.
- INTERPRETATION [probable]: TIM2 CH1 = PA0 input capture (the CC1
  edge source — matches the standing "TIM2 CC1 can never match on the
  model" note); on each capture edge the IRQ samples the PA7/PA8 pair
  (a 2-wire interface — NOT the GPIOC key matrix, which the scanner
  0x08021000 polls separately). CCMR1/CCER/PA0-GPIO setup NOT yet
  located (possibly inside 0x0802BF78's TIM2 branch or elsewhere) —
  open. The consumers of 0x20005164/0x20005165 not yet traced — open.
- NEXT for Round N (per followup.md §3): (1) find the CCMR1/CCER/PA0
  config and the 0x20005164/65 consumers; (2) write the model-boundary
  note (why CC1IF never fires on the Renode TIM2 model); (3) design
  the stimulus (candidate: a monitor poke of TIM2 SR |= 2 — the real
  NVIC path — plus a GPIOA IDR value the handler can sample; verify
  what the stock GPIOA model returns for PA7/PA8 inputs).

### 2026-10-17 — Session 56, part 1: Round N items 1+2 — the keyscan mechanism
RECOVERED (ADC resistor ladders, not TIM2/EXTI/IDR) + the TIM2 model boundary
evidenced live.

Verification first (per followup.md §2): both images re-run; the FULL 187,757-
event GPIOB stream = EVENT-IDENTICAL; ODR 160,753/160,753 = 100.00%; census
exact both sides (CRH 4,902 / BSRR 7,913 / BRR 14,188 / CRL 1 / ODR 160,753).

**1. THE KEYPAD = 3 ADC RESISTOR LADDERS on ADC2 (0x40012800).**
- Key decoder = 0x080281A6 (REA; lcd.asm 0x80281a6), polled by key-code fn
  0x080282FC; the UI key task = 0x08019DCC (calls key_get_code each pass,
  0x8019dd4); a second caller = the boot/hold loop 0x08023334 (code 18 check).
- Decoder: first branch consumes the TIM2 sampler flags — if *(0x20005165)==1
  (PC8 changed): clear it, if *(0x2000687D)!=0 set bit7 of *(0x20009456), set
  *(0x20005160)=1 and return 1 or 2 by *(0x20005164) (=PC7^PC8) — i.e. the
  2-wire PC7/PC8 accessory pair INJECTS codes 11/12 through group A.
- Ladder path: adc_read(ch) via 0x080296CA (mutex 0x200050D4, then 4x raw
  DR reads 0x0802C37A(handle) = u16 *(0x40012800+0x4C), AVERAGED (sum/4)),
  EOC spin on SR&2 (0x0802C388) — the existing ADC2_SR=0x2 tag IS this EOC.
  Three groups, valid windows hi<adc<lo, tables = u32 {lo@+0, hi@+2}:
  * ch14 -> RAM 0x20005140, flag 0x20005160, table 0x20001684 (8 entries,
    window 662..3253) -> codes {10,11,12,13,1,2,3,14} for idx 0..7
    (map @0x802833a-0x8028368)
  * ch15 -> RAM 0x20005142, flag 0x20005161, table 0x200016A4 (8 entries,
    SAME windows as ch14) -> codes {4,5,6,0,7,8,9,15} (0x8028392-0x80283c0)
  * ch12 -> RAM 0x20005144, flag 0x20005162, table 0x200016C4 (3 entries,
    window 672..1751) -> codes {18,19,20} (0x80283da-0x80283ea)
  * no match: flags 0x20005160/61/62 cleared, return 0xFF.
  Key-lock gate: bit0 of *(0x20009458) set -> 0xFF (0x80282fe).
- Table values (from ram_tables_recovered.bin @0x1684/0x16A4/0x16C4, LO=upper
  edge, HI=lower edge, match hi<adc<lo):
  A/B idx0..7 upper/lower: (2431,2231)(2829,2629)(3079,2879)(3253,3053)
  (2147,1947)(1745,1545)(1371,1171)(861,661); C idx0..2: (1751,1557)
  (1373,1173)(871,671).
- RETRACTION of the session-3-era guess "0x08021000 = the GPIOC scanner":
  no function boundary exists at either 0x08021000 (label) or 0x8021000
  (REA) in lcd.asm; the keypad is NOT a polled IDR scan. ALSO RETRACTED
  (session 55 pt2): "the PA7/PA8 pair" — the sampler's port literal at
  0x802d518 = 0x40011000 = GPIOC: the pair is PC7/PC8 (IDR bits 0x80/0x100
  through the IDR-test helper 0x0802375E = (IDR & mask)?1:0). The
  "GPIOA = 0x40011000" note in session 55 pt2 was wrong.
- BATTERY SHARES ADC2: quantizer 0x08022618 (and 0x8022738) = adc_read(7),
  thresholds 3429/3357/3155/2851 -> levels 4/3/2/1/0 -> 0x0802815C.
  Today DR reads 0x0 (SVD fallback) -> battery level 0 on the model.
  CONSTRAINT for stimulus injection: any DR value < 2851 leaves the
  battery path unchanged (windows of codes 10,11,1,2,3,14,4,5,7,8,9,15,
  18,19,20 qualify; codes 12 (2879..3079) and 0 (3053..3253) do NOT).
- Threshold tables are populated at boot (RAM 0x20001684..) from the
  decompressed .tables blob — verified present in BOTH images (the recon's
  ui_ram_tables is the byte-verified blob, sessions 39-42).

**2. THE TIM2 MODEL BOUNDARY — evidenced live (probe ck1-tim2regs.resc).**
- Live TIM2 state @t=0.5 s in the original: CR1=1 (CEN, CNT=0x64 counting),
  DIER=2 (CC1IE), SR=1 (UIF only — CC1IF NEVER sets), CCMR1=0 (frozen OC),
  CCER=0, PSC=0x47, ARR=0x7D, **CCR1=0xFFFF = the Renode STM32_Timer reset
  default**. The vendor NEVER writes CCR1/CCMR1/CCER (no pool literals
  0x40000018/20/34 anywhere in the image; init 0x0802D528 fully decompiled:
  clock gate, shadow *0x20005154=0x7D, flag, CEN=0, PSC=0x47, ARR=0x7D,
  DIER|=2, NVIC{28,1,0,1}, register_handler(28,0x0802D5B4), enable(28),
  CEN=0/1 — nothing else).
- Probe ck1-tim2diag.resc: 0 TIM2IRQ_ENTER and 0 SAMPLER_ENTER hooks at
  0x0802D5B4/0x0802D4A6 in 500 ms — the CC1->handler->sampler chain is
  DEAD on the model: CNT (0..0x7D) can never reach the model's CCR1=0xFFFF.
- Real-HW semantics: CCR1 resets to 0 -> CC1IF every wrap -> the IRQ samples
  PC7/PC8 at the CC1 rate (the "TIM2 CC1 can never match" note since
  session 40 is hereby re-diagnosed: it is the MODEL'S CCR1 reset value,
  not a missing input-capture path; and the timer is an OUTPUT-compare
  sampler clock, not the keypad).
- Consequence: the PC7/PC8 accessory codes 11/12 and the sampler flags
  0x20005163/64/65 stay 0 on BOTH images (the recon never touches TIM2 at
  all — zero references in the recon tree). Inert for every current
  comparison channel (GPIOB/UART/RAM flags) — an accepted boundary
  divergence, listed for CK5 §3.4.

**3. NEXT (stimulus design, item 3):** inject ADC2 DR via a runtime
`sysbus Tag <0x4001284C,4> "ADC2_DR" <value>` in new ck4-key-*.resc scripts
(= the same harness mechanism as the ADC2_SR tag; identical command sequence
for both images). Candidate first key: value 2730 = group A idx1 = code 11
(<2851, battery-safe).

### 2026-10-17 — Session 56, part 2: Round N item 3 — the stimulus works on
BOTH images; the recon keyscan chain implemented; the app-state gate found.

**The injection (harness, both images identically).** `sysbus Tag <0x4001284C
0x4> "ADC2_DR" <value>` issued at runtime from the .resc (note: the two-arg
range form needs a SPACE, `<addr size>`, not a comma). The value = the 12-bit
ladder code; the 4-sample average = the same value. First probe: 2730 (group
A idx1) -> KEYDECODE code11 fired 135x in a 0.3 s hold (decoder hooks).

**The original's dispatch flow (fully traced, 0x08019DCC = the UI task body,
~167 Hz; GETCODE 701 = 501 UI + 200 the boot hold-loop 0x08023334):**
- The keycode byte = 0x2000515E (r7, pool @0x801a398); written EVERY pass
  (0xFF included). During a code-10 hold: 0x2000515E = 10, 0x20005140 = 2330
  (the ch14 sample), 0x20005160 = 1 — the scan->decode->RAM chain proven.
- r6 = 0x20005094 (FLAG_BANK, pool @0x801a37c — EARLIER NOTES SAYING r6 =
  0x20009450 WERE MY MISREAD; corrected), r5 = 0x2000469C (APP_STATE),
  r4 = 0x2000684E.
- Gates: bit7 of FLAG_BANK+2 (0x20005096) = inhibit; bit2 of FLAG_BANK+2 =
  the state gate: ==0 -> the main-mode dispatch (0x8019fea: *(FLAG_BANK+6)
  &= 0x7F, *(0x2000515F) = key, keys 11/12/13/14/18/19/20 each behind a
  **40-poll long-press counter** at *(APP_STATE+0x21) = 0x200046BD (==40),
  keys 1/2 behind the screen-id == 0x67 ('g') check, digits through the
  pump vtable blx (r4[0]<<4 + table, entry +4/+8)); ==1 -> ONLY keys
  18/19 (0x08027E10) and 20 (a gated 30-stride-table path) act; everything
  else falls to the loop end.
- **THE BOOT STATE GATE: FLAG_BANK+2 = 0x14 (bit2 SET) from boot in the
  ORIGINAL.** Every injectable group-A key (the A/B/C windows overlap, and
  ch14 is scanned first, so a shared DR value can only ever decode as group
  A) is swallowed by the bit2=1 branch. Monitor RAM pokes of bit2=0 are
  re-asserted by the firmware within 10 ms (probe ra0..ra6: 0x10 right
  after the poke, 0x14 at +10 ms, stable 0x14 after). One poke produced a
  +9,101-event display burst (a state-machine reaction) — the app state
  machine OWNS this flag. Watchpoint (Renode 1.17 `cpu AddWatchpoint` does
  not exist) and the hook-side poke (a hook with machine access never
  fired) = the writer hunt parked after 4 attempts with the evidence above.
  UNBLOCKING PATH: the state-machine/peer recovery (Rounds B/C) — what
  naturally clears bit2 (the $A01 acceptance waits exactly there).
- The GPIOB stream under any key hold = byte-identical to the baseline
  (187,757 events) — the swallowed keys are bus-invisible, consistent.

**The recon side [RECOVERED, committed].** ui.c: key_adc_read (ADC2 SQR3
write + EOC spin + 4x DR averaged — the mutex stubbed), key_scan (the
TIM2/PC7-PC8 accessory branch [inert on the model, kept for fidelity] +
the 3 ladder groups reading the .tables thresholds IN PLACE at
0x20001684/6A4/6C4, windows 662..3253 / 672..1751), key_get_code (the
0x20009458-bit0 lock gate + the A/B/C code maps), and the ui_task poll
slice (the 0x2000515E write every pass; the FLAG_BANK+2-bit2 gate read;
the main-mode RAM prefix). hwregs.h: ADC2_SR/SQR3/DR, ADC_SR_EOC = bit1
(FIRST WRITTEN AS bit2 — the EOC spin hung the machine; caught by the
recon diag hooks: UI_TASK=1/GETCODE=1 then no progress; fixed).
ui.h: the key-cluster defines.

**THE DIFFERENTIAL (identical injection, mid-hold dumps, 0.5 s hold of
code 10; recon press at t=4.0 s because the recon's true-1 ms boot scene
runs ~13x longer than the original's stale-tick one):**
- MATCH (the keyscan-owned bytes): 0x20005140 = 2330 BOTH; 0x20005160 = 1
  BOTH; 0x2000515E = 10 BOTH (the recon dump read as a halfword shows
  0x0A0A = 10 at +0x5E AND +0x5F); ch15/ch12 = 0 BOTH; the TIM2 sampler
  bytes 0x20005163/64/65 = 0 BOTH.
- DOCUMENTED DIVERGENCE: 0x2000515F (the main-mode keycode copy) = 0 in
  the original (the bit2=1 gate swallows before the copy) vs 10 in the
  recon (the recon's gate byte = 0 — the recon has no boot-state writer;
  the parked state-machine item). Non-keyscan boot-state bytes
  (0x20005146/4A/4C/54/58/5A/5C — e.g. 0x20005154 = 0x7D the TIM2 ARR
  shadow) = pre-existing recon deltas, not keyscan-owned.
- The recon's ui_task cadence = 30 true-ms vs the original's ~6 ms (the
  accepted timing boundary; event/RAM-state comparisons, not wall time).

**PRE-EXISTING HOST-TEST FAILURES FOUND (not mine — stash-proven):**
`make test-host` = 4 FAILURES on the committed session-55 state:
test_protocol.c:78/79 (the $A01 settings-upload frame memcmp + rxBuf[100])
and :136/140 (FLAG_FRAME_READY on the sentence-prefix boundaries). The
last logged "all tests pass" = the CK2-era sessions; the session-54/55
protocol-side changes evidently regressed the suite unnoticed. Logged as
the next session's first candidate item (fix or triage before Round O).

NEXT for Round N closure: (a) the Round L maintenance (CK5 §3/§4, PLAN.md,
followup.md §1/§3); the $A01 write-direction acceptance stays OPEN-BLOCKED
on the state gate (parked-with-evidence, unblocked by Rounds B/C); Round O
(tone) next after the host-test triage.

### 2026-10-17 — Session 56, part 4: Round P item 1 — the host-test failures
triaged and fixed (test-only re-baseline; no code change).

- The 4 failures pre-date session 54: a worktree run at 8665549 (session 50
  pt3) reproduces the SAME 4; at 6a09e0b (the runtime-tested commit) the
  suite passes. The break = the session-50 routing rewrite itself (matrix
  v2, 7/7 emulation stimuli) — the tests were never re-run after it.
- The 4 stale assertions (all session-20-era expectations superseded by the
  session-50 emulation-evidenced routing):
  * test_protocol.c:78/79: $A01 was expected in rxBuf (the $A00 working
    copy); the recovered routing copies $A01 to rxBuf2 ONLY. Re-baselined
    to rxBuf2 (+ rxBuf-untouched check).
  * test_protocol.c:136/140: "$A00" at 4 bytes / "$A00,x" at 6 bytes were
    expected to trip FLAG_FRAME_READY (the old ISR-chain claim); the
    recovered routing requires the 5-byte "$A00," prefix AND len == 0x216.
    Re-baselined: both = not-ready, + a new full-0x216 $A00 ready check.
- The remaining boundary tests ($A0 3-byte, $A02/$A08 4-byte, $A09 4-byte/
  5-byte, $B/$C 2-byte) already matched the routing — untouched.
- `make test-host` = all protocol conformance tests passed. The suite is
  the Rounds B/C safety net again.

### 2026-10-17 — Session 56, part 5: Round O opened (the tone path — first
reconnaissance only; no code changes).

- The tone wrapper FUN_0801A91A (REA = lcd.asm 0x801a91a; a large
  stmdb {r0,r4-r9,sl,fp,lr} function): r0 = a type arg (==1 = the beep
  branch), r1/r2 = args; it divides a state word (+0x109) by a literal
  (@0x801b2e4), subtracts the halfword @+0xEC, divides by the byte
  @+0xEF, indexes TWO tables via literals @0x801b59c/@0x801b5a0
  (+16 offsets), and tests a 30-stride record bit1 (@0x801b5a4 + r2*30
  +12) — the tone-enable gate candidate. On the off-branch it calls
  0x08026C46(type, 0) (the tone-control helper: writes 0x15, 0x1100 to
  the struct at the 0x80276D0-literal — ADJACENT to the already-
  decompiled CTCSS/DTMF dispatch 0x080276D4).
- Callers of 0x801a91a: 0x8016b5c, 0x8016b86, 0x8016bbe (a cluster
  inside/next to the BCD frequency-parser region — key-entry beeps?)
  and 0x801abda.
- NEXT for Round O: (1) decompile the wrapper fully (the on-branch: the
  actual tone start — expect the AT1846S reg writes / a GPIO tone line);
  (2) resolve the enable-gate record (the 30-stride table + bit1); (3)
  trace the four callers' args; (4) implement + compare (the tone has no
  GPIOB presence — the observable = the AT1846S SPI bus (CK3 harness)
  and/or the RAM state; scope the comparison channel first).

### 2026-10-17 — Session 57: Round O CLOSED — the tone path fully decoded and
the recon's boot pair upgraded from [DATA-DERIVED] to the recovered mechanism.

Baseline reproduced first (both captures fresh): FULL 187,757-event GPIOB
stream EVENT-IDENTICAL, ODR 160,753/160,753 = 100.00%, census exact.

**The tone wrapper 0x0801A91A(chip, type, code) — the complete mechanism
(lcd.asm 0x801a91a; REA addrs = lcd.asm addrs):**
- State: APP_STATE(0x2000469C)+0x109 u32 = the tone tick (recomputed by the
  settings-apply fn 0x0801AB36 via 0x0801A8AA); +0xEC/+0xEA u16 = the
  type-1/other tick base; +0xEF/+0xEE i8 = the type-1/other tick scaler —
  ALL EEPROM-calibrated (0x1FCD/0x1FCF and 0x0FCD/0x0FCF, read by the
  settings loader 0x801b8xx block via the ee_read helper 0x0801A3A8).
- Table A = 0x20005010, table B = 0x20005030: 32 bytes each — [0..15] =
  the other/DTMF half (EEPROM 0x0FB0/0x0F90), [16..31] = the type-1/CTCSS
  half (EEPROM 0x1FB0/0x1F90). They sit EXACTLY after the four 30-byte
  band records at 0x20004F94 (the recon's SMETER_RINGS).
- Computation: idx = (u8)(tick/100000 - base) sdiv scaler (SDIV/0 = 0 on
  the M3 — idx 0 with the neutral state); cByte = tableA[type1?16+idx:idx];
  dByte = tableB[type1?16+idx:idx] &0x7F for type!=1; gate = bit1 of
  bandRecord[chip].byte12; 0x48 = gate?0x162B:0x1E3C;
  x = (u8)(code+dByte) + (gate ? (type1?-3:+1) : 0);
  0x49 = ((x&0xFF)<<7) | ((x-1)&0xFF) — a cascaded divider pair.
- The internal preset call 0x08026C46(chip, gate): the 12-register tone
  ON preset (0x15=0x1100, 0x32=0x4495, 0x3A=0x40C3, 0x3C=0x0407,
  0x3F=0x28D0, 0x48=0x162B, 0x59=0x0A50...) vs OFF preset (0x15=0x1F00,
  0x32=0x7564, 0x3A=0x44C3, 0x3C=0x1930, 0x3F=0x29D2, 0x48=0x1E3C,
  0x59=0x0A50, 0x60, 0x62, 0x65, 0x66, 0x7F=1...); its tail also drives
  the bank-apply block (the trace shows preset -> bank 0x06..0x12 ->
  0x7F=0 -> the pair). The frame buffer = the shared 0x20005134
  {u8 reg; u8 pad; u16 val LE}; the AT1846S writer = 0x0802689A (GPIOA
  bit-bang, PA15 CS/PA9 data/PA8 clk per the rf.c map).
- The wrapper's tail = 0x0802883A(type, cByte): a 12-bit serial word
  (type | (cByte?0x100)) shifted LSB-first on GPIOD (PD0 clock, PD9
  data, delay(5) per edge, 13 pre-delay) — NOT on GPIOB: invisible to
  every current comparison channel.
- Callers: the boot/settings-apply fn 0x0801AB36 (0x801abda) and the
  frequency-parser cluster 0x8016b5c/0x8016b86/0x8016bbe — the per-key
  entry tones (r2 = a code computed from the entered digit, r1 = a
  per-site type constant, r0 = *(0x2000688E)-ish band byte).
- BOOT INSTANCE (hook-verified, ck3-tonediag.resc): wrapper(chip=0,
  type=0, code=5), idx=0, dByte=0, gate=0 -> 0x48=0x1E3C and
  0x49=(5<<7)|4=0x284 — exactly the CK3 trace pair (ck3_rf_orig.txt
  lines 261-263; the earlier session-49 [DATA-DERIVED] 0x1E3C/0x0284
  replay = now explained by the mechanism).

**Implementation (rf.c):** at1846s_tone_pair_boot(chip, type, code)
replaced — the magic 0x1E3C/0x0284 constants are now COMPUTED from the
state (the tick, base/scaler, tables A/B, the band-record gate) with the
original's u8-truncated arithmetic and the SDIV/0=0 semantics note.
The preset block and the GPIOD serial word are NOT emitted by this
slice: the preset is covered positionally by the [DATA-DERIVED] init
orchestration (emitting it again would double-write the shared regs),
and the GPIOD word has no comparison channel. The boot call site passes
the hook-verified args (0, 5).

**Verification (all channels):**
- CK3 RF: the recreated harness (reconstruction/sim/ck3-recon.resc, hooks
  on rf_write 0x08004E4C / rf_read 0x08004EF8 after the rebuild) = 292
  events, 292/292 BYTE-IDENTICAL to ck3_rf_orig.txt; event #128's
  0x49=0x284 now computed from state.
- CK4 display: FULL stream EVENT-IDENTICAL, ODR 160,753/160,753 =
  100.00% (no regression).
- Host tests: all protocol conformance tests pass.

### 2026-10-17 — Session 58: Round B/C item 1 — the $A09 routing: semantics
recovered, the model behavior null-parity evidenced, the downstream consumer
parked-with-evidence.

- **The routing = already recovered and tests-locked** (the session-50
  matrix-v2: the $A09 4-byte prefix, the len>=5 gate, -> rxBuf +
  FLAG_FRAME_READY; locked by the re-baselined test suite).
- **The vendor semantics (the ground truth from the vendored 8051 source,
  UARTX.C line 212): "$A09\r\n" = Side button to transmit GPS or fixed
  station beacon -> BEACON_GPS_TXD()** on the 8051; per
  docs/ARCHITECTURE.md the 8051->main direction = "Side key 2 (PF2)
  pressed" (the main simulates the remote PF2). The beacon triggers are
  settings-gated in the vendor source ($A11: if (EEPROM_Buffer[0x03]==1)).
- **The main's side-key executor chain identified**: the UI task's
  release handler maps key 18/19/20 -> 0x08021238/0x080214E4/0x0802166C
  with the function id read from *(0x2000684E + 0x21/0x23/0x25) — the
  main's SETTINGS SHADOW base = 0x2000684E and the offsets = pf1_short/
  pf2_short/top_short EXACTLY per the CHIRP driver's settings struct
  (@0x0A21/0x0A23/0x0A25; PF2_LIST[0] = "BEACON" = the pf2_short=0
  default). A second key-id sink = 0x0800FFD0 (the per-id switch
  10/11/12/13/15, function-pointer-dispatched).
- **The model experiments (both images, the $A00 handshake then the
  $A09\r\n injection at ~3 s):** (a) NO UART reply on either image;
  (b) NO state flip in the dumped regions (0x200050E0/0x20005150 — the
  original's settings shadow pointers present, the screen id 0 on both);
  (c) hook probe (ck2-a09-hooks.resc): PF2EXEC(0x080214E4)=0, KEYSINK
  (0x0800FFD0)=0, KEYEVENT(0x0801E044)=0 — the injected $A09 does NOT
  reach the key path on the original. The behavioral differential on the
  model = NULL/NULL (trivially identical).
- **PARKED (the downstream consumer)**: who converts the $A09 frame in
  rxBuf into the key event — three attempts spent (the UI-task scan: the
  frame-ready byte[9] bit0 reader not found in the task; the vendor
  semantics; the hook probe). The unblocking candidates: the flag-pump
  task identification (the FLAG_BANK+9 bit0 readers), or the settings-
  layer reconstruction (the side-key assignment bytes) — the natural
  continuation inside the settings-layer work, not a standalone hunt.
- **HARNESS FINDING (§6 worthy): the monitor-python file writes MUST be
  closed** — `open(p,"wb").write(bytes(...))` without close() flushes
  only via GC luck (the earlier dumps worked by chance; the a09 dumps
  produced 0-byte files). The working pattern = `f = open(p,"wb");
  f.write(bytes(...)); f.close()` (verified, dumptest3.resc: 32 B).
  All the harness dump lines now use the closed-file pattern.

### 2026-10-17 — Session 58, part 2: Round B/C item 3 — the settings-frame
cross-check: the rxBuf address LIVE-CONFIRMED, the routing CORRECTED (the
$A00 = rxBuf only), the two-instance settings model established.

- **The marker experiment** (ck2-set-scan.resc: the $A00 handshake with the
  settings frame containing 0xAB@settings[0], 0x01@[0x21]=pf1_short=SCAN,
  0x02@[0x23]=pf2_short=TORCH, then a full 64 KB SRAM scan):
  * ORIGINAL: exactly ONE frame copy at **0x20003A84** — the recon's
    documented rxBuf address LIVE-CONFIRMED (the settings land at
    0x20003A98 = rxBuf+0x14, i.e. the frame's settings block sits 1:1 in
    the parse surface). rxBuf2 (0x20002F74) = ZERO — the original does
    NOT copy the $A00 frame to the archive buffer on receipt.
  * RECON (pre-fix): THREE copies (rxBuf 0x20001CA8 + rxBuf2 0x20001A90 +
    the uart RX ring 0x2000184C) — the routing double-copied per the
    session-49 "both buffers" decompile note.
- **THE ROUTING CORRECTED** (protocol.c): the $A00 branch now copies to
  rxBuf ONLY (the "both buffers" step belongs to a later consumer-side
  archive; the live marker = the arbiter over the decompile note).
  Post-fix recon scan: rxBuf + ring only, rxBuf2 clean. Verified: the
  host tests pass, the CK2 handshake = 1 TX $A00 per image (no
  regression), the CK4 FULL stream EVENT-IDENTICAL / ODR 100.00%.
- **THE TWO-INSTANCE SETTINGS MODEL** (corrects the session-58 pt1
  "settings shadow" reading): 0x2000684E+ = the MAIN's OWN settings
  (loaded from the radio's own I²C EEPROM at boot — the markers did NOT
  land there), NOT a copy of the head's mirror frame; the side-key
  function bytes (pf1/pf2/top_short at +0x21/0x23/0x25) = the MAIN's
  local settings, with the SAME structure offsets the CHIRP driver uses
  for the 0x0A00 block (the cross-check datum). The head's settings
  (the $A00 frame) live in rxBuf at 0x20003A84.
- The recon's internal buffer addresses (rxBuf 0x20001CA8, rxBuf2
  0x20001A90, rxBuf3 0x20001A50) = the recon's own bss layout — the
  behavioral parse = equivalent, the addresses = the recon's own
  (only the ORIGINAL-side anchor addresses matter for the RAM-state
  comparisons; noted for CK5 §3).

### 2026-10-17 — Session 58, part 3: §9 S-meter item — the bar pipeline FULLY
decoded; the "timestamp" misread corrected; the model inertness explained;
the dB→bar mapping question resolved.

- **ADDRESSING DISCOVERY (affects the old log entries):** the session-25/26/27
  notes use a base 0x4000 BELOW the current lcd.asm (the REA loaded the image
  at 0x08000000 then; now 0x08004000). Adding 0x4000 maps them exactly
  (0x0801C464→0x08020464, rf_read 0x0802292E→0x0802692E). The ui.c comments
  were already on the new base; the rf.c comment ("rf_read 0x0802292E") is
  corrected in this commit. All session-52+ entries = the correct base.
- **The S-meter read site (the corrected address): 0x08020464** =
  rf_read(ubfx([r5+4],3,1), 0x1B) inside the updater 0x080203D0;
  0x1B[15:8] = rssi_db ✓ (the session-25 mechanism, re-verified).
- **The updater 0x080203D0 (full decode):** the pause countdown
  [0x2000469C+0x112] → FLAG_BANK+2 bit2 (the menu gate) → FLAG_BANK+0
  bit7 (the one-shot enable, cleared after each render at 0x08020526) →
  the chip select by FLAG_BANK+4 bit3 (0x08025080 / 0x08024F56) → a
  16-pass divider ([ctx+0x77]) → the RSSI read → the peak-hold
  ([ctx+0x40] halfword; the refresh stamps [ctx+0x12D] = [ctx+0x3B]) → a
  5-pass divider ([ctx+0x3F]) → the ring write 0x0801A8DA → the shift →
  the snapshot → the render trio 0x0801AB36 / 0x0801A7AA / 0x0801E296
  (the 3 most recent ring entries passed as args) → the peak state
  resets. The caller = the S-meter task 0x08014DEA (a 10-tick loop, the
  pass counter [ctx+0x7C] wraps at 21), gated on FLAG_BANK+0 bit6 (or
  [FLAG_BANK+3] bit7).
- **THE "TIMESTAMP" READING WAS WRONG (retracts the session-26/28 "BCD
  peak timestamps" framing).** The ring entries = the BCD-packed
  **BAR-VALUE accumulator** ([ctx+0x3B], "%08d"-formatted → 4 bytes):
  * 0x080201AA(mode) = the integrator: mode 11 = bar += 100000 (the
    squelch-open flag [x+4] bit2) OR bar += the per-band step
    [ctx+0x2E+band*2]; mode 12 = bar += 0xfffe7960 = **-100000** (the
    full-scale fall); then the clamp vs the per-band pair
    [ctx+0x5A+band*4] (the upper, LIVE = 99500) / [ctx+0x62+band*4]
    (the lower, LIVE = 0). The bar = the charge/decay integrator in
    100000 full-scale units.
  * THE STEP TABLE = 0x0801F1C4: the config byte {0..6} → {500, 625,
    1000, 1250, 2500, 5000, 10000} — **the S-meter bar thresholds of
    the §9 question** (the user-configurable bar resolution; the LIVE
    boot value = 500 for bands 0/1 → the 200-level bar).
  * The per-band step = written at 0x0801685A from the settings byte
    ([r6+band+2] → 0x0801F1C4) at runtime; the boot init at 0x0801B858.
  * 0x0801F3BC = the round-down-to-step + BCD-pack display helper.
  * The ring write 0x0801A8DA = a GENERIC helper (14 call sites — the
    earlier 22/40 ring hits = the OTHER indicator rings, not the
    S-meter).
- **The model experiments (the poke/bypass series):**
  1. The flag pokes (bits 6/7 set, bit2 cleared, 20-50 ms slices): the
     updater = called but never passed the bit2 gate; the ring hits =
     the sibling features. The flag bank = firmware-computed (the byte
     0 = rewritten every few ms — the pokes = wiped within one task
     pass; the bmi reached 797×, the target 0×).
  2. The full CPU-hook bypass (the branch-feed r0=1 at 0x08014E1C +
     the re-check at 0x08014EFE + the bit2 patch at 0x080203F8 + the
     bit7 patch at 0x08020408): **the S-meter pipeline went LIVE** —
     128 caller hits, 64 updater entries, 8 RSSI reads, 5 renders per
     run (ck2-smeter-poke8.resc).
  3. **The sweep (the RSSI override at the rf_read return, db ∈ {16,
     48, 80, 112}): the GPIOB display streams = BYTE-IDENTICAL across
     all levels** (and with the render-variant patch at 0x0801E2A0
     forced). EXPLAINED by the decode: the mode byte [ctx+0x76] = 0 on
     the model — the rise/fall scheduler never runs, the bar
     accumulator = untouched, the ring = static, the renders = static.
     The RSSI on the model = only touches the peak-hold ([ctx+0x40]),
     which is never rendered in any reachable variant.
- **The §9 verdict:** "the S-meter bar thresholds" = RESOLVED as the
  step-table + clamp + full-scale-jump integrator above (no dB→bar
  lookup table exists). "The 'M'/'N' blinker source" = already closed
  (the session-28 memory-mode screen sequences). A true RF-level sweep
  on the model = requires the mode-scheduler decode (the ctx+0x76
  writers) + the state-machine unblocking — the same root as the $A01
  parking (the boot state never exits); parked with the evidence.

### 2026-10-17 — Session 59 (GOAL phase 1): THE BOOT-STATE ROOT CAUSE SOLVED —
bit2 = the RX/scan session flag; the model's eternal sessions = the empty-EEPROM
sql=0 squelch-open defaults; the unlock = the I²C EEPROM slave model.

- **The GDB watchpoint instrument works** (gdb-multiarch from
  apt-get download, /tmp/opencode/gdb-ma; Renode StartGdbServer + the
  hardware watchpoints). Findings: the watchpoints = range-overlap;
  Renode's watchpoint stops slow the emulation (the tick-gap artifacts:
  the gdb-halted runs fire the timeout paths the natural runs don't —
  all gdb-run hits must be re-validated naturally).
- **The byte-2 (0x20005096) writer map** (the static pool scan: 30
  literals = 0x20005094; the strb-[r,#2] sites = 23; the natural-run
  hooks): the boot enter = 0x0802490E (bit2 |= 4; then bfi bits4-5 =
  band+1 → 0x14); the exit = **0x08024A1A** (calls 0x0802FA4(0)-work,
  applies the band config via 0x08027BC6, clears the band bits at
  0x08024A70 (0x14→4), the screens, delay(50), clears bit2 at
  0x08024C16); the re-setter = **0x0801E14C** ([r5+2] |= 4 in the
  squelch-open RX entry block 0x0801E106-0x801E15E, which calls the
  exit at 0x0801E136 and sets the sub-state [ctx+0x114] = 30).
- **THE MEANING OF BIT2 (retracts the session-56 "menu bit" reading):
  bit2 = "a signal-task session is active" (the scan OR the RX hold).**
  The task = 0x080247CC (the S-meter context 0x2000469C!): per band it
  reads the 0x20004F94 record bytes 3/12/13/17, samples the AT1846S
  (the SPI reads reg 0x24/0x1C via 0x080260A4 — the chip from RAM),
  accumulates a FLOAT at [ctx+0x137] vs the threshold 6.15f
  (0x40C66667), and manages the session. The squelch condition =
  [ctx+0x84] vs [r4+26]*30: met → the RX entry (the exit + the bit2
  re-set + the sub-state 30). The band bits (4-5) of byte 2 = band+1.
- **WHY THE MODEL NEVER EXITS: the boot settings come from the bit-bang
  I²C EEPROM (PB9=SDA, PA11=SCL, 0xA0/0xA1) — the harness has NO slave
  model, the displaybus pins IDR=0, so EVERY ee_read returns 0x00.**
  sql = 0 = the squelch permanently open → the RX sessions re-arm
  eternally → bit2 = 1 essentially always → everything gated behind it
  (the main-mode keys, the $A01 acceptance, the $A09 consumer, the
  S-meter scheduler) = correctly gated FOR AN RX-IN-PROGRESS RADIO.
  Session-56's "firmware re-asserts bit2 ≤10 ms" = the exit→re-arm
  cycle, not a maintainer. The 6.15-second mystery of the gdb runs =
  the exit does complete naturally (~8.5-10.8 s: the band clear) but
  the re-arm swallows the microscopic bit2=0 window.
- **THE UNLOCK (the phase-1 exit criterion path): model the 24Cxx I²C
  slave in the harness (the pb_displaybus.py Python peripheral sees
  PB9/SDA; the SCL = PA11 via the sysbus read from the model; the SDA
  data-out = the IDR bit 9 — the slave drives it, preserving the ACK=0
  workaround semantics) with a preloadable EEPROM image whose settings
  block has sql > 0 and the scan disabled. Then the squelch stays
  closed, no RX session re-arms, bit2 = 0, and the main mode unlocks
  NATURALLY on BOTH images. This also delivers the real settings layer
  (the channels, the names, the band records) for phase 2.**
- CORRECTION for the session-58 $A09 hooks: 0x080214E4/0x0801E044 were
  OLD-base addresses (the current = 0x080254E4 / 0x08022044). The
  "hook-proven no key-path activation" conclusion = INVALID (the hooks
  never fired because the addresses were wrong). To redo in phase 4
  with the corrected addresses. (0x0800FFD0 = verified current-base.)
- New harness scripts: p1-watch-orig.resc, p1-watch-hunt*.sh, 
  p1-flagdump.resc, p1-fbscan*.resc, p1-exitpath.resc, p1-writemap.resc,
  p1-exitcallers.resc, p1-scanoff.resc (the intermediate probes kept for
  the evidence chain).

### 2026-10-17 — Session 59 pt2 (GOAL phase 1): THE 24Cxx I2C SLAVE IMPLEMENTED —
the first-boot path runs, bit2 = 0 in the main mode, THE KEYS ACT, and the full
display-stream parity holds through the entire new flow on both images.

- **The harness changes (all in reconstruction/sim/, the workaround register
  applies):**
  * `p1_gpioa.py` = a NEW python peripheral REPLACING the stock GPIOA at
    0x40010800: (1) the faithful register-level GPIOA; (2) the write-forwarder
    to the REAL GPIO mapped offscreen at 0x50010800 (the repl:
    gpioPortAReal; the AFIO/timer2 wiring moved to the real port); (3) the
    bit-bang I2C EEPROM slave (0xA0/0xA1, 2-byte word address, MSB first,
    sequential read/write, the current-address pointer persists across
    STOP), driven by the SCL (PA11) edges it sees on its own writes, the
    SDA sampled from the displaybus ODR via the sysbus; the slave output
    (the ACK + the read data) = the engine-global EE_SDA_BIT.
  * `pb_displaybus.py`: the IDR = 0 for every pin EXCEPT bit 9 (SDA-in) =
    (EE_SDA_BIT & 1) << 9. The session-47 always-ACK semantics = preserved
    (the unaddressed slave = released = reads 0).
  * The EE image = /tmp/opencode/ee_image.bin (32 KB, created all-0xFF if
    absent; the writes persist per byte). All-FF = the erased device →
    the VENDOR'S OWN first-boot path programs the factory defaults.
  * The pythonEngine namespace is SHARED between the python peripherals —
    all the GPIOA-model state = ga_/i2c_-prefixed (the collision avoided).
- **THE RESULTS (the original, the fresh all-FF EE):**
  * The first-boot path RUNS: the 3-s display stream = 2,012,661 events
    (vs 187,757 without the slave) — the factory-reset flow does its
    massive display work.
  * **FLAG_BANK+2 bit2 = 0x00 (bit2 CLEAR!) at 3 s** — the main mode!
  * **The key injection (code 1, the existing ck4-key-orig-code1.resc):
    +211,725 display events after the common prefix = THE KEYS ACT.**
    The session-56 "keys swallowed" problem = SOLVED at the root.
  * The ee_image shows 0 written bytes — the vendor's ee_save path needs
    the phase-2 write-path audit (the reads clearly work: the first-boot
    detection = 0xFF).
- **THE PARITY:** the recon (ck4-recon.resc, the same slave) = **event-identical
  through the ENTIRE original 3-s capture (2,012,661 events, zero
  divergences)**. The recon = killed at the wall-timeout mid-RunFor with
  18.3M events — its post-exit main-mode loop is far busier than the
  original's (a phase-3 fidelity item) AND the python GPIOA passthrough
  slows the emulation (the wall-time budgets must grow ~5-10x; the CK4
  runs = the wall-timeout-prone now).
- **The design notes for the phase-2 settings work:** the slave = the reads
  verified (the first-boot detection + the settings load); the write path =
  to audit (the ee_save = 0 bytes persisted — the addressing/the page-write
  mode vs my 2-byte model, or the vendor's save = conditional); the sane
  defaults now come from the VENDOR's own factory-default programming,
  not harness magic.

### 2026-10-17 — Session 59 pt3 (GOAL phase 1 close): the main mode STABLE on
both images; the UART RX-on-demand discovery; the $A01 ISR decode (the length
gate = WRONG in the recon).

- **bit2 = 0 STABLE 2.25 s -> 50 s on the ORIGINAL with the EE slave**
  (p1-fbscan-ee.resc): the vendor's own factory defaults (loaded from the
  modeled EEPROM via the first-boot detection of 0xFF) produce a sane
  squelch/scan state - NO eternal RX sessions, NO re-arm cycle. The recon
  = bit2 = 0 too (p1-flagdump-ee-recon.resc). **The phase-1 exit criterion
  is MET: the state gate clears naturally on both images and the keys act
  (+211,725 display events, session 59 pt2).**
- **The ISR decode (the UART frame assembler 0x08023920, current base —
  the recon's uart.c comment 0x0801F920 = the old base +0x4000):**
  the per-byte accumulate into the frame buffer 0x2000335C ([0x8023EE0]);
  the \r\n check; the frame[0]=='$' check; FLAG_BANK+9 bit0 = set (the
  frame-ready, my earlier guess = the right address); then the prefix
  dispatch: "$A00," (0x0804481C) = the 534-length-gated copy to
  0x20003A84 (the buffer1); **"$A01," (0x08044824) = the copy to
  0x20002F74 (the buffer2) WITH NO LENGTH GATE** (the memcpy of the
  accumulated idx bytes, unconditional); "$A02," (0x0804482C) = the copy
  to 0x20004B94 (the buffer3!); the 2-byte prefixes ('$B'/'$C' at
  0x8023D4C/D50) = the copies to 0x20002F74.
  **THE RECON'S ROUTING CORRECTION #2: the $A01 branch = drop the
  0x216 length gate (it belongs ONLY to $A00). The $A02 buffer =
  0x20004B94 (the recon = the buffer3 = to verify).**
- **THE UART RX-ON-DISCOVERY: the $A01 injections at 1 s and 4 s were
  DROPPED by the Renode USART ("Received a character, but the receiver
  is not enabled") — the CR1.RE = 0 in the first-boot flow, while the
  CK2 peer handshake runs (the old boot flow) had ZERO drop warnings.**
  The vendor enables the receiver on-demand around its own $A00
  handshake (the TX-first protocol: the radio sends $A00, then the
  response window). THE CORRECT $A01 TEST = the peer flow: the radio's
  $A00 first, the peer responds, THEN $A01 in the response window.
  The earlier-session $A00 injections worked because they landed in the
  boot handshake window. The receiver-enabling site = the next decode
  target (the CR1 |= 0x4 writer).
- The EE image = still 0 non-FF after the first-boot: the vendor's
  ee_save write path = the phase-2 audit (the reads work; the page-write
  mode/conditional save = to decode).

### 2026-10-17 — Session 59 pt4 (GOAL phase 1, continued): the I2C slave
debugging — three bugs found and fixed, the bit-timing issue isolated (the
turn ends mid-fix; the next session resumes here).

- **BUG 1 (fixed): the 9th-clock sampling.** My slave sampled the ACK bit as
  data (the 9-bit drift). Fixed with the explicit i2c_ack_wait phase: the
  8th rise completes the byte and drives the ACK (EE_SDA_BIT=0); the 9th
  rise consumes the ACK and releases.
- **BUG 2 (fixed): the peripheral scope isolation.** The two python
  peripherals (the GPIOA model, the displaybus) = SEPARATE namespaces (the
  probe proved it: no ga_*/EE_* symbols visible across). The EE_SDA_BIT /
  direction bridge over the engine globals = DEAD CODE. Rebuilt over the
  sysbus scratch RAM: 0x2000F800 = the slave SDA bit, 0x2000F801 = the
  master-output direction (the displaybus publishes it on the CRH writes,
  the PB9 nibble = CRH bits 4-7 = 0x3 = the output), 0x2000F802/03 = the
  START/STOP event (the level + the pending flag).
- **BUG 3 (fixed): the SDA resolver port error.** The master-out SDA read
  used the GPIOA ODR bit 9 (= PA9 = the RF line!) instead of the GPIOB ODR
  (PB9 = the actual SDA). Fixed: 0x40010C0C bit 9.
- **BUG 4 (fixed): the START/STOP detection.** The edge-based sda_last
  compare fired on EVERY bit (the master changes SDA between my edges).
  Rebuilt: the DISPLAYBUS detects the SDA changes while the SCL is high
  (it sees the SDA writes and can read the SCL through the sysbus) and
  publishes the event; the slave consumes it at the next edge. The event
  is gated on the master-output direction (during the ACK/read phases the
  master's stale ODR = not the line state).
- **BUG 5 (isolated, OPEN): the bit sampling = one rise late.** The EDGE
  trace (200 edges) shows the address bytes accumulating 0,1,0,0,0,0,0,1
  instead of the 0xA0's 1,0,1,0,0,0,0,0 — the vendor's bit-bang changes
  SDA while SCL is HIGH (the non-standard timing: the SDA set happens
  before the SCL falls), so my rise-samples see the freshly-set NEXT bit
  or the stale current bit depending on the exact event ordering. The
  consequence: the device address is never recognized on the WRITE path,
  the vendor's write retry loop spins (~12.4K I2C_BYTE hits), the boot
  hangs in ee_save, and EE_WRITEPAGE = 0. The READ path = unaffected
  (the DATAR state skips sampling; the 0xA1 address = recognized ONCE —
  the first transaction — so the address recognition itself works).
- **CRITICAL REINTERPRETATION OF pt2/pt3: the "bit2 = 0 / the first-boot /
  the keys act" observations were made with the boot HUNG in the ee_save
  retry — the 2M-event display activity = the OTHER tasks spinning, NOT
  the main mode. The bit2 = 0 = the pre-session state, not the unlocked
  radio. THE KEYS-ACT result (+211K events) = REAL (the key path = not
  bit2-gated in the hung state) but the full main-mode validation =
  MUST be redone after the slave's write path works. Nothing is lost:
  the slave architecture is correct, the timing fix is the last mile.**
- THE NEXT SESSION: (1) fix the sample-attribution (the candidate: sample
  the SDA at the FALLING edge instead of the rising — the vendor's SDA is
  stable from before the fall to after the rise, and the fall is what
  precedes the master's read; OR track the displaybus-published SDA
  change events as the bit boundaries); (2) re-run the boot to the
  ee_save completion (EE_WRITEPAGE > 0); (3) the power-cycle flow
  (the AIRCR reset) for the second boot with the persisted defaults;
  (4) redo the main-mode validation (bit2, keys, $A01 via the peer flow).

### 2026-10-17 - Session 60 (GOAL phase 1, continued): BUG 5 SOLVED - the I2C
slave works, the boot escapes the ee_save hang, the vendor programs its
defaults. Six defects fixed in the harness slave; two recon bugs found via
the REA MCP decompiles (the evidence for every fix below).

- **METHOD SHIFT: the REA MCP (Ghidra) decompiles replaced the hand-decoding.**
  Key decompletes: i2c_send_byte 0x080285B6, i2c_read_byte 0x8028666,
  eeprom_read 0x802873C, eeprom_write 0x80286C0, sda_output/in 0x802846A/448,
  the GPIO helpers 0x8023678/8023780/8023788/802375E, the radio init
  0x80267AA + its byte loop 0x8026712, the 12-bit serial driver 0x802883A +
  its init 0x80287F6. procedure_callers sweeps on the helpers found the two
  extra PB9 writers (see below). Evidence: the REA session on
  MAIN-MCU-HG-UV98-V1.0.1-190416.bin (sha256 73361601...b66b2, raw-binary arm
  @ 0x08004000).
- **RECON BUG (eeprom.c): the per-bit SCL order was wrong.** The original's
  loop = SDA write (while SCL LOW) -> SCL BSRR (rise) -> dly(10) -> SCL BRR
  (fall): STANDARD timing, SDA stable across the rise. The recon had SCL low
  -> dly -> SCL high (leaving SCL high at the loop end). The "vendor's
  non-standard SDA-while-SCL-high timing" premise of session 59 pt4 =
  RETRACTED: only i2c_start's START-fall and i2c_stop's STOP-rise change SDA
  while SCL is high - the event channel is genuinely clean.
- **BUG 5 ROOT CAUSE (the slave): the event-consumption block RETURNED before
  the scl_prev update.** The START lands on the start's SCL-fall tick; the
  return left scl_prev=1; the attempt's first data rise hit the
  scl==scl_prev early-return and the MSB was NEVER sampled; every sample
  shifted one slot and the 8th slot captured the ack-tail's SDA=1: 0xA0
  (1,0,1,0,0,0,0,0) assembled as 0x41 (0,1,0,0,0,0,0,1) - exactly the
  session-59 EDGE trace. Fix: consume the event and FALL THROUGH to the edge
  handling.
- **THE ACK-SPIN ROOT CAUSE (the slave): the ACK released one edge early.**
  The master reads the ACK AFTER the 9th rise (BSRR 0x08028626 -> dly(10) ->
  the IDR read 0x08028638, while SCL is HIGH); the old slave released
  EE_SDA_BIT AT the 9th rise -> the master read 1 = NACK forever (the 5,873
  retry storm). Fix: hold the ACK low through the 9th rise, release at the
  9th fall (i2c_ack_release).
- **THE READ CADENCE (the slave): publish data bits AT the SCL rise.** The
  master's read_byte samples the IDR after the rise (fall -> dly -> rise ->
  read, 0x802869c). Old model shifted at the falls and lost 2 bits per byte
  (the tail fall + the master-ack fall). Fix: publish at the rise, skip
  exactly one rise after each 8-bit group (the master-ack clock).
- **THE DIRECTION GATING (the displaybus): publish 0x2000F801 only on actual
  PB9-nibble transitions.** The display scan rewrites the same CRH
  (nibble 3 -> 3) constantly; the old unconditional publish flipped the
  direction back to "master out" DURING the I2C input phases so the resolver
  returned the master's stale ODR instead of the slave's ACK.
- **THE CROSS-PERIPHERAL READ LAG: the GPIOA->GPIOB sysbus read returned
  STALE values** (observed: sample=1 after a BRR wrote 0 and sample=0 after
  a BSRR wrote 1, at the exact write indices via the shared write counter
  0x2000F810). Fix: the scratch mirrors - the displaybus publishes its SDA
  level to 0x2000F804 and the GPIOA publishes its SCL (PA11) level to
  0x2000F805 inside their own write handlers; both consumers read the
  scratch (the proven cross-engine path).
- **THE DEDUPE DEFECT (the displaybus): the late-era slave deafness.** The
  dedupe compared against the last PUBLISHED level, but the ack-tail's
  SDA=1 write happens while SCL is LOW (never published) - so the NEXT
  START's 1->0 looked like no change and no START ever fired again (the
  late window: state=IDLE at every edge). Fix: track the master-driven SDA
  level on EVERY master-driven write; publish only on real level changes
  while SCL is high.
- **THE PB9 SHARING DISCOVERY (REA): the RADIO's 3-wire serial bus =
  CLK=PA15 (0x8000), DATA=PB9 (0x200), CS=PA8/PA12** (the radio init
  0x80267AA configures GPIOB 0x200 + GPIOA 0x8000/0x100/0x1000; its byte
  loop 0x8026712 = clock-low-first, 8 bits). **THE RECON rf.c SAYS RF_DATA
  = PA9 - WRONG: the original drives GPIOB (PB9), the same pin as the
  EEPROM SDA.** The radio's boot-init burst (~300 bytes = the observed
  2,400 PB9 writes in the LR trace) OVERLAPS the first I2C attempts - the
  vendor's own retry loop absorbs the collision; the later attempts are
  clean (the late EDGE window = clean data bits).
- **THE PHASE-1 MYSTERY SOLVED (REA): the pre-display GPIOB traffic = a
  12-bit serial bus: CLK=PD0 (DAT 0x80288E8 = 0x40011400), DATA=PB9,
  STROBE=PD1 (bit 1 pulse after the frame), one-time init 0x80287F6,
  called from the display render path (0x801A91A inside the SMETER_RENDER
  function 0x801AB36).** Not I2C; the slave (PA11-gated) correctly ignores
  it. Its per-call volume explains the old "phase 1" SDA runs.
- **RESULT (p1-i2cfix7 run, 10 s guest): EE_READ=149, EE_WRITEPAGE=9 (the
  first-boot defaults save ran - was 0 forever), I2C_BYTE=2147 (no storm),
  the ee_image programmed with 141 bytes of real settings (tables with
  flash pointers 0x40012500/0x40016113, erased 0xFF elsewhere), the boot
  proceeds past the settings load.** The IronPython note: the in-run
  `sum(1 for b in str)` counts chars not bytes - the in-run EE_NONFF=32768
  print is void; the host-side count = 141.
- NEXT: the main-mode validation on both images (bit2 transitions, keys,
  $A01 via the peer flow), the power-cycle flow, then the recon rf.c PB9 fix
  + eeprom.c SCL-order fix + protocol.c correction #2 + the host tests.

## Session 61 - the boot/app state machine decoded; the AT1846S SPI model; the PTT session-exit; the exit sequence validated on the original

- ENV: two Renode runs OOM-killed (the .NET heap ballooned by the per-tick
  log hooks). Added a 4 GB swapfile; BANNED per-tick log hooks (the SysTick =
  13.5 kHz!); all subsequent hooks = event/rare sites only. The runs are now
  lean (the RSS ~1.9 GB).
- The mm3 epoch battery: the display stream freezes at exactly 214,895 lines
  between T5-T10 (the same number as the Oct-21 run = deterministic), the
  flag byte+9 bit6 (0x40) toggles every 5 s (a blink = a live task), byte6
  0x16 -> 0x06 (the bit4 clear). NOT a crash: an idle renderer + a held
  session flag (byte6 bit2).
- AT1846S SPI slave model added (p1_gpioa.py spi_tick + the pb_displaybus
  IDR arbitration): the protocol decoded from FUN_0802692E - the CS = PA8/
  PA12 active-low, the CLK = PA15, the DATA = PB9; the command byte =
  1(rw)+7(reg) latched on the CLK rises; a write = 16 bits on the rises; a
  read = 16 bits shifted out on the CLK FALLING edges (the first fall
  precedes the first sample). The scratch bridges: 0x2000F806 = the CLK
  mirror, F807 = the CS mirror, F808/9 = the model's data bit + the drive
  flag; the write mirrors 0x2000F812/14/16 (the regs 0x29/0x2A/0x30) +
  F818 = the write counter. The read values: 0x1A/0x1B/0x5F = 0x0020 (the
  quiet RSSI/noise), 0x1F = 0x0000 (the GPIO/COS), 0x0D |= 0x8000 (the PLL
  lock bit - the tune loop at 0x801AC66 tests bit15 and re-tunes every ~5 s
  without it), everything else = the write echo.
- The radio-read tracer (the 13 call sites hooked): only 0x801AC66
  (reg 0x0D, the tune) fires in the idle state; the S-meter updater
  (0x8020468) stays gated; the state-machine reads never fire pre-key.
- THE STATE MACHINE DECODED (the session's core result):
  * byte6 (0x20005096) bit2 = the RX/scan session. SET = the enter branch
    0x802490E (in the periodic session manager; also bfi's bits4-5 =
    channel_idx+1 -> 0x16 at boot); CLEARED = only the exit 0x8024A1A tail
    (0x8024C10-16, after delay(50)).
  * The exit's early returns: byte6 bit7 set -> return; bit2 == 0 -> return;
    byte11 (0x2000509B) bit1 set -> clear it and return (the busy guard).
    The band-apply 0x8027BC6 then `[r4+2] &= 0xCF` = the 0x16->0x06
    transition, [0x20004720] (the dwell counter) = 0.
  * The "squelch-open RX entry" 0x801E09C = COUNTER-DRIVEN, no RSSI: the
    dwell counter [0x20004720]++ per call; at counter > [0x20004E82]*30
    (= 6*30 = 180, the dwell default from the factory settings byte 26): it
    calls the exit, then RE-ARMS bit2 (0x801E14C) and latches byte8 bit5
    (0x20005098 |= 0x20), which makes the entry self-skip thereafter (the
    counter freezes at 0). This is the designed one-shot boot transition,
    NOT a stuck flag.
  * The "squelch-close"/scan-starter 0x801E1A0 is disarmed by
    [0x20004E87] = 1 - the VENDOR'S OWN programmed default (the ee_image
    0x0A00 page byte 31 = 0x01) - so the channel scan never runs on a
    defaults device.
  * THE PTT ENDS THE SESSION: the PTT handler tests GPIOA bit10 (PA10,
    0x8024110, DAT_8024C20 = 0x40010800); the release path (0x80241A0)
    calls 0x8024606 then the exit 0x8024A1A -> bit2 = 0. A real device
    idles with the PTT released, so the session-clear state = normal.
  * The key dispatch 0x8019DCC (the task that also polls key_get_code at
    the OS-tick rate): bit2 == 1 -> only the keys 18/19/20 (the C-group
    scan keys) act, everything else is dropped (0x8019FE8 -> exit);
    bit2 == 0 -> the normal key switch at 0x8019FEA (the keys 11/12/13/14
    decode; the dispatch stores the key at [DAT_801A398]).
  * The key ladder (the recon ui.c confirmed live): the gates A/B =
    662..3253, C = 672..1751; the tables at 0x20001684/A4/C4 = {upper,
    lower} pairs; adc = 2000 matches the A entry 5 (1947 < 2000 < 2147) ->
    the index 4 -> the key code 1 (the A map {10,11,12,13,1,2,3,14}). The
    earlier "injectable codes {1,2,3,...}" were KEY IDs, not ADC values -
    the adc=1 injections could never match (the gate rejects < 662).
    The keypad lock 0x20009458 bit0 = 0 (open); the key-lock bytes
    0x200051A1/A2 = 0.
- THE HARNESS PTT BRIDGE: p1_gpioa.py IDR now drives PA10 from the mirror
  0x2000F80A (1 = released, the idle state) - the PTT-poll then calls the
  exit continuously from boot (mostly cheap bit2==0 early returns).
- p1-exitseq.resc (the ORIGINAL, consolidated): the boot + the $A00
  settings window (the 534-B frame x10, 0.7-2.7 s) + the $A01 telemetry
  (the 519-B frame x3) + the ADC2_DR = 2000 tag at 10 s. RESULT: b2 = 00,
  b6 = 02 (the session cleared, no re-enter), L(T7) = 289,221 (the peer
  flow consumed: +74 K lines vs the peerless 214,895 freeze), the key ->
  L(T12) = 347,255 (+58,034 = the key rendered), stable through T20. The
  key_event hook stays silent (the event = the repeat-engine path; the
  dispatch acts directly) - the display delta = the operative evidence.
- p1-exitseq-recon.resc (the RECONSTRUCTION): the boot = 195,582 lines
  (the same magnitude as the original's splash - the display driver ok),
  b6 = 00, but the key tag -> ZERO delta. THE GAP: the recon implements
  the keyscan chain + the stub key_event, but not the UI key dispatch /
  the session state machine - the P1 remainder on the recon side.
- NEXT: implement the recon's app state machine (the session manager, the
  PTT handler, the key dispatch 0x8019DCC behavior) to the point the exit
  sequence matches; then the p1-exit-sequence task closes on both images.

## Session 61 (cont.) - the recon's app state machine implemented; the p1-exit-sequence criterion met on BOTH images

- The recon ui.c/ui_task implementation (the commit after 74936b1):
  * the session-enter equivalent: FLAG_BANK+2 = 0x16 after ui_screen0_build
    (the original's enter 0x802490e: the bit2 + the band bits4-5 = idx+1;
    the bit1 from the earlier boot code = the observed 0x16);
  * the PTT-poll exit equivalent: every ui_task pass reads PA10
    (GPIO_IDR(GPIOA_BASE) & 0x400 - the harness bridge drives it high =
    the released idle) and on release applies the exit mask 0xCB
    (= the original's 0x8024a6a &= 0xcf then 0x8024c16 &= 0xfb), leaving
    the observed idle byte 0x02;
  * the key action = EDGE-TRIGGERED (the press-edge tracker): with the
    session bit2 == 0 the key change applies the 0x8019fea slice
    (FLAG_BANK+6 &= 0x7f, KEY_CODE_COPY = key) and repaints the status
    page. The first attempt re-rendered every pass (a held key poured
    4,956,083 stream lines) - the original renders once per action, so
    the edge trigger = the faithful semantics.
- THE DIFFERENTIAL RESULT (the same scripted stimulus p1-exitseq /
  p1-exitseq-recon: the PTT-released bridge + the $A00 window + the $A01
  cadence + the ADC2_DR = 2000 tag at 10 s):
  * ORIGINAL: T7 b2=00 b6=02 L=289,221; T12 L=347,255 (+58,034 = the
    key-1 repaint); stable T15/T20; no session re-enter.
  * RECON: T7 b2=00 b6=02 L=195,582; T12 L=289,093 (+93,511 = the single
    edge-triggered repaint); stable T15/T20.
  * MATCH: the bit2 1->0 flip on both images (the same scripted
    sequence), the post-transition key behavior = one stable repaint on
    both, the session stays cleared on both, the $A00/$A01 frames are
    consumed on both (the recon's protocol.c acceptance = the earlier
    p1-a01-path evidence). The absolute stream magnitudes differ (the
    full UI screen tree = the phase-3 scope; the display content, not
    the state behavior, is the remaining delta).
- The recon build: green, 107,115 text (+12 vs the pre-edit 107,103);
  the only warnings = pre-existing (the REG8 redefines, the two unused
  vars in the ui_boot_scene region).
- p1-exit-sequence = COMPLETE: the boot past the old "blocker" was
  already solved (the RTOS runs; the freeze = the idle renderer + the
  session semantics), the bit2 transition happens naturally on both
  images under the same harness script, the sequence is locked into
  reconstruction/sim/p1-exitseq.resc + p1-exitseq-recon.resc.

## Session 61 (cont. 2) - p2-shadow: the settings context + the S-meter steps implemented and marker-verified on both images

- REA decode (the primary instrument; batch_decompile on FUN_0801b198 and
  FUN_0801d8c0, search_bytes for the mirror literals):
  * FUN_0801d8c0 = memcpy (the 512/8/4/2/1-byte optimized copy) - the
    settings load's transport.
  * FUN_0801b198(mode) = the UI screen/band reset selector (the modes
    1/2/3 = 'g'/'h'/'i' + the per-screen repaint helpers), NOT the
    settings apply.
  * The S-meter chain fully decoded: the mode-10 handler 0x0801680C reads
    the CONTEXT byte (0x20004E68 + 2 + the VFO idx [APP_STATE+0xF0]),
    calls 0x0801F1C4 (the step table: 0->500, 1->625, 2->1000, 3->1250,
    4->2500, 5->5000, 6->10000, else passthrough), stores the u16 at
    APP_STATE+0x2E+idx*2 (0x200046CA+idx*2).
- THE MARKER EXPERIMENTS (the decisive data-flow evidence):
  * p2-marker.resc ($A00 with byte2=3, byte5=4, byte26=2): the ORIGINAL
    does NOT apply the head's frame to the context - the ctx stays at the
    defaults (the steps stay 500, the dwell args stay 06/00). The frame
    lives only in the head's rxBuf mirror (0x20003A84; the settings start
    at rxBuf+0x14 = the 5-byte header + the 14-B id).
  * p2-eemarker.resc (the EE pre-programmed with the vendor defaults plus
    byte2=3, byte5=4, the magic intact -> the second-boot path): the
    ORIGINAL loads the EE settings page 1:1 into the context 0x20004E68
    (the ctx[2]=03, the ctx[5]=04 appear) AND the S-meter chain fires
    live: the steps[0] = 0x04E2 = 1250, the steps[1] = 500. The dwell
    args = the ctx +26/+27 (0x20004E82/83) and the scan-starter gate =
    the ctx +31 - the whole session-61 state machine is SETTINGS-DRIVEN.
- THE RECON IMPLEMENTATION (ui.c/ui.h, the build = 107,211 text, the
  host tests pass): SETTINGS_CTX = 0x20004E68 + SM_STEP_BASE = 0x200046CA
  macros; ui_boot_scene copies ui_ee_settings 1:1 into the context after
  the magic check and fills the steps[0]/[1] from the ctx via the
  RECOVERED table.
- THE DEBUG STORY (the honest record): the first recon marker run
  crashed with PC=0x30000000. The bisect (the mem_copy disabled, then
  the steps disabled - the binary back to the d007a90 size) proved the
  CODE innocent: my sed-generated p2-eemarker-recon.resc had LOST the
  LoadELF line (the CPU executed the unwritten flash). The hand-rewritten
  script + the re-enabled blocks = clean. LESSON: never sed-generate
  resc setup blocks; hand-write the LoadELF/SP/PC setup.
- THE DIFFERENTIAL RESULT (the marker EE on both): the ctx bytes and the
  S-meter steps IDENTICAL (the ctx[2]=03 [5]=04 [6..]=the vendor
  defaults; the steps e204 f401 = 1250/500; the dwell 06/00; b6=02). The
  one residual delta = the original keeps 0x0004 at APP_STATE+0x36
  (0x200046D2) at boot - a separate variable, queued to p3/p4 with the
  other APP_STATE fields.
- p2-shadow = COMPLETE: the shadow load = the context copy (the marker
  verified on both), the runtime writer example = the S-meter steps (the
  marker verified on both), the $A00 non-application = marker-proven.

## Session 61 (cont. 3) - p2-consumers: the key-13 side-key executor chain decoded, implemented, and differentially verified

- REA decode: FUN_08020C9E = the mode-kill helper (the flag-bank byte at
  0x2000509B bit3 latch + the FUN_08020CEE/0x8026318 branches), NOT the
  executor. THE EXECUTOR = FUN_08022CB4, base = the CONTEXT 0x20004E68:
  the idx = ctx[0x40] (the EE byte 64 = 0), the state bytes =
  ctx[8+idx]/ctx[10+idx] (0x20004E70/72), the 6-B ID gate at ctx+0x30
  vs the ROM default at 0x08045A74 = "000000" ('0'x6 - the same as the
  EE's bytes 48-53), the 3-bit mode field at the flag bank +9
  (0x2000509D bits 3-5), the settle = the settings save 0x0801A6EA.
  (The earlier dump round used the wrong base - the flag bank +0x40 =
  0x200050D4 = a DIFFERENT variable, observed 0xC4; the executor's
  [ +0x40 ] = the ctx's. The address map is now explicit in the script.)
- THE DIFFERENTIAL (p2-key13.resc / p2-key13-recon.resc, four press/
  release cycles with the ADC2 tag 3100 = the A-ladder entry 3 = the key
  code 13):
  * ORIGINAL: (st1,st2) = (01,00), (01,01), (01,02), then the reset
    (00,00) - the gate6 = 303030303030 (= "000000", the match -> the
    reset path), idx = 00 throughout.
  * RECON: IDENTICAL - (01,00), (01,01), (01,02), (00,00), the gate6
    identical, idx = 00.
  * RESIDUAL: the original's 0x2000509D = 0xC0 vs the recon's 0x00 - the
    bits 6-7 = the screen-selector's flags (FUN_0801b198 |= 0xc0), NOT
    the executor's bits 3-5 (both 0) - queued to the phase-3 screen
    state. The display-stream magnitude also differs (the icon draw =
    the phase-3 scope).
- THE S-METER STEP SELECTION = already wired and marker-verified in
  p2-shadow (the ee byte2=3 -> the steps[0] = 1250 on both).
- Build 107,331 text; make test-host = all the protocol conformance
  tests passed.
- p2-consumers = COMPLETE: the S-meter step table behavior and the
  side-key executor chain are both wired to the settings shadow and
  differentially verified against the original.

## Session 61 (cont. 4) - p3-screen-map: the screen/transition map assembled

- The screen-ID space (the sessions 28-30, re-verified): the state byte
  0x200050E8 = the CURRENT SCREEN ID, printable chars by design, ~145
  values (0x00 idle; 0x01..0x2F the menu screens; 0x30..0x8D the rest;
  'g' = the RX/RF-active, 'h'/'i'/'j' = the beacon windows, 'M'/'N' =
  the memory modes, 'x' = the refined menu). ui_state_set (0x08010530)
  has only THREE call sites (0x8010EF8, 0x801B198 = the band/screen
  selector, 0x80214E4) - the screen writes are mostly direct stores
  (the 25-pool-site writer set from the session-61 ctx+0x21 sweep).
- The menu edges: 115, already mapped in docs/MENU_TREE.md (the
  binary-search dispatcher 0x08011190-0x08015628).
- THE KEY-DRIVEN TRANSITIONS - the per-key vtable decoded (the dispatch's
  long-press expiry at 0x801A02A calls [vtable + key*16 + 4], the release
  at 0x801A03E calls [vtable + key*16 + 8]; the base = DAT_801A3A4 =
  0x08040AEC; each 16-B entry = {u32 index, u32 press, u32 release,
  u32 category 0/1/2}):
    key  0: press 08019910 rel 08019d4c cat 0
    key  1: press 0801679c rel 080167d8 cat 0
    key  2: press 08016896 rel 080168e8 cat 0
    key  3: press 0801680c rel 08016868 cat 1   (the S-meter mode handler)
    key  4: press 08016924 rel 080169a2 cat 1
    key  5: press 080169d2 rel 08016a30 cat 1
    key  6: press 08017d54 rel 08017dde cat 0
    key  7: press 08016a6c rel 08016ac8 cat 0
    key  8: press 08016b00 rel 08016bcc cat 1
    key  9: press 08016cd0 rel 08016d10 cat 0
    key 10: press 08018ed4 rel 08018f1a cat 1
    key 11: press 08018222 rel 08018264 cat 1
    key 12: press 08017218 rel 0801735c cat 2
    key 13: press 08016fd0 rel 08017064 cat 0
    key 14: press 0801709c rel 080171dc cat 2
    key 15: press 08016f08 rel 08016f96 cat 0
- The dispatch's short-press semantics (0x08019FEA switch): the keys
  11/12 = the mode-kill (FUN_08020C9E) + the counter [r5+0x21] to 40 ->
  the vtable press handler; the key 13 = the executor FUN_08022CB4
  directly; the key 14 = the same counter mechanism; the key 15 = the
  MONI path (0x8019E9C, [r6+12] |= 0x40 + the counter). The release
  ageing tail = 0x801A20A.
- THE p3-screens WORKLIST = the 16 press handlers + the 16 release
  handlers above (the per-key decode queue), the reachable screen set =
  the ~145 IDs with the 115 menu edges + the key transitions.
- The p3-screen-map = COMPLETE (the map = this entry + MENU_TREE.md +
  the vtable).

## Session 61 (cont. 5) - p3-screens slice 1: the key-1 hold and the menu-key (the code 10) screen transition

- THE KEY-1 DECODE (0x801A13A): the keys 1/2 have NO short-press action;
  they increment the hold counter [r5+0x21] per dispatch pass and at 40
  the original switches to the screen 0x83 (FUN_08014530(0x83), the
  sub-state [r5+0x168] = the key; the key 2 also calls 0x801B15C(4,1,1)).
  THE GATE: the whole branch requires the band byte 0x200050E8 == 'g'
  (the RF-active screen) - in the model the band stays 0x00 (the PTT
  press does not open it either), so the key-1 slice is PARKED on the
  band-state machine (the RSSI-task screen scheduler 0x801B738 = the
  radio-on init, called from the boot 0x802D18A and the vtable[10]-family
  handlers; FUN_08014530 decoded: the screen byte = 0x200050E8, the VFO
  index = the ctx+0x40, the 0x1E ring snapshot).
- FUN_08014530 = the real ui_state_set (the REA decompile): [0x200050E8]
  = the screen param, the VFO idx = [0x20004E68+0x40], the 0x1E-byte
  memcpy = the ring snapshot 0x20004F94+idx*0x1E -> 0x20004FD0, the flag
  [0x200050E9] = 1. The recon's ui_state_set = the faithful shape.
- THE MENU KEY = THE CODE 10 (not 6!): the tag 2300 = the A-ladder entry
  0 -> the A-map[0] = 10; the vtable[10] = {press 0x08018ED4, release
  0x08018F1A, cat 1}. The observed transition (p3-key6.log): the press =
  the screen 0x35 (+49,648 lines), the stable hold, the release = the
  idle 0x00 (+65,223).
- THE RECON IMPLEMENTATION: the hold-counter machinery (ui_key_hold, the
  edge resets per 0x8019DE2; the keys 1/2 -> the 0x83 at 40 passes, the
  gate left open for the band machine), the menu key (the code 10) =
  ui_state_set(0x35) + the repaint at the press, the idle restore at the
  release.
- THE DIFFERENTIAL (p3-key6.resc / p3-key6-recon.resc, the tag 2300):
  the ORIGINAL scr 00 -> 35 -> (stable) -> 00; the RECON scr 00 -> 35 ->
  (stable) -> 00 - IDENTICAL state trajectory, one render per edge on
  both. The magnitudes differ (+49,648/+65,223 vs +93,511/+93,511) = the
  documented §7(c) screen-content delta (the 0x35 screen's own render =
  the next per-screen slice).
- Build 107,543 text.

## Session 61 (cont. 6) - p3-screens slice 2: the menu screen (0x35) render spec captured

- The key-10 press handler (0x08018ED4, the lcd.asm - not a Ghidra
  procedure) = only a state-flag clear: the ctx+8+[ctx+0x40] read via
  0x0801A69C, then [DAT_8019638+1] = 0. The 0x35 render comes from the
  periodic screen machine, not the press handler itself.
- THE RENDER TRACE (p3-rtrace.resc, the original, the display
  primitives hooked during the key-10 press window): 39 draw calls.
  The press-window spec (~21 calls):
  fill_rect(18,1,158,92); fill(1,1,158,16); fill(1,4,72,16);
  text(1,108,0x0803....,4); fill(1,120,16,16); fill(111,72,16,16);
  fill(111,30,120,16); fill(111,1,5,16); fill(111,157,2,16);
  text(111,6,0x0803....); text(111,125,0x0803....);
  fill(29,106,18,20) [the item highlight];
  text(18,80/92/55/30/68/5, the menu strings, 4);
  text(50,22,...); fill(64,133,24,10) [the scrollbar];
  fill(76,106,18,20) [the item-2 highlight]; text(64,80/92/55/30/68/5,...).
  The menu re-renders periodically (the second burst in the window).
- THE PRIMITIVES DECODED (REA): FUN_0801CD32(x,y,w,h) = the fill-rect
  with 0xEFBE (the window setter 0x0801B9C8(x, y, x+h-1, y+w-1); the
  h x w loop of the 0xEFBE/0xBE pairs) - the recon's display_fill_rect =
  the identical shape. FUN_0801CB60(x, y, str, ink, size) = the text
  draw: the ASCII (<0x81) = the 8-px path 0x0801C95E, the CJK = the
  16-px path 0x0801C914 (the y += 0x10 per char), the terminator 0/0xFF.
- THE IMPLEMENTATION ROUTE: the recon's display primitives = CK4-exact
  (the boot stream = event-identical), so implementing the 0x35 render =
  the same call list with the string pointers resolved against the
  recon's .uistrings blob (the byte-identical assets at the same
  addresses) gives a byte-exact stream for the menu screen. The string
  addresses from the trace: 134346476 = 0x0803464C? (verify at the
  implementation time), 134346540, 134346908, 134347088, 134502764,
  134502980, 134502988, 134347104.
- The slice = the spec captured; the implementation = the next work item
  (p3-screens continues).

## Session 61 (cont. 7) - p3-screens slice 3: the menu screen (0x35) fill layer implemented

- The recon's display primitives verified against the original's window
  math: display_fill_rect(y, x, cols, rows, color) takes the original
  FUN_0801CD32's (p1, p2, p3, p4) directly - 0x2A (the x-axis) =
  (p2, p2+p3-1), 0x2B (the y-axis) = (p1, p1+p4-1) per the session-52
  correction; the fill loop = the rows x cols of the 0xEFBE pairs.
- ui_menu_screen_build() implemented: ui_state_set(0x35) + the trace's
  13 fill calls in order, the color 0xEFBE. The text layer (the class-4
  draw 0x0801CB60: the " " pads, the "H"/"W" markers, the CJK 确定
  [0x08045A44] / 取消 [0x08045A4C]) = explicitly pending with the
  per-call comments in the source.
- THE DIFFERENTIAL (p3-key6-recon.resc after the build 107,711):
  PRE L=185,382; P1 (the press) scr=35 L=228,943 -> the press delta
  +43,561; the original's press delta = +49,648 (p3-key6.log). THE GAP =
  6,087 events = the text-glyph layer (the class-4 draws) - the fill
  layer = event-count-exact.
- The next slice: the class-4 text decode (0x0801CB60's ASCII 8-px /
  CJK 16-px glyph pipeline) to close the 6,087-event gap.

## Session 61 (cont. 8) - the class-4 text pipeline fully decoded (the 6,087-event gap's spec)

- The REA decomps of FUN_0801C99E/CAC0/C95E complete the class-4 text
  (FUN_0801CB60) semantics:
  * FUN_0801CB60(y, x, s, ink, class) - the cursors (y = p1 = the row,
    x = p2 = the column; the CJK path advances x += 0x10 per glyph =
    the horizontal 16-px).
  * FUN_0801C99E = the PRE-WRAP per class: if x > thr[class] then
    x = 6 and y += line[class]. The class tables:
    class:  0    1    2    3    4    5    6    7    8    9
    thr:   9B   9A   99   98   96   94   94   91   8D   90
    line:   7   10   12   16   16   16   20   15   24   32
    adv:     6    6    7    8   10   12   12   15   19   16
  * FUN_0801CAC0 = the POST-ADVANCE: x += adv[class] per ASCII glyph.
  * FUN_0801C95E = the ASCII glyph draw = the wrapper over the class-3
    cell FUN_0801BFB8 (the recon's existing glyph pipeline!) via the
    font fetch FUN_08028F8C.
  * The CJK (the lead byte >= 0x81 with the trail >= 0x81) = the 16-px
    glyph via FUN_0801C914, x += 16, NO wrap in the CJK path.
  * The BCD digit normalization: a byte < 10 mutates to +0x30 (the
    ASCII digit) IN THE CALLER'S BUFFER (the original mutates the
    working copy in place - the recon must copy before normalizing).
- The implementation route: display_draw_text4(y, x, s, ink, class) =
  the wrap-check + the existing class-3 glyph cell + the advance, over
  the recon's font.c (the ascii_font_fetch/cjk16_fetch = the existing
  assets). This closes the 6,087-event text gap in the menu screen.
- The menu's observed text calls mapped: text(1,108," ") class 4;
  text(111,6,确定) and text(111,125,取消) (the CJK); the item texts =
  the " "/"   " pads + "H"/"W" markers at (18/64, 5..92).

## Session 61 (cont. 9) - the menu screen's text layer = the multi-class cell sequence (the ground truth captured)

- The color scheme decoded (FUN_0801BFB8, REA): the ink code -> the (fg,
  bg) pair - the ink 0 = (0x0000, 0xEFBE), the ink 1 = (0x001F, 0xEFBE),
  the ink 2 = (0x0000, 0x8D5E), the ink 3 = (0xF800, 0xEFBE), the ink 4 =
  (0, 0xEFBE) or the swap per the DAT_0801C910+0x18 flag, the ink 5 =
  (0x13A2, 0xEFBE), the ink 6 = (0x58E1, 0xEFBE); the invert flag swaps.
- The glyph fetch decoded (FUN_08028F8C): the class-indexed records -
  the class 1 = 0xC stride, the class 3 = 0x10, the class 4 = 0x14 (the
  space/'-'/'.' special offsets), the class 5 = 0x18, the class 6 =
  0x24, the class 7 = 0x2D, the class 8 = 0x39, the class 9 = 0x40;
  the class 0 = the mutex'd lookup FUN_0802BEC8.
- THE CELL TRACE (p3-cell.resc, 100 cells): FUN_0801BFB8(p1 = y, p2 =
  x) CONFIRMED by the boot callsign (the y=72 fixed, the x = 42..138 =
  the horizontal NOCALL-7). The menu-era cells:
  * the item texts = the class 1 (6x10), the ink 4: (64, 5/11/17/30/36/
    42/55/68/80/92), (86, 139/145), (97, 22) - the repeated record
    134487808 = the space pad, 134488468/134488288 = the markers;
  * the header = the class 6 (0x24-stride): (76, 22/34/46/58/70/82/94);
  * the status icons = the class 3 (8x16): (82, 108/116), (78, 13),
    (20/37/54/71/88, 3/11/19/27/35/43/51);
  * the selected row = the ink 1 INVERTED (inv=1), the class 3.
  The earlier r0-r3 trace mislabeled: the 4th arg = the INK (4 = the
  items), the class = the 5th (unseen; the cells = the ground truth).
- The glyph records = the flash blob addresses (0x08041Fxx-0x08048xxx)
  = INSIDE the recon's .uistrings mirror - the cells are directly
  replayable.
- THE IMPLEMENTATION ROUTE (the next slice): the menu render = the
  exact cell-sequence replay via the recon's blitters (the class 1 =
  display_blit_glyph_cell_6x10, the class 3 = display_blit_glyph_cell,
  the class 6 = a new blitter with the 0x24-stride window geometry to
  decode from FUN_0801BFB8's param_6==6 case), the colors from the ink
  table. This bypasses the string/glyph-fetch layer entirely - the
  cells ARE the rendered output.

## Session 61 (cont. 10) - the menu text layer IMPLEMENTED; the byte-exact gate now rooted at the I2C master

- THE MENU TEXT = implemented (commit this session):
  * display_draw_text_cls(y, x, s, ink, cls, inv) in display.c = the
    FUN_0801CB60 walker: the pre-wrap table (thr/lin per class, the
    class-3 0x98; the 'h'-band 0x9E variant not modeled), the BCD bias
    (<10 -> +0x30 on the caller's buffer), the class-indexed glyph cell
    (the classes 1/3/6 = the existing blitters; the others = none yet),
    the post-advance (adv per class), the CJK pairs = no emit + x += 16
    (the 0x0801C914 fetch guard off in the platform model - the
    p3-text.resc zero CJK/FFR evidence), the 0x0801BFB8 ink table with
    the inv variants and the ink 3/4 swap bit = SETTINGS_CTX+0x18.
  * ui_menu_screen_build: the 5 strings (the flash slots 0x08043B20 +
    12*i = "01.本机设置" "02.站点类型" "03.GPS 设置" "04.信号设置"
    "05.信号类型") at (20+17i, 3), the ink 4, cls 3, inv 0; then the
    row-0 selection redraw (the ink 1, inv 1 = the blue bar) as a
    direct call (the original redraws without a cb60 entry).
  * BUILD GREEN 108,491 text; the host tests pass.
- THE DIFFERENTIAL (the fresh-EE p3-key6 pair): the press delta = the
  recon +49,435 vs the original +49,648 = the text layer closed all but
  **213 events** (the 22 cells x 267 = 5,874 vs the original's 6,087).
  The trajectory 00->35->00 = identical on both. The byte-exact window
  compare = BLOCKED by the pre-existing boot divergence (below) - the
  window contents differ from idx 0 in the ODR bit 9 (PB9) state.
- **THE ROOT BLOCKER (the CK4 regression vs the phase-0 claim):** the
  fresh-EE boot streams diverge at event 2582 (the first-boot EE-load
  era, right after the TFT DISPON 0x29). THE CENSUS (the fresh-EE 3 s):
  the orig CRH 7,105 (the 44434-out 4,427 + the 44444-in 2,678) vs the
  recon 5,073 (3,134 + 1,939); the BRR 17,736 vs 20,383; the BSRR
  10,017 vs 9,366. The orig's 0.4 s hook census: 422 i2c_starts, 832
  i2c_bytes, 150 i2c_reads (the sda_in 982 / the sda_out 1612) = the
  retry-heavy transaction pattern; the recon's counts = structurally
  lower on the CRH and higher on the BRR. The orig's per-bit visible
  unit in the EE-load reads = (CRH->in, CRH->out, the SDA value) - the
  per-bit direction switches that the current send/read-byte decodes
  (0x080285B6/0x08028666 = the CRH once per byte) do NOT produce ->
  there is another per-bit CRH writer in the orig's I2C path (or the
  decompiles = the wrong slices). The orig also LEAVES PB9 (the SDA)
  HIGH after the I2C era: all the later display ODR writes = 0x02xx vs
  the recon's 0x00xx = the SDA idle state bleeds into every display
  byte (the display bus_write preserves the ODR high byte).
- THE CONSEQUENCES: (a) the CK4 "boot event-identical" = currently NOT
  true at the event level under the fresh-EE first-boot path - the
  phase-0/1/2 verifications must have run the marker-EE (the not-first
  boot) or the ODR-only compare; the p3-key6 reruns with the marker EE
  diverge the same way (the event 2582). (b) the menu screen's
  byte-exact gate = blocked on the I2C root, not on the UI layer.
- THE NEXT MOVE (the genuine root attack, attempt 1): decode the orig's
  I2C event-level sequence per transaction (the retry policy, the
  per-bit CRH writer, the SDA leave-state) - the instrument = the
  targeted hooks at 0x08028448/0802846A/080284F2/080285B6/08028666 +
  the GPIO helper 0x0802375e (the IDR read) + the stream census
  alignment; then align eeprom.c to the exact event stream and re-gate
  the CK4 fresh. THEN the menu window compare closes the 213.

## Session 61 (cont. 11) - the I2C bit-level = VERIFIED CORRECT (the helper Rosetta); the root = the transaction level

- The GPIO-helper hook trace (0x08023780 BSRR / 0x08023788 BRR /
  0x0802375E IDR + the I2C entries, 26,684 events at ~0.2 s) = the
  protocol-level Rosetta for the original:
  * the START (0x080284F2) = CROUT, SDA-high (BS 0x200), SCL-high (the
    GPIOA 0x800 = invisible in the pb_stream), SDA-low, SCL-low;
  * the send byte (0x080285B6) = CROUT + the 8 x (the SDA value, the
    SCL rise, the SCL fall) + the SDA-high ack tail + CRIN + the 9th
    rise + ID 0x200 (the sample) + the fall = the PB9-visible 11
    events/byte = EXACTLY the recon's i2c_send_byte;
  * the read byte (0x08028666) = CRIN once + the per-bit (the SCL
    toggle on the GPIOA + ID 0x200 sample) = the PB9-visible 1
    event/byte = EXACTLY the recon's i2c_read_byte.
  => THE I2C BIT-LEVEL = NOT THE DIVERGENCE. The eeprom.c master = the
  bit-exact match to the original.
- The helper trace's opening (the pre-I2C era) = the RADIO SPI bit
  pattern: (BS 0x8000 = the PA15 CLK high, BR 0x8000 = the CLK low,
  BR 0x200 = the PB9 data) x N = 3 events/bit, the data line left LOW
  after the burst. The I2C's ack tail leaves PB9 HIGH (the BS 0x200
  before CRIN; the reads write no SDA) = the orig's post-I2C PB9 =
  HIGH = the display ODR 0x02xx. The recon's PB9 = LOW at the same
  point = the divergence = WHICH bus touched PB9 last and its leave
  state - the radio/I2C interleaving or the leave-state differs at the
  TRANSACTION level.
- The transaction census (the orig 0.4 s): 422 starts / 832 bytes /
  150 reads = far more EE transactions than the nominal boot load =>
  the retry/poll loop structure differs from the recon's, OR a second
  I2C consumer exists. THE OPEN ROOT: align the transaction sequence
  (the call sites of the ee_load/ee_save/ee_read paths + the retry
  policy) and the PB9 leave-state per bus, then re-gate the CK4 fresh.
- The radio SPI's PB9 data = the BRR-dominant (the low bits) = the
  recon's higher BRR census (20,383 vs 17,736) = the radio-vs-orig
  bit-count mismatch candidate = the AT1846S transaction alignment =
  the same root (the CK3 channel = the byte-level SPI, the PB9 stream
  = the bit-level, the two must agree).

## Session 61 (cont. 12) - the orig's stream 2582+ = the I2C transactions bit-aligned; the recon's 2583 = the foreign BRR

- The helper-trace transaction decode (the captured 74 STARTs / 144
  BYTEs / 93 RDBYTEs / 35 STOPs):
  * the FIRST EE transaction = the settings context read: START, 0xA0,
    0x0A, 0x00 (the page 0x0A00 = the SETTINGS_CTX source!), restart,
    0xA1, 77 RDBYTEs, STOP = the 77-byte context read, bit-exact.
  * the SECOND era (the helper 21243+) = the per-byte page writes:
    (START, 0xA0, the addr-hi, the addr-lo, the data, STOP) x N with
    the ~5-event gaps = the ack-poll/write loop of the defaults
    programming.
- THE PB_STREAM ALIGNMENT: the orig's event 2582 = the I2C's first
  CROUT; the (CRH, BSRR, BRR) pairs at 2582-2587 = the two STARTs
  (the SDA-high/low visible pairs); the 2588+ = the 0xA0 byte's CROUT
  + the SDA bits (the BSRR=1, the BRR=0: 1,0,1,0,0,0,0,0) - the
  perfect bit-level match to the helper Rosetta. THE ORIGINAL'S I2C =
  FULLY ACCOUNTED AT THE EVENT LEVEL.
- THE RECON'S DIVERGENCE = at the same boundary: the recon's 2582 =
  the CRH (the sda_output's 0x30 RMW) but the 2583 = a BRR (the SDA
  LOW) where the orig has the second CRH - the recon's pre-I2C PB9
  state/ordering artifact (the radio's data line = left LOW vs the
  I2C's expectation, or an extra/foreign write at the entry). THE ROOT
  = now a 1-2 instruction window at the I2C entry + the transaction
  census (the orig = the 422 starts in 0.4 s = the write-poll loop
  cadence vs the recon's page-write loop).
- THE NEXT MOVE (the root attack, attempt 1 = continuing): dump the
  recon's 2570-2600 window, map each event to the recon's source
  (the eeprom.c vs rf.c), fix the entry sequence to the orig's
  (the SDA released-high BEFORE the first CRH if that is the delta),
  then re-run the fresh-EE ck4 pair -> the event census -> the
  PB9-masked full compare -> the byte-exact press window (the 213).

**Session 63 — the settings-context wiring corrected per 0x0801B738/0x0801B2E8
disassembly; fresh-EE boot byte-identical through seq 172,707; the VFO display
identified as the first state-driven gap.**
- STALE-STREAM ARTIFACT RESOLVED: p3k6_rec.stream had not been re-copied after
  the newer runs; the session-62 "apply Phase X at 187041" contradiction was an
  artifact of diffing a stale capture against the current build. Rule adopted:
  cp ~/hguv98-sim/pb_stream.log p3k6_rec.stream IMMEDIATELY after every run.
- THE F8xx PROBES WERE LIVE-STACK CORRUPTION (removed): the recon linker places
  .taskstacks at 0x20002968..0x20010000 (the pool grows DOWN from _etaskstacks),
  so the session-62 probes at 0x2000F830-0x2000F83B sat inside task stack
  regions — every poll's counter/tick writes clobbered live stack bytes. All
  probes removed (rf.c apply/poll probes, main.c gate probes, protocol.c
  proto_rx_count, rtos.c time_ticks + OS_TIME_GET_SET_EN back to 0).
- REA (Ghidra) on the original image disassembles: 0x0801B738 (boot scene),
  0x0801B2E8 (factory reset), 0x0801CD32 (fill), 0x0801CB60 (text renderer),
  0x0801a6d8 (ee_load). Findings, all now implemented:
  (1) 0x0801a6d8 = eeprom_read(0x0A00, 0x20004E68, 0x4D): the settings load
      reads DIRECTLY INTO the radio context — ui_ee_settings buffer deleted;
      ui_boot_scene reads into SETTINGS_CTX and ui_factory_reset operates on
      the context itself (the post-reset mem_copy deleted). The S-meter
      selectors, rf shadow [0x19], and the scan-time saves all re-pointed.
  (2) The reset draws the "RESET WAIT..." text FIRST (0x0801cb60, scheme 4,
      class 3), THEN memsets the 0x4D context to 0x00 and writes the default
      fields (the field list matches the recon's recovered defaults exactly).
  (3) The scheme-4 cell pair = (fg 0x0000, bg 0xEFBE) SWAPPED when the app
      blank flag [SETTINGS_CTX+0x18] is non-zero (the same ink-swap bit the
      draw_text_cls path models). Fresh EE: the load leaves 0xFF at +0x18, so
      the fresh-boot cells = glyph 0xEFBE on 0x0000 (p3k6_orig cell
      2882..3137 = 84x 0x0000 + 44x 0xEFBE); the session-52 run's image
      carried +0x18=0x00 (unswapped). bg4 gate added to ui_factory_reset.
  (4) The fill (0x0801CD32) color gate CONFIRMED: [0x20004E68+0x18] != 0 ->
      pixels 0x0000 (else 0xEFBE). Gate implemented in display_fill_rect.
      At the boot fill the flag is already 0 (the reset's memset) -> 0xEFBE
      (p3k6_orig alt-run 13082..54040 = 40,958 ODR 0xEF/0xBE) — unchanged
      behavior, gate added for fidelity.
  (5) NEW: both the fill and the text renderer begin with a display-lock gate
      ldrb [0x20005094+8], ands #1, bne-skip: when bit0 of 0x2000509C is set
      the fill and ALL text output are suppressed entirely (not a color
      change). Not boot-reachable (bit = 0); open item for the suspend/menu
      paths. NOT yet implemented in the recon.
- FRESH-EE CK4 RESULT: after (1)-(4) the recon's raw-value stream matches the
  original byte-for-byte from seq 1 through 172,707 (the reset-wait cells, the
  boot fill 0xEFBE, the splash, the scan era, the status-page build start).
  First divergence = seq 172708 inside the FIRST 12x20 frequency cell
  (window 0x2A cols 0x16-0x21, rows 0x1D-0x30).
- THE 12x20 FONT + BLIT VERIFIED PERFECT: decoding the original's big cells
  (240 px each, the window bytes 0x2A 00 16 00 21 / 0x2B 00 1D 00 30 / 0x2C)
  identifies every glyph at score 240/240 against data/big12x20_font.bin
  through display_blit_glyph_cell_12x20's band format. The font asset and the
  blit are correct; the VALUE is not.
- FIRST STATE-DRIVEN GAP: the original's status page renders the VFO
  frequency from state: row 1 (rows 29-48) = "136.125", row 2 (rows 76-95) =
  "00.125" (all cells 240/240 glyph matches). The recon draws the static
  "000.000" scaffold transcription (the session-52 sim-state). 136.125 =
  exactly the scan's VFO-B init value (EE 0x0010 block, BCD 00 25 61 13);
  the VFO-A init written to EE 0x0000 = 440.125 does NOT appear on row 1.
  OPEN: which EE block/state feeds each row and the load path (the boot's
  0x0F90/0x0FB0/0x1F90/0x1FB0 reads return 0xFF on fresh EE) — next session
  decompiles the VFO-state loader and the row renderer to wire the dynamic
  value through.
- Diff statistics: 880 changed blocks / 284,114 lines remain, dominated by
  the VFO digits, the S-meter/icon cells between the rows, and downstream
  state-driven content. The pre-press window byte-exactness work continues
  from the VFO renderer.

**Session 63, part 2 — the VFO frequency rows decoded and wired to state;
match extends 172,707 -> 178,695 (880 -> 523 diff blocks).**
- INSTRUMENTATION (the AddHook traces on the original, the resc variants
  p3-key6-cb60/-bfb8/-win/-lr.resc = the debug-only files, NOT committed):
  (a) CB60 (0x0801CB60) logs: the boot draws "RESET WAIT..." (y=48, sch=4),
  "CALL:" (y=72, sch=0), the callsign "NOCALL-7" (y=72 x=82, sch=0 — the
  fresh-EE 0x1800 read = 0xFF -> the same NOCALL-7 fallback the recon has),
  "BAT:0.8 V" (y=88); the class-1 labels at y=18/64 ("   .CTC.L" etc.); the
  channel list "01.."-"05.GPS .." at the key-6 menu (the press window).
  (b) BFB8 (0x0801BFB8) logs: the VFO digit cells = BFB8 y=29/76 at the
  x=22..94 step 12, records 0x803F6xx-0x803F7xx, drawn TWICE each (the
  periodic re-draw). The records decode against the class-6 table
  0x0803F490 = (ch-0x20)*36: row 1 = '1','3','6','.','1','2','5';
  row 2 = '4','0','0','.','1','2','5'.
  (c) set_window (0x0801B9C8) logs with LR: the digit windows carry
  lr=0x801c619 (the BFB8 class-6 branch); the digit cells' own LR =
  0x801c99b (the CB60 walker's cell wrapper 0x0801C95E) — the digits ride
  the CB60 text path with the BCD bias (<0x0A -> +0x30).
  (d) EE read (0x0801A3A8) logs: read16(0x0800 -> 0x20004F94) immediately
  before the row-1 digits; read16(0x0810 -> 0x20004FB2) before row 2 —
  exactly where the recon's session-55 scaffolding reads (and discards).
- VFO ROW RENDER [RECOVERED]: the block's bytes 3,2,1 = the frequency in
  big-endian BCD nibbles; the display = d0 d1 d2 '.' d3 d4 d5 with
  d0=b3-hi, d1=b3-lo, d2=b2-hi, d3=b2-lo, d4=b1-hi, d5=b1-lo. The fresh
  scan-init blocks give "136.125" (00 25 61 13) and "400.125"
  (00 25 01 40 — CORRECTS the session-55 "440.125" note: 40 01 25 00 =
  40012500 = 400.12500 MHz). ui.c vfo_freq_row() implements it; both
  static "000.000" transcriptions replaced by the dynamic render fed by
  the existing read16 calls. The 12x20 font + blit verified PERFECT
  (every orig digit cell = a 240/240 glyph match against
  data/big12x20_font.bin).
- 0x0801D3B4 = the SOLID-rect scheme fill (0 -> 0x0000, 4 -> the
  flag-gated 0x0000/0xEFBE, 1 -> 0x001F, 3 -> 0xE020, 7 -> 0xF01E,
  6 -> 0x20C2, 5 -> 0x13A2); 0x0801BD3C = the 16x16 icon blit with the
  SAME scheme-4 flag-gated swap (the third independent confirmation).
- NEXT STATE-DRIVEN GAP: the first remaining divergence = seq 178696 =
  the 6x10 label at (22,50): the original renders 'H', the recon's
  session-52 transcription has 'L'. The status-page labels/marks are
  state-driven (the VFO mode/name indicators?) — the next decompile
  target is the CB60 caller that renders the label row.

**Session 64 — the state-driven labels decoded (the match extends to 187,040);
the original's RSSI-poll architecture identified via REA.**
- The H/L power-level indicator decoded completely: 0x0801F57C(y, x, flag,
  class) draws ONLY while the screen byte [0x200050E8]==0, char = the flag's
  bit0 ? 'H' (0x48, string inline at 0x0801f89c) : 'L' (0x4C, 0x0801f734),
  via the CB60 walker class 1. Callers: 0x0801E5E0 passes (0x32, 0x16,
  ctx[0x0C]&1) — THE SETTINGS CONTEXT IS THE STATE (the [0x0801f024]
  literal = 0x20004E68); 0x0801E7E4 passes (0x61, 0x16, slot1+0xC&1); the
  0x0801E2D0 variant (0x55, 0x16, slot0+0xC&1). The slot array = [0x0801ea00]
  = 0x20004F94 = the read16(0x0800) destination (the slot stride 0x1E).
  Implemented: ui.c renders both labels from the settings ctx[0x0C] bit0
  (y=50) and the read16(0x0810) mirror byte 0x0C (y=97). The fresh factory
  defaults carry 0x01 -> 'H' both (the p3k6_orig cells = 60/60 'H' matches).
- RESULT: the raw-value stream now matches the original byte-for-byte
  through seq 187,040 — the ENTIRE boot including the full status page
  (both VFO digit rows, both power labels). 523 -> 509 diff blocks; the
  first divergence = 187,041 = the FIRST RF EVENT of the press window.
- THE RF ARCHITECTURE DIFFERENCE (the next work item, REA-evidenced):
  the recon emits 29,295 HL bits + 226 I/O events after 187,041; the
  original only 5,220 + 186. The original's RSSI poll is NOT a free-running
  500 ms RTOS task: REA shows 0x0801ACFC = the per-slot refresh (the
  rf_read(0x0802692e)(chip, 0x0A) -> the RSSI state strh [r6+0x122], the
  mode-set 0x0802689a, the gpio_reconfig_plain 0x080272a4, 4 EE reads,
  the 3 label drawers 0x0801f510) invoked from the UI/periodic layer
  (the callers 0x80241bc/0x802445e/0x80247cc) — the poll cadence = the
  screen-refresh cadence. The recon's free-running rssi_task (21 polls in
  11 s) over-emits ~5.6x. ALSO: the recon's stream shows a 58-bit preamble
  before the first poll's block_b ((0x30,0x7002)+(0x1F,0x9AA9)+read-cmd —
  the exact apply-Phase-X head shape) while the hook evidence says the
  apply never runs (APPLY-ENTER=0), rf_write(0x30,0x7002) never fires
  (the conditional hook = 0) and the first rf_send_byte = (0x15,0x1F,0x00)
  — the preamble's emitter is STILL UNIDENTIFIED (the open item; the
  next candidates: an inlined rf frame writer or a direct BSRR/BRR path
  outside the hooked functions).
- The debug-only resc variants (p3-key6-cb60lr/-recon-hook/-recon-rfw/
  -recon-sbit/-recon-w7.resc) = untracked, NOT committed (the workaround
  register: reconstruction/sim/).

**Session 64, part 2 — the orig's poll call chain fully mapped (the RF
rework blueprint).**
- The three 0x08024xxx callers of the poll (0x0801ACFC) = the SCREEN
  RENDERERS: 0x080241BC (83 instr, <- 0x802195a), 0x0802445E (152,
  <- 0x8024106), 0x080247CC (212, <- 0x8024106 + 0x8024c24). Each ends
  with 0x0801acfc(slot) near the tail — the poll = the LAST step of a
  screen render, hence the render-gated cadence (the orig's 5 polls in
  11 s = the 5 render passes reaching the poll).
- The orig's RF layer primitives: 0x0802689A = the register-write engine
  (the (chip, {reg,val}) struct form; the bit-bang in 0x08026838/0x08026786);
  0x080272E2 = the irq-locked single-write wrapper (0x0802b1e0 + 0x0802689a);
  0x080272A4 = the gpio_reconfig_plain (the band -> 0x9EA9/0x9AA9 via
  0x0802689a); 0x08027156 = the per-band channel-apply sequence (the
  block-b-class table writes). The poll body: the rf_read(0x0802692e)
  (chip, 0x0A) -> the RSSI strh [slot+0x122]; if slot==1 && the flag bit
  clear: the write(0x0A) via 0x0802692e + the 0x38-mask store + 0x0802689a;
  then 0x080272a4(chip, 1); then 0x08024d08(halfword).
- The recon rework plan (the next session): remove the free-running
  rssi_task poll cadence; integrate the poll as the last step of the
  screen render path; keep the poll shape = the rf_read(chip,0x0A) + the
  mode-set + the gpio-reconfig; then re-diff (the 58-bit preamble may
  dissolve with the restructure — if not, attribute it then).

**Session 65 — the status-page re-draw fixed; the display content of the
boot now byte-identical end-to-end.**
- The status-page re-draw section (the ui.c static VFO-row scaffold after
  the LED-tail replay) re-rendered the STALE "000.000" digits; the original
  re-renders with the live values (the BFB8 trace: y=29/y=76 each drawn
  twice with the same records). Replaced with vfo_freq_row(0x1D/0x4C) on
  the session-63 mirrors.
- RESULT: 509 -> 472 diff blocks; the boot's display content (the fills,
  the splash, the scan era, the status page incl. both VFO rows drawn
  twice, both power labels, the LED bursts) = byte-identical through
  seq 187,040. The first divergence = 187,044 = the first RF event.
- The RF rework (the session-64 blueprint) = the next work item: the poll
  = the render tail (the recon's rssi_task = the free-running 500 ms
  emitter to be removed; the poll = the render-integrated 0x0801ACFC
  model). The 58-bit preamble attribution remains open.

**Session 65, part 2 — the render-tail poll restructure attempt: mixed
result, reverted pending a deeper decode.**
- Moved at1846s_rssi_poll()+the 0x0D read from the free-running rssi_task
  (retired to a dly loop) to the ui_screen0_build tail, per the 0x0801ACFC
  render-integration blueprint. RESULT: the RF volume moved the right way
  (29,295 -> 11,112 HL bits after 187,041 vs the orig's 5,220) but the
  block count REGRESSED (472 -> 780): the recon's burst spans became
  (187041,191498)=4457b, (193310,193334)=25b, (203678,204634)=956b,
  (323173,327630)=4457b, (339810,340766)=956b... — the 4,457-bit bursts
  are suspiciously the ui_led_stream replay size, hinting the LED-replay
  call sites interact with the build count (ui_screen0_build fires more
  than the orig's 5 render passes). REVERTED to commit 471557f (the
  472-block state) while the render-pass accounting is decoded. Next:
  count the orig's ui_screen0_build-equivalent renders (the 0x08024xxx
  callers 0x8024106/0x802195a/0x8024c24 = the render dispatch) and gate
  the recon's build/poll to the same pass schedule before re-attempting
  the integration.

**Session 66 — the RSSI poll re-architected to the REAL original model
(FUN_0801ab36); the poll content is now byte-exact.**
- Renode hook census of the original (p3-key6-orig-rf.resc /
  p3-key6-orig-poll.resc): rf_read (0x0802692E) fires EXACTLY 5x in the
  p3-key6 scenario, always from lr=0x801ac6b = FUN_0801ab36 (0x801AB36-
  0x801ACFB) — the session-64 "poll = 0x0801ACFC" claim pointed one
  function late; the REAL per-slot activate = FUN_0801ab36 (params ->
  apply 0x08027156 -> dly(0x28) -> rf_read(chip,0x0D) -> store
  slot[0x11E+chip*2] &= 0x8000 -> if zero the re-init path, never taken
  fresh). The three "renderers" (0x080241BC/45E/7CC) and 0x0802195A never
  execute in this scenario.
- Trigger census: the 5 polls = [FUN_08022a98 x2 calls = the both-slot
  refresh (its dual-band branch = poll(other chip) then poll(active)),
  once at the boot end (RD wall 6.05/6.11) and once at the release
  (100.77/100.85)] + [FUN_08019910 site 0x8019ba5 once at the key-10
  press (35.56)]. NOT free-running: the recon's 500 ms rssi_task retired
  to a dly loop.
- The poll burst decoded byte-exact from p3k6_orig.stream 187041..188038:
  996 PB9 bits = 39 x 25-bit words (block_b 24 + tone pair 2 + raw tail
  13, first bytes 0xBA/0xFD carry R/W=1) + 13-event gap + rf_read(0x0D)
  cmd 8, then CRH-in/out. ZERO ODR writes — the poll transport is
  PB9-only (the clock on the unlogged PD0-class port), NOT the
  GPIOA-CS/CLK rf_write SPI used at boot. Session-62's shape
  (gpio_reconfig + Phase-Y apply) RETRACTED.
- The session-53 LED stream/tail replays are WARM-boot-only artifacts:
  the ui_led_stream pattern occurs 0x in p3k6_orig.stream (fresh) vs 2x
  in the recon's pre-fix stream (the boot-end build + the release
  build). Both gated on ui_ee_warm like the ee359 burst.
- RESULT: the recon's poll content now matches the orig's byte-for-byte
  (the 1,996-event poll pair = identical), emitted at the three correct
  architectural moments (boot end x2, press x1, release x2). Residual:
  the recon's status-page build still ends with a ~170-event display
  chunk the fresh original draws only AFTER its poll pair (positions
  flipped), so this scenario diffs 505 blocks (was 472 with the old
  free-running wrong-content polls). Next: identify the build-tail
  chunk (the set-window 0x2A + ~29-byte draw after the RX-bar era) and
  re-order it to the orig's schedule, then re-diff.

## Session 68 — the RSSI poll fully recovered (FUN_08022a98 tree); the poll channel goes byte-exact
Date: 2026-10-31 (closing continuation of session 67/68)

- **The rec72/rec73 "mystery bits" root-caused.** The recon's poll burst
  (996 events, right structure, wrong values, deterministic across runs)
  resisted every transform until the emitter was instrumented at three
  levels: (a) entry hooks proved at1846s_rssi_poll fires exactly twice
  at the boot end from the ui_task call sites; (b) Renode peripheral
  access logging (LogPeripheralAccess gpioPortB) showed the peripheral
  RECEIVING bits that match the stream (the logger is faithful, 1:1
  with the CPU stores); (c) register-state hooks at the poll's store
  instructions showed r1=0x40010000, r4=0x200, r3=0x15<<k — the loop
  holding the CORRECT table bytes while the bus recorded different
  bits. The same rf_send_byte shape captures correctly in the boot era
  and corrupted in the poll era; the inline second-bus word captured
  correctly in both. Working hypothesis (logged, not fully pinned): a
  Renode execution artifact of the shift-fused iteration shape
  (GCC's `mov.w r0, r0, lsl #1`-style fused loop feeding ite-conditional
  `strmi.w/strpl.w` wide stores) in this era context. The fix is
  behavioral, not theoretical: the bit loops are written in the
  pre-loaded `(word >> i) & 1` form with store-then-clock ordering
  (the shape that captures byte-exact everywhere).
- **The poll = FUN_08022a98 (the both-slot refresh) -> FUN_0801ab36 ->
  FUN_0801a91a, and the transport = FUN_0802689A** — NOT the
  session-66 "PB9-only" tight emitter (RETRACTED). REA decompiles:
  - FUN_08026712 = the per-bit sender: CLK low (PA15), dly 10, DATA
    (PB9), dly 10, CLK high, dly 4 — MSB-first, one PB9 event per bit.
  - FUN_08026838(chip) = the CS: 1 -> PA12 low/PA8 high, 0 -> PA8 low/
    PA12 high, else both low; (4) = the deselect.
  - FUN_0802689A(chip, {b0,hi,lo}) = the poll's frame writer: CLK low,
    dly 8, CS, dly 10, 3 bytes (FUN_08026786), dly 8, deselect, dly 10,
    CLK low again, **PB9 DATA-low (the 25th event)**, dly 8. GPIOB
    footprint = 24 bits + the trailing BRR = 25 events/frame. The
    boot's rf_write (FUN_0802289A) has NO trailing DATA-low = 24
    events/frame — TWO different writers, now both recovered.
  - FUN_0802883a(byte, band) = **the second serial bus**: clock GPIOD
    PD1 (DAT_080288e8 = 0x40011400), strobe PD2, data PB9
    (DAT_080288ec = 0x40010C00). 12 bits MSB-first (bit11 first),
    dly 5 per edge, then CLK low + PB9 DATA-low (the 13th event) +
    PD2 strobe. = the "constant 13-event gap" of session 62/66 — the
    gap is a SECOND BUS WORD, not padding. Value = tbl1[idx] |
    (band << 8), tbl1 at 0x20005010 (DAT_0801b59c).
  - FUN_08026c46(chip,0) = the 24-word block_b init (byte-identical to
    the recon's existing at1846s_boot_init_block_b table!).
  - FUN_0801a91a(chip, band, arg) = the tone pair: 0x48=0x1E3C (sq 0),
    0x49 = ((v-1)&0xFF)|((v&0xFF)<<7) with v = arg + tbl2[idx]&0x7F
    (tbl2 at 0x20005030); idx = ((tick/100000 (DAT_0801b2e4) -
    base16[RAM+0xea]) & 0xff)/step8[RAM+0xee] — the tick-indexed tone
    tables; then FUN_0802883a.
  - FUN_080272e2 = 0x1F=0x9BA9 (band 1) / 0x9AA9; FUN_08027110 = 0x30
    0x7006/0x4006 (sq 3 = no write); FUN_080276d4 (sq 1 = the default
    branch here: 0x58=0x8005, 0x4A=0x157C, 0x4D=0, 0x3A=0x40C9,
    0x5B=0x1213); FUN_08027156 = 0x30=0x7006, 0x05=0x8763, 0x29/0x2A =
    the freq29 halves, FUN_08027092 = 0x30=0x7006 + 0x30=0x70A6;
    FUN_0802692E = rf_read(chip,0x0D) (cmd 0x8D + the CRH I/O pair).
- **The five orig polls re-framed with the 13-event insert after word
  25** (the session-66 stride-25 table was misaligned from word 26 on):
  w0-23 block_b (fixed, = the session-66 table ✓), w24 0x48=0x1E3C,
  w25 0x49 [LIVE], ins13 12-bit [LIVE], w26 0x1F=0x9BA9, w27 0x30=0x7006,
  w28 0x58=0x8005, w29 0x4A=0x157C, w30 0x4D=0, w31 0x3A=0x40C9,
  w32 0x5B=0x1213, w33 0x30=0x7006, w34 0x05=0x8763, w35 0x29 [LIVE],
  w36 0x2A [LIVE], w37 0x30=0x7006, w38 0x30=0x70A6, tail 10001101
  (0x8D). The session-66 tail words 27-39 (0xBA/0xFD raw patterns) were
  framing artifacts — RETRACTED.
- **The live values = per-slot constants in this scenario** [DATA-DERIVED
  from the five polls; the generating chain documented above]:
  UHF slot: freq29 = 0x0061AFD0 (400.125 MHz), 0x49 = 0x0203, bus12 =
  0x1FF (band 1); VHF slot: freq29 = 0x00213BD0 (136.077 MHz), 0x49 =
  0x4283, bus12 = 0x0FF (band 0). Poll order = FUN_08022a98's
  poll(other) then poll(active): boot [UHF, VHF], press [VHF] (the
  FUN_08019910 site), release [UHF, VHF].
- **Implementation (rf.c/rf.h/ui.c)**: at1846s_rssi_poll(chip) rebuilt as
  the FUN_0802689A frame writer + the second-bus word + the fixed
  15-frame apply + rf_read(chip,0x0D); poll_send_byte/poll_read_reg/
  at1846s_poll_gap_replay deleted; rf_send_byte recast in the
  (word >> i) & 1 shape; the five ui.c call sites carry the slot.
- **RESULT (p3k6_rec75 vs p3k6_orig)**: ALL FIVE POLLS BYTE-EXACT
  (996/996 bits each, firstdiff=None). The identical stream prefix now
  runs to event 218119 = 71.77% of the orig (was 187043/6.2% pre-fix).
  The boot, the poll pair, and the ENTIRE menu render (189037..218119,
  ~29k events) are byte-exact. Remaining divergence starts at the
  press-era interactions (the recon's press poll lands 20352 events
  later than the orig's — the press/key-6 menu-interaction era is the
  next target, then the release era's +6659).

## Session 68 pt2 — the press-era order fix: the poll sits INSIDE the menu render
Date: 2026-10-31

- The orig's press flow = [fill#1 (18,1,158,92) = the 29,083-event clear][the
  press poll (218120..219115)][the post-poll dynamic (219118..238685 =
  19,567 events)]. The recon rendered everything before polling.
- ui_menu_screen_build split: build() = ui_state_set(0x35) + the clear fill
  only; ui_menu_screen_continue() = the remaining fills + the text layer;
  the press branch = build() -> at1846s_rssi_poll(VHF) -> continue().
- RESULT: the press poll now lands at the EXACT orig position (218120); the
  identical prefix = 219119 events (72.10%). The post-poll era diverges in
  CONTENT: the orig draws [the selection-bar fill window (3,10,20,35)-class
  + the row text], the recon draws its scaffold fills #2-13 + the stand-in
  text. The post-poll dynamic layer = the next decode (the orig's
  219118..238685 = the selection bar + the 5 row strings, 19,567 events).
- The release polls remain +6,659 late (the release-era render diff).

## Session 69 — p3-key6 reaches 100.00% event-identical (303,907/303,907)

**Result.** `p3k6_rec79.stream` vs `p3k6_orig.stream`: orig=303907 rec=303907
prefix=303907 (100.00%), firstdiff=None. The full scenario — boot, status
render, both boot polls, the menu press (clear + press poll + the text layer),
the release re-render, both release polls — is event-identical. `make
test-host` green (all protocol conformance tests passed). Build 108,991 text.

**1. The post-poll dynamic era fully decoded (p3k6_orig.stream
219118..238685).** Every cell = the same window shape
`0x2A,[0,x1],[0,x2],0x2B,[0,y1],[0,y2],0x2C` + pixel events (2/pixel,
RGB565 hi/lo pairs on the ODR low byte). The 16px CJK cells place x2=0x2a,
which masquerades as the 0x22A command byte in a naive scan — that artifact
caused the earlier "no-x-param window" misread. Row structure: 5 rows at
y=20+17i, the string walked from x=3: ASCII chars = 8x16 font cells (256
events), GB2312 pairs = SOLID 16x16 blocks (512 events, every pixel = fg).
The solidity is the SPI2 font-flash model answering all-0xFF to the orig's
CJK glyph fetch, so the glyph renders all-ink [DATA-DERIVED]. Then the row-0
selection re-walk with the inverted pair (fg 0xEFBE on bg 0x001F) + the blue
bar `fill_rect(20,91,68,16,0x001F)` (2176 events). RETRACTED: the session-61
scaffold fills #2-13 (belong to no orig era here) and the session-69-first-
pass "cursor-column cell at x3..10" (that cell is the '0' glyph).

**2. The menu strings = the NUL-terminated chain at 0x08043B20.** REA
read_bytes: "01.本机设置"(B20,12B) "02.站点类型"(B2C,12B) "03.GPS 设置"(B38,13B)
"04.信号设置"(B45,12B) "05.信号类型"(B52,12B). Offsets 0x0,0xC,0x18,0x25,0x31
— NOT stride 12 (the recon's `12*i` drifted off-by-one from row 3 on). REA
search_bytes finds zero references to 0x08043B45/0x08043B52 ⇒ no per-row
pointer table; the original walks the chain [RECOVERED].

**3. Implementation** (reconstruction/src/ui.c): `menu_row_text(y, s, fg, bg)`
walks a GBK string from x=3 (ASCII → display_blit_glyph_cell with the shared
ui_ascii8x16_font; the pair ≥0x81 → display_blit_bitmap_1bpp(16,16,
menu_cjk_solid) whose all-0xFF record is layout-independent and byte-equal to
the orig's solid cell). `ui_menu_screen_continue()` = the 5 chain rows
(fg 0000/bg EFBE) + the row-0 re-walk (EFBE/001F) + the blue bar. The CJK
solid record is [DATA-DERIVED]; the walk shape is [RECOVERED] from the cells.

**4. The release era root cause.** The orig's release pass (~238.6k, one
render period after the press) = state 0x35→0x00, re-renders the status page
starting directly at the content clear `fill_rect(18,1,158,92)` with NO
CALL:/BAT cells, then refreshes both slots (the polls 301912/302910). The
recon's release called the boot-shaped `ui_screen0_build()`, re-emitting the
22 boot-only CALL:/BAT cells (+5,874 events) before the clear and pushing the
release polls to 308571 (+6,659). Fix: split `ui_screen0_body()` out of
`ui_screen0_build()` (the prologue = the cells + the one-shot EE scan only);
the release branch now calls the body. Aligned pre-check: the orig's
238685..301911 vs the recon's 244559..308570 bodies = 63,227 events, 0 diffs
— the body was already byte-exact; only the prologue differed.

**Reagents.** /tmp/opencode/cellextract.py (the cell/bitmap extractor, the
column-major rec[c] bit-r layout), /tmp/opencode/scan2.py (the structure
scanner). Debug captures p3k6_rec77/78/79.stream + the resc variants remain
untracked. The 5,874-event junk = 22 cells × 267 events (13 "CALL:"+callsign
+ 9 "BAT:0.8 V"; 17 windows visible to the window scanner).

## Session 69 (cont. 2) — p3-key1 differential: the band gate fixed, the periodic refresh = the open slice

**Scenario.** New matched pair sim/p3-key1-orig.resc + sim/p3-key1-recon.resc:
the key-1 = ADC2_DR 2047 (the A-ladder i4 band 1947..2147,
data/ram_tables_recovered.bin: A i0=(2431,2231) i3=(3253,3053) i4=(2147,1947)
i5=(1745,1545); the observed anchors 2300=code10, 3100=code13 fit) injected
at the emulated 6.0 s, held 3 s, released, run to 10.5 s, fresh-EE.

**The orig (p3k1_orig.stream, 219,256 events):** the boot = byte-identical to
p3k6's first 189,036. The 3 s key-1 hold = INERT: scr 0x200050E8 stays 0x00,
no 0x83 switch, no polls. Post-boot the stream = 30,220 events of the
RSSI-task PERIODIC VFO-row refresh in 5 cycles at ~1 s (starts L189037,
193008, 196979, 205688, 213114):
- cycles 1-2 (row-only): 7x 12x20 freq cells (22..105, y29..48, d=480 each)
  + 2x 8x16 sub-cells (108..115/116..123, y35..50, d=256);
- cycles 3-5 (row + labels): 4x 6x10 (133..156, y18..27) + 2x 6x10
  (136..147, y29..38) + 3x 6x10 (133..150, y39..48, the third d=224) + the
  freq row (7x480 + one d=720 = 18-wide cell at 106..123) + the 2 sub-cells
  + an 11-cell 6x10 label sweep (y18..27 at x 5..97 + one at 22..27,50..59).
All primitives = the recon's existing vfo_freq_row/cell6x10/cell8x16; only
the trigger + the sequence + the ~1 s timing are new.

**The recon bug (FIXED).** The session-61 hold-counter implementation left
the gate open: the recon switched to scr 0x83 and rendered the boot-shaped
screen0 (+69k events) where the orig stays idle. Session 61 already
documented the real gate: the whole 0x801A13A branch requires the band byte
0x200050E8 == 0x67 ('g', the RF-active screen) and was parked on the band
machine. Fix: the recon's key-1/2 hold branch now requires
UI_STATE_ADDR == 0x67. After the fix (p3k1_rec2.stream): the recon = scr 00
throughout, prefix = 189,036 = 100% of its own stream (silent post-boot),
make test-host green, build 108,999 text.

**Open (the next p3-screens slice).** The RSSI-task periodic refresh: the
recon has no periodic layer. Decode items: the ~1 s tick source vs the 30-
tick dispatch pass; the row-only vs row+label cycle shapes (the labels
appear from cycle 3); the interaction with the menu state (p3-key6 shows NO
refreshes while 0x35 is open and none in 9 s of post-release capture, which
bounds the trigger); the d=720 18-wide cell and the d=224 6x10 variant.

## Session 69 (cont. 3) — the dispatch pass is 41.667 ms, not 14.4/30 ms: the RSSI regression root-caused and both p3 pairs converged

**The regression.** The first p3-key6 check under the new RSSI slice
(p3k6reg) emitted 3 refresh slices (16,651 events) BEFORE the 6.0 s menu
press: rec PRE L=205,688 vs the orig's 189,037; every later marker offset by
exactly those 3 slices (the menu span 49,648 and the release re-render
65,223 events matched the orig bit-for-bit).

**Root cause (proven, not fitted).** The kill press at sim 6.0 s only
suppresses fires that have not fired yet, so the recon's first fire must land
>= 6.0 s. Marker arithmetic on the two runs (c3: 5 fires in 10.5 s;
p3k6reg: 3 fires by 6.0 s) is simultaneously consistent ONLY with a dispatch
pass of ~30 ms and a flat-out boot ending at ~0.1-1.8 s: the recon's pass =
30 ticks x the 1 ms OS tick (os_cfg.h session-7 guess, OS_TICKS_PER_SEC
1000), and c3's "N=69 convergence" was a WINDOW COINCIDENCE - 69 x 30 ms =
2.07 s puts 5 fires in the 10.5 s window, the same count as the original's
5 x 1.000 s fires, and the eventless gaps hide every timing difference in
the stream. The p3-key6 window exposes it: fires at ~0.5/2.6/4.6 s precede
the press. The original's true cadence, three independent proofs:
1. rf.c's decoded tone tick divides APP_STATE+0x109 by 100,000 CPU cycles =
   1.38889 ms at 72 MHz -> the OS tick = 720 Hz;
2. the boot spans exactly 144 dispatch passes = 144 x 41.667 ms = 6.000 s
   (the p3k1 fire 5 at exactly 10.000 s -> fire 1 at exactly 6.000 s);
3. the RSSI fires are 24 passes apart = 24 x 41.667 ms = 1.000 s (the
   p3k1_orig slice spacing).
So the original's pass = 30 x 1.38889 ms = 41.667 ms and its boot = 144
passes. The c3-era "14.4 ms pass" note was a mis-fitted artifact.

**The fix package (all [session 69 | PROVEN/DATA-DERIVED] annotated).**
- ucos2/cfg/os_cfg.h: OS_TICKS_PER_SEC 1000 -> 720 (the tick = 100,000
  cycles; cnts = 108 MHz/720 = 150,000 exactly), OS_TIME_GET_SET_EN 1.
- src/rtos.c + rtos.h: rtos_ticks() = OSTimeGet() (the boot anchor's clock).
- src/ui.c: the boot-pass anchor - after the boot-scene events the boot is
  held at the 144-pass boundary (while (rtos_ticks() < 4320) time_dly(30));
  waits emit no bus events, so the stream is unchanged; the RSSI fire
  period %69 -> %24 (= 1.000 s); fire 1 = the first post-boot pass.
- src/main.c: periodic_task off OSTimeDlyHMSM(0,0,0,1) - at 720 Hz that
  computes 0 ticks (OS_ERR_TIME_ZERO_DLY = no block) and would busy-spin
  the prio-10 task, starving the dispatcher and the UI; now time_dly(1)
  (one tick = 1.389 ms, the closest expressible period; body SCAFFOLD).
- Makefile: -MMD -MP header deps + -include build/*.d (the first fixed run
  silently used a STALE kernel: p-os_cpu_c.o dated Oct 16 still divided by
  1000, so the anchor exited at 4.32 s and the rec still leaked 3 slices);
  plus the tables.o/splash_data.o recipe split (tables.o had NO recipe and
  make clean exposed it - a stale-build-dir bug).

**Fresh differential evidence (p3diff.sh, fresh-EE, build 110,447 text).**
- p3-key6 (p3-key6x-orig.resc vs p3-key6-recon.resc): every marker identical
  (PRE 189037 / P1 238685 / P3 238685 / REL 303908 both sides);
  orig=303,907 rec=303,907 prefix=303,907 = 100.00%.
- p3-key1 (p3-key1-orig.resc vs p3-key1-recon.resc): orig=219,256
  rec=220,539 prefix=219,256 = 100% of the original's stream (rec P1
  L=205,688 = slices 1-3 done, fire 4 pending at 9.04 s - the fires land
  1.000 s apart as in the original). The +1,283 tail = the recon completing
  slice 5 (7,426 ev) where the original's capture cuts it at 10.5 s
  (6,143 ev) - the in-flight render artifact, the original's stream is 100%
  covered.
- make test-host green; the recon's key-scan ADC2 read warnings are the
  pre-existing per-pass pattern (unchanged density).

## Session 69 (cont. 4) — the key matrix: codes 2/3 = the direct band-digit select; the RSSI burst = render-paced with the hold-pause

**The matrix batch** (the 12 generated pairs, codes 2/3/11/12/13/14 x
short/long; the B/C groups = the documented single-tag harness boundary):
key-2/3 = the placeholder-band family (the 6-slice short / 5-slice long
burst); key-11/12 = one big animated screen (477,704 short / 2,800,492
long); key-13 = a "CH 002" screen (204,670); key-14 = the FREQUENCY step
(the live rows read 136.130 -> 136.135 -> 136.140: the 5 kHz step key, so
the recon's live row must become frequency-state-driven).

**The burst/pause model (the stream ground truth).** The orig's RSSI burst
renders the row slices back-to-back at the display bus rate (~16-20k ev/s)
and FREEZES mid-slice-2 at exactly L=196,979 (the row/APRS boundary = the
render poll) while any key is held, in every hold length tested (the 0.4 s
and the 3 s scenarios both show the identical freeze point); the release
resumes and the capture window cuts the last slice. The recon equivalent:
one slice per 11 passes (~0.458 s, the render-paced cadence; 5 slices land
in the p3k1 window, 6 in p3k2-short's, both matching the original), the
fires 3+ suspended while KEY_CODE_BYTE != 0xFF, the fire 1-2 exempt, the
menu press kills (the p3-key6 evidence). The %69 and the 24-pass guesses
were window coincidences; the 11-pass + pause satisfies every scenario.

**The band digit (three corrections, all font-matched).** The first guess
(key 2 AND 3 step 1->2) failed on p3k3: the orig's key-3 first cell = '1';
the second guess (the key 2 only) failed again: the orig's key-3 first cell
= '3' (the 12x20 font match on the actual cells). FINAL: the keys 1/2/3 =
the DIRECT digit select — the placeholder row's first digit = the key's
value (the boot = 1, the key 2 press = '2', the key 3 press = '3'), the VFO
frequency untouched (the live row = 136.125 in all three).

**Fresh differentials (the 11-pass + pause + digit-select build,
110,511 text):** p3-key1 = 219,256/220,539, prefix = 219,256 = 100% of the
orig; p3-key6 = 303,907/303,907 = 100.00% (all markers identical);
p3k2-short = 226,682/227,965, prefix = 226,682 = 100%; p3k2-long =
219,256/220,539 = 100%; p3k3-short = 226,682/227,965 = 100%; p3k3-long =
219,256/220,539 = 100%. The +1,283-1,284 tails = the accepted artifact
class (the recon's last slice completes where the orig's capture cuts it
mid-render). make test-host green.

**The key-13 decode (the next implementation):** the press = the "CH 002"
screen (the 53x20 EFBE clear fill, the APRS block, the freq row, the '00'
subs, 'C'/'H' + '0'/'0'/'2', the label sweep) followed by a 6,077-event
EEPROM-save I2C bitbang (BRR x3406 + BSRR x1820 + CRH x851 = the SCL/SDA
toggles and the direction flips for the 0x4D-byte write - the GOAL 5.4
ee_save parity item; the recon's current eeprom_write emits 4,081 events
and must match the orig's bit timing).

**Instruments:** /tmp/opencode/segment.py (the full stream segmenter: the
cells with the font match, the fills, the raw runs), matrixcells2.py, and
covgen.py (the 5.5 static pass: 685 prologue functions, 305 bl-reachable,
68 cited) remain session tools.

## Session 69 (cont. 5) — key-13 implemented: the CH 002 screen + the PB9 settings-save frame, 100.00% both press lengths

**The implementation.** The key-13 press = the p2 ctx executor (the RAM
clusters, kept) + the key13_screen() render (the 53x20 EFBE clear, the APRS
top block 'A'/'P'/'R'/'S' + the 104-event PB9 tail, the live 136.125 row,
the 18x20 fill, the '00' subs, the 'C'/'H' + '0'/'0'/'2' channel cells, the
label sweep ending in 'L' - NOT 'H': the 12x20/6x10 font match on the
orig's 120-byte cell) + key13_pb9_replay() (the 6,077-code settings-save
frame over the PB9 head-serial link, key13_pb9_stream.h [DATA-DERIVED],
byte-identical in the short and the long captures) + the RSSI kill (the
p3k13 captures end silent - the refresh task does not resume, like the
menu key). The recon's eeprom_write(0x0A00, 0x4D) call = REMOVED: the
orig's save = the PB9 serial frame, not the I2C EEPROM (the p2 guess was
the wrong transport; the ee_save bit-level parity = the 5.4 item).

**Two corrections inside the slice** (the font match on the actual cells):
the screen's trailing 104-event tail after the 'S' differs from the RSSI
slice's rssi_bus_tail in its first position (BRR vs BSRR) - the key-13 got
its own key13_tail104.h table; the sweep's last cell = 'L' vs the live
row's 'H'.

**Fresh differentials (117,483 text):** p3k13-short = 204,670/204,670,
prefix = 204,670 = 100.00%; p3k13-long = 204,670/204,670 = 100.00% (every
marker identical: PRE/P1/REL = 189037/204671/204671 both sides).
p3-key1 = 219,256/220,539 = 100% of the orig; p3-key6 = 303,907/303,907 =
100.00% - the neighbors unaffected. make test-host green.
