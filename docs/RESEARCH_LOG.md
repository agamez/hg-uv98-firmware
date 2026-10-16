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
