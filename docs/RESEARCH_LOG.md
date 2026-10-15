# HG-UV98 Main MCU — RE Research Log

Target: `firmware/MAIN-MCU-HG-UV98-V1.0.1-190416.bin` (primary), `firmware/V1.0.1_190619-KT.bin` (diff target).
MCU: GigaDevice GD32F103RET6, ARM Cortex-M3, 108 MHz, 512 KB flash @ `0x08000000`, 64 KB RAM @ `0x20000000`.

**Current success level: 1 (Architecture) — partial.** Memory map and image structure established with static evidence; subsystem inventory preliminary; no decompiled subsystems documented yet.

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

RAM: SP top `0x20006780` → ~26.5 KB static+heap below. Detail pending (O3).

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
