# REA Reverse Engineering Prompt — HG-UV98 Main MCU

## Purpose and Legal Scope

This is a clean-room-style reverse engineering effort for **interoperability, preservation, and personal/community firmware development** of the HG-UV98 main MCU firmware, whose source was never released.

- Reconstructed code must be written as original, readable C, not as mechanical decompiler dumps.

---

## Operating Rules (Mandatory)

### 1. Discover before acting
Inspect the installed REA MCP tools (names, descriptions, schemas, current analysis state), the repository, `git status`, history, branches, submodules, firmware files, documentation, and any locally available tools (disassemblers, decompilers, emulators). Do not assume tool names, arguments, or capabilities from this prompt. Use only the tools relevant to the current phase.

### 2. Evidence-driven analysis
Everything in this prompt is a **hypothesis until verified**. For each significant finding record:
- The claim and a confidence level (confirmed / probable / speculative).
- Evidence: addresses, bytes, cross-references, disassembly, call sites, strings, or test results.
- Alternative explanations and unresolved questions.
- The next action that would confirm or refute it.

Never invent implementations or claim a structure, protocol, or algorithm is recovered without evidence. Revisit earlier conclusions when new evidence contradicts them.

### 3. Incremental work
Start with reconnaissance, produce a short plan, then prioritize high-value functions and dependencies. Maintain a map of analyzed and unanalyzed regions. Avoid repeating expensive analysis when evidence is already sufficient.

**Session scope:** one session targets one subsystem or one clearly bounded question. Before ending a session, update the log and state what remains.

### 4. Functional recovery before binary equivalence
The primary goal is a maintainable, functionally faithful C reconstruction. Treat functional correctness and byte-for-byte equivalence as separate objectives. If exact reproduction is not achievable, document the differences and their likely causes; never alter output artificially to claim a match.

### 5. Validation
Validate through, in order of availability: static evidence, differential analysis between images, emulation (QEMU, Renode, Unicorn, or similar, if installed), and hardware tests (only if hardware is available). State explicitly which validation method backs each claim. **Without emulation or hardware, Level 3 claims are limited to "builds" and "consistent with static evidence"; do not claim runtime-verified behavior.**

### 6. Git and artifacts discipline
- Check `git status` and history before changing anything. Never assume a clean tree. Do not create an automatic initial commit.
- Never stage unrelated user changes. Never use broad staging (`git add .`) without reviewing the files.
- Commit coherent, verified milestones with meaningful messages. Do not commit failed attempts or disproved hypotheses unless they produced useful documentation or tooling.
- Track source, docs, reusable scripts, build configuration, and compact reports. Ignore (via `.gitignore`) generated binaries, dumps, analysis databases, and tool workspaces.
- Original firmware and reference data are **immutable**. Experiments and patches operate on copies; generated and comparison outputs are stored separately.

### 7. Research log
Maintain a concise, version-controlled log containing: current objective and success level achieved; verified layout, memory map, and entry points; functions and subsystems investigated (with addresses and evidence); findings, hypotheses, contradictions, and open questions; reproducible commands; and next recommended actions. Update it after significant findings and fold changes into the related milestone commit (no per-update commits). Put bulky generated data outside the log and reference it by path. On resumption: read the log, check Git status, verify referenced artifacts exist, then continue.

---

## Context

The HG-UV98 is a Chinese dual-band APRS handheld (Lanch / Wuxi Venus Information Technology) with two MCUs:

| | Main MCU (**TARGET**) | APRS coprocessor |
|---|---|---|
| MCU | GD32F103RET6 | STC8A8K64D4 |
| Architecture | ARM Cortex-M3 | 8051 |
| Clock | 108 MHz | 22.1184 MHz |
| Flash / RAM | 512 KB / 64 KB | 64 KB / 2560 B XRAM |
| Firmware | Binary only | Source available (this repo) |
| Role | FM radio, display, menus, RF, channels | APRS, GPS, Bluetooth, sensors |

The APRS coprocessor source is already in this repository and is useful for understanding the inter-MCU protocol.

### Available resources

**Firmware images (`firmware/`):**
- `MAIN-MCU-HG-UV98-V1.0.1-190416.bin` (267 KB): original image from a groups.io attachment, dated 2019-04-16; believed to be the earliest public version. **Primary target.**
- `V1.0.1_190619-KT.bin` (496 KB): community-modified image attributed to KT2KT. **Diff target.**

**Known anomalies to investigate first (do not assume an explanation):**
- The two images differ greatly in size (267 KB vs 496 KB), which is inconsistent with "UI text changes only." Candidate causes: padding, different layout or base offset, bootloader or data regions in one image, appended assets. Establish this during reconnaissance.
- Dating is inconsistent: the filename says 190619 (June 2019) while the description places the modification in April 2019. Treat provenance and dates as unverified.

**Documentation:** `docs/ARCHITECTURE.md`, `FIRMWARE_INVENTORY.md`, `SOURCE_CODE_ANALYSIS.md`, `HARDWARE.md`, `FIRMWARE_VERSIONS.md`, `COMMUNITY.md`.

**Submodules:** `repos/pwnj0hn-HG-UV98/` (original packages, CPS, manual), `repos/marrold-HG-UV98/` (community issues and reports), `repos/hg-uv98-firmware/` (8051 source).

**External:** GigaDevice GD32 SDK (register definitions, peripheral libraries), Keil MDK / GCC ARM toolchains, ST-Link/OpenOCD (only if hardware extraction is needed), CHIRP driver `hg_uv98.py` (clone protocol reference).

### Prior claims (hypotheses, with their source)

| Claim | Source | Status |
|---|---|---|
| VHF 136–174 MHz, UHF 400–470 MHz, 128 channels | Changelogs / manual | Unverified |
| Color LCD with day/night mode | Changelogs | Unverified |
| 2 programmable PF keys + PTT | Manual | Unverified |
| Main MCU talks to the APRS MCU over UART4 using AT-style commands (`$E01`, `$E02`, `$E00`, `$F01`, `$A09`, …) | Repo docs / 8051 source | Verify in the ARM image; the UART number is an assumption |
| Shared AT24C512 EEPROM (64 KB) | Repo docs | Unverified |
| Programming uses a USB-to-UART bridge (likely CH340) | Community / hardware docs | **Unverified:** if a bridge is present, the MCU speaks only UART, not USB. Determine where the clone protocol is implemented. |
| Battery voltage telemetry | Changelogs | Unverified |
| KT image changes "Sure" → "OK" and menu descriptions | Community | Verify via full diff (code, constants, data, layout) |
| V1.2 uses hardware ID string `P3LL` vs `P3107` | Changelogs | Unverified |

---

## Objectives

### Primary
Recover the firmware's architecture and behavior: main flow and interrupt handling; radio control, frequency management, and channel storage; display and menu state machines; APRS coprocessor communication; EEPROM access and the serial programming protocol; hardware initialization and peripheral drivers. Produce documented, maintainable C with meaningful names, explicit uncertainty markers, and a reproducible build.

### Secondary
Determine whether byte-for-byte reproduction is achievable (toolchain, startup code, linker layout, flags). Measure the achievable match and explain differences. Never sacrifice correctness or evidence quality for the match.

### Success levels

| Level | Meaning | Deliverables |
|---|---|---|
| 1. Architecture | Memory map, entry points, subsystems, call relationships documented | Verified memory map and load address, vector table / reset / startup analysis, subsystem list, call graph or list |
| 2. Behavior | Key functions, data structures, and protocols reconstructed with evidence | Function signatures and descriptions with evidence, EEPROM/data structures, UART and clone protocol docs, string and menu map |
| 3. Functional implementation | Reconstructed code builds and reproduces validated behaviors | Organized C source, build system, inferred-vs-confirmed markers |
| 4. Binary equivalence | Reproducible build matches the original byte-for-byte | Toolchain, flags, linker script, startup code, accounting of offsets/padding/excluded regions, comparison artifacts stored separately |

Report progress per level. **Never claim a level without demonstrating it.** A well-documented partial result at Level 2 is a legitimate success; if full reconstruction is not supported by the evidence, deliver the most useful verified partial result instead of fabricating missing functionality.

---

## Reconstructed Code Quality

For each reconstructed function, record: original address or range, callers/callees, supporting evidence, confidence, and status (**recovered** / **inferred** / **scaffold**). Keep original addresses traceable. Prefer small, verifiable implementations with explicit unresolved behavior over mechanical decompiler translation. Do not claim that code compiles, passes tests, or matches the binary without running the corresponding check.

---

## Priority Order and Decisions

1. **Reconnaissance:** format, image boundaries, load address, vector table, memory map, executable regions, strings, and version differences (starting with the size and date anomalies).
2. **Level 1**, 3. **Level 2**, 4. **Level 3**, 5. **Level 4.** Do not attempt Level 4 before Level 2.

If layout or load address is uncertain, document the competing hypotheses and continue with analyses valid under all of them. Do not require every region to be identified before moving to the highest-value subsystem.

Choose the next action by expected information gain. When blocked: state what is unknown and what evidence is missing, consider alternatives, try a bounded targeted investigation, record the outcome, and change method or assumptions before retrying. When a subsystem is sufficiently understood, document its boundaries, dependencies, confidence, and gaps before moving on.

At each milestone report: what was established, the supporting evidence, what remains unknown, tests actually performed with results, and the next recommended action.

---

## First-session Requirements

Deliver: (1) an inspection of the relevant REA tools and local analysis tools, (2) an inspection of both images and the documentation, (3) a preliminary reconnaissance report, (4) a log initialized with the investigation plan, and (5) a prioritized plan based on the findings. Do not start large-scale C reconstruction. Do not stop after the plan if a high-value, low-risk analysis (e.g., vector table, image diff, strings) can already be done.
