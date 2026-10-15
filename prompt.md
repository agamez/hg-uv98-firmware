# REA Reverse Engineering Prompt

---

## Operating Rules — Mandatory

### 1. Discover Before Acting

Before using REA, inspect the available MCP tools, their descriptions, parameter schemas, current analysis state, and supported workflows. Do not assume tool names, arguments, or capabilities from this prompt. Use the actual installed interfaces.

Inspect the repository, Git status, submodules, firmware files, documentation, and available analysis tools before modifying anything.

Do not overwrite, delete, flash, patch, or otherwise modify the original firmware binaries or existing analysis artifacts. Preserve the initial repository state and any pre-existing user changes.

### 2. Evidence-Driven Reverse Engineering

Treat all hardware details, addresses, compiler assumptions, protocol descriptions, and behavioral claims in this prompt as hypotheses until verified.

For every significant finding, record:
- The claim and confidence level.
- Supporting evidence: addresses, bytes, cross-references, disassembly, call sites, strings, or test results.
- Alternative explanations and unresolved questions.
- The next action needed to confirm or refute the claim.

Clearly distinguish confirmed facts, probable interpretations, and speculation. Never invent function implementations or claim that a structure, protocol, or algorithm has been recovered without supporting evidence.

Maintain a persistent research log so the analysis can be resumed after context loss.

### 3. Incremental Investigation

Begin with a reconnaissance phase. Establish the binary format, image boundaries, likely load address, vector table, memory map, executable regions, strings, and differences between firmware versions.

Produce a short investigation plan based on those findings before starting large-scale reconstruction.

Work incrementally. Prioritize high-value functions and dependencies, and maintain a map of analyzed and unanalyzed regions. Avoid repeating expensive analysis when existing evidence is sufficient.

Use REA and other available tools as appropriate. Do not invoke every tool merely because it exists.

### 4. Functional Recovery Before Binary Equivalence

The primary goal is a maintainable, functionally faithful C reconstruction of the original firmware.

Do not assume that recompilation can reproduce the original binary byte-for-byte. First establish the likely compiler, toolchain version, optimization settings, linker layout, startup code, and original build configuration.

Keep functional correctness and binary equivalence as separate objectives. If exact reproduction is not achievable, document the differences and their likely causes rather than modifying the output artificially to claim a successful reconstruction.

### 5. Validation and Reproducibility

Validate reconstructed behavior against the original firmware using static evidence, differential analysis, and hardware tests or emulation where available.

Keep original binaries immutable. Store generated files and comparison outputs separately.

Every significant conclusion must be independently checkable from the recorded evidence. Revisit earlier conclusions when new evidence contradicts them.

### 6. Git Discipline

Inspect `git status` and the existing history before making changes. Never include unrelated user changes in commits.

Commit coherent, reviewable milestones after verifying the corresponding results. Do not create commits solely because an action failed or a hypothesis was disproved.

Track source code, useful scripts, documentation, and reproducible configuration. Exclude large generated workspaces, temporary files, and reproducible build outputs unless there is a concrete reason to version them.

---

## Context

The HG-UV98 is a Chinese dual-band APRS handheld radio (Lanch/Wuxi Venus Information Technology) with a **dual-MCU architecture**:

| Component | Main MCU (TARGET OF THIS TASK) | APRS Coprocessor |
|-----------|-------------------------------|-------------------|
| MCU | **GD32F103RET6** | STC8A8K64D4 |
| Architecture | **ARM Cortex-M3 (32-bit)** | 8051 (8-bit) |
| Clock | 108 MHz | 22.1184 MHz |
| Flash | 512 KB | 64 KB |
| RAM | 64 KB | 2560 bytes XRAM |
| Firmware | **Binary only (proprietary)** | Source available (this repo) |
| Role | Radio FM, display, menus, RF, channels | APRS, GPS, Bluetooth, sensors |

The source code of the **APRS coprocessor (8051)** is already available in this repository and was released by the manufacturer. We need the same for the **main ARM MCU**.

---

## Available Resources

### 1. Binary Firmware Files (in `firmware/` directory)

- **`firmware/MAIN-MCU-HG-UV98-V1.0.1-190416.bin`** (267 KB) — Original firmware from groups.io attachment, dated 2019-04-16. This is the **earliest publicly available version** and should be the primary target.
- **`firmware/V1.0.1_190619-KT.bin`** (496 KB) — Modified firmware by KT2KT (April 2019). Community reports indicate UI text changes. Compare both images to establish the complete set of changes, including code, constants, data, layout, and build differences. Useful as a **diff target**.

### 2. Documentation in Repository

- **`docs/ARCHITECTURE.md`** — Dual-MCU system diagram, UART protocol between CPUs, shared EEPROM map
- **`docs/FIRMWARE_INVENTORY.md`** — All known firmware versions, flash procedures, version history
- **`docs/SOURCE_CODE_ANALYSIS.md`** — Analysis of the APRS coprocessor (useful for understanding the communication protocol)
- **`docs/HARDWARE.md`** — Hardware specs, sensor details, telemetry formats
- **`docs/FIRMWARE_VERSIONS.md`** — Version history with changelog details
- **`docs/COMMUNITY.md`** — Community resources, known issues, user reports

### 3. Submodule Repositories

- **`repos/pwnj0hn-HG-UV98/`** — Original firmware packages: binaries, CPS software, APRS configuration software, manual
- **`repos/marrold-HG-UV98/`** — Community repository with issues, firmware discussions, user reports
- **`repos/hg-uv98-firmware/`** — Source code for the APRS coprocessor (8051) — useful for understanding the UART protocol

### 4. External Resources

- **GigaDevice GD32 SDK**: Available at https://www.gd32mcu.com/en/download/ — Contains peripheral libraries, register definitions, and examples for GD32F103 series
- **Keil MDK ARM / GCC ARM**: Standard toolchains for ARM Cortex-M3
- **ST-Link/OpenOCD**: For firmware extraction from physical hardware
- **Chirp driver source**: `hg_uv98.py` (Python) from CHIRP project — reverse-engineered driver showing the clone protocol

### 5. Known Behavioral Information

From changelogs and community reports, the firmware:

- Implements FM radio functions: VHF 136-174MHz, UHF 400-470MHz, 128 memory channels
- Manages color LCD display with day/night mode
- Implements keypad/button interface (2x PF programmable + PTT)
- Handles audio: mic input, speaker output, PL/CTCSS/DCS tones
- Implements menu system with numbered options and sub-menus
- Communicates with APRS coprocessor via UART4 using custom AT-command protocol ($E01, $E02, $E00, $F01, $A09, etc.)
- Reads/writes AT24C512 EEPROM (64KB) — shared configuration storage
- Handles the serial programming protocol through the USB-to-UART interface; determine the actual division of responsibilities between the CH340 bridge and the main MCU.
- Implements battery voltage telemetry
- Changed in V1.0.1_190619-KT: "Sure" → "OK" button text, improved menu descriptions
- V1.2 versions have different hardware identification string (`P3LL` vs `P3107`)

---

## Objective and Success Criteria

Reverse engineer the main MCU firmware (GD32F103RET6, ARM Cortex-M3) of the HG-UV98 from the available binary images and supporting documentation.

### Primary objective

Recover the firmware's architecture and behavior, including:
- Main execution flow and interrupt handling.
- Radio control, frequency management, and channel storage.
- Display rendering and menu state machines.
- Communication with the APRS coprocessor.
- EEPROM access and USB programming protocols.
- Hardware initialization and peripheral drivers.

Produce a documented, maintainable C reconstruction with meaningful names, explicit uncertainty markers, and a reproducible build system.

### Secondary objective

Determine whether the original firmware can be reproduced byte-for-byte. Investigate the original toolchain, startup code, linker layout, compiler options, and binary layout. Measure the achievable match and explain remaining differences.

Do not sacrifice functional correctness or evidence quality in pursuit of a byte-for-byte match.

### Success levels

1. **Architecture recovered:** memory map, entry points, major subsystems, and call relationships documented.
2. **Behavior recovered:** important functions, data structures, and communication protocols reconstructed with supporting evidence.
3. **Functional implementation:** reconstructed code builds and reproduces validated behaviors.
4. **Binary equivalence:** a reproducible build matches the original binary byte-for-byte, with any image offsets, padding, or excluded regions explicitly accounted for.

Report progress against these levels. Never claim a level has been achieved without demonstrating it.

---

## Deliverables

### For each success level, deliver:

**Level 1 — Architecture recovered:**
- Memory map document with verified load addresses and regions
- Entry point analysis (vector table, reset handler, startup code)
- Major subsystem identification (radio, display, UART, EEPROM, USB)
- Call relationship diagrams or lists

**Level 2 — Behavior recovered:**
- Function signatures, behavioral descriptions, and supporting evidence
- Data structures recovered from EEPROM map and behavioral analysis
- Protocol documentation for UART bridge, USB clone, and EEPROM layouts
- String extraction and menu structure mapping

**Level 3 — Functional implementation:**
- Complete C source code organized in a maintainable structure
- Build system (Makefile or equivalent)
- Reconstructed code that builds and reproduces validated behaviors
- Explicit markers where implementation is inferred vs. confirmed

**Level 4 — Binary equivalence (if achievable):**
- Reproducible build matching original binary byte-for-byte
- Documentation of toolchain, compiler flags, linker script, and startup code
- Explanation of any remaining differences
- Comparison artifacts stored separately from source code

---

## Working Process — Commits, Artifacts, and Recovery

### Git Discipline

Before any changes, inspect `git status`, recent history, branches, and submodule states. Never assume the repository is clean.

Do not create an initial commit automatically. Preserve existing history and user changes. Never stage or commit unrelated modifications.

Commit coherent, verified milestones using meaningful messages. A disproved hypothesis does not need a commit unless it produced useful documentation, tests, or reproducible tools.

Keep original firmware binaries immutable. Store generated artifacts separately from source code.

Track source code, documentation, reusable scripts, build configuration, and compact analysis reports when useful. Do not commit generated binaries, temporary dumps, large analysis databases, or tool workspaces by default. Use `.gitignore` for reproducible artifacts.

Never use broad staging commands such as `git add .` without first reviewing the affected files.

### Persistent Research State

Maintain a concise, version-controlled research log containing:

- Current objective and achieved success levels.
- Verified binary layout, memory map, and entry points.
- Functions and subsystems investigated, with addresses and evidence.
- Confirmed findings, hypotheses, contradictions, and unresolved questions.
- Tools and scripts used, including reproducible commands where practical.
- Next recommended actions and their expected value.

Update the log after significant findings and before ending a session. Keep detailed generated data outside the main log and reference it by path. Do not create a separate commit for each log update. Accumulate log changes throughout the session and include them in the same commit as the related milestone. On resumption, read the log, inspect Git status, and verify that referenced artifacts still exist before repeating analysis.

### Investigation Decisions

Choose the next action based on expected information gain and relevance to the primary objective.

When blocked:
1. State exactly what is unknown and what evidence is missing.
2. Consider alternative explanations.
3. Try a bounded, targeted investigation using available tools.
4. Record the outcome and choose another path if the evidence does not support the current hypothesis.

Do not repeatedly retry failed approaches without changing the assumptions or method.

When a subsystem is sufficiently understood, document its boundaries, dependencies, confidence, and remaining gaps before moving on.

### Reconstructed Code Quality

Keep original addresses and function references traceable throughout reconstruction.

For each reconstructed function, record its original address or range, calling relationships, supporting evidence, confidence, and implementation status.

Distinguish between:
- Directly recovered behavior.
- Inferred behavior supported by multiple observations.
- Provisional scaffolding or unimplemented behavior.

Do not translate decompiler output mechanically into supposedly complete C source. Prefer small, verifiable implementations with explicit unresolved behavior.

Do not claim that code compiles, passes tests, behaves correctly, or matches the original binary without executing the corresponding verification.

### Validation and Reporting

At each milestone, report:
- What was established.
- The evidence supporting it.
- What remains unknown.
- Tests or comparisons performed and their actual results.
- The next recommended action.

Keep the original firmware and reference data unchanged. Any patches or experiments must operate on separate copies.

If the analysis cannot support a complete reconstruction, deliver the most useful verified partial result rather than fabricating missing functionality.

---

## Priority Order

1. **Reconnaissance**: Establish binary format, boundaries, vector table, memory map, and differences between versions
2. **Level 1**: Document architecture — entry points, subsystems, call relationships
3. **Level 2**: Recover behavior — functions, data structures, protocols, evidence
4. **Level 3**: Implement functional C code — build system, maintainable reconstruction
5. **Level 4**: Investigate binary equivalence — toolchain analysis, build reproduction, explain differences

Do not attempt Level 4 before achieving at least Level 2. Functional understanding must precede binary reproduction.

Reconnaissance must produce a useful preliminary report before expanding into exhaustive analysis. Do not require every memory region or function to be identified before proceeding.

If the image layout or load address remains uncertain, document the competing hypotheses and continue with analyses that remain valid under those hypotheses.

Once the entry point, probable memory map, major executable regions, and initial function map are sufficiently understood, proceed to the highest-value subsystem. Refine the map as new evidence becomes available.

### First-session requirements

The first session must produce:
- Inspect all available REA MCP tools, schemas, and capabilities, but investigate only those relevant to the current phase. Do not execute every tool merely to demonstrate discovery.
- Inspection of both firmware images and the available documentation.
- A preliminary reconnaissance report.
- A persistent research log initialized with the investigation plan.
- A prioritized investigation plan based on initial findings.

Do not begin large-scale C reconstruction during this first phase. Do not stop after producing the plan if a high-value, low-risk initial analysis can already be performed.

---

*This project aims to achieve what was done for the APRS coprocessor (8051 source was publicly released) but for the main ARM MCU — whose source was never made public. Successful reverse engineering would be a major contribution to the amateur radio community and enable full custom firmware development for this hardware platform.*