# Building, flashing, and what to expect — the HG-UV98 main-MCU reconstruction

This document covers three things: how to build the reconstructed firmware,
how it would be installed on a radio, and — most importantly — what the
resulting radio will and will not do, stated plainly.

**Read the status box first.** The reconstruction has been proven equivalent
to the original firmware *inside an emulator* (Renode), event-for-event,
across a large differential test battery. It has **never run on real
hardware**. Everything below is written with that boundary in mind.

---

## 1. Status at a glance

| Question | Answer |
|---|---|
| Does it compile to a complete firmware image? | Yes — `build/hguv98.bin`, ~267 KB, fits the app region. |
| Does it behave like the original? | In the emulator, yes — bit-exact bus traffic on every tested scenario (see `docs/COVERAGE.md`, `docs/RESEARCH_LOG.md`). |
| Has it ever run on real hardware? | **No.** |
| Is it safe to flash? | Probably recoverable if it fails (the vendor bootloader is never touched), but behavior on real silicon is unproven — see §6. |
| Does it contain any recorded/faked behavior? | No. Every bus transaction is produced by reconstructed code; only the original's own data assets (splash bitmap, fonts, string tables) are carried as data. |

The reference original is `firmware/MAIN-MCU-HG-UV98-V1.0.1-190416.bin`
(md5 `c061dab643fb67f722e8e0d720308926`). Keep it — it is the fallback.

---

## 2. What you need

- **Toolchain:** `arm-none-eabi-gcc` (binutils + newlib not required — the
  build is freestanding). The project was developed with the distro GCC at
  `/usr/bin`; pass the location with `TOOLCHAIN_PATH=` if yours differs.
- **GNU make.**
- **Renode 1.17+** (optional, but strongly recommended — it is the
  verification bench; see §5).
- **For flashing only:** an SWD debug adapter (ST-Link v2 clone or CMSIS-DAP)
  wired to the radio's SWD pads, plus OpenOCD or pyOCD. The radio's MCU is a
  GD32F103RET6 (Cortex-M3, 512 KB flash, 64 KB RAM) — it works with the
  standard STM32F1-class tooling.

---

## 3. Building

```sh
cd reconstruction
make TOOLCHAIN_PATH=/usr/bin/        # the firmware
make test-host                       # host-side protocol unit tests
```

Outputs:

| File | What it is |
|---|---|
| `build/hguv98.elf` | The full image with symbols. This is what the emulator loads. |
| `build/hguv98.bin` | The raw flash image, ~273 KB. This is what a flasher consumes. |

A healthy build reports `text` ≈ 103,283 bytes. If your toolchain produces a
`.bin` of hundreds of megabytes, you have an older `linker.ld` — `.data` must
carry a flash load address (`AT(ALIGN(LOADADDR(.tables) + SIZEOF(.tables),4))`);
with it, the boot-time data copy mirrors the original's own runtime init
(bss-zero loop + .data copy, decompiled at 0x0801D966).

### Image layout (where things go)

| Region | Address | Content |
|---|---|---|
| Vendor bootloader | `0x08000000 – 0x08003FFF` | 16 KB. **Not distributed, never touched by this project.** The real chip boots from here. |
| **Application** | `0x08004000 – ...` | The reconstruction: vector table at `0x08004000`, code, recovered data tables, and the UI string/asset blob mapped at `0x08040000`. Total span ~267 KB of the 496 KB app region. |

The reconstruction's own vector table is complete and live (unlike the stock
distributed file, whose reset vector was neutered by the vendor's packer —
that file is an update payload, not a directly bootable image).

### If you changed any code: re-derive the test pins

The emulator harnesses hook two functions by address. After **every** rebuild,
re-derive them and update any hook-based `.resc` scripts:

```sh
arm-none-eabi-nm build/hguv98.elf | grep -E " (rf_write|rf_read)$"
```

(Currently `0x08004f80` / `0x0800502c`. The `p3diff.sh` scenarios are
hook-free and unaffected.)

---

## 4. Verifying a build before thinking about hardware

The project's verification bench runs the original and the reconstruction
side by side in Renode and diffs the complete bus-event streams:

```sh
cd reconstruction/sim
bash p3diff.sh <name> "$PWD/ck4-orig.resc" "$PWD/ck4-recon.resc"
```

The CK4 scenario (a plain 3-second boot) should report
`189036/189036 (100.00%)`. That single line is the project's strictest
always-on regression gate. Many more scenarios exist (`p3-key*.resc` pairs,
the CK2/CK3 harnesses) — see `docs/RESEARCH_LOG.md` session 75 for the full
battery and its results.

---

## 5. Flashing

> **Warning.** This image has never run on real hardware. Flashing it is an
> experiment. Read §6 first. The original binary is your fallback.

The radio's MCU exposes SWD (SWDIO/SWCLK) on the PCB. Locate the pads, attach
your adapter, and program the application region — the 16 KB bootloader at
`0x08000000` is left alone, exactly as the vendor's own update does:

```sh
openocd -f interface/stlink.cfg -c "transport select swd" \
        -f target/stm32f1x.cfg \
        -c "program reconstruction/build/hguv98.bin 0x08004000 verify exit"
```

Notes:

- The `stm32f1x` target configuration handles the GD32F103's F1-compatible
  flash controller. pyOCD works too (`pyocd flash --base-address 0x08004000
  -t gd32f103re build/hguv98.bin`, or a generic Cortex-M target with the
  default flash driver).
- `program ... verify` reads the flash back and compares — let it finish.
- **Do not** write anything below `0x08004000`.

### Restoring the radio

- **Preferred:** re-flash the stock firmware through the vendor's own update
  tool/procedure. The distributed stock file is that tool's payload (its
  reset vector is intentionally neutered; the vendor flow patches it).
- **Via SWD:** the stock body can be written back at `0x08004000`
  (`program firmware/MAIN-MCU-HG-UV98-V1.0.1-190416.bin 0x08004000`), but be
  aware of the neutered reset vector — the stock update flow exists precisely
  because of that. When in doubt, use the vendor tool.
- The untouched bootloader remains your recovery path in every case.

---

## 6. What to expect

### Proven (emulator, event-for-event against the original)

- The complete boot sequence: vendor-era radio-chip initialization, display
  init, the splash screen, the boot scene, the status page.
- The settings lifecycle: the `$A00` request to the head-unit processor, the
  534-byte settings frame, the warm-boot path, the settings save on the
  menu key (all 77 bytes, byte-exact I²C traffic).
- The key paths exercised by the test scenarios: channel/frequency display
  and stepping, scan mode, the menu screen, the RSSI refresh cycles.
- The inter-MCU protocol checkpoints (490/490 observables, warm and fresh).
- The dual radio-chip (AT1846S) register programming, polls, and reads.

### Likely to work on hardware, but unproven

- Everything above, on real silicon. The logic is the same; only the
  physical environment differs.

### Known risks on real hardware (be prepared for misbehavior)

1. **Timing.** Delay loops are calibrated against the emulator, not
   cycle-exact. The decompiled structure is known (e.g. the main delay
   primitive is an n × 7000 empty loop), so a hardware-timer-based rewrite is
   straightforward — but the shipped image does not do it yet. Expect UI
   pacing to be approximately right, not exactly right.
2. **The settings-memory acknowledgment.** The driver was written against an
   emulator model whose acknowledgment always succeeds. A real 24Cxx EEPROM
   should also acknowledge normally, but a slow chip or different pull-up
   timing was never exercised.
3. **The head-unit (8051) partner.** In tests it is a scripted peer. A real
   partner chip speaks the same protocol (the command set comes from vendor
   sources), but real framing/timing edges were never exercised.
4. **Peripherals the emulator models only coarsely:** the battery/ADC readings
   (the harness feeds fixed values), the keypad ADC ladder (injected values),
   and the external SPI flash holding the Chinese-font glyphs (the read
   protocol is reconstructed; the chip itself was never emulated).
5. **Coverage corners.** Menu screens and key paths that no test scenario
   exercises were decoded only partially (mapped call graphs, not full
   mirrors). The coverage ledger (`docs/COVERAGE.md`) distinguishes these.

### Known behavior differences (documented, parked)

- **Warm-restart scan resume:** with saved settings present, the original
  resumes its channel scan at a saved position; the reconstruction starts
  from slot 0. Diverges at one specific bus event in the warm-boot window
  (RESEARCH_LOG session 75, "the warm scan-gap").
- **One long key-hold test** (key-11 long): matches the original for the
  first 99.93% of its stream; the tail diverges (the first-ever complete run
  of that scenario; the tail is an open analysis item).

### What you will NOT find

- Any recorded/replayed bus traffic — the criterion of the whole effort was
  its elimination, and the audit is clean.
- The vendor bootloader (never distributed, deliberately untouched).
- The 8051 companion's firmware (a separate chip; the project reconstructs
  the main MCU only).

---

## 7. Where the evidence lives

- `docs/RESEARCH_LOG.md` — the append-only engineering log; session 75 and
  its continuations cover this entire sweep, including every commit.
- `docs/COVERAGE.md` — the address-by-address completeness ledger (685 rows,
  zero unclassified).
- `docs/rea/README.md` — the decompile ledger and the validation classes.
- `reconstruction/sim/` — the emulator scenarios and the differential driver
  (`p3diff.sh`) used for every claim above.
