# Building HG-UV98 firmware

The radio has two processors and therefore two separate firmware images:

| Target | Build output | Toolchain |
|---|---|---|
| Main MCU reconstruction (GD32F103, ARM) | `reconstruction/build/hguv98.bin` | GNU Arm Embedded GCC; local or Docker |
| APRS coprocessor (STC8A8K64D4, 8051) | `build/aprs/UV98_868_220430_D4.hex` | Licensed Keil C51 / µVision `UV4` |

These are not interchangeable. The main-MCU reconstruction is experimental
and has not been run on real hardware; read [BUILD_AND_FLASH.md](./BUILD_AND_FLASH.md)
before considering it for a radio.

## Build both images

Install Docker and Keil C51, make the Keil `UV4` command available on `PATH`
(or provide its executable path), then run from the repository root. Keil's
`UV4` command must be runnable by the host shell (typically Windows with
Git Bash); it is not run inside Docker.

```sh
make
```

This builds the ARM reconstruction inside Docker and builds the APRS firmware
with the installed Keil compiler. For a non-default Keil location:

```sh
make aprs KEIL_UV4=/path/to/UV4
```

The licensed Keil compiler is intentionally not copied into the Docker image.
The APRS build therefore runs on the host where Keil is installed; the ARM
build is self-contained in Docker.

## Build one target

Build the ARM reconstruction with a local `arm-none-eabi-gcc` toolchain:

```sh
make main
```

Or build it in Docker:

```sh
make main-container
```

Build just the APRS coprocessor firmware with Keil:

```sh
make aprs
```

The Keil project is `UV98_868_220430_D4/UV98_868_220430_D4.uvproj`. Its
configured output is Intel HEX; `make aprs` checks that the build creates that
file and copies it into `build/aprs/`. A successful build only establishes
that the project produced an image: verify that the radio's firmware-update
procedure accepts this HEX format before flashing. Do not select the
reconstructed ARM `.bin` in the APRS updater.

**The APRS result is not yet a verified radio-update package.** The checked-in
Keil project still names the target as Atmel AT89C55 (20 KB ROM, 24 MHz),
while the radio's APRS MCU is STC8A8K64D4 (64 KB flash, 22.1184 MHz). Also,
the documented radio upgrade flow expects a `.bin`, whereas this Keil project
emits Intel HEX. Confirm/correct the target configuration and establish the
required HEX-to-update-image process before using a generated file on a radio.

The ARM reconstruction's `.bin` is a raw application image for the SWD
procedure in [BUILD_AND_FLASH.md](./BUILD_AND_FLASH.md), not a vendor updater
package. That reconstruction has never run on real hardware.

Run the host-side protocol tests with:

```sh
make test-host
```

Remove generated build outputs with `make clean`.
