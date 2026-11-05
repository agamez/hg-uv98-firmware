# Building HG-UV98 firmware

The radio has two processors and therefore two separate firmware images:

| Target | Build output | Toolchain |
|---|---|---|
| Main MCU reconstruction (GD32F103, ARM) | `reconstruction/build/hguv98.bin` | GNU Arm Embedded GCC; local or Docker |
| APRS coprocessor (STC8A8K64D4, 8051) | `build/aprs/UV98_868_220430_D4.hex` | Licensed Keil C51 / µVision `UV4` |

These are not interchangeable. The main-MCU reconstruction is experimental
and has not been run on real hardware; read [BUILD_AND_FLASH.md](./BUILD_AND_FLASH.md)
before considering it for a radio.

## Build the verified target

Install Docker and run this from the repository root:
```sh
make
```

This builds the main-MCU reconstruction and writes
`reconstruction/build/hguv98.bin`. The ARM image builds in Docker, but it is
an experimental reconstruction and has not been validated on radio hardware;
see [BUILD_AND_FLASH.md](./BUILD_AND_FLASH.md).

## Build the APRS coprocessor

The APRS source requires licensed Keil C51 / µVision `UV4`, which is not
included in Docker. Make `UV4` available to the host shell (typically Windows
with Git Bash), then run:

```sh
make aprs
```

To select a non-default Keil executable:

```sh
make aprs KEIL_UV4=/path/to/UV4
```

This target expects the project's configured Intel HEX output and copies it
to `build/aprs/`. It cannot be verified in the Linux build container because
the proprietary Keil compiler is unavailable.

## Build both targets

After installing Keil C51 as above, build both targets with:
```sh
make all
```

`make all` needs Keil `UV4` and will fail at the APRS target if it is missing.
The Keil project currently identifies an Atmel AT89C55 (20 KB ROM, 24 MHz),
not the radio's STC8A8K64D4 (64 KB flash, 22.1184 MHz). Correct and validate
that configuration before relying on its output.

## Alternative ARM build

To build the ARM reconstruction with a local `arm-none-eabi-gcc` toolchain:

```sh
make main
```

Run the host-side protocol tests with:

```sh
make test-host
```

Remove generated build outputs with `make clean`.
