# HG-UV98 main-MCU firmware — Level-3 reconstruction build
#
# Cross-toolchain: arm-none-eabi-gcc (14.2.x tested). If not on PATH, point
# TOOLCHAIN_PATH at the install prefix (works with a dpkg -x extracted tree).
TOOLCHAIN_PATH ?=

CC      = $(TOOLCHAIN_PATH)arm-none-eabi-gcc
OBJCOPY = $(TOOLCHAIN_PATH)arm-none-eabi-objcopy
SIZE    = $(TOOLCHAIN_PATH)arm-none-eabi-size

CPUFLAGS = -mcpu=cortex-m3 -mthumb
CFLAGS   = $(CPUFLAGS) -std=c11 -O2 -g -Wall -Wextra -ffreestanding -nostdlib \
           -fno-builtin -ffunction-sections -fdata-sections -DGD32F10X_HD \
           -MMD -MP
# Note: no -D__SYSTEM_CLOCK_* needed — the vendored V2.2.1 system_gd32f10x.c
# defaults to __SYSTEM_CLOCK_108M_PLL_HXTAL, which is exactly the original's
# configuration [VENDOR | RECOVERED, see startup.c].
LDFLAGS  = $(CPUFLAGS) -nostdlib -Wl,--gc-sections -Wl,-T,linker.ld \
           -Wl,-Map=build/hguv98.map

SRC  = src/startup.c src/mem.c src/os_hooks.c src/font.c src/rf.c src/rtos.c src/uart.c src/protocol.c src/display.c src/eeprom.c src/ui.c src/main.c
KFILES := ucos_ii.c os_dbg_r.c   # ucos_ii.c includes all os_*.c
KOBJ  = $(addprefix build/k-,$(KFILES:.c=.o))
POBJ  = build/p-os_cpu_c.o build/p-os_cpu_a.o
COBJ  = build/c-cpu_core.o build/c-cpu_a.o

# Vendored GigaDevice GD32F10x firmware library V2.2.1 (SLA-GD0001, see
# vendor/gd32f10x/GIGADEVICE-SLA.txt): era-matching vendor BSP for the 190416
# firmware. Unused modules are pruned by --gc-sections.
VSRC = vendor/gd32f10x/CMSIS/GD/GD32F10x/Source/system_gd32f10x.c \
       $(filter-out %gd32f10x_enet.c,$(wildcard vendor/gd32f10x/SPL/Source/*.c))
VOBJ = $(addprefix build/v-,$(notdir $(VSRC:.c=.o)))
HDR = src/hwregs.h src/rtos.h src/uart.h src/protocol.h src/display.h

OBJ = $(patsubst src/%.c,build/%.o,$(SRC)) build/splash_data.o
TOBJ = build/tables.o

.DEFAULT_GOAL := all

build/tables.o: src/tables.S data/ram_tables_recovered.bin data/uistrings_region.bin
	$(CC) -x assembler-with-cpp -mcpu=cortex-m3 -mthumb -Isrc -c $< -o $@

build/splash_data.o: src/splash_data.S data/splash_128x160_rgb565.bin data/ascii8x16_font.bin data/ascii6x10_font.bin data/big12x20_font.bin
	$(CC) -x assembler-with-cpp -mcpu=cortex-m3 -mthumb -Isrc -c $< -o $@

INC = -Isrc -Iucos2/cfg -Iucos2/ucos2 -Iucos2/port -Iucos2/cpu -Iucos2/lib \
      -Ivendor/gd32f10x/CMSIS -Ivendor/gd32f10x/CMSIS/GD/GD32F10x/Include \
      -Ivendor/gd32f10x/conf -Ivendor/gd32f10x/SPL/Include

all: build/hguv98.bin
	@$(SIZE) build/hguv98.elf

# Host-side conformance tests for the protocol layer (session 17):
# compiles protocol.c natively with a mocked UART sink and executes the
# documented frame rules. Gives the protocol layer an executed-test claim.
CC_HOST ?= cc
test-host: build/test_protocol
	./build/test_protocol

build/test_protocol: tests/test_protocol.c src/protocol.c src/mem.c src/mem.h src/protocol.h
	mkdir -p build
	$(CC_HOST) -std=c11 -O1 -g -Wall -Wextra -Isrc -o $@ tests/test_protocol.c src/mem.c

build:
	mkdir -p build

build/%.o: src/%.c $(HDR) | build
	$(CC) $(CFLAGS) $(INC) -c $< -o $@

build/k-%.o: ucos2/ucos2/%.c | build
	$(CC) $(CFLAGS) $(INC) -c $< -o $@

build/v-system_gd32f10x.o: vendor/gd32f10x/CMSIS/GD/GD32F10x/Source/system_gd32f10x.c | build
	$(CC) $(CFLAGS) $(INC) -c $< -o $@

build/v-%.o: vendor/gd32f10x/SPL/Source/%.c | build
	$(CC) $(CFLAGS) $(INC) -c $< -o $@

build/p-os_cpu_c.o: ucos2/port/os_cpu_c.c | build
	$(CC) $(CFLAGS) $(INC) -c $< -o $@

build/p-os_cpu_a.o: ucos2/port/os_cpu_a.S | build
	$(CC) $(CPUFLAGS) $(INC) -c $< -o $@

build/c-cpu_core.o: ucos2/cpu/cpu_core.c | build
	$(CC) $(CFLAGS) $(INC) -c $< -o $@

build/c-cpu_a.o: ucos2/cpu/cpu_a.s | build
	$(CC) $(CPUFLAGS) $(INC) -c $< -o $@

build/hguv98.elf: $(OBJ) $(TOBJ) $(KOBJ) $(POBJ) $(COBJ) $(VOBJ) linker.ld
	$(CC) $(LDFLAGS) $(OBJ) $(TOBJ) $(KOBJ) $(POBJ) $(COBJ) $(VOBJ) -o $@

build/hguv98.bin: build/hguv98.elf
	$(OBJCOPY) -O binary $< $@

# Disassembly of the reconstruction (for side-by-side comparison with the
# original image; keep out of VCS)
disasm: build/hguv98.elf
	$(TOOLCHAIN_PATH)arm-none-eabi-objdump -d build/hguv98.elf > build/hguv98.asm

clean:
	rm -rf build

# Header dependency tracking (-MMD): without this, ucos2/cfg/os_cfg.h edits
# silently left the kernel objects stale (the session-69 tick fix first ran
# against a 1 ms-tick kernel).
-include $(wildcard build/*.d)

.PHONY: all clean disasm
