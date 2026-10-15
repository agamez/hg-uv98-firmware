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
           -fno-builtin -ffunction-sections -fdata-sections
LDFLAGS  = $(CPUFLAGS) -nostdlib -Wl,--gc-sections -Wl,-T,linker.ld \
           -Wl,-Map=build/hguv98.map

SRC = src/startup.c src/mem.c src/rtos.c src/uart.c src/protocol.c src/display.c src/main.c
HDR = src/gd32f10x.h src/rtos.h src/uart.h src/protocol.h src/display.h

OBJ = $(patsubst src/%.c,build/%.o,$(SRC))

all: build/hguv98.bin
	@$(SIZE) build/hguv98.elf

build:
	mkdir -p build

build/%.o: src/%.c $(HDR) | build
	$(CC) $(CFLAGS) -c $< -o $@

build/hguv98.elf: $(OBJ) linker.ld
	$(CC) $(LDFLAGS) $(OBJ) -o $@

build/hguv98.bin: build/hguv98.elf
	$(OBJCOPY) -O binary $< $@

# Disassembly of the reconstruction (for side-by-side comparison with the
# original image; keep out of VCS)
disasm: build/hguv98.elf
	$(TOOLCHAIN_PATH)arm-none-eabi-objdump -d build/hguv98.elf > build/hguv98.asm

clean:
	rm -rf build

.PHONY: all clean disasm
