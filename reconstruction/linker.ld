/* HG-UV98 main-MCU reconstruction — GD32F103RET6 memory map.
 *
 * Evidence (confirmed, static; session 11):
 *   - The distributed file is the APPLICATION image based at flash
 *     0x08004000: main() sets VTOR = (0x4000 & 0x1FFFFF80) | 0x08000000
 *     = 0x08004000 [main prologue 0x08029120..0x08029130], every vector
 *     entry resolves inside the file only under that base (75/76 — the
 *     one "outside" is the MSP, a RAM pointer), the Default-handler
 *     pointers land on the six `b .` stubs at file 0x40684, and the KT
 *     file length (0x7C000) is exactly 512 KB minus the 16 KB below
 *     0x08004000. A bootloader occupies 0x08000000..0x08003FFF and is
 *     NOT part of the distributed image.
 *   - Vector table = 76 entries: MSP 0x20006780, Reset (neutered to
 *     2x NOP.W in the distribution), 6+6 Default `b .` stubs, PendSV
 *     0x0802B21A (REAL: CPSID I; MRS r0,PSP — Micrium ARMv7-M port),
 *     SysTick 0x0802BD69 (REAL), then 60 x 10-byte trampolines
 *     dispatching through a RAM vector table at 0x20004AA4.
 *   - RAM: 64 KB at 0x20000000, flash 512 KB.
 */
MEMORY
{
    FLASH (rx)  : ORIGIN = 0x08004000, LENGTH = 496K
    RAM   (rwx) : ORIGIN = 0x20000000, LENGTH = 64K
}

/* The original image places the main stack top at 0x20006780 (vector word 0)
 * with uC/OS-II task stacks above it. We keep the same split: MSP area first,
 * then a task-stack pool occupying the remainder, mirroring the original
 * layout where globals extended up to ~0x200051xx and task stacks lived at
 * 0x20001F74..0x2000449C (session 7-8 task table). */
_estack = ORIGIN(RAM) + 26K;   /* 0x20006800, matches original 0x20006780 top */

ENTRY(Reset_Handler)   /* the vector table's Reset word (0x08004165) — harnesses
                          that start at the ELF entry must land here, not on
                          the first byte after .isr_vector */

SECTIONS
{
    .isr_vector ORIGIN(FLASH) : {
        KEEP(*(.isr_vector))
    } > FLASH

    .text : {
        *(.text*)
        *(.rodata*)
        KEEP(*(.init))
        KEEP(*(.fini))
        . = ALIGN(4);
        _etext = .;
    } > FLASH

    /* The original's string/asset region (file 0x3C000..0x42B97) at its
     * linked address — every pointer in .tables resolves into this blob. */
    .uistrings 0x08040000 : {
        KEEP(*(.uistrings))
    } > FLASH

    /* Original assets (fonts/bitmaps) start right after the 0x130-byte
     * vector table (file 0x130..0xB000 image-labels); we do not reproduce
     * them here. */

    /* Recovered .data tables (session 39/42: the armlink compressed-RW-data
     * streams decompressed) — the original's initialized globals at their
     * original RAM addresses: key-action tables, menu string pointer
     * arrays, CSS tone table, screen attributes. */
    .tables ORIGIN(RAM) : AT(_etext) {
        _stables = .;
        KEEP(*(.tables))
        . = ALIGN(4);
        _etables = .;
    } > RAM
    _sitables = LOADADDR(.tables);

    /* The original's globals span starts right after the tables
     * (0x20001734 — the .bss zero-init target, session 41). */
    .data : AT(_etables) {
        _sdata = .;
        *(.data*)
        . = ALIGN(4);
        _edata = .;
    } > RAM

    .bss (NOLOAD) : {
        _sbss = .;
        *(.bss*)
        *(COMMON)
        . = ALIGN(4);
        _ebss = .;
    } > RAM

    /* Task-stack pool: consumed by rtos.c task_create() from the top. */
    .taskstacks (NOLOAD) : {
        _staskstacks = .;
        . = ORIGIN(RAM) + LENGTH(RAM);
        _etaskstacks = .;
    } > RAM

    _sidata = LOADADDR(.data);
    _sitables = LOADADDR(.tables);
}
