/* HG-UV98 main-MCU reconstruction — GD32F103RET6 memory map.
 *
 * Evidence (confirmed, static):
 *   - Vector table at 0x08000000 with initial SP 0x20006780 (image word 0).
 *   - 76 vector entries = 16 ARMv7-M system + 60 device IRQs, matching the
 *     STM32F103xE / GD32F103 high-density layout (research log, session 1).
 *   - RAM: 64 KB at 0x20000000 (initial SP inside it), flash 512 KB.
 */
MEMORY
{
    FLASH (rx)  : ORIGIN = 0x08000000, LENGTH = 512K
    RAM   (rwx) : ORIGIN = 0x20000000, LENGTH = 64K
}

/* The original image places the main stack top at 0x20006780 (vector word 0)
 * with uC/OS-II task stacks above it. We keep the same split: MSP area first,
 * then a task-stack pool occupying the remainder, mirroring the original
 * layout where globals extended up to ~0x200051xx and task stacks lived at
 * 0x20001F74..0x2000449C (session 7-8 task table). */
_estack = ORIGIN(RAM) + 26K;   /* 0x20006800, matches original 0x20006780 top */

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

    /* Original assets (fonts/bitmaps) lived between the vector table and the
     * code region (0x08000130-0x0800B000); we do not reproduce them here. */

    .data : AT (_etext) {
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
}
