/* Startup code and vector table.
 *
 * SCAFFOLD: the original startup/reset code is stripped from the
 * distributed images (research log O1: reset handler 0x08044690 and the
 * fault-handler cluster 0x0804468x lie beyond the 190416 truncation; IRQ
 * trampolines, PendSV and SysTick entries are BEEF placeholders). The
 * vector-table STRUCTURE is confirmed (76 entries, initial SP 0x20006780,
 * PendSV/SysTick present), and the original handler addresses are kept in
 * comments for traceability.
 */
#include "hwregs.h"
#include "gd32f10x.h"      /* [VENDOR] SystemInit from the GigaDevice BSP */
#include "uart.h"

extern uint32_t _estack, _sdata, _edata, _sidata, _sbss, _ebss;

int  main(void);
void Reset_Handler(void);
void Default_Handler(void);

/* Implemented handlers (see respective modules). */
void OS_CPU_PendSVHandler(void);    /* uC/OS-II port — context switch        */
void OS_CPU_SysTickHandler(void);   /* uC/OS-II port — 1 ms tick             */
void USART1_IRQHandler(void);       /* uart.c    — RXNE, ring + sink         */
void USART2_IRQHandler(void);       /* uart.c    — RXNE, frame assembler    */

/* ------------------------------------------------------------------ *
 * IRQ dispatch — [RECOVERED] architecture (session 11):
 *
 *   flash vector [16+e] -> 10-byte trampoline (image file 0x26F4E+e*10):
 *       push {r7, lr}; movs r0, #e; bl dispatcher; pop {r0, pc}
 *   dispatcher [0x080271A6]:
 *       cpu_sr = OS_CPU_SR_Save(); OSIntNesting++;
 *       if (e < 60) { h = ram_vectors[e]; if (h) h(); }   [ints masked]
 *       OS_CPU_SR_Restore(cpu_sr); OSIntExit();
 *   irq_install [0x08026F30]:
 *       cpu_sr = Save(); if (irq < 60) ram_vectors[irq] = handler; Restore();
 *
 * The RAM table lives at 0x20004AA4 in the original (between the task
 * stacks and the flag bank 0x20005094); handlers are installed at init
 * time: irq_install(38, &USART2_IRQHandler) [USART2 init 0x0801F8A2],
 * irq_install(37, &USART1_IRQHandler) [USART1 init 0x080293CE].
 * The tail call 0x0802731A after the restore is OSIntExit.
 * ------------------------------------------------------------------ */
#include "ucos_ii.h"

static uint32_t ram_vectors[60];        /* orig 0x20004AA4                  */

void irq_install(uint32_t irq, void (*handler)(void))
{
    OS_CPU_SR cpu_sr;
    cpu_sr = OS_CPU_SR_Save((CPU_CFG_KA_IPL_BOUNDARY << (8u - CPU_CFG_NVIC_PRIO_BITS)));
    if (irq < 60u)
        ram_vectors[irq] = (uint32_t)handler;
    OS_CPU_SR_Restore(cpu_sr);
}

extern uint8_t OSIntNesting;            /* uC/OS-II kernel global           */

static void irq_dispatcher(uint32_t irq)
{
    OS_CPU_SR cpu_sr;
    cpu_sr = OS_CPU_SR_Save((CPU_CFG_KA_IPL_BOUNDARY << (8u - CPU_CFG_NVIC_PRIO_BITS)));
    OSIntNesting++;                     /* original inlines OSIntEnter      */
    if (irq < 60u) {
        void (*h)(void) = (void (*)(void))ram_vectors[irq];
        if (h)
            h();
    }
    OS_CPU_SR_Restore(cpu_sr);
    OSIntExit();                        /* orig tail call 0x0802731A        */
}

#define IRQ_STUB(n) \
    static void irq_stub_##n(void) { irq_dispatcher(n); }
#define IRQ_STUB2(n) IRQ_STUB(n##0) IRQ_STUB(n##1) IRQ_STUB(n##2) \
                     IRQ_STUB(n##3) IRQ_STUB(n##4) IRQ_STUB(n##5) \
                     IRQ_STUB(n##6) IRQ_STUB(n##7) IRQ_STUB(n##8) \
                     IRQ_STUB(n##9)
IRQ_STUB2() IRQ_STUB2(1) IRQ_STUB2(2) IRQ_STUB2(3) IRQ_STUB2(4) IRQ_STUB2(5)
#undef IRQ_STUB
#undef IRQ_STUB2

static void (*const irq_trampolines[60])(void) = {
    irq_stub_0,  irq_stub_1,  irq_stub_2,  irq_stub_3,  irq_stub_4,
    irq_stub_5,  irq_stub_6,  irq_stub_7,  irq_stub_8,  irq_stub_9,
    irq_stub_10, irq_stub_11, irq_stub_12, irq_stub_13, irq_stub_14,
    irq_stub_15, irq_stub_16, irq_stub_17, irq_stub_18, irq_stub_19,
    irq_stub_20, irq_stub_21, irq_stub_22, irq_stub_23, irq_stub_24,
    irq_stub_25, irq_stub_26, irq_stub_27, irq_stub_28, irq_stub_29,
    irq_stub_30, irq_stub_31, irq_stub_32, irq_stub_33, irq_stub_34,
    irq_stub_35, irq_stub_36, irq_stub_37, irq_stub_38, irq_stub_39,
    irq_stub_40, irq_stub_41, irq_stub_42, irq_stub_43, irq_stub_44,
    irq_stub_45, irq_stub_46, irq_stub_47, irq_stub_48, irq_stub_49,
    irq_stub_50, irq_stub_51, irq_stub_52, irq_stub_53, irq_stub_54,
    irq_stub_55, irq_stub_56, irq_stub_57, irq_stub_58, irq_stub_59,
};

/* Original vector values (190416 file; entries are FLASH addresses,
 * file base 0x08004000 — session 11 correction):
 *   [0] 0x20006780 (MSP)     [1] 0x08044691 Reset -> 2x NOP.W (neutered
 *       in the distribution; the real startup code was removed)
 *   [2]..[7]  0x08044685..0x0804468D: six `b .` stubs (file 0x40684)
 *   [8]..[13] 0x0804468F (shared Default)
 *   [14] 0x0802B21B PendSV  -> file 0x2721A, REAL (CPSID I; MRS r0,PSP
 *       — Micrium ARMv7-M port; earlier "stripped BEEF" verdict was a
 *       wrong-base artifact)
 *   [15] 0x0802BD69 SysTick -> file 0x27D68, REAL (SR save, OSIntNesting
 *       ++, OSTimeTick 0x0802740E, OSIntExit 0x0802731A)
 *   [16+e] 0x0802AF4F+e*10 trampoline stair -> dispatcher 0x080271A6 */
__attribute__((section(".isr_vector"), used))
void *const vector_table[] = {
    &_estack,                       /* [0]  initial MSP                    */
    Reset_Handler,                  /* [1]  Reset (SCAFFOLD: distribution  */
                                    /*      has 2x NOP.W here)             */
    Default_Handler,                /* [2]  NMI         orig 0x08044684    */
    Default_Handler,                /* [3]  HardFault   orig 0x08044686    */
    Default_Handler,                /* [4]  MemManage   orig 0x0804468C    */
    Default_Handler,                /* [5]  BusFault    orig 0x08044688    */
    Default_Handler,                /* [6]  UsageFault  orig 0x0804468A    */
    Default_Handler,                /* [7]  SVCall etc  orig 0x0804468E    */
    Default_Handler, Default_Handler, Default_Handler,
    Default_Handler, Default_Handler, Default_Handler,
    OS_CPU_PendSVHandler,           /* [14] PendSV   orig 0x0802B21A REAL  */
    OS_CPU_SysTickHandler,          /* [15] SysTick  orig 0x0802BD69 REAL  */
    irq_trampolines[0], irq_trampolines[1], irq_trampolines[2],
    irq_trampolines[3], irq_trampolines[4], irq_trampolines[5],
    irq_trampolines[6], irq_trampolines[7], irq_trampolines[8],
    irq_trampolines[9], irq_trampolines[10], irq_trampolines[11],
    irq_trampolines[12], irq_trampolines[13], irq_trampolines[14],
    irq_trampolines[15], irq_trampolines[16], irq_trampolines[17],
    irq_trampolines[18], irq_trampolines[19], irq_trampolines[20],
    irq_trampolines[21], irq_trampolines[22], irq_trampolines[23],
    irq_trampolines[24], irq_trampolines[25], irq_trampolines[26],
    irq_trampolines[27], irq_trampolines[28], irq_trampolines[29],
    irq_trampolines[30], irq_trampolines[31], irq_trampolines[32],
    irq_trampolines[33], irq_trampolines[34], irq_trampolines[35],
    irq_trampolines[36], irq_trampolines[37], irq_trampolines[38],
    irq_trampolines[39], irq_trampolines[40], irq_trampolines[41],
    irq_trampolines[42], irq_trampolines[43], irq_trampolines[44],
    irq_trampolines[45], irq_trampolines[46], irq_trampolines[47],
    irq_trampolines[48], irq_trampolines[49], irq_trampolines[50],
    irq_trampolines[51], irq_trampolines[52], irq_trampolines[53],
    irq_trampolines[54], irq_trampolines[55], irq_trampolines[56],
    irq_trampolines[57], irq_trampolines[58], irq_trampolines[59],
};

void Reset_Handler(void)
{
    uint32_t *src, *dst;

    src = &_sidata;
    for (dst = &_sdata; dst < &_edata; )
        *dst++ = *src++;
    for (dst = &_sbss; dst < &_ebss; )
        *dst++ = 0;

    /* [VENDOR | RECOVERED]: the original's startup is stripped, but its
     * SystemInit code is present in the image — the compiled vendor 108 MHz
     * HXTAL constants (RCU base 0x40021000, HXTAL_VALUE 0x7A1200, prediv
     * 0x3D0900, mask 0xFEF6FFFF) cluster at 0x08026B70..0x08026BA8, and the
     * SystemInit prologue pattern (ldr r3,[r2]; mov.w r4,#0x9F0000) appears
     * at 0x08029008/0x08029018, immediately before main() at 0x08029120
     * (session 9, tools/vendor_fingerprint.py). The GigaDevice vendor
     * startup convention calls SystemInit() here; the original binary
     * corroborates that the vendor file was used. */
    SystemInit();

    main();
    for (;;)
        ;
}

void Default_Handler(void)
{
    for (;;)
        ;
}
