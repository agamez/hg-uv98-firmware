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

/* Original vector values (190416 image, word 0..15):
 *   [0] 0x20006780 (MSP)   [1] 0x08044691 (Reset, OUTSIDE file)
 *   [2] 0x08044685 NMI     [3] 0x08044687 HardFault
 *   [4] 0x0804468D         [5] 0x08044689   [6] 0x0804468B
 *   [7..13] 0x0804468F (Default)
 *   [14] 0x0802B21B PendSV [15] 0x0802BD69 SysTick   (both -> stripped BEEF) */
__attribute__((section(".isr_vector"), used))
void *const vector_table[] = {
    &_estack,                       /* [0]  initial MSP                    */
    Reset_Handler,                  /* [1]  Reset (SCAFFOLD: original lost)*/
    Default_Handler,                /* [2]  NMI         orig 0x08044684    */
    Default_Handler,                /* [3]  HardFault   orig 0x08044686    */
    Default_Handler,                /* [4]  MemManage   orig 0x0804468C    */
    Default_Handler,                /* [5]  BusFault    orig 0x08044688    */
    Default_Handler,                /* [6]  UsageFault  orig 0x0804468A    */
    Default_Handler, Default_Handler, Default_Handler, Default_Handler,
    Default_Handler,                /* [11] SVCall                         */
    Default_Handler,                /* [12] DebugMon                       */
    0,                              /* [13] reserved                       */
    OS_CPU_PendSVHandler,           /* [14] PendSV   orig 0x0802B21A       */
    OS_CPU_SysTickHandler,          /* [15] SysTick  orig 0x0802BD68       */

    /* [16..75] device IRQs — F103xE high-density layout, 60 entries.
     * Original trampolines are BEEF placeholders; only USART1 (IRQ 37) and
     * USART2 (IRQ 38) are wired to recovered handlers. */
    Default_Handler,                /*  0 WWDG        */
    Default_Handler,                /*  1 PVD         */
    Default_Handler,                /*  2 TAMPER      */
    Default_Handler,                /*  3 RTC         */
    Default_Handler,                /*  4 FLASH       */
    Default_Handler,                /*  5 RCC         */
    Default_Handler,                /*  6 EXTI0       */
    Default_Handler,                /*  7 EXTI1       */
    Default_Handler,                /*  8 EXTI2       */
    Default_Handler,                /*  9 EXTI3       */
    Default_Handler,                /* 10 EXTI4       */
    Default_Handler,                /* 11 DMA1_Ch1    */
    Default_Handler,                /* 12 DMA1_Ch2    */
    Default_Handler,                /* 13 DMA1_Ch3    */
    Default_Handler,                /* 14 DMA1_Ch4    */
    Default_Handler,                /* 15 DMA1_Ch5    */
    Default_Handler,                /* 16 DMA1_Ch6    */
    Default_Handler,                /* 17 DMA1_Ch7    */
    Default_Handler,                /* 18 ADC1_2      */
    Default_Handler,                /* 19 USB_HP      */
    Default_Handler,                /* 20 USB_LP      */
    Default_Handler,                /* 21 CAN_RX1     */
    Default_Handler,                /* 22 CAN_SCE     */
    Default_Handler,                /* 23 EXTI9_5     */
    Default_Handler,                /* 24 TIM1_BRK    */
    Default_Handler,                /* 25 TIM1_UP     */
    Default_Handler,                /* 26 TIM1_TRG_COM*/
    Default_Handler,                /* 27 TIM1_CC     */
    Default_Handler,                /* 28 TIM2        */
    Default_Handler,                /* 29 TIM3        */
    Default_Handler,                /* 30 TIM4        */
    Default_Handler,                /* 31 I2C1_EV     */
    Default_Handler,                /* 32 I2C1_ER     */
    Default_Handler,                /* 33 I2C2_EV     */
    Default_Handler,                /* 34 I2C2_ER     */
    Default_Handler,                /* 35 SPI1        */
    Default_Handler,                /* 36 SPI2        */
    USART1_IRQHandler,              /* 37 USART1      */
    USART2_IRQHandler,              /* 38 USART2      */
    Default_Handler,                /* 39 EXTI15_10   */
    Default_Handler,                /* 40 RTC_Alarm   */
    Default_Handler,                /* 41 USBWake     */
    Default_Handler,                /* 42 TIM8_BRK    */
    Default_Handler,                /* 43 TIM8_UP     */
    Default_Handler,                /* 44 TIM8_TRG_COM*/
    Default_Handler,                /* 45 TIM8_CC     */
    Default_Handler,                /* 46 ADC3        */
    Default_Handler,                /* 47 FSMC        */
    Default_Handler,                /* 48 SDIO        */
    Default_Handler,                /* 49 TIM5        */
    Default_Handler,                /* 50 SPI3        */
    Default_Handler,                /* 51 UART4       */
    Default_Handler,                /* 52 UART5       */
    Default_Handler,                /* 53 TIM6_DAC    */
    Default_Handler,                /* 54 TIM7        */
    Default_Handler,                /* 55 DMA2_Ch1    */
    Default_Handler,                /* 56 DMA2_Ch2    */
    Default_Handler,                /* 57 DMA2_Ch3    */
    Default_Handler,                /* 58 DMA2_Ch4    */
    Default_Handler,                /* 59 DMA2_Ch5    */
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
