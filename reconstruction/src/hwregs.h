/* Minimal GD32F103 register sugar used by the reconstruction.
 * Register layout is public in the GigaDevice reference manuals and the
 * vendored GigaDevice BSP; addresses cross-checked against constants found
 * in the original image (research log: peripheral literal census).
 *
 * REG32/REG16/REG8 and the peripheral bases come from the vendored
 * GigaDevice header (gd32f10x.h); this header only adds the peripheral
 * register sugar used by our recovered code. NOTE: this file uses ST-style
 * peripheral numbering (USART1 = APB2 0x40013800, USART2 = APB1
 * 0x40004400) matching the original image's evidence; the vendor header
 * numbers them USART0/USART1 respectively. */
#ifndef HWREGS_H
#define HWREGS_H

#include "gd32f10x.h"
#include <stdint.h>

/* --- RCC (0x40021000) --- */
#define RCC_BASE        0x40021000U
#define RCC_APB2ENR     REG32(RCC_BASE + 0x18)
#define RCC_APB1ENR     REG32(RCC_BASE + 0x1C)
#define RCC_APB2ENR_AFIOEN  (1U << 0)
#define RCC_APB2ENR_GPIOAEN (1U << 2)
#define RCC_APB2ENR_GPIOBEN (1U << 3)
#define RCC_APB2ENR_GPIOCEN (1U << 4)
#define RCC_APB2ENR_USART1EN (1U << 14)
#define RCC_APB1ENR_USART2EN (1U << 17)

/* --- GPIO (0x40010800: GPIOA, 0x40010C00: B, 0x40011000: C) --- */
#define GPIOA_BASE      0x40010800U
#define GPIOB_BASE      0x40010C00U
#define GPIOC_BASE      0x40011000U
#define GPIOD_BASE      0x40011400U
#define GPIO_CRL(g)     REG32((g) + 0x00)
#define GPIO_CRH(g)     REG32((g) + 0x04)
#define GPIO_IDR(g)     REG32((g) + 0x08)
#define GPIO_ODR(g)     REG32((g) + 0x0C)
#define GPIO_BSRR(g)    REG32((g) + 0x10)
#define GPIO_BRR(g)     REG32((g) + 0x14)

/* --- USART --- */
#define USART1_BASE     0x40013800U
#define USART2_BASE     0x40004400U
#define USART_SR(u)     REG16((u) + 0x00)
#define USART_DR(u)     REG16((u) + 0x04)
#define USART_BRR(u)    REG16((u) + 0x08)
#define USART_CR1(u)    REG16((u) + 0x0C)
#define USART_CR2(u)    REG16((u) + 0x10)
#define USART_CR3(u)    REG16((u) + 0x14)
#define USART_SR_RXNE   (1U << 5)
#define USART_SR_TC     (1U << 6)
#define USART_SR_TXE    (1U << 7)
#define USART_CR1_UE    (1U << 13)
#define USART_CR1_TE    (1U << 3)
#define USART_CR1_RE    (1U << 2)
#define USART_CR1_RXNEIE (1U << 5)

/* --- ADC (session 56, Round N: the keyscan ladders + the battery quantizer
 * share ADC2; the vendor's driver polls SR.EOC (bit2, tagged 0x2 by the
 * harness) then reads DR (0x0802C37A = u16 *(handle + 0x4C)). --- */
#define ADC2_BASE       0x40012800U
#define ADC2_SR         REG32(ADC2_BASE + 0x00)
#define ADC2_SQR3       REG32(ADC2_BASE + 0x34)
#define ADC2_DR         REG32(ADC2_BASE + 0x4CU)
#define ADC_SR_EOC      (1U << 1)   /* SR bit1 (the original polls 0x2;
                                         * the harness tags ADC2_SR = 0x2) */

/* --- NVIC / SysTick / SCB (Cortex-M3 core) --- */
#define NVIC_ISER0      REG32(0xE000E100U)
#define NVIC_ISER1      REG32(0xE000E104U)
#define NVIC_ICER0      REG32(0xE000E180U)
#define NVIC_IP_BASE    0xE000E400U
#define SCB_VTOR        REG32(0xE000ED08U)
#define SCB_ICSR        REG32(0xE000ED04U)
#define SCB_ICSR_PENDSTSET (1U << 26)
#define SCB_ICSR_PENDSVSET (1U << 28)
#define SYST_CSR        REG32(0xE000E010U)
#define SYST_RVR        REG32(0xE000E014U)
#define SYST_CVR        REG32(0xE000E018U)

#define IRQ_USART1      37
#define IRQ_USART2      38

#endif /* HWREGS_H */
