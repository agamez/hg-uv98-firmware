/* Application-level config consumed by ucos_ii.h and the ARMv7-M port
 * (Apache-2.0 template). */
#ifndef APP_CFG_H
#define APP_CFG_H

/* NVIC implements 4 interrupt priority bits on the GD32F103. */
#define  CPU_CFG_NVIC_PRIO_BITS                            4u

/* Kernel-aware interrupt priority boundary: interrupts with priority
 * >= 1 are kernel-aware (maskable inside critical sections); priority 0
 * stays non-maskable. The recovered ISRs run at priority 3 (NVIC pre/sub
 * 3 in the recovered USART init), i.e. inside the kernel-aware range. */
#define  CPU_CFG_KA_IPL_BOUNDARY                           1u

#endif /* APP_CFG_H */
