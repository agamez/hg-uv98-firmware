/* uC/CPU configuration for the GD32F103 (Cortex-M3) reconstruction.
 * Base: Cfg/Template/cpu_cfg.h (Apache-2.0). */
#ifndef  CPU_CFG_H
#define  CPU_CFG_H

/* ------------------------------------------------------------------------ */
/*                                 CPU NAME                                  */
/* ------------------------------------------------------------------------ */

#define  CPU_CFG_NAME_EN                       DEF_ENABLED
#define  CPU_CFG_NAME_SIZE                              16u

/* ------------------------------------------------------------------------ */
/*                              TIMESTAMP SOURCE                             */
/* ------------------------------------------------------------------------ */

/* DWT cycle counter is not configured in this reconstruction; timestamp
 * features stay disabled (the kernel does not require them). When TS is
 * disabled the timer size must still be a valid word size (8 bytes).
 * CPU_CFG_TS_TMR_EN is defaulted by cpu_core.h; TS_32/TS_64 must be set
 * here. */
#define  CPU_CFG_TS_32_EN                      DEF_DISABLED
#define  CPU_CFG_TS_64_EN                      DEF_DISABLED
#define  CPU_CFG_TS_TMR_SIZE                             8u

/* ------------------------------------------------------------------------ */
/*                        CRITICAL SECTION / INTERRUPTS                      */
/* ------------------------------------------------------------------------ */

/* Critical method (3 = save/restore PRIMASK) is defaulted by cpu.h. */

/* NVIC implements 4 interrupt priority bits on the GD32F103. */
#define  CPU_CFG_NVIC_PRIO_BITS                           4u

/* Kernel-aware boundary: priorities >= 1 are maskable inside critical
 * sections; priority 0 stays non-maskable. The recovered ISRs run at
 * priority 3 (NVIC pre/sub 3 in the recovered USART init). */
#define  CPU_CFG_KA_IPL_BOUNDARY                           1u

/* ------------------------------------------------------------------------ */
/*                             MISC DATA TYPES                               */
/* ------------------------------------------------------------------------ */

/* CPU_CFG_ADDR_SIZE / CPU_CFG_DATA_SIZE / CPU_CFG_DATA_SIZE_MAX /
 * CPU_CFG_ENDIAN_TYPE / CPU_CFG_STK_GROWTH / CPU_CFG_CRITICAL_METHOD are
 * defaulted by cpu.h for the ARMv7-M GNU port — do not duplicate them here. */

/* ARMv7-M provides CLZ/RBIT instructions used by the lead/trail-zero
 * helpers (cpu_a.s); cpu.h defaults CPU_CFG_LEAD/TRAIL_ZEROS_ASM_PRESENT
 * for this port — do not duplicate them here. */

/* Interrupt-disable measurement is not reconstructed; the boundary value
 * must still be in the template's legal range. */
#define  CPU_CFG_INT_DIS_MEAS_EN                DEF_DISABLED
#define  CPU_CFG_INT_DIS_MEAS_OVRHD_NBR                    1u

#endif /* CPU_CFG_H */
