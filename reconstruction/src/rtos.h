/* RTOS adapter — thin wrappers over the vendored uC/OS-II kernel
 * (Apache-2.0, ucos2/). The original firmware ran uC/OS-II (confirmed by
 * the "uC/OS-II Idle/Stat/Tmr" strings in the image); the kernel source is
 * now vendored rather than replaced, so these wrappers preserve the app's
 * recovered call shapes while using the real kernel.
 *
 * Primitive mapping (original API sites in the image):
 *   mbox_create -> OSMboxCreate      [0x08024C78]
 *   mbox_post   -> OSMboxPost        [0x08024DAC, called from ISRs]
 *   mbox_pend   -> OSMboxPend        [0x08024CC6, called from tasks]
 *   task_create -> OSTaskCreateExt   [0x080249A4]
 *   time_dly    -> OSTimeDly         [0x0801FF4C]
 *   time_dly_hmsm -> OSTimeDlyHMSM   [0x0801FFB0]
 *   rtos_start  -> OSStart            [0x08027392 candidate]
 */
#ifndef RTOS_H
#define RTOS_H

#include <stdint.h>

typedef struct mbox mbox_t;
typedef void (*task_fn)(void *arg);

mbox_t  *mbox_create(void);
int      mbox_post(mbox_t *m, uint32_t msg);        /* 0 = ok, 1 = full/err */
int      mbox_pend(mbox_t *m, uint32_t ticks, uint32_t *msg); /* 0=ok,1=timeout */
                                  /* ticks: 0 = wait forever                */

int      task_create(task_fn fn, void *arg, uint32_t stack_size, uint8_t prio);

void     time_dly(uint32_t ticks);
void     time_dly_hmsm(uint8_t h, uint8_t m, uint8_t s, uint8_t ms);
void     rtos_start(void);
void     board_setup_tick(uint32_t hclk_hz);        /* OS_CPU_SysTickInitFreq */

#endif /* RTOS_H */
