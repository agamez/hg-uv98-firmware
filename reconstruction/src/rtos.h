/* Minimal priority scheduler exposing the RTOS primitives observed in the
 * original image.
 *
 * REPLACEMENT component. The original firmware uses Micrium uC/OS-II
 * (task table recovered in session 7-8; kernel API sites: OSMboxCreate
 * 0x08024C78, OSMboxPost 0x08024DAC, OSMboxPend 0x08024CC6,
 * OSTaskCreateExt 0x080249A4, OSTimeDly 0x0801FF4C,
 * OSTimeDlyHMSM 0x0801FFB0, OSStart 0x08027392-candidate). The original
 * kernel code itself is stripped from the distributed images, so this
 * module re-implements the observed primitive set from scratch with the
 * same semantics the reconstructed callers rely on:
 *   - fixed-priority preemptive scheduling, 1 ms tick (usage evidence:
 *     OSTimeDly(500) used for a ~500 ms poll task)
 *   - mailboxes: single-depth post semantics per waiter, FIFO of pend-ers
 *
 * Task table recovered from the original (prio / entry / stack):
 *   3  0x08029178  main/UI task          1 KB
 *   10 0x08017664  1 ms periodic        1 KB
 *   11 0x08014DEA  UI event dispatcher  1 KB
 *   13 0x08017704  RSSI poll (500 ms)   1 KB
 *   29 0x08028D74  mbox service ring    512 B
 *   30 0x08027938  timeout/counter      512 B
 *   31 0x08027918  statistics-style     512 B
 */
#ifndef RTOS_H
#define RTOS_H

#include <stdint.h>

typedef struct mbox mbox_t;

typedef void (*task_fn)(void *arg);

mbox_t  *mbox_create(void);
int      mbox_post(mbox_t *m, uint32_t msg);        /* 0 = ok, 1 = full */
int      mbox_pend(mbox_t *m, uint32_t ticks, uint32_t *msg); /* 0=ok,1=timeout */
                                  /* ticks: 0 = wait forever */

int      task_create(task_fn fn, void *arg, uint32_t stack_size, uint8_t prio);

void     time_dly(uint32_t ticks);                  /* OSTimeDly equivalent   */
void     time_dly_hmsm(uint8_t h, uint8_t m, uint8_t s, uint8_t ms); /* 1 ms tick */
void     rtos_start(void);                          /* OSStart equivalent     */
uint32_t rtos_tick_count(void);
void     board_setup_tick(uint32_t hclk_hz);        /* SysTick @1 ms          */

#endif /* RTOS_H */
