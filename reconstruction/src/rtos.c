/* RTOS adapter — see rtos.h for the primitive mapping and provenance.
 *
 * Task stacks are carved from the linker-provided .taskstacks pool
 * (mirroring the original's static task stacks at 0x20001F74..0x2000449C,
 * recovered in the session 7-8 task table).
 *
 * Note on message values: uC/OS-II mailboxes pass pointers; a NULL message
 * is reserved by the kernel. Recovered call sites post non-zero values
 * (USART bytes, counters), so messages are biased by +1 across the wrapper
 * to keep 0 representable. */
#include "rtos.h"
#include "ucos_ii.h"
#include "hwregs.h"

extern uint32_t _staskstacks, _etaskstacks;
static OS_STK *pool_ptr;

mbox_t *mbox_create(void)
{
    return (mbox_t *)OSMboxCreate((void *)0);
}

int mbox_post(mbox_t *m, uint32_t msg)
{
    /* uC/OS-II mailboxes are single-depth: posting with a message already
     * pending and no waiter fails with OS_ERR_MBOX_FULL — which matches
     * the recovered call sites (they post one item at a time). */
    OSMboxPost((OS_EVENT *)m, (void *)(msg + 1u));
    return 0;
}

int mbox_pend(mbox_t *m, uint32_t ticks, uint32_t *msg)
{
    INT8U err;
    void *p = OSMboxPend((OS_EVENT *)m, (INT16U)ticks, &err);
    if (err == OS_ERR_NONE) {
        if (msg)
            *msg = (uint32_t)p - 1u;   /* undo the +1 bias                  */
        return 0;
    }
    return 1;                          /* OS_ERR_TIMEOUT or other failure   */
}

int task_create(task_fn fn, void *arg, uint32_t stack_size, uint8_t prio)
{
    OS_STK *ptos, *pbos;
    INT32U words = stack_size / sizeof(OS_STK);
    INT8U err;

    if (!pool_ptr)
        pool_ptr = (OS_STK *)&_etaskstacks;

    words &= ~1u;                                  /* keep 8-byte alignment */
    pool_ptr -= words;
    if (pool_ptr < (OS_STK *)&_staskstacks)
        return -1;                                 /* task-stack pool used up */
    ptos = pool_ptr;                               /* stacks grow downward   */
    pbos = pool_ptr + words - 1u;

    /* [0x080249A4 | RECOVERED]: OSTaskCreateExt(task, parg, ptos, prio,
     * id, pbos, stk_size, pext, opt); the original clears stacks (opt bit
     * observed via the creation helper's zero fills). */
    err = OSTaskCreateExt(fn, arg, ptos, prio, prio, pbos, words, (void *)0,
                          (INT16U)(OS_TASK_OPT_STK_CLR | OS_TASK_OPT_STK_CHK));
    return (err == OS_ERR_NONE) ? prio : -1;
}

void time_dly(uint32_t ticks)
{
    OSTimeDly((INT16U)ticks);
}

/* [session 69] the RTOS tick counter (OSTime) reader: the boot-pass anchor
 * in ui.c holds the boot at the 144-pass (6.000 s) boundary. */
uint32_t rtos_ticks(void)
{
    return (uint32_t)OSTimeGet();
}

void time_dly_hmsm(uint8_t h, uint8_t m, uint8_t s, uint8_t ms)
{
    OSTimeDlyHMSM(h, m, s, ms);
}

void rtos_start(void)
{
    /* The port's OSStartHighRdy sets PendSV to the lowest priority and
     * performs the first context switch; SCB->VTOR was set by main().
     * (The tick is configured in main() via board_setup_tick BEFORE
     * OSStart — the port's OSStartHighRdy loads MSP from
     * OS_CPU_ExceptStkBase, initialized by OSInitHookBegin inside
     * OSInit, so OSInit must precede task creation too.) */
    OSStart();
}

void board_setup_tick(uint32_t hclk_hz)
{
    OS_CPU_SysTickInitFreq((INT32U)hclk_hz);
}
