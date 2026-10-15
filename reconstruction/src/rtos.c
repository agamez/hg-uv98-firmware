/* Minimal fixed-priority scheduler — see rtos.h for provenance.
 *
 * Context switch: Cortex-M3 PSP/MSP scheme. Tasks run on PSP; PendSV saves
 * r4-r11 into the task's stack (task_sp[]) and swaps. r0-r3/r12/PC/xPSR are
 * restored by the exception return itself. The original's PendSV handler is
 * stripped from the distributed image (research log O1); this is a
 * replacement implementation with the observed API semantics. */
#include "rtos.h"
#include "gd32f10x.h"

#define MAX_TASKS   8
#define MAX_MBOXES  8
#define MBOX_MAX_CAP 8

typedef enum { TSTATE_READY = 0, TSTATE_SLEEPING, TSTATE_PEND, TSTATE_UNUSED } tstate_t;

typedef struct {
    uint8_t   prio;
    tstate_t  state;
    uint8_t   deadline;      /* PEND with wake_tick valid */
    uint32_t  wake_tick;
    mbox_t   *pend_mbox;
    uint32_t *pend_msg;
    uint32_t  pend_result;
} task_t;

struct mbox {
    uint8_t   used;
    uint32_t *buf;
    uint8_t   capacity, head, count;
};

static task_t tasks[MAX_TASKS];
uint32_t task_sp[MAX_TASKS] __attribute__((used));   /* referenced by PendSV asm */
static int ntasks;
static volatile int current = -1;
static volatile uint32_t tick_count;
static task_t * const T = tasks;

static uint32_t mbox_mem[MAX_MBOXES][MBOX_MAX_CAP];
static mbox_t mboxes[MAX_MBOXES];

/* ---- context switch ------------------------------------------------------ */

__attribute__((naked)) void PendSV_Handler(void)
{
    __asm volatile(
        "mrs    r0, psp                 \n"
        "stmdb  r0!, {r4-r11}           \n"
        "ldr    r1, =current            \n"
        "ldr    r2, [r1]                \n"
        "lsls   r2, r2, #2              \n"
        "ldr    r3, =task_sp            \n"
        "adds   r3, r3, r2              \n"
        "str    r0, [r3]                \n"
        "bl     rtos_pick_next          \n"
        "ldr    r1, =current            \n"
        "ldr    r2, [r1]                \n"
        "lsls   r2, r2, #2              \n"
        "ldr    r3, =task_sp            \n"
        "adds   r3, r3, r2              \n"
        "ldr    r0, [r3]                \n"
        "ldmia  r0!, {r4-r11}           \n"
        "msr    psp, r0                 \n"
        "bx     lr                      \n");
}

/* kernel-context helper: promote due tasks, pick highest-prio READY. */
int rtos_pick_next(void)
{
    int i, best = -1;

    for (i = 0; i < ntasks; i++) {
        task_t *t = &T[i];
        if (t->state == TSTATE_SLEEPING &&
            (int32_t)(tick_count - t->wake_tick) >= 0)
            t->state = TSTATE_READY;
        if (t->state == TSTATE_PEND && t->deadline &&
            (int32_t)(tick_count - t->wake_tick) >= 0) {
            t->state = TSTATE_READY;
            t->pend_result = 1;              /* timeout */
            t->pend_mbox = 0;
        }
        if (t->state == TSTATE_READY &&
            (best < 0 || t->prio < T[best].prio))
            best = i;
    }
    if (best >= 0)
        current = best;
    return current;
}

static void schedule(void)
{
    SCB_ICSR = SCB_ICSR_PENDSVSET;
    __asm volatile("isb" ::: "memory");
}

void rtos_tick(void)                 /* SysTick, 1 ms */
{
    tick_count++;
    schedule();
}

uint32_t rtos_tick_count(void) { return tick_count; }

/* ---- tasks ---------------------------------------------------------------- */

/* Initial frame: r4-r11 below, then r0-r3,r12,LR,PC,xPSR (exception return
 * pops the top 8). PSP starts at the r0 slot. */
static uint32_t *stack_init(uint32_t *top, task_fn fn, void *arg)
{
    top -= 16;
    for (int i = 0; i < 8; i++)
        top[i] = 0xDEAD0000u + i;        /* r4-r11 */
    top[8]  = (uint32_t)arg;             /* r0     */
    top[9]  = 0; top[10] = 0; top[11] = 0;
    top[12] = 0;                         /* r12    */
    top[13] = 0;                         /* LR     */
    top[14] = (uint32_t)fn;              /* PC     */
    top[15] = 0x01000000u;               /* xPSR Thumb */
    return top;
}

extern uint32_t _staskstacks, _etaskstacks;   /* linker-provided pool */
static uint32_t *pool_ptr;

int task_create(task_fn fn, void *arg, uint32_t stack_size, uint8_t prio)
{
    task_t *t;

    if (ntasks >= MAX_TASKS)
        return -1;
    if (!pool_ptr)
        pool_ptr = &_etaskstacks;

    stack_size = (stack_size + 7u) & ~7u;
    pool_ptr = (uint32_t *)((uint32_t)pool_ptr - stack_size);
    if (pool_ptr < &_staskstacks)
        return -1;                        /* pool exhausted */

    t = &T[ntasks];
    t->prio = prio;
    t->state = TSTATE_READY;
    t->deadline = 0;
    t->pend_mbox = 0;
    t->pend_msg = 0;
    t->pend_result = 0;
    task_sp[ntasks] = (uint32_t)stack_init(pool_ptr, fn, arg);
    return ntasks++;
}

void time_dly(uint32_t ticks)
{
    if (current < 0 || ticks == 0)
        return;
    T[current].wake_tick = tick_count + ticks;
    T[current].deadline = 1;
    T[current].state = TSTATE_SLEEPING;
    schedule();
}

void time_dly_hmsm(uint8_t h, uint8_t m, uint8_t s, uint8_t ms)
{
    time_dly((uint32_t)h * 3600000u + (uint32_t)m * 60000u +
             (uint32_t)s * 1000u + ms);
}

/* ---- mailboxes ------------------------------------------------------------ */

mbox_t *mbox_create(void)
{
    for (int i = 0; i < MAX_MBOXES; i++) {
        if (!mboxes[i].used) {
            mboxes[i].used = 1;
            mboxes[i].buf = mbox_mem[i];
            mboxes[i].capacity = MBOX_MAX_CAP;
            mboxes[i].head = 0;
            mboxes[i].count = 0;
            return &mboxes[i];
        }
    }
    return 0;
}

int mbox_post(mbox_t *m, uint32_t msg)
{
    int woke = 0;

    if (!m)
        return 1;

    __asm volatile("cpsid i" ::: "memory");
    for (int i = 0; i < ntasks; i++) {
        if (T[i].state == TSTATE_PEND && T[i].pend_mbox == m) {
            if (T[i].pend_msg)
                *T[i].pend_msg = msg;
            T[i].pend_result = 0;
            T[i].state = TSTATE_READY;
            woke = 1;
            break;
        }
    }
    if (!woke) {
        if (m->count >= m->capacity) {
            __asm volatile("cpsie i" ::: "memory");
            return 1;
        }
        m->buf[(m->head + m->count) % m->capacity] = msg;
        m->count++;
    }
    __asm volatile("cpsie i" ::: "memory");

    if (woke)
        schedule();                    /* may preempt the poster */
    return 0;
}

int mbox_pend(mbox_t *m, uint32_t ticks, uint32_t *msg)
{
    int me;

    if (!m || current < 0)
        return 1;

    __asm volatile("cpsid i" ::: "memory");
    if (m->count > 0) {
        if (msg)
            *msg = m->buf[m->head];
        m->head = (m->head + 1) % m->capacity;
        m->count--;
        __asm volatile("cpsie i" ::: "memory");
        return 0;
    }
    me = current;
    T[me].pend_mbox = m;
    T[me].pend_msg = msg;
    T[me].pend_result = 1;
    if (ticks) {
        T[me].wake_tick = tick_count + ticks;
        T[me].deadline = 1;
    } else {
        T[me].deadline = 0;
    }
    T[me].state = TSTATE_PEND;
    __asm volatile("cpsie i" ::: "memory");

    schedule();                        /* PendSV switches away and back */
    return T[me].pend_result;
}

/* ---- start ----------------------------------------------------------------- */

static void idle_task(void *arg)
{
    (void)arg;
    for (;;)
        __asm volatile("wfi");
}

void rtos_start(void)
{
    /* idle task guarantees the scheduler never runs dry */
    task_create(idle_task, 0, 256, 0xFF);

    SCB_VTOR = 0x08000000u;
    rtos_pick_next();

    __asm volatile(
        "ldr    r0, =current            \n"
        "ldr    r0, [r0]                \n"
        "lsls   r0, r0, #2              \n"
        "ldr    r1, =task_sp            \n"
        "adds   r1, r1, r0              \n"
        "ldr    r0, [r1]                \n"
        "adds   r0, r0, #32             \n"   /* PSP -> r0 slot           */
        "msr    psp, r0                 \n"
        "movs   r0, #2                  \n"
        "msr    CONTROL, r0             \n"
        "isb                            \n"
        "ldr    r1, =0xFFFFFFF9         \n"   /* exception return, thread/PSP */
        "bx     r1                      \n");
    for (;;)
        ;
}

void board_setup_tick(uint32_t hclk_hz)
{
    SYST_RVR = hclk_hz / 1000u - 1u;
    SYST_CVR = 0;
    SYST_CSR = 7;                    /* ENABLE | TICKINT | CLKSOURCE */
}

/* SysTick_Handler: the original's is stripped (vector entry [15] points into
 * a BEEF region); reconstruction asserts the 1 ms tick directly. */
void SysTick_Handler(void) __attribute__((weak, alias("rtos_tick")));
