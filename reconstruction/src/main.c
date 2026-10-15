/* Boot chain and task set.
 *
 * RECOVERED structure (research log sessions 6-8):
 *   - main() [0x08029120]: NVIC setup, create the main task, start the OS.
 *   - seven tasks created via OSTaskCreateExt with the priorities, stacks
 *     and roles tabulated below. Task BODIES are SCAFFOLD/INFERRED where
 *     noted; the task structure itself is RECOVERED.
 *   - hardware init chain 0x080290D0 -> 0x08029080: GPIO inits, RF chip
 *     init 0x080227AA, USART1 init + pump, misc. USART2 init 0x0801F810
 *     sits in the same cluster.
 */
#include "hwregs.h"
#include "rtos.h"
#include "uart.h"
#include "protocol.h"
#include "display.h"

/* ---- tasks ---------------------------------------------------------------- */

/* prio 3 [0x08029178 | RECOVERED structure]: key-scan loop. The key
 * matrix/scan function 0x080175A8 and handler 0x08017738 are not yet
 * reconstructed; body is SCAFFOLD. */
static volatile uint8_t ui_key_last;

static void ui_task(void *arg)
{
    (void)arg;
    for (;;) {
        /* INFERRED cadence: original delays 30 ticks per loop. */
        if (ui_key_last)
            protocol_send_status_text("\0\0\0\0\0\0");   /* placeholder  */
        time_dly(30);
    }
}

/* prio 10 [0x08017664 | RECOVERED cadence]: 1 ms periodic background task;
 * every 8th pass runs a maintenance call (0x0801E6F4). Body SCAFFOLD. */
static void periodic_task(void *arg)
{
    (void)arg;
    uint32_t n = 0;
    for (;;) {
        if (++n >= 8) {
            n = 0;
            /* original calls 0x0801E6F4 here (unidentified maintenance)  */
        }
        time_dly_hmsm(0, 0, 0, 1);
    }
}

/* prio 11 [0x08014DEA | RECOVERED]: trampoline into the master UI
 * dispatcher (FUN_08010D5E, 67 callees). The dispatcher's event source
 * and case table are not yet mapped; body SCAFFOLD. */
static void dispatcher_task(void *arg)
{
    (void)arg;
    for (;;) {
        time_dly(5);
    }
}

/* prio 13 [0x08017704 | RECOVERED cadence + RF read]: 500 ms RSSI poll.
 * The RF register read (0x0801F4CC via the bit-bang helper at 0x0802292E,
 * register 0x1F, 12-bit value, 0xFFF = invalid) is RECOVERED; the S-meter
 * distribution goes to the 6-byte status feed. RF transport SCAFFOLD. */
static void rssi_task(void *arg)
{
    (void)arg;
    for (;;) {
        uint32_t rssi = 0xFFF;    /* original: read RF reg 0x1F here        */
        if (rssi != 0xFFF) {
            /* original updates the S-meter state consumed by the
             * status-text dispatcher ("-9.0 dB" .. "-0 dB", "NN S")       */
        }
        time_dly(500);
    }
}

/* prio 29 [0x08028D74 | RECOVERED structure]: mailbox service task over an
 * 8-slot ring. SCAFFOLD body. */
static void service_task(void *arg)
{
    (void)arg;
    for (;;) {
        uint32_t msg;
        (void)mbox_pend((mbox_t *)arg, 0, &msg);
        /* original: process ring slot (index & 7) */
    }
}

/* prio 30 [0x08027938 | RECOVERED cadence]: timeout/counter task dividing a
 * software counter by 100, firing an event at zero. SCAFFOLD body. */
static void timeout_task(void *arg)
{
    (void)arg;
    for (;;) {
        time_dly(200);            /* original polls its flag every 200 ms  */
    }
}

/* prio 31 [0x08027918 | RECOVERED cadence]: low-priority statistics-style
 * task (critical-section counter + helper). SCAFFOLD body. */
static void stats_task(void *arg)
{
    (void)arg;
    for (;;) {
        time_dly(1000);
    }
}

/* ---- hardware init --------------------------------------------------------- */

/* [0x080290D0 -> 0x08029080 | RECOVERED structure]: clock/GPIO setup, RF
 * chip init (0x080227AA), USART1 (0x0802932C), USART2 (0x0801F810).
 * PLL/clock tree configuration lived in the stripped SystemInit; the
 * reconstruction asserts the documented 108 MHz HCLK as INFERRED. */
static void hw_init(void)
{
    /* Clock tree (108 MHz HCLK / 2 flash wait states) is configured by the
     * vendor SystemInit() called from Reset_Handler — its code is present
     * in the original image [VENDOR | RECOVERED, see startup.c]. */
    RCC_APB2ENR |= RCC_APB2ENR_GPIOAEN | RCC_APB2ENR_GPIOBEN |
                   RCC_APB2ENR_GPIOCEN | RCC_APB2ENR_AFIOEN;

    board_setup_tick(108000000u);

    usart1_init();        /* GPS input  [0x0802932C RECOVERED]             */
    usart2_init();        /* APRS/BT link [0x0801F810 RECOVERED]           */
    display_init();       /* SCAFFOLD                                      */
    /* RF chip init [0x080227AA | RECOVERED call site; transport SCAFFOLD]  */
}

/* ---- main -------------------------------------------------------------------
 * [0x08029120 | RECOVERED]: NVIC setup, task creation, OS start. The
 * original creates the main task first (prio 3) and starts the OS from
 * main; remaining tasks are created by the main task's init path
 * (0x080175A8 creates prios 10/13/11 with 1 KB stacks; service tasks
 * 29/30/31 with 512 B stacks are created in the 0x080277xx-0x08028C6x
 * cluster). We mirror that ordering. */
int main(void)
{
    int mbox_owner;

    /* PendSV lowest-priority setup is performed by the port's
     * OSStartHighRdy (os_cpu_a.S).                                        */

    hw_init();

    /* Main/UI task, prio 3, 1 KB stack [0x0802915A]. */
    (void)ui_key_last;
    (void)task_create(ui_task, 0, 1024, 3);

    /* App tasks created by the original's task-creator (0x080175A8):
     * prio 10 / 13 with 1 KB stacks, prio 11 with 1 KB stack. */
    (void)task_create(periodic_task,   0, 1024, 10);
    (void)task_create(rssi_task,       0, 1024, 13);
    (void)task_create(dispatcher_task, 0, 1024, 11);

    /* Service tasks 29/30/31 with 512 B stacks; the service task owns a
     * mailbox (original creates mailboxes in the 0x08028C26 cluster). */
    mbox_owner = task_create(service_task, 0, 512, 29);
    (void)mbox_owner;
    (void)task_create(timeout_task, 0, 512, 30);
    (void)task_create(stats_task,   0, 512, 31);

    rtos_start();                        /* [0x08027392 candidate]          */
    return 0;
}
