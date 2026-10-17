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
#include "ucos_ii.h"   /* OSInit — [session 44 fix] the kernel init was omitted */
#include "uart.h"
#include "protocol.h"
#include "display.h"
#include "ui.h"
#include "eeprom.h"
#include "rf.h"

/* ---- tasks ---------------------------------------------------------------- */

/* prio 3 [0x08029178 | RECOVERED structure]: key-scan loop. The key
 * matrix/scan function 0x080175A8 and handler 0x08017738 are not yet
 * reconstructed; body is SCAFFOLD. */
static volatile uint8_t ui_key_last;

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
 * dispatcher (FUN_08010D5E, 67 callees). The dispatcher's boot path
 * requests the settings through the command dispatcher (0x08010128): the
 * original's "$A00" is observed on USART2 at ~0.8 s virtual in Renode
 * (session 48) and the 8051 answers with the 534-byte settings frame.
 * The remaining dispatcher body is SCAFFOLD. */
static void dispatcher_task(void *arg)
{
    (void)arg;
    protocol_send_cmd(0);            /* CMD_A00_READ — boot settings fetch */
    /* [session 49] The original applies its power-on default RF channel
     * ~3.1 s virtual after boot (plain-boot trace, no peer). The event
     * ORDER is what CK3 compares; the wall-clock gap is a §7(c) timing
     * boundary (the original's task/delay chain is not reconstructed). */
    at1846s_boot_default_apply();
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
 * 8-slot ring. SCAFFOLD body. The original creates the task's queue in the
 * 0x08028C26 cluster and passes the handle as parg (read by the task into
 * the ring-service loop); we mirror that so the task blocks on an empty
 * queue instead of spinning on a NULL handle. */
static void service_task(void *arg)
{
    mbox_t *m = (mbox_t *)arg;
    for (;;) {
        uint32_t msg;
        (void)mbox_pend(m, 0, &msg);   /* blocks while no producer posts */
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
void hw_init(void)
{
    /* Clock tree (108 MHz HCLK / 2 flash wait states) is configured by the
     * vendor SystemInit() called from Reset_Handler — its code is present
     * in the original image [VENDOR | RECOVERED, see startup.c]. */
    RCC_APB2ENR |= RCC_APB2ENR_GPIOAEN | RCC_APB2ENR_GPIOBEN |
                   RCC_APB2ENR_GPIOCEN | RCC_APB2ENR_AFIOEN;

    board_setup_tick(108000000u);

    usart1_init();        /* GPS input  [0x0802932C RECOVERED]             */
    usart2_init();        /* APRS/BT link [0x0801F810 RECOVERED]           */
    /* EEPROM probe [session 51]: the original's pre-LCD I2C = a tiny
     * 2-byte access (5 CRH direction switches in the stream head). The
     * exact call site = in the Ghidra-missed boot region — inferred.
     * [session 53] the vendor's pre-LCD ops leave the PB9 latch LOW
     * (the LCD init's ODR = 0x00xx), so the probe tail restores SDA
     * low — the app-style stop leaves it HIGH and would poison every
     * later display ODR read-modify-write. DATA-DERIVED tail. */
    {
        static uint8_t ee_probe[2];
        eeprom_read(0x0000u, 2u, ee_probe);
        GPIO_BRR(GPIOB_BASE) = 0x200u;
    }
    display_init();       /* [0x0801BB22 RECOVERED]                        */
    /* [0x0802BDF4 boot GPIO init, missed-region disassembly]: PB13-15 =
     * open-drain output (CRH nibbles 0xB — the backlight/control lines),
     * PD2 = output low (gpio_init mask 4 mode 3, then BRR). The original
     * calls this immediately after the LCD init (0x0802D0C6). */
    GPIO_CRH(GPIOB_BASE) = (GPIO_CRH(GPIOB_BASE) & ~0xFFF00000u) | 0xBBB00000u;
    GPIO_CRL(GPIOD_BASE) = (GPIO_CRL(GPIOD_BASE) & ~0xF00u) | 0x300u;
    GPIO_BRR(GPIOD_BASE) = 4u;
    /* the 0x4D-byte settings load moved to ui_boot_scene() [0x0801B738
     * runs it after the task creators, per the recovered main task]. */
    /* RF chip init [0x080227AA call site RECOVERED]: bus GPIO config
     * (0x080267AA, session 49 decompile) then the 50-register AT1846S
     * init table applied to chip 1 then chip 0 (session 49 trace,
     * analysis/traces/ck3_rf_orig.txt events 1-100). */
    rf_bus_gpio_init();
    at1846s_boot_init(RF_CHIP_UHF);   /* cs mode 1 = first-configured chip */
    at1846s_boot_init(RF_CHIP_VHF);   /* cs mode 0 = second chip           */
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
    /* [RECOVERED, main prologue 0x08029120..0x08029130]: VTOR = (0x4000 &
     * 0x1FFFFF80) | 0x08000000 = 0x08004000 — the app's own vector table
     * (the 16 KB below it is the non-distributed bootloader). Our link
     * base is 0x08004000, so this points at our table exactly like the
     * original. */
    REG32(0xE000ED08u) = (0x4000u & 0x1FFFFF80u) | 0x08000000u;

    /* PendSV lowest-priority setup is performed by the port's
     * OSStartHighRdy (os_cpu_a.S).                                        */

    /* [session 52 fix] hw_init is NOT called here: the original's
     * peripheral init (FUN_0802D0D0) runs INSIDE the main task
     * (FUN_0802D178), once. The duplicate call doubled the display
     * init on the CK4 stream. */

    /* [session 44 fix | RECOVERED correction] The original's main calls
     * OSInit() before task creation (standard Micrium flow; OSStart at
     * 0x08027392 is preceded by kernel init). The reconstruction omitted
     * it: the kernel pools stayed zero and OSInitHookBegin — which
     * initializes OS_CPU_ExceptStkBase, loaded into MSP by
     * OSStartHighRdy — never ran, so the first exception bus-faulted
     * (Renode CK1 evidence: BFSR.PRECISERR, HFSR.FORCED). */
    OSInit();

    /* Main/UI task, prio 3, 1 KB stack [0x0802915A]. */
    (void)ui_key_last;
    (void)task_create(ui_task, 0, 1024, 3);

    /* App tasks created by the original's task-creator (0x080175A8):
     * prio 10 / 13 with 1 KB stacks, prio 11 with 1 KB stack. */
    (void)task_create(periodic_task,   0, 1024, 10);
    (void)task_create(rssi_task,       0, 1024, 13);
    (void)task_create(dispatcher_task, 0, 1024, 11);

    /* Service tasks 29/30/31 with 512 B stacks; the service task owns a
     * mailbox (original creates mailboxes in the 0x08028C26 cluster and
     * passes the handle as the task's parg). */
    (void)task_create(service_task, (void *)mbox_create(), 512, 29);
    (void)task_create(timeout_task, 0, 512, 30);
    (void)task_create(stats_task,   0, 512, 31);

    rtos_start();                        /* [0x08027392 — OSStart, now      */
    return 0;                            /*  confirmed from main's tail]    */
}
