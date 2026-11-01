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

/* prio 10 [0x08017664 | RECOVERED cadence, session 69 tick correction]: the
 * background maintenance task; every 8th pass runs a maintenance call
 * (0x0801E6F4). Body SCAFFOLD. The "1 ms" cadence was the session-7 1 ms
 * tick-era reading; on the original's 720 Hz kernel the closest expressible
 * period is one tick (1.389 ms). OSTimeDlyHMSM(0,0,0,1) computes 0 ticks at
 * 720 Hz (OS_ERR_TIME_ZERO_DLY = no block), which would busy-spin this
 * prio-10 task and starve the dispatcher and the UI. */
static void periodic_task(void *arg)
{
    (void)arg;
    uint32_t n = 0;
    for (;;) {
        if (++n >= 8) {
            n = 0;
            /* original calls 0x0801E6F4 here (unidentified maintenance)  */
        }
        time_dly(1);
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
    /* [s72] the request-time alignment: the original's $A00 request TX =
     * virt 0.79 s = pass 19 on the 30-tick pass grid (the peer pair +
     * the ck2matrix runs, 1x $A00 both). The recon's unaligned task start
     * fired it at 0.99 s (the documented boot-flush delta) — the delta
     * also straddled the ck2matrix F2 checkpoint (the recon's drain
     * landed before the 1.1-s dump, the orig's after). 19 passes x 30
     * ticks = 570 ticks = 0.7917 s. */
    while (rtos_ticks() < 570u)
        time_dly(5);
    protocol_send_cmd(0);            /* CMD_A00_READ — boot settings fetch */
    /* [session 62] the radio apply is REPLY-GATED: with no peer (the
     * fresh-EE differential harness) the original performs NO radio
     * work at boot — its first RF traffic is the RSSI task's first
     * poll (p3k6_orig/p3k6_rec streams agree bit-for-bit until the
     * 187041 poll). The full default apply (the CK3 peer-harness
     * content) runs only when the 0x216-byte $A00 settings frame
     * actually arrives; ~1 s bounded wait then idle. */
    /* [session 71] the wait is the request-gated RX poll: the settings
     * frame is HELD in the USART2 ring by the ISR and drained here at
     * the first post-request pass that finds it complete — mirroring
     * the original's consumption (the ck2matrix differential: the
     * pre-request frame consumed post-request; the buffers frozen
     * after; the frame-ready bit0 never observable between passes).
     * The window closes at the drain (pre-apply), so any later
     * unsolicited frames are never routed. No-peer boots (p3/CK4
     * fresh-EE) idle in this loop with the window never opened.
     * [s72 | the cadence + the drain-gate replay] the original's
     * dispatcher is grid-aligned: the request TX at pass 19 (0.7917 s,
     * the peer/matrix TX observable) and the WARM-boot drain at pass 27
     * (1.125 s — the session-71 warm-EE matrix checkpoints bracket it:
     * the F2 dump at 1.1 s is PRE-drain (rxBuf = 0x00, the flag = 0x00),
     * the F3 dump at 1.4 s is POST-drain (the frame head in rxBuf+rxBuf2,
     * the flag = 0x01); the ck3 reference (session 49, the warm-EE era)
     * shows the same drain-then-apply 292-event structure.
     * [s72 round-1 DISCOVERY] the FRESH-EE original NEVER drains: the
     * fresh-EE matrix pair (this session) shows the orig's rxBuf/rxBuf2
     * = 0x00 and the flag = 0x00 at EVERY checkpoint through 4.4 s while
     * the request still fires (the fresh peer pair: 1x $A00 @0.8 s) —
     * the factory-reset boot ignores the settings reply entirely (the
     * reset IS the settings source), consistent with the fresh-EE "no
     * radio work at boot" evidence (p3k6/ck4). The drain is therefore
     * gated on the WARM boot (ui_ee_warm = the settings magic matched).
     * The recon's ring frame IDENTITY (the recon receives the
     * pre-request F1 frame its receiver caught at 0.5 s; the warm-EE
     * orig run dropped it — the receiver not yet enabled) is
     * unobservable in the 49-byte checkpoint dumps (both frames share
     * the dumped 16-byte "$A00,000..." head). */
    for (;;) {
        if (ui_ee_warm != 0u && rtos_ticks() >= 810u && u2_rx_poll()) {
            u2_rx_drain();                   /* route the first held frame  */
            u2_rx_open = 0u;                 /* the one-shot close          */
            if (protocol_rx_len() == 0x216u) {     /* the REAL $A00 reply   */
                protocol_settings_archive(); /* [session 71] the consumer-side
                                                * rxBuf2 archive (the ck2matrix
                                                * F4 evidence) */
                at1846s_boot_default_apply();
                /* [s72] the settings application consumed the dirty state:
                 * the boot-end both-slot refresh is cancelled (rf.c
                 * rf_boot_refresh_pending for the chain). The ck3
                 * differential: the orig's peer trace (the apply fired in
                 * the reply window) contains zero poll frames/reads in the
                 * 5-s window, while the fresh no-peer boot refreshes at the
                 * render end (the ck4/p3k6 census at stream 187041). */
                rf_boot_refresh_pending = 0u;
                break;
            }
        }
        time_dly(30u);   /* [s72] the dispatcher's pass grid (the orig's
                          * request/drain are both 30-tick aligned) */
    }
    /* [s72] the boot's late flag phase (protocol.c for the provenance):
     * the +9 byte = 0xC0 from the F4 checkpoint (1.7 s) on, while the F3
     * dump (1.4 s) still reads the drain's 0x01 pulse — the write lands
     * at the pass-38 boundary (1140 ticks = 1.583 s), the observed
     * bounds' middle. The no-apply paths leave the byte at the drain
     * pulse (unobserved in any committed scenario's checkpoints). */
    while (rtos_ticks() < 1140u)
        time_dly(30u);
    protocol_boot_flags_phase();
    for (;;) {
        time_dly(5);
    }
}

/* prio 13 [0x08017704 | RECOVERED cadence + RF read]: 500 ms RSSI poll.
 * The poll = the per-poll RF refresh (at1846s_rssi_poll: block_b + tone
 * pair + the gap replay + the GPIO reconfig + the Phase-Y apply + the
 * rf_read, session 62 frame-decode of the 996-bit bursts); the read
 * value feeds the S-meter state (0xFFF = invalid). */
static void rssi_task(void *arg)
{
    /* [session 66 | FUN_0801ab36 REARCHITECTURE] the original's per-slot
     * activate is NOT free-running: REA + the Renode hook census show its
     * rf_read (0x0802692E) fires exactly 5x in the p3-key6 scenario, from
     * FUN_08022a98 (the both-slot refresh, sites 0x8022ae7/0x8022b07) at
     * the boot end and at the release, and FUN_08019910 (site 0x8019ba5)
     * at the key-10 press — never on a 500 ms cadence. The poll moved to
     * the ui.c call sites; the task blocks forever. */
    (void)arg;
    for (;;) {
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
    /* [session 55] the vendor pre-LCD I2C/boot traffic: replayed
     * [DATA-DERIVED] (2,507 events, ends with the PB9 latch LOW as the
     * original's stream requires). Supersedes the inferred 2-byte probe
     * (which emitted 63 events the original's vendor region emits
     * differently — see display.c display_vendor_replay). */
    display_vendor_replay();
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
     * (0x080267AA, session 49 decompile). The AT1846S register init
     * does NOT run here: the fresh-EE boot stream goes display init ->
     * the vendor replay -> straight into the I2C context read with NO
     * radio traffic (session 62 — the 2,400-bit init block the recon
     * used to emit at this point is the dispatcher's default-apply
     * content, which runs later from the dispatcher task). */
    rf_bus_gpio_init();
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
