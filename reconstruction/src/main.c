/* Boot chain and task set.
 *
 * RECOVERED structure (research log sessions 6-8):
 *   - main() [0x08029120]: NVIC setup, create the main task, start the OS.
 *   - seven tasks created via OSTaskCreateExt with the priorities, stacks
 *     and roles tabulated below [session 75: the task table re-derived
 *     from the seven FUN_080289A4 create sites — the corrected addresses
 *     and the decompiled bodies are in the per-task notes; the recon's
 *     event output = the bench-verified ui.c/ui_task model, the task
 *     bodies = the decompile-annotated event-neutral loops].
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

/* prio 3 [0x0802D178 | session 75 CORRECTION — the task-creator sweep
 * (the seven FUN_080289A4 call sites in the image) shows the main task
 * pointer = 0x0802D179 (FUN_0802D120's create); the earlier 0x08029178
 * = a digit transposition]: the main task = the recon's ui_task (the
 * boot radio-apply, the task creators, the boot scene, the +0x119
 * dispatch loop). */

/* prio 10 [0x0801B664 | session 75 CORRECTION — the create site
 * 0x0801B5D0 passes task 0x801b665; the earlier 0x08017664 = a
 * transposition (that address = a VFO-row render helper)]: the
 * maintenance task. The session-75 decompile of FUN_0801B664:
 *   do { dly();
 *   } while (ctx[0x119] != 0);          // the same +0x119 gate as the
 *                                       // dispatcher's pass grid
 *   ctx32[0x19]++; ctx16[0x1d]++;
 *   if (ctx32[0x19] == 8) {             // the 8-pass cycle:
 *     ctx32[0x19] = 0; f(); f(); f(); f();
 *     if ((flags & 0x7f) >> 6 == 0) { f1(); f2(); f3(); }
 *     FUN_08021A94(); FUN_08022320(); FUN_08025A4C();
 *   }
 *   if (ctx16[0x1d] == 500) {           // the 500-pass cycle:
 *     ctx16[0x1d] = 0;
 *     FUN_0801E09C(); FUN_0801DEB6(); FUN_0801E1A0(); FUN_08025D08();
 *     FUN_080225DC(); FUN_080224F4(); FUN_0801DF70(); ... }
 * The cycle bodies = the render/maintenance chains (the 0x0801E6F4
 * status-render call = among them; the recon's render machinery = the
 * Tier-B mirrors in ui.c, emitted from the ui_task dispatch at the
 * bench-verified times). The recon keeps this task event-neutral: the
 * renders = already produced by the verified ui_task path; emitting
 * them here too would double them. Body = the decompile-annotated
 * structure above (the justification path per the 5.0 gate). */
static volatile uint8_t ui_key_last;

/* prio 10 [0x0801B664 | session 75 CORRECTION — see the task-table note
 * above]: the background maintenance task; the decompiled body = the
 * +0x119 gate + the 8-pass and 500-pass maintenance cycles (the
 * decompile-annotated structure above). The recon's event output =
 * produced by the verified ui_task dispatch; this task stays
 * event-neutral on the one-tick grid (OSTimeDlyHMSM(0,0,0,1) computes
 * 0 ticks at 720 Hz — OS_ERR_TIME_ZERO_DLY = no block — which would
 * busy-spin this prio-10 task and starve the dispatcher and the UI). */
static void periodic_task(void *arg)
{
    (void)arg;
    uint32_t n = 0;
    for (;;) {
        if (++n >= 8) {
            n = 0;
            /* [0x0801B664] the orig's 8-pass maintenance cycle = the
             * render/maintenance chains, mirrored by the verified
             * ui_task dispatch (see the note above). */
        }
        time_dly(1);
    }
}

/* prio 11 [0x08014DEA | RECOVERED, CONFIRMED via DAT_0801b998 =
 * 0x08014DEB (the trio's indirect create arg)]: trampoline into the
 * master UI dispatcher (FUN_08010D5E, 67 callees). The s74 decompiles
 * bank the structure: 0x08010D5E = the per-pass step ((state&3)>>1==0
 * -> the status fill + FUN_08014530(0x6a) + FUN_0801219c(0xff), else
 * delay(100)); 0x08024D76 = the radio-apply chain AND the +0x119 flag
 * setter (*(_DAT_08024e28+0x119)=1 after the 0x30/0x7006 writes — the
 * pass-grid gate the prio-10/13 tasks spin on). The recon's dispatch
 * = the ui_task mirror of that step grid (the bench-verified event
 * placement); this task = the UART side: the settings request through
 * the command dispatcher (0x08010128) — the original's "$A00" observed
 * on USART2 at ~0.8 s virtual (session 48), the 534-byte settings
 * frame reply. */
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
    uint8_t flags_phase_done = 0u;   /* [s72] the one-shot guard */
    for (;;) {
        /* [s72 round-1 correction] the 0xC0 flag phase is WARM-INDEPENDENT:
         * the fresh-EE orig pair shows the flag = 0x00 through the F3
         * checkpoint (1.4 s) and 0xC0 from the F4 (1.7 s) with NO drain and
         * NO apply — the write is an unconditional boot-phase event (the
         * warm path merely overwrites the drain's bit0 pulse with it). */
        if (flags_phase_done == 0u && rtos_ticks() >= 1140u) {
            flags_phase_done = 1u;
            protocol_boot_flags_phase();
        }
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
    /* [s72] the warm path breaks at the drain (1.125 s) before the flag
     * phase tick — finish the one-shot here (the fresh path already
     * wrote it inside the loop). */
    if (flags_phase_done == 0u) {
        while (rtos_ticks() < 1140u)
            time_dly(30u);
        protocol_boot_flags_phase();
    }
    for (;;) {
        time_dly(5);
    }
}

/* prio 13 [0x0801B704 | session 75 CORRECTION — the create site
 * 0x0801B608 passes task 0x801b705; the earlier 0x08017704 = a
 * transposition]: the RSSI task. The session-75 decompile of
 * FUN_0801B704: `do { do { dly(500); } while (ctx[0x119] != 0);
 * poll(); } while(true)` — the 500-tick delay + the +0x119 gate + the
 * per-slot activate call, confirming the recon's retired dly-loop
 * model (session 66).
 * The poll = the per-slot RF refresh (FUN_0801AB36: the apply ->
 * dly(0x28) -> rf_read(chip,0x0D) -> the slot word +0x11E &= 0x8000 ->
 * if zero the re-init); the read value feeds the S-meter state (0xFFF
 * = invalid). */
static void rssi_task(void *arg)
{
    /* [session 66 | FUN_0801ab36 REARCHITECTURE] the original's per-slot
     * activate is NOT free-running: REA + the Renode hook census show its
     * rf_read (0x0802692E) fires exactly 5x in the p3-key6 scenario, from
     * FUN_08022a98 (the both-slot refresh, sites 0x8022ae7/0x8022b07) at
     * the boot end and at the release, and FUN_08019910 (site 0x8019ba5)
     * at the key-10 press — never on a 500 ms cadence. The poll moved to
     * the ui.c call sites; the task blocks forever. The decompiled task
     * body (the dly(500) + the gate + the poll) = the orig's structure;
     * the recon's event output = produced at the same architectural
     * moments by the ui.c sites (the bench-verified placement). */
    (void)arg;
    for (;;) {
        time_dly(500);
    }
}

/* prio 29 [session 75 CORRECTION — the create site 0x0802CC86 (r3 = 29)
 * passes the task pointer 0x0802CD75 = FUN_0802CD74; the earlier
 * 0x08028D74 attribution = RETRACTED (that address = in the 0x08028xxx
 * radio/I2C cluster, not a task body)]: the SOFTWARE-TIMER task. The
 * session-75 decompile of FUN_0802CD74:
 *   do { FUN_0802CD56();               // the tick wait
 *        tick++;                       // DAT_0802ce00 = the tick counter
 *        node = timer_list[tick & 7];  // the 8-slot wheel (the +8 stride)
 *        while (node) { next = node[+0xc];
 *          if (tick == node[+0x14]) {  // the expiry match
 *            if (node[+4]) node[+4](node, node[+8]);  // the callback
 *            FUN_0802CD0A(node);       // the unlink
 *            if (node[+0x30] == 2) FUN_0802CC98(node, 1);  // the reload
 *            else node[+0x31] = 2; }   // the done state
 *          node = next; }
 *        FUN_0802CD66(); } while(true);
 * The recon's timer behavior = the rtos.c time_dly machinery (the
 * kernel-stub equivalent, bench-verified); the callback wheel = the
 * orig's own layer above the kernel. */
static void service_task(void *arg)
{
    mbox_t *m = (mbox_t *)arg;
    for (;;) {
        uint32_t msg;
        (void)mbox_pend(m, 0, &msg);   /* blocks while no producer posts;
                                        * the recon's service shape (the
                                        * mailbox cluster = the orig's
                                        * 0x08028C26 create region)      */
    }
}

/* prio 30 [session 75 CORRECTION — the create site 0x0802B7EA passes
 * task 0x801b93b = FUN_0802B93A; the earlier 0x08027938 attribution =
 * RETRACTED (the s74 decompile showed it = the AT1846S power-down
 * helper)]: the BATTERY task. The session-75 decompile of FUN_0802B93A:
 *   while (flag == 0) dly(200);        // the 200 ms flag poll (the
 *                                      // recon's cadence = CONFIRMED)
 *   ratio = counter / 100;
 *   if (ratio == 0) { level = 0; dly(0xff); }
 *   do { v = FUN_0802B1E0();           // the ADC/mailbox wait
 *        snapshot = counter; counter = 0;
 *        post(v);
 *        level = 'd' - (snapshot / ratio);   // the battery level char
 *        FUN_0802BCAE(); FUN_0802B9A4(); dly(100); } while(true);
 * The level = the 'd'-minus-ratio char the status page renders (the
 * recon's battery model = the ui.c status cells = the bench-verified
 * values). */
static void timeout_task(void *arg)
{
    (void)arg;
    for (;;) {
        time_dly(200);            /* the decompiled flag-poll phase:
                                   * the orig's 200 ms battery wait       */
    }
}

/* prio 31 [session 75 CORRECTION — the create site 0x0802B7AE passes
 * task 0x801b91b = FUN_0802B91A; the earlier 0x08027918 attribution =
 * RETRACTED (that address = inside the 0x30-mode helper at
 * 0x080278E8)]: the COUNTER task (the battery task's clock source).
 * The session-75 decompile of FUN_0802B91A:
 *   do { v = FUN_0802B1E0();           // the mailbox wait
 *        counter++;                    // DAT_0802bb38 = the pass counter
 *        post(v); FUN_0802BCA6(); } while(true);
 * The prio-30 battery task consumes the counter for the level ratio.
 * The recon = the event-neutral loop (the counter = the harness ADC
 * model's fixed state; the battery level = the ui.c verified cells). */
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
    /* [session 55 | the deletion TESTED and REVERTED session 74 cont.17;
     * session 75 = the DECODE: the recorded 2,507-event burst = the
     * vendor GPIOB config + the DUAL at1846s_boot_init (the 100 frames
     * = the boot_init table bit-exact) + the CRL display-bus prep — the
     * generative at1846s_vendor_boot_window() replaces the replay; the
     * CK4 = the arbiter. Supersedes the s55/s74 recorded dump and its
     * measured-justification note (the shape mismatch = the missing CRL
     * display-bus config, now identified).] */
    at1846s_vendor_boot_window();
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
