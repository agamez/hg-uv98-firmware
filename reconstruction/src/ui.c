/* Master UI task, input policy, and UI state machine — [RECOVERED]
 * from the original image (sessions 18-22).
 *
 * TASK (image labels; physical flash = label + 0x4000):
 *   main 0x08029120: NVIC_SetVectorTable(0x08000000, 0x4000) -> inits ->
 *   OSTaskCreateExt(ui_task, prio 3, stack 0x100, name-set 0x08024A6C)
 *   -> OSStart 0x08027392.
 *   ui_task 0x08029178: hw_init 0x080290D0 -> task creators 0x080175A8
 *   (returns -1 -> task exits) and 0x08017738 -> loop:
 *     wait 30 ticks (0x0801FF4C); heartbeat [0x2000469C+0x166]++ (u16);
 *     skip UI work while suspend byte [0x2000469C+0x119] != 0;
 *     0x0800FE58 + 0x08010302 + 0x08015DCC every pass;
 *     0x0800ED24 every 17th pass (counter resets at 0x11).
 *
 * ROUTERS (roles, session 21-22):
 *   0x0800FE58  protocol-sentence -> UI bridge: memcmp 5-byte prefixes
 *               ("$A00," from flash at linked 0x0804476C; two .data RAM
 *               prefixes), handling gated on the UI state byte.
 *   0x08010302  per-UI-state progress driver (state 'h' runs a 41-item
 *               sequence via the u16 counter at struct+0x135).
 *   0x08015DCC  battery-gauge poll: quantizer 0x080242FC (0xFF pending)
 *               -> zero gauge state, seed bars=10, redraw 0x080202B8.
 *   0x0800ED24  blinking mode indicator: states 'M'/'N', toggles flag
 *               bit6, erases a 2-char field via 0x08018B60(.., "  ", ..),
 *               restores via 0x0800BC48(0xFF).
 *
 * UI STATE BYTE at RAM 0x200050E8: printable-character enum (session
 * 22): 0x00 idle (27 compare sites) + 0x02 0x04 0x2F 0x36 0x3F 0x40 0x45
 * 0x47 0x4D 0x50 0x58 0x67 0x68 0x69 0x78 0x7B 0x86. NOT protocol
 * letters (the 8051 sources contain no such sentence prefixes).
 *
 * INPUT POLICY (0x0801E044, ~750 B): every key event passes gates —
 * menu-open bit (flag-bank byte+2 bit2), key-lock bytes (+0x10D/+0x10E
 * at 0x200051A1/A2), lock-flag bits (+8 bits 2-3), then per-source
 * branching. Sources (session 20):
 *   - GPIOC scanner cluster 0x08021000: direct pins -> IDs 0x00/0x01
 *     with press/release + 500 ms auto-repeat (debounce 5).
 *   - PTT synthesizer 0x08019F70: state transitions via
 *     key_state_test() emit key_event(0/1) pairs, or key_event(prog_id).
 *   - Software key loop 0x08020530: for i: key_event(i,
 *     key_state_test(state_table[i]), 0) — extended keypad is
 *     software-polled; per-key press/release timestamps recorded at
 *     [base+0x5A / +0x62 + key*4] by 0x0801B2A0.
 *   - Programmable-key dispatchers 0x0802021C / 0x0801C54E /
 *     0x0801D41E: emit key_event([struct+0x40], ..); 0x0801D41E also
 *     writes per-key state into the key-lock byte +0x10D.
 * key IDs for programmable keys = RAM config (.data image absent from
 * the distributed file — session 19).
 *
 * BATTERY CHAIN (sessions 21-22): reader 0x080241A6 arms a conversion
 * (pending returns 1/2) then quantizes the ADC count (channel 14 = PC4
 * divider via BSP driver 0x080256CA, lock 0x08024CC6) against thresholds
 * 0xCB5 (3253) and 0x6D7 (1751) into levels {15, 12, 1, 0};
 * 0x080242FC maps a level to bar counts (bounds 0/2/4/6/7). The raw
 * threshold tree bodies are SCAFFOLD here — constants recovered, branch
 * polarity per-level not fully decoded.
 */
#include <stdint.h>

#include "ui.h"


/* RAM anchors used by the recovered code (addresses as in the log). */
#define FLAG_BANK       0x20005094u     /* protocol/UI flag bytes        */
#define UI_STATE_ADDR   0x200050E8u     /* master UI state byte          */
#define APP_STATE       0x2000469Cu     /* app-state struct (heartbeat)  */

#define REG8(a)   (*(volatile uint8_t *)(a))
#define REG16(a)  (*(volatile uint16_t *)(a))

/* --- extern hooks (SCAFFOLD; wired in main.c/rtos.c glue) ------------- */
void hw_init(void);
uint32_t ui_tasks_create_1(void);
void ui_tasks_create_2(void);
void os_wait_30_ticks(void);
void router_protocol_bridge(void);
void router_state_progress(void);
void router_battery_poll(void);
void router_slow_housekeeping(void);
int  battery_adc_read_ch14(void);

/* [0x0801B28E | RECOVERED] the complete body: bit-6 active test. */
int key_state_test(uint8_t state_byte)
{
    return (state_byte >= 0x40) ? 1 : 0;
}

/* [0x0801E044 | RECOVERED-mechanism] key-event policy gate. The gate
 * order below follows the disassembly; the per-source tail (~500 B of
 * branching) is SCAFFOLD. */
void key_event(uint8_t key_id, uint8_t src, uint8_t pressed)
{
    (void)key_id; (void)pressed;

    /* menu-open gate: flag-bank byte+2 bit2 */
    if ((REG8(FLAG_BANK + 2) >> 2) & 1u) {
        /* both key-lock bytes must be set for events to pass */
        if (REG8(FLAG_BANK + 0x10D) == 0 || REG8(FLAG_BANK + 0x10E) == 0)
            return;
    }
    /* lock flags: +8 bits 2-3 block events */
    if (REG8(FLAG_BANK + 8) & 0x0Cu)
        return;
    /* src-specific policy continues (SCAFFOLD) */
    (void)src;
}

/* [0x080241A6 | RECOVERED constants / SCAFFOLD branches] battery level.
 * Thresholds and output levels from the disassembly; the conversion
 * arm/pending handshake returns 1/2 while a conversion is in flight. */
int battery_level_read(void)
{
    enum { TH_FULL = 0xCB5, TH_LOW = 0x6D7 };   /* 3253 / 1751 */
    int adc = battery_adc_read_ch14();          /* [0x080256CA(0xE)] SCaffold */
    if (adc < 0)
        return -1;                              /* pending (original: 1/2) */
    if (adc >= TH_FULL)
        return 15;                              /* full */
    if (adc >= TH_LOW)
        return 12;                              /* high */
    return (adc > 0) ? 1 : 0;                   /* low / critical */
}

/* [0x080242FC | RECOVERED-mechanism] level -> bar count; the original
 * bounds the level at 0/2/4/6/7 with descending bar counts (10 = full
 * seed at struct+0xDF). */
int battery_bars(uint8_t level)
{
    switch (level) {
    case 0:  return 10;
    case 2:  return 8;
    case 4:  return 6;
    case 6:  return 4;
    case 7:  return 3;
    default: return 2;
    }
}

/* [0x08029178 | RECOVERED] master UI task. The four routers are named
 * stubs here — their bodies live in the log (sessions 20-22). */
void ui_task(void *p_arg)
{
    (void)p_arg;
    hw_init();                                  /* 0x080290D0 */
    if (ui_tasks_create_1() == 0xFFFFFFFFu)     /* 0x080175A8 */
        return;
    ui_tasks_create_2();                        /* 0x08017738 */

    for (;;) {
        os_wait_30_ticks();                     /* 0x0801FF4C(0x1E) */
        REG16(APP_STATE + 0x166)++;             /* heartbeat */
        if (REG8(APP_STATE + 0x119) != 0)       /* UI suspended */
            continue;
        router_protocol_bridge();               /* 0x0800FE58 */
        router_state_progress();                /* 0x08010302 */
        router_battery_poll();                  /* 0x08015DCC */
        if (REG16(APP_STATE + 0x166) == 0x11) {
            REG16(APP_STATE + 0x166) = 0;
            router_slow_housekeeping();         /* 0x0800ED24 */
        }
    }
}

