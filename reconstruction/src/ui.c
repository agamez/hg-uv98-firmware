/* Master UI task, screen-state machine, input policy, gauges —
 * [RECOVERED] from the original image (sessions 18-30; REA/Ghidra
 * validated in session 29 at the linked base 0x08004000, so REA
 * addresses = log labels + 0x4000).
 *
 * TASK (0x08029178 label, priority 3; created by main 0x08029120 via
 * OSTaskCreateExt 0x080249A4 + OSTaskNameSet 0x08024A6C; OSStart
 * 0x08027392): hw_init 0x080290D0 -> task creators 0x080175A8 (returns
 * -1 -> task exits) / 0x08017738 -> loop:
 *   wait 30 ticks (0x0801FF4C); heartbeat [0x2000469C+0x166]++ (u16);
 *   skip UI work while suspend byte [+0x119] != 0;
 *   routers 0x0800FE58 (protocol bridge) + 0x08010302 (per-state
 *   progress) + 0x08015DCC (battery poll) every pass;
 *   0x0800ED24 (blinking mode indicator) every 17th pass (counter
 *   resets at 0x11).
 *
 * SCREEN MACHINE: ui_state_set(screen) 0x08010530 —
 *   if gate flag clear && screen != 0:
 *     band byte [0x2000469C+0xF0] = [0x20004E68+0x40]
 *     memcpy 0x1E: 0x20004FD0 <- 0x20004F94 + band*0x1E  (ring snapshot)
 *   companion flag 0x200050EA = 1; screen id byte 0x200050E8 = screen.
 * Transitions: menu dispatcher (0x08011190+) = binary-search tree over
 * the selection index -> per-item byte setup + screen init
 * 0x0801715C(4, 0x1E) -> ui_state_set(screen); protocol handlers set
 * the RF states 'g'-'j'; guard paths reset to 0. ~150 setter sites
 * total (session 30).
 *
 * BEACON WINDOWS ('h'/'i'/'j', 0x08010302): pass counter
 * [0x2000469C+0x135] runs to 0x28 (fires on pass 41) or 0x32 (50) per
 * 2-bit submode (flag+0xB bits 7:6); completion clears the 50-byte
 * buffer 0x2000335C and the frame-ready bit (0x20005094+9 bit0), then
 * formats and TXes 6 bytes: "$B%02d\r\n" (index = raw byte) or
 * "$C%02d\r\n" (index = byte/5) — flash templates at linked
 * 0x08043B08/0x08043B14; TX via 0x0801FEAA.
 *
 * BATTERY: quantizer 0x080242FC (pending -> 0xFF; armed conversion
 * returns 1/2); reader 0x080241A6 quantizes ADC ch14 (PC4 divider,
 * BSP driver 0x080256CA, lock 0x08024CC6) against thresholds 0xCB5
 * (3253) and 0x6D7 (1751) into levels {15, 12, 1, 0}; 0x080242FC maps
 * a level to bar counts (bounds 0/2/4/6/7). Every pass the poller
 * 0x08015DCC zeroes gauge state, seeds bars=10, and triggers redraw
 * 0x080202B8.
 *
 * S-METER (peak-hold history): rf_read(chip, 0x1B) >> 8 = RSSI dB
 * (AT1846S 0x1B[15:8], guide-confirmed); updater 0x080203d0 keeps the
 * peak vs [+0x40] with timestamp [+0x3B]->[+0x12D]; every 5th pass
 * (sub-counter +0x3F): ring write 0x0801A8DA, shift +4 (0x0801D8C0),
 * snapshot 0x1E, and the 3 most recent entries render as bars
 * (0x0801AB36 / 0x0801A7AA / 0x0801E296). Ring entries = BCD-packed
 * peak timestamps (0x0801652C pack / 0x080164DC pair).
 *
 * INPUT POLICY (key_event 0x0801E044, ~750 B): gates — menu bit
 * (flag+2 bit2), key-lock bytes (+0x10D/+0x10E at 0x200051A1/A2),
 * lock flags (+8 bits 2-3), then per-source branching. Sources
 * (session 20): GPIOC scanner 0x08021000 (IDs 0x00/0x01 + auto-repeat,
 * debounce 5); PTT synthesizer 0x08019F70 (state transitions via
 * key_state_test; emits IDs 0/1 or the programmable ID [r6+0x40]);
 * software key loop 0x08020530 (key_event(i, key_state_test(table[i]), 0)
 * — per-key timestamps at base+0x5A/0x62 + key*4 via 0x0801B2A0);
 * programmable dispatchers 0x0802021C/0x0801C54E/0x0801D41E.
 * key_state_test 0x0801B28E = full body `state >= 0x40` (bit-6 test).
 *
 * MENU REDRAW BLOCK (0x08010D5E; corrects the session-18 pointer
 * misread): 20-byte block = packed bytes {row base (+0x11 = 17 px per
 * row), column, start index, ?, end index, selected index} + word2 =
 * u32 pointer into the RAM string tables (u32[] of string addresses)
 * + attribute bytes; rows draw via 0x08018B60 (frame-ready gated,
 * GB2312-aware, >= 0x81 pairs); then the selected row re-draws
 * (highlight). ui_list_select 0x08010D06 = leaf selection bookkeeping
 * (mod-5 wrap); activation goes through the dispatcher above.
 */
#include <stdint.h>

#include "ui.h"

#define REG8(a)   (*(volatile uint8_t *)(a))
#define REG16(a)  (*(volatile uint16_t *)(a))
#define REG32(a)  (*(volatile uint32_t *)(a))

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
void ring_write_bcd(uint32_t peak_ts, uint8_t *ring);
void screen_init(uint8_t a, uint8_t b);            /* 0x0801715C */
uint16_t rf_read_reg(uint8_t chip, uint8_t reg);   /* rf.c: 0x0802292E */

/* [0x0801B28E | RECOVERED] complete body: bit-6 active test. */
int key_state_test(uint8_t state_byte)
{
    return (state_byte >= 0x40) ? 1 : 0;
}

/* [0x08010530 | RECOVERED] the screen-switch function (REA-validated).
 * memcpy of 0x1E bytes is the active-band ring snapshot; SCAFFOLD for
 * the memcpy itself. */
void ui_state_set(uint8_t screen)
{
    if (REG8(UI_STATE_FLAG) == 0 && screen != 0) {
        REG8(APP_STATE + 0xF0) = REG8(BAND_BYTE + 0x40);
        uint8_t band = REG8(APP_STATE + 0xF0);
        for (int i = 0; i < 0x1E; i++)
            REG8(SMETER_SNAPSHOT + i) = REG8(SMETER_RINGS + band * 0x1E + i);
    }
    REG8(UI_STATE_FLAG) = 1;
    REG8(UI_STATE_ADDR) = screen;
}

/* [0x0801E044 | RECOVERED-mechanism] key-event policy gate; the
 * per-source tail (~500 B) is SCAFFOLD. */
void key_event(uint8_t key_id, uint8_t src, uint8_t pressed)
{
    (void)key_id; (void)pressed;

    if ((REG8(FLAG_BANK + 2) >> 2) & 1u) {
        if (REG8(FLAG_BANK + 0x10D) == 0 || REG8(FLAG_BANK + 0x10E) == 0)
            return;
    }
    if (REG8(FLAG_BANK + 8) & 0x0Cu)
        return;
    (void)src;
}

/* [0x080241A6 | RECOVERED constants / SCAFFOLD branches] battery level;
 * pending conversions reported as -1 (original: 1/2 handshake). */
int battery_level_read(void)
{
    enum { TH_FULL = 0xCB5, TH_LOW = 0x6D7 };   /* 3253 / 1751 */
    int adc = battery_adc_read_ch14();
    if (adc < 0)
        return -1;
    if (adc >= TH_FULL)
        return 15;
    if (adc >= TH_LOW)
        return 12;
    return (adc > 0) ? 1 : 0;
}

/* [0x080242FC | RECOVERED-mechanism] level -> bar count. */
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

/* [0x080203d0 label 0x0801C3D0 region | RECOVERED] RSSI dB read:
 * rf_read(chip, 0x1B) >> 8 = AT1846S 0x1B[15:8] rssi_db. */
uint16_t smeter_rssi_db(uint8_t chip)
{
    return (uint16_t)(rf_read_reg(chip, 0x1B) >> 8);
}

/* [0x08029178 | RECOVERED] master UI task; routers are named stubs. */
void ui_task(void *p_arg)
{
    (void)p_arg;
    hw_init();
    if (ui_tasks_create_1() == 0xFFFFFFFFu)
        return;
    ui_tasks_create_2();

    for (;;) {
        os_wait_30_ticks();
        REG16(APP_STATE + 0x166)++;
        if (REG8(APP_STATE + 0x119) != 0)
            continue;
        router_protocol_bridge();
        router_state_progress();
        router_battery_poll();
        if (REG16(APP_STATE + 0x166) == 0x11) {
            REG16(APP_STATE + 0x166) = 0;
            router_slow_housekeeping();
        }
    }
}
