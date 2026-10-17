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
 * the selection index -> per-item setup + list-window scroller
 * 0x0801715C(4, 0x1E) (keeps the selection visible: start = min(end,sel),
 * end = selection) -> ui_state_set(screen); protocol handlers set
 * the RF states 'g'-'j'; guard paths reset to 0. ~150 setter sites,
 * 116 menu-tree edges extracted (docs/MENU_TREE.md, session 31).
 *
 * BEACON WINDOWS ('h'/'i'/'j', 0x08010302): pass counter
 * [0x2000469C+0x135] compared against 0x29 (41) for BOTH 'h' and 'i'
 * (cmp sites 0x0801033C/0x08010392; session 43 re-check) and against
 * 0x33 (51) for 'j' (0x080103F0/0x08010450), 'j' selecting a 1-bit
 * submode (bit 6, ubfx 0x080103D8); completion clears the 50-byte
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
#include "eeprom.h"
#include "display.h"

#define REG8(a)   (*(volatile uint8_t *)(a))
#define REG16(a)  (*(volatile uint16_t *)(a))
#define REG32(a)  (*(volatile uint32_t *)(a))

/* --- extern hooks (SCAFFOLD; wired in main.c/rtos.c glue) ------------- */
void hw_init(void);
void *mem_copy(void *dst, const void *src, unsigned int n);
void mem_fill(void *dst, unsigned int n, unsigned int v);
int mem_cmp(const void *a, const void *b, unsigned int n);
void time_dly(uint32_t ticks);
int  battery_adc_read_ch14(void);
void ring_write_bcd(uint32_t peak_ts, uint8_t *ring);
void screen_init(uint8_t a, uint8_t b);            /* 0x0801715C */
uint16_t rf_read_reg(uint8_t chip, uint8_t reg);   /* rf.c: 0x0802292E */
/* [session 52] The task-creator/router helpers are recovered SHAPES: the
 * creators' bodies live in main()'s task setup (the recon consolidates
 * them there), the routers = named stubs pending the state-machine
 * recovery (sessions 18-30 mapped the transitions they drive). */
uint32_t ui_tasks_create_1(void) { return 0u; }
void ui_tasks_create_2(void) { }
void os_wait_30_ticks(void) { time_dly(30); }
void router_protocol_bridge(void) { }        /* 0x0800FE58 */
void router_state_progress(void) { }         /* 0x08010302 */
void router_battery_poll(void) { }           /* 0x08015DCC */
void router_slow_housekeeping(void) { }      /* every 17th pass */

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

/* ---- boot scene [0x0801B738 | RECOVERED session 52] ------------------------
 * The original's main task (FUN_0802D178) runs, in order: the peripheral
 * init (hw_init here), the task creators (0x0801B5A8), then this scene:
 *   1. zero the state structs (0x169/0x15B bytes)      [DATA-DERIVED here]
 *   2. ee_read(0x0A00, settings, 0x4D)                 [0x0801A6D8]
 *   3. FUN_0801A700 = a no-op (decompiled)
 *   4. the magic check: app-state +0x44 vs the ROM word at 0x0804594C -
 *      the vendor flash BEYOND the distributed image; unmapped on the
 *      simulator, so the check fails and the factory-reset path runs,
 *      exactly as observed in the original's stream
 *   4a. [0x0801B2E8(0)] draw "RESET WAIT..." (13 cells, scheme 4 =
 *       fg 0x0000 on bg 0xEFBE, class 3), write the default fields,
 *       ee_read(0x0FD0, 16) + the first-boot conditional write (skipped
 *       on the sim: the model returns 0x00, not 0xFF), ee_save
 *       = eeprom_write(0x0A00, 0x4D)
 *   5. led_write(app[0x19]*0x10 + 0x5F) = 0x145 after the reset
 *                                                         [0x08024CF8]
 *   6. fill(0,0,160,128) = the 0xEFBE background         [0x0801CD32]
 *   7. the splash blit 128x160 @0x08004130, 0xA000 bytes [0x0801D334]
 *   8. the callsign draw at (0x48,0x2A)                  [0x0801CB60] -
 *      on the sim the EEPROM reads return 0x00 so the string is empty
 *      and no glyphs are emitted
 *   9. delay(2000) - the splash hold                     [0x0801BAA4]
 *  10. the VFO/config EEPROM reads: 0x0FCD(2), 0x0FCF(1), 0x0FB0(16),
 *      0x0F90(16), 0x1FCD(2), 0x1FCF(1), 0x1FB0(16), 0x1F90(16)
 * Step 8 emits no bus events under the simulator; the rest is
 * reproduced exactly. */
extern const uint8_t ui_splash_image[0xA000];
extern const uint8_t ui_ascii8x16_font[];   /* 95 chars x 16 B @0x080413CC */

/* [0x0801B2E8(0) | RECOVERED session 52] factory reset: the boot text
 * (scheme 4 = black on 0xEFBE, class 3 = the 8x16 table), the default
 * field list (the offsets/values decompiled), and the EEPROM tail:
 * the 0x0FD0 block read with the first-boot conditional write, then
 * the settings save (FUN_0801A6EA). The VFO-ring init touches RAM
 * only. */
static void ui_factory_reset(uint8_t *st)
{
    static const char msg[13] = "RESET WAIT...";
    uint8_t blk[16];

    for (uint32_t i = 0; i < 13u; i++) {
        const uint8_t *g = &ui_ascii8x16_font[(uint32_t)(uint8_t)msg[i] * 16u - 0x200u];
        display_blit_glyph_cell(0x30u, (uint8_t)(0x20u + i * 8u), g, 0x0000u, 0xEFBEu);
    }
    mem_fill(st, 0x4D, 0x00);
    mem_copy(&st[0x44], (const void *)0x0804594Cu, 4);   /* the magic word */
    mem_copy(&st[0x30], (const void *)0x08045954u, 6);
    st[0x42] = 1;  st[0x41] = st[0x42];
    st[0x0A] = 2;  st[0x0B] = 0;
    st[0x06] = 5;  st[0x05] = st[0x06];
    st[0x19] = 9;  st[0x0C] = 1;  st[0x29] = 1;  st[0x15] = 0;
    st[0x01] = 0;  st[0x1A] = 6;  st[0x16] = 1;  st[0x17] = 1;
    st[0x1D] = 2;  st[0x1F] = 1;  st[0x20] = 2;  st[0x21] = 4;
    st[0x22] = 2;  st[0x23] = 0;  st[0x24] = 1;  st[0x25] = 2;
    st[0x26] = 1;  st[0x2C] = 1;  st[0x39] = 1;
    st[0x09] = 0;  st[0x08] = st[0x09];
    eeprom_read(0x0FD0u, 0x10u, blk);
    if (blk[0] == 0xFFu) {                    /* first boot only        */
        mem_copy(blk, (const void *)0x08042C48u, 0x10);
        eeprom_write(0x0FD0u, 0x10u, blk);
    }
    eeprom_write(0x0A00u, 0x4Du, st);         /* FUN_0801A6EA the save  */
}

void ui_boot_scene(void)
{
    static uint8_t ee_settings[0x4D];
    static uint8_t ee_buf[16];

    eeprom_read(0x0A00u, 0x4Du, ee_settings);
    if (mem_cmp(&ee_settings[0x44], (const void *)0x0804594Cu, 4) != 0)
        ui_factory_reset(ee_settings);
    display_led_write((uint16_t)(ee_settings[0x19] * 0x10u + 0x5Fu), 0u);
    display_fill_rect(0u, 0u, 0xA0u, 0x80u, 0u);
    display_blit_image(0u, 0u, 0x80u, 0xA0u, ui_splash_image, 0xA000u);
    {
        uint8_t callsign[9];
        eeprom_read(0x1800u, 9u, callsign);
        if (callsign[0] == 0xFFu)
            mem_copy(callsign, "NOCALL-7", 8u);   /* DAT_0801B9B0 [RECOVERED] */
        /* draw_text(0x48, 0x2A, callsign, 0, 3): empty under the sim     */
    }
    {
        volatile uint32_t i;               /* delay(2000) [0x0801BAA4 shape]   */
        for (uint32_t n = 0; n < 2000u; n++)
            for (i = 0; i < 7000u; i++)
                ;
    }
    eeprom_read(0x0FCDu, 2u, ee_buf);
    eeprom_read(0x0FCFu, 1u, ee_buf);
    eeprom_read(0x0FB0u, 16u, ee_buf);
    eeprom_read(0x0F90u, 16u, ee_buf);
    eeprom_read(0x1FCDu, 2u, ee_buf);
    eeprom_read(0x1FCFu, 1u, ee_buf);
    eeprom_read(0x1FB0u, 16u, ee_buf);
    eeprom_read(0x1F90u, 16u, ee_buf);
}

void ui_task(void *p_arg)
{
    (void)p_arg;
    hw_init();
    if (ui_tasks_create_1() == 0xFFFFFFFFu)
        return;
    ui_tasks_create_2();
    ui_boot_scene();      /* [0x0802D178 order: the periph init, the task
                           * creators, then the boot scene] */

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
