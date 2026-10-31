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
 * S-METER (bar-value integrator; corrected session 58 pt3): rf_read(chip,
 * 0x1B) >> 8 = RSSI dB (AT1846S 0x1B[15:8], guide-confirmed); updater
 * 0x080203d0 keeps the peak vs [+0x40]; every 5th pass (sub-counter +0x3F):
 * ring write 0x0801A8DA, shift +4 (0x0801D8C0), snapshot 0x1E, and the 3
 * most recent entries render as bars (0x0801AB36 / 0x0801A7AA / 0x0801E296).
 * Ring entries = BCD-packed BAR-VALUE accumulator [ctx+0x3B] ("timestamp"
 * was the session-26 misread): the integrator 0x080201AA(mode) does
 * mode 11: bar += 100000 (the squelch-open flag) or += the per-band step
 * [ctx+0x2E+band*2]; mode 12: bar += 0xfffe7960 (-100000); clamped per band
 * to [ctx+0x62+band*4]=0 .. [ctx+0x5A+band*4]=99500 (LIVE boot values).
 * THE STEP TABLE = 0x0801F1C4: the config byte {0..6} -> {500, 625, 1000,
 * 1250, 2500, 5000, 10000} (the bar resolution; the boot default = 500).
 * The mode scheduler ([ctx+0x76] = 11/12) is dormant on the model (the
 * boot state never exits) — hence the display = RSSI-inert there.
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
#include "hwregs.h"
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
 * pending conversions reported as -1 (original: 1/2 handshake). */int battery_level_read(void)
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
 * field list (the offsets/values decompiled), and the EEPROM tail.
 * [session 55, from the @ODR#3,545 capture op map]: the reset path
 * first writes the 32-byte VFO/default block at 0x0800 (one byte per
 * transaction, 53 events each), THEN the 0x0FD0 block read with the
 * first-boot conditional write, then the settings save
 * (FUN_0801A6EA). The VFO-ring init touches RAM only.] */
static const uint8_t ui_ee_vfo_defaults[32] = {   /* [DATA-DERIVED session 55] */
    0x00, 0x25, 0x61, 0x13, 0x00, 0x25, 0x61, 0x13,
    0xFF, 0xFF, 0xFF, 0xFF, 0x01, 0x00, 0x00, 0x00,
    0x00, 0x25, 0x01, 0x40, 0x00, 0x25, 0x01, 0x40,
    0xFF, 0xFF, 0xFF, 0xFF, 0x01, 0x00, 0x00, 0x00
};

static void ui_factory_reset(uint8_t *st)
{
    static const char msg[13] = "RESET WAIT...";
    uint8_t blk[16];
    /* [session 63 | 0x0801B2E8 disassembly] the reset draws the text
     * FIRST (scheme 4), before zeroing the 0x4D context. The scheme-4
     * pair = (fg 0x0000, bg 0xEFBE), SWAPPED when the app blank flag
     * [SETTINGS_CTX+0x18] is set — the same ink-swap bit display.c's
     * draw_text_cls models. On the fresh EE the ee_load (directly into
     * the context, 0x0801a6d8) leaves 0xFF at +0x18, so the fresh-boot
     * cells = glyph pixels 0xEFBE on 0x0000 background: p3k6_orig
     * cell 2882..3137 = 84x 0x0000 + 44x 0xEFBE (the inverted pair vs
     * the session-52 run whose image carried +0x18=0x00). The boot fill
     * (0x0801CD32) then sees +0x18=0 after the memset and emits 0xEFBE
     * (p3k6_orig 13082..54040). */
    uint16_t fg4 = 0x0000u, bg4 = 0xEFBEu;
    if ((*(volatile uint8_t *)(SETTINGS_CTX + 0x18u)) != 0u) {
        fg4 = 0xEFBEu;
        bg4 = 0x0000u;
    }

    for (uint32_t i = 0; i < 13u; i++) {
        const uint8_t *g = &ui_ascii8x16_font[(uint32_t)(uint8_t)msg[i] * 16u - 0x200u];
        display_blit_glyph_cell(0x30u, (uint8_t)(0x20u + i * 8u), g, fg4, bg4);
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
    eeprom_write(0x0800u, 0x20u, ui_ee_vfo_defaults);    /* [session 55] */
    eeprom_read(0x0FD0u, 0x10u, blk);
    if (blk[0] == 0xFFu) {                    /* first boot only        */
        mem_copy(blk, (const void *)0x08042C48u, 0x10);
        eeprom_write(0x0FD0u, 0x10u, blk);
    }
    eeprom_write(0x0A00u, 0x4Du, st);         /* FUN_0801A6EA the save  */
}

uint8_t ui_callsign[9];         /* [session 62] the 0x1800 callsign block,
                                 * read once at the boot scene; the
                                 * screen0 build draws it after "CALL:"
                                 * with the CB60 BCD bias (the fresh-EE
                                 * 0x00 bytes render as eight '0' cells —
                                 * p3k6_orig.stream 95094..100969 = the 13
                                 * + 9 cell rows, 22 x 267 events) */
uint8_t ui_ee_warm;             /* [session 62] 1 = the boot found the
                                 * valid settings magic (the warm path):
                                 * the LED/0x0FA0 burst (ee359) runs on
                                 * the warm boot only — the fresh-EE
                                 * stream shows no such era in 6 s. */

void ui_boot_scene(void)
{
    static uint8_t ee_buf[16];

    /* [session 63 | RESTRUCTURED to the confirmed original wiring] the
     * ee_load reads the 0x4D settings image STRAIGHT INTO the radio
     * context at 0x20004E68 (RESEARCH_LOG session 51: "ee_load (0x4D
     * @0x0A00 -> 0x20004E68; the recon's inferred wiring CONFIRMED").
     * Consequence verified on p3k6_orig.stream: on the fresh EE the
     * context = all 0xFF, so the app blank flag [ctx+0x18] reads SET
     * during the factory-reset text (cells all-black) and only clears
     * when the reset's default fields are written into the context
     * before the boot fill (0xEFBE from seq 13082). The earlier
     * separate ui_ee_settings buffer + post-reset copy had the flag
     * still at its power-on 0 during the cells (bg 0xEFBE) — the exact
     * first divergence of the fresh-EE CK4 pair. */
    eeprom_read(0x0A00u, 0x4Du, (void *)SETTINGS_CTX);
    if (mem_cmp((const void *)(SETTINGS_CTX + 0x44u), (const void *)0x0804594Cu, 4) != 0)
        ui_factory_reset((void *)SETTINGS_CTX);
    else
        ui_ee_warm = 1u;   /* the valid-magic (warm) boot path        */
    /* [session 61 | RECOVERED] the settings image lands 1:1 in the radio
     * context at 0x20004E68 (the ee-marker experiment p2-eemarker.resc:
     * the ee bytes 2/5 appear at the ctx +2/+5; the dwell args =
     * the ctx +26/+27; the scan-starter gate = the ctx +31; the S-meter
     * step selectors = the ctx +2+idx). The head's $A00 frame is NOT
     * applied to the ctx (p2-marker.resc: a distinguishable-settings
     * $A00 leaves the ctx at the defaults - the frame lives in the
     * head's rxBuf mirror at 0x20003A84 only). */

    /* [session 61 | 0x0801F1C4 RECOVERED] the S-meter step durations for
     * both VFOs: the ctx byte (2+idx) = the step index, mapped
     * {0:500, 1:625, 2:1000, 3:1250, 4:2500, 5:5000, 6:10000} (else
     * passthrough), stored as the u16 at SM_STEP_BASE+idx*2. The observed
     * boot fills both entries (the steps[0]/[1] pair); the mode-10
     * handler 0x0801680C refreshes one per pass live. */
    {
        static const uint16_t sm_tbl[7] = {500u, 625u, 1000u, 1250u, 2500u, 5000u, 10000u};
        uint8_t i;
        for (i = 0u; i < 2u; i++) {
            uint8_t sel = *(const volatile uint8_t *)(SETTINGS_CTX + 2u + i);
            uint16_t d = (sel < 7u) ? sm_tbl[sel] : (uint16_t)sel;
            REG16(SM_STEP_BASE + (uint32_t)i * 2u) = d;
        }
    }

    {   /* [session 55 correction] step 5 = 0x08024CF8 -> 0x0802BC28(0, v):
         * the RF register-SHADOW update (0x0802D058 = shadow[8] = v,
         * RAM-only) — the original emits NO bus traffic here. The
         * earlier wiring to display_led_write emitted a spurious 12-bit
         * PB9 word (0xEF) the original does not emit. */
        static uint16_t rf_boot_shadow[16];
        rf_boot_shadow[8] = (uint16_t)(REG8(SETTINGS_CTX + 0x19u) * 0x10u + 0x5Fu);
    }
    display_fill_rect(0u, 0u, 0xA0u, 0x80u, 0xEFBEu);
    display_blit_image(0u, 0u, 0x80u, 0xA0u, ui_splash_image, 0xA000u);
    /* [session 62] the callsign draw (the boot scene step 8): the 9-byte
     * 0x1800 read lands in ui_callsign; the screen0 build renders it
     * after the "CALL:" label with the CB60 BCD bias (0x00 -> '0'). The
     * original's fresh stream draws 8 value cells (the 9th byte = the
     * terminator slot); the draw happens in the screen0 label phase,
     * immediately before the scan era. */
    eeprom_read(0x1800u, 9u, ui_callsign);
    if (ui_callsign[0] == 0xFFu)
        mem_copy(ui_callsign, "NOCALL-7", 8u);   /* DAT_0801B9B0 [RECOVERED] */
    {
        volatile uint32_t i;               /* delay(2000) [0x0801BAA4 shape]   */
        for (uint32_t n = 0; n < 2000u; n++)
            for (i = 0; i < 7000u; i++)
                ;
    }
    /* [session 55] the 8 VFO/config reads MOVED to ui_screen0_build's
     * scan tail: the original's capture shows them INSIDE the scan
     * burst (ops 208-215 @ODR#89,225), not at the end of the boot
     * scene as previously placed. */
}

/* ---- main screen build (status page) ---------------------------------------
 * [RECOVERED structure, session 52 pt6: the exact window sequence observed
 * in the original's stream right after the boot scene. The strings are the
 * sim-state: the callsign value is empty (the EEPROM model returns 0x00) so
 * only the "CALL:" label renders, and the battery ADC reads 0 which the
 * original formats as "0.8 V" — reproduced literally here, DATA-DERIVED.]
 * 1. the glyph grid: "CALL:" @ (x=0x2A, y=0x48), "BAT:0.8 V" @ (x=0x2A,
 *    y=0x58), the scheme 4 (fg 0x0000 on bg 0xEFBE)
 * 2. the content fill: (x=1, y=18) 158x92 0xEFBE
 * 3. the black lines: the rows 63, 110 (160x1)
 * 4. the header band: (x=1, y=1) 158x16 0xEFBE
 * 5. the 1-px border frame: the row 0, the col 0 (127), the col 159 (128),
 *    the row 127 (159) in 0x0000
 * 6. the row 17 line + the rows 63/110 re-lines
 * 7. the header text fill: (x=4, y=1) 72x16 0xEFBE
 * The small-font elements and the periodic updates = OPEN (the class-0
 * path and the state-machine cadence). */

/* the glyph-cell helpers for the status page: the class tables and the
 * scheme 4 colors (fg 0x0000 on bg 0xEFBE). */
extern const uint8_t ui_ascii6x10_font[];
extern const uint8_t ui_ascii12x20_font[];

static void cell6x10(uint8_t y, uint8_t x, uint8_t ch)
{
    display_blit_glyph_cell_6x10(y, x, &ui_ascii6x10_font[(uint32_t)(ch - 0x20u) * 12u],
                                 0x0000u, 0xEFBEu);
}
static void cell8x16(uint8_t y, uint8_t x, const void *g)
{
    display_blit_glyph_cell(y, x, (const uint8_t *)g, 0x0000u, 0xEFBEu);
}
static void cell8x16_ch(uint8_t y, uint8_t x, uint8_t ch)
{
    display_blit_glyph_cell(y, x, &ui_ascii8x16_font[(uint32_t)(ch - 0x20u) * 16u],
                            0x0000u, 0xEFBEu);
}
static void cell12x20(uint8_t y, uint8_t x, uint8_t ch)
{
    display_blit_glyph_cell_12x20(y, x, &ui_ascii12x20_font[(uint32_t)(ch - 0x20u) * 36u],
                                  0x0000u, 0xEFBEu);
}
/* [session 62] the boot EE scan (the settings save + the VFO init +
 * the 128-channel validity scan + the 8 VFO/config reads). Runs once,
 * at the first screen0 build — see ui_screen0_build. */
static void ui_screen0_scan(void)
{
    /* [0x0801FDDC | RECOVERED session 53; session 62 frame-decode]
     * 1. three 16-byte block reads: 0x0FD0, 0x0800, 0x0810 (the
     *    validation-range / VFO blocks),
     * 2. the settings save: 77 one-byte transactions @0x0A00..0x0A4C
     *    (eeprom_write(0x0A00, 0x4D)),
     * 3. the channel-1 read @0x002, then the VFO-A init: the 16-byte
     *    default block @0x0000 (00 25 01 40 ... = 440.1250 MHz BCD)
     *    and the settings save AGAIN (unchanged data),
     * 4. the channel-2 read @0x012, then the VFO-B init: the 16-byte
     *    default block @0x0010 (00 25 61 13 ... = 136.1250 MHz BCD)
     *    and the settings save with the channel pointer bytes
     *    0x41/0x42 = 02,
     * 5. the remaining 126 two-byte channel reads @0x02 + 0x10*k,
     * 6. the 8 VFO/config reads (0x0FCD(2), 0x0FCF(1), 0x0FB0(16),
     *    0x0F90(16), 0x1FCD(2), 0x1FCF(1), 0x1FB0(16), 0x1F90(16)).
     * [session 62 tx-decode of p3k6_orig.stream 100969..123811: the
     * exact write-embedded structure W[77] R0002 W[16] W[77] R0012
     * W[16] W[77] R0022..R07F2; the recon's flat W[77]+R[128] shape is
     * RETRACTED.] */
    {
        /* [DATA-DERIVED session 62] the VFO-init default blocks,
         * transcribed from the original's write bursts; byte 12..13 =
         * 00 02 (the factory-reset 0x0800 block uses 01 00 00 00) */
        static const uint8_t vfo_a_init[16] = {
            0x00, 0x25, 0x01, 0x40, 0x00, 0x25, 0x01, 0x40,
            0xFF, 0xFF, 0xFF, 0xFF, 0x00, 0x02, 0x00, 0x00
        };
        static const uint8_t vfo_b_init[16] = {
            0x00, 0x25, 0x61, 0x13, 0x00, 0x25, 0x61, 0x13,
            0xFF, 0xFF, 0xFF, 0xFF, 0x00, 0x02, 0x00, 0x00
        };
        uint8_t blk[16];
        uint8_t scanbuf[2];

        eeprom_read(0x0FD0u, 0x10u, blk);
        eeprom_read(0x0800u, 0x10u, blk);
        eeprom_read(0x0810u, 0x10u, blk);
        eeprom_write(0x0A00u, 0x4Du, (void *)SETTINGS_CTX);
        eeprom_read(0x0002u, 2u, scanbuf);                    /* ch 1  */
        eeprom_write(0x0000u, 0x10u, vfo_a_init);             /* VFO-A */
        eeprom_write(0x0A00u, 0x4Du, (void *)SETTINGS_CTX);
        eeprom_read(0x0012u, 2u, scanbuf);                    /* ch 2  */
        eeprom_write(0x0010u, 0x10u, vfo_b_init);             /* VFO-B */
        REG8(SETTINGS_CTX + 0x41u) = 2u;                      /* the   */
        REG8(SETTINGS_CTX + 0x42u) = 2u;                      /* ptr   */
        eeprom_write(0x0A00u, 0x4Du, (void *)SETTINGS_CTX);
        for (uint32_t ch = 3u; ch < 0x81u; ch++)
            eeprom_read((uint16_t)(ch * 0x10u - 0xEu), 2u, scanbuf);
        eeprom_read(0x0FCDu, 2u, blk);
        eeprom_read(0x0FCFu, 1u, blk);
        eeprom_read(0x0FB0u, 0x10u, blk);
        eeprom_read(0x0F90u, 0x10u, blk);
        eeprom_read(0x1FCDu, 2u, blk);
        eeprom_read(0x1FCFu, 1u, blk);
        eeprom_read(0x1FB0u, 0x10u, blk);
        eeprom_read(0x1F90u, 0x10u, blk);
    }
}

static void ui_screen0_build(void)
{
    static const char line1[5] = "CALL:";
    static const char line2[9] = "BAT:0.8 V";

    /* [session 62] the header rows: "CALL:" + the 8 callsign value
     * cells (the CB60 BCD bias: the fresh-EE 0x00 bytes = '0') at
     * y=0x48, then "BAT:0.8 V" at y=0x58 — the original's pre-scan gap
     * = 13 + 9 = 22 cells of 267 events (p3k6_orig.stream
     * 95094..100969). The EE scan block runs ONCE after them. */
    for (uint32_t i = 0; i < 5u; i++)
        display_blit_glyph_cell(0x48u, (uint8_t)(0x2Au + i * 8u),
                                &ui_ascii8x16_font[(uint32_t)(uint8_t)line1[i] * 16u - 0x200u],
                                0x0000u, 0xEFBEu);
    for (uint32_t i = 0; i < 8u; i++) {
        uint8_t ch = ui_callsign[i];
        if (ch < 10u)
            ch = (uint8_t)(ch + 0x30u);          /* the CB60 BCD bias    */
        if (ch == 0u)
            break;                               /* the terminator slot  */
        display_blit_glyph_cell(0x48u, (uint8_t)(0x2Au + (5u + i) * 8u),
                                &ui_ascii8x16_font[(uint32_t)ch * 16u - 0x200u],
                                0x0000u, 0xEFBEu);
    }
    for (uint32_t i = 0; i < 9u; i++)
        display_blit_glyph_cell(0x58u, (uint8_t)(0x2Au + i * 8u),
                                &ui_ascii8x16_font[(uint32_t)(uint8_t)line2[i] * 16u - 0x200u],
                                0x0000u, 0xEFBEu);
    {
        static uint8_t scanned;
        if (scanned == 0u) {
            scanned = 1u;
            ui_screen0_scan();
        }
    }

    display_fill_rect(18u, 1u, 158u, 92u, 0xEFBEu);
    {   /* [session 55] the 2x read16 burst @ODR#118,308 (0x0800 + 0x0810,
         * the VFO A/B blocks) right after the content fill. */
        uint8_t blk[16];
        eeprom_read(0x0800u, 0x10u, blk);
        eeprom_read(0x0810u, 0x10u, blk);
    }
    display_fill_rect(63u, 0u, 160u, 1u, 0x0000u);
    display_fill_rect(110u, 0u, 160u, 1u, 0x0000u);
    display_fill_rect(1u, 1u, 158u, 16u, 0xEFBEu);
    display_fill_rect(0u, 0u, 160u, 1u, 0x0000u);
    display_fill_rect(0u, 0u, 1u, 127u, 0x0000u);
    display_fill_rect(0u, 159u, 1u, 128u, 0x0000u);
    display_fill_rect(127u, 0u, 159u, 1u, 0x0000u);
    display_fill_rect(17u, 0u, 160u, 1u, 0x0000u);
    display_fill_rect(63u, 0u, 159u, 1u, 0x0000u);
    display_fill_rect(110u, 0u, 160u, 1u, 0x0000u);
    display_fill_rect(1u, 4u, 72u, 16u, 0xEFBEu);
    /* [session 52 pt8, DATA-DERIVED] the observed tail: a space cell in
     * the header, the 16x16 fills at the header right and the bottom
     * row (the 0xEFBE), then the four solid-black 16x16 status boxes
     * at the bottom row. */
    display_blit_glyph_cell(1u, 108u, ui_ascii8x16_font, 0x0000u, 0xEFBEu);
    /* [session 52 pt8] the battery icon: a custom glyph (not the ASCII
     * table) rendered in scheme 5 (fg 0x13A2 on bg 0xEFBE). The 16-byte
     * record below = the bitmap extracted from the original's stream
     * (the column-major class-3 layout). DATA-DERIVED. */
    {
        /* the record = the 16 column-pairs (the LSB = the row 0), the
         * bitmap extracted from the original's stream */
        static const uint8_t batt_icon[32] = {
            0xF8, 0x07, 0xF8, 0x07, 0xF8, 0x07, 0xF8, 0x07,
            0xF8, 0x07, 0xF8, 0x07, 0xF8, 0x07, 0xF8, 0x07,
            0xF8, 0x07, 0xF8, 0x07, 0xF8, 0x07, 0xF8, 0x07,
            0xF8, 0x07, 0xF8, 0x07, 0xE0, 0x01, 0xE0, 0x01
        };
        display_blit_icon_16x16(1u, 139u, batt_icon, 0x13A2u, 0xEFBEu);
    }
    /* [session 52 pt10 | DATA-DERIVED transcription] the exact ordered
     * window sequence from the original's stream: the status labels
     * (class 1), the VFO frequency "000.000" (class 6, the sim-state
     * VFO = 0), the "00" sub-displays (class 3), the left-arrow cursor
     * glyph, and the periodic re-draws of the same elements. */
    /* (the battery icon call is above) */
    display_fill_rect(1u, 120u, 16u, 16u, 0xEFBE);;
    display_fill_rect(111u, 72u, 16u, 16u, 0xEFBE);;
    display_fill_rect(111u, 30u, 120u, 16u, 0xEFBE);;
    display_fill_rect(111u, 1u, 5u, 16u, 0xEFBE);;
    display_fill_rect(111u, 157u, 2u, 16u, 0xEFBE);;
    display_fill_rect(111u, 6u, 16u, 16u, 0x0000);;
    display_fill_rect(111u, 22u, 16u, 16u, 0x0000);;
    display_fill_rect(111u, 125u, 16u, 16u, 0x0000);;
    display_fill_rect(111u, 141u, 16u, 16u, 0x0000);;
    cell6x10(0x12u, 0x85u, 0x41u);
    cell6x10(0x12u, 0x8Bu, 0x50u);
    cell6x10(0x12u, 0x91u, 0x52u);
    cell6x10(0x12u, 0x97u, 0x53u);
    cell6x10(0x1Du, 0x88u, 0x20u);
    cell6x10(0x1Du, 0x8Eu, 0x20u);
    cell6x10(0x27u, 0x85u, 0x20u);
    cell6x10(0x27u, 0x8Bu, 0x20u);
    cell6x10(0x27u, 0x91u, 0x20u);
    {   /* [session 55] read16(0x0800) @ODR#137,739 — the VFO A block,
         * right before the first frequency row (0x1D) renders. */
        uint8_t blk[16];
        eeprom_read(0x0800u, 0x10u, blk);
    }
    cell12x20(0x1Du, 0x16u, 0x30u);
    cell12x20(0x1Du, 0x22u, 0x30u);
    cell12x20(0x1Du, 0x2Eu, 0x30u);
    cell12x20(0x1Du, 0x3Au, 0x2Eu);
    cell12x20(0x1Du, 0x46u, 0x30u);
    cell12x20(0x1Du, 0x52u, 0x30u);
    cell12x20(0x1Du, 0x5Eu, 0x30u);
    display_fill_rect(29u, 106u, 18u, 20u, 0xEFBE);;
    cell8x16_ch(0x23u, 0x6Cu, 0x30u);
    cell8x16_ch(0x23u, 0x74u, 0x30u);
    cell6x10(0x12u, 0x50u, 0x20u);
    cell6x10(0x12u, 0x5Cu, 0x20u);
    cell6x10(0x12u, 0x37u, 0x20u);
    cell6x10(0x12u, 0x1Eu, 0x20u);
    cell6x10(0x12u, 0x24u, 0x20u);
    cell6x10(0x12u, 0x2Au, 0x20u);
    cell6x10(0x12u, 0x44u, 0x57u);
    cell6x10(0x12u, 0x05u, 0x20u);
    cell6x10(0x12u, 0x0Bu, 0x20u);
    cell6x10(0x12u, 0x11u, 0x20u);
    cell6x10(0x32u, 0x16u, 0x4Cu);
    display_fill_rect(63u, 0u, 160u, 1u, 0x0000);;
    display_fill_rect(64u, 133u, 24u, 10u, 0xEFBE);;
    cell6x10(0x4Cu, 0x88u, 0x20u);
    cell6x10(0x4Cu, 0x8Eu, 0x20u);
    cell6x10(0x56u, 0x85u, 0x20u);
    cell6x10(0x56u, 0x8Bu, 0x20u);
    cell6x10(0x56u, 0x91u, 0x20u);
    {   /* [session 55] read16(0x0810) @ODR#145,359 — the VFO B block,
         * right before the second frequency row (0x4C) renders. */
        uint8_t blk[16];
        eeprom_read(0x0810u, 0x10u, blk);
    }
    cell12x20(0x4Cu, 0x16u, 0x30u);
    cell12x20(0x4Cu, 0x22u, 0x30u);
    cell12x20(0x4Cu, 0x2Eu, 0x30u);
    cell12x20(0x4Cu, 0x3Au, 0x2Eu);
    cell12x20(0x4Cu, 0x46u, 0x30u);
    cell12x20(0x4Cu, 0x52u, 0x30u);
    cell12x20(0x4Cu, 0x5Eu, 0x30u);
    display_fill_rect(76u, 106u, 18u, 20u, 0xEFBE);;
    cell8x16_ch(0x52u, 0x6Cu, 0x30u);
    cell8x16_ch(0x52u, 0x74u, 0x30u);
    cell6x10(0x40u, 0x50u, 0x20u);
    cell6x10(0x40u, 0x5Cu, 0x20u);
    cell6x10(0x40u, 0x37u, 0x20u);
    cell6x10(0x40u, 0x1Eu, 0x20u);
    cell6x10(0x40u, 0x24u, 0x20u);
    cell6x10(0x40u, 0x2Au, 0x20u);
    cell6x10(0x40u, 0x44u, 0x57u);
    cell6x10(0x40u, 0x05u, 0x20u);
    cell6x10(0x40u, 0x0Bu, 0x20u);
    cell6x10(0x40u, 0x11u, 0x20u);
    cell6x10(0x61u, 0x16u, 0x4Cu);
    {
        static const uint8_t left_arrow[16] = {   /* the '<' cursor glyph [DATA-DERIVED] */
            0x00, 0xFC, 0xF8, 0xF0, 0xE0, 0xC0, 0x80, 0x00,
            0x00, 0x1F, 0x0F, 0x07, 0x03, 0x01, 0x00, 0x00
        };
        cell8x16(0x1Fu, 0x0Du, left_arrow);
    }
    cell8x16_ch(0x4Eu, 0x0Du, 0x20u);
    display_led_stream_replay();   /* [DATA-DERIVED] the 4,458-event LED burst @ODR#152,036 */
    display_fill_rect(97u, 30u, 90u, 10u, 0xEFBE);;
    display_led_tail_replay();     /* [DATA-DERIVED] the 25-event burst @ODR#153,847 */
    cell12x20(0x1Du, 0x16u, 0x30u);
    cell12x20(0x1Du, 0x22u, 0x30u);
    cell12x20(0x1Du, 0x2Eu, 0x30u);
    cell12x20(0x1Du, 0x3Au, 0x2Eu);
    cell12x20(0x1Du, 0x46u, 0x30u);
    cell12x20(0x1Du, 0x52u, 0x30u);
    cell12x20(0x1Du, 0x5Eu, 0x30u);
    display_fill_rect(29u, 106u, 18u, 20u, 0xEFBE);;
    cell8x16_ch(0x23u, 0x6Cu, 0x30u);
    cell8x16_ch(0x23u, 0x74u, 0x30u);
    cell6x10(0x12u, 0x1Eu, 0x20u);
    cell6x10(0x12u, 0x24u, 0x20u);
    cell6x10(0x12u, 0x2Au, 0x20u);
    {   /* [session 55 + 62] the 359-event burst @ODR#158,942: the
         * original's ee_read(0x0FA0, 101) setup interleaved by the RTOS
         * with LED-serial words — replayed [DATA-DERIVED] (see
         * display.c). WARM PATH ONLY: the fresh-EE boot stream has no
         * such era (the LED/0x0FA0 task pair only runs once the
         * settings are valid), so the replay is gated on ui_ee_warm. */
        if (ui_ee_warm)
            display_ee359_replay();
    };
    {
        /* [session 52 pt10 | DATA-DERIVED] the RX-bar indicator row:
         * the 90x10 1-bpp bitmap extracted from the original's stream. */
        static const uint8_t rxbars[120] = {
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
    0x00, 0x00, 0x07, 0xEF, 0xC0, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
    0xFD, 0xFF, 0xF7, 0xEF, 0xC0, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
    0xFD, 0xFF, 0xF7, 0xEF, 0xC0, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
    0xFD, 0xFF, 0xF7, 0xEF, 0xC0, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,        };
        /* [session 53] the original has a separate 359-event I2C burst
         * right before this window (open item). */
        display_blit_bitmap_1bpp(50u, 30u, 90u, 10u, rxbars, 0x0000u, 0xEFBEu);
    };
}

/* ---- Round N (session 56): the keyscan chain [RECOVERED] ----------------
 * 0x080296CA adc_read(ch): RTOS-mutex-wrapped single ADC2 conversion —
 * EOC spin on SR&2 (the harness tags SR=0x2), then FOUR u16 DR reads,
 * averaged (sum/4). The mutex object (0x200050D4) is a no-op here (the
 * UI task is the only reader in the recon).
 * 0x080281A6 key_scan(): FIRST consumes the TIM2/PC7-PC8 accessory flags
 * (0x20005165==1 -> clear it, status bit, 0x20005160=1, return 1|2 by
 * 0x20005164 = PC7^PC8 — injects codes 11/12; inert on the model: the
 * Renode STM32_Timer resets CCR1=0xFFFF > ARR=0x7D so CC1IF never sets
 * and the sampler never runs — session-56 boundary evidence). Then the
 * three resistor ladders: ch14/group A (table 0x20001684), ch15/group B
 * (0x200016A4), ch12/group C (0x200016C4); entry = {u16 upper @+0,
 * u16 lower @+2}, scan gated to 662..3253 (A/B) resp. 672..1751 (C),
 * match = lower < adc < upper; on match: set the group flag, return the
 * index; total miss: clear all three flags, return 0xFF.
 * 0x080282FC key_get_code(): keypad-lock gate (bit0 of 0x20009458), then
 * scan + the (group,index)->code map:
 *   A: {10,11,12,13,1,2,3,14}  B: {4,5,6,0,7,8,9,15}  C: {18,19,20}.
 * The threshold tables live in the byte-verified .tables blob at the
 * original RAM addresses, so the C reads them in place. */
#define KEY_ACC_STATE   0x2000687Du  /* active-state byte (0x8019ef0)    */
#define ACC_STATUS      0x20009456u  /* accessory status (bit7 set)      */

static uint16_t key_adc_read(uint8_t ch)
{
    uint32_t sum;
    ADC2_SQR3 = ch;                      /* 0x0802C268(handle, ch, 1, 1) */
    while (!(ADC2_SR & ADC_SR_EOC))
        ;                                /* 0x0802C388 EOC spin          */
    sum  = (uint16_t)ADC2_DR;            /* 4x 0x0802C37A                */
    sum += (uint16_t)ADC2_DR;
    sum += (uint16_t)ADC2_DR;
    sum += (uint16_t)ADC2_DR;
    return (uint16_t)(sum / 4u);
}

static uint8_t key_scan(void)
{
    const uint16_t *tbl;
    uint16_t adc;
    uint8_t i;

    /* --- the TIM2/PC7-PC8 accessory branch [RECOVERED; inert on the model] */
    if (REG8(KEY_PC8_CHANGED) == 1u) {
        REG8(KEY_PC8_CHANGED) = 0u;
        if (REG8(KEY_ACC_STATE) != 0u)
            REG8(ACC_STATUS) |= 0x80u;
        REG8(KEY_VALID_A) = 1u;
        return (REG8(KEY_PC7XOR8) == 1u) ? 1u : 2u;
    }

    /* --- group A: ADC ch14, table 0x20001684 ---------------------------- */
    adc = key_adc_read(14u);
    REG16(KEY_ADC_CH14) = adc;
    if (adc >= 662u && adc < 3253u) {
        tbl = (const uint16_t *)KEY_LADDER_A;
        for (i = 0u; i < 8u; i++) {
            uint16_t upper = tbl[i * 2u];
            uint16_t lower = tbl[i * 2u + 1u];
            if (lower < adc && adc < upper) {
                REG8(KEY_VALID_A) = 1u;
                return i;
            }
        }
    }

    /* --- group B: ADC ch15, table 0x200016A4 ---------------------------- */
    adc = key_adc_read(15u);
    REG16(KEY_ADC_CH15) = adc;
    if (adc >= 662u && adc < 3253u) {
        tbl = (const uint16_t *)KEY_LADDER_B;
        for (i = 0u; i < 8u; i++) {
            uint16_t upper = tbl[i * 2u];
            uint16_t lower = tbl[i * 2u + 1u];
            if (lower < adc && adc < upper) {
                REG8(KEY_VALID_B) = 1u;
                return i;
            }
        }
    }

    /* --- group C: ADC ch12, table 0x200016C4 ---------------------------- */
    adc = key_adc_read(12u);
    REG16(KEY_ADC_CH12) = adc;
    if (adc >= 672u && adc < 1751u) {
        tbl = (const uint16_t *)KEY_LADDER_C;
        for (i = 0u; i < 3u; i++) {
            uint16_t upper = tbl[i * 2u];
            uint16_t lower = tbl[i * 2u + 1u];
            if (lower < adc && adc < upper) {
                REG8(KEY_VALID_C) = 1u;
                return i;
            }
        }
    }

    REG8(KEY_VALID_A) = 0u;
    REG8(KEY_VALID_B) = 0u;
    REG8(KEY_VALID_C) = 0u;
    return 0xFFu;
}

uint8_t key_get_code(void)
{
    static const uint8_t map_a[8] = {10u, 11u, 12u, 13u, 1u, 2u, 3u, 14u};
    static const uint8_t map_b[8] = {4u, 5u, 6u, 0u, 7u, 8u, 9u, 15u};
    static const uint8_t map_c[3] = {18u, 19u, 20u};
    uint8_t idx;

    if (REG8(KEYLOCK_BYTE) & 1u)         /* 0x80282fe keypad lock        */
        return 0xFFu;
    idx = key_scan();
    if (REG8(KEY_VALID_A) == 1u)
        return (idx < 8u) ? map_a[idx] : 0xFFu;
    if (REG8(KEY_VALID_B) == 1u)
        return (idx < 8u) ? map_b[idx] : 0xFFu;
    if (REG8(KEY_VALID_C) == 1u)
        return (idx < 3u) ? map_c[idx] : 0xFFu;
    return 0xFFu;
}

/* [session 61 | p3-screens slice 3 | RECOVERED via p3-rtrace.resc] the
 * menu screen (0x35) fill layer: the trace's 13 fill calls in order, the
 * color 0xEFBE (the original's FUN_0801CD32 fill pairs 0xEF/0xBE). The
 * recon's display_fill_rect(y, x, cols, rows, color) takes the original's
 * (p1, p2, p3, p4) directly (the window = the cols p1..p1+p3-1?? no:
 * 0x2A = the x-axis from (p2, p2+p3-1), 0x2B = the y-axis from
 * (p1, p1+p4-1); the recon's session-52-corrected mapping matches). The
 * text layer = the session-61 p3-text decode (the 5 settings-list rows +
 * the selection bar, below). An earlier r0-r3-only trace mislabeled the
 * ink/class args and invented "确定/取消 at 0x08045A44" — superseded. The
 * original re-renders periodically (the RSSI-task screen machine); the
 * recon renders once per press. */
void ui_menu_screen_build(void)
{
    ui_state_set(0x35u);
    display_fill_rect(18u, 1u, 158u, 92u, 0xEFBEu);
    display_fill_rect(1u, 1u, 158u, 16u, 0xEFBEu);
    display_fill_rect(1u, 4u, 72u, 16u, 0xEFBEu);
    /* text(1, 108, " ", 0, 4) — the class-4 text = pending */
    display_fill_rect(1u, 120u, 16u, 16u, 0xEFBEu);
    display_fill_rect(111u, 72u, 16u, 16u, 0xEFBEu);
    display_fill_rect(111u, 30u, 120u, 16u, 0xEFBEu);
    display_fill_rect(111u, 1u, 5u, 16u, 0xEFBEu);
    display_fill_rect(111u, 157u, 2u, 16u, 0xEFBEu);
    display_fill_rect(29u, 106u, 18u, 20u, 0xEFBEu);
    display_fill_rect(64u, 133u, 24u, 10u, 0xEFBEu);
    display_fill_rect(76u, 106u, 18u, 20u, 0xEFBEu);
    /* [session 61 | p3-text.resc] the text layer, drawn after all fills:
     * the 5 settings-list strings (the flash slots 0x08043B20 + 12*i:
     * "01.本机设置" "02.站点类型" "03.GPS 设置" "04.信号设置"
     * "05.信号类型") walked at (20 + 17*i, 3) with the ink 4, cls 3, inv 0.
     * The GB2312 tails emit nothing (the CJK font-fetch guard is off in
     * the platform model) but advance the cursor 16 px per pair. Then the
     * selection: the row 0 re-walked with the ink 1, inv 1 (the blue bar);
     * the original does this as a direct per-char loop, not through the
     * string walker (no cb60 entry in the trace). */
    {
        uint8_t i;

        for (i = 0u; i < 5u; i++) {
            display_draw_text_cls((uint8_t)(20u + 17u * i), 3u,
                                  (const uint8_t *)(0x08043B20u + 12u * (uint32_t)i),
                                  4u, 3u, 0u);
        }
        display_draw_text_cls(20u, 3u, (const uint8_t *)0x08043B20u, 1u, 3u, 1u);
    }
}

void ui_task(void *p_arg)
{
    (void)p_arg;
    static uint8_t ui_key_prev_edge = 0xFFu;   /* [session 61] the press-edge tracker */
    static uint16_t ui_key_hold;               /* [session 61] the hold-pass counter [r5+0x21] */
    hw_init();
    if (ui_tasks_create_1() == 0xFFFFFFFFu)
        return;
    ui_tasks_create_2();
    ui_boot_scene();      /* [0x0802D178 order: the periph init, the task
                           * creators, then the boot scene] */
    ui_screen0_build();   /* the status page: the first runtime pass */

    /* [session 61 | 0x802490e enter equivalent] the periodic session
     * manager's enter branch sets FLAG_BANK+2 bit2 (the RX/scan session,
     * the dispatch's bit2 gate at 0x8019f24) and bfi's bits4-5 =
     * channel_idx+1 (0x10 at boot, idx 0); byte bit1 is set by earlier
     * boot code, giving the observed 0x16. The PTT-poll exit below then
     * clears bit2 within the first passes (a real device idles with the
     * PTT released, so the cleared session = the normal idle state). */
    REG8(FLAG_BANK + 2u) = 0x16u;

    for (;;) {
        os_wait_30_ticks();
        /* [session 61 | 0x8024106 PTT handler + 0x80241a0 release path]
         * the original polls PA10 (GPIOA bit10, DAT_8024C20 = 0x40010800,
         * the 0x400 mask at 0x8024110) periodically; a released PTT (bit10
         * = 1) reaches 0x80241a0 -> 0x8024606 -> the session exit
         * 0x8024a1a. The exit clears bits4-5 (0x8024a6a, &= 0xcf) and
         * bit2 (0x8024c16, &= 0xfb); bit1 survives, leaving the observed
         * idle byte 0x02. The harness bridge drives PA10 high (the
         * released idle) so this matches the original's trajectory. */
        if ((GPIO_IDR(GPIOA_BASE) & 0x400u) != 0u) {
            REG8(FLAG_BANK + 2u) &= 0xCBu;   /* the exit: 0xcf then 0xfb */
        }
        /* [0x08019DCC | RECOVERED slice] the key poll: every pass decodes
         * the keypad into 0x2000515E; 0xFF -> the release-ageing tail
         * ([SCAFFOLD] 0x801a20a — touches no key-cluster byte); a real
         * key passes the flag gates into the state-gated dispatch. The
         * gate byte FLAG_BANK+2 bit2 = 1 from boot in the original (the
         * firmware re-asserts it within 10 ms — session 56), which
         * swallows every non-side key there; the recon reproduces the
         * gate read and the main-mode RAM prefix only — the pump/vtable
         * dispatch itself is the Round B/C machinery. */
        {
            uint8_t key = key_get_code();
            REG8(KEY_CODE_BYTE) = key;
            /* [session 61] the original renders on the press EDGE: a held
             * key produces repeat events (the per-key timestamps at
             * 0x0801b2a0) but the screen repaint happens once per action;
             * re-rendering every pass diverges (the recon poured 4.9M
             * stream lines vs the original's +58,034 for one key-1
             * press). Edge-trigger the action on the key change. */
            if (key != ui_key_prev_edge) {
                ui_key_prev_edge = key;
                ui_key_hold = 0;   /* [0x8019DE2] the per-press resets */
                if (key == 0xFFu) {
                    /* [session 61 | RECOVERED] the menu-key (the code 10)
                     * release: the observed sequence scr 0x35 -> 0x00 with
                     * the idle re-render (+65,223 lines, p3-key6.log); the
                     * release handler = the vtable[10].release
                     * (0x08018F1A). */
                    if (REG8(UI_STATE_ADDR) == 0x35u) {
                        ui_state_set(0x00u);
                        ui_screen0_build();
                    }
                } else if (((REG8(FLAG_BANK + 2u) >> 2u) & 1u) == 0u) {
                        REG8(FLAG_BANK + 6u) &= 0x7Fu;   /* 0x8019fea */
                        REG8(KEY_CODE_COPY) = key;       /* 0x8019ff2 */
                        /* [session 61 | 0x08022CB4 RECOVERED] the key-13
                         * executor: the base = the CONTEXT (0x20004E68).
                         * idx = ctx[0x40] (the EE byte 64 = 0); the state
                         * bytes = ctx[8+idx]/ctx[10+idx]. The press cycle:
                         * OFF -> ON (1,0); ON counts ctx[10+idx] 1..2; at
                         * >2 the reset to (0,0) iff the 6-B ID at ctx+0x30
                         * equals the ROM default "000000" (0x08045A74),
                         * else clamp to 2. The clean-mode settle saves the
                         * settings (0x0801A6EA). The icon draw + the mode
                         * field (0x2000509D bits 3-5) = the phase-3 UI
                         * scope; the RAM state + the save = the p2
                         * observable (the original p2k13c.log: (1,0),
                         * (1,1), (1,2), (0,0)). */
                        if (key == 13u) {
                            volatile uint8_t *ctx = (volatile uint8_t *)SETTINGS_CTX;
                            uint8_t slot = ctx[0x40];
                            if (ctx[8u + slot] == 0u) {
                                ctx[8u + slot] = 1u;
                                ctx[10u + slot] = 0u;
                            } else {
                                ctx[10u + slot] += 1u;
                                if (ctx[10u + slot] > 2u) {
                                    static const uint8_t id_def[6] = {'0', '0', '0', '0', '0', '0'};
                                    if (mem_cmp((const void *)(SETTINGS_CTX + 0x30u), id_def, 6) == 0) {
                                        ctx[10u + slot] = 0u;
                                        ctx[8u + slot] = 0u;
                                    } else {
                                        ctx[10u + slot] = 2u;
                                    }
                                }
                            }
                            eeprom_write(0x0A00u, 0x4Du, (void *)SETTINGS_CTX);
                        } else if (key == 10u) {
                            /* [session 61 | RECOVERED] the menu key (the
                             * code 10 = the A-ladder entry 0; the tag 2300
                             * in p3-key6.log): the press = the screen 0x35
                             * render. The press handler = the
                             * vtable[10].press (0x08018ED4) only clears the
                             * executor state; the render comes from the
                             * periodic screen machine - the recon renders
                             * it directly at the press.
                             * [p3-screens slice 3 | p3-rtrace.resc spec]
                             * The fill layer = the trace's 13 fill calls,
                             * byte-exact via the CK4-exact primitives (the
                             * recon's fill_rect(y, x, cols, rows) = the
                             * original's (p1, p2, p3, p4) in order). The
                             * text layer (the ASCII 8-px / the CJK 16-px
                             * class-4 draw 0x0801CB60) = the next decode;
                             * the glyphs here = the stand-in. */
                            ui_menu_screen_build();
                        }
                        /* [session 61 | 0x0801A13A RECOVERED] the keys 1/2
                         * have NO short-press action: they increment the
                         * hold counter [r5+0x21] each dispatch pass while
                         * held, and at 40 the original switches to the
                         * screen 0x83 (0x8014530) with the sub-state
                         * [r5+0x168] = the key (the key 2 also calls
                         * 0x801B15C(4,1,1) first). The hold path lives in
                         * the else-branch below. The remaining keys'
                         * short-press actions = the phase-3 per-screen
                         * work; the generic repaint here = the stand-in
                         * (the key-1 short press = NO action in the
                         * original - excluded above). */
                }
            } else if (key == 1u || key == 2u) {
                /* [0x801A13A] the held-key counter: the gates re-checked
                 * per pass; at 40 passes -> the screen 0x83. */
                if (key != 0xFFu &&
                    ((REG8(FLAG_BANK + 2u) >> 2u) & 1u) == 0u) {
                    ui_key_hold++;
                    if (ui_key_hold == 40u) {
                        ui_state_set(0x83u);   /* 0x8014530(0x83) */
                        ui_screen0_build();
                    }
                }
            }
        }
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
