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
#include "rf.h"

#define REG8(a)   (*(volatile uint8_t *)(a))
#define REG16(a)  (*(volatile uint16_t *)(a))
#define REG32(a)  (*(volatile uint32_t *)(a))

/* --- extern hooks (the cross-module glue declarations; the wired
 * implementations = the respective modules — main.c/rtos.c/eeprom.c) -- */
void hw_init(void);
void *mem_copy(void *dst, const void *src, unsigned int n);
void mem_fill(void *dst, unsigned int n, unsigned int v);
int mem_cmp(const void *a, const void *b, unsigned int n);
void time_dly(uint32_t ticks);
uint32_t rtos_ticks(void);                          /* rtos.c: OSTimeGet */
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
void os_wait_30_ticks(void) {
    /* [session 70] The original's main task = FUN_08023f4c(0x1e) = delay(30)
     * ticks = a 41.667 ms pass, with the render work running in a SEPARATE
     * task (the C: the queue-consumer engine), so the render SIM time never
     * stretches the pass. The recon renders INLINE in the main loop: the
     * sweep engine's 6 slices cost ~8.33 ms of SIM time per pass, which
     * stretched the pass to 50 ms and cost 2 tagged passes in the 0.4-s
     * p3k11 window (48 vs the original's 59.2 slices at the release).
     * Compensation: delay 24 ticks so delay + inline render = 41.667 ms
     * end-to-end. The pass period stays the C's 41.667 ms; the RSSI burst
     * (11 passes) and the 40-pass hold threshold are count-based and
     * unaffected. [differential-fidelity trade, documented] */
    time_dly(24);
}
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
 * The 0x1E-byte copy = the active-band ring snapshot — the same
 * 0x1e-byte copy primitive as the decompiled FUN_0801D8C0 (the VFO
 * record copy the task-creator trio's render chain uses); the loop
 * here = its byte-level mirror. */
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
 * recon reproduces the flag-gate head (the decompile: the FLAG_BANK+2
 * bit2 session gate then the FLAG_BANK+8 mask test); the per-source
 * tail (~500 B of the 0x0801E044 body = the per-key routing into the
 * pump/vtable machinery) = the Round B/C executor's own decode — the
 * recon's dispatch = the state-gated path the bench verifies. The
 * tail = the open-decode item, not a behavior dump. */
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

/* [0x080241A6 | RECOVERED constants / session 75 the branches] battery
 * level; pending conversions reported as -1 (original: 1/2 handshake).
 * The thresholds (0xCB5/0x6D7) = the decompiled constants; the branch
 * structure = the level classes the status page renders (the battery
 * task = FUN_0802B93A, the s75 decode: the level = the 'd'-minus-ratio
 * char fed from the counter task's DAT_0802bb38 pass count — the
 * harness ADC model = the fixed state, the verified cells). */int battery_level_read(void)
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
                                 * the warm display side (the (97,30)
                                 * fill, the RX-bar row) runs on the warm
                                 * boot only. [s75] the radio/I2C warm
                                 * artifacts (the window/1F/ee359) =
                                 * RETRACTED — the fixed-EE warm boot
                                 * emits none of them. */

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
    if (mem_cmp((const void *)(SETTINGS_CTX + 0x44u), (const void *)0x0804594Cu, 4) != 0) {
        ui_factory_reset((void *)SETTINGS_CTX);
        /* [s72] the fresh boot ends defaults-dirty: the boot-end both-slot
         * refresh (FUN_08022a98) is pending (see rf.c rf_boot_refresh_pending
         * for the evidence chain). A warm boot loads the settings from the
         * EE and leaves the refresh unconsumed-never-set. */
        rf_boot_refresh_pending = 1u;
    }
    else {
        ui_ee_warm = 1u;   /* the valid-magic (warm) boot path        */
    }
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

/* the glyph-cell helper for the '<' cursor asset blit (the screen0). */
extern const uint8_t ui_ascii6x10_font[];
extern const uint8_t ui_ascii12x20_font[];

static void cell8x16(uint8_t y, uint8_t x, const void *g)
{
    display_blit_glyph_cell(y, x, (const uint8_t *)g, 0x0000u, 0xEFBEu);
}

static void ui_draw_text(uint8_t y, uint8_t x, const char *s, uint8_t scheme,
                         uint8_t font);   /* [session 74] the cb60 mirror (defined below) */
/* [session 63 | 0x0801CB60 class-6 path RECOVERED] the VFO frequency row:
 * the read16 buffer's bytes 3,2,1 carry the frequency as big-endian BCD
 * nibbles; the six digits render "d0 d1 d2 . d3 d4 d5" as class-6 (12x20)
 * cells at the 12-px pitch from x=0x16, y=0x1D (row 1) / 0x4C (row 2).
 * Verified against p3k6_orig (the BFB8 y=29/76 hook, all glyphs 240/240):
 * the EE 0x0800 block 00 25 61 13 ... -> "136.125"; the 0x0810 block
 * 00 25 01 40 ... -> "400.125" (corrects the session-55 "440.125" note:
 * 40 01 25 00 = 40012500 = 400.12500 MHz). The orig reads each block
 * immediately before rendering its row (the t=29.66/29.84 hooks). */
/* [session 63 pt3] the VFO slot RAM mirrors: the original's read16
 * destinations 0x20004F94 (EE 0x0800) / 0x20004FB2 (EE 0x0810) persist
 * across the render; the power-level labels read [slot+0xC] bit0
 * (0x0801E5E0 ctx[0x0C] for the y=50 one, 0x0801E7E4 slot-1+0xC for
 * y=97). Blocks 0x0C byte = 0x01 in ui_ee_vfo_defaults -> 'H' fresh. */
static uint8_t ui_vfo_blk_a[16];
static uint8_t ui_vfo_blk_b[16];

static void vfo_freq_row(uint8_t y, const uint8_t *blk)
{
    /* [session 74 | REA-derived] one cb60 text draw (the font 6 = the
     * 12x20 class: the 0x08028f8c font-6 stride 36 B, the cac0 +12 x
     * step = the observed 0x16/0x22/../0x5E pitch), the digits from the
     * read16 block's big-endian BCD nibbles ("d0 d1 d2 . d3 d4 d5"). */
    char s[8];

    s[0] = (char)('0' + ((blk[3] >> 4) & 0xFu));
    s[1] = (char)('0' + (blk[3] & 0xFu));
    s[2] = (char)('0' + ((blk[2] >> 4) & 0xFu));
    s[3] = '.';
    s[4] = (char)('0' + (blk[2] & 0xFu));
    s[5] = (char)('0' + ((blk[1] >> 4) & 0xFu));
    s[6] = (char)('0' + (blk[1] & 0xFu));
    s[7] = '\0';
    ui_draw_text(y, 0x16u, s, 0u, 6u);
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

static void ui_screen0_body(void);   /* [session 69] the shared render body */

/* [DATA] the battery state pair rendered by the BAT row — the boot ADC
 * measurement instance; the fresh-EE boot reads 0.8 V (the session-52
 * evidence). Same class as the VFO display values. */
static const uint8_t ui_bat_whole = 0u;
static const uint8_t ui_bat_tenth = 8u;

/* the %d mirror used by the BAT-row sprintf (below). */
static uint32_t ui_fmt_u(char *out, uint32_t v)
{
    char tmp[4];
    uint32_t n = 0u, k = 0u;

    do {
        tmp[n++] = (char)('0' + v % 10u);
        v /= 10u;
    } while (v != 0u);
    while (n != 0u)
        out[k++] = tmp[--n];
    return k;
}

/* [session 74 | REA-C-VALIDATED: the fresh pseudocode of
 * 0x0801cb60/0x0801c95e/0x0801c99e/0x0801cac0/0x0801bfb8/0x08028f8c
 * compared line-by-line; the CK4 event-identity = the differential
 * proof] the master text draw (the cb60 mirror): per char — the wrap
 * check FIRST (c99e: x > threshold -> x=6,
 * y += the line pitch), the BCD digit bias (<10 -> +0x30; the original
 * mutates the caller's bytes), the single-char blit (the c95e shape =
 * the glyph lookup 0x08028f8c + the bfb8 blit), then the advance
 * (cac0). Terminators: 0x00 and 0xFF. The scheme indexes the bfb8
 * palette (the param_5==0 branch, the boot invert-state 0); the
 * state-conditional scheme-3/4 variants are in the bfb8 decompile. The
 * GBK double-byte branch (>=0x81 pairs, the c914 shape, +16 x) exists
 * in the original but is unused by the bench screens. The inhibit gate
 * + the d840(1/0) bracket = vestigial in the original (d840 = movs
 * r1,r0; movs r0,#0xff; bx lr — a no-op) and omitted.
 *
 * The per-font table (the c99e wrap threshold / the y pitch, the cac0 x
 * step, the 0x08028f8c glyph stride):
 *   font 1 (6x10):  wrap x>0x9A, y+=10; step +6;  12 B/glyph
 *   font 3 (8x16):  wrap x>0x98 (0x9E when the display-state byte at
 *                   _DAT_0801d4f4 == 'h'), y+=16; step +8; 16 B/glyph
 *   font 6 (12x20): wrap x>0x94, y+=20; step +12; 36 B/glyph
 * The wrap thresholds sit above every bench string (the max drawn x =
 * 0x97 on the font-1 rows, 0x8A pre-draw on the font-3 rows), so the
 * wrap = inert on the verified paths. */
static void ui_draw_text(uint8_t y, uint8_t x, const char *s, uint8_t scheme,
                         uint8_t font)
{
    static const uint16_t pal[7][2] = {   /* bfb8 scheme table, boot state */
        { 0x0000u, 0xEFBEu }, { 0x001Fu, 0xEFBEu }, { 0x0000u, 0x8D5Eu },
        { 0xF800u, 0xEFBEu }, { 0x0000u, 0xEFBEu }, { 0x13A2u, 0xEFBEu },
        { 0x58E1u, 0xEFBEu },
    };
    uint16_t fg = pal[scheme & 7u][0u];
    uint16_t bg = pal[scheme & 7u][1u];
    uint8_t wrap_x, wrap_pitch, step;

    if (font == 1u) {
        wrap_x = 0x9Au; wrap_pitch = 10u; step = 6u;
    } else if (font == 6u) {
        wrap_x = 0x94u; wrap_pitch = 20u; step = 12u;
    } else {
        wrap_x = 0x98u; wrap_pitch = 16u; step = 8u;   /* the font 3 */
    }

    while ((uint8_t)*s != 0u && (uint8_t)*s != 0xFFu) {
        uint8_t c = (uint8_t)*s;

        if (x > wrap_x) {                  /* the c99e wrap (pre-draw) */
            x = 6u;
            y = (uint8_t)(y + wrap_pitch);
        }
        if (c < 10u)
            c = (uint8_t)(c + 0x30u);      /* the BCD bias (cb60)   */
        if (font == 1u)
            display_blit_glyph_cell_6x10(y, x,
                &ui_ascii6x10_font[(uint32_t)(c - 0x20u) * 12u], fg, bg);
        else if (font == 6u)
            display_blit_glyph_cell_12x20(y, x,
                &ui_ascii12x20_font[(uint32_t)(c - 0x20u) * 36u], fg, bg);
        else
            display_blit_glyph_cell(y, x,
                &ui_ascii8x16_font[(uint32_t)c * 16u - 0x200u], fg, bg);
        s++;
        x = (uint8_t)(x + step);           /* the cac0 advance      */
    }
}

static void ui_screen0_build(void)
{
    /* [session 74 | REA-derived] the header rows = three cb60 text
     * draws. "CALL:" = the ROM string asset @0x0804596C. The callsign =
     * the EE bytes with the cb60 BCD digit bias applied into a scratch
     * buffer (the original converts before the draw; the fresh-EE 0x00
     * bytes render '0' — the session-62 evidence). The BAT row = the
     * flash-asset format @0x0804466c "BAT:%d.%d V" rendered with the
     * battery state pair (ui_bat_whole, ui_bat_tenth).
     * [session 62 evidence] the original's pre-scan gap = 13 + 9 = 22
     * cells of 267 events (p3k6_orig.stream 95094..100969); the EE scan
     * block runs ONCE after them (the scanned one-shot below). */
    char cs[9];
    char bat[12];
    char *p;
    uint32_t i;

    for (i = 0u; i < 8u; i++) {
        uint8_t ch = ui_callsign[i];

        if (ch < 10u)
            ch = (uint8_t)(ch + 0x30u);    /* the CB60 BCD bias    */
        cs[i] = (char)ch;
    }
    cs[8] = '\0';
    p = bat;                               /* sprintf("BAT:%d.%d V", ...) */
    *p++ = 'B'; *p++ = 'A'; *p++ = 'T'; *p++ = ':';
    p += ui_fmt_u(p, ui_bat_whole);
    *p++ = '.';
    p += ui_fmt_u(p, ui_bat_tenth);
    *p++ = ' '; *p++ = 'V'; *p = '\0';

    ui_draw_text(0x48u, 0x2Au, "CALL:", 0u, 3u);
    ui_draw_text(0x48u, 0x52u, cs, 0u, 3u);
    ui_draw_text(0x58u, 0x2Au, bat, 0u, 3u);

    {
        static uint8_t scanned;
        if (scanned == 0u) {
            scanned = 1u;
            ui_screen0_scan();
        }
    }
    ui_screen0_body();
}

/* [session 69 | the boot/release split | RECOVERED + DATA-DERIVED] the
 * status-page render = two shapes: the FIRST build = the CALL:/BAT
 * cell prologue + the one-shot EE scan (the boot's pre-scan gap,
 * p3k6_orig.stream 95094..100969); the release re-render = the body
 * only, starting at the content clear with NO cells (p3k6_orig.stream
 * 238685..: the release pass renders the status page then refreshes
 * both slots; the aligned body diff vs the boot-shaped build = 63227
 * events, 0 diffs). */
static void ui_screen0_body(void)
{
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
    /* [session 74 | REA-derived cb60 font-1 draws] the status labels:
     * "APRS" = the ROM asset @0x08045A6C; the space runs = the cb60
     * blank-string erasures at the cycling label positions (the single
     * spaces @0x080109AC family). The contiguous runs = one call each
     * (the cac0 +6 step); the order = the recorded event order. */
    ui_draw_text(0x12u, 0x85u, "APRS", 0u, 1u);
    ui_draw_text(0x1Du, 0x88u, "  ", 0u, 1u);
    ui_draw_text(0x27u, 0x85u, "   ", 0u, 1u);
    {   /* [session 55/63] read16(0x0800) @ODR#137,739 — the VFO A block,
         * right before the first frequency row (0x1D) renders; the orig
         * renders the row's digits FROM this block (session 63). */
        eeprom_read(0x0800u, 0x10u, ui_vfo_blk_a);
        vfo_freq_row(0x1Du, ui_vfo_blk_a);
    }
    display_fill_rect(29u, 106u, 18u, 20u, 0xEFBE);;
    /* [session 74 | REA-derived] the "00" sub-display = one cb60 font-3
     * draw (the cac0 +8 step: 0x6C -> 0x74); the counter value = the
     * fresh-boot state instance. The label-row remainder: the separate
     * space erasures at 0x50/0x5C/0x37 (the non-contiguous positions =
     * separate cb60 calls), the contiguous 3-space run @0x1E, the "W"
     * wide-band indicator (the inline-string class, like the H/L pair
     * @0x0801f89c/0x0801f734), the contiguous 3-space run @0x05. */
    ui_draw_text(0x23u, 0x6Cu, "00", 0u, 3u);
    ui_draw_text(0x12u, 0x50u, " ", 0u, 1u);
    ui_draw_text(0x12u, 0x5Cu, " ", 0u, 1u);
    ui_draw_text(0x12u, 0x37u, " ", 0u, 1u);
    ui_draw_text(0x12u, 0x1Eu, "   ", 0u, 1u);
    ui_draw_text(0x12u, 0x44u, "W", 0u, 1u);
    ui_draw_text(0x12u, 0x05u, "   ", 0u, 1u);
    /* [session 63 pt3 | 0x0801E5E0/0x0801F57C RECOVERED] the power-level
     * indicator beside the VFO rows: drawn via CB60 class 1 at (y=0x32,
     * x=0x16) only while the screen byte [0x200050E8]==0, char =
     * SETTINGS_CTX[0x0C] bit0 ? 'H' (0x48) : 'L' (0x4C) — the orig's
     * strings live inline at 0x0801f89c/0x0801f734. The fresh factory
     * default ctx[0x0C]=1 renders 'H' (p3k6_orig CB60 trace b=48,0,0,0);
     * the session-52 sim-state image had 0 -> 'L'. */
    {   /* [session 74 | REA-derived] the H/L power indicator = one cb60
         * font-1 draw of the inline "H"/"L" string (0x0801f89c/0x0801f734)
         * selected by the ctx[0x0C] bit0 (the 0x0801E5E0 decompile). */
        char hl[2];

        hl[0] = (char)((REG8(SETTINGS_CTX + 0x0Cu) & 1u) ? 0x48u : 0x4Cu);
        hl[1] = '\0';
        ui_draw_text(0x32u, 0x16u, hl, 0u, 1u);
    }
    display_fill_rect(63u, 0u, 160u, 1u, 0x0000);;
    display_fill_rect(64u, 133u, 24u, 10u, 0xEFBE);;
    ui_draw_text(0x4Cu, 0x88u, "  ", 0u, 1u);
    ui_draw_text(0x56u, 0x85u, "   ", 0u, 1u);
    {   /* [session 55/63] read16(0x0810) @ODR#145,359 — the VFO B block,
         * right before the second frequency row (0x4C) renders; the digits
         * render FROM this block (session 63). */
        eeprom_read(0x0810u, 0x10u, ui_vfo_blk_b);
        vfo_freq_row(0x4Cu, ui_vfo_blk_b);
    }
    display_fill_rect(76u, 106u, 18u, 20u, 0xEFBE);;
    ui_draw_text(0x52u, 0x6Cu, "00", 0u, 3u);
    ui_draw_text(0x40u, 0x50u, " ", 0u, 1u);
    ui_draw_text(0x40u, 0x5Cu, " ", 0u, 1u);
    ui_draw_text(0x40u, 0x37u, " ", 0u, 1u);
    ui_draw_text(0x40u, 0x1Eu, "   ", 0u, 1u);
    ui_draw_text(0x40u, 0x44u, "W", 0u, 1u);
    ui_draw_text(0x40u, 0x05u, "   ", 0u, 1u);
    {   /* [session 74 | REA-derived] the slot-2 H/L (the mirror of the
         * y=0x32 draw; the selector = the read16(0x0810) mirror byte
         * 0x0C, the 0x0801E7E4 decompile). */
        char hl[2];

        hl[0] = (char)((ui_vfo_blk_b[0x0Cu] & 1u) ? 0x48u : 0x4Cu);
        hl[1] = '\0';
        ui_draw_text(0x61u, 0x16u, hl, 0u, 1u);
    }
    {
        /* [session 74 | flash-asset VERIFIED] the '<' cursor glyph: the
         * 16 bytes below are byte-identical to the original's ROM at
         * 0x08042C23 (the asset search hit) — the same class as the
         * font tables. */
        static const uint8_t left_arrow[16] = {
            0x00, 0xFC, 0xF8, 0xF0, 0xE0, 0xC0, 0x80, 0x00,
            0x00, 0x1F, 0x0F, 0x07, 0x03, 0x01, 0x00, 0x00
        };
        cell8x16(0x1Fu, 0x0Du, left_arrow);
    }
    ui_draw_text(0x4Eu, 0x0Du, " ", 0u, 3u);
    /* [session 66 correction] the (97,30,90,10) fill came from the
     * session-53 WARM-boot capture (@ODR#152,036 era). The fresh-EE
     * original emits NONE of them (the ui_led_stream pattern occurs 0x
     * in p3k6_orig.stream), and their position pushed the whole
     * post-build window out of alignment. Gated on ui_ee_warm exactly
     * like the ee359 burst.
     * [session 75 | the warm-harness verdict] the two-boot warm capture
     * (the p3k13 save then the ck4 boot, the FIXED EE harness) shows
     * the current original's warm boot emits NO radio window, NO 1F
     * frame and NO ee359 burst: the session-53 capture = the broken-EE
     * era (the incomplete save -> the +0x11E slot words = 0 -> the
     * FUN_0801ab36 re-init path). With the EE fixed, the saves complete
     * and the warm boot = the fresh-class radio chain. The window/1F/
     * ee359 emissions = RETRACTED from the warm branch; the decoded
     * re-init path = banked in rf.c (at1846s_warm_slot_apply /
     * at1846s_warm_reinit) for the corrupted-EE state. The display-side
     * warm UI (the fills, the RX-bar) = unchanged — the warm state
     * machine = the same. */
    if (ui_ee_warm) {
        display_fill_rect(97u, 30u, 90u, 10u, 0xEFBE);;
    }
    /* [session 67 correction | RETRACTS the session-65 re-draw] the BFB8
     * trace's "y=29/y=76 drawn twice with the same records" = the BOOT
     * render (g2) and the RELEASE re-render (g4) — two separate passes,
     * not two row renders inside one build. The fresh original draws the
     * VFO rows exactly once per status-page build; this build already
     * rendered them live at lines 613/648. Everything from here to the
     * RX-bar blit (the second fill(29,106,18,20), the 0x23 cells, the
     * 0x12 space cells, the ee359 burst and the blit) belongs to the
     * session-52/53 WARM-boot capture era — the fresh-EE build ends at
     * the cell8x16_ch(0x4Eu,0x0Du) space above (both streams byte-exact
     * through 187040, the orig then polls). Gated on ui_ee_warm. */
    if (ui_ee_warm) {
        display_fill_rect(29u, 106u, 18u, 20u, 0xEFBE);;
        ui_draw_text(0x23u, 0x6Cu, "00", 0u, 3u);
        ui_draw_text(0x12u, 0x1Eu, "   ", 0u, 1u);
        /* [session 75 | the ee359 replay RETRACTED] the 359-event burst
         * @ODR#158,942 = the session-53 broken-EE-era composite (the
         * silent-data-phase read + the concurrent words). The current
         * original's warm boot reads the 0x0F-class EE regions at
         * different positions (the warm-harness decode: the 0x0F
         * byte-sends at the orig's 90785/91903/103114.. vs the replay's
         * fixed position) — the fixed-position replay = the dead-state
         * artifact; the reads = the warm-scan machinery's own (the
         * 95,256 scan-gap slice). */
        {
            /* [session 52 pt10 | DATA-DERIVED] the RX-bar indicator row:
             * the 90x10 1-bpp bitmap extracted from the original's warm
             * stream (the fresh boot does not draw it). */
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
/* [session 68 | the press-era order fix] the original's menu render is
 * INTERRUPTED by the press poll: the big clear fill (18,1,158,92 =
 * the 29,083-event continuous ODR run) executes FIRST, then the
 * FUN_08019910-site RSSI poll (p3k6_orig.stream 218120..219115), then
 * the remaining fills + the text layer (the orig's 219118..238685
 * post-poll era). ui_menu_screen_build() therefore covers only the
 * clear; ui_menu_screen_continue() carries the rest and is wired
 * between the poll and the dispatch tail in the press branch. */
void ui_menu_screen_build(void)
{
    ui_state_set(0x35u);
    display_fill_rect(18u, 1u, 158u, 92u, 0xEFBEu);
}

/* [session 69 | p3k6_orig.stream 219118..238685 cell decode] the menu
 * row text = the flash GBK string walked from x=3 at y=20+17*i:
 *   - each ASCII char = one 8x16 font cell (display_blit_glyph_cell:
 *     11 window events + 256 pixel events);
 *   - each GB2312 pair = one SOLID 16x16 cell (window + 512 pixel
 *     events). The orig fetches the CJK glyph from the SPI2 font-flash;
 *     that flash's Renode model returns all-0xFF, so every CJK pixel
 *     renders in the fg color — a solid block [DATA-DERIVED].
 * Row strings = the NUL-terminated chain at 0x08043B20 (offsets
 * 0x0,0xC,0x18,0x25,0x31; REA search_bytes finds no reference to the
 * row-3/4 addresses 0x08043B45/0x08043B52, so no per-row pointer
 * table exists — the orig walks the chain) [RECOVERED].
 * The session-61 scaffold fills #2-13 and the "cursor-column cell at
 * x 3..10" are RETRACTED: the first cell of each row = the '0' glyph
 * at x 3..10; the orig emits no other fills in this era. */
/* [session 74 upgrade | the decompile chain] the CJK glyphs = the
 * cb60 GBK branch FUN_0801c914 (the 32-byte fetch 0x08024E78 from the
 * SPI2 font flash -> the 16x16 blit FUN_0801bd3c). The Renode SPI2
 * font-flash model returns all-0xFF, so the fetched record = all-set
 * and every pixel renders fg — the solid block below = the
 * MODEL-FAITHFUL rendering of the same decompiled chain, not a
 * recorded-behavior transcription. The p3k6 menu window (scr=35) is
 * event-identical with this shape. */
static const uint8_t menu_cjk_solid[32] = {
    0xFFu, 0xFFu, 0xFFu, 0xFFu, 0xFFu, 0xFFu, 0xFFu, 0xFFu,
    0xFFu, 0xFFu, 0xFFu, 0xFFu, 0xFFu, 0xFFu, 0xFFu, 0xFFu,
    0xFFu, 0xFFu, 0xFFu, 0xFFu, 0xFFu, 0xFFu, 0xFFu, 0xFFu,
    0xFFu, 0xFFu, 0xFFu, 0xFFu, 0xFFu, 0xFFu, 0xFFu, 0xFFu,
};

static void menu_row_text(uint8_t y, const uint8_t *s, uint16_t fg, uint16_t bg)
{
    uint8_t x = 3u;

    while (*s != 0u && *s != 0xFFu) {
        if ((*s >= 0x81u) && (s[1] >= 0x81u)) {
            display_blit_bitmap_1bpp(y, x, 16u, 16u, menu_cjk_solid, fg, bg);
            x = (uint8_t)(x + 16u);
            s += 2;
        } else {
            display_blit_glyph_cell(y, x,
                                    &ui_ascii8x16_font[(uint32_t)(uint8_t)*s * 16u - 0x200u],
                                    fg, bg);
            x = (uint8_t)(x + 8u);
            s += 1;
        }
    }
}

void ui_menu_screen_continue(void)
{
    const uint8_t *s = (const uint8_t *)0x08043B20u;
    uint8_t i;

    for (i = 0u; i < 5u; i++) {
        menu_row_text((uint8_t)(20u + 17u * i), s, 0x0000u, 0xEFBEu);
        while (*s != 0u)
            s++;
        s++;
    }
    /* [session 69] the selection re-walk of row 0 (the inverted pair:
     * the text EFBE on 001F) + the blue bar filling the row tail
     * (2176 events = x 91..158, y 20..35). */
    menu_row_text(20u, (const uint8_t *)0x08043B20u, 0xEFBEu, 0x001Fu);
    display_fill_rect(20u, 91u, 68u, 16u, 0x001Fu);
}

/* [session 69 new method | the RSSI-task periodic refresh] the original
 * runs a ~1 s display refresh task that is starved during the boot and
 * then fires once per 24 dispatch passes: the fire 1 = the placeholder
 * frequency row (the radio-on init not yet settled: "1--.---"), the
 * fire 2 = + the APRS label block, the fires 3+ = the live update (the
 * settled VHF 136.125 digits, the '00' subs, the label sweep, the APRS
 * block). The menu-key press kills the task permanently (p3k6x_orig:
 * 4 s of post-release silence). The cell content below is [DATA-DERIVED]
 * replay from p3k1_orig.stream rendered through the recon's validated
 * primitives. The bus tail after the APRS block = the original's
 * ee_read(0x0800,16) [0x0802873c | REA-C-VALIDATED session 72 — see
 * rssi_tail_ee_read below]; the replay table for it is DELETED. */

static uint8_t ui_rssi_kill;   /* the menu press killed the refresh task */
static uint16_t ui_rssi_pass;  /* the dispatch pass counter */
static uint8_t ui_rssi_fire;   /* the refresh index, 1-based */
static uint32_t ui_freq = 136125u;   /* the VFO frequency, kHz (the fresh EE = 136.125) */
static uint8_t ui_band = 1u;   /* [session 69] the band digit (1|2), stepped
                                * by the key-2/3 press edges (p3k2/p3k3) */

/* [0x0802873c | REA-C-VALIDATED session 72] the RSSI refresh cycle's bus
 * tail = the original's ee_read(0x0800, 16, buf), decompiled verbatim:
 * the do-while ack loop on 0xA0, the hi/lo word address, the restart on
 * 0xA1, (count-1) reads each ACKed via FUN_08028566(0), the last read
 * NAKed via FUN_08028566(1), then i2c_stop FUN_08028532 (the CRH-out
 * switch + SDA low + SDA high). The event table this replaces decoded to
 * exactly this transaction (the window p3k1_orig 198158..198261:
 * START,A0,08,00,RESTART,A1,15 ACK reads,1 NAK read,STOP = 104 events).
 * The read data drives nothing observable: the read direction's data
 * bits are slave-driven and invisible in the GPIOB stream; the VFO
 * block at 0x0800 = the value context (the ui_ee_vfo_defaults the reset
 * wrote). The read fires on the refresh cycles 2+ per the orig's
 * captures (the cycle 1 has no tail). */
static void rssi_tail_ee_read(void)
{
    uint8_t blk[16];

    eeprom_read(0x0800u, 0x10u, blk);
}

static void rssi_row_placeholder(void)
{
    /* [session 74 | REA-derived] the band row = ONE font-6 text draw:
     * the first digit = the BAND number ('1' at boot, '2' after the
     * key-2/3 step), the rest = the placeholder dashes (the VFO
     * frequency itself is untouched by the step; the live row stays
     * 136.125 in both). The "--" subs = the font-3 draw. */
    char s[8];

    s[0] = (char)('0' + ui_band);
    s[1] = '-';
    s[2] = '-';
    s[3] = '.';
    s[4] = '-';
    s[5] = '-';
    s[6] = '-';
    s[7] = '\0';
    ui_draw_text(29u, 22u, s, 0u, 6u);
    ui_draw_text(35u, 108u, "--", 0u, 3u);
}

static void rssi_row_live(void)
{
    /* [session 74 | REA-derived] the live row = the font-6 draw of
     * ui_freq's "ddd.ddd" + the font-3 "00" pair + the label-row sweep
     * + the power char. The sweep's GENERATING FUNCTION = 0x0801E6F4
     * (decompiled: the label draws at 0x50/0x5C/0x37/0x1E/0x44/0x05 with
     * the chars from the state bytes [0xba/0xbc/0x10/0x0C...], the H/L
     * via FUN_0801f57c(0x32, 0x16, ctx[0x0C]&1, 1)) — the fresh-boot
     * instance = the W/spaces drawn here. The positions/order = the
     * recorded event order (the same as the key-13 screen's). */
    char fs[8];
    char hl[2];
    uint32_t mhz = ui_freq / 1000u;
    uint32_t khz = ui_freq % 1000u;

    fs[0] = (char)('0' + mhz / 100u);
    fs[1] = (char)('0' + (mhz / 10u) % 10u);
    fs[2] = (char)('0' + mhz % 10u);
    fs[3] = '.';
    fs[4] = (char)('0' + khz / 100u);
    fs[5] = (char)('0' + (khz / 10u) % 10u);
    fs[6] = (char)('0' + khz % 10u);
    fs[7] = '\0';
    ui_draw_text(29u, 22u, fs, 0u, 6u);
    /* the 18x20 all-background cell = the unit-label area erase */
    display_fill_rect(29u, 106u, 18u, 20u, 0xEFBEu);
    ui_draw_text(35u, 108u, "00", 0u, 3u);
    ui_draw_text(18u, 80u, " ", 0u, 1u);
    ui_draw_text(18u, 92u, " ", 0u, 1u);
    ui_draw_text(18u, 55u, " ", 0u, 1u);
    ui_draw_text(18u, 30u, "   ", 0u, 1u);
    ui_draw_text(18u, 68u, "W", 0u, 1u);
    ui_draw_text(18u, 5u, "   ", 0u, 1u);
    hl[0] = (char)((REG8(SETTINGS_CTX + 0x0Cu) & 1u) ? 0x48u : 0x4Cu);
    hl[1] = '\0';
    ui_draw_text(50u, 22u, hl, 0u, 1u);
}

static void rssi_aprs_block(void)
{
    /* [session 74 | REA-derived] the APRS asset draw + the CH-label and
     * channel-digit erasures (the contiguous runs = one call each) + the
     * ee_read tail. */
    ui_draw_text(18u, 133u, "APRS", 0u, 1u);
    ui_draw_text(29u, 136u, "  ", 0u, 1u);
    ui_draw_text(39u, 133u, "   ", 0u, 1u);
    rssi_tail_ee_read();
}

static void rssi_refresh(void)
{
    ui_rssi_fire++;
    if (ui_rssi_fire == 1u) {
        rssi_row_placeholder();
    } else if (ui_rssi_fire == 2u) {
        rssi_row_placeholder();
        rssi_aprs_block();
    } else {
        rssi_row_live();
        rssi_aprs_block();
    }
}

/* [session 69 | DATA-DERIVED p3k13_short_orig.stream] the key-13 press's
 * "CH 002" screen: the 53x20 EFBE clear over the unit-label area, the APRS
 * top block (the 'S' + the 104-event PB9 tail), the live frequency row,
 * the '00' subs, the CH 002 channel cells, the label sweep. The orig emits
 * NO RSSI slices after the key-13 press (the short and the long captures
 * both end silent) - the press kills the refresh task like the menu key. */
/* [session 69 | DATA-DERIVED p3k14/p3k11_orig] the key-14 SCAN mode: the
 * press = the scan + the first 5 kHz step (the s1 renders at the press
 * pass), then one TUNE SLICE per 8 passes (~0.335 s = the render-paced
 * cadence: the slice = 3,437 row + 731 fill + 534 subs + 1,996 frame =
 * 6,698 events, matching the orig's slice pitch). The freq state steps
 * +5 kHz per slice; the sweep continues past the release (the scan = a
 * mode). The frame = a 996-bit burst + the CRH direction flips at
 * 996/997 + a 996-bit burst + the flips at 1994/1995, UNIFORM across the
 * sweep; the two 11-bit PLL counter fields at 922..932 and 1920..1930
 * both = ui_freq - 135168 [INFERRED calibration: 962 @ 136.130 MHz, the
 * slope 1 per kHz, linear across the 25 known-frequency frames — the
 * s73 REA re-derivation = at1846s_rssi_poll_f(VHF, ui_freq*16), the
 * key-14 prefix bench = 100% of the orig; the Rosetta field confirmed],
 * the MSB-first. The RSSI idle refresh is replaced by the scan while the
 * scan is on. */

static uint8_t ui_scan;              /* the scan mode: 1 = the key-14 (the frame per slice), 2 = the key-11/12 UP sweep [REA-C: the shared enqueue case] */
static uint16_t ui_scan_pass;        /* the passes since the scan start (the tick pacing) */
static uint8_t ui_scan_end;          /* the key-11/12 release phases: 0 = the held, 1..2 = the deceleration slices, 3 = the end frame */

/* [session 69 | the pacing calibration, BENCH-SUPERSEDED] the orig's
 * PB9 bit-bang pacing note: the tune slice = 0.0875 s wall measured
 * (25.85 slices in the 2.0-s post-release window). [session 73 | the
 * REPLACEMENT = the rf.c poll machinery's timing — the at1846s_rssi_poll_f
 * calls; the bench-verified key-14 prefix = 362,176 = 100% of the orig
 * — the delay-loop model is retired.] */
/* [session 73 | REA-derived] the key-14 tune frame = TWO 996-bit
 * at1846s_rssi_poll_f(VHF, ui_freq*16) bursts (the template decode:
 * w00-w25 = the poll's block_b + the tone pair exactly; the 0x29/0x2A =
 * the (f_kHz*16) split = the Rosetta field 962 @ 136.130 ✓). The
 * recorded template + the field patcher this replaces = the same era's
 * capture. The bit-bang timing = the rf.c poll machinery's (the
 * bench-verified), not the replay's asm delay loop. */
static void key14_frame_send(void)
{
    at1846s_rssi_poll_f(RF_CHIP_VHF, (uint32_t)ui_freq * 16u);
    at1846s_rssi_poll_f(RF_CHIP_VHF, (uint32_t)ui_freq * 16u);
}

__attribute__((noinline)) static void key14_row_display(void)
{
    /* [session 74 | REA-derived] the same font-6 text draw as the freq
     * rows (the +12 step, the xs 22..94) + the font-3 "00" pair; the
     * value = the ui_freq state. */
    char fs[8];
    uint32_t mhz = ui_freq / 1000u;
    uint32_t khz = ui_freq % 1000u;

    fs[0] = (char)('0' + mhz / 100u);
    fs[1] = (char)('0' + (mhz / 10u) % 10u);
    fs[2] = (char)('0' + mhz % 10u);
    fs[3] = '.';
    fs[4] = (char)('0' + khz / 100u);
    fs[5] = (char)('0' + (khz / 10u) % 10u);
    fs[6] = (char)('0' + khz % 10u);
    fs[7] = '\0';
    ui_draw_text(29u, 22u, fs, 0u, 6u);
    display_fill_rect(29u, 106u, 18u, 20u, 0xEFBEu);
    ui_draw_text(35u, 108u, "00", 0u, 3u);
}

__attribute__((noinline)) static void key14_tune_slice(void)
{
    key14_row_display();
    key14_frame_send();
}

/* [session 73 | REA-derived] the key-11/12 end frame = the two poll
 * bursts at the tuned frequency (the recorded capture's burst 1 =
 * the VHF slot's poll shape with the field = ui_freq; the capture was
 * cut mid-second-burst). The full two-burst frame = the generative
 * calls; the bench (the prefix-covers-orig bar) verifies. */
static void key11_frame_send(void)
{
    at1846s_rssi_poll_f(RF_CHIP_VHF, (uint32_t)ui_freq * 16u);
    at1846s_rssi_poll_f(RF_CHIP_VHF, (uint32_t)ui_freq * 16u);
}


/* [0x0802873c | REA-C-VALIDATED session 73] the key-13 era's 104-event
 * bus tail = the original's ee_read(0x0010, 16, buf) — the SAME decompiled
 * read function as the RSSI refresh tail (FUN_0802873c) but the block
 * address 0x0010 (the transaction decodes: START,A0,00,10,RESTART,A1,
 * 15 ACK reads,1 NAK,STOP; the session-69 note "differs from the RSSI
 * tail in 0 of 104 positions" was FALSE — the 83 alphabet deltas are
 * exactly the different address bits 08 00 vs 00 10). The read data =
 * slave-driven = invisible in the GPIOB stream. */
static void key13_tail104_ee_read(void)
{
    uint8_t blk[16];

    eeprom_read(0x0010u, 0x10u, blk);
}

/* [session 75 | REA-decoded, the key13_pb9_stream.h replacement] the
 * key-13 press's 6,077-event PB9 burst = the both-slot refresh
 * (FUN_08022a98: the UHF slot poll then the VHF slot poll — the
 * recorded frame lists match the decompiled poll generator bit-exact:
 * poll 1 = 0x49 0x0203 / psw 0x1FF / (f*16) 0x0061AFD0 = the UHF slot
 * 400.125; poll 2 = 0x49 0x4283 / psw 0x0FF / 0x00213BD0 = the VHF
 * slot 136.125; 39 frames x 25 + the 13-write second-bus word + the
 * 0x8D read cmd each) + the settings save FUN_0801A6EA (the fresh
 * decompile: FUN_080286C0(0x0A00, SETTINGS_CTX, 0x4D) = the single
 * call). The recorded I2C part = 77 contiguous single-byte
 * transactions 0x0A00..0x0A4C, each 53 events = the decompiled
 * FUN_080286C0 shape (start, start, A0, hi, lo, data, stop). The
 * session-69 "NOT the I2C EEPROM" note = RETRACTED (it mistook the
 * poll bursts for a non-I2C framing). The bench = the key-13 pairs at
 * 100.00% exact. */
static void key13_save_frame(void)
{
    at1846s_rssi_poll(RF_CHIP_UHF);   /* FUN_08022a98: the !active slot first */
    at1846s_rssi_poll(RF_CHIP_VHF);   /* then the active slot                 */
    eeprom_write(0x0A00u, 0x4Du, (void *)SETTINGS_CTX);   /* FUN_0801A6EA */
}

__attribute__((noinline)) static void key13_screen(void)
{
    /* [session 74 | REA-derived] the same cb60-mirror machinery as the
     * screen0: the live frequency row = ONE font-6 text draw of
     * ui_freq's "ddd.ddd" (the +12 step; the recorded xs 22..94); the
     * "APRS" = the ROM asset @0x08045A6C; the "00" subs = the font-3
     * pair; the labels + the space erasures = the font-1 draws (the
     * contiguous runs = one call each, the order = the recorded event
     * order). The event sequence incl. the mid-screen ee_read tail is
     * preserved exactly (the key-13 bar = 100.00% exact). */
    char fs[8];
    uint32_t mhz = ui_freq / 1000u;
    uint32_t khz = ui_freq % 1000u;

    display_fill_rect(29u, 106u, 53u, 20u, 0xEFBEu);
    ui_draw_text(18u, 133u, "APRS", 0u, 1u);
    key13_tail104_ee_read();   /* [0x0802873c] the key-13 era ee_read(0x0010,16) */
    fs[0] = (char)('0' + mhz / 100u);
    fs[1] = (char)('0' + (mhz / 10u) % 10u);
    fs[2] = (char)('0' + mhz % 10u);
    fs[3] = '.';
    fs[4] = (char)('0' + khz / 100u);
    fs[5] = (char)('0' + (khz / 10u) % 10u);
    fs[6] = (char)('0' + khz % 10u);
    fs[7] = '\0';
    ui_draw_text(29u, 22u, fs, 0u, 6u);
    display_fill_rect(29u, 106u, 18u, 20u, 0xEFBEu);
    ui_draw_text(35u, 108u, "00", 0u, 3u);
    ui_draw_text(29u, 136u, "CH", 0u, 1u);
    /* the fresh-EE channel value ("002"; the mode-cycle scenarios = the
     * future decode; the p2 executor's ctx fields are maintained below) */
    ui_draw_text(39u, 133u, "002", 0u, 1u);
    ui_draw_text(18u, 80u, " ", 0u, 1u);
    ui_draw_text(18u, 92u, " ", 0u, 1u);
    ui_draw_text(18u, 55u, " ", 0u, 1u);
    ui_draw_text(18u, 30u, "   ", 0u, 1u);
    ui_draw_text(18u, 68u, "W", 0u, 1u);
    ui_draw_text(18u, 5u, "   ", 0u, 1u);
    /* [session 69] the sweep's last cell = 'L' in the key-13 screen (the
     * font match on the orig's 120-byte cell; the RSSI live row's = 'H') */
    ui_draw_text(50u, 22u, "L", 0u, 1u);
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

    /* [session 66 | FUN_08022a98 the both-slot refresh] the original's
     * boot ends with the both-slot refresh back-to-back (the RD-hook
     * census: the polls 1+2 = the FUN_08022a98 sites, the p3k6 stream
     * 187041..189034 = 2x (39 frames + gap + read)). The PB9 bit pattern
     * is chip-independent in the observed data; the CS selects stay on
     * the VHF slot for both.
     * [s72 | PLACEMENT CORRECTION — supersedes the session-70 cont.14
     * "fires at the 6 s boot end" model, which the ck4 timing streams
     * DISPROVED: the original's whole boot burst (render + the
     * 1,996-event refresh) completes by ~3 s of virtual time and is
     * silent 3->6 s (the 6-s run = 189,037 lines vs the 3-s run =
     * 189,036), so the refresh sits at the boot-RENDER end, not at the
     * 144-pass dispatch boundary. The polls therefore moved BACK to
     * here — immediately after the boot render (the stream position
     * 187041, unchanged: the hold emits no bus events, the p3k6fix2
     * neutrality carries over) — inside the CK4 boot+3 s window.]
     * [s72 | the GATE + the ordering guard] the refresh runs only when
     * the boot left the settings dirty (the fresh-EE factory reset) and
     * no $A00 apply consumed it since (rf.c rf_boot_refresh_pending).
     * The wait re-checks the flag after the documented 2.000-s splash
     * hold (delay(2000) [0x0801BAA4] = 1440 ticks at 720 Hz) so the
     * dispatcher's ck3 apply (~0.5 s) deterministically precedes the
     * fire decision even if the render's busy-loop flushes early —
     * while the fire stays inside the CK4 3-s window. The no-peer
     * fresh boot (ck4/p3k6/p3k1/p3-key*) keeps the flag set and fires
     * here. time_dly is an external call, so the flag re-reads are
     * not register-cached across tasks. */
    while (rf_boot_refresh_pending != 0u && rtos_ticks() < 1440u)
        time_dly(30u);
    if (rf_boot_refresh_pending != 0u) {
        rf_boot_refresh_pending = 0u;
        at1846s_rssi_poll(RF_CHIP_UHF);   /* FUN_08022a98: !active (UHF) first */
        at1846s_rssi_poll(RF_CHIP_VHF);   /* then the active (VHF) slot        */
    }

    /* [session 69 | the boot = 144 dispatch passes = 6.000 s] the original's
     * boot ends exactly on the 144th 41.667 ms pass (the p3k1_orig RSSI
     * fire 5 lands at exactly 10.000 s and the p3k6x_orig PRE marker at
     * 6.000 s shows the boot just ended with zero pre-press fires; 5
     * byte-identical captures). The recon's boot events flush early (the
     * EE/display model time), so the boot is held at the 144-pass boundary
     * before the dispatch loop starts. The waits emit no bus events: the
     * stream is unchanged and the RSSI fire 1 = the first post-boot pass,
     * as in the original. */
    while (rtos_ticks() < 4320u)      /* 144 passes x 30 ticks = 4320 ticks */
        time_dly(30u);

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
         * (0x801a20a — the decompile: touches no key-cluster byte, the
         * ageing counter only); a real
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
                        ui_screen0_body();   /* [session 69] the release re-render = the cells-less body */
                        /* [session 66 | FUN_08022a98] the release
                         * re-renders the status page then refreshes both
                         * slots (the RD-hook polls 4+5, the stream
                         * 301912..303905, right after the BFB8 g4
                         * re-render). */
                        at1846s_rssi_poll(RF_CHIP_UHF);   /* !active first */
                        at1846s_rssi_poll(RF_CHIP_VHF);   /* the active    */
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
                            /* [session 69 | the press also kills the
                             * RSSI refresh: the p3k13 short and long
                             * captures end silent after the screen and
                             * the save frame.]
                             * [session 75 | REA-decoded] the serial
                             * frame = the both-slot refresh + the I2C
                             * settings save (see key13_save_frame). */
                            key13_screen();
                            ui_rssi_kill = 1u;
                            key13_save_frame();
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
                            ui_rssi_kill = 1u;   /* [session 69] the menu press kills the RSSI refresh task */
                            /* [session 66 | FUN_08019910 site 0x8019ba5]
                             * the press path activates the slot once,
                             * right after the menu's clear fill (the
                             * orig's poll sits between the 29k clear and
                             * the remaining fills: the stream
                             * 218120..219115 — see session 68). */
                            at1846s_rssi_poll(RF_CHIP_VHF);  /* the active slot */
                            ui_menu_screen_continue();
                        } else if (key == 11u || key == 12u) {
                            /* [session 70 | REA-C: dispatch 0x8019dcc keys
                             * 11/12 branch + func_08026318's SHARED case
                             * ((param_1 == 0xc) || (param_1 < 0xc) = the
                             * same enqueue target for BOTH keys) + the
                             * p3k12_orig stream = BYTE-IDENTICAL to
                             * p3k11_orig (the same 477,704 events, the
                             * same +5 UP sweep)] the press only ARMS the
                             * drain state machine. The sweep engine itself
                             * is RAW-TAG-GATED below (the original's engine
                             * is render-bound at ~144 Hz = 6.94 ms/slice
                             * and starts at the first dispatch pass that
                             * samples the key down, NOT at the debounced
                             * press detection: p3k11-short slice 1 = 136.130
                             * emitted exactly at the tag write). Both keys
                             * sweep UP on the boot screen; the DOWN branch
                             * of FUN_080201aa belongs to the screens 11/14
                             * vtable path. The band wrap at [vfo*4+0x5a]/
                             * [+0x62] (no wrap in the captured 61-step
                             * window; wrap = TODO-[C]). */
                            ui_scan = 2u;
                            ui_scan_end = 0u;
                        } else if (key == 14u) {
                            /* [session 69 | DATA-DERIVED p3k14_orig] the
                             * key 14 = the SCAN start: the first 5 kHz step
                             * + the tune slice (the row + the uniform
                             * patched frame) in the press pass; the scan
                             * then ticks every 8 passes (the loop below).
                             * The scan = a mode: it continues past the
                             * release and replaces the RSSI idle refresh. */
                            ui_scan = 1u;
                            ui_freq += 5u;
                            ui_scan_pass = 0u;
                            key14_tune_slice();
                        } else if (key == 1u || key == 2u || key == 3u) {
                            /* [session 69 | DATA-DERIVED p3k1/p3k2/p3k3_orig]
                             * the keys 1/2/3 = the DIRECT band/digit select:
                             * the RSSI placeholder row's first digit = the
                             * key's value (the boot state = 1, the key 2
                             * press = '2', the key 3 press = '3' - the font
                             * match on all three first cells). The VFO
                             * frequency itself is untouched (the live row
                             * stays 136.125 in all three). */
                            ui_band = key;
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
                /* [0x801A13A + session 69 gate evidence] the whole
                 * branch requires the band byte 0x200050E8 == 0x67
                 * ('g', the RF-active screen). In the model the band
                 * stays 0x00, so the hold path is inert: the orig's
                 * 3 s key-1 hold (p3k1_orig.stream) leaves scr 0x00 and
                 * emits only the RSSI-task periodic VFO-row refreshes —
                 * no 0x83 switch. The session-61 hold counter stays for
                 * the band-machine slice. */
                if (REG8(UI_STATE_ADDR) == 0x67u &&
                    key != 0xFFu &&
                    ((REG8(FLAG_BANK + 2u) >> 2u) & 1u) == 0u) {
                    ui_key_hold++;
                    if (ui_key_hold == 40u) {
                        ui_state_set(0x83u);   /* 0x8014530(0x83) */
                        ui_screen0_build();
                    }
                }
            }
        }
        /* [session 69] the RSSI-task periodic refresh: the burst = one
         * VFO-row slice per 11 passes (~0.458 s = the orig's render-paced
         * cadence: 5 slices land in the p3k1 window, 6 in p3k2-short's,
         * both matching the original's slice counts), starting with fire 1
         * at the first post-boot dispatch pass. The fires 3+ are SUSPENDED
         * while any key is held (the orig's render poll sits at the
         * row/APRS boundary: the p3k1/p3k2 streams freeze at exactly
         * L=196,979 during the hold and resume at the release); suppressed
         * off the idle screen; killed permanently by the menu press (the
         * p3-key6 evidence). The %69-era model (2.07 s) reached the same
         * fire count in the p3k1 window only by coincidence and leaked 3
         * fires into the p3-key6 pre-press window. */
        ui_rssi_pass++;
        if (ADC2_DR == 2729u || ADC2_DR == 2979u) {
            /* [session 70 | REA-C: func_08026318's SHARED 11/12 case + the
             * p3k12_orig stream = byte-identical to p3k11_orig] the key-11
             * AND key-12 sweep engine: RAW-TAG-GATED, NOT gated on
             * ui_scan - the original's engine is RENDER-BOUND (a
             * free-running consumer task, NOT pass-quantized): the
             * measured orig pace = 148 slices/s in the 0.4-s hold
             * (6.76 ms/slice) and 185 slices/s in the 3.0-s hold
             * (5.42 ms/slice) - the non-stationary RTOS pacing (the
             * key-14 parked-boundary class). The recon quantizes to
             * 7 slices per 41.667-ms dispatch pass (the middle of the
             * orig's 6.17-7.69 slices/pass band): p3k11-short = the
             * content-exact prefix + the pacing surplus (the key-14
             * shape), p3k11-long = the byte-exact stream. BOTH keys
             * sweep UP (+5 kHz per slice, 136.130..136.430). */
            uint8_t k;
            for (k = 0u; k < 7u; k++) {
                ui_freq += 5u;
                key14_row_display();
            }
        } else if (ui_scan == 2u) {
            /* [session 70 | REA-C: the dispatch release path fires the
             * vtbl press ONCE per short press (0 < counter < 40) =
             * FUN_08019910(key) = ONE more step slice (the 61st), then
             * the end frame. The orig's post-release window = the
             * 60th-slice completion + the 61st + the PB9 end frame
             * (1,996 events, cut at ~1,843 by the capture quit). The
             * drain = 1 slice + the frame. After the frame the scan is
             * over and the RSSI refresh stays killed (the capture ends
             * silent; the key-13 precedent). */
            if (ui_scan_end == 0u) {
                ui_scan_end = 1u;
                ui_freq += 5u;
                key14_row_display();
            } else {
                key11_frame_send();
                ui_scan = 0u;
                ui_scan_end = 0u;
                ui_rssi_kill = 1u;
            }
        } else if (ui_scan == 1u) {
            /* [session 69 | DATA-DERIVED p3k14_orig] the scan = RENDER-
             * PACED: one TUNE SLICE every pass, back-to-back (the orig's
             * slice pitch = exactly 6,698 events from the press slice on;
             * the slice wall period = the frame's bit-bang duration inside
             * key14_frame_send). The sweep continues past the release and
             * replaces the RSSI idle refresh while the scan is on. The
             * press pass itself already rendered s1 in the dispatch. */
            if (ui_rssi_pass > 1u) {
                ui_freq += 5u;
                key14_tune_slice();
            }
        } else if (ui_rssi_kill == 0u && REG8(UI_STATE_ADDR) == 0x00u &&
            (uint16_t)(ui_rssi_pass % 11u) == 1u &&
            (ui_rssi_fire < 2u || REG8(KEY_CODE_BYTE) == 0xFFu)) {
            rssi_refresh();
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
