/* Display subsystem — color TFT reconstruction.
 *
 * Color-LCD evidence [RECOVERED, high confidence]:
 *   - area-draw function 0x08018D32 computes x1=x+w-1 / y1=y+h-1 and calls
 *     a window setter then a command write of 0x2C — the standard
 *     ILI9341-family "memory write" command (research log session 6).
 *   - text renderer 0x08018B60 iterates a string: bytes 0 and 0xFF are
 *     terminators, values < 0x81 are single-byte cells advancing the
 *     cursor by 8 px (digits < 0x0A are biased by +0x30 into the font
 *     table), values >= 0x81 are GB2312 lead bytes consuming two bytes and
 *     advancing 16 px (16x16 Chinese glyph cells).
 *   - the font/bitmap assets live in the image's data region
 *     (0x08000130-0x0800B000, byte-identical across builds) and are NOT
 *     reproduced here: glyph blitting is a SCAFFOLD against a placeholder
 *     font interface.
 */
#ifndef DISPLAY_H
#define DISPLAY_H

#include <stdint.h>

void display_init(void);                       /* [0x0801BB22] recovered init  */
void display_set_window(uint16_t x0, uint16_t y0, uint16_t x1, uint16_t y1);
void display_draw_area(uint16_t x, uint16_t y, uint16_t w, uint16_t h, uint16_t color);
/* display_draw_text: DELETED session 74 — the session-13 stub was the
   wrong shape; the canonical mirrors = display_draw_text_cls (the full
   cb60) and the ui.c ui_draw_text (the verified subset). */
void display_led_write(uint16_t word, uint8_t flag);   /* [0x0802883A] LED link */
void display_fill_rect(uint8_t y, uint8_t x, uint8_t cols, uint8_t rows,
                       uint16_t color);                /* [0x0801CD32]         */
void display_blit_image(uint8_t y, uint8_t x, uint8_t rows, uint8_t cols,
                        const uint8_t *img, uint32_t len);   /* [0x0801D334] */
void display_blit_glyph_cell(uint8_t y, uint8_t x, const uint8_t *rec,
                             uint16_t fg, uint16_t bg);  /* [0x0801BFB8 cl.3] */
void display_blit_glyph_cell_6x10(uint8_t y, uint8_t x, const uint8_t *rec,
                                  uint16_t fg, uint16_t bg); /* [cl.1]      */
void display_blit_icon_16x16(uint8_t y, uint8_t x, const uint8_t *rec,
                             uint16_t fg, uint16_t bg);  /* [pt8 icon]  */
void display_blit_glyph_cell_12x20(uint8_t y, uint8_t x, const uint8_t *rec,
                                   uint16_t fg, uint16_t bg); /* [cl.6]   */
void display_draw_text_cls(uint8_t y, uint8_t x, const uint8_t *s, uint8_t ink,
                           uint8_t cls, uint8_t inv);  /* [0x0801CB60] */
void display_blit_bitmap_1bpp(uint8_t y, uint8_t x, uint8_t w, uint8_t h,
                              const uint8_t *bits, uint16_t fg, uint16_t bg);
/* [session 75 | the warm artifacts DELETED: display_warm_radio_window,
 * display_warm_1f_frame, display_ee359_replay — the session-53
 * broken-EE-era emissions; the fixed-EE warm boot emits none of them.
 * See ui.c's session-75 note and rf.c's at1846s_warm_slot_apply /
 * at1846s_warm_reinit for the banked decode.] */
void display_vendor_replay(void);       /* [DATA-DERIVED, 2507 events, s55; the measured justification s74] */

#endif /* DISPLAY_H */
