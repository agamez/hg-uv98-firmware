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

void display_init(void);                       /* SCAFFOLD (panel ID unknown)   */
void display_set_window(uint16_t x0, uint16_t y0, uint16_t x1, uint16_t y1);
void display_draw_area(uint16_t x, uint16_t y, uint16_t w, uint16_t h, uint16_t color);
void display_draw_text(uint16_t x, uint16_t y, const char *s, uint8_t style);

#endif /* DISPLAY_H */
