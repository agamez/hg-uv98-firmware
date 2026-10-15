/* Display subsystem — see display.h for provenance. */
#include "display.h"
#include "font.h"
#include "hwregs.h"

/* ---- transport (SCAFFOLD) --------------------------------------------------
 * The original drives the panel through GPIOB/GPIOC control lines plus a
 * data bus (GPIOA/GPIOC show the heaviest literal usage: 17/16 references).
 * The exact bus width and pin map are not yet disentangled from the
 * stripped init; the reconstruction exposes the documented command/data
 * primitives over a stubbed transport. */
static void tft_write_cmd(uint8_t cmd)
{
    (void)cmd;    /* panel bus wiring is an open question (research log) */
}

static void tft_write_data(uint16_t v)
{
    (void)v;
}

/* [0x080179C8 set-window | RECOVERED via call context of 0x08018D32] */
void display_set_window(uint16_t x0, uint16_t y0, uint16_t x1, uint16_t y1)
{
    tft_write_cmd(0x2A);            /* column address set (ILI9341 family)  */
    tft_write_data(x0); tft_write_data(x1);
    tft_write_cmd(0x2B);            /* row address set                      */
    tft_write_data(y0); tft_write_data(y1);
    tft_write_cmd(0x2C);            /* memory write — confirmed constant    */
}

/* [0x08018D32 | RECOVERED]: compute x1=x+w-1, y1=y+h-1, set window, fill. */
void display_draw_area(uint16_t x, uint16_t y, uint16_t w, uint16_t h, uint16_t color)
{
    display_set_window(x, y, (uint16_t)(x + w - 1), (uint16_t)(y + h - 1));
    for (uint32_t i = 0; i < (uint32_t)w * h; i++)
        tft_write_data(color);
}

/* [0x08018B60 | RECOVERED logic; CJK glyph source now RECOVERED via
 * font.c (SPI2 flash), ASCII glyphs still SCAFFOLD] */
void display_draw_text(uint16_t x, uint16_t y, const char *s, uint8_t style)
{
    (void)style;    /* style byte rides the 5-arg original call signature    */
    (void)y;        /* the original's renderer keeps y in its cursor state   */
    while (*s && *s != (char)0xFF) {          /* terminators: 0 and 0xFF    */
        if ((uint8_t)*s < 0x81u) {
            uint8_t ch = (uint8_t)*s;
            if (ch < 10u)
                ch = (uint8_t)(ch + '0');     /* binary-digit bias to ASCII  */
            (void)ch;
            /* ASCII cell: 8 px advance; glyph blit is a SCAFFOLD pending
             * identification of the ASCII bitmap source (likely also in
             * the external SPI flash; see font.c header). */
            x = (uint16_t)(x + 8);
            s++;
        } else {
            /* GB2312 lead byte: consume two bytes, 16 px cell.
             * [RECOVERED 0x08018914 -> 0x08024E78]: fill a 32-byte buffer
             * from the external font flash, then blit 16x16. The blit
             * itself (orig 0x08017D3C) rides on the SCAFFOLD transport;
             * glyph bytes now come from the recovered fetcher. */
            if (s[1] == 0 || s[1] == (char)0xFF)
                break;
            /* volatile: the blit transport is SCAFFOLD, so without this
             * the fetch is dead code and gets elided; the original's
             * blit (0x08017D3C) consumes these bytes row by row. */
            volatile uint8_t glyph[32];
            if (cjk16_fetch((uint8_t *)glyph, (const uint8_t *)s) == 0) {
                for (uint32_t i = 0; i < 32; i++)
                    (void)glyph[i];           /* blit pending: rows to follow */
            }
            x = (uint16_t)(x + 16);
            s += 2;
        }
    }
}

void display_init(void)
{
    /* SCAFFOLD: panel identification/reset sequence not yet recovered. */
}
