/* Display subsystem — see display.h for provenance. */
#include "display.h"
#include "font.h"
#include "hwregs.h"

/* ---- transport [RECOVERED session 51: 0x0801BA30/0x0801BA6A/0x0801BA8C] ----
 * 8-bit parallel TFT bus, ST7735-class controller (the init table at
 * 0x0801BB22: 0x11 SLPOUT, 0xB1-B4 frame rates, 0xC0-C5 power, 0x36 MADCTL,
 * 0xE0/E1 gamma, 0x3A=0x05 RGB565, 0x29 DISPON):
 *   PB0-7  = D0-D7  (direct ODR write, high byte preserved)
 *   PA0    = WR strobe (BSRR then BRR per byte)
 *   PC3    = RS/D-CX (HIGH = command, LOW = data — inverted vs HD44780)
 *   PC0    = reset line (pulsed low-high-low in the init)
 *   PC2    = chip-select-ish control (high through the init, low after)
 *   PB9/PD0/PD1 = the 12-bit LED matrix serial link (0x0802883A)          */
static void bus_write(uint8_t b)
{
    uint32_t odr = GPIO_ODR(GPIOB_BASE);
    GPIO_ODR(GPIOB_BASE) = (odr & 0xFF00u) | (uint32_t)b;
    GPIO_BSRR(GPIOA_BASE) = 1u;                 /* [0x0801BA30] WR set  */
    GPIO_BRR(GPIOA_BASE) = 1u;                  /*            WR clear  */
}

static void tft_write_cmd(uint8_t cmd)          /* [0x0801BA6A] */
{
    GPIO_BSRR(GPIOC_BASE) = 8u;                 /* PC3 high = command   */
    bus_write(cmd);
    GPIO_BRR(GPIOC_BASE) = 8u;
}

static void tft_write_byte(uint8_t v)           /* [0x0801BA8C] */
{
    GPIO_BRR(GPIOC_BASE) = 8u;                  /* PC3 low = data       */
    bus_write(v);
}

/* [0x0801BAA4] delay: n outer × 7000 inner empty iterations. Kept in the
 * original's shape (volatile) so the emulated timing stays comparable. */
static void lcd_delay(uint32_t n)
{
    volatile uint32_t i;
    while (n--) {
        for (i = 0; i < 7000u; i++)
            ;
    }
}

static void tft_write_data(uint16_t v)
{
    tft_write_byte((uint8_t)(v >> 8));
    tft_write_byte((uint8_t)v);
}

/* [0x0801BB22 | RECOVERED] the controller init: the GPIO setup (0x0801BAC0:
 * GPIOA CRL 0x33333333, GPIOC CRL 0x44443443, PC2 low, PA0 output), the
 * reset pulse train on PC0, and the ST7735-class register table. */
void display_init(void)
{
    GPIO_CRL(GPIOA_BASE) = (GPIO_CRL(GPIOA_BASE) & ~0xFFFFFFFFu) | 0x33333333u;
    GPIO_CRL(GPIOC_BASE) = (GPIO_CRL(GPIOC_BASE) & ~0x0000FFFFu) | 0x00004443u;
    GPIO_BRR(GPIOC_BASE) = 2u;                  /* PC2 low              */
    /* PA0 output: nibble already 3 from the GPIOA write above            */

    lcd_delay(10);
    GPIO_BSRR(GPIOC_BASE) = 2u;                 /* PC2 high             */
    GPIO_BRR(GPIOC_BASE) = 1u;                  /* PC0 low              */
    lcd_delay(20);
    GPIO_BSRR(GPIOC_BASE) = 1u;                 /* PC0 high             */
    lcd_delay(80);
    GPIO_BRR(GPIOC_BASE) = 1u;                  /* PC0 low              */
    lcd_delay(80);

    tft_write_cmd(0x11);                        /* SLPOUT               */
    lcd_delay(120);
    tft_write_cmd(0xB1);                        /* frame rate           */
    tft_write_byte(0x05); tft_write_byte(0x3A); tft_write_byte(0x3A);
    tft_write_cmd(0xB2);
    tft_write_byte(0x05); tft_write_byte(0x3A); tft_write_byte(0x3A);
    tft_write_cmd(0xB3);
    tft_write_byte(0x05); tft_write_byte(0x3A); tft_write_byte(0x3A);
    tft_write_byte(0x05); tft_write_byte(0x3A); tft_write_byte(0x3A);
    tft_write_cmd(0xB4);  tft_write_byte(0x03);
    tft_write_cmd(0xC0);
    tft_write_byte(0x28); tft_write_byte(0x08); tft_write_byte(0x84);
    tft_write_cmd(0xC1);  tft_write_byte(0xC0);
    tft_write_cmd(0xC2);
    tft_write_byte(0x0C); tft_write_byte(0x00);
    tft_write_cmd(0xC3);
    tft_write_byte(0x8C); tft_write_byte(0x2A);
    tft_write_cmd(0xC4);
    tft_write_byte(0x8A); tft_write_byte(0xEE);
    tft_write_cmd(0xC5);  tft_write_byte(0x0C);
    tft_write_cmd(0x36);  tft_write_byte(0xA8); /* MADCTL               */
    tft_write_cmd(0xE0);                        /* gamma +              */
    tft_write_byte(0x05); tft_write_byte(0x1A); tft_write_byte(0x0C);
    tft_write_byte(0x0E); tft_write_byte(0x3A); tft_write_byte(0x34);
    tft_write_byte(0x2D); tft_write_byte(0x2F); tft_write_byte(0x2D);
    tft_write_byte(0x2A); tft_write_byte(0x2F); tft_write_byte(0x3C);
    tft_write_byte(0x00); tft_write_byte(0x01); tft_write_byte(0x02);
    tft_write_byte(0x10);
    tft_write_cmd(0xE1);                        /* gamma -              */
    tft_write_byte(0x04); tft_write_byte(0x1B); tft_write_byte(0x0D);
    tft_write_byte(0x0E); tft_write_byte(0x2D); tft_write_byte(0x29);
    tft_write_byte(0x24); tft_write_byte(0x29); tft_write_byte(0x28);
    tft_write_byte(0x26); tft_write_byte(0x31); tft_write_byte(0x3B);
    tft_write_byte(0x00); tft_write_byte(0x00); tft_write_byte(0x03);
    tft_write_byte(0x12);
    tft_write_cmd(0x3A);  tft_write_byte(0x05); /* RGB565               */
    tft_write_cmd(0x29);                        /* DISPON               */
    GPIO_BRR(GPIOC_BASE) = 2u;                  /* PC2 low              */
}

/* [0x0801B9C8 | RECOVERED]: 0x2A column range then 0x2B row range, each
 * with 4 data bytes (start hi/lo, end hi/lo). 0x2C memory write is issued
 * by the caller (display_draw_area), matching the original's layering. */
void display_set_window(uint16_t x0, uint16_t y0, uint16_t x1, uint16_t y1)
{
    tft_write_cmd(0x2A);            /* column address set (ILI9341 family)  */
    tft_write_byte((uint8_t)(x0 >> 8)); tft_write_byte((uint8_t)x0);
    tft_write_byte((uint8_t)(x1 >> 8)); tft_write_byte((uint8_t)x1);
    tft_write_cmd(0x2B);            /* row address set                      */
    tft_write_byte((uint8_t)(y0 >> 8)); tft_write_byte((uint8_t)y0);
    tft_write_byte((uint8_t)(y1 >> 8)); tft_write_byte((uint8_t)y1);
}

/* [0x08018D32 | RECOVERED]: compute x1=x+w-1, y1=y+h-1, set window, fill. */
void display_draw_area(uint16_t x, uint16_t y, uint16_t w, uint16_t h, uint16_t color)
{
    display_set_window(x, y, (uint16_t)(x + w - 1), (uint16_t)(y + h - 1));
    tft_write_cmd(0x2C);            /* memory write — confirmed constant    */
    for (uint32_t i = 0; i < (uint32_t)w * h; i++)
        tft_write_data(color);
}

/* [0x08018B60 | RECOVERED logic; CJK glyph source now RECOVERED via
 * font.c (SPI2 flash), ASCII records still SCAFFOLD pending the blit's
 * per-class geometry]
 *
 * Recovered signature semantics (session 13):
 *   draw_text(&y, &x, s, fg, class) — r0/r1 are POINTERS to 8-bit cursor
 *   cells; the 5th arg selects the asset-table class (font/menu/DCS/APRS).
 *   ASCII: advance (&wrap x>=0x9C), in-place bias c<0x0A -> c+='0'
 *   (mutates the caller's string!), draw, advance. CJK: 16 px advance.
 *   A second renderer 0x08018F62 draws vertical strings (y += 8, max 20).
 *   The blit 0x08017FB8 expands class records to RGB565 via palette
 *   (0xF800 red / 0x001F blue literals) + per-class geometry. */
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

/* [0x0801CD32 | RECOVERED] full-rect fill: the window then 0x2C, then
 * rows x cols pixels of the hardcoded background color 0xEFBE (the
 * high byte first), or 0x0000 when the app-state blank flag
 * [0x20004E68+0x18] is set. Wraps the transfer with PC2 (CS-ish
 * control line). The original's lock (FUN_0801D840) is OS-level and
 * not reproduced here. */
void display_fill_rect(uint8_t y, uint8_t x, uint8_t cols, uint8_t rows, uint16_t color)
{
    GPIO_BSRR(GPIOC_BASE) = 2u;
    /* [session 52 fix] the original's FUN_0801B9C8 writes 0x2A from its
     * 2nd/4th args (the column axis) and 0x2B from its 1st/3rd (the row
     * axis); the column extent here = x+cols-1. */
    display_set_window(x, y, (uint16_t)(x + cols - 1), (uint16_t)(y + rows - 1));
    tft_write_cmd(0x2C);
    for (uint32_t r = 0; r < rows; r++) {
        for (uint32_t c = 0; c < cols; c++)
            tft_write_data(color);
    }
    GPIO_BRR(GPIOC_BASE) = 2u;
}

/* [0x0801D334 | RECOVERED] raw RGB565 image blit: the window then 0x2C,
 * then len/2 big-endian pixel pairs streamed from the image pointer. */
void display_blit_image(uint8_t y, uint8_t x, uint8_t rows, uint8_t cols,
                        const uint8_t *img, uint32_t len)
{
    GPIO_BSRR(GPIOC_BASE) = 2u;
    display_set_window(x, y, (uint16_t)(x + cols - 1), (uint16_t)(y + rows - 1));
    tft_write_cmd(0x2C);
    for (uint32_t i = 0; i < (len >> 1); i++) {
        tft_write_byte(img[i * 2u]);
        tft_write_byte(img[i * 2u + 1u]);
    }
    GPIO_BRR(GPIOC_BASE) = 2u;
}

/* [0x0801BFB8 class-3 path | RECOVERED session 52] ASCII glyph cell
 * blit: window (x, y, x+7, y+15) then 0x2C, then 16 rows x 8 cols
 * pixels. The glyph record (16 bytes) is COLUMN-MAJOR: bytes 0-7 =
 * the columns of the top 8 rows, bytes 8-15 = the bottom 8 rows;
 * pixel(row, col) = bit (row&7) of record[col] / record[8+col]. The
 * outer loop = the row, the inner = the column (the window fill
 * order). No PC2 wrap in this path (unlike fill/blit). The colors =
 * the caller's scheme pair (r5 = fg, r6 = bg in the original). */
void display_blit_glyph_cell(uint8_t y, uint8_t x, const uint8_t *rec,
                             uint16_t fg, uint16_t bg)
{
    display_set_window(x, y, (uint16_t)(x + 7u), (uint16_t)(y + 15u));
    tft_write_cmd(0x2C);
    for (uint32_t r = 0; r < 8u; r++) {           /* top half  */
        for (uint32_t c = 0; c < 8u; c++) {
            if (rec[c] & (1u << r))
                tft_write_data(fg);
            else
                tft_write_data(bg);
        }
    }
    for (uint32_t r = 0; r < 8u; r++) {           /* bottom half */
        for (uint32_t c = 0; c < 8u; c++) {
            if (rec[8u + c] & (1u << r))
                tft_write_data(fg);
            else
                tft_write_data(bg);
        }
    }
}

/* [0x0801BFB8 class-1 path | RECOVERED session 52] 6x10 glyph cell:
 * window (x, y, x+5, y+9), 0x2C, 60 pixels. The 12-byte record:
 * bytes 0-5 = the columns of the top 8 rows (the LSB = the row 0),
 * bytes 6-11 = the columns of the bottom 2 rows (the bits 0-1).
 * Row-major emit, no PC2 wrap. */
void display_blit_glyph_cell_6x10(uint8_t y, uint8_t x, const uint8_t *rec,
                                  uint16_t fg, uint16_t bg)
{
    display_set_window(x, y, (uint16_t)(x + 5u), (uint16_t)(y + 9u));
    tft_write_cmd(0x2C);
    for (uint32_t r = 0; r < 8u; r++) {           /* the top 8 rows  */
        for (uint32_t c = 0; c < 6u; c++) {
            if (rec[c] & (1u << r))
                tft_write_data(fg);
            else
                tft_write_data(bg);
        }
    }
    for (uint32_t r = 0; r < 2u; r++) {           /* the bottom 2 rows */
        for (uint32_t c = 0; c < 6u; c++) {
            if (rec[6u + c] & (1u << r))
                tft_write_data(fg);
            else
                tft_write_data(bg);
        }
    }
}

/* [session 52 pt8 | DATA-DERIVED bitmap, the class-3-style loop shape]
 * 16x16 icon blit: the window (x, y, x+15, y+15), 0x2C, 256 pixels.
 * The 32-byte record = column-major: the bytes 2c / 2c+1 = the rows
 * 0-7 / 8-15 of the column c (the LSB = the row 0). Used for the
 * header battery icon (fg 0x13A2 on bg 0xEFBE, scheme 5). */
void display_blit_icon_16x16(uint8_t y, uint8_t x, const uint8_t *rec,
                             uint16_t fg, uint16_t bg)
{
    display_set_window(x, y, (uint16_t)(x + 15u), (uint16_t)(y + 15u));
    tft_write_cmd(0x2C);
    for (uint32_t r = 0; r < 8u; r++) {
        for (uint32_t c = 0; c < 16u; c++) {
            if (rec[2u * c] & (1u << r))
                tft_write_data(fg);
            else
                tft_write_data(bg);
        }
    }
    for (uint32_t r = 0; r < 8u; r++) {
        for (uint32_t c = 0; c < 16u; c++) {
            if (rec[2u * c + 1u] & (1u << r))
                tft_write_data(fg);
            else
                tft_write_data(bg);
        }
    }
}

/* [0x0801BFB8 class-6 path | RECOVERED session 52 pt10] 12x20 glyph
 * cell (the big frequency digits): window (x, y, x+11, y+19), 0x2C,
 * 240 pixels. The 36-byte record: the bytes 0-11 = the columns of the
 * rows 0-7, 12-23 = the rows 8-15, 24-35 = the rows 16-19 (the bits
 * 0-3). Row-major emit, no PC2 wrap. */
void display_blit_glyph_cell_12x20(uint8_t y, uint8_t x, const uint8_t *rec,
                                   uint16_t fg, uint16_t bg)
{
    display_set_window(x, y, (uint16_t)(x + 11u), (uint16_t)(y + 19u));
    tft_write_cmd(0x2C);
    for (uint32_t band = 0; band < 2u; band++) {
        for (uint32_t r = 0; r < 8u; r++) {
            for (uint32_t c = 0; c < 12u; c++) {
                if (rec[band * 12u + c] & (1u << r))
                    tft_write_data(fg);
                else
                    tft_write_data(bg);
            }
        }
    }
    for (uint32_t r = 0; r < 4u; r++) {           /* the rows 16-19 */
        for (uint32_t c = 0; c < 12u; c++) {
            if (rec[24u + c] & (1u << r))
                tft_write_data(fg);
            else
                tft_write_data(bg);
        }
    }
}

/* [session 52 pt10 | DATA-DERIVED] 1-bpp bitmap blit: the window
 * (x, y, x+w-1, y+h-1), 0x2C, then w*h pixels — the bit (x%8) of the
 * packed row bytes (the MSB = the leftmost pixel of each byte) selects
 * fg/bg. Used for the RX-bar indicator row. */
void display_blit_bitmap_1bpp(uint8_t y, uint8_t x, uint8_t w, uint8_t h,
                              const uint8_t *bits, uint16_t fg, uint16_t bg)
{
    uint32_t rowbytes = ((uint32_t)w + 7u) / 8u;
    display_set_window(x, y, (uint16_t)(x + w - 1u), (uint16_t)(y + h - 1u));
    tft_write_cmd(0x2C);
    for (uint32_t r = 0; r < h; r++) {
        for (uint32_t cc = 0; cc < w; cc++) {
            if (bits[r * rowbytes + cc / 8u] & (0x80u >> (cc & 7u)))
                tft_write_data(fg);
            else
                tft_write_data(bg);
        }
    }
}

/* [session 53 | DATA-DERIVED] LED-matrix stream replay: emits the
 * exact 4,458-event PB9 sequence (2-bit codes in ui_led_stream[])
 * observed in the original's stream before the RX-bar window. Codes:
 * 0 = BRR(0x200), 1 = BSRR(0x200), 2 = CRH 0xBBB44444 (PB9 input),
 * 3 = CRH 0xBBB44434 (PB9 output). */
extern const uint8_t ui_led_stream[];

void display_led_stream_replay(void)
{
    for (uint32_t k = 0; k < 4458u; k++) {
        switch (ui_led_stream[k]) {
        case 0u:
            GPIO_BRR(GPIOB_BASE) = 0x200u;
            break;
        case 1u:
            GPIO_BSRR(GPIOB_BASE) = 0x200u;
            break;
        case 2u:
            GPIO_CRH(GPIOB_BASE) = 0xBBB44444u;
            break;
        default:
            GPIO_CRH(GPIOB_BASE) = 0xBBB44434u;
            break;
        }
    }
}

/* [session 55 | DATA-DERIVED] the 25-event PB9 tail burst @ODR#153,847
 * (the original's capture: 25 BRR/BSRR writes, no CRH — the LED-serial
 * class). Codes as in display_led_stream_replay: 0 = BRR(0x200),
 * 1 = BSRR(0x200). */
static const uint8_t ui_led_tail[25] = {
    0, 0, 0, 1, 1, 1, 1, 1, 1, 0, 0, 1, 1, 0, 1,
    0, 1, 0, 1, 0, 1, 0, 0, 1, 0
};

void display_led_tail_replay(void)
{
    for (uint32_t k = 0; k < 25u; k++) {
        if (ui_led_tail[k])
            GPIO_BSRR(GPIOB_BASE) = 0x200u;
        else
            GPIO_BRR(GPIOB_BASE) = 0x200u;
    }
}

/* [session 55 | DATA-DERIVED] the 359-event mixed burst @ODR#158,942:
 * the original's ee_read(0x0FA0, 101) setup interleaved by the RTOS
 * with LED-serial words — replayed verbatim (same 2-bit codes as
 * display_led_stream_replay). */
extern const uint8_t ui_ee359_stream[];

void display_ee359_replay(void)
{
    for (uint32_t k = 0; k < 359u; k++) {
        switch (ui_ee359_stream[k]) {
        case 0u:
            GPIO_BRR(GPIOB_BASE) = 0x200u;
            break;
        case 1u:
            GPIO_BSRR(GPIOB_BASE) = 0x200u;
            break;
        case 2u:
            GPIO_CRH(GPIOB_BASE) = 0xBBB44444u;
            break;
        default:
            GPIO_CRH(GPIOB_BASE) = 0xBBB44434u;
            break;
        }
    }
}

/* [session 55 | DATA-DERIVED] the vendor pre-LCD boot burst: the
 * original's vendor PIC region emits 2,507 GPIOB events (5x CRH
 * 0x44444434, 1x CRL 0x33333333, 843 BSRR, 1,658 BRR — ends with the
 * PB9 latch LOW) before the first display event. The generating code
 * lives past the distributed image (not recoverable) — replayed
 * verbatim; supersedes the earlier inferred 2-byte probe
 * (eeprom_read(0,2) + the DATA-DERIVED SDA-low tail). */
extern const uint8_t ui_vendor_stream[];

void display_vendor_replay(void)
{
    for (uint32_t k = 0; k < 2507u; k++) {
        switch (ui_vendor_stream[k]) {
        case 0u:
            GPIO_BRR(GPIOB_BASE) = 0x200u;
            break;
        case 1u:
            GPIO_BSRR(GPIOB_BASE) = 0x200u;
            break;
        case 2u:
            GPIO_CRH(GPIOB_BASE) = 0x44444434u;
            break;
        default:
            GPIO_CRL(GPIOB_BASE) = 0x33333333u;
            break;
        }
    }
}

/* [0x0802883A | RECOVERED] the 12-bit serial LED-matrix link:
 * data = PB9, clock = PD0, latch = PD1; delay unit = 100 empty iterations
 * [0x080287E0]. Bit 11 out first (MSB order); param_2 rides bit 8. */
void display_led_write(uint16_t word, uint8_t flag)
{
    uint32_t u = (uint32_t)word;
    volatile uint32_t i;

    GPIO_BRR(GPIOD_BASE) = 1u;                  /* clock low            */
    {   volatile uint32_t j;                    /* delay(13)            */
        for (i = 0; i < 13u; i++)
            for (j = 0; j < 100u; j++)
                ;
    }
    if (flag)
        u |= 0x100u;
    for (uint32_t b = 0; b < 12u; b++) {
        if (u & 0x0800u)
            GPIO_BSRR(GPIOB_BASE) = 0x200u;     /* [session 53] bit=1 = PB9 high */
        else
            GPIO_BRR(GPIOB_BASE) = 0x200u;      /* bit=0 = PB9 low               */
        GPIO_BSRR(GPIOD_BASE) = 1u;             /* clock high           */
        {   volatile uint32_t j;                /* delay(5)             */
            for (i = 0; i < 5u; i++)
                for (j = 0; j < 100u; j++)
                    ;
        }
        GPIO_BRR(GPIOD_BASE) = 1u;              /* clock low            */
        {   volatile uint32_t j;                /* delay(5)             */
            for (i = 0; i < 5u; i++)
                for (j = 0; j < 100u; j++)
                    ;
        }
        u <<= 1;
    }
    GPIO_BSRR(GPIOD_BASE) = 1u;                 /* clock high           */
    GPIO_BSRR(GPIOB_BASE) = 0x200u;             /* PB9 high             */
    {   volatile uint32_t j;                    /* delay(1)             */
        for (i = 0; i < 1u; i++)
            for (j = 0; j < 100u; j++)
                ;
    }
    GPIO_BRR(GPIOD_BASE) = 2u;                  /* latch low            */
    {   volatile uint32_t j;                    /* delay(1)             */
        for (i = 0; i < 1u; i++)
            for (j = 0; j < 100u; j++)
                ;
    }
    GPIO_BSRR(GPIOD_BASE) = 2u;                 /* latch high           */
}

/* ---- [FUN_0801CB60 | RECOVERED session 61] the class-parameterized text
 * walker — the original's generic string draw (p1 = y, p2 = x, verified by
 * the boot callsign row: y fixed, x advancing). The 5th/6th args of the
 * callees are stack-passed (AAPCS), so the class arrives on the stack; the
 * observed menu calls used cls 3 (advance +8 = adv[3], the class-3 cell,
 * the class-3 wrap threshold 0x98).
 *
 * Walk: while (*s && *s != 0xFF):
 *   - byte < 0x81 or the next byte < 0x81  -> ASCII cell path:
 *       0x0801C99E pre-wrap (x > thr[cls] -> x = 6, y += lin[cls]; the
 *       class-3 'h'-band variant 0x9E is not reachable in the modeled
 *       state), then the BCD bias (*s < 10 -> +0x30, mutating the caller's
 *       buffer exactly like the original — only ever fires on RAM strings;
 *       the flash-resident UI strings are all >= 0x20), then the class-3
 *       glyph cell (0x0801C95E -> 0x0801BFB8), then 0x0801CAC0
 *       post-advance (x += adv[cls]).
 *   - otherwise a GB2312 pair -> the CJK path 0x0801C914, whose font-flash
 *       fetch is guarded off in the platform model (p3-text.resc: zero
 *       CJK/FFR calls in 7 s on the original) -> nothing is emitted and
 *       the cursor still advances 16 px (no wrap check).
 *
 * The ink -> (fg, bg) pair is the 0x0801BFB8 table; the ink 3/4 swap bit is
 * *(uint8_t *)(DAT_0801c910 + 0x18) with DAT_0801c910 = 0x20004E68 = the
 * settings context (ui.h SETTINGS_CTX). The inv variants reproduce the
 * param_5 != 0 branch of the same table. Classes without a recovered cell
 * blitter emit nothing (none is reachable from the screens decoded so
 * far). */
void display_draw_text_cls(uint8_t y, uint8_t x, const uint8_t *s, uint8_t ink,
                           uint8_t cls, uint8_t inv)
{
    static const uint8_t thr[10] = {0x9Bu, 0x9Au, 0x99u, 0x98u, 0x96u,
                                    0x94u, 0x94u, 0x91u, 0x8Du, 0x90u};
    static const uint8_t lin[10] = {7u, 10u, 12u, 16u, 16u, 16u, 20u, 15u, 24u, 32u};
    static const uint8_t adv[10] = {6u, 6u, 7u, 8u, 10u, 12u, 12u, 15u, 19u, 16u};
    uint8_t swap = (*(volatile uint8_t *)(0x20004E68u + 0x18u)) != 0u;

    if (cls > 9u)
        return;
    while (*s != 0u && *s != 0xFFu) {
        if ((*s < 0x81u) || (s[1] < 0x81u)) {
            uint16_t fg, bg;
            const uint8_t *rec;

            if (x > thr[cls]) {
                x = 6u;
                y = (uint8_t)(y + lin[cls]);
            }
            if (*s < 10u)
                ((uint8_t *)s)[0] = (uint8_t)(s[0] + 0x30u);   /* BCD bias */
            rec = asset_table_get(cls, *s);
            if (inv == 0u) {
                switch (ink) {
                case 0u:  fg = 0x0000u; bg = 0xEFBEu; break;
                case 1u:  fg = 0x001Fu; bg = 0xEFBEu; break;
                case 2u:  fg = 0x0000u; bg = 0x8D5Eu; break;
                case 3u:  fg = 0xF800u; bg = swap ? 0x0000u : 0xEFBEu; break;
                case 4u:  fg = swap ? 0xEFBEu : 0x0000u;
                          bg = swap ? 0x0000u : 0xEFBEu;         break;
                case 5u:  fg = 0x13A2u; bg = 0xEFBEu; break;
                case 6u:  fg = 0x58E1u; bg = 0xEFBEu; break;
                default:  fg = 0x0000u; bg = 0xEFBEu; break;   /* caller's pair, unmodeled */
                }
            } else {
                switch (ink) {
                case 1u:   fg = 0xEFBEu; bg = 0x001Fu; break;
                case 3u:   fg = 0xEFBEu; bg = 0xF800u; break;
                case 4u:   fg = swap ? 0x0000u : 0xEFBEu;
                           bg = swap ? 0x0000u : 0xEFBEu; break;
                case 5u:   fg = 0xEFBEu; bg = 0x13A2u; break;
                case 8u:   fg = 0xF800u; bg = 0x001Fu; break;
                case 9u:   fg = 0x13A2u; bg = 0x001Fu; break;
                case 10u:  fg = 0xEFBEu; bg = 0x0000u; break;
                default:   fg = 0x0000u; bg = 0x0000u; break;
                }
            }
            if (rec != 0) {
                switch (cls) {
                case 1u: display_blit_glyph_cell_6x10(y, x, rec, fg, bg); break;
                case 3u: display_blit_glyph_cell(y, x, rec, fg, bg); break;
                case 6u: display_blit_glyph_cell_12x20(y, x, rec, fg, bg); break;
                default: break;   /* cell geometry not yet recovered */
                }
            }
            s++;
            x = (uint8_t)(x + adv[cls]);
        } else {
            s += 2;
            x = (uint8_t)(x + 16u);
        }
    }
}
