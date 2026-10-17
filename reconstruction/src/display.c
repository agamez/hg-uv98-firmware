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
            GPIO_BRR(GPIOB_BASE) = 0x200u;      /* PB9 low              */
        else
            GPIO_BSRR(GPIOB_BASE) = 0x200u;     /* PB9 high             */
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
