/* Font & asset storage — [RECOVERED] from the original image (session 11).
 *
 * ARCHITECTURE: the CJK 16x16 glyphs live in an EXTERNAL SPI flash on
 * hardware SPI2 (0x40003800), CS on GPIOD pin 4. Reads use flash READ
 * opcode 0x03 with a 24-bit address. Internal flash holds record tables
 * (menu strings, DCS tone names, APRS symbol names/icons) addressed by
 * the asset lookup below.
 *
 * Recovered functions (image labels; physical flash = label + 0x4000):
 *   spi2_xfer        0x08027E92  poll TXE, write DR, poll RXNE, read DR
 *   font_flash_read  0x08024F28  CS low, 0x03, addr24 MSB-first, read len
 *   cjk16_fetch      0x08024E78  HZK16-style lookup into SPI flash
 *   asset_table_get  0x08024F8C  class-indexed record table lookup
 *   blit (caller)    0x08017FB8  uses 0xBEEF as an internal sentinel
 *
 * cjk16_fetch addressing math (verified in the disassembly):
 *   reject unless hi in [0xB0, 0xF8) and lo >= 0xA1 (else fallback at
 *   0x08024ED4, body not yet recovered — returns without filling)
 *   addr24 = ((hi - 0xB0) * 94 + (lo + 685)) << 5
 *   i.e. HZK16 layout whose array starts 846 glyphs (9 rows of 94)
 *   before GB2312 zone-16 row 0, so zones 7..15 (symbols) are included.
 *
 * asset_table_get(class, *ch) returns a pointer into the class's record
 * table: record = base + (ch - 0x20) * stride, except class 4 and 7 map
 * ' ' -> slot 0, '.' -> fixed slot, '-' -> fixed slot, else (ch-0x2D).
 * Content by class (verified by dumping):
 *   1 @0x08041F00 12B/record  (bitmap-ish, packing TBD)
 *   2 @0x080419CC 14B/record  menu strings ("MEM-CH:", "DEL-CH:", ...)
 *   3 @0x080413CC 16B/record  DCS tone names ("D371N", "D411N", ...)
 *   4 @0x08042410 20B/record  (bitmap-ish)
 *   5 @0x080401EC 24B/record  APRS symbol names ("$  Phone", "/  Red Dot")
 *   6 @0x0803F490 36B/record  APRS symbol names ext ("t  Truck Stop", ...)
 *   7 @0x080421C4 45B/record  (bitmap-ish)
 *   8 @0x0803DF68 57B/record  bitmap
 *   9 @0x0803C7A8 64B/record  bitmap
 * The actual ASCII text bitmap font was NOT found in internal flash; the
 * second SPI block reader (0x08024E10, split-arg form) suggests text
 * glyphs are fetched from the external flash as well (open follow-up).
 */
#include <stdint.h>

#define SPI2_BASE       0x40003800u
#define SPI2_SR         (*(volatile uint16_t *)(SPI2_BASE + 0x08u))
#define SPI2_DR         (*(volatile uint16_t *)(SPI2_BASE + 0x0Cu))
#define SPI_SR_TXE      (1u << 1)
#define SPI_SR_RXNE     (1u << 0)

#define GPIOD_BASE      0x40011400u
#define FONT_FLASH_CS   0x10u                   /* PD4 */

/* [0x08027E92 | RECOVERED] full-duplex SPI2 byte transfer. */
static uint8_t spi2_xfer(uint8_t data)
{
    while (!(SPI2_SR & SPI_SR_TXE))
        ;
    SPI2_DR = data;
    while (!(SPI2_SR & SPI_SR_RXNE))
        ;
    return (uint8_t)SPI2_DR;
}

/* GPIOD BSRR-style pin set/clear — mirrors the original's pin helpers
 * 0x0801F780 (set) / 0x0801F788 (clear), which wrap the same registers. */
static void cs_assert(void)   { *(volatile uint32_t *)(GPIOD_BASE + 0x10u) = FONT_FLASH_CS; }
static void cs_deassert(void) { *(volatile uint32_t *)(GPIOD_BASE + 0x14u) = FONT_FLASH_CS; }

/* [0x08024F28 | RECOVERED] read len bytes at addr24 from the font flash. */
void font_flash_read(uint32_t addr24, uint32_t len, uint8_t *dst)
{
    cs_assert();
    spi2_xfer(0x03);
    spi2_xfer((uint8_t)(addr24 >> 16));
    spi2_xfer((uint8_t)(addr24 >> 8));
    spi2_xfer((uint8_t)addr24);
    for (uint32_t i = 0; i < len; i++)
        dst[i] = spi2_xfer(0xFF);
    cs_deassert();
}

/* [0x08024E78 | RECOVERED] fetch the 32-byte 16x16 glyph for a GB2312
 * character (gb[0] = high byte, gb[1] = low byte) into buf.
 * Returns 0 if filled, non-zero if the code is outside the covered
 * range (original falls through to 0x08024ED4 — body not recovered). */
int cjk16_fetch(uint8_t *buf, const uint8_t *gb)
{
    uint32_t hi = gb[0], lo = gb[1];

    if (hi < 0xB0u || hi >= 0xF8u || lo < 0xA1u)
        return 1;
    uint32_t addr24 = (((hi - 0xB0u) * 94u) + (lo + 685u)) << 5;
    font_flash_read(addr24, 32, buf);
    return 0;
}

/* [0x08024F8C | RECOVERED] record-table lookup. class 1..9 selects the
 * table; ch is the character byte (see header comment for slot rules).
 * Returns the record pointer, or 0 for an unknown class. */const uint8_t *asset_table_get(uint8_t class, uint8_t ch)
{
    uint32_t base, stride;

    switch (class) {
    case 1: base = 0x08041F00u; stride = 12; break;
    case 2: base = 0x080419CCu; stride = 14; break;
    case 3: base = 0x080413CCu; stride = 16; break;
    case 4:                                     /* ' ' '.' '-' special */
        if (ch == 0x20u) return (const uint8_t *)0x08042410u;
        if (ch == 0x2Eu) return (const uint8_t *)(0x08042410u + 0x14u);
        if (ch == 0x2Du) return (const uint8_t *)(0x08042410u + 0x28u);
        base = 0x08042410u; stride = 20;
        return (const uint8_t *)(base + (uint32_t)(ch - 0x2Du) * stride);
    case 5: base = 0x080401ECu; stride = 24; break;
    case 6: base = 0x0803F490u; stride = 36; break;
    case 7:                                     /* ' ' '.' '-' special */
        if (ch == 0x20u) return (const uint8_t *)0x080421C4u;
        if (ch == 0x2Eu) return (const uint8_t *)(0x080421C4u + 0x2Du);
        if (ch == 0x2Du) return (const uint8_t *)(0x080421C4u + 0x5Au);
        base = 0x080421C4u; stride = 45;
        return (const uint8_t *)(base + (uint32_t)(ch - 0x2Du) * stride);
    case 8: base = 0x0803DF68u; stride = 57; break;
    case 9: base = 0x0803C7A8u; stride = 64; break;
    default:
        return 0;
    }
    return (const uint8_t *)(base + (uint32_t)(ch - 0x20u) * stride);
}

/* ------------------------------------------------------------------------ *
 * ASCII fonts — also in the EXTERNAL SPI flash.
 *
 * [0x08027EC8 | RECOVERED] fetches one glyph for a printable ASCII char
 * (0x20..0x7E, else returns 0). Class = font set; geometry per class:
 *
 *   class 1:  8x8  (stride  8)  base 0x03BEC0
 *   class 2:  8x8  (stride  8)  base 0x0065C0
 *   class 3:  8x16 (stride 16)  base 0x03B5C0
 *   class 4:  8x16 (stride 16)  base 0x03CD80
 *   class 5: 16x16 (stride 34, 32 used) base 0x03BE82
 *   class 6: 16x16 (stride 34, 32 used) base 0x03D142
 *
 * This resolves O5 fully: ALL text glyphs (CJK 16x16 and ASCII 8x8/8x16/
 * 16x16) live in the external SPI flash; the internal-flash record
 * tables (asset_table_get) hold icons and strings, not text fonts.
 * ------------------------------------------------------------------------ */
int ascii_font_fetch(uint8_t *dst, int class, uint8_t ch)
{
    uint32_t addr;

    if (ch < 0x20u || ch > 0x7Eu)
        return 0;
    switch (class) {
    case 1: addr = 0x03BEC0u + (uint32_t)ch * 8u;  font_flash_read(addr, 8,  dst); break;
    case 2: addr = 0x0065C0u + (uint32_t)ch * 8u;  font_flash_read(addr, 8,  dst); break;
    case 3: addr = 0x03B5C0u + (uint32_t)ch * 16u; font_flash_read(addr, 16, dst); break;
    case 4: addr = 0x03CD80u + (uint32_t)ch * 16u; font_flash_read(addr, 16, dst); break;
    case 5: addr = 0x03BE82u + (uint32_t)ch * 34u; font_flash_read(addr, 32, dst); break;
    case 6: addr = 0x03D142u + (uint32_t)ch * 34u; font_flash_read(addr, 32, dst); break;
    default:
        return 0;
    }
    return 1;
}
