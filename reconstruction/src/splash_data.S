/* Boot splash image — [RECOVERED-DATA, session 52]: the original's
 * 128x160 RGB565 bitmap blitted by the boot scene (FUN_0801B738 ->
 * FUN_0801D334(0,0,0x80,0xA0, 0x08004130, 0xA000)). Extracted from the
 * original image at file offset 0x130 (the flash-mapped asset region
 * 0x08004130). 40960 bytes = 20480 pixels. */
.section .rodata.splash, "a"
.align 2
.global ui_splash_image
ui_splash_image:
    .incbin "data/splash_128x160_rgb565.bin"

/* ASCII 8x16 glyph table — [RECOVERED-DATA, session 52]: the class-3
 * font at 0x080413CC (the base DAT_080290C8), 16 bytes per char for
 * 0x20..0x7E. Column-major: bytes 0-7 = the columns of the top 8 rows,
 * bytes 8-15 = the bottom 8 rows; pixel(row, col) = bit (row&7) of
 * s[col] / s[8+col]. Rendered by FUN_0801BFB8's class-3 path. */
.section .rodata.font, "a"
.align 2
.global ui_ascii8x16_font
ui_ascii8x16_font:
    .incbin "data/ascii8x16_font.bin"

/* ASCII 6x10 glyph table — [RECOVERED-DATA, session 52]: the class-1
 * font at 0x08041F00 (the base DAT_080290D0), 12 bytes per char.
 * Column-major: bytes 0-5 = the columns of the top 8 rows (the LSB =
 * the row 0), bytes 6-11 = the columns of the bottom 2 rows (the bits
 * 0-1). Rendered by FUN_0801BFB8's class-1 path (the window
 * y..y+9 x..x+5, 60 pixels). */
.section .rodata.font1, "a"
.align 2
.global ui_ascii6x10_font
ui_ascii6x10_font:
    .incbin "data/ascii6x10_font.bin"

/* ASCII 12x20 glyph table — [RECOVERED-DATA, session 52]: the class-6
 * font at 0x0803F490 (the base DAT_080290E0), 36 bytes per char. The
 * column-major bands: the bytes 0-11 = the rows 0-7, 12-23 = the rows
 * 8-15, 24-35 = the rows 16-19 (the bits 0-3). The big frequency
 * digits. */
.section .rodata.font6, "a"
.align 2
.global ui_ascii12x20_font
ui_ascii12x20_font:
    .incbin "data/big12x20_font.bin"

/* LED-matrix stream replay table — [DATA-DERIVED, session 53]: the
 * exact 4,458-event PB9 sequence emitted by the original between the
 * status-page cells and the RX-bar window (2,966 BRR + 1,488 BSRR +
 * 4 CRH direction switches). Two bits per event: 0 = BRR(0x200),
 * 1 = BSRR(0x200), 2 = CRH 0xBBB44444 (input), 3 = CRH 0xBBB44434
 * (output). The generating call structure is an open item. */
.section .rodata.ledstream, "a"
.align 2
.global ui_led_stream
ui_led_stream:
    .incbin "data/led_stream.bin"
