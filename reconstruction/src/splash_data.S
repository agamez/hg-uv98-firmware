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
