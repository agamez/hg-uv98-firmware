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
