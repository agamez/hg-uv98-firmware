/* Font & asset storage — recovered SPI2-flash reader and record tables.
 * See font.c for the full provenance and addressing math. */
#ifndef FONT_H
#define FONT_H

#include <stdint.h>

void           font_flash_read(uint32_t addr24, uint32_t len, uint8_t *dst);
int            cjk16_fetch(uint8_t *buf, const uint8_t *gb);   /* 0 = filled */
const uint8_t *asset_table_get(uint8_t class, uint8_t ch);

#endif /* FONT_H */
