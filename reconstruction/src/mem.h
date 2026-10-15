/* Freestanding memory helpers — see mem.c for provenance. */
#ifndef MEM_H
#define MEM_H

#include <stdint.h>

void    *mem_copy(void *dst, const void *src, uint32_t n);
int      mem_cmp(const void *a, const void *b, uint32_t n);
void    *mem_fill(void *dst, uint32_t count, uint8_t val);
uint32_t str_len(const char *s);

#endif /* MEM_H */
