/* Freestanding memory helpers.
 *
 * RECOVERED counterparts in the original image:
 *   memcpy (word-optimized) 0x080198E0 / byte-wise 0x080198C0
 *   memcmp                  0x0801F610 (returns 0 on equal)
 *   fill (dst, count, val)  0x080199A0
 * These are simple re-implementations of the observed semantics. */
#include <stdint.h>

void *mem_copy(void *dst, const void *src, uint32_t n)
{
    uint8_t *d = dst;
    const uint8_t *s = src;
    while (n--)
        *d++ = *s++;
    return dst;
}

int mem_cmp(const void *a, const void *b, uint32_t n)
{
    const uint8_t *x = a, *y = b;
    while (n--) {
        if (*x != *y)
            return (int)*x - (int)*y;
        x++; y++;
    }
    return 0;
}

void *mem_fill(void *dst, uint32_t count, uint8_t val)
{
    uint8_t *d = dst;
    while (count--)
        *d++ = val;
    return dst;
}

uint32_t str_len(const char *s)
{
    uint32_t n = 0;
    while (s[n])
        n++;
    return n;
}
