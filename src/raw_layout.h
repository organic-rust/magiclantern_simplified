/** \file
 * Raw pixel layouts, shared by the camera code and host tools (mlv_dump).
 *
 * RAW_LAYOUT_ML: Magic Lantern's layout, used in MLV files and by
 * struct raw_pixblock (raw.h): a bit stream read MSB first from 16-bit
 * little-endian words.  Works at any bit depth.
 *
 * RAW_LAYOUT_LSB: DIGIC 8 LiveView raw (M6II): a bit stream read LSB first
 * from bytes.  At 12 bits, 3 bytes hold 2 pixels:
 *     p0 = b0 | (b1 & 0x0F) << 8,  p1 = b1 >> 4 | b2 << 4
 * 12-bit is verified on M6II; 10 and 14 bit are assumed to follow the same
 * rule (Canon's EDMAC pack unit uses one depth code for all three).
 *
 * The accessors read up to 3 bytes past the pixel's last byte, and write
 * only the bytes the pixel occupies.
 */

#ifndef _raw_layout_h_
#define _raw_layout_h_

#include <stdint.h>

#define RAW_LAYOUT_ML   0
#define RAW_LAYOUT_LSB  1

static inline int raw_layout_get(const void *row, int layout, int bpp, int x)
{
    const uint8_t *p = row;
    uint32_t bit = (uint32_t)x * bpp;
    uint32_t mask = (1u << bpp) - 1;

    if (layout == RAW_LAYOUT_LSB)
    {
        const uint8_t *q = p + (bit >> 3);
        uint32_t v = q[0] | q[1] << 8 | (uint32_t)q[2] << 16;
        return (v >> (bit & 7)) & mask;
    }

    const uint8_t *w = p + (bit >> 4) * 2;
    uint32_t v = (uint32_t)(w[0] | w[1] << 8) << 16 | (w[2] | w[3] << 8);
    return (v >> (32 - (bit & 15) - bpp)) & mask;
}

static inline void raw_layout_set(void *row, int layout, int bpp, int x, int value)
{
    uint8_t *p = row;
    uint32_t bit = (uint32_t)x * bpp;
    uint32_t mask = (1u << bpp) - 1;
    uint32_t val = (uint32_t)value & mask;

    if (layout == RAW_LAYOUT_LSB)
    {
        uint8_t *q = p + (bit >> 3);
        int shift = bit & 7;
        int nbytes = (shift + bpp + 7) / 8;
        uint32_t v = q[0] | q[1] << 8 | (uint32_t)q[2] << 16;
        v = (v & ~(mask << shift)) | val << shift;
        for (int i = 0; i < nbytes; i++)
        {
            q[i] = v >> (8 * i);
        }
        return;
    }

    uint8_t *w = p + (bit >> 4) * 2;
    int shift = 32 - (bit & 15) - bpp;
    uint32_t v = (uint32_t)(w[0] | w[1] << 8) << 16 | (w[2] | w[3] << 8);
    v = (v & ~(mask << shift)) | val << shift;
    w[0] = v >> 16;
    w[1] = v >> 24;
    if ((bit & 15) + bpp > 16)
    {
        w[2] = v;
        w[3] = v >> 8;
    }
}

/* rows must not overlap */
static inline void raw_layout_convert_row(const void *src, int src_layout,
                                          void *dst, int dst_layout,
                                          int bpp, int width)
{
    for (int x = 0; x < width; x++)
    {
        raw_layout_set(dst, dst_layout, bpp, x, raw_layout_get(src, src_layout, bpp, x));
    }
}

#endif
