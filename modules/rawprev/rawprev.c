/* Raw preview (test): LiveView picture drawn by ML from the raw frame
 * (M6II 1.1.1, DIGIC 8).
 *
 * Canon's LiveView shows only the top 2000 rows of a 2x2 3:2 frame, and
 * patching its development path for more froze the camera.  This draws
 * the raw buffer (Mem1, raw_info) instead, into a display layer of our
 * own:
 * - XCM (Ximr Context Maker) on M6II has 4 source layers (layerIndex < 4
 *   in XCM_SetSourceSurface 0xE009ACBA); Canon's GUI is 0, ML's overlay
 *   takes the first free one at boot (compositor.c).  Our layer goes at
 *   ML's index and ML's overlay moves to the next free one, assuming the
 *   higher index is drawn on top.
 * - layer format RGBA 0x05040100 (MARV flags, compositor.h), as ML's
 *   overlay, 0xAARRGGBB.  (A YUV 4:2:2 layer, 0x04020100, also displays
 *   but was slower and its colours worse; dropped with the resizer plan,
 *   notes/sensor-readout.md.)
 * - drawing: each output pixel takes one 2x2 Bayer block (RGGB at even
 *   coordinates), fixed white balance, square-root gamma.  360x240 is
 *   drawn sampled and doubled to 720x480 by the CPU; 720x480 samples
 *   every pixel.
 * Nothing here is saved; the layer is hidden when the preview is off. */

#include <dryos.h>
#include <module.h>
#include <menu.h>
#include <config.h>
#include <raw.h>
#include <raw_layout.h>
#include <propvalues.h>
#include <zebra.h>
#include <bmp.h>

/* Canon / ML core */
/* vram.h's struct MARV is for the core only (DIGIC 8 layout, no DIGIC X
 * fields) */
struct MARV
{
    uint32_t signature;         /* MARV */
    uint8_t *bitmap_data;
    uint8_t *opacity_data;
    uint32_t flags;
    uint32_t width;
    uint32_t height;
    void *pmem;
};
#define M6II_XCM (*(void **)0x78ce8)    /* _pXCM (stubs.S), XCM_PTR in consts.h */
#define XCM_LAYERS 4
/* per-layer records in XCM: surface, then source area x, y, w, h (u16),
 * layer offset (XCM_SetSourceArea 0xE009AFA4, SetLayerOffset 0xE009AFF2) */
#define XCM_LAYER_REC  0x974
#define XCM_LAYER_SIZE 0x24
extern struct MARV *rgb_vram_info;      /* ML's overlay layer (bmp.c) */
extern struct MARV *XCM_GetSourceSurface(void *pXCM, uint32_t layer);
extern uint32_t XCM_SetSourceSurface(void *pXCM, uint32_t layer, struct MARV *pMARV);
extern uint32_t XCM_SetSourceArea(void *pXCM, uint32_t layer, uint16_t x, uint16_t y, uint16_t w, uint16_t h);
extern uint32_t XOC_SetLayerEnable(int p1, int p2, uint32_t layer, int enable);
extern int XimrExe(void *);
extern struct semaphore *winsys_sem;

#define PREV_W 720
#define PREV_H 480
#define FLAGS_RGBA 0x05040100

static int prev_on = 0;
static int prev_size = 0;   /* 0: 360x240 doubled, 1: 720x480 */
static int prev_trigger = 0;    /* 0: always, 1: half-shutter toggles */
static int prev_toggled = 0;    /* half-shutter toggle: preview shown */

static struct MARV *prev_marv;
static int prev_layer = -1;         /* our XCM index, -1 until the swap */
static int prev_shown = 0;          /* layer enabled with a picture */
static char prev_status[64] = "off";
static char prev_last[64] = "";       /* last speed measured */
static int prev_ms = 0;             /* last frame's drawing time */
static int prev_fps10 = 0;

static uint8_t gamma_lut[4096];     /* 12-bit linear -> 8-bit */

static void ximr_redraw(void)
{
    take_semaphore(winsys_sem, 0);
    XimrExe((void *)((uintptr_t)M6II_XCM + 0x10));  /* XIMR_CONTEXT */
    give_semaphore(winsys_sem);
}

static struct MARV *marv_create(void)
{
    struct MARV *m = malloc(sizeof(struct MARV));
    uint8_t *buf = malloc(PREV_W * PREV_H * 4 + 0x100);
    if (!m || !buf)
    {
        if (m) free(m);
        if (buf) free(buf);
        return NULL;
    }
    /* 256-byte aligned and uncacheable, as compositor.c does for Zico */
    buf = UNCACHEABLE((uint8_t *)((((uintptr_t)buf + 0x100) >> 8) << 8));
    m->signature = 0x5652414D;  /* MARV */
    m->bitmap_data = buf;
    m->opacity_data = 0;
    m->flags = FLAGS_RGBA;
    m->width = PREV_W;
    m->height = PREV_H;
    m->pmem = 0;
    bzero32(buf, PREV_W * PREV_H * 4);
    return m;
}

/* put our layer at ML's index, ML's overlay one above; once */
static int layer_setup(void)
{
    if (prev_layer >= 0)
        return 0;
    void *xcm = M6II_XCM;
    int ml = -1, free_id = -1;
    for (int i = 0; i < XCM_LAYERS; i++)
    {
        struct MARV *s = XCM_GetSourceSurface(xcm, i);
        if (s == rgb_vram_info && ml < 0)
            ml = i;
        else if (!s && ml >= 0 && free_id < 0)
            free_id = i;
    }
    if (ml <= 0 || free_id < 0)
    {
        snprintf(prev_status, sizeof(prev_status), "no layer (ML %d, free %d)", ml, free_id);
        return -1;
    }
    if (!prev_marv && !(prev_marv = marv_create()))
    {
        snprintf(prev_status, sizeof(prev_status), "no memory");
        return -1;
    }
    /* ML's overlay is 960x540, shown through a 720x480 window at
     * (120, 30) (bmp.h BMP_W_MINUS / BMP_H_MINUS): keep its window */
    uint16_t *area = (uint16_t *)((uintptr_t)xcm + XCM_LAYER_REC + ml * XCM_LAYER_SIZE + 4);
    XCM_SetSourceSurface(xcm, free_id, rgb_vram_info);
    XCM_SetSourceArea(xcm, free_id, area[0], area[1], area[2], area[3]);
    XOC_SetLayerEnable(0, 0, free_id, 1);
    XCM_SetSourceSurface(xcm, ml, prev_marv);
    XCM_SetSourceArea(xcm, ml, 0, 0, PREV_W, PREV_H);
    XOC_SetLayerEnable(0, 0, ml, 0);
    prev_layer = ml;
    prev_shown = 0;
    DryosDebugMsg(0, 15, "rawprev: layer %d, ML overlay moved to %d", ml, free_id);
    ximr_redraw();
    return 0;
}

static void layer_show(int show)
{
    if (prev_layer < 0)
        return;
    if (show == prev_shown)
        return;
    XOC_SetLayerEnable(0, 0, prev_layer, show);
    prev_shown = show;
    ximr_redraw();
}

static void gamma_init(void)
{
    /* out = 255 * sqrt(x / 4095) */
    int r = 0;
    for (int x = 0; x < 4096; x++)
    {
        int t = x * 255 * 255 / 4095;
        while ((r + 1) * (r + 1) <= t)
            r++;
        gamma_lut[x] = r;
    }
}

/* raw -> 12-bit scale factor (x 2^16) */
static int lv_scale;

static inline int lin12(int v, int black)
{
    v -= black;
    if (v <= 0)
        return 0;
    v = (v * lv_scale) >> 16;
    return v > 4095 ? 4095 : v;
}

static int col_x[PREV_W];
#define PF 6                        /* prefetch distance, samples */
static uint32_t row_rgb[PREV_W];    /* one output row, 0x00RRGGBB */

extern void dcache_clean(uint32_t addr, uint32_t size);

/* two neighbours (x even, x + 1) from ML's 14-bit layout: 8 pixels in
 * 14 bytes (struct raw_pixblock) */
static inline void ml14_pair(const uint8_t *row, int x, int *v0, int *v1)
{
    const struct raw_pixblock *p = (const void *)(row + (x >> 3) * 14);
    switch (x & 6)
    {
        case 0: *v0 = p->a;                       *v1 = p->b_lo | (p->b_hi << 12); break;
        case 2: *v0 = p->c_lo | (p->c_hi << 10);  *v1 = p->d_lo | (p->d_hi << 8);  break;
        case 4: *v0 = p->e_lo | (p->e_hi << 6);   *v1 = p->f_lo | (p->f_hi << 4);  break;
        default: *v0 = p->g_lo | (p->g_hi << 2);  *v1 = p->h;                      break;
    }
}

static int t_sample, t_write, t_show;   /* us, last frame */

static void draw_frame(void)
{
    int full = prev_size;
    int dw = full ? PREV_W : PREV_W / 2;
    int dh = full ? PREV_H : PREV_H / 2;
    int step = full ? 1 : 2;

    int x1 = raw_info.active_area.x1, y1 = raw_info.active_area.y1;
    int aw = raw_info.active_area.x2 - x1, ah = raw_info.active_area.y2 - y1;
    if (aw <= 0 || ah <= 0)
        return;
    /* fit the active area, keep its aspect ratio */
    int ow = dw, oh = dw * ah / aw;
    if (oh > dh)
    {
        oh = dh;
        ow = dh * aw / ah;
    }
    int ox = (dw - ow) / 2, oy = (dh - oh) / 2;

    int bpp = raw_info.bits_per_pixel;
    int layout = raw_get_layout();
    int ml14 = raw_uses_pixblocks();
    int black = raw_info.black_level;
    int range = raw_info.white_level - black;
    if (range <= 0)
        return;
    lv_scale = (4095 << 16) / range;
    /* daylight-ish white balance (x 256) */
    const int gain_r = 512, gain_b = 410;

    for (int x = 0; x < ow; x++)
        col_x[x] = (x1 + x * aw / ow) & ~1;

    const uint8_t *raw = CACHEABLE(raw_info.buffer);
    /* drawn through the cacheable view, cleaned to memory at the end
     * (Zico reads the uncacheable buffer) */
    uint8_t *dst = CACHEABLE(prev_marv->bitmap_data);
    int pitch = PREV_W * 4;
    t_sample = t_write = 0;

    for (int y = 0; y < dh; y++)
    {
        int t0 = get_us_clock();
        int yy = y - oy;
        int inside = yy >= 0 && yy < oh;
        for (int x = 0; x < dw; x++)
            row_rgb[x] = 0;
        if (inside)
        {
            int ry = (y1 + yy * ah / oh) & ~1;
            const uint8_t *row0 = raw + ry * raw_info.pitch;
            const uint8_t *row1 = row0 + raw_info.pitch;
            /* RAM is cached in L1 only (TEX 4, C 0, B 1: outer
             * non-cacheable, notes/memory-map.md), so each line comes
             * from DRAM: ask for lines PF samples ahead to keep several
             * fills in flight */
            for (int k = 0; k < PF && k < ow; k++)
            {
                __builtin_prefetch(row0 + col_x[k] * bpp / 8);
                __builtin_prefetch(row1 + col_x[k] * bpp / 8);
            }
            for (int xx = 0; xx < ow; xx++)
            {
                int rx = col_x[xx];
                if (xx + PF < ow)
                {
                    int ahead = col_x[xx + PF] * bpp / 8;
                    __builtin_prefetch(row0 + ahead);
                    __builtin_prefetch(row1 + ahead);
                }
                int r, g1, g2, b;
                if (ml14)
                {
                    ml14_pair(row0, rx, &r, &g1);
                    ml14_pair(row1, rx, &g2, &b);
                }
                else
                {
                    r  = raw_layout_get(row0, layout, bpp, rx);
                    g1 = raw_layout_get(row0, layout, bpp, rx + 1);
                    g2 = raw_layout_get(row1, layout, bpp, rx);
                    b  = raw_layout_get(row1, layout, bpp, rx + 1);
                }
                r = lin12(r, black);
                int g = (lin12(g1, black) + lin12(g2, black)) >> 1;
                b = lin12(b, black);
                r = MIN((r * gain_r) >> 8, 4095);
                b = MIN((b * gain_b) >> 8, 4095);
                row_rgb[xx + ox] = gamma_lut[r] << 16 | gamma_lut[g] << 8 | gamma_lut[b];
            }
        }
        int t1 = get_us_clock();
        uint8_t *line = dst + y * step * pitch;
        uint32_t *p = (uint32_t *)line;
        if (step == 2)
            for (int x = 0; x < dw; x++)
                p[2 * x] = p[2 * x + 1] = 0xFF000000 | row_rgb[x];
        else
            for (int x = 0; x < dw; x++)
                p[x] = 0xFF000000 | row_rgb[x];
        if (step == 2)
            memcpy(line + pitch, line, pitch);
        int t2 = get_us_clock();
        t_sample += t1 - t0;
        t_write += t2 - t1;
    }
    int t3 = get_us_clock();
    dcache_clean((uint32_t)dst, PREV_W * PREV_H * 4);
    t_write += get_us_clock() - t3;
}

static int prev_can_draw(void)
{
    return prev_on && lv && (!prev_trigger || prev_toggled) && liveview_display_idle();
}

/* raw LiveView kept on while the preview is on */
static int prev_raw = 0;

static void raw_hold(int on)
{
    if (on && !prev_raw)
        raw_lv_request();
    else if (!on && prev_raw)
        raw_lv_release();
    prev_raw = on;
}

static void prev_task(void *unused)
{
    int frames = 0, t_fps = get_ms_clock();
    TASK_LOOP
    {
        if (!prev_on)
        {
            layer_show(0);
            raw_hold(0);
            snprintf(prev_status, sizeof(prev_status), "off");
            msleep(200);
            continue;
        }
        if (layer_setup() < 0)
        {
            msleep(500);
            continue;
        }
        if (!prev_can_draw())
        {
            layer_show(0);
            raw_hold(lv);
            snprintf(prev_status, sizeof(prev_status), "%s", prev_last[0] ? prev_last : "waiting (LiveView idle)");
            msleep(100);
            continue;
        }
        raw_hold(1);
        if (!raw_update_params())
        {
            layer_show(0);
            snprintf(prev_status, sizeof(prev_status), "no raw");
            msleep(200);
            continue;
        }
        int t0 = get_us_clock();
        draw_frame();
        int t1 = get_us_clock();
        prev_ms = (t1 - t0) / 1000;
        layer_show(1);
        ximr_redraw();
        t_show = get_us_clock() - t1;
        frames++;
        int now = get_ms_clock();
        if (now - t_fps >= 1000)
        {
            prev_fps10 = frames * 10000 / (now - t_fps);
            frames = 0;
            t_fps = now;
            snprintf(prev_last, sizeof(prev_last), "%d.%dfps r%d w%d s%d ms",
                     prev_fps10 / 10, prev_fps10 % 10,
                     t_sample / 1000, t_write / 1000, t_show / 1000);
            snprintf(prev_status, sizeof(prev_status), "%s", prev_last);
        }
        msleep(10);
    }
}

static MENU_UPDATE_FUNC(prev_status_update)
{
    MENU_SET_VALUE("%s", prev_status);
}

static struct menu_entry prev_menu[] =
{
    {
        .name       = "Raw preview (test)",
        .priv       = &prev_on,
        .max        = 1,
        .help       = "LiveView drawn by ML from the raw frame (whole read-out).",
        .help2      = "Test: not saved. Hides Canon's LiveView and info screen.",
        .children   = (struct menu_entry[])
        {
            {
                .name       = "Show",
                .priv       = &prev_trigger,
                .max        = 1,
                .choices    = CHOICES("Always", "Half-shutter toggles"),
                .help       = "Half-shutter toggles: each half-press switches between",
                .help2      = "Canon's LiveView and the raw preview (focus works as usual).",
            },
            {
                .name       = "Size",
                .priv       = &prev_size,
                .max        = 1,
                .choices    = CHOICES("360x240 doubled", "720x480"),
                .help       = "Pixels drawn: 360x240 is repeated 2x2 on screen.",
            },
            {
                .name       = "Status",
                .update     = prev_status_update,
                .icon_type  = IT_ALWAYS_ON,
                .help       = "fps; ms per frame: r reading raw, w writing the layer,",
                .help2      = "s showing it (XimrExe).",
            },
            MENU_EOL,
        },
    },
};

static unsigned int prev_keypress(unsigned int key)
{
    if (key == MODULE_KEY_PRESS_HALFSHUTTER && prev_on && prev_trigger && lv && !gui_menu_shown())
    {
        prev_toggled = !prev_toggled;
    }
    /* Canon still gets the key (autofocus) */
    return 1;
}

static unsigned int prev_init(void)
{
    if (!is_camera("M6II", "1.1.1"))
    {
        return CBR_RET_ERROR;
    }
    gamma_init();
    menu_add("Movie", prev_menu, COUNT(prev_menu));
    task_create("rawprev", 0x1d, 0x1000, prev_task, (void *) 0);
    return 0;
}

static unsigned int prev_deinit(void)
{
    return 0;
}

MODULE_INFO_START()
    MODULE_INIT(prev_init)
    MODULE_DEINIT(prev_deinit)
MODULE_INFO_END()

MODULE_CBRS_START()
    MODULE_CBR(CBR_KEYPRESS, prev_keypress, 0)
MODULE_CBRS_END()
