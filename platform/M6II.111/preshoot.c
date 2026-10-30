/* M6II: 1:1 crop video in movie mode.
 *
 * PoCom MechalessStart switches movie LiveView to Canon's 1:1 sensor mode
 * (drive mode 8, 5376x3560, 29.97 or 23.976 with the timing patch below);
 * EDMAC ch 1 then writes each frame into one buffer, and once per LiveView
 * frame we start Canon's CR3 encoder (AlgsMgr StartEncode) on the crop:
 * the RAW burst's C-RAW job (type 9, encoder 0x12) or its RAW stills'
 * lossless one (type 0, encoder 0), 2 tiles each.  MechalessStop ends it.
 *
 * AlgsMgr sets an encoder up from the job's size only while its flag at
 * state +0x138 + id * 4 is 0 (0xE02ACF30), so the flag is cleared whenever
 * the job's size, depth or pitch changes, and at the end of a take.
 *
 * The take's settings come from a sensor-mode module (d8_modes) and are
 * read by the recorder (mlv_lite): src/preshoot.h.  (The file's name is
 * from an earlier source, Canon's RAW burst pre-shooting.) */

#include "dryos.h"
#include "patch.h"
#include "property.h"
#include "gui-common.h"
#include "crx_enc.h"
#include "preshoot.h"

#ifdef CONFIG_PRESHOOT_1TO1

/* the take's settings, from the sensor-mode module (preshoot.h) */
static struct preshoot_take ps_take_cfg;

void preshoot_take_config(const struct preshoot_take *take)
{
    ps_take_cfg = *take;
}

int preshoot_take_get(struct preshoot_take *take)
{
    if (take)
    {
        *take = ps_take_cfg;
    }
    return ps_take_cfg.on;
}

static int ps_crop_w = 0, ps_crop_h = 0;

void preshoot_take_set_crop(int width, int height)
{
    ps_crop_w = width;
    ps_crop_h = height;
}

int preshoot_take_get_crop(int *width, int *height)
{
    *width = ps_crop_w;
    *height = ps_crop_h;
    return ps_crop_w > 0 && ps_crop_h > 0;
}

/* Canon's RAW burst and pre-shooting settings, followed so that ML knows
 * their values and lengths (prop_request_change refuses properties it has
 * no handler for) */
static volatile int ps_setting_burst = -1, ps_setting_prerec = -1;

PROP_HANDLER(PROP_MECHALESS)
{
    ps_setting_burst = buf[0];
}

PROP_HANDLER(PROP_MECHALESS_PRE_REC)
{
    ps_setting_prerec = buf[0];
}

/* -1 until Canon reports it */
int preshoot_get_setting(int prerec)
{
    return prerec ? ps_setting_prerec : ps_setting_burst;
}

/* Canon's message bus (seen in Canon's debug log when its GUI starts RAW
 * burst): PoCom send 0xE054AE80(cmd, data, done_cbr, ctx); 0x30033 =
 * "PO: MechalessStart" (the sensor's 1:1 mode), 0x30034 = "PO:
 * MechalessStop". */
#define POCOM_SEND ((void (*)(uint32_t cmd, void *data, void (*cbr)(), void *ctx)) 0xe054ae81)
#define PO_MECHALESS_START  0x30033
#define PO_MECHALESS_STOP   0x30034
static struct semaphore *ps_po_sem = NULL;

static void ps_po_done(void)
{
    give_semaphore(ps_po_sem);
}

static int ps_pocom(uint32_t cmd)
{
    if (!ps_po_sem)
    {
        ps_po_sem = create_named_semaphore("preshoot_po", 0);
    }
    POCOM_SEND(cmd, NULL, (void (*)()) ps_po_done, NULL);
    return take_semaphore(ps_po_sem, 3000) == 0;
}

#define JOB_WORDS       15              /* 0x3C bytes */
#define JOB_TYPE_FULL   9
#define ENC_ID_FULL     0x12
#define FULL_W          5376
#define FULL_H          3560

static int ps_x0 = 0, ps_width = 0, ps_y0 = 0, ps_height = 0;
static uint32_t ps_last_size = 0;       /* job width << 16 | height last set up */

/* Movie mode: MechalessStart alone switches movie LiveView to the 1:1
 * mode (LiveView slot 11, drive mode 8, sensor slot 4), and EDMAC ch 1
 * then writes each 5376 x 3560 frame (14 bit, LSB-first) into one buffer
 * (Canon's Mechaless memory, 45 MB from SRM).  Canon encodes nothing
 * there: we start the encode of the crop ourselves (preshoot.h: C-RAW or
 * lossless), once per LiveView frame, one at a time.  The parts stay in
 * AlgsMgr's ring buffers until freed, which must happen in order (a
 * handle held for good stalled the encoder). */
extern uint32_t algs_start_encode(void *jobs, uint32_t n, void (*cbr)(), void *ctx);
extern void algs_free_enc_buffer(uint32_t handle, uint32_t a, uint32_t b);
extern uint32_t edmac_get_address(uint32_t channel);
extern uint32_t edmac_get_length(uint32_t channel);
extern uint32_t edmac_get_pointer(uint32_t channel);

static uint32_t ps_movie_job[JOB_WORDS];
static volatile int ps_movie_on = 0;
static volatile int ps_movie_busy = 0;
static int (*ps_movie_cbr)(uint32_t handle) = NULL;
static volatile int ps_movie_started = 0, ps_movie_skipped = 0, ps_movie_failed = 0;
static volatile int ps_movie_wrong_depth = 0, ps_movie_resets = 0;
static int ps_movie_bits = 14;          /* the take's depth */
/* lossless: Canon's full RAW job (type 0 = encoder 0, as its lossless
 * stills: 2 tiles, encType 0, 0 DWT levels) instead of the RAW burst's
 * C-RAW job (type 9, encoder 0x12) */
/* flags packed in one word: wrote = we changed ch 1's depth; lossless as
 * above */
static struct { uint8_t wrote, lossless; } ps_movie_flags;
#define ps_movie_wrote    ps_movie_flags.wrote
#define ps_movie_lossless ps_movie_flags.lossless
#define JOB_TYPE_RAW    0
#define ENC_ID_RAW      0
static uint32_t ps_movie_cfg = 0;       /* size, depth, pitch the encoder was set up for */

/* where ch 1's writer is (row of the full frame, -1 = not in this frame's
 * buffer) at vsync, after the encode started and when it ended: the
 * encoder reads the frame while the next one is being written into the
 * same buffer (point 1 repeats vsync's; preshoot_movie_race).  Allocated
 * per take. */
#define RACE_POINTS 4
struct ps_race
{
    int n;
    int last[RACE_POINTS], min[RACE_POINTS], max[RACE_POINTS];
    int64_t sum[RACE_POINTS];
    int enc_us_max, enc_us_sum;
    uint64_t enc_t0;
    volatile int vals[RACE_POINTS];     /* this frame's */
};
static struct ps_race *ps_race;

static int ps_writer_row(uint32_t base, int xb)
{
    uint32_t p = edmac_get_pointer(1);
    if (p < base || p >= base + (uint32_t) xb * FULL_H)
    {
        return -1;
    }
    return (p - base) / xb;
}

static void ps_race_add(void)
{
    struct ps_race *r = ps_race;
    if (!r)
    {
        return;
    }
    for (int i = 0; i < RACE_POINTS; i++)
    {
        int v = r->vals[i];
        r->last[i] = v;
        if (!r->n || v < r->min[i]) r->min[i] = v;
        if (!r->n || v > r->max[i]) r->max[i] = v;
        r->sum[i] += v;
    }
    r->n++;
}

/* EDMAC ch 1 (1_WR_L0): pack register (f2, bits 4-5: depth 10 + 2 * f2;
 * Canon's 14 in this mode) and row size, an "L" channel's at +0x5C (as
 * raw.c's raw_lv_bpp_vsync switches them) */
#define CH1_PACK_REG    0xD0422124
#define CH1_XB_REG      0xD042005C
#define CANON_XB        (FULL_W * 14 / 8)

/* the 1:1 mode alone: MechalessStart / Stop, no settings, no GUI */
int preshoot_mode_start(void)
{
    if (!ps_pocom(PO_MECHALESS_START))
    {
        printf("preshoot: no MechalessStart completion\n");
        return -1;
    }
    return 0;
}

void preshoot_mode_stop(void)
{
    if (!ps_pocom(PO_MECHALESS_STOP))
    {
        printf("preshoot: no MechalessStop completion\n");
    }
}

/* AlgsMgr's task: hand the result to mlv_lite; if it doesn't take it (or
 * we've stopped), give it back at once */
static void ps_movie_done(void *ctx, uint32_t *results, uint32_t a2, uint32_t a3)
{
    uint32_t handle = results ? results[0] : 0;
    int (*cbr)(uint32_t) = ps_movie_cbr;
    if (ps_race && ps_race->enc_t0)
    {
        ps_race->vals[3] = ps_writer_row(edmac_get_address(1), MEM(CH1_XB_REG));
        int us = get_us_clock() - ps_race->enc_t0;
        ps_race->enc_t0 = 0;
        ps_race->enc_us_sum += us;
        if (us > ps_race->enc_us_max) ps_race->enc_us_max = us;
        ps_race_add();
    }
    ps_movie_busy = 0;
    if (!handle)
    {
        ps_movie_failed++;
        return;
    }
    if (!ps_movie_on || !cbr || !cbr(handle))
    {
        algs_free_enc_buffer(handle, 0, 0);
    }
}

int preshoot_movie_enable(int x0, int width, int y0, int height, int bits, int (*frame_cbr)(uint32_t handle))
{
    if (x0 < 0 || (x0 & 15) || width < 64 || (width & 7) || x0 + width > FULL_W ||
        y0 < 0 || (y0 & 1) || height < 64 || (height & 7) || y0 + height > FULL_H ||
        (bits != 10 && bits != 12 && bits != 14))
    {
        return -1;
    }
    ps_movie_bits = bits;
    ps_movie_wrote = 0;
    ps_movie_wrong_depth = ps_movie_resets = 0;
    ps_x0 = x0;
    ps_width = width;
    ps_y0 = y0;
    ps_height = height;
    ps_movie_cbr = frame_cbr;
    ps_movie_started = ps_movie_skipped = ps_movie_failed = 0;
    ps_movie_busy = 0;
    if (!ps_race)
    {
        ps_race = malloc(sizeof(*ps_race));
    }
    if (ps_race)
    {
        memset(ps_race, 0, sizeof(*ps_race));
    }
    ps_movie_on = 1;
    return 0;
}

/* before preshoot_movie_enable: lossless (1) or C-RAW (0) encodes */
void preshoot_movie_set_lossless(int on)
{
    ps_movie_lossless = on ? 1 : 0;
}

/* stop starting encodes; the one in flight (if any) is freed when it ends */
void preshoot_movie_disable(void)
{
    ps_movie_on = 0;
    for (int t = 0; t < 500 && ps_movie_busy; t += 10)
    {
        msleep(10);
    }
    /* the encoder we set up keeps our size and depth until its flag is
     * cleared: encoder 0 is Canon's RAW stills' one, 0x12 RAW burst's */
    uint32_t st = MEM(ALGS_STATE_PTR);
    if (st)
    {
        MEM(st + ALGS_ENC_INITED + ENC_ID_RAW * 4) = 0;
        MEM(st + ALGS_ENC_INITED + ENC_ID_FULL * 4) = 0;
    }
    ps_last_size = 0;
    ps_movie_cfg = 0;
    if (ps_movie_wrote)
    {
        /* Canon's depth back for whatever reads ch 1 until the mode ends */
        MEM(CH1_PACK_REG) = (MEM(CH1_PACK_REG) & ~0x30) | 0x20;
        MEM(CH1_XB_REG) = CANON_XB;
        ps_movie_wrote = 0;
    }
    printf("preshoot: %d encodes started, %d frames skipped (encoder busy), %d failed, "
           "%d at another depth, %d depth resets\n",
           ps_movie_started, ps_movie_skipped, ps_movie_failed, ps_movie_wrong_depth, ps_movie_resets);
}

/* At vsync (Canon's read-out done): the depth and row size ch 1 wrote the
 * frame that just ended with, as its registers show them now; then our
 * depth for the next frame.  If Canon set its own back meanwhile, that
 * frame was written at Canon's depth: we read the registers before
 * rewriting them, so each frame is encoded as it was written. */
static int ps_movie_ch1_depth(int *xb)
{
    uint32_t pack = MEM(CH1_PACK_REG);
    int bits = 10 + 2 * ((pack >> 4) & 3);
    *xb = MEM(CH1_XB_REG);
    if (ps_movie_bits != 14)
    {
        int want_xb = FULL_W * ps_movie_bits / 8;
        if (bits != ps_movie_bits || *xb != want_xb)
        {
            if (ps_movie_wrote)
            {
                ps_movie_resets++;
            }
            MEM(CH1_PACK_REG) = (pack & ~0x30) | (((ps_movie_bits - 10) / 2) << 4);
            MEM(CH1_XB_REG) = want_xb;
            ps_movie_wrote = 1;
        }
    }
    return bits;
}

/* once per LiveView frame (mlv_lite's vsync): encode the frame in ch 1's
 * buffer.  Returns 1 if started; 0 before the 1:1 mode runs (ch 1 isn't
 * 3560 rows yet) or while the last frame is still being encoded */
int preshoot_movie_frame(void)
{
    if (!ps_movie_on || ((edmac_get_length(1) >> 16) + 1) != FULL_H)
    {
        return 0;
    }
    int xb = 0;
    int bits = ps_movie_ch1_depth(&xb);
    if (bits != ps_movie_bits)
    {
        /* written at another depth than the take's (the first frame, or
         * Canon set its own back): not recorded */
        ps_movie_wrong_depth++;
        return 0;
    }
    if (xb <= 0 || (xb * 8) % bits || xb * 8 / bits < FULL_W)
    {
        ps_movie_wrong_depth++;
        return 0;
    }
    if (ps_movie_busy)
    {
        ps_movie_skipped++;
        return 0;
    }
    uint32_t base = edmac_get_address(1);
    if (ps_race)
    {
        ps_race->vals[0] = ps_writer_row(base, xb);
    }
    if (ps_race)
    {
        ps_race->vals[1] = ps_race->vals[0];    /* point 1: unused, kept for the stats layout */
    }
    uint32_t *j = ps_movie_job;
    memset(j, 0, sizeof(ps_movie_job));
    j[0] = ps_movie_lossless ? JOB_TYPE_RAW : JOB_TYPE_FULL;
    j[1] = ps_width;
    j[2] = ps_height;
    j[5] = bits;
    j[6] = edmac_get_address(1) + ps_y0 * xb + ps_x0 * bits / 8;
    j[7] = xb * 8 / bits;   /* pitch in pixels: rows of the full frame */
    j[8] = ps_movie_lossless ? 1 : 0;   /* as Canon's lossless stills / RAW burst jobs */
    uint32_t size = j[1] << 16 | j[2];
    uint32_t cfg = size ^ (j[5] << 27) ^ (j[7] << 13) ^ (j[0] << 4);
    if (size != ps_last_size || cfg != ps_movie_cfg)
    {
        /* AlgsMgr sets an encoder up (size, depth, pitch) only while its
         * flag is clear */
        uint32_t st = MEM(ALGS_STATE_PTR);
        if (st)
        {
            MEM(st + ALGS_ENC_INITED + (ps_movie_lossless ? ENC_ID_RAW : ENC_ID_FULL) * 4) = 0;
        }
        ps_last_size = size;
        ps_movie_cfg = cfg;
    }
    ps_movie_busy = 1;
    if (ps_race)
    {
        ps_race->enc_t0 = get_us_clock();
    }
    if (algs_start_encode(j, 1, (void (*)()) ps_movie_done, NULL) & 1)
    {
        ps_movie_busy = 0;
        ps_movie_failed++;
        if (ps_race)
        {
            ps_race->enc_t0 = 0;
        }
        return 0;
    }
    if (ps_race)
    {
        ps_race->vals[2] = ps_writer_row(base, xb);
    }
    ps_movie_started++;
    return 1;
}

/* counters of the last (or current) take */
void preshoot_movie_stats(int *started, int *skipped, int *failed, int *wrong_depth, int *resets)
{
    *started = ps_movie_started;
    *skipped = ps_movie_skipped;
    *failed = ps_movie_failed;
    *wrong_depth = ps_movie_wrong_depth;
    *resets = ps_movie_resets;
}

/* the writer's row at vsync (twice), after the encode start and at its
 * end: min / mean / max of each (12 values), then the encode time max /
 * mean (us); returns the frames measured */
int preshoot_movie_race(int *v)
{
    struct ps_race *r = ps_race;
    if (!r || !r->n)
    {
        return 0;
    }
    for (int i = 0; i < RACE_POINTS; i++)
    {
        v[i * 3] = r->min[i];
        v[i * 3 + 1] = r->sum[i] / r->n;
        v[i * 3 + 2] = r->max[i];
    }
    v[12] = r->enc_us_max;
    v[13] = r->enc_us_sum / r->n;
    return r->n;
}

/* Canon's 1:1 mode at 23.976: Canon runs it in TG mode 8, which has only a 29.97 column (fps index 2).  Between
 * frame rates Canon's tables change only the frame length (VD, +0x000),
 * VD minus a per-mode constant (+0x690, 141 for mode 8) and the u32 table
 * (frame period in us - 500; 0xA0F9 at 23.976 in every mode).  Mode 8's
 * line is 1192 clocks (15.52 us): 41.708 ms = 2687 lines.  Patched in RAM
 * (Canon takes the table when the mode starts: not while it runs),
 * restored when off. */
#define SDRV_STATE          0x9CD8      /* +0x78 TG table */
#define DTS_MASTER_DRIVE    0x4962C     /* DTS master +0xC: sensor drive mode */
#define TG_STRIDE           0x1C
#define TG_VD               0x000
#define TG_VD_MINUS         0x690
#define TG_U32_TABLE        0xF50       /* + mode * 0x38 + fps * 4 */
#define TG_MODE_BURST       8
#define TG_FPS_2997         2
#define VD_BURST_2997       2149
#define VD_BURST_23976      2687
#define TG_PERIOD_23976     0xA0F9
static uint16_t ps_24p_orig[2];
static uint32_t ps_24p_orig32;
static uint32_t ps_24p_tg = 0;          /* table patched, 0 = not */

/* returns the frame rate the next 1:1 mode start will run at (x1000) */
int preshoot_set_23976(int on)
{
    uint32_t tg = MEM(SDRV_STATE + 0x78);
    if ((MEM(DTS_MASTER_DRIVE) & 0xFF) == TG_MODE_BURST)
    {
        /* the 1:1 mode runs: leave the table alone */
        return ps_24p_tg ? 23976 : 29970;
    }
    if (on && !ps_24p_tg && tg)
    {
        volatile uint16_t *vd = (volatile uint16_t *)(tg + TG_VD + TG_MODE_BURST * TG_STRIDE + TG_FPS_2997 * 2);
        volatile uint16_t *vm = (volatile uint16_t *)(tg + TG_VD_MINUS + TG_MODE_BURST * TG_STRIDE + TG_FPS_2997 * 2);
        volatile uint32_t *per = (volatile uint32_t *)(tg + TG_U32_TABLE + TG_MODE_BURST * 0x38 + TG_FPS_2997 * 4);
        /* only over Canon's own 29.97 values */
        if (*vd != VD_BURST_2997 || *vm != VD_BURST_2997 - 141)
        {
            printf("preshoot: 1:1 TG column unexpected (%d %d), staying at 29.97\n", *vd, *vm);
            return 29970;
        }
        ps_24p_orig[0] = *vd;
        ps_24p_orig[1] = *vm;
        ps_24p_orig32 = *per;
        *vd = VD_BURST_23976;
        *vm = VD_BURST_23976 - 141;
        *per = TG_PERIOD_23976;
        ps_24p_tg = tg;
    }
    else if (!on && ps_24p_tg)
    {
        uint32_t t = ps_24p_tg;
        *(volatile uint16_t *)(t + TG_VD + TG_MODE_BURST * TG_STRIDE + TG_FPS_2997 * 2) = ps_24p_orig[0];
        *(volatile uint16_t *)(t + TG_VD_MINUS + TG_MODE_BURST * TG_STRIDE + TG_FPS_2997 * 2) = ps_24p_orig[1];
        *(volatile uint32_t *)(t + TG_U32_TABLE + TG_MODE_BURST * 0x38 + TG_FPS_2997 * 4) = ps_24p_orig32;
        ps_24p_tg = 0;
    }
    return ps_24p_tg ? 23976 : 29970;
}

/* mlv_lite copied a frame's parts: give its buffers back (in order) */
void preshoot_movie_release(uint32_t handle)
{
    algs_free_enc_buffer(handle, 0, 0);
}

#endif
