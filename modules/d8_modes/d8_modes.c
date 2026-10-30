/* DIGIC 8 movie sensor modes (so far the M6II 1.1.1): the Sensor mode
 * menu (3x3 / 2x2 / 1:1, frame rate, Readout band) and the LiveView table
 * edits that make the readouts Canon doesn't have.
 *
 * Canon programs the sensor from per-mode tables in RAM (found on M6II
 * 1.1.1):
 * - CMOS register lists (ImageSequence, sent over SIO0 when LiveView
 *   starts): table at MEM(SEQ_STATE + 8), one main list per mode slot
 *   and three side lists.  64-bit frames, register number in the top 12
 *   bits.  Slot 15 is 1080p, slot 16 reads the whole sensor height at
 *   the same 3x3 sampling.
 * - Image processor (EngIO) lists: table at MEM(SEQ_STATE + 0xC); the
 *   frame heights per slot (1348 rows in 15, 1592 in 16).
 * - Timing generator (LiveView::SensorDrive): table at
 *   MEM(SDRV_STATE + 0x78), u16 [tg mode][fps index]; the read-out
 *   duration "ROD" at +0x460 cuts 1080p 23.976 (tg mode 5, index 4)
 *   at ~1360 lines.
 * - Mem1Path copies its EDMAC size every frame from its state (+0x18 w,
 *   +0x1C h for the HEAD tap); Canon sets 1348 there at mode changes.
 *
 * Full 3:2 at 3x3 (1080p 23.976): slot 16's differing values go into
 * slot 15 (keeping slot 15's frame period), ROD becomes 800 and Mem1 is
 * kept at 1592 rows while the patched mode runs.  The other readouts
 * (readout engine below) move a slot's values by deltas.  Every original
 * value is checked first and kept, so Canon's tables can be put back. */

#include <dryos.h>
#include <module.h>
#include <menu.h>
#include <config.h>
#include <property.h>
#include <raw.h>
#include <raw_layout.h>
#include <propvalues.h>
#include <timer.h>
#include <notify_box.h>
#include <patch.h>
#include <lens.h>
#include <fio-ml.h>
#include <preshoot.h>
#include <cropmarks.h>
#include <bmp.h>

#define SEQ_STATE           0x1AB1C     /* +8 CMOS table, +0xC EngIO table */
#define SEQ_MODE            (0x10D88 + 8)
#define SDRV_STATE          0x9CD8      /* +0x78 TG table, +0x7C fps index */
#define MEM1_STATE          0x10970

#define SLOT_1080P          15
#define SLOT_FULL           16
#define CMOS_MAIN           0xBC4
#define CMOS_MAIN_STRIDE    0x36C
#define CMOS_MAIN_WORDS     (107 * 2)

#define TG_MODE_1080P       5
#define TG_FPS_23976        4
#define TG_STRIDE           0x1C
#define TG_VD               0x000
#define TG_ROD              0x460

#define HEAD_SIZE_REG       0xD0411214
#define HEAD_SIZE_1080P     0x0543094C  /* 1348 rows, 2380 */
#define HEAD_SIZE_FULL      0x0637094C  /* 1592 rows */
#define ROWS_1080P          1348
#define ROWS_FULL           1592
#define ROD_1080P           680
#define MEM1_CH3            0xD0420200  /* EDMAC channel 3: +0x50 yb/xb */
#define MEM1_CH3_PACK       0xD0422224
#define ROD_FULL            800
#define VD_1080P_23976      2085

/* full 3:2 wanted (Sensor mode 3x3, Readout 3:2, 23.976): from the mode
 * settings below; ML core's boot patch reads ML/SETTINGS/d8_32.on */
static int full32_enabled = 0;
static int d8_readout_code(void);
static void ps_take_update(void);
static const char *vid_cant_apply(void);
static void vid_apply(void);
static void vid_read_canon(void);
static void vid_request(void);
static uint32_t canon_movie_param[10];  /* PROP_VIDEO_MODE (below) */
static void ps_cropmarks(void);
static void full32_marker(int on);

static int patched = 0;
static char status[64] = "";

/* Mem1 alignment: one transfer with fewer rows than the sensor sends (a
 * frame Canon programs with its 1348 before we put 1592 back) leaves the
 * rest of that frame at the top of every later buffer - frames start
 * ~244 lines late until LiveView restarts.
 * One transfer of exactly the rows left to the next frame start brings
 * the stream back in step. */
static volatile int realign_rows = 0;   /* task -> vsync: rows for one frame */
static int realign_restore = 0;         /* vsync: put the full height back */
static int realigns = 0;

/* The table patch lives in ML core (platform/M6II.111/full32_core.c): it
 * has to be in place before Canon's first LiveView start after a boot,
 * which is before modules load. */
extern int m6ii_full32_patch(char *status, int len);
extern void m6ii_full32_restore(void);
extern int m6ii_full32_patched(void);
extern void m6ii_full32_claim(void);

static void patch_tables(void)
{
    patched = m6ii_full32_patch(status, sizeof(status));
}

static void restore_all(void)
{
    m6ii_full32_restore();
    patched = 0;
}

/* LiveView table edits (3x1 test, readout engine): every word written is
 * saved first, so Canon's values can be put back (t_restore) */
static volatile int m1dump_request = 0, m1dump_frames = 0;    /* see m1dump_task */
static struct { uint32_t addr, old; } t_saved[400];
static int t_n = 0;

static int t_write(uint32_t addr, uint32_t value)
{
    if (MEM(addr) == value)
    {
        return 1;
    }
    if (t_n >= COUNT(t_saved))
    {
        return 0;
    }
    t_saved[t_n].addr = addr;
    t_saved[t_n].old = MEM(addr);
    t_n++;
    MEM(addr) = value;
    return 1;
}

static void tl(const char *what);

static void t_restore(void)
{
    while (t_n > 0)
    {
        t_n--;
        MEM(t_saved[t_n].addr) = t_saved[t_n].old;
    }
}

/* a u16 in the TG table, by word */
static int t_write16(uint32_t a, uint16_t v)
{
    uint32_t shift = (a & 2) * 8, w = a & ~3;
    return t_write(w, (MEM(w) & ~(0xFFFF << shift)) | ((uint32_t) v << shift));
}

/* ---- 3x1 test: every line, 3x horizontal binning ("1x3") ----
 * Unresolved: the frames are full length and every line is read, but on
 * most LiveView starts half of each 149-px column group comes out as
 * noise (one of two channels per group out of sync).  Slot 15 keeps
 * drive mode 5 (3x horizontally, HD 1536) and its horizontal window; the
 * vertical sampling registers come from slot 7 (1:1 rows at the same HD
 * 1536: Canon's 5x magnify); window and line count are computed for a
 * centred band (rules from slots 7/15/16/18): c = window units (2 rows)
 * + 39, 0x020 = 16c + 1168, 0x060 = c << 28 | (c + 41) << 14 | 0x3FFF,
 * 0x098 low 12 bits = c - 6, 0x0A8 bits 16-27 = c, ROD = c + 2, lines
 * R = 2c - 4; EngIO heights R - 1 (HEAD, D041224C) and R - 37 (paths; the
 * half-height ones stay Canon's, as full 3:2: AF).  Side-list frames are
 * changed only where they repeat the main list's.  All written at once
 * (the same set written word by word over USB hung the camera while
 * LiveView ran); applies at the next LiveView start, 1080p 23.976 only. */
static int test31 = 0;                  /* menu, not saved: 1 2.39:1, 2 16:9 */
static int test31_patched = 0;
static int t31_rows = 0;                /* Mem1 lines R */
/* SetP (the next frame's settings point), ROD - 204 as Canon's own modes
 * and later in 100-line steps: frames stopped at ~1592 lines with ROD -
 * 204, Canon's 476 gave misparsed data (settings mid-readout), VD - 100
 * froze.  Does the frame end follow SetP? (menu, not saved) */
static int t31_setp = 0;
static int t31_setp_patched = 0;
static int t31_ob_row = 0;              /* the next frame's optical black in Mem1 */
/* input stage lane order D0412244: 1080p's 0x73625140 or the magnify
 * slot's 0x73516240 (whose sampling registers, 0x002 1414 -> 1616
 * included, the test uses); runs came out clean on some boots and
 * garbled (noise bands) on others with 1080p's (menu, not saved) */
static int t31_lane = 0;
static int t31_lane_patched = 0;
/* where the vertical sampling registers come from: 0 slot 7 (magnify),
 * 1 slot 7 with 0x002 kept binned (1414: 1616 in every 1:1 slot, 1414 in
 * every binned one - horizontal?), 2 slot 4 (RAW burst: an earlier 3x1
 * run with drive mode 5 was clean).  All three garbled in the same runs
 * (menu, not saved) */
static int t31_donor = 0;
static int t31_donor_patched = 0;
#define T31_DONOR_SLOT      (t31_donor == 2 ? 4 : SLOT_MAGNIFY)
/* measure / dump only after a LiveView restart: a re-patch while
 * LiveView runs reaches the sensor half-applied (per-frame list) */
static volatile int t31_wait_lv_stop = 0;
static int t31_c = 0;
#define SLOT_MAGNIFY        7
#define T31_WIDTH_SENSOR    (2336 * 3)  /* image columns from 48, 3x binned */
#define T31_UNITS_MID       1226        /* full-height window 62..2391 */
static const uint16_t t31_donor_regs[] = {
    0x002, 0x034, 0x07D, 0x07F, 0x08D, 0x094, 0x095, 0x096, 0x097,
    0x099, 0x09E, 0x09F, 0x0A0, 0x0A1, 0x0A3, 0x0A4, 0x0A5, 0x0A9,
};

static uint64_t cmos_data(uint32_t a)
{
    return (uint64_t) (MEM(a) & 0xFFFFF) << 32 | MEM(a + 4);
}

/* a slot's main-list frame of a register, or 0 */
static uint32_t cmos_frame(uint32_t cmos, int slot, uint32_t reg)
{
    uint32_t a = cmos + CMOS_MAIN + slot * CMOS_MAIN_STRIDE;
    for (int i = 0; i < CMOS_MAIN_WORDS / 2; i++)
    {
        if (MEM(a + 8 * i) >> 20 == reg)
        {
            return a + 8 * i;
        }
    }
    return 0;
}

/* the 3x1 value of a register (from slot 15's `old`); 0 if not one of ours */
static int t31_value(uint32_t cmos, uint32_t reg, uint64_t old, int c, int start, int units, uint64_t *out)
{
    if (reg == 0x002 && t31_donor == 1)
    {
        return 0;
    }
    for (int i = 0; i < COUNT(t31_donor_regs); i++)
    {
        if (t31_donor_regs[i] == reg)
        {
            uint32_t f = cmos_frame(cmos, T31_DONOR_SLOT, reg);
            *out = f ? cmos_data(f) : old;
            return f != 0;
        }
    }
    uint32_t f;
    switch (reg)
    {
        case 0x07E:
            f = cmos_frame(cmos, T31_DONOR_SLOT, reg);
            *out = (cmos_data(f) & ~0xFFFFFFULL) | (uint64_t) start << 12 | (start + units);
            return f != 0;
        case 0x020:
            *out = (old & ~0xFFFFULL) | (16 * c + 1168);
            return 1;
        case 0x060:
            *out = (uint64_t) c << 28 | (uint64_t) (c + 41) << 14 | 0x3FFF;
            return 1;
        case 0x098:
            f = cmos_frame(cmos, T31_DONOR_SLOT, reg);
            *out = (cmos_data(f) & ~0xFFFULL) | (c - 6);
            return f != 0;
        case 0x0A8:
            *out = (old & ~(0xFFFULL << 16)) | (uint64_t) c << 16;
            return 1;
    }
    return 0;
}

#define TG_SETP             0x230       /* "[SetP]" (0xE06D62D6) */
#define TG_HD               0x8C0       /* line period, clocks */
#define T31_TG(field)       (*(volatile uint16_t *)(MEM(SDRV_STATE + 0x78) + (field) + TG_MODE_1080P * TG_STRIDE + TG_FPS_23976 * 2))
#define T31_HEAD_SIZE       ((uint32_t) (t31_rows - 1) << 16 | (HEAD_SIZE_1080P & 0xFFFF))

static int test31_patch(int aspect)
{
    uint32_t cmos = MEM(SEQ_STATE + 8), eng = MEM(SEQ_STATE + 0xC), tg = MEM(SDRV_STATE + 0x78);
    if (!cmos || !eng || !tg)
    {
        return 0;
    }
    /* Canon's values as measured (full 3:2 undone first) */
    uint32_t f15 = cmos_frame(cmos, SLOT_1080P, 0x07D), f7 = cmos_frame(cmos, SLOT_MAGNIFY, 0x07D);
    if (!f15 || !f7 || (cmos_data(f15) & 0xFFFFFF) != 0x211227 || (cmos_data(f7) & 0xFFFFFF) != 0x010010 ||
        *(volatile uint16_t *)(tg + TG_ROD + TG_MODE_1080P * TG_STRIDE + TG_FPS_23976 * 2) != ROD_1080P)
    {
        tl("3x1 test: tables not as expected");
        return 0;
    }
    int rows = (aspect == 2 ? T31_WIDTH_SENSOR * 9 / 16 : T31_WIDTH_SENSOR * 100 / 239) & ~7;
    int units = rows / 2;
    int start = T31_UNITS_MID - units / 2;
    int c = units + 39;
    int R = 2 * c - 4;

    /* CMOS main list, then side-list frames that repeat the main list's */
    uint32_t main = cmos + CMOS_MAIN + SLOT_1080P * CMOS_MAIN_STRIDE;
    static const struct { uint32_t off, stride; } side[] = { { 0x684C, 0x24 }, { 0x53A0, 0xFC }, { 0x70B0, 0x1C } };
    static uint32_t w_addr[160], w_val[160];
    int n = 0;
    for (int k = -1; k < (int) COUNT(side); k++)
    {
        uint32_t a = k < 0 ? main : cmos + side[k].off + SLOT_1080P * side[k].stride;
        int frames = k < 0 ? CMOS_MAIN_WORDS / 2 : (int) side[k].stride / 8;
        for (int i = 0; i < frames; i++)
        {
            uint32_t fa = a + 8 * i;
            if (MEM(fa) == 0xFFFFFFFF && MEM(fa + 4) == 0xFFFFFFFF)
            {
                break;
            }
            uint32_t reg = MEM(fa) >> 20;
            uint64_t old = cmos_data(fa), v;
            /* Canon edits side list +0x53A0's 0x060 in place every frame
             * (bits 14-27 = VD - 33; FUN_e06A4DD4) and sends it: our line
             * count goes in there too (the first runs left it at 686 ->
             * 1372-line frames), Canon's bits 14-27 stay */
            int per_frame = k >= 0 && reg == 0x060;
            if (k >= 0 && !per_frame)
            {
                uint32_t mf = cmos_frame(cmos, SLOT_1080P, reg);
                /* compare with Canon's main value: the main list is not written yet */
                if (!mf || cmos_data(mf) != old)
                {
                    continue;
                }
            }
            if (!t31_value(cmos, reg, old, c, start, units, &v))
            {
                continue;
            }
            if (per_frame)
            {
                v = (v & ~(0x3FFFULL << 14)) | (old & (0x3FFFULL << 14));
            }
            if (v == old)
            {
                continue;
            }
            if (n + 2 > COUNT(w_addr))
            {
                tl("3x1 test: too many values");
                return 0;
            }
            w_addr[n] = fa;     w_val[n++] = (MEM(fa) & 0xFFF00000) | (uint32_t) (v >> 32);
            w_addr[n] = fa + 4; w_val[n++] = (uint32_t) v;
        }
    }
    /* EngIO heights */
    /* + list 0x3E68 (sent at the LiveView start and every frame): HEAD
     * windows D0411510-0560, height - 1 in the high half (slot 15 0x547,
     * slot 16 0x637) */
    static const struct { uint32_t off, stride; } lists[] = { { 0xF40, 0x78 }, { 0x3340, 0x68 }, { 0x1A38, 0xB0 }, { 0x3E68, 0x80 } };
    for (int k = 0; k < COUNT(lists); k++)
    {
        uint32_t a = eng + lists[k].off + SLOT_1080P * lists[k].stride;
        for (int i = 0; i < 64 && MEM(a + 8 * i) != 0xFFFFFFFF; i++)
        {
            uint32_t reg = MEM(a + 8 * i), val = MEM(a + 8 * i + 4), v = val;
            if (reg == 0xD0411514 || reg == 0xD041151C || reg == 0xD0411520 ||
                reg == 0xD0411554 || reg == 0xD041155C || reg == 0xD0411560)
            {
                v = (uint32_t) (R - 1) << 16 | (val & 0xFFFF);
            }
            else if (reg == 0xD0411214 || reg == 0xD041121C || reg == 0xD0411220)
            {
                v = (uint32_t) (R - 1) << 16 | (val & 0xFFFF);
            }
            else if (reg == 0xD041224C)
            {
                v = R - 1;
            }
            else if (reg == 0xD0412244 && t31_lane)
            {
                v = 0x73516240;
            }
            else if (reg == 0xD04115EC || reg == 0xD041163C || reg == 0xD041168C ||
                     reg == 0xD04116DC || reg == 0xD041176C || reg == 0xD04117EC)
            {
                v = (uint32_t) (R - 37) << 16 | (val & 0xFFFF);
            }
            if (v != val && n < COUNT(w_addr))
            {
                w_addr[n] = a + 8 * i + 4; w_val[n++] = v;
            }
        }
    }
    /* all at once */
    int ok = 1;
    for (int i = 0; i < n; i++)
    {
        ok &= t_write(w_addr[i], w_val[i]);
    }
    ok &= t_write16(tg + TG_ROD + TG_MODE_1080P * TG_STRIDE + TG_FPS_23976 * 2, c + 2);
    /* the next frame's settings point: ROD - 204 in every HD 1536 mode */
    /* 0: ROD - 204; 1-3: +100 / +200 / +300 (1985 froze, 476 garbled) */
    int setp = c + 2 - 204 + 100 * t31_setp;
    ok &= t_write16(tg + TG_SETP + TG_MODE_1080P * TG_STRIDE + TG_FPS_23976 * 2, setp);
    if (!ok)
    {
        t_restore();
        return 0;
    }
    t31_rows = R;
    t31_c = c;
    test31_patched = aspect;
    t31_setp_patched = t31_setp;
    t31_lane_patched = t31_lane;
    t31_donor_patched = t31_donor;
    t31_ob_row = 0;
    t31_wait_lv_stop = 1;
    m1dump_frames = 0;
    char msg[96];
    snprintf(msg, sizeof(msg), "3x1 test: %d values, band %d rows, c %d, %d lines", t_n, rows, c, R);
    tl(msg);
    return 1;
}

/* ---- Readout engine (Sensor mode > Readout): a Canon slot's vertical
 * extent, for 3x3 at 2.39:1 (slot 15) and 2x2 at 2.39:1 / 3:2 (slot 11).
 * 3x3 at 3:2 is full 3:2 above (Canon's slot 16 copied).  Everything
 * moves by deltas from the slot's own values (rules from slots 7, 11,
 * 15, 16, 18 and the 3x1 runs; they give slot 16 from slot 15: c 798,
 * 1588 lines vs Canon's 1592):
 * - window 0x07E (2-row units) centred on the sensor; lines R = window
 *   rows / vertical sampling + the slot's optical black lines;
 * - line count c = c0 + (R - R0) / 2: 0x0A8 bits 16-27, 0x060 bits 28+
 *   (and bits 14-27 + the same delta; in the side list +0x53A0, which
 *   Canon sends every frame, those bits stay Canon's: VD - k), 0x098 low
 *   bits where they follow c (c - 6), 0x020 + 16 x delta (Canon writes
 *   the frame length there every frame anyway);
 * - TG ROD x c / c0 and SetP keeping Canon's distance below ROD, for the
 *   running drive mode and fps index;
 * - EngIO heights (HEAD, D041224C, paths, HEAD windows 0x3E68): values
 *   within 64 of R0 move by R - R0 (they track the raw frame, as in
 *   1080p), others scale by R / R0 (4K's display / AF paths are scaled
 *   down); the half-height path registers stay Canon's, as full 3:2.
 * Written at once; applies at the next LiveView start. */
#define RO_3X3_239          2
#define RO_2X2_239          3
#define RO_2X2_32           4
#define DTS_MASTER          0x49620     /* +8 fps index, +0xC drive mode */
static int ro_patched = 0;              /* the code applied */
static int ro_fps_patched = -1;
static int ro_mode_patched = -1;        /* the drive mode whose timing was patched */

/* the sensor slot a readout code is for */
static int ro_target_slot(int code)
{
    return code == RO_3X3_239 ? SLOT_1080P : 11;
}
static int ro_slot = 0, ro_R0 = 0, ro_R = 0;

/* the drive mode and rate (TG table column) a readout's slot runs at:
 * 3x3 drive mode 5, 2x2 (4K) 17 (SensorMode(), 0xE06D591C), the rate
 * index from Canon's movie rate (59.94 0, 50 1, 29.97 2, 25 3, 23.976 4;
 * the same columns DTS_MASTER showed while those slots ran).  From
 * Canon's settings, not DTS_MASTER: the slot may not be running yet
 * (Canon's 4K comes back as 1080p after a restart; d8_modes sets 4K and
 * LiveView restarts straight into slot 11) */
static int ro_timing(int code, int *mode, int *fps)
{
    static const int rates[] = { 5994, 5000, 2997, 2500, 2398 };
    *mode = code == RO_3X3_239 ? 5 : 17;
    *fps = -1;
    for (int i = 0; i < COUNT(rates); i++)
    {
        if (ABS((int) canon_movie_param[2] - rates[i]) <= 2)
        {
            *fps = i;
        }
    }
    return *fps >= 0;
}
/* test: apply only part of a readout, to tell which part stops LiveView:
 * 0 all, 1 sensor + EngIO only (Canon's timing), 2 timing only (ROD /
 * SetP, Canon's sensor lists), 3 sensor (CMOS) lists only, 4 EngIO
 * heights only.  Not saved: a boot is always "all".  No menu entry
 * while the 2.39:1 read-outs are off (RO_239_READOUT). */
static int ro_part = 0;
static int ro_part_patched = 0;

static int ro_scale(int v, int r0, int r)
{
    return (ABS(v - r0) < 64) ? v + (r - r0) : v * r / r0;
}

static uint32_t eng_frame(uint32_t eng, uint32_t off, uint32_t stride, int slot, uint32_t reg)
{
    uint32_t a = eng + off + slot * stride;
    for (int i = 0; i < 64 && MEM(a + 8 * i) != 0xFFFFFFFF; i++)
    {
        if (MEM(a + 8 * i) == reg)
        {
            return a + 8 * i;
        }
    }
    return 0;
}

static int ro_patch(int code)
{
    uint32_t cmos = MEM(SEQ_STATE + 8), eng = MEM(SEQ_STATE + 0xC), tg = MEM(SDRV_STATE + 0x78);
    if (!cmos || !eng || !tg)
    {
        return 0;
    }
    int slot = ro_target_slot(code);
    int vs = code == RO_3X3_239 ? 3 : 2;                /* vertical sampling */
    int img_w = code == RO_3X3_239 ? 2336 : 3488;       /* image columns */
    int lines = code == RO_2X2_32 ? img_w * 2 / 3 : img_w * 100 / 239;

    uint32_t f7e = cmos_frame(cmos, slot, 0x07E), fa8 = cmos_frame(cmos, slot, 0x0A8);
    uint32_t fhead = eng_frame(eng, 0xF40, 0x78, slot, HEAD_SIZE_REG);
    if (!f7e || !fa8 || !fhead)
    {
        tl("readout: slot lists not found");
        return 0;
    }
    uint64_t w7e = cmos_data(f7e);
    int units0 = (int) (w7e & 0xFFF) - (int) ((w7e >> 12) & 0xFFF);
    int c0 = (cmos_data(fa8) >> 16) & 0xFFF;
    int R0 = (MEM(fhead + 4) >> 16) + 1;
    int ob = R0 - units0 * 2 / vs;
    int units = MIN(lines * vs / 2, 2391 - 62);
    int R = (units * 2 / vs + ob + 1) & ~1;
    int start = 1227 - units / 2;
    if (vs == 3)
    {
        /* 3x3 bins 3 units (6 rows) per group: every Canon 3x3 window
         * (slots 14, 15, 16) starts at 1 and ends at 0 mod 3.  Off that
         * grid (2.39:1 was 495..1960) the sensor never sent a frame: no
         * VD interrupt after SyncProcess Start, LiveView black.  So trim
         * the slot's own window by the same multiple of 3 at each end */
        int start0 = (w7e >> 12) & 0xFFF;
        int t = ((units0 - units) / 2 + 1) / 3 * 3;
        start = start0 + t;
        units = units0 - 2 * t;
        R = R0 - 4 * t / 3;
    }
    int c = c0 + (R - R0) / 2;
    int dc = c - c0;

    /* timing, for the running mode and rate */
    int mode, fps;
    ro_timing(code, &mode, &fps);
    volatile uint16_t *t = (volatile uint16_t *)(tg + mode * TG_STRIDE + fps * 2);
    int vd = t[TG_VD / 2], rod0 = t[TG_ROD / 2], setp0 = t[TG_SETP / 2], hd = t[TG_HD / 2];
    /* Canon's own tables (every drive mode, ROM1): ROD is never less than
     * c + the mode's margin (2 in all 3x3-family modes, HD 1536; 174 in
     * 4K), and with HD 1536 SetP is always ROD - 204.  With HD 1104 (2x2)
     * SetP stays 723 for other RODs at the same c.  Scaling ROD by c / c0
     * alone shrank the margin with the read-out: 3x3 2.39:1 got ROD = c + 1
     * and hung LiveView's start.  A read-out that grows keeps the scaled
     * values, as before (3:2 works). */
    int rod = MAX(rod0 * c / c0, c + (rod0 - c0));
    int setp = (hd == 1536) ? rod - 204
             : (rod >= rod0) ? rod - (rod0 - setp0)
             : setp0;
    if (fps < 0 || fps > 13 || !rod0 || !c0 || rod >= vd - 20 || setp < 20)
    {
        char msg[96];
        snprintf(msg, sizeof(msg), "readout: no room (mode %d fps %d VD %d ROD %d -> %d)", mode, fps, vd, rod0, rod);
        tl(msg);
        return 0;
    }

    static uint32_t w_addr[160], w_val[160];
    int n = 0;
    #define RO_W(addr, val) do { if (n < COUNT(w_addr)) { w_addr[n] = (addr); w_val[n++] = (val); } } while (0)

    /* CMOS main list, then side-list frames */
    static const struct { uint32_t off, stride; } side[] = { { 0x684C, 0x24 }, { 0x53A0, 0xFC }, { 0x70B0, 0x1C } };
    for (int k = -1; k < (int) COUNT(side); k++)
    {
        uint32_t a = k < 0 ? cmos + CMOS_MAIN + slot * CMOS_MAIN_STRIDE : cmos + side[k].off + slot * side[k].stride;
        int frames = k < 0 ? CMOS_MAIN_WORDS / 2 : (int) side[k].stride / 8;
        for (int i = 0; i < frames; i++)
        {
            uint32_t fa = a + 8 * i;
            if (MEM(fa) == 0xFFFFFFFF && MEM(fa + 4) == 0xFFFFFFFF)
            {
                break;
            }
            uint32_t reg = MEM(fa) >> 20;
            uint64_t old = cmos_data(fa), v = old;
            uint32_t mf = cmos_frame(cmos, slot, reg);
            uint64_t main_old = mf ? cmos_data(mf) : old;
            int same = old == main_old;
            switch (reg)
            {
                case 0x07E:
                    if (same) v = (old & ~0xFFFFFFULL) | (uint64_t) start << 12 | (start + units);
                    break;
                case 0x0A8:
                    if (same) v = (old & ~(0xFFFULL << 16)) | (uint64_t) c << 16;
                    break;
                case 0x060:
                {
                    int f2 = (main_old >> 14) & 0x3FFF;
                    uint64_t low = (k >= 0) ? (old & (0x3FFFULL << 14)) : (uint64_t) ((f2 + dc) & 0x3FFF) << 14;
                    v = (uint64_t) c << 28 | low | (old & 0x3FFF);
                    break;
                }
                case 0x098:
                    if (same && (int) (old & 0xFFF) == c0 - 6) v = (old & ~0xFFFULL) | (c - 6);
                    break;
                case 0x020:
                    if (same) v = (old & ~0xFFFFULL) | (((old & 0xFFFF) + 16 * dc) & 0xFFFF);
                    break;
            }
            if (v != old)
            {
                RO_W(fa, (MEM(fa) & 0xFFF00000) | (uint32_t) (v >> 32));
                RO_W(fa + 4, (uint32_t) v);
            }
        }
    }
    int n_cmos = n;                     /* EngIO values from here on */
    /* EngIO heights */
    static const struct { uint32_t off, stride; } lists[] = { { 0xF40, 0x78 }, { 0x3340, 0x68 }, { 0x1A38, 0xB0 }, { 0x3E68, 0x80 } };
    int path_h = 0;                     /* D04115EC's new height - 1 */
    for (int k = 0; k < COUNT(lists); k++)
    {
        uint32_t a = eng + lists[k].off + slot * lists[k].stride;
        for (int i = 0; i < 64 && MEM(a + 8 * i) != 0xFFFFFFFF; i++)
        {
            uint32_t reg = MEM(a + 8 * i), val = MEM(a + 8 * i + 4), v = val;
            int high = reg == 0xD0411214 || reg == 0xD041121C || reg == 0xD0411220 ||
                       reg == 0xD04115EC || reg == 0xD041163C || reg == 0xD041168C ||
                       reg == 0xD04116DC || reg == 0xD041176C || reg == 0xD04117EC ||
                       reg == 0xD0411514 || reg == 0xD041151C || reg == 0xD0411520 ||
                       reg == 0xD0411554 || reg == 0xD041155C || reg == 0xD0411560;
            if (high)
            {
                /* heights - 1 in the high half (HEAD: rows - 1) */
                v = (uint32_t) (ro_scale((val >> 16) + 1, R0, R) - 1) << 16 | (val & 0xFFFF);
                if (reg == 0xD04115EC)
                {
                    path_h = v >> 16;
                }
            }
            else if ((reg == 0xD041184C || reg == 0xD0411898 || reg == 0xD041189C) && R < R0 && path_h)
            {
                /* half-height paths: (D04115EC + 1) / 2 - 1 in every Canon
                 * slot.  A taller read-out leaves them at Canon's (cut off,
                 * as full 3:2); a shorter one starved them (2.39:1: 667
                 * rows expected of 499 - black LiveView, then shutdown) */
                v = (uint32_t) ((path_h + 1) / 2 - 1) << 16 | (val & 0xFFFF);
            }
            else if (reg == 0xD041224C)
            {
                v = ro_scale(val, R0, R);
            }
            if (v != val)
            {
                RO_W(a + 8 * i + 4, v);
            }
        }
    }
    #undef RO_W
    if (n >= (int) COUNT(w_addr))
    {
        tl("readout: too many values");
        return 0;
    }
    int ok = 1;
    if (ro_part != 2)
    {
        for (int i = (ro_part == 4 ? n_cmos : 0); i < (ro_part == 3 ? n_cmos : n); i++)
        {
            ok &= t_write(w_addr[i], w_val[i]);
        }
    }
    if (ro_part == 2 || ro_part == 3)
    {
        R = R0;                         /* Mem1 keeps Canon's rows */
    }
    if (ro_part == 0 || ro_part == 2)
    {
        ok &= t_write16(tg + TG_ROD + mode * TG_STRIDE + fps * 2, rod);
        ok &= t_write16(tg + TG_SETP + mode * TG_STRIDE + fps * 2, setp);
    }
    if (!ok)
    {
        t_restore();
        tl("readout: patch failed");
        return 0;
    }
    ro_patched = code;
    ro_part_patched = ro_part;
    ro_fps_patched = fps;
    ro_mode_patched = mode;
    ro_slot = slot;
    ro_R0 = R0;
    ro_R = R;
    char msg[192];
    snprintf(msg, sizeof(msg), "readout %d part %d: slot %d, drive mode %d fps %d HD %d, %d -> %d lines, window %d..%d, c %d -> %d, ROD %d -> %d, SetP %d -> %d, %d values",
             code, ro_part, slot, mode, fps, hd, R0, R, start, start + units, c0, c, rod0, rod, setp0, setp, t_n);
    tl(msg);
    return 1;
}

static int mem1_wanted_rows(void)
{
    if (!lv || MEM(SEQ_MODE) != SLOT_1080P)
    {
        return 0;
    }
    uint32_t head = MEM(HEAD_SIZE_REG);
    return (head == HEAD_SIZE_FULL) ? ROWS_FULL : (head == HEAD_SIZE_1080P) ? ROWS_1080P : 0;
}

/* runs right after Canon's read-out done handling, ~40 ms before Mem1Path
 * programs the next transfer from +0x1C */
static volatile int tl_first_vsync = 0;     /* boot timeline, see tl() */

/* Mem1 dump (3x1 test): ch 3's rows to ML/LOGS/M1Snnnn.RAW after a
 * 256-byte text header (buffer, row bytes, rows, pack register) */
#define M1DUMP_BYTES (17 << 20)
static char m1dump_hdr[256];

/* 3x1 test: Mem1 row where the next sensor frame's optical black starts
 * (32 rows within 16 of the first frame's OB level, rows 10-60), i.e.
 * the sensor frame length + ~8; 0 if none.  14-bit, LSB first. */
static int m1_row_mean(uint8_t *row)
{
    int sum = 0;
    for (int i = 100; i < 2300; i += 40)
    {
        int b = i * 14;
        uint32_t v = row[b / 8] | row[b / 8 + 1] << 8 | row[b / 8 + 2] << 16;
        sum += (v >> (b % 8)) & 0x3FFF;
    }
    return sum / 55;
}

static int m1_next_ob_row(uint8_t *buf, uint32_t xb, uint32_t yb)
{
    int ob = 0;
    for (int r = 10; r < 60; r++)
    {
        ob += m1_row_mean(buf + r * xb);
    }
    ob /= 50;
    int run = 0;
    for (uint32_t r = 200; r < yb; r++)
    {
        run = (m1_row_mean(buf + r * xb) < ob + 16) ? run + 1 : 0;
        if (run == 32)
        {
            return r - 31;
        }
    }
    return 0;
}

/* garbled (misparsed lanes: noise bands) if same-colour neighbours differ
 * a lot: clean frames ~60, garbled ~3200 */
static int m1_neighbour_diff(uint8_t *buf, uint32_t xb, uint32_t yb)
{
    int sum = 0, n = 0;
    for (uint32_t r = 300; r + 1 < yb; r += 200)
    {
        uint8_t *row = buf + r * xb;
        for (int i = 100; i < 2300; i += 7)
        {
            int b = i * 14, b2 = (i + 2) * 14;
            int a = ((row[b / 8] | row[b / 8 + 1] << 8 | row[b / 8 + 2] << 16) >> (b % 8)) & 0x3FFF;
            int c = ((row[b2 / 8] | row[b2 / 8 + 1] << 8 | row[b2 / 8 + 2] << 16) >> (b2 % 8)) & 0x3FFF;
            sum += ABS(a - c);
            n++;
        }
    }
    return n ? sum / n : 0;
}

static void m1dump_task(void *unused)
{
    while (1)
    {
        msleep(20);
        if (!m1dump_request)
        {
            continue;
        }
        m1dump_request = 0;
        uint32_t base = MEM(MEM1_CH3 + 0x28 * 4);
        uint32_t xy = MEM(MEM1_CH3 + 0x14 * 4);
        uint32_t xb = xy & 0xFFFF, yb = (xy >> 16) + 1;
        uint32_t pack = MEM(MEM1_CH3_PACK);
        uint32_t head = MEM(HEAD_SIZE_REG);
        uint32_t len = MIN(M1DUMP_BYTES, xb * yb);
        memset(m1dump_hdr, ' ', sizeof(m1dump_hdr));
        int n = snprintf(m1dump_hdr, sizeof(m1dump_hdr),
                         "M1DUMP base %x xb %d yb %d pack %x head %x state %dx%d len %d",
                         (int) base, (int) xb, (int) yb, (int) pack, (int) head,
                         (int) MEM(MEM1_STATE + 0x18), (int) MEM(MEM1_STATE + 0x1C), (int) len);
        m1dump_hdr[n] = ' ';
        m1dump_hdr[sizeof(m1dump_hdr) - 1] = '\n';
        char name[32] = "ML/LOGS/M1_1TO1.RAW";
        if (test31_patched)
        {
            snprintf(name, sizeof(name), "ML/LOGS/M1S%04d.RAW", T31_TG(TG_SETP));
        }
        FILE *f = FIO_CreateFile(name);
        if (f)
        {
            FIO_WriteFile(f, m1dump_hdr, sizeof(m1dump_hdr));
            if (base && len)
            {
                FIO_WriteFile(f, UNCACHEABLE(base), len);
            }
            FIO_CloseFile(f);
        }
        tl(f ? "1:1: Mem1 dumped to ML/LOGS/M1_1TO1.RAW" : "1:1: Mem1 dump: could not create the file");
        if (test31_patched && base && xb >= 4172)
        {
            t31_ob_row = m1_next_ob_row((uint8_t *) UNCACHEABLE(base), xb, yb);
            int nd = m1_neighbour_diff((uint8_t *) UNCACHEABLE(base), xb, yb);
            char line[320];
            int n = snprintf(line, sizeof(line), "%d ms: 3x1 %s, donor %s, c %d, %d lines, ROD %d, SetP %d, lanes %08x, shutter 1/%d ISO %d, sensor %d C: %s (neighbour diff %d), next frame's OB at Mem1 row %d%s, %s\n",
                get_ms_clock(), test31_patched == 2 ? "16:9" : "2.39:1",
                t31_donor_patched == 2 ? "slot 4" : t31_donor_patched == 1 ? "slot 7, 002 binned" : "slot 7", t31_c, t31_rows,
                T31_TG(TG_ROD), T31_TG(TG_SETP), (int) MEM(0xD0412244), raw2shutter_ms(lens_info.raw_shutter) ? 1000 / raw2shutter_ms(lens_info.raw_shutter) : 0,
                raw2iso(lens_info.raw_iso), efic_temp - 128, nd > 500 ? "GARBLED" : "clean", nd,
                t31_ob_row, t31_ob_row ? "" : " (none)", name + 8);
            FILE *lf = FIO_OpenFile("ML/LOGS/3X1.LOG", O_RDWR | O_SYNC);
            if (!lf)
            {
                lf = FIO_CreateFile("ML/LOGS/3X1.LOG");
            }
            if (lf)
            {
                FIO_SeekSkipFile(lf, 0, SEEK_END);
                FIO_WriteFile(lf, line, n);
                FIO_CloseFile(lf);
            }
        }
    }
}

/* the last LiveView frame: none for a while = LiveView is stopped (Canon's
 * menu, a mode change), which ML's lv doesn't tell reliably on this camera */
static volatile int d8_vsync_ms = 0;
#define LV_STOPPED (get_ms_clock() - d8_vsync_ms > 300)
/* readouts change only once LiveView has run in this session: a saved
 * readout that hangs LiveView's start would otherwise hang every boot */
#define LV_SEEN (d8_vsync_ms != 0)

/* LiveView picture centred for 2x2 3:2 (test): the development path's
 * first block (ROM 0xE15EEBAA = RAM 0x022F0052, from SapSetPath) fills a
 * struct {.., +4 x0, +8 w, +12 y0, +16 h} and writes the rectangle
 * D04130F8 = size - 1 (from the SapPath entry's height), D04130FC =
 * start (y0, x0), hard-coded (0, 0), and D0413100 = end (h, w): the
 * picture is the frame's top 2000 rows.  Writing the start live froze
 * LiveView; so Canon's RAM code is patched instead, for the next
 * LiveView start: start row 178 (movs r0, #178 for ldrh r0, [r5, #12]),
 * end row 2178 (struct +16 loaded from a spare word holding 2178, for
 * the height: ldr r0, [r0, #124] for [r0, #4]); the size stays 2000.
 * The spare word: SapPath entry 8's 4th word (2384 x 196, a mode our
 * modes don't run), 124 bytes past 4K's entry - the farthest that load
 * reaches.  4K's own 4th word (first try) froze LiveView: SapSetPath
 * also hands words 3 and 4 on as an offset (FUN_e1584596).  Even start
 * row: Bayer order.  Not saved. */
#define LVC_ENTRY_W3        0x0233E5F0  /* SapPath entry 8, 4th word (0) */
#define LVC_Y               178
static int lvc_test = 0;                /* menu, not saved */
static int lvc_patched = 0;
static const struct { uint32_t addr, canon, patched; } lvc_code[] = {
    { 0x022F0068, 0x68402102, 0x6FC02102 },     /* +16 from entry +124 */
    { 0x022F009C, 0x89A86048, 0x20B26048 },     /* start row: movs r0, #178 */
};

static void lvc_apply(int on)
{
    if (on == lvc_patched)
    {
        return;
    }
    if (!on)
    {
        for (int i = 0; i < COUNT(lvc_code); i++)
        {
            unpatch_memory(lvc_code[i].addr);
        }
        MEM(LVC_ENTRY_W3) = 0;
        lvc_patched = 0;
        tl("LiveView centre: Canon's code back");
        return;
    }
    if (MEM(LVC_ENTRY_W3) != 0 || MEM(0x0233E578) != 2000)
    {
        tl("LiveView centre: SapPath entry not as expected");
        lvc_test = 0;
        return;
    }
    struct patch p[COUNT(lvc_code)];
    for (int i = 0; i < COUNT(lvc_code); i++)
    {
        p[i] = (struct patch) {
            .addr = (uint8_t *) lvc_code[i].addr,
            .old_value = lvc_code[i].canon,
            .new_value = lvc_code[i].patched,
            .size = 4,
            .description = "d8_modes: LiveView centre (dev window start / end)",
            .is_instruction = 1,
        };
    }
    MEM(LVC_ENTRY_W3) = LVC_Y + 2000;
    int err = apply_patches(p, COUNT(lvc_code));
    char msg[80];
    snprintf(msg, sizeof(msg), "LiveView centre: code patched: %s %x (applies at the next LiveView start)", err ? "FAILED" : "ok", err);
    tl(msg);
    if (err)
    {
        MEM(LVC_ENTRY_W3) = 0;
        lvc_test = 0;
        return;
    }
    lvc_patched = 1;
}

/* Defect map for 2x2 3:2 (test): Canon's LiveView defect navigator
 * (Warp::DefmarkNavigator.c, EDMAC ch 42) walks the mode's Tune map in
 * raster order - entry = distance from the previous one (bits 16-31),
 * level (4-15), kind (0-3), 15 = end - and every Tune map ends exactly at
 * its frame: 4K's (drive modes 3/4/17, map struct *0x1A660 + 0x24) at
 * 3568 x 2004.  Every development window reaching past row ~2000 froze;
 * an empty map (the Rasen maps: end at pixel 65535) left LiveView black.
 * This builds 4K's map for the 3:2 read-out: Canon's entries moved down
 * by the rows the 3:2 window starts higher ((R - R0) / 2 = 179), the end
 * at 3568 x (R + 4), long gaps as kind-0 level-0 entries of up to 65535 px
 * (marking one pixel each at most), Canon's 0xFFFF000F filler after.
 * Written through the cache, cleaned, handed over uncached (like Canon's
 * 0x4xxxxxxx maps).  Read at each LiveView start; not saved. */
#define DEFM_STRUCT_PTR     0x1A660
#define DEFM_TUNE_4K        0x24
#define DEFM_W              3568
extern void dcache_clean(uint32_t addr, uint32_t size);
static int defm_test = 0;               /* menu, not saved */
static uint32_t defm_saved = 0;         /* Canon's 4K Tune map, 0 = not swapped */
static uint32_t *defm_ext = NULL;       /* our map (cached pointer) */
static uint32_t defm_ext_size = 0;

static int defm_put_gap(uint32_t *m, int n, uint32_t px)
{
    while (px > 0xFFFF)
    {
        m[n++] = 0xFFFF << 16;          /* kind 0, level 0 */
        px -= 0xFFFF;
    }
    if (px)
    {
        m[n++] = px << 16;
    }
    return n;
}

static uint32_t defm_build(uint32_t canon, int R, int R0)
{
    /* Canon's entries up to the end entry */
    int n0 = 0;
    uint32_t sum = 0, d_end = 0;
    for (; n0 < 100000; n0++)
    {
        uint32_t e = MEM(canon + 4 * n0);
        if ((e & 0xF) == 0xF)
        {
            d_end = e >> 16;
            break;
        }
        sum += e >> 16;
    }
    if (n0 >= 100000 || sum + d_end != DEFM_W * 2004)
    {
        char msg[80];
        snprintf(msg, sizeof(msg), "defect map: 4K map not as expected (%d entries, %d px)", n0, sum + d_end);
        tl(msg);
        return 0;
    }
    uint32_t pre = DEFM_W * ((R - R0) / 2);
    uint32_t total = DEFM_W * (R + 4);
    int max = n0 + 64 + 64;
    if (!defm_ext)
    {
        uint8_t *raw = malloc(max * 4 + 64);
        if (!raw)
        {
            tl("defect map: no memory");
            return 0;
        }
        defm_ext = (uint32_t *) (((uint32_t) raw + 63) & ~63);
    }
    uint32_t *m = defm_ext;
    int n = defm_put_gap(m, 0, pre);
    /* Canon's first entry's distance now counts from our last gap entry
     * (or from pixel 0): positions move by `pre` */
    for (int i = 0; i < n0; i++)
    {
        m[n++] = MEM(canon + 4 * i);
    }
    uint32_t rest = total - pre - sum;
    while (rest > 0xFFFF)
    {
        m[n++] = 0xFFFF << 16;
        rest -= 0xFFFF;
    }
    m[n++] = rest << 16 | 0xF;          /* end, at pixel `total` */
    for (int i = 0; i < 64; i++)
    {
        m[n++] = 0xFFFF000F;
    }
    defm_ext_size = n * 4;
    dcache_clean((uint32_t) m, defm_ext_size);
    char msg[96];
    snprintf(msg, sizeof(msg), "defect map: 3:2 map %x, %d entries (Canon %d), end %d x %d", (uint32_t) UNCACHEABLE(m), n - 64, n0 + 1, DEFM_W, total / DEFM_W);
    tl(msg);
    return (uint32_t) UNCACHEABLE(m);
}

static MENU_UPDATE_FUNC(defm_update)
{
    if (defm_saved)
    {
        MENU_SET_RINFO("in %x", MEM(MEM(DEFM_STRUCT_PTR) + DEFM_TUNE_4K));
        MENU_SET_WARNING(MENU_WARN_INFO, "3:2 map in place (%d bytes); the next LiveView start uses it.", defm_ext_size);
    }
    else if (defm_test)
    {
        MENU_SET_WARNING(MENU_WARN_NOT_WORKING, "Not applied: needs the 2x2 3:2 read-out patched (ro %d, R0 %d).", ro_patched, ro_R0);
    }
}

static void defm_step(void)
{
    uint32_t st = MEM(DEFM_STRUCT_PTR);
    if (!st)
    {
        return;
    }
    int want = defm_test && ro_patched == RO_2X2_32 && ro_R0 == 2000;
    if (want && !defm_saved)
    {
        uint32_t canon = MEM(st + DEFM_TUNE_4K);
        uint32_t ext = canon ? defm_build(canon, ro_R, ro_R0) : 0;
        if (!ext)
        {
            defm_test = 0;
            return;
        }
        defm_saved = canon;
        MEM(st + DEFM_TUNE_4K) = ext;
        tl("defect map test: 4K Tune map -> 3:2 map (next LiveView start)");
    }
    else if (!want && defm_saved)
    {
        MEM(st + DEFM_TUNE_4K) = defm_saved;
        defm_saved = 0;
        tl("defect map test: Canon's 4K Tune map back");
    }
}

/* in the task: with the 2x2 3:2 read-out; the code runs only at a
 * LiveView start, so patching while LiveView runs is safe */
static void lvc_step(void)
{
    lvc_apply(lvc_test && ro_patched == RO_2X2_32 && ro_R == 2358);
}

static unsigned int d8_vsync(unsigned int ctx)
{
    d8_vsync_ms = get_ms_clock();
    if (test31_patched && t31_wait_lv_stop)
    {
        return 0;
    }
    if (ro_patched && lv && MEM(SEQ_MODE) == (uint32_t) ro_slot &&
        (MEM(HEAD_SIZE_REG) >> 16) + 1 == (uint32_t) ro_R)
    {
        /* Mem1Path takes the height from state word 7 (+0x1C) or 9
         * (+0x24, 4K): whichever holds the slot's own */
        for (int w = 7; w <= 9; w += 2)
        {
            if (MEM(MEM1_STATE + 4 * w) == (uint32_t) ro_R0)
            {
                MEM(MEM1_STATE + 4 * w) = ro_R;
                raw_set_dirty();
            }
        }
        return 0;
    }
    if (test31_patched && lv && MEM(SEQ_MODE) == SLOT_1080P && MEM(HEAD_SIZE_REG) == T31_HEAD_SIZE)
    {
        if (MEM(MEM1_STATE + 0x1C) != (uint32_t) t31_rows)
        {
            MEM(MEM1_STATE + 0x1C) = t31_rows;
            raw_set_dirty();
        }
        if (++m1dump_frames == 8)
        {
            m1dump_request = 1;
        }
        return 0;
    }
    if (!tl_first_vsync)
    {
        tl_first_vsync = get_ms_clock() | 1;
    }
    int want = mem1_wanted_rows();
    if (!want)
    {
        return 0;
    }
    if (realign_restore)
    {
        MEM(MEM1_STATE + 0x1C) = want;
        realign_restore = 0;
    }
    else if (realign_rows && want == ROWS_FULL)
    {
        MEM(MEM1_STATE + 0x1C) = realign_rows;
        realign_rows = 0;
        realign_restore = 1;
    }
    else if (MEM(MEM1_STATE + 0x1C) != (uint32_t) want)
    {
        /* Canon puts back its size at mode changes */
        MEM(MEM1_STATE + 0x1C) = want;
        raw_set_dirty();
    }
    return 0;
}

/* mean of a few pixels across one Mem1 row (LSB-first packed) */
static int row_mean(const uint8_t *row, int bits)
{
    int sum = 0, n = 0;
    for (int x = 400; x < 2000; x += 100)
    {
        uint32_t bit = x * bits, byte = bit / 8;
        uint32_t v = row[byte] | (row[byte + 1] << 8) | (row[byte + 2] << 16);
        sum += (v >> (bit % 8)) & ((1 << bits) - 1);
        n++;
    }
    return sum / n;
}

/* The first 4 rows of a frame are optical black, rows 4-7 carry Canon's
 * frame data: bright, mid, bright, mid (14-bit ~5410/2912).  Aligned
 * buffers have them at row 4; shifted ones further down. */
static int marker_at(const int *m, int y, int hi)
{
    /* and 4 optical black rows before them: a bright scene alternates
     * bright/mid rows too (Bayer rows), which made the short-frame check
     * report '33 rows', '11xx rows' with frames that were fine */
    int black = hi * 10 / 23;
    for (int k = y - 4; k < y; k++)
    {
        if (k < 0 || ABS(m[k] - black) > black / 16)
        {
            return 0;
        }
    }
    return m[y] > hi && m[y + 2] > hi && m[y + 1] < hi && m[y - 1] < hi;
}

static int marker_misses = 0;

/* what the alignment check last saw, for the menu (it has missed shifted
 * clips: not run then, or didn't see it?) */
static int align_checks = 0;
static char align_seen[80] = "not run yet";
/* tallies by depth (index 0: 12-bit or less, 1: 14-bit): aligned,
 * shifted, no marker, skipped - the menu showed only the last result,
 * seen before/after recording (14-bit), not during it (12-bit) */
static int tally[2][4];
static char last_shift[48] = "";
static char last_other[64] = "";

/* Short frames: a second frame start inside one 1592-row buffer - the
 * sensor still has 1080p's read-out (1352 rows) while Mem1 takes 1592, so
 * each buffer ends with the next frame's first 240 rows (the split CRX
 * clips).  ML core applies the tables before the first LiveView start
 * (full32_core.c); if that missed, the menu says to open and close
 * Canon's menu (a real LiveView start; ML's pause/resume isn't one). */
static int short_seen = 0;           /* in a row */
static int short_rows = 0;           /* frame height seen */

static void check_alignment(void)
{
    static int means[ROWS_FULL];
    uint32_t pack = MEM(MEM1_CH3_PACK);
    uint32_t sz = MEM(MEM1_CH3 + 0x50);
    const uint8_t *buf = (const uint8_t *) MEM(MEM1_STATE + 0x58);
    int xb = sz & 0xFFFF, rows = (sz >> 16) + 1;
    align_checks++;
    if (!(pack & 1) || !buf || rows != ROWS_FULL || !xb)
    {
        snprintf(align_seen, sizeof(align_seen), "skipped: pack %x buf %x rows %d xb %d", pack, buf, rows, xb);
        tally[((pack >> 4) & 3) == 2][3]++;
        snprintf(last_other, sizeof(last_other), "skip: pack %x rows %d xb %d", pack, rows, xb);
        return;
    }
    int bits = 10 + 2 * ((pack >> 4) & 3);
    int hi = (2048 >> (14 - bits)) * 23 / 10;
    for (int y = 0; y < rows; y++)
    {
        means[y] = row_mean(buf + y * xb, bits);
    }
    if (marker_at(means, 4, hi))
    {
        marker_misses = 0;
        for (int y = 12; y < rows - 3; y++)
        {
            if (marker_at(means, y, hi))
            {
                snprintf(align_seen, sizeof(align_seen), "%d-bit, frames %d rows (2nd start at row %d)", bits, y - 4, y);
                short_rows = y - 4;
                if (!short_seen)
                {
                    tl("short sensor frames seen");
                }
                if (++short_seen == 2)
                {
                    /* once per occurrence: short_seen resets when aligned */
                    NotifyBox(5000, "Full 3:2: sensor frames are %d rows, not %d.\nOpen and close Canon's menu to apply.", y - 4, ROWS_FULL);
                }
                return;
            }
        }
        short_seen = 0;
        snprintf(align_seen, sizeof(align_seen), "%d-bit, marker at row 4 (aligned)", bits);
        tally[bits == 14][0]++;
        return;
    }
    for (int y = 8; y < rows - 3; y++)
    {
        if (marker_at(means, y, hi))
        {
            snprintf(align_seen, sizeof(align_seen), "%d-bit, marker at row %d: SHIFTED", bits, y);
            tally[bits == 14][1]++;
            snprintf(last_shift, sizeof(last_shift), "%d-bit row %d", bits, y);
            /* frame starts at y - 4: one transfer of that many rows ends
             * exactly there.  Twice in a row, not one odd frame. */
            if (++marker_misses >= 2)
            {
                marker_misses = 0;
                realigns++;
                printf("full32: Mem1 shifted (frame start at row %d), realigning\n", y - 4);
                realign_rows = y - 4;
            }
            return;
        }
    }
    snprintf(align_seen, sizeof(align_seen), "%d-bit, no marker; rows 4-7 %d %d %d %d (hi %d)",
             bits, means[4], means[5], means[6], means[7], hi);
    tally[bits == 14][2]++;
    snprintf(last_other, sizeof(last_other), "%d-bit no marker: %d %d %d %d", bits, means[4], means[5], means[6], means[7]);
}

/* boot timeline (console): can the patch land before Canon's first
 * LiveView start, so the timing generator gets ROD 800 straight away? */
static int tl_tables = 0, tl_vsync_printed = 0;

/* the console keeps only its last lines: also into a buffer, read over
 * USB as PTP mailbox slot 4 "d8_modes" */
static char tl_buf[1024];
static volatile uint32_t tl_len = 0;

extern WEAK_FUNC(ret_0) void ptp_mailbox_set(int slot, const char *name, void *ptr, volatile uint32_t *len);

static void tl_add(const char *line)
{
    if (!tl_len)
    {
        ptp_mailbox_set(4, "d8_modes", tl_buf, &tl_len);
    }
    int n = MIN((int) strlen(line), (int) sizeof(tl_buf) - (int) tl_len);
    memcpy(tl_buf + tl_len, line, n);
    tl_len += n;
}

static void tl(const char *what)
{
    char line[224];
    snprintf(line, sizeof(line), "d8_modes %6d ms: %s (lv %d)\n", get_ms_clock(), what, lv);
    printf("%s", line);
    tl_add(line);
    /* also through Canon's logger: dmlog's autosave keeps it across a
     * battery pull (class 0x9e = SensorDrive, not dropped by Quiet) */
    DryosDebugMsg(0x9e, 3, "%s", line);
}

/* LiveView picture for 2x2 3:2 (test): Canon develops the LiveView
 * image (D0413xxx) from its LiveView geometry tables in ROM0, 3568 x 2000
 * for 4K, so with the 2358-row read-out it shows the frame's top.  Those
 * tables (one 64 kB page, read through the getters at 0xE069438E..) are
 * raised to the 3:2 frame through ML's MMU remap: frame and active
 * heights, the developed output (MvRawProPath dump size, 3488 x ...) and
 * its half-size vector output.  Moving only the active area's y (41 ->
 * 220) froze LiveView: MvRawProDs asserts offset + size within the frame.
 * Only with the 2x2 3:2 tables, and only while slot 11 isn't running (it
 * applies at the next LiveView start, like the readout). */
/* all ten froze LiveView's start, and step 1 alone too; every run where
 * the patch applied froze, the one where it failed (MMU page taken)
 * didn't.  Step 1 now remaps the page with Canon's value unchanged, to
 * tell the remap from the height.  Steps: 1 remap only, 2 frame, 3 +
 * active area, 4 + dump / vector buffer sizes, 5 + output (all) */
extern WEAK_FUNC(ret_0) int mmu_reserve_remap_pages(int n);
static int lv32_test = 0;               /* menu, not saved: the step */
static int lv32_patched = 0;            /* the step applied */
static const struct { uint32_t addr, canon, full; int step; } lv32_words[] = {
    { 0xE14DE230,    2,    2, 1 },      /* 4K geometry +0x1C: unchanged (remap only) */
    { 0xE14DE218, 2000, 2358, 2 },      /* 4K geometry: frame height */
    { 0xE14DE22C, 1958, 2316, 3 },      /* active area height (y 41) */
    { 0xE14DE23C, 1958, 2316, 3 },      /* its second copy */
    { 0xE14DDF7C, 1958, 2316, 4 },      /* dump size table (3488 x ..) */
    { 0xE14DDF8C, 1958, 2316, 4 },
    { 0xE14DE054,  976, 1154, 4 },      /* vector output (1740 x ..) */
    { 0xE14DE064,  976, 1154, 4 },
    { 0xE14DE444,    3,    4, 5 },      /* output: y offset in the active area */
    { 0xE14DE44C, 1952, 2308, 5 },      /* output height (3480 wide) */
    { 0xE14DE474, 1952, 2308, 5 },      /* its second copy */
};

/* RAM tables (Canon's RAM code copy, ROM 0xE157EB58 -> 0x02280000):
 * SapSetPath (LiveViewDrive::SapPath.c, RAM 0x022C5BA4) takes the mode's
 * {width, height} from a per-mode table (RAM 0x0233E564, 16 bytes each;
 * 4K = entry 1) and passes it to the path setters that write the
 * development sizes (D0413xxx, 3568 x 2000).  The geometry table alone
 * left these at 2000 rows: the LiveView start froze. */
static const struct { uint32_t addr, canon, full; int step; } lv32_ram[] = {
    { 0x0233E578, 2000, 2358, 2 },      /* SapPath 4K height */
    /* development records {8, 11, 2, w, h, 0x77772222, ..}, 0x54 bytes
     * per mode, three groups (ROM 0xE1645FCC..; users RAM 0x022D6270..
     * 0x022D8988): 4K's height.  RAW burst's 5376 x 3560 record has the
     * same parameters, and the burst runs D0413xxx at 5376 x 3560 - so
     * the development path isn't limited to 2000 / 2048 rows. */
    { 0x023474D8, 2000, 2358, 2 },
    { 0x02347A1C, 2000, 2358, 2 },
    { 0x02347E60, 2000, 2358, 2 },
};
static int lv32_ram_patched = 0;

static void lv32_ram_apply(int step)
{
    /* all checked first: none or all written */
    for (int i = 0; i < COUNT(lv32_ram); i++)
    {
        uint32_t now = MEM(lv32_ram[i].addr);
        if (now != lv32_ram[i].canon && now != lv32_ram[i].full)
        {
            tl("LiveView 3:2: RAM tables not as expected, left alone");
            return;
        }
    }
    for (int i = 0; i < COUNT(lv32_ram); i++)
    {
        MEM(lv32_ram[i].addr) = (step >= lv32_ram[i].step) ? lv32_ram[i].full : lv32_ram[i].canon;
    }
    lv32_ram_patched = step;
}

static void lv32_apply(int step)
{
    if (step == lv32_patched)
    {
        return;
    }
    if (lv32_ram_patched != step)
    {
        lv32_ram_apply(step);
    }
    if (lv32_patched)
    {
        for (int i = 0; i < COUNT(lv32_words); i++)
        {
            if (lv32_words[i].step <= lv32_patched)
            {
                unpatch_memory(lv32_words[i].addr);
            }
        }
        lv32_patched = 0;
        tl("LiveView 3:2: Canon's 4K geometry back");
    }
    if (!step)
    {
        return;
    }
    struct patch p[COUNT(lv32_words)];
    int n = 0;
    for (int i = 0; i < COUNT(lv32_words) && lv32_words[i].step <= step; i++, n++)
    {
        /* 4 bytes: values inline (patch_mmu.c uses new_value for size 4) */
        p[i] = (struct patch) {
            .addr = (uint8_t *) lv32_words[i].addr,
            .old_value = lv32_words[i].canon,
            .new_value = lv32_words[i].full,
            .size = 4,
            .description = "d8_modes: 4K LiveView geometry, 3:2",
            .is_instruction = 0,
        };
    }
    /* a remap page (ML core adds one from the heap if the static one is
     * taken, e.g. by dmlog's hooks), then all or none */
    mmu_reserve_remap_pages(1);
    int err = apply_patches(p, n);
    char msg[128];
    snprintf(msg, sizeof(msg), "LiveView 3:2: 4K geometry step %d (%d values, SapPath %d): %s %x%s", step, n,
             MEM(lv32_ram[0].addr), err ? "FAILED" : "ok", err,
             (err & E_PATCH_BAD_MMU_PAGE) ? " (ML's one MMU page in use: dmlog hooks?)" : "");
    tl(msg);
    if (err)
    {
        lv32_ram_apply(0);
        lv32_test = 0;
        return;
    }
    lv32_patched = step;
}

/* Applied at once, also while LiveView runs in slot 11: Canon sizes the
 * development path (D0413xxx) right after LiveView stops (engio log,
 * ~0.2 s into the menu), before d8_modes saw LiveView stopped for 300 ms
 * - it stayed at 2000 rows while the start used the patched 2358. */
static void lv32_step(void)
{
    int want = (ro_patched == RO_2X2_32 && ro_R == 2358) ? lv32_test : 0;
    if (want != lv32_patched)
    {
        lv32_apply(want);
    }
}

/* a read-out patched (or asked for) but not running: HEAD takes the new
 * height only at a real LiveView start.  Patched in a pause that wasn't
 * one (a raw take's start), the sensor kept Canon's rows; told once per
 * occurrence, as full 3:2's short frames are (check_alignment) */
static int ro_unapplied_seen = 0;

static void ro_check_applied(int ro)
{
    int slot = ro ? ro_target_slot(ro) : 0;
    if (!ro || !lv || LV_STOPPED || RECORDING || MEM(SEQ_MODE) != (uint32_t) slot)
    {
        ro_unapplied_seen = 0;
        return;
    }
    int head = (MEM(HEAD_SIZE_REG) >> 16) + 1;
    if (ro_patched == ro && head == ro_R)
    {
        ro_unapplied_seen = 0;
        return;
    }
    if (++ro_unapplied_seen == 2)
    {
        if (ro_patched == ro)
        {
            NotifyBox(5000, "Readout: sensor frames are %d rows, not %d.\nOpen and close Canon's menu to apply.", head, ro_R);
        }
        else
        {
            NotifyBox(5000, "Readout: sensor frames are %d rows (Canon's).\nOpen and close Canon's menu to apply.", head);
        }
    }
}

/* Canon's movie size / rate to be written from ML's Sensor mode, by the
 * task (vid_apply), which then patches the readout in the same pass:
 * before LiveView restarts into the new slot.  Set at start (ML's saved
 * mode wins: Canon's 4K comes back as 1080p after a restart) and by the
 * Sensor mode / rate menu; waits while it can't apply (photo mode). */
static int vid_boot_apply = 0;
static int d8_init_ms = 0;

static void d8_task(void)
{
    tl("task started");
    TASK_LOOP
    {
        if (!tl_tables && MEM(SEQ_STATE + 8) && MEM(SEQ_STATE + 0xC) && MEM(SDRV_STATE + 0x78))
        {
            tl_tables = 1;
            tl(m6ii_full32_patched() ? "Canon's tables present, patched by ML core at boot" : "Canon's tables present, not patched yet");
        }
        if (tl_first_vsync && !tl_vsync_printed)
        {
            tl_vsync_printed = 1;
            char line[64];
            snprintf(line, sizeof(line), "d8_modes %6d ms: first LiveView frame (vsync)\n", tl_first_vsync);
            printf("%s", line);
            tl_add(line);
        }
        if (test31_patched && !lv && (t31_wait_lv_stop || m1dump_frames))
        {
            /* every LiveView restart is measured once */
            t31_wait_lv_stop = 0;
            m1dump_frames = 0;
            tl("3x1 test: LiveView stopped, measuring after the restart");
        }
        if (test31 != test31_patched || (test31 && (t31_setp != t31_setp_patched || t31_lane != t31_lane_patched || t31_donor != t31_donor_patched)))
        {
            if (test31_patched || ro_patched)
            {
                ro_patched = 0;
                /* off, or switched: Canon's values back first */
                t_restore();
                test31_patched = 0;
                tl("3x1 test: restored");
            }
            if (test31)
            {
                if (full32_enabled || patched)
                {
                    full32_enabled = 0;
                    restore_all();
                }
                if (!test31_patch(test31))
                {
                    test31 = 0;
                }
            }
        }
        vid_read_canon();
        if (vid_boot_apply && !vid_cant_apply())
        {
            /* before the readout below: it patches for the size and rate
             * written here, and LiveView restarts into that slot */
            vid_boot_apply = 0;
            char msg[96];
            snprintf(msg, sizeof(msg), "Sensor mode: Canon's movie size %d rate %d -> ML's mode (slot %d)",
                     canon_movie_param[1], canon_movie_param[2], MEM(SEQ_MODE));
            tl(msg);
            vid_apply();
        }
        /* full 3:2 and the readout engine follow Sensor mode / Readout
         * (d8_readout_code); left alone while the 3x1 test runs and before
         * Canon's movie settings are known (-1: at boot the tables may be
         * patched already, for the first LiveView) */
        if (!test31)
        {
            int w = d8_readout_code();
            if (w >= 0)
            {
                int full = w == 1;
                int ro = w >= RO_3X3_239 ? w : 0;
                if (full != full32_enabled)
                {
                    full32_enabled = full;
                    tl(full ? "full 3:2 on (RAW video > Readout)" : "full 3:2 off (RAW video > Readout)");
                }
                /* a slot's tables (and its drive mode's timing column) change
                 * only while they aren't in use: LiveView running another
                 * slot, or stopped (Canon's menu open).  The per-frame
                 * sensor list acts on the next frame - setting 2.39:1 in
                 * the running slot hung the whole camera.  Patching a slot
                 * that isn't running lets LiveView start straight into it
                 * (at start: Canon's 1080p -> 4K restart). */
                int fps = -1, dmode = -1;
                if (ro)
                {
                    ro_timing(ro, &dmode, &fps);
                }
                int running = lv && !LV_STOPPED;
                int busy_old = ro_patched && running && MEM(SEQ_MODE) == (uint32_t) ro_slot;
                int busy_new = ro && running && MEM(SEQ_MODE) == (uint32_t) ro_target_slot(ro);
                /* before the first frame only another slot than LiveView's
                 * own (at start: slot 11 while 1080p starts; d8_modes has
                 * just set 4K and LiveView restarts into it - waiting for
                 * the frame lost that race half the time) */
                int own_slot = ro && MEM(SEQ_MODE) == (uint32_t) ro_target_slot(ro);
                static int own_wait_logged = 0;
                if (!LV_SEEN && own_slot && ro != ro_patched && !own_wait_logged)
                {
                    own_wait_logged = 1;
                    tl("readout: LiveView starts in the readout's own slot, waiting");
                }
                if ((LV_SEEN || !own_slot) && !busy_old && !busy_new &&
                    (ro != ro_patched || (ro && (ro_part != ro_part_patched ||
                                                 fps != ro_fps_patched || dmode != ro_mode_patched))))
                {
                    if (ro_patched)
                    {
                        t_restore();
                        ro_patched = 0;
                        tl("readout: Canon's values back");
                    }
                    /* full 3:2 patches slot 15 too: only 3x3 readouts
                     * wait for it (2x2's slot 11 and drive mode 17 are
                     * separate) */
                    int full32_in = ro_target_slot(ro) == SLOT_1080P && (full32_enabled || patched);
                    if (ro && fps >= 0 && !full32_in)
                    {
                        ro_patch(ro);
                    }
                }
            }
        }
        if (full32_enabled && !patched)
        {
            patch_tables();
            if (patched)
            {
                tl("tables patched");
            }
        }
        else if (!full32_enabled && patched &&
                 (LV_STOPPED || (lv && MEM(SEQ_MODE) != SLOT_1080P)))
        {
            /* back to Canon's shorter read-out: only while LiveView is
             * stopped, as for the readout engine above */
            restore_all();
            snprintf(status, sizeof(status), "Canon tables restored");
        }

        if (lv && MEM(SEQ_MODE) == SLOT_1080P && MEM(HEAD_SIZE_REG) == HEAD_SIZE_FULL && !realign_rows)
        {
            check_alignment();
        }
        if (lv && full32_enabled && !(MEM(SEQ_MODE) == SLOT_1080P && MEM(HEAD_SIZE_REG) == HEAD_SIZE_FULL && !realign_rows))
        {
            snprintf(align_seen, sizeof(align_seen), "not checked: slot %d head %x realign %d",
                     MEM(SEQ_MODE), MEM(HEAD_SIZE_REG), realign_rows);
        }
        /* Canon's movie size / rate now, after the readout step */
        vid_request();
        if (!test31)
        {
            int w = d8_readout_code();
            ro_check_applied(w >= RO_3X3_239 ? w : 0);
            lv32_step();
            lvc_step();
            defm_step();
        }
        ps_take_update();
        ps_cropmarks();
        full32_marker(full32_enabled);
        /* faster while LiveView is stopped: a quick Canon menu open /
         * close (300 ms without frames, then the next pass) was missed */
        msleep(LV_STOPPED ? 100 : 500);
    }
}

static MENU_UPDATE_FUNC(test31_update)
{
    if (!test31)
    {
        return;
    }
    if (!test31_patched)
    {
        MENU_SET_WARNING(MENU_WARN_NOT_WORKING, "Not patched (see the console / timeline): 1080p 23.976, full 3:2 off.");
    }
    else if (MEM(HEAD_SIZE_REG) != T31_HEAD_SIZE || t31_wait_lv_stop)
    {
        MENU_SET_WARNING(MENU_WARN_ADVICE, "Patched (%d values, c %d, ROD %d, SetP %d). Open and close Canon's menu to apply.",
            t_n, t31_c, T31_TG(TG_ROD), T31_TG(TG_SETP));
    }
    else
    {
        if (t31_ob_row)
        {
            MENU_SET_WARNING(MENU_WARN_INFO, "Running, ROD %d, SetP %d: next frame's OB at Mem1 row %d (frames ~%d lines of %d).",
                T31_TG(TG_ROD), T31_TG(TG_SETP), t31_ob_row, t31_ob_row - 8, t31_rows);
        }
        else
        {
            MENU_SET_WARNING(MENU_WARN_INFO, "Running: HEAD 2384x%d, ROD %d, SetP %d; no second frame in Mem1 (full length?) or no dump yet.",
                t31_rows, T31_TG(TG_ROD), T31_TG(TG_SETP));
        }
    }
}


/* ---- Sensor mode (Movie tab): Mode, Frame rate, Readout, 1:1 crop ----
 * 3x3 = Canon's 1080p, 2x2 = Canon's 4K, 1:1 = Canon's 1080p with
 * its 1:1 sensor mode at REC (core preshoot.c; 1080p leaves the most
 * memory).  Choosing a mode or rate writes Canon's movie size and rate
 * (PROP_MOVIE_PARAM) as Canon's menu does; a change in Canon's menu is
 * followed.  Each mode keeps its frame rate and Readout; 3x3 and 2x2 also
 * keep their own mlv_lite resolution, aspect ratio, format, compression
 * and lossy source (swapped by name: get / set_config_var, no link to
 * mlv_lite); 1:1's crop, depth, compression and rate go to the recorder
 * through core (preshoot_take_config). */
#define VID_3X3     0
#define VID_2X2     1
#define VID_1TO1    2
static CONFIG_INT("d8.mode", vid_mode, VID_3X3);
static CONFIG_INT("d8.rate.3x3", rate_3x3, 0);
static CONFIG_INT("d8.rate.2x2", rate_2x2, 0);
static int rate_view = 0;               /* the menu shows the mode's */

/* Canon's movie size ([1]) and NTSC rates ([2], 1/100 fps; [3] = rate / 2,
 * a GOP length?) in PROP_MOVIE_PARAM, as its menu writes them.  PAL rates
 * haven't been seen yet: then the rate stays Canon's. */
#define CANON_SIZE_FHD  0
#define CANON_SIZE_4K   5
static const int vid_rate_canon[] = { 2398, 2997, 5994 };
static const int vid_rate_gop[] = { 12, 15, 30 };
#define VID_RATES(mode) ((mode) == VID_3X3 ? 3 : 2)
static uint32_t canon_movie_param[10];
static int canon_movie_param_ok = 0;

/* Readout: the band the sensor reads, 0 2.39:1, 1 16:9, 2 3:2 (1:1: the
 * band cut from Canon's whole 1:1 frame at encoding) */
#define RO_239      0
#define RO_169      1
#define RO_32       2
/* 3x3 and 2x2 at 2.39:1 stop LiveView's start (no first VD, whole
 * camera hangs) with every part of the patch tried on its own: sensor
 * lists, EngIO heights, timing.  Until that is understood the sensor
 * keeps Canon's 16:9 read-out there; the setting itself stays. */
#define RO_239_READOUT      0
static CONFIG_INT("d8.ro.3x3", ro_3x3, RO_169);
static CONFIG_INT("d8.ro.2x2", ro_2x2, RO_169);
static CONFIG_INT("d8.ro.1to1", ps_band, RO_239);
static int ro_view = RO_169;

/* 1:1: Canon's frame is 5376 x 3560, optical black in columns 0..143 and
 * rows 0..71.  The band (Readout) is centred; RAW video's Resolution and
 * Aspect ratio cut the crop from it (mlv_lite) */
#define PS_W_MAX    5232
static const int ps_band_h[] = { 2192, 2944, 3488 };
static CONFIG_INT("d8.ps.24p", ps_24p, 1);

/* mlv_lite's settings that each mode (3x3, 2x2, 1:1) keeps: mlv_lite holds
 * the set of raw_set's mode, raw_store[] the others' */
static const char * const raw_keys[] = { "raw.res_x", "raw.res_x_fine", "raw.aspect.ratio", "raw.output_format", "raw.crx", "raw.lossy.source" };
static CONFIG_INT("d8.set", raw_set, VID_3X3);
static CONFIG_INT("d8.st0.0", st0_0, 4);
static CONFIG_INT("d8.st0.1", st0_1, 0);
static CONFIG_INT("d8.st0.2", st0_2, 10);
static CONFIG_INT("d8.st0.3", st0_3, 0);
static CONFIG_INT("d8.st0.4", st0_4, 0);
static CONFIG_INT("d8.st0.5", st0_5, 0);
static CONFIG_INT("d8.st1.0", st1_0, 4);
static CONFIG_INT("d8.st1.1", st1_1, 0);
static CONFIG_INT("d8.st1.2", st1_2, 10);
static CONFIG_INT("d8.st1.3", st1_3, 0);
static CONFIG_INT("d8.st1.4", st1_4, 0);
static CONFIG_INT("d8.st1.5", st1_5, 0);
/* 1:1: 5232 wide, 3:2 = the whole band, 12-bit, C-RAW */
static CONFIG_INT("d8.st2.0", st2_0, 11);
static CONFIG_INT("d8.st2.1", st2_1, 0);
static CONFIG_INT("d8.st2.2", st2_2, 12);
static CONFIG_INT("d8.st2.3", st2_3, 1);
static CONFIG_INT("d8.st2.4", st2_4, 2);
static CONFIG_INT("d8.st2.5", st2_5, 0);
static int * const raw_store[3][6] = {
    { &st0_0, &st0_1, &st0_2, &st0_3, &st0_4, &st0_5 },
    { &st1_0, &st1_1, &st1_2, &st1_3, &st1_4, &st1_5 },
    { &st2_0, &st2_1, &st2_2, &st2_3, &st2_4, &st2_5 },
};

/* earlier settings, read once to fill raw_store (raw_store_migrate): the
 * other of 3x3 / 2x2, and 1:1's own crop / depth / compression items */
static CONFIG_INT("d8.st.ver", raw_store_ver, 0);
static CONFIG_INT("d8.alt.0", alt_0, 4);
static CONFIG_INT("d8.alt.1", alt_1, 0);
static CONFIG_INT("d8.alt.2", alt_2, 10);
static CONFIG_INT("d8.alt.3", alt_3, 0);
static CONFIG_INT("d8.alt.4", alt_4, 0);
static CONFIG_INT("d8.alt.5", alt_5, 0);
static CONFIG_INT("d8.ps.res", ps_res, 11);
static CONFIG_INT("d8.ps.aspect", ps_aspect, 12);
static CONFIG_INT("d8.ps.depth", ps_depth, 1);
static CONFIG_INT("d8.ps.lossless", ps_lossless, 0);

static void raw_store_migrate(void)
{
    if (raw_store_ver)
    {
        return;
    }
    int * const alt[] = { &alt_0, &alt_1, &alt_2, &alt_3, &alt_4, &alt_5 };
    static const int alt_def[] = { 4, 0, 10, 0, 0, 0 };
    if (raw_set == VID_3X3 || raw_set == VID_2X2)
    {
        for (int i = 0; i < COUNT(alt); i++)
        {
            *raw_store[1 - raw_set][i] = *alt[i];
        }
    }
    for (int i = 0; i < COUNT(alt); i++)
    {
        *alt[i] = alt_def[i];
    }
    /* same preset lists (mlv_lite's widest is 5796, cut to the band) */
    *raw_store[VID_1TO1][0] = ps_res;
    *raw_store[VID_1TO1][2] = ps_aspect;
    *raw_store[VID_1TO1][3] = COERCE(ps_depth, 0, 2);
    *raw_store[VID_1TO1][4] = ps_lossless ? 1 : 2;
    ps_res = 11; ps_aspect = 12; ps_depth = 1; ps_lossless = 0;
    raw_store_ver = 1;
}

static void raw_use_set(int set)
{
    if (set == raw_set || set < 0 || set >= COUNT(raw_store))
    {
        return;
    }
    if (!module_get_config_var(raw_keys[0]))
    {
        /* mlv_lite isn't loaded: its 0s would replace the stored set */
        return;
    }
    for (int i = 0; i < COUNT(raw_keys); i++)
    {
        *raw_store[raw_set][i] = get_config_var(raw_keys[i]);
        set_config_var(raw_keys[i], *raw_store[set][i]);
    }
    raw_set = set;
}

static int *rate_current(void)
{
    return vid_mode == VID_2X2 ? &rate_2x2 : &rate_3x3;
}

static int *ro_current(void)
{
    return vid_mode == VID_1TO1 ? &ps_band : vid_mode == VID_2X2 ? &ro_2x2 : &ro_3x3;
}

static int vid_canon_rate_index(int rate)
{
    for (int i = 0; i < COUNT(vid_rate_canon); i++)
    {
        if (vid_rate_canon[i] == rate)
        {
            return i;
        }
    }
    return -1;
}

/* Canon's rate isn't one of the NTSC ones (PAL): no rates written */
#define VID_CANON_PAL (canon_movie_param_ok && vid_canon_rate_index(canon_movie_param[2]) < 0)

/* 1:1 band and rate -> core, for the next take (mlv_lite cuts the crop) */
static void ps_take_update(void)
{
    struct preshoot_take t;
    t.on = vid_mode == VID_1TO1;
    t.width = PS_W_MAX;
    t.height = ps_band_h[COERCE(ps_band, 0, 2)];
    t.x0 = 144;
    t.y0 = 72 + (3488 - t.height) / 2;
    t.fps_24p = ps_24p;
    preshoot_take_config(&t);
}

/* the crop mlv_lite reported, else the whole band */
static void ps_crop(int *w, int *h)
{
    if (!preshoot_take_get_crop(w, h))
    {
        *w = PS_W_MAX;
        *h = ps_band_h[COERCE(ps_band, 0, 2)];
    }
}

/* the menu: one mode's items shown */
static struct menu_entry *d8_children = NULL;   /* set at init */

static void d8_hide(void *priv, int hide)
{
    for (struct menu_entry *e = d8_children; e && !MENU_IS_EOL(e); e++)
    {
        if (e->priv == priv)
        {
            e->shidden = hide;
        }
    }
}

/* the Sensor mode item shows vid_mode + 1: a submenu whose parent reads 0
 * is greyed out as "disabled" (menu.c), and 3x3 is mode 0 */
static int vid_mode_view = VID_3X3 + 1;

static void vid_menu_refresh(void)
{
    vid_mode_view = vid_mode + 1;
}

/* Canon's menu changed the movie size or rate: follow it */
static void vid_follow_canon(void)
{
    int size = canon_movie_param[1];
    if (size == CANON_SIZE_4K && vid_mode != VID_2X2)
    {
        vid_mode = VID_2X2;
        raw_use_set(VID_2X2);
    }
    else if (size == CANON_SIZE_FHD && vid_mode == VID_2X2)
    {
        vid_mode = VID_3X3;
        raw_use_set(VID_3X3);
    }
    int r = vid_canon_rate_index(canon_movie_param[2]);
    if (vid_mode != VID_1TO1 && r >= 0 && r < VID_RATES(vid_mode))
    {
        *rate_current() = r;
    }
    vid_menu_refresh();
    ps_take_update();
}

PROP_HANDLER(PROP_VIDEO_MODE)
{
    if (len != sizeof(canon_movie_param))
    {
        return;
    }
    memcpy(canon_movie_param, buf, len);
    canon_movie_param_ok = 1;
    int size = canon_movie_param[1];
    int differs = (vid_mode == VID_2X2 && size == CANON_SIZE_FHD) || (vid_mode != VID_2X2 && size == CANON_SIZE_4K);
    if (differs && get_ms_clock() - d8_init_ms < 10000)
    {
        /* Canon's own reset to 1080p, after d8_modes loaded (one start
         * out of several): ML's saved mode wins, as in vid_read_canon */
        vid_boot_apply = 1;
        return;
    }
    vid_boot_apply = 0;     /* a change after start: Canon's menu wins */
    vid_follow_canon();
}

/* the handler above only sees changes (modules' handlers are added after
 * Canon delivered the current values to ML's): read it once at start */
extern int prop_get_value(unsigned property, void **addr, size_t *len);
static void vid_read_canon(void)
{
    void *data = NULL;
    size_t len = 0;
    if (canon_movie_param_ok)
    {
        return;
    }
    if (prop_get_value(PROP_VIDEO_MODE, &data, &len) == 0 && data && len == sizeof(canon_movie_param))
    {
        memcpy(canon_movie_param, data, len);
        canon_movie_param_ok = 1;
        int size = canon_movie_param[1];
        if ((vid_mode == VID_2X2 && size == CANON_SIZE_FHD) || (vid_mode != VID_2X2 && size == CANON_SIZE_4K))
        {
            /* ML's saved mode wins at start (d8_task writes it to Canon) */
            vid_boot_apply = 1;
            vid_menu_refresh();
            ps_take_update();
        }
        else
        {
            vid_follow_canon();
        }
    }
}

/* why the mode can't be applied now; NULL if it can */
static const char *vid_cant_apply(void)
{
    if (!canon_movie_param_ok)
    {
        return "Canon's movie settings not read yet.";
    }
    if (!is_movie_mode())
    {
        return "Movie mode only.";
    }
    if (RECORDING || lens_info.job_state)
    {
        return "Not while recording.";
    }
    if (vid_mode == VID_2X2 && VID_CANON_PAL && canon_movie_param[2] > 3000)
    {
        return "Canon's 4K has no 50 fps: choose 25 fps in Canon's menu first.";
    }
    return NULL;
}

/* write Canon's movie size and rate for vid_mode (if they differ) */
/* Canon's movie size / rate for vid_mode, written in two halves: the
 * values first (the readout step patches for them), the request after
 * it.  Canon starts restarting LiveView inside prop_request_change: with
 * the request first, the readout step already saw the new slot starting
 * (in use) and waited - Canon's 2000 rows after some starts. */
static uint32_t vid_pending[10];
static int vid_pending_n = 0;

static void vid_apply(void)
{
    if (vid_cant_apply())
    {
        return;
    }
    uint32_t p[COUNT(canon_movie_param)];
    memcpy(p, canon_movie_param, sizeof(p));
    p[1] = (vid_mode == VID_2X2) ? CANON_SIZE_4K : CANON_SIZE_FHD;
    /* 1:1: Canon's rate stays (any 4K or 1080p rate is a 1080p one; the
     * 1:1 mode's rate is its own); PAL: stays, not known yet */
    if (vid_mode != VID_1TO1 && !VID_CANON_PAL)
    {
        int r = COERCE(*rate_current(), 0, VID_RATES(vid_mode) - 1);
        p[2] = vid_rate_canon[r];
        p[3] = vid_rate_gop[r];
    }
    if (memcmp(p, canon_movie_param, sizeof(p)) == 0)
    {
        return;
    }
    printf("d8_modes: Canon's movie size %d -> %d, rate %d -> %d\n", canon_movie_param[1], p[1], canon_movie_param[2], p[2]);
    /* what will be asked for, until the handler confirms it: the readout
     * step patches for this size and rate */
    memcpy(canon_movie_param, p, sizeof(p));
    memcpy(vid_pending, p, sizeof(p));
    vid_pending_n = sizeof(p);
}

/* the request half of vid_apply, after the readout step */
static void vid_request(void)
{
    if (vid_pending_n)
    {
        prop_request_change(PROP_VIDEO_MODE, vid_pending, vid_pending_n);
        vid_pending_n = 0;
    }
}

/* for the task: -1 = Canon's movie settings not known yet, 0 = Canon's
 * read-out, 1 = 3x3 at 3:2 (full 3:2, 23.976 only), 2 = 3x3 at 2.39:1,
 * 3 = 2x2 at 2.39:1, 4 = 2x2 at 3:2 */
static int d8_readout_code(void)
{
    vid_read_canon();
    if (!canon_movie_param_ok)
    {
        return -1;
    }
    if (vid_mode == VID_3X3)
    {
        return ro_3x3 == RO_32 ? (canon_movie_param[2] == 2398) : (RO_239_READOUT && ro_3x3 == RO_239) ? RO_3X3_239 : 0;
    }
    if (vid_mode == VID_2X2)
    {
        return (RO_239_READOUT && ro_2x2 == RO_239) ? RO_2X2_239 : ro_2x2 == RO_32 ? RO_2X2_32 : 0;
    }
    return 0;
}

/* ML core's boot patch (full32_core.c) applies full 3:2 before the first
 * LiveView if this file exists */
static void full32_marker(int on)
{
    static int last = -1;
    if (on == last)
    {
        return;
    }
    last = on;
    if (on)
    {
        FILE *f = FIO_CreateFile("ML/SETTINGS/d8_32.on");
        if (f)
        {
            FIO_CloseFile(f);
        }
    }
    else
    {
        FIO_RemoveFile("ML/SETTINGS/d8_32.on");
    }
}

/* 1:1 takes: the crop in Canon's idle 3x3 LiveView (the raw frame's
 * active width is the sensor's 6960 px, 3x3 binned; assumed centred) */
static void ps_cropmarks(void)
{
    static int drawn = 0;
    int show = vid_mode == VID_1TO1 && lv && is_movie_mode() && !RECORDING &&
               MEM(SEQ_MODE) == SLOT_1080P && get_config_var("raw.video.enabled");
    int aw = raw_info.active_area.x2 - raw_info.active_area.x1;
    int ah = raw_info.active_area.y2 - raw_info.active_area.y1;
    if (show && aw > 0 && ah > 0)
    {
        int cw, ch;
        ps_crop(&cw, &ch);
        int w = cw * aw / 6960;
        int h = MIN(ch * aw / 6960, ah);
        int x = raw_info.active_area.x1 + (aw - w) / 2;
        int y = raw_info.active_area.y1 + (ah - h) / 2;
        set_movie_cropmarks(RAW2BM_X(x), RAW2BM_Y(y), RAW2BM_DX(w), RAW2BM_DY(h));
        drawn = 1;
    }
    else if (drawn)
    {
        reset_movie_cropmarks();
        drawn = 0;
    }

    /* while a 1:1 take records, LiveView shows Canon's 1:1 frame (5232 x
     * 3488 active) on the whole screen: the crop as plain frame lines
     * (full width: the usual 16:9 / 2.39:1 ones).  Drawn straight on the
     * overlay and refreshed by d8_task, as ML's overlays don't redraw
     * during these takes; erased when the take ends */
    static int rec_drawn = 0, rx, ry, rw, rh;
    int rec = vid_mode == VID_1TO1 && lv && is_movie_mode() && RECORDING;
    if (rec)
    {
        int cw, ch;
        ps_crop(&cw, &ch);
        int w = cw * os.x_ex / PS_W_MAX;
        int h = MIN(ch * os.y_ex / 3488, os.y_ex);
        int x = os.x0 + (os.x_ex - w) / 2;
        int y = os.y0 + (os.y_ex - h) / 2;
        if (rec_drawn && (x != rx || y != ry || w != rw || h != rh))
        {
            bmp_draw_rect(COLOR_EMPTY, rx, ry, rw, rh);
        }
        if (w < os.x_ex || h < os.y_ex)
        {
            bmp_draw_rect(COLOR_WHITE, x, y, w, h);
            rec_drawn = 1;
            rx = x; ry = y; rw = w; rh = h;
        }
    }
    else if (rec_drawn)
    {
        bmp_draw_rect(COLOR_EMPTY, rx, ry, rw, rh);
        rec_drawn = 0;
    }
}

static MENU_SELECT_FUNC(vid_mode_select)
{
    if (RECORDING || lens_info.job_state)
    {
        return;
    }
    vid_mode = MOD(vid_mode + delta, 3);
    raw_use_set(vid_mode);
    vid_menu_refresh();
    ps_take_update();
    vid_boot_apply = 1;     /* d8_task: Canon's size, then the readout */
}

static MENU_UPDATE_FUNC(vid_mode_update)
{
    vid_menu_refresh();
    const char *why = vid_cant_apply();
    int canon_size = canon_movie_param[1];
    int canon_ok = canon_movie_param_ok &&
        canon_size == ((vid_mode == VID_2X2) ? CANON_SIZE_4K : CANON_SIZE_FHD);
    if (vid_mode == VID_1TO1)
    {
        int w, h;
        ps_crop(&w, &h);
        MENU_SET_RINFO("%dx%d", w, h);
    }
    if (canon_ok)
    {
        MENU_SET_WARNING(MENU_WARN_INFO, vid_mode == VID_1TO1
            ? "Canon's 1080p in LiveView (cropmarks: the 1:1 crop); REC switches to 1:1."
            : "Canon's movie size matches. Changing it in Canon's menu changes this mode.");
    }
    else if (why)
    {
        MENU_SET_WARNING(MENU_WARN_NOT_WORKING, "%s", why);
    }
    else
    {
        MENU_SET_WARNING(MENU_WARN_ADVICE, "Canon's movie size doesn't match yet (size %d).", canon_size);
    }
}

static MENU_SELECT_FUNC(vid_rate_select)
{
    if (vid_mode == VID_1TO1)
    {
        /* Canon's 1:1 mode: 23.976 (timing table patched) or its 29.97,
         * applied at each take's start; Canon's 1080p rate stays */
        if (!RECORDING)
        {
            ps_24p = !ps_24p;
            rate_view = ps_24p ? 0 : 1;
            ps_take_update();
        }
        return;
    }
    if (VID_CANON_PAL || RECORDING || lens_info.job_state)
    {
        return;
    }
    int *r = rate_current();
    *r = MOD(*r + delta, VID_RATES(vid_mode));
    rate_view = *r;
    vid_boot_apply = 1;     /* d8_task: Canon's rate, then the readout */
}

static MENU_UPDATE_FUNC(vid_rate_update)
{
    if (vid_mode == VID_1TO1)
    {
        rate_view = ps_24p ? 0 : 1;
        MENU_SET_WARNING(MENU_WARN_INFO, ps_24p
            ? "1:1: Canon's 1:1 mode at 23.976 (its timing table patched in RAM), from the take's start."
            : "1:1: Canon's own rate for its 1:1 mode.");
        return;
    }
    rate_view = *rate_current();
    if (VID_CANON_PAL)
    {
        MENU_SET_VALUE("%d.%02d (Canon's)", canon_movie_param[2] / 100, canon_movie_param[2] % 100);
        MENU_SET_WARNING(MENU_WARN_INFO, "PAL rates: set in Canon's menu (only the NTSC ones are written).");
    }
}

static MENU_SELECT_FUNC(ro_select)
{
    if (RECORDING || lens_info.job_state)
    {
        return;
    }
    int *ro = ro_current();
    *ro = MOD(*ro + delta, 3);
    ro_view = *ro;
    ps_take_update();
}

static MENU_UPDATE_FUNC(ro_update)
{
    int ro = ro_view = *ro_current();
    if (vid_mode == VID_1TO1)
    {
        MENU_SET_RINFO("%dx%d", PS_W_MAX, ps_band_h[COERCE(ps_band, 0, 2)]);
        MENU_SET_WARNING(MENU_WARN_INFO, "1:1: the sensor reads the whole 5376x3560 frame; the band is cut at encoding.");
        return;
    }
    /* lines the sensor delivers now (HEAD), vs Canon's 1348 / 2000 */
    int canon_lines = vid_mode == VID_2X2 ? 2000 : 1348;
    int head_lines = (MEM(HEAD_SIZE_REG) >> 16) + 1;
    int slot_ok = MEM(SEQ_MODE) == (uint32_t) (vid_mode == VID_2X2 ? 11 : SLOT_1080P);
    if (ro == RO_239 && !RO_239_READOUT)
    {
        MENU_SET_WARNING(MENU_WARN_INFO, "2.39:1 is cut from Canon's 16:9 read-out for now (a shorter one hangs).");
    }
    else if (ro == RO_169)
    {
        MENU_SET_WARNING(MENU_WARN_INFO, vid_mode == VID_2X2 ? "Canon's 4K read-out (2x2)." : "Canon's 1080p read-out (3x3).");
    }
    else if (vid_mode == VID_3X3 && ro == RO_32 && (!canon_movie_param_ok || canon_movie_param[2] != 2398))
    {
        MENU_SET_WARNING(MENU_WARN_NOT_WORKING, "3x3 at 3:2 (full sensor height): 23.976 only so far.");
    }
    else if (lv && slot_ok && (ro == RO_239 ? head_lines < canon_lines : head_lines > canon_lines))
    {
        MENU_SET_WARNING(MENU_WARN_INFO, "The sensor reads %d lines (Canon's %d).", head_lines, canon_lines);
    }
    else
    {
        MENU_SET_WARNING(MENU_WARN_ADVICE, "Open and close Canon's menu to apply (LiveView restart).");
    }
}

/* ---- Mem1 layout test: Mem1 packs in ML's 16-bit word layout (pack
 * fields f1 = 1, f3 = 0: Canon's own setting) or LSB-first (f1 = 0, f3 =
 * 1: what Canon's CR3 encoder reads).  Both held at 14, 12 and 10 bit on
 * the M6II, so mlv_lite records uncompressed takes in ML's layout.  One
 * Mem1 frame per depth and layout to ML/LOGS/ML14.RAW .. LS10.RAW (a
 * 256-byte text header, then ch 3's rows): decoding each both ways, the
 * right layout gives a smooth image. */
static char lt_status[64] = "";

static void layout_test_task(void)
{
    static const int depths[] = { 14, 12, 10 };
    if (!lv || !is_movie_mode())
    {
        snprintf(lt_status, sizeof(lt_status), "needs movie LiveView");
        return;
    }
    raw_lv_request();
    int n = 0;
    for (int l = 0; l < 2; l++)
    {
        for (int d = 0; d < COUNT(depths); d++)
        {
            int layout = l ? RAW_LAYOUT_LSB : RAW_LAYOUT_ML;
            raw_lv_request_layout(layout);
            raw_lv_request_bpp(depths[d]);
            for (int i = 0; i < 15; i++)
            {
                msleep(100);
                raw_update_params();
            }
            uint32_t base = MEM(MEM1_CH3 + 0x28 * 4);
            uint32_t xy = MEM(MEM1_CH3 + 0x14 * 4);
            uint32_t xb = xy & 0xFFFF, yb = (xy >> 16) + 1;
            uint32_t len = MIN(xb * yb, 16u << 20);
            char hdr[256];
            memset(hdr, ' ', sizeof(hdr));
            int k = snprintf(hdr, sizeof(hdr), "M1LAYOUT asked %s %d, pack %x xb %d yb %d raw_info %d-bit layout %d len %d",
                             l ? "LSB" : "ML", depths[d], (int) MEM(MEM1_CH3_PACK), (int) xb, (int) yb,
                             raw_info.bits_per_pixel, raw_get_layout(), (int) len);
            hdr[k] = ' ';
            hdr[sizeof(hdr) - 1] = '\n';
            char name[32];
            snprintf(name, sizeof(name), "ML/LOGS/%s%02d.RAW", l ? "LS" : "ML", depths[d]);
            FILE *f = FIO_CreateFile(name);
            if (f)
            {
                FIO_WriteFile(f, hdr, sizeof(hdr));
                if (base && len)
                {
                    FIO_WriteFile(f, UNCACHEABLE(base), len);
                }
                FIO_CloseFile(f);
                n++;
            }
            snprintf(lt_status, sizeof(lt_status), "%d of 6 written (%s %d: pack %x)", n, l ? "LSB" : "ML", depths[d], (int) MEM(MEM1_CH3_PACK));
        }
    }
    raw_lv_request_layout(RAW_LAYOUT_LSB);
    raw_lv_request_bpp(14);
    raw_lv_release();
    snprintf(lt_status, sizeof(lt_status), "done: %d of 6 files in ML/LOGS (ML*, LS*.RAW)", n);
}

static MENU_UPDATE_FUNC(layout_test_update)
{
    if (lt_status[0])
    {
        MENU_SET_WARNING(MENU_WARN_INFO, "%s", lt_status);
    }
}

static struct menu_entry d8_menu[] =
{
    {
        .name       = "Sensor mode",
        .priv       = &vid_mode_view,   /* vid_mode + 1, see vid_menu_refresh */
        .min        = 1,
        .max        = 3,
        .choices    = CHOICES("3x3 (1080p)", "2x2 (4K)", "1:1 crop"),     /* choices[value - min] */
        .icon_type  = IT_DICE,
        .select     = vid_mode_select,
        .update     = vid_mode_update,
        .submenu_width = 710,
        .depends_on = DEP_MOVIE_MODE,
        .help       = "3x3 = Canon's 1080p, 2x2 = Canon's 4K, 1:1 = a crop of Canon's 1:1 mode (REC).",
        .help2      = "Sets Canon's movie size; each mode keeps its settings. Follows Canon's menu.",
        .children   = (struct menu_entry[]) {
            {
                .name       = "Frame rate",
                .priv       = &rate_view,
                .max        = 2,
                .choices    = CHOICES("23.976", "29.97", "59.94"),
                .select     = vid_rate_select,
                .update     = vid_rate_update,
                .help       = "Canon's frame rate for this mode (59.94: 3x3 only); written to Canon's settings.",
                .help2      = "1:1: Canon's 1:1 mode at 23.976 or its 29.97 (Canon's 1080p rate stays).",
            },
            {
                .name       = "Readout",
                .priv       = &ro_view,
                .max        = 2,
                .choices    = CHOICES("2.39:1", "16:9", "3:2"),
                .select     = ro_select,
                .update     = ro_update,
                .help       = "The band the sensor reads. 2.39:1 in 3x3 / 2x2: Canon's 16:9 read-out for now.",
                .help2      = "RAW video's resolution, aspect ratio, depth and compression: each mode keeps its own.",
            },
            {
                .name    = "4K defect map (test)",
                .priv    = &defm_test,
                .max     = 1,
                .choices = CHOICES("Canon's", "3:2"),
                .update  = defm_update,
                .help    = "TEST, 2x2 3:2: 4K defect map extended to the 3:2 frame (Canon's ends at row 2004).",
                .help2   = "Not saved. Applies at the next LiveView start.",
            },
            {
                .name    = "LiveView centre (test)",
                .priv    = &lvc_test,
                .max     = 1,
                .help    = "TEST, 2x2 3:2: LiveView development window rows 178-2178 (centred band).",
                .help2   = "Not saved. Applies at the next LiveView start (Canon menu). May freeze LiveView.",
            },
            {
                .name    = "LiveView 3:2 (test)",
                .priv    = &lv32_test,
                .max     = 5,
                .choices = CHOICES("OFF", "Remap only", "Frame", "+ active", "+ buffers", "All"),
                .help    = "TEST, 2x2 3:2: 4K LiveView heights raised in steps (remap, frame, active, buffers, output).",
                .help2   = "Not saved. Applies at the next LiveView start (Canon menu); may freeze LiveView.",
            },
            {
                .name    = "3x1 test",
                .priv    = &test31,
                .max     = 2,
                .choices = CHOICES("OFF", "2.39:1", "16:9"),
                .update  = test31_update,
                .help    = "TEST, 1080p 23.976: every sensor line, 3x binned across (2384-px lines, full width).",
                .help2   = "Applies at the next LiveView start (Canon menu). 16:9 leaves little blanking.",
            },
            {
                .name    = "3x1 donor",
                .priv    = &t31_donor,
                .max     = 2,
                .choices = CHOICES("magnify", "magnify, 002 binned", "RAW burst"),
                .help    = "3x1 test: vertical sampling registers from slot 7 (5x magnify) or slot 4 (RAW burst).",
                .help2   = "'002 binned': keeps 1080p's 0x002 (1414). Changing it re-patches; Canon menu to apply.",
            },
            {
                .name    = "3x1 lane order",
                .priv    = &t31_lane,
                .max     = 1,
                .choices = CHOICES("1080p", "magnify"),
                .help    = "3x1 test: input stage lane order D0412244: 1080p's or the 5x magnify slot's.",
                .help2   = "Changing it re-patches; open and close Canon's menu to apply.",
            },
            {
                .name    = "3x1 SetP",
                .priv    = &t31_setp,
                .max     = 3,
                .choices = CHOICES("ROD - 204", "ROD - 104", "ROD - 4", "ROD + 96"),
                .help    = "3x1 test: when Canon sends the next frame's sensor settings (line periods).",
                .help2   = "Changing it re-patches; open and close Canon's menu to apply.",
            },
            {
                .name       = "Mem1 layout test",
                .select     = run_in_separate_task,
                .priv       = layout_test_task,
                .icon_type  = IT_ACTION,
                .update     = layout_test_update,
                .help       = "TEST, movie LiveView: one Mem1 frame at 14 / 12 / 10 bit in ML's layout and LSB-first.",
                .help2      = "Files ML/LOGS/ML14.RAW .. LS10.RAW (~30 MB); back to LSB-first 14 bit afterwards.",
            },
            MENU_EOL,
        },
    },
};

static unsigned int d8_init(void)
{
    if (!is_camera("M6II", "1.1.1"))
    {
        return CBR_RET_ERROR;
    }
    d8_init_ms = get_ms_clock();
    char line[96];
    snprintf(line, sizeof(line), "d8_modes %6d ms: module init (lv %d, tables %x %x %x)\n", get_ms_clock(), lv,
             MEM(SEQ_STATE + 8), MEM(SEQ_STATE + 0xC), MEM(SDRV_STATE + 0x78));
    printf("%s", line);
    tl_add(line);
    /* ML core patched at boot, before LiveView started: keep it if the
     * settings still say full 3:2, else undo it.  Without the boot patch
     * (no marker, or Canon at another rate) d8_task patches once Canon's
     * movie settings are known */
    full32_enabled = vid_mode == VID_3X3 && ro_3x3 == RO_32 && m6ii_full32_patched();
    if (full32_enabled)
    {
        m6ii_full32_claim();
        /* the boot patch leaves the image processor lists for now (Canon's
         * AF setup must not see them); this adds them */
        patch_tables();
    }
    else
    {
        m6ii_full32_restore();
    }
    d8_children = d8_menu[0].children;
    raw_store_migrate();
    /* earlier versions kept 1:1's settings apart: mlv_lite may still hold
     * another mode's set */
    raw_use_set(vid_mode);
    vid_read_canon();
    vid_menu_refresh();
    ps_take_update();
    menu_add("Movie", d8_menu, COUNT(d8_menu));
    task_create("d8_task", 0x1e, 0x1000, d8_task, (void *) 0);
    task_create("d8_m1dump", 0x1c, 0x1000, m1dump_task, (void *) 0);
    return 0;
}

static unsigned int d8_deinit(void)
{
    return 0;
}

MODULE_INFO_START()
    MODULE_INIT(d8_init)
    MODULE_DEINIT(d8_deinit)
MODULE_INFO_END()

MODULE_CBRS_START()
    MODULE_CBR(CBR_VSYNC, d8_vsync, 0)
MODULE_CBRS_END()

MODULE_CONFIGS_START()
    MODULE_CONFIG(vid_mode)
    MODULE_CONFIG(rate_3x3)
    MODULE_CONFIG(rate_2x2)
    MODULE_CONFIG(ro_3x3)
    MODULE_CONFIG(ro_2x2)
    MODULE_CONFIG(ps_band)
    MODULE_CONFIG(ps_24p)
    MODULE_CONFIG(raw_set)
    MODULE_CONFIG(st0_0)
    MODULE_CONFIG(st0_1)
    MODULE_CONFIG(st0_2)
    MODULE_CONFIG(st0_3)
    MODULE_CONFIG(st0_4)
    MODULE_CONFIG(st0_5)
    MODULE_CONFIG(st1_0)
    MODULE_CONFIG(st1_1)
    MODULE_CONFIG(st1_2)
    MODULE_CONFIG(st1_3)
    MODULE_CONFIG(st1_4)
    MODULE_CONFIG(st1_5)
    MODULE_CONFIG(st2_0)
    MODULE_CONFIG(st2_1)
    MODULE_CONFIG(st2_2)
    MODULE_CONFIG(st2_3)
    MODULE_CONFIG(st2_4)
    MODULE_CONFIG(st2_5)
    MODULE_CONFIG(raw_store_ver)
    MODULE_CONFIG(ps_res)
    MODULE_CONFIG(ps_aspect)
    MODULE_CONFIG(ps_depth)
    MODULE_CONFIG(ps_lossless)
    MODULE_CONFIG(alt_0)
    MODULE_CONFIG(alt_1)
    MODULE_CONFIG(alt_2)
    MODULE_CONFIG(alt_3)
    MODULE_CONFIG(alt_4)
    MODULE_CONFIG(alt_5)
MODULE_CONFIGS_END()

MODULE_PROPHANDLERS_START()
    MODULE_PROPHANDLER(PROP_VIDEO_MODE)
MODULE_PROPHANDLERS_END()
