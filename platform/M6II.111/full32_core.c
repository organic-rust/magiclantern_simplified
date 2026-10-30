/* Full 3:2 table patch for modules/d8_modes, in ML core so it can run
 * before Canon's first LiveView start after a boot.
 *
 * Canon sends the sensor its register list (CMOS tables) and programs the
 * timing generator (ROD) only when LiveView starts.  The module loads just
 * after that start (first frame ~0.9-2 s after boot, the module 10-60 ms
 * later), so the first LiveView after a boot had 1080p's 1348-row frames
 * and needed a Canon menu round trip.  Setting only ROD early was not
 * enough: the CMOS list has to be in place too.  The tables are filled
 * by ~211 ms (boot probe), so a task started right after Canon's
 * init_task applies the whole set then - the same consistent set the
 * module applied (a partial set of sensor registers has hung the camera).
 *
 * ML's settings aren't loaded that early, but the card is readable: the
 * module is loaded if ML/SETTINGS/d8_modes.en exists, and full 3:2 is on
 * if d8_modes keeps ML/SETTINGS/d8_32.on (Sensor mode 3x3, Readout 3:2).
 * Patching regardless made the first LiveView with full 3:2 off start in
 * the full 3:2 geometry and visibly shift when the module restored
 * Canon's tables.  If the card can't be
 * read before LiveView (AUTOEXEC.BIN not found), it's patched anyway.
 * d8_modes claims the patch, or restores it; unclaimed, it's restored 5 s
 * after ML has started.  See modules/d8_modes/d8_modes.c for the tables. */

#include <dryos.h>
extern int _FIO_GetFileSize(const char *filename, uint32_t *size);
#include "timer.h"
#include "ptp.h"

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


static struct { uint32_t addr, old, val; } saved[64];
static int n_saved = 0;
static int patched = 0;
static int engio_later = 0;     /* boot patch: EngIO lists later (patch_engio) */
static int engio_done = 0;
static char status[48] = "";


static const struct { uint32_t off, stride; } cmos_side[] = {
    { 0x684C, 0x24 }, { 0x53A0, 0xFC }, { 0x70B0, 0x1C },
};
static const struct { uint32_t off, stride; } eng_lists[] = {
    { 0xF40, 0x78 }, { 0x3340, 0x68 }, { 0x1A38, 0xB0 },
};

/* dry run: count what patch_tables would change, write nothing */
static int dry_run = 0;
static int dry_count = 0;

static int save_and_write(uint32_t addr, uint32_t value)
{
    if (dry_run)
    {
        dry_count++;
        return 1;
    }
    if (n_saved >= COUNT(saved))
    {
        return 0;
    }
    saved[n_saved].addr = addr;
    saved[n_saved].old = MEM(addr);
    saved[n_saved].val = value;
    n_saved++;
    MEM(addr) = value;
    return 1;
}

/* Canon's values back - except where Canon has written since (one word
 * gets rewritten at ~1.1 s on boot): its newer value stands */
static void restore_all(void)
{
    engio_done = 0;
    while (n_saved > 0)
    {
        n_saved--;
        if (MEM(saved[n_saved].addr) == saved[n_saved].val)
        {
            MEM(saved[n_saved].addr) = saved[n_saved].old;
        }
    }
    patched = 0;
}

/* first value of `reg` in an (addr, value) list ending at addr -1, or 0 */
static uint32_t eng_value(uint32_t list, uint32_t reg)
{
    for (int i = 0; i < 64 && MEM(list + 8 * i) != 0xFFFFFFFF; i++)
    {
        if (MEM(list + 8 * i) == reg)
        {
            return MEM(list + 8 * i + 4);
        }
    }
    return 0;
}

static uint32_t tg_u16(uint32_t tg, uint32_t field)
{
    uint32_t a = tg + field + TG_MODE_1080P * TG_STRIDE + TG_FPS_23976 * 2;
    return *(volatile uint16_t *) a;
}

/* 1 if the tables hold exactly what we measured on M6II 1.1.1 */
static int tables_as_expected(uint32_t cmos, uint32_t eng, uint32_t tg)
{
    if (!cmos || !eng || !tg)
    {
        snprintf(status, sizeof(status), "Canon tables not loaded yet");
        return 0;
    }
    if (eng_value(eng + 0xF40 + SLOT_1080P * 0x78, HEAD_SIZE_REG) != HEAD_SIZE_1080P ||
        eng_value(eng + 0xF40 + SLOT_FULL * 0x78, HEAD_SIZE_REG) != HEAD_SIZE_FULL)
    {
        snprintf(status, sizeof(status), "unexpected frame sizes, not patched");
        return 0;
    }
    if (tg_u16(tg, TG_VD) != VD_1080P_23976 || tg_u16(tg, TG_ROD) != ROD_1080P)
    {
        snprintf(status, sizeof(status), "unexpected timing table, not patched");
        return 0;
    }
    return 1;
}

/* Image processor (EngIO) lists: same registers, slot 16's values - all
 * heights.  Movie Servo AF and AF-ON break when LiveView starts with all
 * of them patched (after a boot, or a Canon menu round trip); with
 * Canon's values at LiveView start AF works, even with the full-height
 * sensor and the values patched later.  Canon sizes the Dual Pixel AF
 * data at LiveView start ("DAFRAW" frame buffer warnings).  Suspects:
 * the three half-height registers (0x29B -> 0x313 rows), so they keep
 * Canon's values; the rest (HEAD 0x543 -> 0x637 etc.) go in at boot. */
static int engio_reg_skipped(uint32_t reg)
{
    return reg == 0xD041184C || reg == 0xD0411898 || reg == 0xD041189C;
}


static int patch_engio(uint32_t eng)
{
    int ok = 1;
    for (int k = 0; k < COUNT(eng_lists); k++)
    {
        uint32_t dst = eng + eng_lists[k].off + SLOT_1080P * eng_lists[k].stride;
        uint32_t src = eng + eng_lists[k].off + SLOT_FULL * eng_lists[k].stride;
        for (int i = 0; i < 64 && MEM(dst + 8 * i) != 0xFFFFFFFF; i++)
        {
            if (MEM(dst + 8 * i) == MEM(src + 8 * i) && MEM(dst + 8 * i + 4) != MEM(src + 8 * i + 4))
            {
                /* skipped ones still count in the dry run (24-value gate) */
                if (engio_reg_skipped(MEM(dst + 8 * i)) && !dry_run)
                {
                    continue;
                }
                ok &= save_and_write(dst + 8 * i + 4, MEM(src + 8 * i + 4));
            }
        }
    }
    if (!dry_run)
    {
        engio_done = ok;
    }
    return ok;
}

static void patch_tables(void)
{
    uint32_t cmos = MEM(SEQ_STATE + 8);
    uint32_t eng  = MEM(SEQ_STATE + 0xC);
    uint32_t tg   = MEM(SDRV_STATE + 0x78);

    if (!tables_as_expected(cmos, eng, tg))
    {
        return;
    }

    int ok = 1;

    /* CMOS main list: every word that differs */
    uint32_t dst = cmos + CMOS_MAIN + SLOT_1080P * CMOS_MAIN_STRIDE;
    uint32_t src = cmos + CMOS_MAIN + SLOT_FULL * CMOS_MAIN_STRIDE;
    for (int i = 0; i < CMOS_MAIN_WORDS; i++)
    {
        if (MEM(dst + 4 * i) != MEM(src + 4 * i))
        {
            ok &= save_and_write(dst + 4 * i, MEM(src + 4 * i));
        }
    }

    /* CMOS side lists: frames up to the -1, -1 end mark (the slot ids
     * after it stay) */
    for (int k = 0; k < COUNT(cmos_side); k++)
    {
        dst = cmos + cmos_side[k].off + SLOT_1080P * cmos_side[k].stride;
        src = cmos + cmos_side[k].off + SLOT_FULL * cmos_side[k].stride;
        for (uint32_t i = 0; i + 1 < cmos_side[k].stride / 4; i += 2)
        {
            if (MEM(dst + 4 * i) == 0xFFFFFFFF && MEM(dst + 4 * i + 4) == 0xFFFFFFFF)
            {
                break;
            }
            for (int j = 0; j < 2; j++)
            {
                if (MEM(dst + 4 * (i + j)) != MEM(src + 4 * (i + j)))
                {
                    ok &= save_and_write(dst + 4 * (i + j), MEM(src + 4 * (i + j)));
                }
            }
        }
    }

    /* image processor heights: at boot only counted, applied later */
    if (dry_run || !engio_later)
    {
        ok &= patch_engio(eng);
    }

    /* read-out duration, a u16 in a word */
    uint32_t a = tg + TG_ROD + TG_MODE_1080P * TG_STRIDE + TG_FPS_23976 * 2;
    uint32_t shift = (a & 2) * 8;
    a &= ~3;
    ok &= save_and_write(a, (MEM(a) & ~(0xFFFF << shift)) | (ROD_FULL << shift));

    if (dry_run)
    {
        return;
    }
    if (!ok)
    {
        restore_all();
        snprintf(status, sizeof(status), "too many values, not patched");
        return;
    }
    patched = 1;
    snprintf(status, sizeof(status), "%d values patched", n_saved);
}

int m6ii_full32_patched(void)
{
    return patched;
}

/* patch if not yet; 1 if patched; status for the menu */
int m6ii_full32_patch(char *out, int len)
{
    if (!patched)
    {
        engio_later = 0;
        patch_tables();
    }
    else if (!engio_done)
    {
        /* patched at boot: the image processor lists now */
        int before = n_saved;
        if (patch_engio(MEM(SEQ_STATE + 0xC)))
        {
            snprintf(status, sizeof(status), "%d values patched (%d at boot)", n_saved, before);
        }
        else
        {
            restore_all();
            snprintf(status, sizeof(status), "too many values, not patched");
        }
    }
    snprintf(out, len, "%s", status);
    return patched;
}

void m6ii_full32_restore(void)
{
    restore_all();
    snprintf(status, sizeof(status), "Canon tables restored");
}

static volatile int claimed = 0;

/* the module is enabled and keeps the early patch */
void m6ii_full32_claim(void)
{
    claimed = 1;
}

static char boot_log[320];
static volatile uint32_t boot_log_len = 0;

static void boot_log_line(const char *what)
{
    int n = snprintf(boot_log + boot_log_len, sizeof(boot_log) - boot_log_len, "%6d ms: %s\n", get_ms_clock(), what);
    if (n > 0 && boot_log_len + n < sizeof(boot_log))
    {
        boot_log_len += n;
    }
}

static int file_exists(const char *path)
{
    uint32_t size = 0;
    return _FIO_GetFileSize(path, &size) == 0;
}

/* 1 = full 3:2 enabled, 0 = not, -1 = card not readable yet */
static int full32_enabled_on_card(void)
{
    if (!file_exists("B:/AUTOEXEC.BIN"))
    {
        return -1;
    }
    return file_exists("B:/ML/SETTINGS/d8_modes.en") && file_exists("B:/ML/SETTINGS/d8_32.on");
}

/* What patch_tables would change.  At ~211 ms that is 24 values (the
 * patch then worked); while Canon is still filling the tables (44 ms) or
 * redoing them for movie mode (1.0-1.9 s) it is 60, and Canon writes
 * over ours after.  So patch when it is exactly the 24 we know, whatever
 * the time; -1 if the tables aren't the ones we know. */
#define FULL32_PATCH_VALUES 24

static int would_patch(void)
{
    uint32_t cmos = MEM(SEQ_STATE + 8), eng = MEM(SEQ_STATE + 0xC), tg = MEM(SDRV_STATE + 0x78);
    if (!cmos || !eng || !tg || !tables_as_expected(cmos, eng, tg))
    {
        return -1;
    }
    dry_run = 1;
    dry_count = 0;
    patch_tables();
    dry_run = 0;
    return dry_count;
}

/* patched words Canon has written over since */
static int overwritten(void)
{
    int n = 0;
    for (int i = 0; i < n_saved; i++)
    {
        n += MEM(saved[i].addr) != saved[i].val;
    }
    return n;
}

static void full32_early_task(void)
{
    int want = -1, last_count = -2, last_over = 0;
    char line[48];
    /* until LiveView starts (or 5 s): patch when the tables hold exactly
     * the base values, undo it if the card says full 3:2 is off */
    for (int i = 0; i < 1000; i++)
    {
        /* Canon's Mem1 state (RAM; the EDMAC block may be unpowered) */
        if (MEM(MEM1_STATE + 0x58) && MEM(MEM1_STATE + 0x1C))
        {
            boot_log_line(patched ? "LiveView started, patched" : "LiveView started, not patched");
            break;
        }
        if (want < 0)
        {
            want = full32_enabled_on_card();
            if (want >= 0)
            {
                boot_log_line(want ? "card: full 3:2 on" : "card: full 3:2 off");
            }
        }
        if (!want)
        {
            if (patched)
            {
                restore_all();
                boot_log_line("restored (full 3:2 off)");
            }
            break;
        }
        if (!patched)
        {
            int n = would_patch();
            if (n != last_count)
            {
                snprintf(line, sizeof(line), "tables: %d values to patch", n);
                boot_log_line(line);
                last_count = n;
            }
            if (n == FULL32_PATCH_VALUES)
            {
                engio_later = 0;
                patch_tables();
                boot_log_line(status);
            }
        }
        else
        {
            int over = overwritten();
            if (over != last_over)
            {
                snprintf(line, sizeof(line), "Canon wrote over %d of our values", over);
                boot_log_line(line);
                for (int k = 0; k < n_saved; k++)
                {
                    if (MEM(saved[k].addr) != saved[k].val)
                    {
                        snprintf(line, sizeof(line), " %x: ours %x, now %x (was %x)", saved[k].addr, saved[k].val, MEM(saved[k].addr), saved[k].old);
                        boot_log_line(line);
                    }
                }
                last_over = over;
            }
        }
        msleep(5);
    }
    ptp_mailbox_set(5, "boot", boot_log, &boot_log_len);

    extern int ml_started;
    while (patched && !ml_started)
    {
        msleep(100);
    }
    msleep(5000);
    if (patched && !claimed)
    {
        restore_all();
        boot_log_line("not claimed by d8_modes: restored");
    }
}

void full32_early_start(void)
{
    task_create("full32_early", 0x18, 0x800, full32_early_task, 0);
}
