// Canon debug message logger, for working out how the firmware drives
// the CRX (CR3) encoder and which registers set the LV frame timing.
// Replaces DryosDebugMsg, the CRaw trace logger and engio_write with our
// own, and dumps the CRaw encoder's parameter blocks.
// Start it, take a CR3 still or RAW burst, stop it: the log is saved
// as ML/LOGS/DMLOGnn.LOG.
//
// M6II 1.1.1 only: the hooks are at ROM addresses, patched via the MMU.

#include "module.h"
#include "dryos.h"
#include "menu.h"
#include "config.h"
#include "bmp.h"
#include "patch.h"
#include "propvalues.h"
#include "raw.h"
#include "dialog.h"
#include "property.h"
#include "edmac.h"

#include "hooks.h"
#include "../../raw_video/mlv_rec/mlv.h"

extern WEAK_FUNC(ret_0) int mmu_reserve_remap_pages(int n);
extern WEAK_FUNC(ret_0) int convert_f_patch_to_patch(struct function_hook_patch *f_patch_in,
                                                     struct patch *patch_out,
                                                     uint8_t *hook_mem_out);

/* Canon stub (DIGIC 7/8): disables interrupts, takes a spinlock shared
 * between the cores, returns the old CPSR.  arm-mcr.h only declares it for
 * D78 builds.  Modules are built for ARMv5, which has no DMB/DSB mnemonics,
 * and the CP15 barrier operations are undefined on ARMv7 unless
 * SCTLR.CP15BEN is set, so emit the ARMv7 (ARM state) encodings directly. */
extern uint32_t cli_spin_lock(volatile uint32_t *lock);

/* ptp.c (core, CONFIG_PTP); weak so dmlog still loads without it */
extern WEAK_FUNC(ret_0) void ptp_mailbox_set(int slot, const char *name, void *ptr, volatile uint32_t *len);

static inline void spin_unlock(volatile uint32_t *lock, uint32_t old_cpsr)
{
    asm volatile (".word 0xf57ff05f" ::: "memory");   /* DMB SY */
    *lock = 0;
    asm volatile (".word 0xf57ff04f" ::: "memory");   /* DSB SY */
    asm volatile ("msr CPSR_c, %0" :: "r"(old_cpsr) : "memory");
}

/* startup progress on screen, so a freeze shows which step caused it */
static void step(const char *what)
{
    bmp_printf(FONT_MED, 0, 0, "dmlog: %s            ", what);
    msleep(300);
}

static char *log_buf = NULL;
static int log_size = 0;
static volatile int log_len = 0;
static volatile int log_dropped = 0;
/* a ring: when full, writing starts again at the top, so the newest lines
 * (those before an ERR70) are kept; log_wrap = where the older lines end */
static volatile int log_wrap = 0;
static volatile uint32_t log_lock = 0;
static int is_logging = 0;

static uint32_t cpu_id(void)
{
    uint32_t mpidr;
    asm volatile ("mrc p15, 0, %0, c0, c0, 5" : "=r"(mpidr));
    return mpidr & 3;
}

/* one line: "[cpu] sec.usec task: prefix" + formatted text */
static void log_line(const char *prefix, uint32_t lr, const char *fmt, va_list ap)
{
    if (!log_buf)
        return;

    /* Canon calls these from both cores and from interrupts */
    uint32_t old = cli_spin_lock(&log_lock);

    if (log_size - log_len < 512)
    {
        log_dropped++;
        log_wrap = log_len;
        log_len = 0;
    }

    /* D8 has the 20-bit microsecond timer at 0xC0242014 (src/timer.h);
     * 0xD400000C (log-d678.c, D6/D7) is never used by M6II firmware, and
     * reading it is the likely cause of a freeze on starting the log */
    uint32_t us = MEM(0xC0242014) & 0xFFFFF;
    char *p = log_buf + log_len;
    int room = log_size - log_len - 2;
    int n = snprintf(p, room, "[%d] %07d %10s %08x %s",
                     cpu_id(), us, get_current_task_name(), lr, prefix);
    n += vsnprintf(p + n, room - n, fmt, ap);
    if (n > room)
        n = room;
    p[n++] = '\n';
    log_len += n;

    spin_unlock(&log_lock, old);
}

/* Quiet: drop the classes that filled the 512 KB ring in a few seconds of
 * LiveView (power management 00, audio 0c, file I/O 14/15/17, LED 86, face
 * tracking cc, fa, eb, b2), so a LiveView start fits in it */
static CONFIG_INT("dmlog.quiet", quiet, 0);
static int quiet_class(int class)
{
    switch (class)
    {
        case 0x00: case 0x0c: case 0x14: case 0x15: case 0x17:
        case 0x86: case 0xb2: case 0xcc: case 0xeb: case 0xfa:
            return 1;
    }
    return 0;
}

/* Storage: only file I/O, the filesystem / SD speed class code (14-17,
 * 21), errors, and mlv_lite's write markers ("mlv_lite tl"), for slow
 * card writes at the start of a raw take */
/* Quiet = Shooting: RAW burst / pre-shooting ("Mechaless"): LiveView and
 * GUI (0x82, 0x83, 0x84), the camera state machine (0x92), the movie
 * recorder (0x21) */
static int shooting_class(int class)
{
    return class == 0x82 || class == 0x83 || class == 0x84 || class == 0x92 || class == 0x21;
}

static int storage_class(int class, int level, const char *fmt)
{
    switch (class)
    {
        case 0x14: case 0x15: case 0x16: case 0x17: case 0x21:
            return 1;
    }
    if (level >= 6)
    {
        return 1;
    }
    return fmt && strncmp(fmt, "mlv_lite", 8) == 0;
}

/* Slow LV start: for a LiveView start that hangs the whole camera (tasks,
 * card, USB) within 0.25 s of closing Canon's menu, before an autosave can
 * catch it (as sensor table experiments did).
 * From Canon's "PROP_LV_ACTION = 0" (menu closed, LiveView starting) for
 * 8 s, every debug message from a task with interrupts enabled sleeps
 * 20 ms after being logged, so the start takes seconds and the 0.25 s
 * autosave ends a few messages before the hang. */
#define FMT_PROP_LV_ACTION_EQ   0xE0504D6C  /* "PROP_LV_ACTION = %d" */
#define FMT_ACK                 0xE03DB26C  /* "Ack:%d(%d)" */
static CONFIG_INT("dmlog.slow_lv_start", slow_lv_start, 0);
static volatile int slow_since = 0;     /* ms clock + 1, 0 = off */
/* "PROP_LV_ACTION = 0" also comes when the ML menu opens (LiveView keeps
 * running; sleeping then stalled the camera), so only after Ack:22(2),
 * which comes once LiveView has stopped for Canon's menu */
static volatile int slow_armed = 0;

static int in_task_irqs_on(void)
{
    uint32_t cpsr;
    asm volatile ("mrs %0, CPSR" : "=r"(cpsr));
    uint32_t interrupt_level = *(volatile uint32_t *)((uintptr_t)&current_interrupt - 4);
    return !interrupt_level && !(cpsr & 0x80) && current_task;
}

/* replaces DryosDebugMsg(class, level, fmt, ...) */
void dmlog_DebugMsg(int class, int level, const char *fmt, ...)
{
    uint32_t lr = read_lr();
    if (slow_lv_start && (uint32_t) fmt == FMT_ACK)
    {
        va_list ap0;
        va_start(ap0, fmt);
        int act = va_arg(ap0, int), step = va_arg(ap0, int);
        va_end(ap0);
        if (act == 22 && step == 2)
        {
            slow_armed = 1;
        }
    }
    if (slow_lv_start && slow_armed && (uint32_t) fmt == FMT_PROP_LV_ACTION_EQ)
    {
        va_list ap0;
        va_start(ap0, fmt);
        if (va_arg(ap0, int) == 0)
        {
            slow_armed = 0;
            slow_since = get_ms_clock() + 1;
        }
        va_end(ap0);
    }
    if (quiet == 1 && quiet_class(class))
    {
        return;
    }
    if (quiet == 3 && !shooting_class(class))
    {
        return;
    }
    if (quiet == 2 && !storage_class(class, level, fmt))
    {
        return;
    }
    char prefix[16];
    snprintf(prefix, sizeof(prefix), "%02x:%02x: ", class, level);
    va_list ap;
    va_start(ap, fmt);
    log_line(prefix, lr, fmt, ap);
    va_end(ap);
    if (slow_since)
    {
        if (get_ms_clock() + 1 - slow_since > 8000)
        {
            slow_since = 0;
        }
        else if (in_task_irqs_on())
        {
            msleep(20);
        }
    }
}

/* replaces the CRaw trace logger 0xe05465a6(id, handle, fmt, ...),
 * which prints the encoder's own "EncIni:" / "Bit..Dwt.." lines */
void dmlog_CRawTrace(int id, int handle, const char *fmt, ...)
{
    uint32_t lr = read_lr();
    char prefix[24];
    snprintf(prefix, sizeof(prefix), "CRAW %02x: ", id);
    va_list ap;
    va_start(ap, fmt);
    log_line(prefix, lr, fmt, ap);
    va_end(ap);
}

static void log_printf(const char *fmt, ...)
{
    va_list ap;
    va_start(ap, fmt);
    log_line("", 0, fmt, ap);
    va_end(ap);
}

static void log_hex(const char *what, uint32_t id, const void *addr, int size)
{
    log_printf("%s id %d @%08x (%d bytes):", what, id, addr, size);
    const uint8_t *b = addr;
    for (int i = 0; i < size; i += 16)
    {
        log_printf("  +%02x: %02x%02x%02x%02x %02x%02x%02x%02x %02x%02x%02x%02x %02x%02x%02x%02x",
                   i, b[i], b[i+1], b[i+2], b[i+3], b[i+4], b[i+5], b[i+6], b[i+7],
                   b[i+8], b[i+9], b[i+10], b[i+11], b[i+12], b[i+13], b[i+14], b[i+15]);
    }
}

/* called from the Thumb stubs in hooks_thumb_M6II.c, before Canon's code runs */
void dmlog_EncInit(uint32_t id, const void *params)
{
    log_hex("CRawEncInit params", id, params, 0x60);
}

static int ram_ptr(uint32_t p)
{
    return p >= 0x1000 && p < 0x40000000;
}

void dmlog_EncSetParam(uint32_t id, const void *block, const void *ctx)
{
    const uint32_t *b = block;
    log_hex("CRawEncSetParam block", id, block, 0x80);
    log_hex("CRawEncSetParam ctx", id, ctx, 0x60);

    /* CRawEnc.c 0xe0301306: b[0] = number of 12-byte buffer descriptors
     * at b[1], read in pairs (one pair per plane; bases and sizes are
     * alignment-checked); b[4], b[5] point to more parameters */
    /* RAW burst pre-shooting: b[0] = 0x50 (one per subband and tile part) */
    if (ram_ptr(b[1]) && b[0] && b[0] <= 0x80)
    {
        log_hex("CRawEncSetParam bufs", id, (void *) b[1], b[0] * 12);
    }
    if (ram_ptr(b[4]))
    {
        log_hex("CRawEncSetParam b[4]", id, (void *) b[4], 0xB0);
    }
    if (ram_ptr(b[5]))
    {
        log_hex("CRawEncSetParam b[5]", id, (void *) b[5], 0xB0);
    }
}

/* replaces engio_write 0xe0663a7a(list): DIGIC 8 writes (register, value)
 * pairs straight to MMIO until register == -1, no shadow copy.  We do the
 * same writes and log each register whose value differs from the last one
 * we saw, which is how the frame timing registers should show up when
 * switching 24/30/60 fps. */
#define ENGIO_SLOTS 4096
static uint32_t engio_reg[ENGIO_SLOTS];
static uint32_t engio_val[ENGIO_SLOTS];
/* since the last LV slot change: number of changes, OR / AND of the values */
static uint32_t engio_cnt[ENGIO_SLOTS];
static uint32_t engio_or[ENGIO_SLOTS];
static uint32_t engio_and[ENGIO_SLOTS];
static volatile uint32_t engio_lock = 0;
static volatile int engio_full = 0;

/* LiveView sensor slot (SEQ_MODE, see d8_modes.c), logged when it changes so
 * the writes around a mode switch can be told apart */
#define LV_SLOT_M6II (0x10D88 + 8)
static uint32_t engio_last_slot = 0xFFFFFFFF;

/* Formatting a line takes ~20 us; hundreds of them inside a LiveView start
 * delayed Canon's pipeline setup until HEAD overflowed (ERR70, HeadControl.c
 * "BUFFER FULL").  The hook only stores entries; engio_flush formats them
 * when the log stops. */
#define ENGIO_EVENTS 6000
/* events kept per register and LV slot: the rest are per-frame toggles
 * (start bits 0 <-> 0x80000000), which filled 6000 in seconds */
#define ENGIO_EV_PER_REG 3

/* the whole table at each LV slot change (the old slot's state) and at
 * stop, saved as ML/LOGS/ENGIOnn.BIN: per snapshot a header {"SNAP",
 * slot, lv, time} and ENGIO_SLOTS x {reg, last value, changes, OR, AND} */
#define ENGIO_SNAPS 3     /* 80 kB each (was 5: memory is tight on M6II) */
struct engio_snap_hdr { uint32_t magic, slot, lv, us; };
#define ENGIO_SNAP_SIZE (sizeof(struct engio_snap_hdr) + ENGIO_SLOTS * 5 * 4)
static uint8_t *engio_snaps = NULL;
static volatile int engio_snap_n = 0;

static void engio_snapshot(uint32_t slot)
{
    if (!engio_snaps || engio_snap_n >= ENGIO_SNAPS)
    {
        return;
    }
    uint8_t *s = engio_snaps + engio_snap_n * ENGIO_SNAP_SIZE;
    struct engio_snap_hdr *h = (struct engio_snap_hdr *) s;
    h->magic = 0x50414E53;  /* "SNAP" */
    h->slot = slot;
    h->lv = lv;
    h->us = MEM(0xC0242014) & 0xFFFFF;
    uint32_t *e = (uint32_t *)(s + sizeof(*h));
    for (int i = 0; i < ENGIO_SLOTS; i++, e += 5)
    {
        e[0] = engio_reg[i];
        e[1] = engio_val[i];
        e[2] = engio_cnt[i];
        e[3] = engio_or[i];
        e[4] = engio_and[i];
        engio_cnt[i] = 0;
        engio_or[i] = engio_and[i] = engio_val[i];
    }
    engio_snap_n++;
}

static void engio_save_snapshots(void)
{
    if (!engio_snaps || !engio_snap_n)
    {
        return;
    }
    char filename[100];
    get_numbered_file_name("ML/LOGS/ENGIO%02d.BIN", 99, filename, sizeof(filename));
    FILE *f = FIO_CreateFile(filename);
    if (f)
    {
        FIO_WriteFile(f, engio_snaps, engio_snap_n * ENGIO_SNAP_SIZE);
        FIO_CloseFile(f);
    }
    log_printf("engio: %d snapshots -> %s", engio_snap_n, f ? filename : "(could not create)");
}
#define ENGIO_EV_SLOT 0         /* reg 0: val = LV slot, prev = lv */
static struct engio_event { uint32_t us, reg, val, prev, lr; } *engio_ev = NULL;
static volatile int engio_ev_n = 0;
static volatile int engio_ev_dropped = 0;

static void engio_event(uint32_t reg, uint32_t val, uint32_t prev, uint32_t lr)
{
    if (!engio_ev || engio_ev_n >= ENGIO_EVENTS)
    {
        engio_ev_dropped++;
        return;
    }
    struct engio_event *e = &engio_ev[engio_ev_n++];
    e->us = MEM(0xC0242014) & 0xFFFFF;
    e->reg = reg;
    e->val = val;
    e->prev = prev;
    e->lr = lr;
}

/* prev's top bit set: the register was new */
#define ENGIO_NEW 0x80000000u

/* events formatted into the text log so far: autosave formats new ones
 * on each pass (a freeze leaves no stop, and the fast saves only carry
 * the text log); stop formats the rest */
static int engio_flushed = 0;

static void engio_flush_new(void)
{
    int n = engio_ev_n;
    for (int i = engio_flushed; i < n; i++)
    {
        struct engio_event *e = &engio_ev[i];
        if (e->reg == ENGIO_EV_SLOT)
            log_printf("engio %07d: LV slot %d (lv %d, lr %08x)", e->us, e->val, e->prev, e->lr);
        else if (e->prev == ENGIO_NEW)
            log_printf("engio %07d %08x = %08x (new, lr %08x)", e->us, e->reg, e->val, e->lr);
        else
            log_printf("engio %07d %08x = %08x (was %08x, lr %08x)", e->us, e->reg, e->val, e->prev, e->lr);
    }
    engio_flushed = n;
}

static void engio_flush(void)
{
    engio_flush_new();
    if (engio_ev_n || engio_ev_dropped)
        log_printf("engio: %d events, %d dropped", engio_ev_n, engio_ev_dropped);
    engio_ev_n = 0;
    engio_flushed = 0;
}

uint32_t dmlog_engio_write(uint32_t *list)
{
    uint32_t lr = read_lr();
    uint32_t n = 0;

    if (MEM(LV_SLOT_M6II) != engio_last_slot)
    {
        uint32_t old = cli_spin_lock(&engio_lock);
        if (engio_last_slot != 0xFFFFFFFF)
        {
            engio_snapshot(engio_last_slot);
        }
        engio_last_slot = MEM(LV_SLOT_M6II);
        engio_event(ENGIO_EV_SLOT, engio_last_slot, lv, lr);
        spin_unlock(&engio_lock, old);
    }

    for (; list[0] != 0xFFFFFFFF && n < 500; list += 2, n++)
    {
        uint32_t reg = list[0];
        uint32_t val = list[1];
        MEM(reg) = val;

        /* open addressing on the register address */
        uint32_t old = cli_spin_lock(&engio_lock);
        uint32_t h = (reg >> 2) & (ENGIO_SLOTS - 1);
        uint32_t prev = 0;
        for (int probe = 0; probe < 16; probe++, h = (h + 1) & (ENGIO_SLOTS - 1))
        {
            if (engio_reg[h] == reg)
            {
                prev = engio_val[h];
                if (prev != val)
                {
                    if (engio_cnt[h] < ENGIO_EV_PER_REG)
                        engio_event(reg, val, prev, lr);
                    engio_cnt[h]++;
                }
                engio_val[h] = val;
                engio_or[h] |= val;
                engio_and[h] &= val;
                break;
            }
            if (engio_reg[h] == 0)
            {
                engio_reg[h] = reg;
                engio_val[h] = val;
                engio_cnt[h] = 0;
                engio_or[h] = engio_and[h] = val;
                engio_event(reg, val, ENGIO_NEW, lr);
                break;
            }
            if (probe == 15)
                engio_full++;
        }
        spin_unlock(&engio_lock, old);
    }
    return n;
}

/* M6II 1.1.1 hook sites; orig_content is checked before patching.
 * ML on M6II has room to remap one 64 KB ROM page (generic_mmu_space in
 * patch_mmu.c), and a used page stays allocated until reboot, so hooks
 * come in groups that each fit one page: one group per boot. */
enum { HOOKS_NONE, HOOKS_DEBUGMSG, HOOKS_CRAW, HOOKS_ENGIO, HOOKS_EDMAC, HOOKS_ENC, HOOKS_CMOS };
/* kept across restarts: a hook group is chosen, then the camera restarted */
static CONFIG_INT("dmlog.hooks", hook_group, HOOKS_NONE);

static struct function_hook_patch f_patches_debugmsg[] = {
    {
        .patch_addr = 0xe0599848, // DryosDebugMsg
        .orig_content = {0x0f, 0xb4, 0x2d, 0xe9, 0xf0, 0x4f, 0x8a, 0x46},
        .target_function_addr = (uint32_t)dmlog_DebugMsg,
        .description = "dmlog: DryosDebugMsg"
    },
};

static struct function_hook_patch f_patches_craw[] = {
    {
        .patch_addr = 0xe0300ed0, // CRawEnc.c EncInit(id, params)
        .orig_content = {0x2d, 0xe9, 0xff, 0x41, 0x06, 0x46, 0x34, 0x4f},
        .target_function_addr = (uint32_t)hook_EncInit_M6II,
        .description = "dmlog: CRaw EncInit"
    },
    {
        .patch_addr = 0xe0301306, // CRawEnc.c direct SetParam/start(id, block, ctx)
        .orig_content = {0x2d, 0xe9, 0xf7, 0x4f, 0x0c, 0x46, 0x61, 0x48},
        .target_function_addr = (uint32_t)hook_EncSetParam_M6II,
        .description = "dmlog: CRaw EncSetParam"
    },
};

static struct function_hook_patch f_patches_engio[] = {
    {
        .patch_addr = 0xe0663a7a, // engio_write (EngineDS::DSEngIO.c)
        .orig_content = {0x70, 0xb5, 0x00, 0x25, 0x04, 0x46, 0x4f, 0xf4},
        .target_function_addr = (uint32_t)dmlog_engio_write,
        .description = "dmlog: engio_write"
    },
    {
        /* same list format, with Canon's per-write trace; the sensor mode
         * tables are written through this one.  Replaced without the trace. */
        .patch_addr = 0xe0663ad8, // engio_write_traced
        .orig_content = {0x2d, 0xe9, 0xf0, 0x41, 0x04, 0x46, 0x16, 0x48},
        .target_function_addr = (uint32_t)dmlog_engio_write,
        .description = "dmlog: engio_write (traced)"
    },
};

/* EDMAC setup: which channels Canon programs, with what geometry, buffers
 * and connections, e.g. during a RAW burst (where do the full frames go?).
 * All channels; sizes, packs and connections logged when they change,
 * addresses on every change (rings show up as alternating buffers), starts
 * counted and summarised once a second with the sensor drive mode. */
#define EDMAC_TABLE_M6II 0xe1008944     /* 8-byte entries, base address first */
#define EDMAC_CHANNELS   76
#define DTS_MASTER_DRIVE_MODE 0x4962C   /* DTS master +0xC: sensor drive mode */

static uint32_t edmac_last[3][EDMAC_CHANNELS][16];
static volatile uint32_t edmac_starts[EDMAC_CHANNELS];
static uint32_t edmac_start_lr[EDMAC_CHANNELS];
static uint32_t edmac_conn[2][EDMAC_CHANNELS];

static int edmac_changed(int kind, uint32_t ch, const uint32_t *words, int n)
{
    int changed = 0;
    for (int i = 0; i < n; i++)
    {
        if (edmac_last[kind][ch][i] != words[i])
        {
            edmac_last[kind][ch][i] = words[i];
            changed = 1;
        }
    }
    return changed;
}

void dmlog_edmac_read_pack(uint32_t ch, const uint32_t *fields, uint32_t lr, uint32_t kind)
{
    if (ch >= EDMAC_CHANNELS || !edmac_changed(0, ch, fields, 4))
        return;
    log_printf("edmac %d ms read_pack ch %d lr %08x fields %x %x %x %x",
               get_ms_clock(), ch, lr, fields[0], fields[1], fields[2], fields[3]);
}

/* info = ML's D8 struct edmac_info: off1s off1a off1b off2s off2a off2b
 * off3 xs xa xb ys ya yb xn yn */
void dmlog_edmac_set_size(uint32_t ch, const uint32_t *info, uint32_t lr, uint32_t kind)
{
    if (ch >= EDMAC_CHANNELS || !edmac_changed(1, ch, info, 15))
        return;
    log_printf("edmac %d ms set_size ch %d lr %08x xb %d yb %d xn %d yn %d xa %d ya %d off %x %x %x %x %x %x %x xs %d ys %d",
               get_ms_clock(), ch, lr, info[9], info[12], info[13], info[14], info[8], info[11],
               info[0], info[1], info[2], info[3], info[4], info[5], info[6], info[7], info[10]);
}

/* replaces edmac_set_addr 0xe0580962 (too short to hook and replay):
 * MEM(base(ch) + 0xA0) = addr */
void dmlog_edmac_set_addr(uint32_t ch, uint32_t addr)
{
    uint32_t lr = read_lr();
    MEM(MEM(EDMAC_TABLE_M6II + ch * 8) + 0xA0) = addr;
    if (ch < EDMAC_CHANNELS && edmac_changed(2, ch, &addr, 1))
    {
        log_printf("edmac %d ms set_addr ch %d lr %08x addr %08x", get_ms_clock(), ch, lr, addr);
    }
}

/* called for every transfer: count only */
void dmlog_edmac_start(uint32_t ch, uint32_t lr)
{
    if (ch < EDMAC_CHANNELS)
    {
        edmac_starts[ch]++;
        edmac_start_lr[ch] = lr;
    }
}

void dmlog_edmac_connect(uint32_t ch, uint32_t conn, uint32_t lr, uint32_t kind)
{
    if (ch >= EDMAC_CHANNELS || edmac_conn[kind][ch] == conn + 1)
        return;
    edmac_conn[kind][ch] = conn + 1;
    log_printf("edmac %d ms connect_%s ch %d lr %08x conn %x",
               get_ms_clock(), kind ? "read" : "write", ch, lr, conn);
}

/* once a second while logging: starts per channel since the last summary */
static void edmac_summary(void)
{
    static uint32_t prev[EDMAC_CHANNELS];
    char line[400];
    int n = snprintf(line, sizeof(line), "edmac %d ms drive mode %d, starts/s:",
                     get_ms_clock(), MEM(DTS_MASTER_DRIVE_MODE));
    for (int ch = 0; ch < EDMAC_CHANNELS && n < (int) sizeof(line) - 16; ch++)
    {
        uint32_t now = edmac_starts[ch];
        if (now != prev[ch])
        {
            n += snprintf(line + n, sizeof(line) - n, " %d:%d", ch, now - prev[ch]);
            prev[ch] = now;
        }
    }
    log_printf("%s", line);
}

/* first start site per channel, logged once */
static void edmac_start_sites(void)
{
    static uint32_t logged[EDMAC_CHANNELS];
    for (int ch = 0; ch < EDMAC_CHANNELS; ch++)
    {
        if (edmac_start_lr[ch] && logged[ch] != edmac_start_lr[ch])
        {
            logged[ch] = edmac_start_lr[ch];
            log_printf("edmac start ch %d lr %08x", ch, edmac_start_lr[ch]);
        }
    }
}

static struct function_hook_patch f_patches_edmac[] = {
    {
        .patch_addr = 0xe0581942, // read pack setter (ch, fields)
        .orig_content = {0x7c, 0xb5, 0x05, 0x46, 0x7f, 0x48, 0x0c, 0x46},
        .target_function_addr = (uint32_t)hook_ReadPack_M6II,
        .description = "dmlog: EDMAC read pack"
    },
    {
        .patch_addr = 0xe058096e, // edmac_set_size (ch, info)
        .orig_content = {0x2d, 0xe9, 0xf0, 0x4f, 0x05, 0x46, 0x90, 0x48},
        .target_function_addr = (uint32_t)hook_SetSize_M6II,
        .description = "dmlog: edmac_set_size"
    },
    {
        .patch_addr = 0xe0580962, // edmac_set_addr (ch, addr), replaced
        .orig_content = {0x95, 0x4a, 0x52, 0xf8, 0x30, 0x00, 0xc0, 0xf8},
        .target_function_addr = (uint32_t)dmlog_edmac_set_addr,
        .description = "dmlog: edmac_set_addr"
    },
    {
        .patch_addr = 0xe0580782, // StartEDmac (ch)
        .orig_content = {0x70, 0xb5, 0x04, 0x46, 0xff, 0xf7, 0x29, 0xfc},
        .target_function_addr = (uint32_t)hook_Start_M6II,
        .description = "dmlog: StartEDmac"
    },
    {
        .patch_addr = 0xe0580502, // ConnectWriteEDmac (ch, conn)
        .orig_content = {0x2d, 0xe9, 0xf0, 0x5f, 0x04, 0x46, 0xdf, 0xf8},
        .target_function_addr = (uint32_t)hook_ConnectWrite_M6II,
        .description = "dmlog: ConnectWriteEDmac"
    },
    {
        .patch_addr = 0xe05806e4, // ConnectReadEDmac (ch, conn)
        .orig_content = {0x2d, 0xe9, 0xf0, 0x41, 0x05, 0x46, 0x35, 0x4f},
        .target_function_addr = (uint32_t)hook_ConnectRead_M6II,
        .description = "dmlog: ConnectReadEDmac"
    },
};

/* Encoder start: Canon starts every CRX encode with StartEncode
 * 0xe06662f6(jobs, n, cbr, ctx): n 0x3C-byte job records (w1/w2 size, w5
 * bits, w6 source, w7 pitch in px), cbr(ctx, results[], ...) on AlgsMgr's
 * task when done, results[i] = handle of job i.  Two callers: the still /
 * CR3 path 0xe025a4bc (cbr 0xe0257ac3 AlgsEncCompCBR; RAW burst and
 * pre-shooting use this one, 2 jobs per frame) and Warp's image codec
 * 0xe068af12 (cbr 0xe068ae33).  Logged: the jobs, and each result's
 * handles (the first ones in full).  Optionally the full raw job
 * (5376x3560, 1:1) is cut to 2.39:1: rows 692..2939 of the frame (active
 * rows 72..3559). */
static CONFIG_INT("dmlog.crop239", crop239, 0);
#define STILL_ENC_CBR   0xe0257ac3
#define CODEC_ENC_CBR   0xe068ae33
#define FULL_W          5376
#define FULL_H          3560
#define CROP_H          2248
#define CROP_Y          692
/* AlgsMgr (state *0x5bbc) sets an encoder up from the job's size only
 * while its "initialised" flag (+0x138 + id * 4) is 0, then sets it to 1
 * (0xe02acf30): later jobs keep the first size.  Job type 9 (the 1:1 full
 * raw) runs on encoder id 0x12 (0xe02acde8).  Cleared when the height
 * changes (and on the first one seen), so the job sets it up again. */
#define ALGS_STATE_PTR_ENC  0x5bbc
#define ALGS_INIT_FLAGS     0x138
#define ENC_ID_TYPE9        0x12
static uint32_t enc_last_h9 = 0;
static uint32_t enc_last_src9 = 0;     /* source of the latest type 9 job */
static uint32_t enc_last_pitch9 = 0;

typedef void (*enc_cbr_t)(uint32_t ctx, uint32_t *results, uint32_t a2, uint32_t a3);
static volatile int enc_starts = 0;
static volatile int enc_done = 0;

/* the latest cropped full-raw frame, for checking it on a PC (mailbox slot
 * 3 "encframe"): its part addresses and sizes from
 * the result handle (+0x1C, +0x20; entry 0 = Canon's sample header, then
 * parts and empty entries) and a copy of the header, which lives in a heap
 * buffer that is reused soon.  The parts stay in AlgsMgr's ring buffers
 * until the next encode, so read them right after releasing the
 * half-press. */
#define ENC_FRAME_MAGIC 0x46434E45      /* "ENCF" */
static struct
{
    uint32_t magic, seq, width, height, count, hdr_size, src, pitch_bytes;
    uint32_t addr[256];
    uint32_t size[256];
    uint8_t hdr[2048];
} enc_frame;
static volatile uint32_t enc_frame_len = 0;

static void enc_keep_frame(int k, uint32_t h)
{
    uint32_t cnt = MIN(MEM(h + 0x18), 256);
    uint32_t addrs = MEM(h + 0x1C), sizes = MEM(h + 0x20);
    if (!ram_ptr(addrs) || !ram_ptr(sizes) || !cnt)
    {
        return;
    }
    enc_frame_len = 0;
    enc_frame.magic = ENC_FRAME_MAGIC;
    enc_frame.seq = k;
    enc_frame.width = FULL_W;
    enc_frame.height = CROP_H;
    enc_frame.count = cnt;
    enc_frame.src = enc_last_src9;
    enc_frame.pitch_bytes = enc_last_pitch9;
    for (uint32_t i = 0; i < cnt; i++)
    {
        enc_frame.addr[i] = MEM(addrs + 4 * i);
        enc_frame.size[i] = MEM(sizes + 4 * i);
    }
    /* header buffer: uncached heap (0x40xxxxxx) */
    uint32_t ha = enc_frame.addr[0];
    enc_frame.hdr_size = (ha >= 0x1000 && ha < 0xC0000000) ? MIN(enc_frame.size[0], sizeof(enc_frame.hdr)) : 0;
    memcpy(enc_frame.hdr, (void *) ha, enc_frame.hdr_size);
    enc_frame_len = sizeof(enc_frame);
}

static void enc_log_done(const char *path, uint32_t *results, int n)
{
    int k = ++enc_done;
    /* result handle: +0x18 part count (161 for a 5376-wide frame), arrays
     * at +0x1C/+0x20/+0x24/+0x28 (inside the handle's allocation): which
     * one holds the part sizes is open, so log the sum of each */
    for (int i = 0; i < n && i < 2; i++)
    {
        uint32_t h = results[i];
        if (!ram_ptr(h))
        {
            continue;
        }
        uint32_t cnt = MIN(MEM(h + 0x18), 256);
        uint32_t sum[4] = {0};
        for (int a = 0; a < 4; a++)
        {
            uint32_t arr = MEM(h + 0x1C + 4 * a);
            for (uint32_t e = 0; ram_ptr(arr) && e < cnt; e++)
            {
                sum[a] += MEM(arr + 4 * e);
            }
        }
        /* ML's vsnprintf has no %u */
        if (i == 1 && enc_last_h9 == CROP_H)
        {
            enc_keep_frame(k, h);
        }
        log_printf("enc %d ms done #%d (%s) job %d handle %x parts %d sums %d %d %d %d",
                   get_ms_clock(), k, path, i, h, (int) cnt, (int) sum[0], (int) sum[1], (int) sum[2], (int) sum[3]);
        for (int a = 0; k <= 2 && a < 4; a++)
        {
            uint32_t arr = MEM(h + 0x1C + 4 * a);
            if (ram_ptr(arr))
            {
                log_printf("  array +%x @%x: %x %x %x %x %x %x %x %x", 0x1C + 4 * a, arr,
                           MEM(arr), MEM(arr + 4), MEM(arr + 8), MEM(arr + 12),
                           MEM(arr + 16), MEM(arr + 20), MEM(arr + 24), MEM(arr + 28));
            }
        }
    }
    for (int i = 0; k <= 4 && i < n; i++)
    {
        if (ram_ptr(results[i]))
        {
            log_hex("enc handle", i, (void *) results[i], 0x100);
        }
    }
}

/* Record test: while armed, every cropped full-raw frame Canon encodes
 * (RAW burst pre-shooting, half-press) goes into an MLV on the card, to
 * measure whether we can keep up: copying ~8.7 MB of encoder output a
 * frame out of AlgsMgr's ring buffers (scatter-gather DMA, core
 * crx_copy_parts) and writing ~200 MB/s.  In Canon's completion callback
 * (AlgsMgr task) only the part list and Canon's sample header (a heap
 * buffer Canon reuses soon) are taken; a copy task fetches the parts
 * (they stay in the rings for the ~0.5 s of pre-shooting), a writer task
 * appends VIDF blocks.  The take ends 0.7 s after the last frame.  Each
 * VIDF payload = Canon's CR3 sample (header for 2 tiles + parts): RAWI
 * says 5376x2248 14-bit, the class CRX flag; lossy and tiled are not
 * flagged (test files for a PC-side decoder).  From the time pre-shooting
 * was the 1:1 source; d8_modes' 1:1 takes replaced it. */
extern int crx_copy_parts(uint8_t *dst, uint32_t n, const uint32_t *addrs, const uint32_t *sizes);

#define REC_SLOTS       16
#define REC_SLOT_SIZE   (12 << 20)
#define REC_PARTS       200
#define REC_MEM         (128 << 20)
#define REC_END_MS      700
#define REC_VIDF        sizeof(mlv_vidf_hdr_t)

enum { REC_FREE, REC_COPY, REC_WRITE };
static struct rec_slot
{
    uint8_t *ptr;
    volatile int state;
    uint32_t n, hdr_size, data_size, frame;
    uint64_t us;
    uint32_t addr[REC_PARTS], size[REC_PARTS];
} rec_slots[REC_SLOTS];
static int rec_nslots = 0;
static int rec_next = 0;
static struct memSuite *rec_suite = NULL;
static volatile int rec_armed = 0;
static volatile uint32_t rec_frames = 0, rec_dropped = 0, rec_written = 0, rec_bad = 0;
static volatile uint32_t rec_last_ms = 0;
static uint64_t rec_t0 = 0;
static uint32_t rec_copy_sum = 0, rec_copy_max = 0, rec_write_sum = 0, rec_write_max = 0;
static uint64_t rec_bytes = 0;
static uint32_t rec_write_ms0 = 0;
static struct msg_queue *rec_copy_mq = NULL, *rec_write_mq = NULL;
static FILE *rec_file = NULL;
static char rec_name[64];

/* AlgsMgr task, before Canon's callback: h = the full raw job's handle */
static void rec_frame_done(uint32_t h)
{
    if (!rec_armed || enc_last_h9 != CROP_H || !ram_ptr(h))
    {
        return;
    }
    uint32_t cnt = MEM(h + 0x18), addrs = MEM(h + 0x1C), sizes = MEM(h + 0x20);
    uint32_t k = rec_frames++;
    if (!cnt || cnt > REC_PARTS || !ram_ptr(addrs) || !ram_ptr(sizes))
    {
        rec_bad++;
        return;
    }
    uint32_t hsz = MEM(sizes), total = 0;
    for (uint32_t i = 1; i < cnt; i++)
    {
        total += MEM(sizes + 4 * i);
    }
    struct rec_slot *s = &rec_slots[rec_next];
    if (s->state != REC_FREE || hsz > 4096 || REC_VIDF + hsz + total + 512 > REC_SLOT_SIZE)
    {
        rec_dropped++;
        return;
    }
    rec_next = (rec_next + 1) % rec_nslots;
    if (!k)
    {
        rec_t0 = get_us_clock();
    }
    memcpy(s->ptr + REC_VIDF, (void *) MEM(addrs), hsz);
    for (uint32_t i = 0; i < cnt; i++)
    {
        s->addr[i] = MEM(addrs + 4 * i);
        s->size[i] = MEM(sizes + 4 * i);
    }
    s->n = cnt;
    s->hdr_size = hsz;
    s->frame = k;
    s->us = get_us_clock() - rec_t0;
    s->state = REC_COPY;
    rec_last_ms = get_ms_clock();
    msg_queue_post(rec_copy_mq, s - rec_slots);
}

static void rec_copy_task(void)
{
    TASK_LOOP
    {
        uint32_t i;
        if (msg_queue_receive(rec_copy_mq, &i, 0) || i >= REC_SLOTS)
        {
            continue;
        }
        struct rec_slot *s = &rec_slots[i];
        uint64_t t = get_us_clock();
        s->data_size = crx_copy_parts(s->ptr + REC_VIDF + s->hdr_size, s->n, s->addr, s->size);
        uint32_t dt = get_us_clock() - t;
        rec_copy_sum += dt;
        rec_copy_max = MAX(rec_copy_max, dt);
        s->state = REC_WRITE;
        msg_queue_post(rec_write_mq, i);
    }
}

static void rec_write_headers(void)
{
    mlv_file_hdr_t fh;
    memset(&fh, 0, sizeof(fh));
    memcpy(fh.fileMagic, "MLVI", 4);
    fh.blockSize = sizeof(fh);
    memcpy(fh.versionString, MLV_VERSION_STRING, sizeof(MLV_VERSION_STRING));
    fh.fileGuid = get_us_clock();
    fh.fileCount = 1;
    fh.videoClass = MLV_VIDEO_CLASS_RAW | MLV_VIDEO_CLASS_FLAG_CRX | MLV_VIDEO_CLASS_FLAG_CRX_FULL;
    fh.sourceFpsNom = 23976;
    fh.sourceFpsDenom = 1000;
    FIO_WriteFile(rec_file, &fh, sizeof(fh));

    mlv_rawi_hdr_t ri;
    memset(&ri, 0, sizeof(ri));
    memcpy(ri.blockType, "RAWI", 4);
    ri.blockSize = sizeof(ri);
    ri.xRes = FULL_W;
    ri.yRes = CROP_H;
    ri.raw_info.api_version = raw_info.api_version;
    ri.raw_info.width = FULL_W;
    ri.raw_info.height = CROP_H;
    ri.raw_info.pitch = FULL_W * 14 / 8;
    ri.raw_info.bits_per_pixel = 14;
    ri.raw_info.black_level = 2048;
    ri.raw_info.white_level = 16380;
    /* optical black: the left 144 columns (rows 692.. have none above) */
    ri.raw_info.jpeg.x = 144;
    ri.raw_info.jpeg.width = FULL_W - 144;
    ri.raw_info.jpeg.height = CROP_H;
    ri.raw_info.active_area.x1 = 144;
    ri.raw_info.active_area.y2 = CROP_H;
    ri.raw_info.active_area.x2 = FULL_W;
    ri.raw_info.cfa_pattern = 0x02010100;
    ri.raw_info.calibration_illuminant1 = raw_info.calibration_illuminant1;
    memcpy(ri.raw_info.color_matrix1, raw_info.color_matrix1, sizeof(ri.raw_info.color_matrix1));
    ri.raw_info.dynamic_range = raw_info.dynamic_range;
    FIO_WriteFile(rec_file, &ri, sizeof(ri));

    /* a NULL block up to 512 bytes, so every frame starts on a sector
     * (unaligned 8.6 MB writes went at ~25 MB/s) */
    static uint8_t pad[512];
    mlv_hdr_t *nh = (mlv_hdr_t *) pad;
    memset(pad, 0, sizeof(pad));
    memcpy(nh->blockType, "NULL", 4);
    nh->blockSize = 512 - (sizeof(fh) + sizeof(ri)) % 512;
    if (nh->blockSize < sizeof(mlv_hdr_t))
    {
        nh->blockSize += 512;
    }
    FIO_WriteFile(rec_file, pad, nh->blockSize);
}

static void rec_finish(void)
{
    uint32_t ms = get_ms_clock() - rec_write_ms0;
    if (rec_file)
    {
        FIO_CloseFileSC(rec_file, rec_name);
        rec_file = NULL;
    }
    int n = MAX((int) rec_written, 1);
    log_printf("rec: %s: %d frames seen, %d written, %d dropped (no slot), %d unreadable; %d MB in %d ms = %d MB/s;"
               " copy avg %d max %d us, write avg %d max %d us",
               rec_name, rec_frames, rec_written, rec_dropped, rec_bad, (int) (rec_bytes >> 20), ms,
               ms ? (int) (rec_bytes / 1000 / ms) : 0,
               rec_copy_sum / n, rec_copy_max, rec_write_sum / n, rec_write_max);
    NotifyBox(8000, "%d frames, %d dropped, %d MB/s", rec_written, rec_dropped, ms ? (int) (rec_bytes / 1000 / ms) : 0);
    rec_armed = 0;
    if (rec_suite)
    {
        shoot_free_suite(rec_suite);
        rec_suite = NULL;
    }
}

static int rec_busy(void)
{
    for (int i = 0; i < rec_nslots; i++)
    {
        if (rec_slots[i].state != REC_FREE)
        {
            return 1;
        }
    }
    return 0;
}

static void rec_write_task(void)
{
    TASK_LOOP
    {
        uint32_t i;
        if (msg_queue_receive(rec_write_mq, &i, 100))
        {
            /* end of the take: no frames for a while, everything written */
            if (rec_armed && rec_frames && get_ms_clock() - rec_last_ms > REC_END_MS && !rec_busy())
            {
                rec_finish();
            }
            continue;
        }
        if (i >= REC_SLOTS)
        {
            continue;
        }
        struct rec_slot *s = &rec_slots[i];
        if (!rec_written)
        {
            rec_write_ms0 = get_ms_clock();     /* rate from the first frame on */
        }
        mlv_vidf_hdr_t *v = (mlv_vidf_hdr_t *) s->ptr;
        memset(v, 0, REC_VIDF);
        memcpy(v->blockType, "VIDF", 4);
        /* padded to whole sectors with zeros (the CRX header gives the
         * part sizes, so readers ignore what follows) */
        uint32_t used = REC_VIDF + s->hdr_size + s->data_size;
        v->blockSize = (used + 511) & ~511;
        memset(s->ptr + used, 0, v->blockSize - used);
        v->timestamp = s->us;
        v->frameNumber = s->frame;
        v->cropPosY = CROP_Y;
        uint64_t t = get_us_clock();
        if (rec_file)
        {
            FIO_WriteFile(rec_file, s->ptr, v->blockSize);
        }
        uint32_t dt = get_us_clock() - t;
        rec_write_sum += dt;
        rec_write_max = MAX(rec_write_max, dt);
        rec_bytes += v->blockSize;
        rec_written++;
        if (rec_written <= 3 || rec_written % 24 == 0)
        {
            log_printf("rec %d ms: frame %d, %d + %d bytes, copied, written in %d us; %d seen, %d dropped",
                       get_ms_clock(), s->frame, s->hdr_size, s->data_size, dt, rec_frames, rec_dropped);
        }
        s->state = REC_FREE;
    }
}

/* A new file starts slowly on this card (~25 MB/s for the first 1-2.5 s:
 * frames 0-80 of PRE00001 dropped, then 8.3 MB in ~38 ms): open the file
 * when armed and write 4 MB NULL blocks until one goes at 160 MB/s or
 * better (as mlv_lite's start warm-up), up to 8 s */
#define REC_WARM_BLOCK  (4 << 20)
#define REC_WARM_US     (REC_WARM_BLOCK / 160)
#define REC_WARM_MS     8000

static int rec_open_and_warm_up(void)
{
    get_numbered_file_name("B:/DCIM/100CANON/PRE%05d.MLV", 99999, rec_name, sizeof(rec_name));
    rec_file = FIO_CreateFileSC(rec_name);
    if (!rec_file)
    {
        log_printf("rec: can't create %s", rec_name);
        return 0;
    }
    rec_write_headers();
    uint8_t *buf = rec_slots[0].ptr;
    memset(buf, 0, REC_WARM_BLOCK);
    mlv_hdr_t *nh = (mlv_hdr_t *) buf;
    memcpy(nh->blockType, "NULL", 4);
    nh->blockSize = REC_WARM_BLOCK;
    uint32_t t0 = get_ms_clock();
    int n = 0;
    while (get_ms_clock() - t0 < REC_WARM_MS)
    {
        uint64_t t = get_us_clock();
        FIO_WriteFile(rec_file, buf, REC_WARM_BLOCK);
        uint32_t dt = get_us_clock() - t;
        n++;
        if (dt < REC_WARM_US)
        {
            break;
        }
    }
    log_printf("rec: warm-up %d x 4 MB in %d ms", n, get_ms_clock() - t0);
    return n;
}

static void rec_toggle_task(void)
{
    if (rec_armed)
    {
        if (!rec_frames)
        {
            /* nothing recorded: close the warmed-up file, give the memory back */
            rec_finish();
        }
        rec_armed = 0;
        NotifyBox(3000, "Record test off");
        return;
    }
    if (hook_group != HOOKS_ENC || !is_logging || !crop239)
    {
        NotifyBox(5000, "Needs Hooks = Encoder start, log ON, crop ON");
        return;
    }
    if (rec_suite || rec_file)
    {
        NotifyBox(3000, "Record test: previous take still open");
        return;
    }
    if (!rec_copy_mq)
    {
        rec_copy_mq = msg_queue_create("rec_copy", REC_SLOTS + 2);
        rec_write_mq = msg_queue_create("rec_write", REC_SLOTS + 2);
        task_create("rec_copy", 0x12, 0x1000, rec_copy_task, 0);
        task_create("rec_write", 0x13, 0x1000, rec_write_task, 0);
    }
    rec_suite = shoot_malloc_suite(REC_MEM);
    if (!rec_suite)
    {
        NotifyBox(5000, "Record test: no shoot memory");
        return;
    }
    rec_nslots = 0;
    for (struct memChunk *c = GetFirstChunkFromSuite(rec_suite); c && rec_nslots < REC_SLOTS;
         c = GetNextMemoryChunk(rec_suite, c))
    {
        uint32_t a = (uint32_t) GetMemoryAddressOfMemoryChunk(c);
        uint32_t end = a + GetSizeOfMemoryChunk(c);
        a = (a + 63) & ~63;
        while (a + REC_SLOT_SIZE <= end && rec_nslots < REC_SLOTS)
        {
            rec_slots[rec_nslots].ptr = (uint8_t *) a;
            rec_slots[rec_nslots].state = REC_FREE;
            rec_nslots++;
            a += REC_SLOT_SIZE;
        }
    }
    rec_next = 0;
    rec_frames = rec_dropped = rec_written = rec_bad = 0;
    rec_copy_sum = rec_copy_max = rec_write_sum = rec_write_max = 0;
    rec_bytes = 0;
    rec_last_ms = get_ms_clock();
    if (rec_nslots && !rec_open_and_warm_up())
    {
        shoot_free_suite(rec_suite);
        rec_suite = NULL;
        NotifyBox(5000, "Record test: can't create the file");
        return;
    }
    rec_write_ms0 = get_ms_clock();
    log_printf("rec: armed, %d slots of %d MB", rec_nslots, REC_SLOT_SIZE >> 20);
    NotifyBox(5000, "Record test armed: %d slots. Half-press to record.", rec_nslots);
    rec_armed = 1;
}

static MENU_UPDATE_FUNC(rec_update)
{
    MENU_SET_VALUE(rec_armed ? "ARMED" : "OFF");
    if (rec_frames)
    {
        MENU_SET_RINFO("%d/%d, %d drop", rec_written, rec_frames, rec_dropped);
    }
}

static void enc_done_still(uint32_t ctx, uint32_t *results, uint32_t a2, uint32_t a3)
{
    enc_log_done("still", results, 2);
    rec_frame_done(results[1]);
    ((enc_cbr_t) STILL_ENC_CBR)(ctx, results, a2, a3);
}

static void enc_done_codec(uint32_t ctx, uint32_t *results, uint32_t a2, uint32_t a3)
{
    /* a2 = number of jobs */
    enc_log_done("codec", results, MIN((int) a2, 3));
    ((enc_cbr_t) CODEC_ENC_CBR)(ctx, results, a2, a3);
}

uint32_t dmlog_StartEncode(uint32_t *jobs, uint32_t n, uint32_t cbr, uint32_t ctx)
{
    int k = ++enc_starts;
    for (uint32_t i = 0; i < n && i < 3; i++)
    {
        uint32_t *j = jobs + i * 15;
        int cropped = 0;
        if (crop239 && j[1] == FULL_W && j[2] == FULL_H)
        {
            j[6] += CROP_Y * (j[7] * j[5] / 8);
            j[2] = CROP_H;
            cropped = 1;
        }
        if (j[0] == 9 && j[2] != enc_last_h9)
        {
            uint32_t st = MEM(ALGS_STATE_PTR_ENC);
            /* also the first time: a half-press before the log started
             * may have set the encoder up already */
            if (st)
            {
                log_printf("enc: height %d -> %d, encoder %d set up again (flag was %d)",
                           enc_last_h9, j[2], ENC_ID_TYPE9, MEM(st + ALGS_INIT_FLAGS + ENC_ID_TYPE9 * 4));
                MEM(st + ALGS_INIT_FLAGS + ENC_ID_TYPE9 * 4) = 0;
            }
            enc_last_h9 = j[2];
        }
        if (j[0] == 9)
        {
            enc_last_src9 = j[6];
            enc_last_pitch9 = j[7] * j[5] / 8;
        }
        log_printf("enc %d ms start #%d cbr %x job %d: type %d %dx%d w3 %d w4 %d bits %d src %08x pitch %d w8 %d%s",
                   get_ms_clock(), k, cbr, i, j[0], j[1], j[2], j[3], j[4], j[5], j[6], j[7], j[8],
                   cropped ? " (cropped)" : "");
    }
    if (cbr == STILL_ENC_CBR)
        return (uint32_t) enc_done_still;
    if (cbr == CODEC_ENC_CBR)
        return (uint32_t) enc_done_codec;
    return cbr;
}

static struct function_hook_patch f_patches_enc[] = {
    {
        .patch_addr = 0xe06662f6, // StartEncode (jobs, n, cbr, ctx)
        .orig_content = {0x2d, 0xe9, 0xff, 0x4f, 0x80, 0x46, 0xdf, 0xf8},
        .target_function_addr = (uint32_t)hook_StartEncode_M6II,
        .description = "dmlog: StartEncode"
    },
};

/* the CRaw trace logger (0xe05465a6, "EncIni:"/"Bit..Dwt.." lines) needs a
 * page of its own; left out until more pages are available */

/* CMOS (sensor register) writes, ImageSequence::Sio0Driver.c: every list
 * Canon sends to the sensor, with the caller (0xE06A5815: the per-frame
 * side list; 0xE06A65DD / 0xE06A53AF: the lists at a LiveView start).  PIO lists are 64-bit frames (hi, lo; register
 * = hi >> 20) up to a pair of 0xFFFFFFFF; DMA data is n halfwords.  To
 * find what changes with ISO: start logging, change ISO. */
/* Canon sends 7 lists every frame (M6II movie LV: callers e06a596d,
 * e06a4ff1, e06a4ffd, e06a5815 (29 frames: gain 0x031 etc.), e06a5829,
 * e06a5897, e068d1b5), so a list is logged only when it differs from the
 * last one from the same caller; each logged line says how many calls
 * came in between. */
#define CMOS_CALLERS 32
#define CMOS_MAX_FRAMES 512
#define CMOS_MAX_LOGGED 3000
static volatile int cmos_calls = 0, cmos_logged = 0;
static struct { uint32_t lr, sum, n, calls; } cmos_seen[CMOS_CALLERS];

/* 1 if this caller's list changed (or is new): log it */
static int cmos_changed(uint32_t lr, const void *data, int bytes)
{
    uint32_t sum = 0x811C9DC5;
    const uint8_t *b = data;
    for (int i = 0; i < bytes; i++)
    {
        sum = (sum ^ b[i]) * 0x01000193;
    }
    int free_i = -1;
    for (int i = 0; i < CMOS_CALLERS; i++)
    {
        if (cmos_seen[i].lr == lr)
        {
            cmos_seen[i].calls++;
            if (cmos_seen[i].sum == sum && cmos_seen[i].n == (uint32_t) bytes)
            {
                return 0;
            }
            cmos_seen[i].sum = sum;
            cmos_seen[i].n = bytes;
            return 1;
        }
        if (!cmos_seen[i].lr && free_i < 0)
        {
            free_i = i;
        }
    }
    if (free_i >= 0)
    {
        cmos_seen[free_i].lr = lr;
        cmos_seen[free_i].sum = sum;
        cmos_seen[free_i].n = bytes;
        cmos_seen[free_i].calls = 1;
    }
    return 1;
}

static int cmos_calls_of(uint32_t lr)
{
    for (int i = 0; i < CMOS_CALLERS; i++)
    {
        if (cmos_seen[i].lr == lr)
        {
            return cmos_seen[i].calls;
        }
    }
    return 0;
}

void dmlog_cmos_pio(const uint32_t *f, uint32_t unused, uint32_t lr)
{
    cmos_calls++;
    if (!is_logging || cmos_logged >= CMOS_MAX_LOGGED)
    {
        return;
    }
    int n = 0;
    while (n < CMOS_MAX_FRAMES && !(f[2 * n] == 0xFFFFFFFF && f[2 * n + 1] == 0xFFFFFFFF))
    {
        n++;
    }
    if (!cmos_changed(lr, f, n * 8))
    {
        return;
    }
    cmos_logged++;
    log_printf("cmos %d ms pio lr %08x: %d frames (call %d from it)", get_ms_clock(), lr, n, cmos_calls_of(lr));
    for (int i = 0; i < n; i += 4)
    {
        char line[120];
        int len = 0;
        for (int k = i; k < n && k < i + 4; k++)
        {
            len += snprintf(line + len, sizeof(line) - len, " %08x:%08x", f[2 * k], f[2 * k + 1]);
        }
        log_printf(" cmos%s", line);
    }
}

void dmlog_cmos_dma(const uint16_t *d, uint32_t n, uint32_t lr)
{
    cmos_calls++;
    if (!is_logging || cmos_logged >= CMOS_MAX_LOGGED)
    {
        return;
    }
    if (!cmos_changed(lr, d, MIN(n, 0x1400) * 2))
    {
        return;
    }
    cmos_logged++;
    log_printf("cmos %d ms dma lr %08x: %d halfwords (call %d from it)", get_ms_clock(), lr, n, cmos_calls_of(lr));
    for (uint32_t i = 0; i < n && i < 0x1400; i += 16)
    {
        char line[100];
        int len = 0;
        for (uint32_t k = i; k < n && k < i + 16; k++)
        {
            len += snprintf(line + len, sizeof(line) - len, " %04x", d[k]);
        }
        log_printf(" cmosd%s", line);
    }
}

static struct function_hook_patch f_patches_cmos[] = {
    {
        .patch_addr = 0xe06a0dee, // cmos_write_pio
        .orig_content = {0x2d, 0xe9, 0xfc, 0x5f, 0x04, 0x46, 0xdf, 0xf8},
        .target_function_addr = (uint32_t)hook_CmosPio_M6II,
        .description = "dmlog: cmos_write_pio"
    },
    {
        .patch_addr = 0xe06a0f6a, // cmos_write_dma
        .orig_content = {0x2d, 0xe9, 0xf8, 0x43, 0x06, 0x46, 0xdf, 0xf8},
        .target_function_addr = (uint32_t)hook_CmosDma_M6II,
        .description = "dmlog: cmos_write_dma"
    },
};

static struct function_hook_patch *f_patches = NULL;
static int f_patch_count = 0;

static struct patch patches[6];
static int num_patches_used = 0;
static uint8_t code_hooks[8 * 6];

/* Property logger: raw bytes of the properties behind the MLV metadata
 * (IDNT, EXPO, LENS, WBAL, RAWC), to work out their D8 layouts.  Canon
 * delivers current values on registration, then every change. */
extern void prop_register_slave(unsigned *list, unsigned count,
                                void *(*handler)(unsigned, void *, void *, unsigned),
                                void *priv, void (*token_handler)(void *));
#define PROP_CLEANUP_M6II ((void *(*)(void *, unsigned)) 0xe058c8dd)  /* _prop_cleanup */

static unsigned meta_props[] = {
    0x00000002, /* CAM_MODEL: name, model id at +32 (IDNT) */
    0x01000006, /* BODY_ID */
    0x80000000, 0x80000001, /* SHOOTING_MODE, _2 */
    0x80000005, 0x80000006, 0x80000007, /* SHUTTER, APERTURE, ISO */
    0x80000008, /* AE */
    0x8000002C, 0x8000002D, 0x8000002E, /* SHUTTER_AUTO, APERTURE_AUTO, ISO_AUTO */
    0x8000003A, /* AE_MODE_MOVIE */
    0x80000039, 0x02050010, /* VIDEO_MODE (two variants) */
    0x80030037, /* FIXED_MOVIE */
    0x80050010, /* BV */
    0x80030010, 0x80030011, 0x80030021, /* LENS (two variants), LENS_NAME */
    0x8003009A, 0x80020037, /* LENS_STATIC_DATA, LENS_DYNAMIC_DATA */
    0x80050000, 0x8008003C, 0x80050005, /* LV_LENS, LV_LENS_D67, LV_LENS_STABILIZE */
    0x80050018, 0x80050019, /* WB_MODE_LV, WB_KELVIN_LV */
    0x8000000D, 0x8000000E, /* WB_MODE_PH, WB_KELVIN_PH */
    0x80000010, 0x80000011, /* WBS_GM, WBS_BA */
    0x02020000, /* CUSTOM_WB (gains) */
    0x80050007, /* LV_AFFRAME (RAWC sensor res) */
};
static void *prop_token = NULL;
static int props_registered = 0;

static void prop_token_handler(void *token)
{
    prop_token = token;
}

static void *meta_prop_handler(unsigned prop, void *priv, void *buf, unsigned len)
{
    if (is_logging)
    {
        log_printf("prop %08x len %d:", prop, len);
        const uint8_t *b = buf;
        int n = MIN((int)len, 96);
        char line[100];
        for (int i = 0; i < n; i += 16)
        {
            int k = 0;
            for (int j = i; j < MIN(i + 16, n); j++)
                k += snprintf(line + k, sizeof(line) - k, "%02x%s", b[j], (j % 4 == 3) ? " " : "");
            log_printf("  +%02x: %s", i, line);
        }
    }
    return PROP_CLEANUP_M6II(prop_token, prop);
}

static void register_meta_props()
{
    if (props_registered)
        return;
    props_registered = 1;
    /* M6II LVAE: ML ignores PROP_APERTURE while CONTROL_BV != 0 */
    log_printf("CONTROL_BV (LVAE 0x5167c+0x28) = %x", *(volatile uint16_t *)(0x5167c + 0x28));
    prop_register_slave(meta_props, COUNT(meta_props), meta_prop_handler, NULL, prop_token_handler);
}

/* Property spy: which properties change when a Canon setting does (e.g.
 * RAW burst, pre-shooting).  Registers the wide ID list of ML's old debug
 * spy (src/debug.c) once and logs every value delivered: first the current
 * ones, then each change.  Needs no hooks; registration can't be undone. */
static unsigned *spy_props = NULL;
static void *spy_token = NULL;
static int spy_on = 0;
static volatile int spy_count = 0;

static void spy_token_handler(void *token)
{
    spy_token = token;
}

static void *spy_prop_handler(unsigned prop, void *priv, void *buf, unsigned len)
{
    spy_count++;
    if (is_logging)
    {
        const uint8_t *b = buf;
        char hex[3 * 24 + 1];
        int n = MIN((int) len, 24), k = 0;
        for (int i = 0; i < n; i++)
        {
            k += snprintf(hex + k, sizeof(hex) - k, "%02x%s", b[i], (i % 4 == 3) ? " " : "");
        }
        log_printf("propspy %d ms %08x len %d: %s", get_ms_clock(), prop, len, hex);
    }
    return PROP_CLEANUP_M6II(spy_token, prop);
}

static void spy_start_task(void)
{
    if (spy_on)
    {
        NotifyBox(3000, "Property spy already on (%d values)", spy_count);
        return;
    }
    if (!is_logging)
    {
        NotifyBox(3000, "Turn the Canon debug log on first");
        return;
    }
    static const unsigned classes[] = {0x2, 0x80, 0xe, 0x5, 0x4, 0x1, 0x0};
    int n = 0, max = COUNT(classes) * 11 * 0x50;
    spy_props = malloc(max * sizeof(unsigned));
    if (!spy_props)
    {
        return;
    }
    for (int i = 0; i < COUNT(classes); i++)
        for (int j = 0; j <= 0xA; j++)
            for (int k = 0; k < 0x50; k++)
                spy_props[n++] = classes[i] << 24 | j << 16 | k;
    log_printf("propspy: registering %d properties", n);
    spy_on = 1;
    prop_register_slave(spy_props, n, spy_prop_handler, NULL, spy_token_handler);
    log_printf("propspy: registered, %d values so far", spy_count);
    NotifyBox(3000, "Property spy on");
}

/* Pre-rec test: start RAW burst pre-shooting without a half-press, as
 * Canon's own components do it (seen in Canon's debug log):
 * 1. PoCom (Canon's message bus, send 0xE054AE80(cmd, data, done_cbr, ctx))
 *    command 0x30033 to LiveView (channel 3): "PO: MechalessStart" sets
 *    RAW burst up; done_cbr runs at "PO:Comp:51";
 * 2. GUI command 0xCD (GUICMD_MECHALESS_START_PREREC): pre-shooting starts.
 * Stop as a release does: GUI 0xCE (STOP_PREREC), then PoCom 0x30034
 * ("PO: MechalessStop").  The menu argument goes with the GUI commands. */
static CONFIG_INT("dmlog.prerec_arg", prerec_arg, 0);
#define POCOM_SEND ((void (*)(uint32_t cmd, void *data, void (*cbr)(), void *ctx)) 0xe054ae81)
#define PO_MECHALESS_START 0x30033
#define PO_MECHALESS_STOP  0x30034
static volatile int pocom_done = 0;
static volatile uint32_t pocom_args[3];

static void pocom_done_cbr(uint32_t a0, uint32_t a1, uint32_t a2)
{
    pocom_args[0] = a0; pocom_args[1] = a1; pocom_args[2] = a2;
    pocom_done++;
}

static int pocom_wait(int before, int ms)
{
    for (int t = 0; t < ms && pocom_done == before; t += 10)
    {
        msleep(10);
    }
    return pocom_done != before;
}

/* each step on screen (bmp_printf draws even when Canon's GUI is stuck)
 * and in the log, so a freeze shows how far the test got */
static void prerec_step(const char *what)
{
    log_printf("prerec test %d ms: %s", get_ms_clock(), what);
    bmp_printf(FONT_LARGE, 0, 60, "Pre-rec: %s          ", what);
}

extern WEAK_FUNC(ret_0) int preshoot_get_setting(int prerec);

/* the GUI's copies of the RAW burst and pre-shooting settings (its START_
 * PREREC handler does nothing while RAW burst reads 0 there) */
#define GUI_STATE_PTR 0x879c
#define GUI_MECHALESS(off) MEM(MEM(GUI_STATE_PTR + 4) + (off))

static void prerec_test_task(void)
{
    char msg[80];
    int one = 1, zero = 0;
    snprintf(msg, sizeof(msg), "0 RAW burst + pre-shooting on (GUI %d %d, prop %d %d)",
             GUI_MECHALESS(0x1fd8), GUI_MECHALESS(0x1fdc), preshoot_get_setting(0), preshoot_get_setting(1));
    prerec_step(msg);
    prop_request_change(PROP_MECHALESS, &one, 4);
    msleep(1500);
    prop_request_change(PROP_MECHALESS_PRE_REC, &one, 4);
    msleep(1500);
    snprintf(msg, sizeof(msg), "0 settings now (GUI %d %d, prop %d %d)", GUI_MECHALESS(0x1fd8), GUI_MECHALESS(0x1fdc),
             preshoot_get_setting(0), preshoot_get_setting(1));
    prerec_step(msg);
    msleep(500);
    int n = pocom_done;
    prerec_step("1 PoCom 30033 (MechalessStart)");
    POCOM_SEND(PO_MECHALESS_START, NULL, (void (*)()) pocom_done_cbr, (void *) 0x5051);
    prerec_step("2 sent, waiting for completion");
    int ok = pocom_wait(n, 3000);
    snprintf(msg, sizeof(msg), "3 %s (%x %x %x)", ok ? "completed" : "NO completion in 3 s",
             pocom_args[0], pocom_args[1], pocom_args[2]);
    prerec_step(msg);
    msleep(100);
    snprintf(msg, sizeof(msg), "4 GUI START_PREREC (0xCD, %d)", prerec_arg);
    prerec_step(msg);
    GUI_Control(0xCD, prerec_arg, 0, 0);
    msleep(4000);
    prerec_step("5 GUI STOP_PREREC (0xCE)");
    GUI_Control(0xCE, prerec_arg, 0, 0);
    msleep(1500);
    n = pocom_done;
    prerec_step("6 PoCom 30034 (MechalessStop)");
    POCOM_SEND(PO_MECHALESS_STOP, NULL, (void (*)()) pocom_done_cbr, (void *) 0x5052);
    ok = pocom_wait(n, 3000);
    prerec_step(ok ? "7 stop completed" : "7 NO stop completion");
    msleep(1000);
    prop_request_change(PROP_MECHALESS_PRE_REC, &zero, 4);
    msleep(1000);
    prop_request_change(PROP_MECHALESS, &zero, 4);
    msleep(1500);
    snprintf(msg, sizeof(msg), "8 settings off, done (GUI %d %d)", GUI_MECHALESS(0x1fd8), GUI_MECHALESS(0x1fdc));
    prerec_step(msg);
}

/* Mechaless mode test: only Canon's mode switch, for movie mode.
 * PoCom MechalessStart -> Mechaless(3): LiveView slot (lv +0xAC8) := 11,
 * mode change 12 (0xE04F80B8 -> 0xE04F7B1A), LibCond Mc copies the slot's
 * 0x68-byte entry (lv + slot * 0x68 + 0x580; ROM template 0xE096C128:
 * drive mode 8, sensor slot 4, fps idx 2) into the DTS master.  Nothing
 * checks for photo mode on the way (static), so in movie LV this may
 * switch to the 1:1 drive mode as in RAW burst.  Stop: Mechaless(5)
 * restores the slot (2 = movie when lv +0xACC is 2), mode change 13.
 * No settings, no GUI commands.  Logs slot, drive mode and EDMAC ch 1
 * (the LV raw writer; full 5376 x 3560 frames in a 5-buffer ring in RAW
 * burst) every 250 ms. */
#define LV_OBJ          MEM(0x99E0)
#define LV_SLOT(lv)     MEM((lv) + 0xAC8)
#define LV_ENTRY(lv, off) MEM((lv) + LV_SLOT(lv) * 0x68 + (off))

/* the 1:1 frame as in RAW burst (sensor-readout.md): 5376 px at 14 bit,
 * contiguous LSB-first, 3560 rows */
#define MCL_PITCH 9408
#define MCL_ROWS  3560

static void mechaless_state(const char *what)
{
    char msg[100];
    uint32_t lv_obj = LV_OBJ;
    uint32_t len = edmac_get_length(1);
    snprintf(msg, sizeof(msg), "%s: %s slot %d (S %d A %d), drive %d, ch1 %x len %08x",
             what, is_movie_mode() ? "movie" : "photo",
             lv_obj ? (int) LV_SLOT(lv_obj) : -1, lv_obj ? (int) LV_ENTRY(lv_obj, 0x588) : -1,
             lv_obj ? (int) LV_ENTRY(lv_obj, 0x59C) : -1, MEM(DTS_MASTER_DRIVE_MODE),
             edmac_get_address(1), len);
    prerec_step(msg);
}

/* Defect correction state (rom-inventory.md, "How defect correction
 * works"), read only: DefmarkNavigator Tune / Rasen path off bits
 * (0xD0411344 / 0xD0411A24, 0 = on), threshold enables and pairs, the
 * map read channels 42 / 40 and their connections, DEFOE's registers and
 * mark source (0xD0414080: 5 in LiveView, 1 in the factory pass), and
 * Canon's navigator state at 0x10480 (+0x18 mode, +0x20 Rasen, +0x28
 * active). */
static char defect_text[4096];
static int defect_len;

/* to the log and to defect_text, saved as ML/LOGS/MCLREGS.TXT at the end
 * of the test: the log is a ring, and its test lines get overwritten
 * when logging runs on for long */
static void defect_step(const char *msg)
{
    prerec_step(msg);
    int n = snprintf(defect_text + defect_len, sizeof(defect_text) - defect_len, "%s\n", msg);
    if (n > 0 && defect_len + n < (int) sizeof(defect_text))
    {
        defect_len += n;
    }
}

static void defect_regs(const char *what)
{
    char msg[160];
    uint32_t nav = 0x10480;
    snprintf(msg, sizeof(msg), "%s: defmark off T %x R %x, en %x, thr %08x %08x %08x %08x / %08x %08x %08x %08x",
             what, MEM(0xD0411344), MEM(0xD0411A24), MEM(0xD0411318),
             MEM(0xD0411304), MEM(0xD0411308), MEM(0xD041130C), MEM(0xD0411310),
             MEM(0xD041131C), MEM(0xD0411320), MEM(0xD0411324), MEM(0xD0411328));
    defect_step(msg);
    snprintf(msg, sizeof(msg), "%s: map ch42 %x len %x, ch40 %x, conn %x %x; nav mode %d rasen %d active %d",
             what, edmac_get_address(42), edmac_get_length(42), edmac_get_address(40),
             MEM(0xD0423280), MEM(0xD0423288), MEM(nav + 0x18), MEM(nav + 0x20), MEM(nav + 0x28));
    defect_step(msg);
    snprintf(msg, sizeof(msg), "%s: DEFOE %x %x %x %x %x %x %x %x %x %x %x %x, src 78 %x 7c %x 80 %x",
             what, MEM(0xD0413654), MEM(0xD0413658), MEM(0xD041365C), MEM(0xD0413660),
             MEM(0xD0413664), MEM(0xD0413668), MEM(0xD041366C), MEM(0xD0413670),
             MEM(0xD0413674), MEM(0xD0413678), MEM(0xD041367C), MEM(0xD0413680),
             MEM(0xD0414078), MEM(0xD041407C), MEM(0xD0414080));
    defect_step(msg);
}

/* Mem1 (HEAD tap) in each state, from Canon's RAM state at 0x10970 only
 * ({type, SAP sel, SANTA sel, conn, ...}; +0x1C rows / +0x58 buffer are
 * set while it runs): reading EDMAC ch 3's or HEAD's
 * registers froze the camera in this test - the Mem1 block is likely
 * unpowered in the 1:1 mode */
static void mem1_regs(const char *what)
{
    char msg[200];
    const volatile uint32_t *m = (const volatile uint32_t *) 0x10970;
    int n = snprintf(msg, sizeof(msg), "%s: Mem1 state", what);
    for (int i = 0; i < 24 && n < (int) sizeof(msg) - 10; i++)
    {
        n += snprintf(msg + n, sizeof(msg) - n, " %x", m[i]);
    }
    defect_step(msg);
}

static void mechaless_test_task(void)
{
    if (!lv || !LV_OBJ)
    {
        NotifyBox(3000, "LiveView first");
        return;
    }
    defect_len = 0;
    mechaless_state("0 before");
    defect_regs("0 before");
    mem1_regs("0 before");
    msleep(500);
    int n = pocom_done;
    prerec_step("1 PoCom 30033 (MechalessStart)");
    POCOM_SEND(PO_MECHALESS_START, NULL, (void (*)()) pocom_done_cbr, (void *) 0x5053);
    int ok = pocom_wait(n, 3000);
    char msg[80];
    snprintf(msg, sizeof(msg), "2 %s (%x %x %x)", ok ? "completed" : "NO completion in 3 s",
             pocom_args[0], pocom_args[1], pocom_args[2]);
    prerec_step(msg);
    uint32_t last = 0;
    int changes = 0;
    /* is the buffer refreshed? 64 bytes from the middle of row 1780
     * (sensor noise differs every frame) */
    static uint8_t probe[64];
    int refreshed = 0;
    for (int t = 0; t < 5000; t += 250)
    {
        msleep(250);
        uint32_t a = edmac_get_address(1);
        changes += (a != last);
        last = a;
        uint8_t *row = (uint8_t *) UNCACHEABLE(a) + MCL_PITCH * 1780 + MCL_PITCH / 2;
        refreshed += (t > 0 && memcmp(probe, row, sizeof(probe)) != 0);
        memcpy(probe, row, sizeof(probe));
        mechaless_state("3 running");
        if (t == 2000)
        {
            defect_regs("3 running");
            mem1_regs("3 running");
        }
    }
    snprintf(msg, sizeof(msg), "4 ch1 address changed %d times, row 1780 changed in %d of 19 samples",
             changes, refreshed);
    prerec_step(msg);

    /* one frame to the card (the buffer keeps being written meanwhile:
     * rows later in the file may be from the next frame) */
    uint32_t a = edmac_get_address(1);
    prerec_step("4 saving ML/LOGS/MCLFRAME.RAW");
    FILE *f = FIO_CreateFile("ML/LOGS/MCLFRAME.RAW");
    if (f)
    {
        int t0 = get_ms_clock();
        int written = FIO_WriteFile(f, (void *) UNCACHEABLE(a), MCL_PITCH * MCL_ROWS);
        FIO_CloseFile(f);
        snprintf(msg, sizeof(msg), "4 saved %d bytes from %x in %d ms (%d x %d, pitch %d)",
                 written, a, get_ms_clock() - t0, MCL_PITCH * 8 / 14, MCL_ROWS, MCL_PITCH);
    }
    else
    {
        snprintf(msg, sizeof(msg), "4 couldn't create ML/LOGS/MCLFRAME.RAW");
    }
    prerec_step(msg);

    n = pocom_done;
    prerec_step("5 PoCom 30034 (MechalessStop)");
    POCOM_SEND(PO_MECHALESS_STOP, NULL, (void (*)()) pocom_done_cbr, (void *) 0x5054);
    ok = pocom_wait(n, 3000);
    prerec_step(ok ? "6 stop completed" : "6 NO stop completion");
    msleep(1500);
    mechaless_state("7 after");
    defect_regs("7 after");
    mem1_regs("7 after");
    FILE *rf = FIO_CreateFile("ML/LOGS/MCLREGS.TXT");
    if (rf)
    {
        FIO_WriteFile(rf, defect_text, defect_len);
        FIO_CloseFile(rf);
    }
    prerec_step(rf ? "8 saved ML/LOGS/MCLREGS.TXT" : "8 couldn't create ML/LOGS/MCLREGS.TXT");
}

static MENU_UPDATE_FUNC(spy_update)
{
    MENU_SET_VALUE(spy_on ? "ON" : "OFF");
    if (spy_on)
    {
        MENU_SET_RINFO("%d values", spy_count);
    }
}

static int start_logging()
{
    if (!is_camera("M6II", "1.1.1"))
    {
        NotifyBox(3000, "dmlog: hooks are for M6II 1.1.1 only");
        return 0;
    }

    step("allocating");
    if (!log_buf)
    {
        /* as much as we can get, up to 8 MB */
        for (log_size = 8 << 20; log_size >= (512 << 10); log_size /= 2)
        {
            log_buf = malloc(log_size);
            if (log_buf)
                break;
        }
        if (!log_buf)
        {
            NotifyBox(3000, "dmlog: no memory for the log");
            return 0;
        }
    }
    log_len = 0;
    log_dropped = 0;
    log_wrap = 0;
    /* let a PC read the log over USB (ptp.c mailbox) */
    ptp_mailbox_set(0, "dmlog", log_buf, (volatile uint32_t *)&log_len);
    ptp_mailbox_set(3, "encframe", &enc_frame, &enc_frame_len);
    memset(engio_reg, 0, sizeof(engio_reg));
    engio_full = 0;
    engio_last_slot = 0xFFFFFFFF;
    engio_ev_n = 0;
    engio_flushed = 0;
    engio_ev_dropped = 0;
    engio_snap_n = 0;
    if (hook_group == HOOKS_ENGIO && !engio_ev)
    {
        engio_ev = malloc(ENGIO_EVENTS * sizeof(*engio_ev));
        engio_snaps = malloc(ENGIO_SNAPS * ENGIO_SNAP_SIZE);
        if (!engio_ev || !engio_snaps)
        {
            NotifyBox(3000, "dmlog: no memory for engio events");
            return 0;
        }
    }
    step("first log line");
    log_printf("dmlog: logging to %d KB buffer", log_size >> 10);
    step("converting hooks");

    switch (hook_group)
    {
        case HOOKS_DEBUGMSG: f_patches = f_patches_debugmsg; f_patch_count = COUNT(f_patches_debugmsg); break;
        case HOOKS_CRAW:     f_patches = f_patches_craw;     f_patch_count = COUNT(f_patches_craw);     break;
        case HOOKS_ENGIO:    f_patches = f_patches_engio;    f_patch_count = COUNT(f_patches_engio);    break;
        case HOOKS_EDMAC:    f_patches = f_patches_edmac;    f_patch_count = COUNT(f_patches_edmac);    break;
        case HOOKS_ENC:      f_patches = f_patches_enc;      f_patch_count = COUNT(f_patches_enc);      break;
        case HOOKS_CMOS:     f_patches = f_patches_cmos;     f_patch_count = COUNT(f_patches_cmos);     cmos_calls = cmos_logged = 0; memset(cmos_seen, 0, sizeof(cmos_seen)); break;
        default:             f_patches = NULL;               f_patch_count = 0;                         break;
    }
    num_patches_used = f_patch_count;
    if (f_patch_count)
    {
        /* a remap page for the group (ML core adds one from the heap if
         * the static one is taken: d8_modes' tests) */
        mmu_reserve_remap_pages(1);
    }
    for (int i = 0; i < num_patches_used; i++)
    {
        if (convert_f_patch_to_patch(&f_patches[i], &patches[i], &code_hooks[8 * i]))
        {
            NotifyBox(3000, "dmlog: bad hook %d", i);
            return 0;
        }
    }
    step("applying patches");
    int err = num_patches_used ? apply_patches(patches, num_patches_used) : 0;
    if (err)
    {
        NotifyBox(3000, "dmlog: patching failed (%x)", err);
        return 0;
    }
    log_printf("dmlog: hook group %d, %d hooks installed", hook_group, num_patches_used);
    /* is_logging is set by the caller after we return; values delivered
     * during registration must be logged, so set it here already */
    is_logging = 1;
    step("registering properties");
    register_meta_props();
    step("running");
    return 1;
}

static void stop_logging()
{
    for (int i = 0; i < num_patches_used; i++)
    {
        unpatch_memory(f_patches[i].patch_addr);
    }
    /* let anything still inside our functions finish */
    msleep(100);
    if (engio_last_slot != 0xFFFFFFFF)
    {
        engio_snapshot(engio_last_slot);
    }
    engio_flush();
    engio_save_snapshots();

    log_printf("dmlog: stopped, %d bytes, wrapped %d times, engio table full %d times",
               log_len, log_dropped, engio_full);

    char filename[100];
    get_numbered_file_name("ML/LOGS/DMLOG%02d.LOG", 99, filename, sizeof(filename));
    FILE *f = FIO_CreateFile(filename);
    if (f)
    {
        if (log_wrap > log_len)
        {
            /* older part first (its first line may be cut) */
            FIO_WriteFile(f, log_buf + log_len, log_wrap - log_len);
        }
        FIO_WriteFile(f, log_buf, log_len);
        FIO_CloseFile(f);
        NotifyBox(5000, "Saved %s (%d KB)", filename, log_len >> 10);
    }
    else
    {
        NotifyBox(5000, "dmlog: could not create %s", filename);
    }
}

/* Canon asserted (ERR70 dialog): ML keeps running, so save what led up to
 * it - the menus can't be reached any more */
extern WEAK_FUNC(ret_0) void ErrForCamera_handler();
static void dmlog_save_on_error_task()
{
    if (is_logging)
    {
        is_logging = 0;
        log_printf("dmlog: Canon error dialog (ERR70?), saving");
        stop_logging();
    }
}

/* Autosave: a freeze (no ERR70) loses the log with the battery, so while
 * logging it is written to the card every 3 s, and every 0.2 s while
 * LiveView is off (Canon menu, LiveView start): the 1:1 test hung the
 * whole camera within 3 s of closing the menu.  Alternates between
 * ML/LOGS/DMLAUTO0.LOG and DMLAUTO1.LOG, so a write cut off by a hang
 * leaves the previous one; each starts with its sequence number. */
static CONFIG_INT("dmlog.autosave", autosave, 0);
static volatile int autosave_running = 0;
static int autosave_count = 0;

static void dmlog_autosave_once()
{
    char name[32];
    snprintf(name, sizeof(name), "ML/LOGS/DMLAUTO%d.LOG", autosave_count & 1);
    FILE *f = FIO_CreateFile(name);
    if (f)
    {
        int len = log_len, wrap = log_wrap;
        char line[64];
        int n = snprintf(line, sizeof(line), "dmlog autosave %d, %d bytes\n", ++autosave_count, len);
        FIO_WriteFile(f, line, n);
        if (wrap > len)
        {
            FIO_WriteFile(f, log_buf + len, wrap - len);
        }
        FIO_WriteFile(f, log_buf, len);
        FIO_CloseFile(f);
    }
}

/* Stream save: every 50 ms, what was logged since the last pass is
 * appended to ML/LOGS/DMLSTRM.LOG (opened and closed each time, so the
 * card holds it if the camera freezes).  The 64 kB fast saves that came
 * before kept missing the last LiveView-start writes before a freeze
 * (hundreds of engio events in a fraction of a second).  The file
 * starts empty when logging starts. */
#define STREAM_FILE "ML/LOGS/DMLSTRM.LOG"
static int stream_pos = 0;              /* ring offset written up to */
static int stream_wraps = 0;            /* log_dropped at that point */

static void dmlog_stream_reset()
{
    FILE *f = FIO_CreateFile(STREAM_FILE);
    if (f)
    {
        FIO_CloseFile(f);
    }
    stream_pos = 0;
    stream_wraps = log_dropped;
}

static void dmlog_stream_once()
{
    int len = log_len, wrap = log_wrap, wraps = log_dropped;
    if (wraps == stream_wraps && len == stream_pos)
    {
        return;
    }
    FILE *f = FIO_CreateFileOrAppend(STREAM_FILE);
    if (!f)
    {
        return;
    }
    if (wraps != stream_wraps)
    {
        /* the ring wrapped: the rest of the old part, then from the top
         * (more than one wrap in 50 ms loses text: noted in the file) */
        if (wraps - stream_wraps > 1)
        {
            char line[48];
            int n = snprintf(line, sizeof(line), "\n[dmlog stream: %d wraps lost]\n", wraps - stream_wraps - 1);
            FIO_WriteFile(f, line, n);
        }
        else if (wrap > stream_pos)
        {
            FIO_WriteFile(f, log_buf + stream_pos, wrap - stream_pos);
        }
        stream_pos = 0;
    }
    if (len > stream_pos)
    {
        FIO_WriteFile(f, log_buf + stream_pos, len - stream_pos);
    }
    FIO_CloseFile(f);
    stream_pos = len;
    stream_wraps = wraps;
}

static void dmlog_autosave_task()
{
    int ms = 0;
    dmlog_stream_reset();
    while (autosave && is_logging)
    {
        msleep(50);
        ms += 50;
        if (hook_group == HOOKS_ENGIO)
        {
            engio_flush_new();
        }
        dmlog_stream_once();
        /* every 0.25 s: ML's lv flag stays on while Canon's menu is open
         * here, so "fast while LiveView is off" never triggered */
        if (ms >= 250)
        {
            ms = 0;
            dmlog_autosave_once();
        }
    }
    autosave_running = 0;
}

/* Stop after LV start: the 512 KB ring holds only a few seconds of Canon's
 * messages, so a LiveView start is overwritten unless the log stops soon
 * after it.  Stops (and saves) 2 s after LiveView comes back on. */
static CONFIG_INT("dmlog.stop_after_lv", stop_after_lv, 0);
static void dmlog_stop_task();

static unsigned int dmlog_seconds_cbr(unsigned int ctx)
{
    static int sec = 0;
    static int lv_was = 1, lv_on_for = -1;
    if (stop_after_lv && is_logging)
    {
        if (!lv)
        {
            lv_on_for = 0;
        }
        else if (!lv_was && lv_on_for >= 0)
        {
            lv_on_for = 1;
        }
        else if (lv_on_for > 0 && ++lv_on_for > 1)
        {
            lv_on_for = -1;
            task_create("dmlog_stop", 0x1e, 0x1000, dmlog_stop_task, 0);
        }
        lv_was = lv;
    }
    (void) sec;
    if (is_logging && hook_group == HOOKS_EDMAC && num_patches_used)
    {
        edmac_start_sites();
        edmac_summary();
    }
    if (autosave && is_logging && log_buf && !autosave_running)
    {
        autosave_running = 1;
        task_create("dmlog_auto", 0x11, 0x1000, dmlog_autosave_task, 0);
    }
    if (is_logging && (void *) ErrForCamera_handler != (void *) ret_0
        && get_current_dialog_handler() == (void *) &ErrForCamera_handler)
    {
        task_create("dmlog_err", 0x1e, 0x1000, dmlog_save_on_error_task, 0);
    }
    return 0;
}

static void dmlog_stop_task()
{
    if (is_logging)
    {
        is_logging = 0;
        log_printf("dmlog: LiveView back on, stopping");
        stop_logging();
    }
}

static void dmlog_toggle_task()
{
    if (is_logging)
    {
        is_logging = 0;
        stop_logging();
    }
    else
    {
        is_logging = start_logging();
    }
}

/* SRM 1stJob test (M6II 1.1.1); on this camera it gives nothing, so ML
 * uses Canon's movie recorder memory instead (movrec_mem.c).  Candidates:
 * 0xe0533f4e(cbr, ctx): RscMgr message type 0xa1; Canon's caller waits
 *   5 s and prints "SRM_AllocateMemoryResourceFor1stJob failed"
 * 0xe0536354(buf, cbr, ctx): free
 * The job-memory handlers seen so far call cbr(ctx, buf, 0), i.e. no size,
 * and assert (crash) if the pool is empty, so this asks for two buffers
 * only, in photo mode, and logs what the callback really gets. */
typedef void (*srm_cbr_t)(void *ctx, void *buf, uint32_t size);
#define SRM_ALLOC_1STJOB ((void (*)(srm_cbr_t, void *)) 0xe0533f4f)
#define SRM_FREE_1STJOB  ((void (*)(void *, void *, void *)) 0xe0536355)

static struct srm_result
{
    void *buf;
    uint32_t size;
    int called;
} srm_results[2];
static struct semaphore *srm_sem = NULL;

static void srm_test_cbr(void *ctx, void *buf, uint32_t size)
{
    struct srm_result *r = ctx;
    r->buf = buf;
    r->size = size;
    r->called = 1;
    log_printf("srm_test_cbr: task %s ctx %x buf %x size %x",
               get_current_task_name(), ctx, buf, size);
    give_semaphore(srm_sem);
}

static void srm_test_task()
{
    if (!is_camera("M6II", "1.1.1"))
    {
        NotifyBox(3000, "SRM test: M6II 1.1.1 only");
        return;
    }
    /* mirrorless: LiveView is always on.  Canon allocates job memory for
     * stills taken from photo-mode LiveView, so that's where we test;
     * movie mode keeps more memory for itself (200D: "NG AllocMem1") */
    if (is_movie_mode() || RECORDING)
    {
        NotifyBox(3000, "SRM test: switch to a photo mode first");
        return;
    }
    if (!srm_sem)
        srm_sem = create_named_semaphore("srm_test", 0);

    memset(srm_results, 0, sizeof(srm_results));
    int got = 0;
    for (int i = 0; i < COUNT(srm_results); i++)
    {
        log_printf("srm_test: alloc %d", i);
        SRM_ALLOC_1STJOB(srm_test_cbr, &srm_results[i]);
        if (take_semaphore(srm_sem, 5000))
        {
            log_printf("srm_test: alloc %d timed out", i);
            break;
        }
        got++;
    }

    if (got == 2)
    {
        log_printf("srm_test: buffer distance %x",
                   (uint32_t)srm_results[1].buf - (uint32_t)srm_results[0].buf);
    }
    NotifyBox(10000, "SRM: %d buffers\n%x size %x\n%x size %x", got,
              srm_results[0].buf, srm_results[0].size,
              srm_results[1].buf, srm_results[1].size);
    msleep(1000);

    for (int i = 0; i < got; i++)
    {
        log_printf("srm_test: free %x", srm_results[i].buf);
        SRM_FREE_1STJOB(srm_results[i].buf, 0, 0);
    }
    log_printf("srm_test: done");
}

/* CRX encode test (M6II 1.1.1), step 1:
 * drive Canon's CR3 encoder ourselves through AlgsMgr's StartEncode, with
 * one job copied from what Canon logs for the reduced (1624x1080) raw of a
 * still.  Input: the reduced raw Canon leaves at 0x80000000 after a photo
 * (known-good, 14-bit, pitch 1632 px), so this only tests whether we can
 * run a job; LV frames come next.  Results go to the log; Canon's own
 * AlgsMgr messages show buffers and sizes with the "Debug messages" hooks. */
#define ALGS_STATE_PTR      0x5BBC          /* AlgsMgr state, asserted != 0 */
#define ALGS_START_ENCODE   0xE06662F7      /* (jobs, n <= 3, cbr, ctx) */
#define ALGS_FREE_ALL_ENC   0xE06668F9      /* () posts event 7 */
#define ALGS_ENC_INITED     0x138           /* state: per-id EncInit done flags */
#define ALGS_FREE_ENC       0xE0666873      /* (handle, 0, 0) posts event 4 */

/* give a job's encode buffers back the way Canon's CR3 writer does; once
 * all are free, AlgsMgr returns its SRM memory ("Free AlgsBuf") */
static void crx_release(uint32_t handle)
{
    if (handle && ram_ptr(handle))
    {
        void (*free_enc)(uint32_t, uint32_t, uint32_t) = (void *) ALGS_FREE_ENC;
        free_enc(handle, 0, 0);
        log_printf("crx: FreeEncBuffer %x posted", handle);
    }
    else
    {
        void (*free_all)(void) = (void *) ALGS_FREE_ALL_ENC;
        free_all();
        log_printf("crx: FreeAllEncBuffer posted");
    }
}
#define CRX_TEST_SRC        0x80000000      /* Canon's reduced raw (Tw) */

static struct semaphore *crx_sem = NULL;
static volatile uint32_t crx_cbr_args[4];
static volatile int crx_cbr_called = 0;
static int crx_cbr_quiet = 0;

static void crx_test_cbr(uint32_t a0, uint32_t a1, uint32_t a2, uint32_t a3)
{
    crx_cbr_args[0] = a0;
    crx_cbr_args[1] = a1;
    crx_cbr_args[2] = a2;
    crx_cbr_args[3] = a3;
    crx_cbr_called++;
    if (!crx_cbr_quiet)
    {
        log_printf("crx_test_cbr: task %s args %x %x %x %x",
                   get_current_task_name(), a0, a1, a2, a3);
        if (ram_ptr(a1))
        {
            log_hex("crx_test_cbr result", 0, (void *) a1, 0x40);
        }
    }
    give_semaphore(crx_sem);
}

static void crx_test_task()
{
    if (!is_camera("M6II", "1.1.1"))
    {
        NotifyBox(3000, "CRX test: M6II 1.1.1 only");
        return;
    }
    if (is_movie_mode() || RECORDING)
    {
        NotifyBox(3000, "CRX test: switch to a photo mode first");
        return;
    }
    if (!MEM(ALGS_STATE_PTR))
    {
        NotifyBox(3000, "CRX test: AlgsMgr not initialised");
        return;
    }
    if (!crx_sem)
        crx_sem = create_named_semaphore("crx_test", 0);

    /* the input should look like a 14-bit raw: sample some pixels */
    uint32_t *src = (uint32_t *) CRX_TEST_SRC;
    log_printf("crx_test: src %x words %x %x %x %x", CRX_TEST_SRC, src[0], src[1], src[1000], src[100000]);

    /* job record, 0x3C bytes: [0] encoder id, [1] width, [2] height, [3]
     * start x, [5] bits, [6] source, [7] pitch in pixels, [8] 1 */
    static uint32_t job[15];
    memset(job, 0, sizeof(job));
    job[0] = 1;             /* encoder id: 1 = reduced raw, lossless */
    job[1] = 1624;          /* width */
    job[2] = 1080;          /* height */
    job[3] = 8;             /* as Canon (X offset base?) */
    job[4] = 3;             /* as Canon */
    job[5] = 14;            /* bits */
    job[6] = CRX_TEST_SRC;  /* source */
    job[7] = 1632;          /* pitch, pixels */
    job[8] = 1;             /* Canon: 1 for the first job */
    log_hex("crx_test job", 1, job, sizeof(job));

    crx_cbr_called = 0;
    uint32_t (*start_encode)(void *, uint32_t, void *, void *) = (void *) ALGS_START_ENCODE;
    int t0 = get_ms_clock();
    uint32_t ret = start_encode(job, 1, crx_test_cbr, (void *) 0x12345678);
    log_printf("crx_test: StartEncode returned %x", ret);

    int timeout = take_semaphore(crx_sem, 10000);
    int dt = get_ms_clock() - t0;
    log_printf("crx_test: %s after %d ms, cbr called %d times",
               timeout ? "TIMEOUT" : "done", dt, crx_cbr_called);

    /* let Canon's result messages arrive, then give back the buffers
     * (Canon's CR3 writer normally does this; the data stays in memory) */
    msleep(1000);
    crx_release(ram_ptr(crx_cbr_args[1]) ? MEM(crx_cbr_args[1]) : 0);

    NotifyBox(10000, "CRX test: %s, %d ms\ncbr %x %x %x %x",
              timeout ? "TIMEOUT" : "done", dt,
              crx_cbr_args[0], crx_cbr_args[1], crx_cbr_args[2], crx_cbr_args[3]);
}

/* CRX step 2: encode one LiveView raw frame.
 * The frame is copied into a buffer with rows padded to 16 px (Canon's
 * pitch rule), at 12-bit as delivered or repacked to 14-bit (LSB-first,
 * values << 2), and encoded as one lossless single-tile job (id 1).
 * Saves ML/LOGS/CRXSnn.RAW (the exact encoder input) and CRXPnn.BIN
 * (header + the 4 compressed planes) for checking on a PC (LibRaw). */
static CONFIG_INT("dmlog.crx.bits", crx_lv_bits14, 0);
static CONFIG_INT("dmlog.crx.lvpitch", crx_lv_lvpitch, 0);

#define CRX_HANDLE_PLANES   0x38    /* {ptr, 0} x 4 */
#define CRX_HANDLE_SIZES    0x6C    /* {size, 0} x 4 */

struct crx_planes_hdr
{
    char magic[4];                  /* "CRXP" */
    uint32_t version;               /* 1 */
    uint32_t width, height, bits, pitch_px;
    uint32_t size[4];
    uint32_t encode_ms;
    uint32_t id;
    uint32_t reserved[2];
};

static void crx_repack_row(const uint8_t *src, int src_bits, uint8_t *dst, int dst_bits, int width)
{
    uint64_t in = 0, out = 0;
    int in_n = 0, out_n = 0;
    const uint32_t mask = (1 << src_bits) - 1;
    for (int x = 0; x < width; x++)
    {
        while (in_n < src_bits)
        {
            in |= (uint64_t)(*src++) << in_n;
            in_n += 8;
        }
        uint32_t v = in & mask;
        in >>= src_bits;
        in_n -= src_bits;
        out |= (uint64_t)(v << (dst_bits - src_bits)) << out_n;
        out_n += dst_bits;
        while (out_n >= 8)
        {
            *dst++ = out & 0xFF;
            out >>= 8;
            out_n -= 8;
        }
    }
    if (out_n)
    {
        *dst++ = out & 0xFF;
    }
}

/* returns the AlgsMgr handle (0 on timeout), *ms = time to the callback;
 * reinit: run the encoder setup with this job's parameters */
static uint32_t crx_encode_job(uint32_t *job, int *ms, int reinit, int quiet)
{
    if (!crx_sem)
        crx_sem = create_named_semaphore("crx_test", 0);

    /* algsmgrEncInit (0xE02ACF30) configures each encoder id only once,
     * flagged at AlgsMgr state + 0x138 + id * 4; a 14-bit job after a
     * 12-bit one ran with the 12-bit setup.  Clear it so ours applies. */
    if (reinit)
    {
        uint32_t *inited = (uint32_t *)(MEM(ALGS_STATE_PTR) + ALGS_ENC_INITED + job[0] * 4);
        log_printf("crx: encoder %d init flag was %d", job[0], *inited);
        *inited = 0;
    }

    crx_cbr_called = 0;
    crx_cbr_args[1] = 0;
    uint32_t (*start_encode)(void *, uint32_t, void *, void *) = (void *) ALGS_START_ENCODE;
    int t0 = get_ms_clock();
    uint32_t ret = start_encode(job, 1, crx_test_cbr, (void *) 0x12345678);
    int timeout = take_semaphore(crx_sem, 10000);
    *ms = get_ms_clock() - t0;
    if (!quiet || timeout || ret)
    {
        log_printf("crx: StartEncode %x, %s after %d ms", ret, timeout ? "TIMEOUT" : "done", *ms);
    }
    if (timeout || !ram_ptr(crx_cbr_args[1]))
    {
        return 0;
    }
    return MEM(crx_cbr_args[1]);    /* result +0 = handle */
}

static void crx_lv_test_task()
{
    if (!is_camera("M6II", "1.1.1"))
    {
        NotifyBox(3000, "CRX test: M6II 1.1.1 only");
        return;
    }
    if (RECORDING || !lv)
    {
        NotifyBox(3000, "CRX test: LiveView, not recording");
        return;
    }
    if (!MEM(ALGS_STATE_PTR))
    {
        NotifyBox(3000, "CRX test: AlgsMgr not initialised");
        return;
    }

    raw_lv_request();
    msleep(500);
    if (!raw_update_params())
    {
        raw_lv_release();
        NotifyBox(3000, "CRX test: no LV raw");
        return;
    }
    int src_bits = raw_info.bits_per_pixel;
    int bits = crx_lv_bits14 ? 14 : src_bits;
    int width = raw_info.width & ~1;
    int height = raw_info.height & ~1;
    /* LV pitch: as Canon's buffer (1928 / 2328 px, not 16 px multiples),
     * to check what encoding straight from the LV buffer gives */
    int pitch_px = crx_lv_lvpitch ? raw_info.pitch * 8 / src_bits : (width + 15) & ~15;
    int row_bytes = pitch_px * bits / 8;
    log_printf("crx_lv: LV raw %dx%d pitch %d, %d-bit at %x -> %d-bit, pitch %d px",
               raw_info.width, raw_info.height, raw_info.pitch, src_bits,
               raw_info.buffer, bits, pitch_px);

    /* the encoder's input DMA needs an aligned source: Canon uses MB
     * boundaries, an address ending in 0x8A4 gave noise */
    uint8_t *buf = fio_malloc(row_bytes * height + 0x1000);
    if (!buf)
    {
        raw_lv_release();
        /* movie mode: mlv_lite keeps the shoot memory while raw video is on */
        NotifyBox(3000, "CRX test: no memory (turn raw video off)");
        return;
    }
    uint8_t *src_buf = UNCACHEABLE((void *)(((uint32_t) buf + 0xFFF) & ~0xFFF));
    memset(src_buf, 0, row_bytes * height);
    for (int y = 0; y < height; y++)
    {
        const uint8_t *row = (uint8_t *) raw_info.buffer + y * raw_info.pitch;
        if (bits == src_bits)
            memcpy(src_buf + y * row_bytes, row, width * bits / 8);
        else
            crx_repack_row(row, src_bits, src_buf + y * row_bytes, bits, width);
    }
    raw_lv_release();

    static uint32_t job[15];
    memset(job, 0, sizeof(job));
    job[0] = 1;                     /* encoder id: reduced raw, lossless */
    job[1] = width;
    job[2] = height;
    job[5] = bits;
    job[6] = (uint32_t) src_buf;
    job[7] = pitch_px;
    job[8] = 1;
    log_hex("crx_lv job", 1, job, sizeof(job));

    int ms = 0;
    uint32_t handle = crx_encode_job(job, &ms, 1, 0);
    msleep(200);

    struct crx_planes_hdr hdr;
    memset(&hdr, 0, sizeof(hdr));
    memcpy(hdr.magic, "CRXP", 4);
    hdr.version = 1;
    hdr.width = width;
    hdr.height = height;
    hdr.bits = bits;
    hdr.pitch_px = pitch_px;
    hdr.encode_ms = ms;
    hdr.id = job[0];
    uint32_t planes[4] = {0};
    if (handle && ram_ptr(handle))
    {
        log_hex("crx_lv handle", 0, (void *) handle, 0x80);
        for (int i = 0; i < 4; i++)
        {
            planes[i] = MEM(handle + CRX_HANDLE_PLANES + 8 * i);
            hdr.size[i] = MEM(handle + CRX_HANDLE_SIZES + 8 * i);
        }
    }
    log_printf("crx_lv: planes %x %x %x %x sizes %x %x %x %x",
               planes[0], planes[1], planes[2], planes[3],
               hdr.size[0], hdr.size[1], hdr.size[2], hdr.size[3]);

    char sname[100], pname[100];
    get_numbered_file_name("ML/LOGS/CRXS%02d.RAW", 99, sname, sizeof(sname));
    FILE *f = FIO_CreateFile(sname);
    if (f)
    {
        FIO_WriteFile(f, src_buf, row_bytes * height);
        FIO_CloseFile(f);
    }
    snprintf(pname, sizeof(pname), "%s", sname);
    memcpy(pname + strlen(pname) - 10, "CRXP", 4);       /* CRXSnn.RAW -> CRXPnn.BIN */
    memcpy(pname + strlen(pname) - 3, "BIN", 3);
    int total = 0;
    f = FIO_CreateFile(pname);
    if (f)
    {
        FIO_WriteFile(f, &hdr, sizeof(hdr));
        for (int i = 0; i < 4; i++)
        {
            /* plane buffers are large (worst case); sizes come from the handle */
            if (planes[i] && hdr.size[i] && hdr.size[i] < 64*1024*1024)
            {
                FIO_WriteFile(f, (void *) planes[i], hdr.size[i]);
                total += hdr.size[i];
            }
        }
        FIO_CloseFile(f);
    }
    fio_free(buf);
    crx_release(handle);

    int raw_bytes = width * height * bits / 8;
    NotifyBox(10000, "CRX LV %dx%d %d-bit: %s, %d ms\n%d -> %d bytes (%d.%02d bpp)\n%s",
              width, height, bits, handle ? "ok" : "FAILED", ms, raw_bytes, total,
              total * 8 / (width * height), (total * 800 / (width * height)) % 100, pname);
}

/* CRX step 3: encoder throughput on live LV frames (photo or movie mode).
 * Encodes CRX_RATE_FRAMES frames back to back, one job at a time, each
 * job's buffers given back with FreeEncBuffer(handle, 0, 0) as Canon's
 * CR3 writer does.  Source: a copy of the current LV frame in a buffer
 * padded to 16 px (copy time measured), or Canon's LV raw buffer itself
 * (pitch = LV pitch, often not a multiple of 16 px). */
#define CRX_RATE_FRAMES     48
static CONFIG_INT("dmlog.crx.rate.direct", crx_rate_direct, 0);

static void crx_rate_test_task()
{
    if (!is_camera("M6II", "1.1.1"))
    {
        NotifyBox(3000, "CRX test: M6II 1.1.1 only");
        return;
    }
    if (RECORDING || !lv)
    {
        NotifyBox(3000, "CRX test: LiveView, not recording");
        return;
    }
    if (!MEM(ALGS_STATE_PTR))
    {
        NotifyBox(3000, "CRX test: AlgsMgr not initialised");
        return;
    }

    raw_lv_request();
    msleep(500);
    if (!raw_update_params())
    {
        raw_lv_release();
        NotifyBox(3000, "CRX test: no LV raw");
        return;
    }
    int bits = raw_info.bits_per_pixel;
    int width = raw_info.width & ~1;
    int height = raw_info.height & ~1;
    int lv_pitch_px = raw_info.pitch * 8 / bits;
    int pitch_px = crx_rate_direct ? lv_pitch_px : (width + 15) & ~15;
    int row_bytes = pitch_px * bits / 8;
    uint8_t *lv_buf = raw_info.buffer;
    log_printf("crx_rate: %s mode, LV %dx%d %d-bit pitch %d px at %x, source %s, pitch %d px",
               is_movie_mode() ? "movie" : "photo", width, height, bits, lv_pitch_px, lv_buf,
               crx_rate_direct ? "LV buffer" : "copy", pitch_px);

    uint8_t *buf = 0, *src_buf;
    if (crx_rate_direct)
    {
        src_buf = UNCACHEABLE(lv_buf);
    }
    else
    {
        buf = fio_malloc(row_bytes * height + 0x1000);
        if (!buf)
        {
            raw_lv_release();
            NotifyBox(3000, "CRX test: no memory");
            return;
        }
        src_buf = UNCACHEABLE((void *)(((uint32_t) buf + 0xFFF) & ~0xFFF));
        memset(src_buf, 0, row_bytes * height);
    }

    static uint32_t job[15];
    memset(job, 0, sizeof(job));
    job[0] = 1;
    job[1] = width;
    job[2] = height;
    job[5] = bits;
    job[6] = (uint32_t) src_buf;
    job[7] = pitch_px;
    job[8] = 1;
    log_hex("crx_rate job", 1, job, sizeof(job));

    void (*free_enc)(uint32_t, uint32_t, uint32_t) = (void *) ALGS_FREE_ENC;
    int n = 0, fails = 0;   /* each frame's buffers are freed right after it */
    int enc_min = 1000000, enc_max = 0, enc_sum = 0, copy_sum = 0;
    uint64_t bytes_sum = 0;
    crx_cbr_quiet = 1;
    int t_start = get_ms_clock();
    for (int i = 0; i < CRX_RATE_FRAMES; i++)
    {
        if (!crx_rate_direct)
        {
            int c0 = get_ms_clock();
            for (int y = 0; y < height; y++)
            {
                memcpy(src_buf + y * row_bytes, lv_buf + y * raw_info.pitch, width * bits / 8);
            }
            copy_sum += get_ms_clock() - c0;
        }

        int ms = 0;
        uint32_t handle = crx_encode_job(job, &ms, i == 0, 1);
        if (!handle || !ram_ptr(handle))
        {
            fails++;
            log_printf("crx_rate: frame %d failed (%d ms)", i, ms);
            if (fails >= 3)
                break;
            continue;
        }
        uint32_t size = 0;
        for (int p = 0; p < 4; p++)
        {
            size += MEM(handle + CRX_HANDLE_SIZES + 8 * p);
        }
        if (i < 3 || i == CRX_RATE_FRAMES - 1)
        {
            log_printf("crx_rate: frame %d %d ms, %d bytes", i, ms, size);
        }
        free_enc(handle, 0, 0);
        n++;
        enc_sum += ms;
        enc_min = MIN(enc_min, ms);
        enc_max = MAX(enc_max, ms);
        bytes_sum += size;
    }
    int total_ms = get_ms_clock() - t_start;
    crx_cbr_quiet = 0;
    raw_lv_release();

    if (buf)
        fio_free(buf);

    int raw_bytes = width * height * bits / 8;
    int avg = n ? (int)(bytes_sum / n) : 0;
    int fps10 = total_ms ? n * 10000 / total_ms : 0;
    log_printf("crx_rate: %d/%d frames, %d fails, total %d ms (%d.%d fps), encode avg %d min %d max %d ms, "
               "copy avg %d ms, avg %d bytes of %d (ratio x100 %d)",
               n, CRX_RATE_FRAMES, fails, total_ms, fps10 / 10, fps10 % 10,
               n ? enc_sum / n : 0, n ? enc_min : 0, enc_max, n ? copy_sum / n : 0,
               avg, raw_bytes, avg ? raw_bytes * 100 / avg : 0);
    NotifyBox(15000, "CRX rate %dx%d (%s): %d/%d ok\n%d.%d fps, encode %d ms (%d-%d)\ncopy %d ms, ratio %d.%02d:1",
              width, height, crx_rate_direct ? "LV buf" : "copy", n, CRX_RATE_FRAMES,
              fps10 / 10, fps10 % 10, n ? enc_sum / n : 0, n ? enc_min : 0, enc_max,
              n ? copy_sum / n : 0, avg ? raw_bytes / avg : 0, avg ? (raw_bytes * 100 / avg) % 100 : 0);
}

/* Sound input test (M6II 1.1.1), modelled on Canon's voice memo code
 * (FileMgr::Voice_Hardware.c, voiSoundDevInPre/InStart, 0xe02c43fc..):
 *   open  0xe03b884e(params)   start 0xe03b8932()
 *   stop  0xe03b894e()         close 0xe03b896a()
 * params: +4/+8 stream (ring) buffer address/size, 0 = Canon's default,
 * written back aligned; +0x0C rate, +0x10 bits, +0x14 channels;
 * +0x18 completion cbr(ctx, info), +0x1C stop cbr(ctx, ...), +0x20 ctx.
 * Logs each completion's info block (voice memo forwards 0xE4 bytes of it
 * as EV_VOICE_RECWRITE) and saves the ring buffer for checking. */
struct snd_in_params
{
    uint32_t unk0;
    void    *stream;
    uint32_t stream_size;
    uint32_t rate;
    uint32_t bits;
    uint32_t channels;
    void   (*complete_cbr)(void *ctx, void *info);
    void   (*stop_cbr)(void *ctx, void *info);
    void    *ctx;
};
#define SND_IN_OPEN   ((int (*)(struct snd_in_params *)) 0xe03b884f)
#define SND_IN_START  ((void (*)(void)) 0xe03b8933)
#define SND_IN_STOP   ((void (*)(void)) 0xe03b894f)
#define SND_IN_CLOSE  ((void (*)(void)) 0xe03b896b)

static volatile int snd_completions = 0;
static struct semaphore *snd_stop_sem = NULL;

/* the test's own record, saved as ML/LOGS/SNDnn.TXT: the debug log can be
 * full of Canon messages before the test runs (DMLOG17) */
static char snd_txt[48 * 1024];
static volatile int snd_txt_len = 0;

static void snd_txtf(const char *fmt, ...)
{
    if (snd_txt_len >= (int) sizeof(snd_txt) - 256)
    {
        return;
    }
    va_list ap;
    va_start(ap, fmt);
    int n = vsnprintf(snd_txt + snd_txt_len, sizeof(snd_txt) - snd_txt_len, fmt, ap);
    va_end(ap);
    if (n > 0)
    {
        snd_txt_len += MIN(n, (int) sizeof(snd_txt) - snd_txt_len - 1);
    }
}

static void snd_complete_cbr(void *ctx, void *info)
{
    int n = snd_completions++;
    if (n < 40 || n % 25 == 0)
    {
        /* voice memo forwards 0xE4 bytes of it */
        snd_txtf("c%d t%u:", n, (uint32_t) get_us_clock());
        for (int i = 0; i < 0xE4 / 4; i++)
        {
            snd_txtf(" %x", MEM((uint32_t) info + 4 * i));
        }
        snd_txtf("\n");
    }
}

static void snd_stop_cbr(void *ctx, void *info)
{
    snd_txtf("stop cbr t%u: ctx %x info %x\n", (uint32_t) get_us_clock(), ctx, info);
    give_semaphore(snd_stop_sem);
}

static void snd_test_task()
{
    if (!is_camera("M6II", "1.1.1"))
    {
        NotifyBox(3000, "Sound test: M6II 1.1.1 only");
        return;
    }
    if (RECORDING)
    {
        NotifyBox(3000, "Sound test: not while recording");
        return;
    }
    if (!snd_stop_sem)
        snd_stop_sem = create_named_semaphore("snd_test", 0);

    static struct snd_in_params params;
    memset(&params, 0, sizeof(params));
    params.rate = 48000;
    params.bits = 16;
    params.channels = 2;
    params.complete_cbr = snd_complete_cbr;
    params.stop_cbr = snd_stop_cbr;
    params.ctx = &params;
    snd_completions = 0;
    snd_txt_len = 0;

    snd_txtf("open t%u\n", (uint32_t) get_us_clock());
    int err = SND_IN_OPEN(&params);
    snd_txtf("open returned %x, stream %x size %d\n", err, params.stream, params.stream_size);
    if (err)
    {
        NotifyBox(5000, "Sound open failed: %x", err);
        return;
    }

    /* mark the ring, so the file shows exactly what got written */
    if (params.stream && params.stream_size)
    {
        for (uint32_t i = 0; i < params.stream_size / 4; i++)
        {
            MEM((uint32_t) params.stream + 4 * i) = 0x5A5AA5A5;
        }
    }

    snd_txtf("start t%u\n", (uint32_t) get_us_clock());
    SND_IN_START();
    /* 7 s: longer than the 5 s ring, to see it wrap */
    msleep(7000);
    snd_txtf("stop t%u\n", (uint32_t) get_us_clock());
    SND_IN_STOP();
    int timed_out = take_semaphore(snd_stop_sem, 5000);
    snd_txtf("stopped (%s), %d completions\n",
             timed_out ? "stop cbr timed out" : "stop cbr ok", snd_completions);
    SND_IN_CLOSE();

    /* save the ring buffer: 48 kHz 16-bit stereo, little endian if PCM */
    char filename[100];
    get_numbered_file_name("ML/LOGS/SND%02d.RAW", 99, filename, sizeof(filename));
    FILE *f = FIO_CreateFile(filename);
    if (f && params.stream && params.stream_size)
    {
        FIO_WriteFile(f, params.stream, params.stream_size);
        FIO_CloseFile(f);
    }
    /* SNDnn.TXT next to it */
    int len = strlen(filename);
    if (len > 3)
    {
        strcpy(filename + len - 3, "TXT");
        f = FIO_CreateFile(filename);
        if (f)
        {
            FIO_WriteFile(f, snd_txt, snd_txt_len);
            FIO_CloseFile(f);
        }
    }
    NotifyBox(10000, "Sound: %d completions\nbuffer %x, %d bytes\n%s",
              snd_completions, params.stream, params.stream_size, filename);
}

/* 2 GB models (CONFIG_MEM_2GB): is 0x40000000 an uncached alias of the
 * first GB, or real memory?  Compare the LV raw buffer (EDMAC channel 1,
 * M6II 0xD0420000, address at +0xA0) at its address and with bit 30
 * flipped.  Aliases hold the same data (give or take a frame in flight). */
static void mem_alias_task()
{
    uint32_t a = MEM(0xD04200A0);
    if (!a)
    {
        NotifyBox(3000, "Alias check: no LV raw address (enable LV raw)");
        return;
    }
    uint32_t b = a ^ 0x40000000;
    int same = 0, total = 256;
    for (int i = 0; i < total; i++)
    {
        if (MEM(a + i * 4096) == MEM(b + i * 4096))
            same++;
    }
    log_printf("alias check: %08x vs %08x: %d/%d words equal", a, b, same, total);
    NotifyBox(10000, "%08x vs %08x\n%d/%d sample words equal\n%s", a, b, same, total,
              same > total * 9 / 10 ? "=> aliases" : same < total / 10 ? "=> different memory" : "=> unclear");
}

/* Why mlv_lite shows no recording status: it only draws when
 * liveview_display_idle() is true (zebra.c).  Show each of its inputs for
 * 10 s; run it in movie mode LiveView with the menu closed. */
extern int liveview_display_idle();
extern int display_is_on();
extern int job_state_ready_to_take_pic();
extern int menu_active_and_not_hidden();
extern int get_global_draw();
extern int gui_state, lv, lv_paused, mirror_down;
extern void *LiveViewApp_dialog;

static void lv_idle_task()
{
    gui_stop_menu();
    msleep(1000);
    for (int i = 0; i < 20; i++)
    {
        bmp_printf(FONT_MED, 0, 60,
            "idle %d  display_on %d  lv %d paused %d  \n"
            "job_ready %d  menu %d  mirror %d  \n"
            "gui_state %d  gui_mode %x  \n"
            "LiveViewApp_dialog %x  global_draw %d  ",
            liveview_display_idle(), display_is_on(), lv, lv_paused,
            job_state_ready_to_take_pic(), menu_active_and_not_hidden(), mirror_down,
            gui_state, MEM(0x8484) /* CURRENT_GUI_MODE, M6II */,
            LiveViewApp_dialog, get_global_draw());
        msleep(500);
    }
}

/* 10-bit LV raw turns the LV image white/noisy: Canon's pipeline reads
 * the raw buffer back.  Candidate reader: EDMAC channel 40 (1_RD_L0,
 * 0xD0421000, pack reg 0xD0422724), next to the writer, channel 1
 * (1_WR_L0, 0xD0420000, pack reg 0xD0422124).  Show both for 10 s:
 * address, size registers, pointer, pack register. */
static void raw_reader_task()
{
    const uint32_t bases[2] = { 0xD0420000, 0xD0421000 };
    const uint32_t packs[2] = { 0xD0422124, 0xD0422724 };
    gui_stop_menu();
    msleep(1000);
    for (int i = 0; i < 20; i++)
    {
        for (int c = 0; c < 2; c++)
        {
            uint32_t b = bases[c];
            bmp_printf(FONT_MED, 0, 60 + c * 70,
                "ch %d: addr %08x ptr %08x pack %08x  \n"
                "  +50 %08x +58 %08x +5C %08x  ",
                c ? 40 : 1, MEM(b + 0xA0), MEM(b + 0xEC), MEM(packs[c]),
                MEM(b + 0x50), MEM(b + 0x58), MEM(b + 0x5C));
            if (i == 0)
            {
                log_printf("ch %d: addr %08x ptr %08x pack %08x +50 %08x +58 %08x +5C %08x",
                    c ? 40 : 1, MEM(b + 0xA0), MEM(b + 0xEC), MEM(packs[c]),
                    MEM(b + 0x50), MEM(b + 0x58), MEM(b + 0x5C));
            }
        }
        msleep(500);
    }
}

/* Find the channel that reads the LV raw buffer back (ch 40 doesn't):
 * show RawPro's channels (state 0x28F38: +0x30 write, +0x34 read, set at
 * runtime) and RawSynchronize's (0x2900C: [0], [0xC], [0x12]), then scan
 * the read channels' address registers for the raw buffer.  The channel
 * about to be read is shown first, in case an unpowered one hangs. */
extern uint32_t edmac_get_base(uint32_t ch);
extern uint32_t edmac_get_pack_reg(uint32_t ch);

static void raw_reader_scan_task()
{
    gui_stop_menu();
    msleep(1000);
    uint32_t raw = MEM(0xD04200A0);
    uint32_t raw_end = raw + (MEM(0xD042005C) & 0xFFFF) * ((MEM(0xD0420050) >> 16) + 1);
    uint32_t *rp = (uint32_t *)0x28F38, *rs = (uint32_t *)0x2900C;
    bmp_printf(FONT_MED, 0, 40, "raw %08x-%08x  RawPro wr %x rd %x mode %x  ",
               raw, raw_end, rp[0x30/4], rp[0x34/4], rp[1]);
    bmp_printf(FONT_MED, 0, 60, "RawSync %x %x %x  ", rs[0], rs[0xC], rs[0x12]);
    log_printf("raw %08x-%08x RawPro wr %x rd %x mode %x RawSync %x %x %x",
               raw, raw_end, rp[0x30/4], rp[0x34/4], rp[1], rs[0], rs[0xC], rs[0x12]);
    msleep(3000);

    int y = 100, found = 0;
    for (uint32_t ch = 39; ch <= 75; ch++)
    {
        uint32_t base = edmac_get_base(ch);
        if (!base) continue;
        bmp_printf(FONT_MED, 0, 80, "reading ch %d (%08x)...  ", ch, base);
        msleep(50);
        uint32_t addr = MEM(base + 0xA0);
        if (addr >= raw && addr < raw_end && raw)
        {
            uint32_t pr = edmac_get_pack_reg(ch);
            bmp_printf(FONT_MED, 0, y, "ch %d: addr %08x pack %08x +50 %08x +5C %08x  ",
                       ch, addr, pr ? MEM(pr) : 0, MEM(base + 0x50), MEM(base + 0x5C));
            log_printf("reader ch %d base %08x addr %08x pack %08x +50 %08x +58 %08x +5C %08x",
                       ch, base, addr, pr ? MEM(pr) : 0, MEM(base + 0x50), MEM(base + 0x58), MEM(base + 0x5C));
            y += 20;
            found++;
        }
    }
    bmp_printf(FONT_MED, 0, 80, "scan done: %d reader(s) of the raw buffer  ", found);
    msleep(10000);
}

static MENU_UPDATE_FUNC(dmlog_update)
{
    MENU_SET_VALUE(is_logging ? "ON" : "OFF");
    if (is_logging && log_buf)
    {
        MENU_SET_RINFO("%d / %d KB", log_len >> 10, log_size >> 10);
    }
}

static struct menu_entry dmlog_menu[] =
{
    {
        .name   = "Canon debug log",
        .select = run_in_separate_task,
        .priv   = dmlog_toggle_task,
        .update = dmlog_update,
        .icon_type = IT_ACTION,
        .help   = "Log Canon debug messages, CRaw encoder params, engio register changes.",
        .help2  = "Start, take a CR3 still / RAW burst, stop: ML/LOGS/DMLOGnn.LOG",
    },
    {
        .name   = "Hooks",
        .priv   = &hook_group,
        .max    = 6,
        .choices = CHOICES("Properties only", "Debug messages", "CRaw encoder", "engio writes", "EDMAC setup", "Encoder start", "CMOS writes"),
        .help   = "What to hook besides properties. One kind per boot (MMU page limit).",
        .help2  = "Debug msgs: CRX/SRM/sound. CRaw: CRX params. engio: FPS. EDMAC: channels. Enc: jobs. CMOS: sensor registers.",
    },
    {
        .name   = "Pre-rec command test",
        .select = run_in_separate_task,
        .priv   = prerec_test_task,
        .icon_type = IT_ACTION,
        .help   = "RAW burst + pre-shooting on: start them as Canon does (no half-press), 4 s,",
        .help2  = "stop. PoCom MechalessStart, GUI START_PREREC ... STOP_PREREC, MechalessStop.",
    },
    {
        .name   = "Mechaless mode test",
        .select = run_in_separate_task,
        .priv   = mechaless_test_task,
        .icon_type = IT_ACTION,
        .help   = "Canon's RAW burst mode switch only (PoCom MechalessStart), 5 s, stop.",
        .help2  = "Logs slot, drive mode, ch 1; saves one 1:1 frame to ML/LOGS/MCLFRAME.RAW.",
    },
    {
        .name   = "Pre-rec argument",
        .priv   = &prerec_arg,
        .max    = 2,
        .help   = "Second argument of the pre-rec commands (Canon's value unknown).",
    },
    {
        .name   = "Property spy",
        .select = run_in_separate_task,
        .priv   = spy_start_task,
        .update = spy_update,
        .icon_type = IT_ACTION,
        .help   = "Log ~6000 Canon properties (values, then every change) while the log runs.",
        .help2  = "Then change a Canon setting: its property shows in the log. Until restart.",
    },
    {
        .name   = "Record test",
        .select = run_in_separate_task,
        .priv   = rec_toggle_task,
        .update = rec_update,
        .icon_type = IT_ACTION,
        .help   = "Encoder start + crop: write each cropped pre-shoot frame to PRExxxxx.MLV",
        .help2  = "Arm, then half-press (RAW burst, pre-shooting). Measures copy and card speed.",
    },
    {
        .name   = "Crop 1:1 to 2.39:1",
        .priv   = &crop239,
        .max    = 1,
        .help   = "Hooks = Encoder start: Canon's 5376x3560 encodes (RAW burst, pre-shooting)",
        .help2  = "take rows 692-2939 only (5376x2248). Don't full-press: the CR3 would be wrong.",
    },
    {
        .name   = "Autosave",
        .priv   = &autosave,
        .max    = 1,
        .help   = "While logging: whole log to DMLAUTO0/1.LOG every 0.25 s, new text appended to DMLSTRM.LOG every 50 ms.",
        .help2  = "For freezes: the last save before the battery pull shows what led up to it.",
    },
    {
        .name   = "Quiet",
        .priv   = &quiet,
        .max    = 3,
        .choices = CHOICES("OFF", "ON", "Storage", "Shooting"),
        .help   = "ON: drop noisy classes (power, audio, file I/O, LED, face tracking).",
        .help2  = "Storage: file I/O, SD speed class, errors. Shooting: LV, GUI, RAW burst.",
    },
    {
        .name   = "Slow LV start",
        .priv   = &slow_lv_start,
        .max    = 1,
        .help   = "After Canon's menu closes, sleep 20 ms after each debug message for 8 s.",
        .help2  = "With Autosave: the last save ends a few messages before a LiveView start hang.",
    },
    {
        .name   = "Stop after LV start",
        .priv   = &stop_after_lv,
        .max    = 1,
        .help   = "Stop and save the log 2 s after LiveView comes back on (Canon menu round trip).",
        .help2  = "The ring holds a few seconds only: keeps the LiveView start in the saved log.",
    },
    {
        .name   = "SRM 1stJob test",
        .select = run_in_separate_task,
        .priv   = srm_test_task,
        .icon_type = IT_ACTION,
        .help   = "Allocate and free two SRM 1stJob buffers (M6II). May crash!",
        .help2  = "Run with the debug log ON, in a photo mode (not movie).",
    },
    {
        .name   = "CRX encode test",
        .select = run_in_separate_task,
        .priv   = crx_test_task,
        .icon_type = IT_ACTION,
        .help   = "Encode Canon's last reduced raw (1624x1080) with the CR3 encoder.",
        .help2  = "Photo mode, take a RAW photo first; log ON, hooks: Debug messages.",
    },
    {
        .name   = "CRX LV frame test",
        .select = run_in_separate_task,
        .priv   = crx_lv_test_task,
        .icon_type = IT_ACTION,
        .help   = "Encode one LV raw frame with the CR3 encoder (lossless).",
        .help2  = "LiveView (movie: raw video off). Saves ML/LOGS/CRXSnn.RAW + CRXPnn.BIN.",
    },
    {
        .name   = "CRX LV test depth",
        .priv   = &crx_lv_bits14,
        .max    = 1,
        .choices = CHOICES("As LV (12-bit)", "14-bit"),
        .help   = "Encoder input depth for the CRX LV frame test.",
    },
    {
        .name   = "CRX LV test pitch",
        .priv   = &crx_lv_lvpitch,
        .max    = 1,
        .choices = CHOICES("Padded to 16 px", "LV buffer pitch"),
        .help   = "Row pitch of the CRX LV frame test input.",
    },
    {
        .name   = "CRX rate test",
        .select = run_in_separate_task,
        .priv   = crx_rate_test_task,
        .icon_type = IT_ACTION,
        .help   = "Encode 48 live LV frames back to back; reports fps and ratio.",
        .help2  = "Photo or movie mode LiveView. Movie mode may fail (SRM memory).",
    },
    {
        .name   = "CRX rate source",
        .priv   = &crx_rate_direct,
        .max    = 1,
        .choices = CHOICES("Copy (16 px pitch)", "LV buffer"),
        .help   = "Copy each frame to a padded buffer, or encode Canon's LV buffer directly.",
    },
    {
        .name   = "Sound input test",
        .select = run_in_separate_task,
        .priv   = snd_test_task,
        .icon_type = IT_ACTION,
        .help   = "Record 7 s through Canon's sound input (voice memo path). M6II.",
        .help2  = "Saves ML/LOGS/SNDnn.RAW (48k 16-bit stereo ring) and SNDnn.TXT (callbacks).",
    },
    {
        .name   = "Raw reader check",
        .select = run_in_separate_task,
        .priv   = raw_reader_task,
        .icon_type = IT_ACTION,
        .help   = "EDMAC ch 1 (LV raw writer) and ch 40 (reader?) registers, 10 s.",
        .help2  = "Run with LV raw on, at 12-bit and again with 10-bit enabled.",
    },
    {
        .name   = "Raw reader scan",
        .select = run_in_separate_task,
        .priv   = raw_reader_scan_task,
        .icon_type = IT_ACTION,
        .help   = "Finds EDMAC read channels pointing at the LV raw buffer (may hang!).",
        .help2  = "LV raw on, 12-bit. If it hangs, note the 'reading ch N' line.",
    },
    {
        .name   = "LV display idle check",
        .select = run_in_separate_task,
        .priv   = lv_idle_task,
        .icon_type = IT_ACTION,
        .help   = "Shows the conditions mlv_lite needs to draw its status (10 s).",
        .help2  = "Run in movie mode LiveView; the menu closes by itself.",
    },
    {
        .name   = "Memory alias check",
        .select = run_in_separate_task,
        .priv   = mem_alias_task,
        .icon_type = IT_ACTION,
        .help   = "Is RAM at 0x6xxxxxxx an alias of 0x2xxxxxxx? (LV raw buffer, M6II)",
        .help2  = "Needs LV raw on (e.g. mlv_lite enabled, or edmac module LV raw stream).",
    }
};

static unsigned int dmlog_init()
{
    menu_add("Debug", dmlog_menu, COUNT(dmlog_menu));
    return 0;
}

static unsigned int dmlog_deinit()
{
    if (is_logging)
    {
        is_logging = 0;
        stop_logging();
    }
    return 0;
}

MODULE_INFO_START()
    MODULE_INIT(dmlog_init)
    MODULE_DEINIT(dmlog_deinit)
MODULE_INFO_END()

MODULE_CBRS_START()
    MODULE_CBR(CBR_SECONDS_CLOCK, dmlog_seconds_cbr, 0)
MODULE_CBRS_END()

MODULE_CONFIGS_START()
    MODULE_CONFIG(hook_group)
    MODULE_CONFIG(quiet)
    MODULE_CONFIG(autosave)
    MODULE_CONFIG(slow_lv_start)
    MODULE_CONFIG(stop_after_lv)
    MODULE_CONFIG(crop239)
    MODULE_CONFIG(prerec_arg)
    MODULE_CONFIG(crx_lv_bits14)
    MODULE_CONFIG(crx_rate_direct)
    MODULE_CONFIG(crx_lv_lvpitch)
MODULE_CONFIGS_END()
