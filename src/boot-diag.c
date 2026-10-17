/*
 * Boot progress diagnostics (CONFIG_BOOT_LED_DIAG).
 *
 * For a camera that hangs with black screens while DryOS keeps running.
 *
 * Card LED:
 *   entry       3 slow blinks   ML is about to jump into Canon's startup code
 *   pre-init    2 fast blinks   DryOS is up, Canon's init_task is about to run
 *   heartbeat   long pulse + N short pulses, then a pause, repeated
 *               N = furthest milestone task that has been scheduled (0..9)
 *               two long pulses instead of one: a log hook failed to install
 *   saved       5 fast blinks   BOOTDIAG.LOG / BOOTDIA2.LOG written to the card
 *
 *   hooks       k very fast flickers before installing log hook k (1..3)
 *
 * Log (CONFIG_BOOT_DIAG_LOG, on by default with CONFIG_BOOT_LED_DIAG):
 *   after the first heartbeat, buffers come from Canon's heap and the hooks
 *   (DryosDebugMsg, mpu_send, MPU receive analyser) are installed one per
 *   heartbeat. Hooking before Canon's init_task stopped a real 5D2, so
 *   nothing is patched that early. Raw arguments are recorded (no formatting
 *   in the hook), plus the first context switch of every task seen after
 *   capture starts. The heartbeat task writes A:/BOOTDIAG.LOG after ~30 s
 *   and A:/BOOTDIA2.LOG after ~120 s.
 *
 * Addresses are for 5D2 2.1.2 and are checked against the original
 * instruction before hooking.
 */

#include "dryos.h"
#include "boot-diag.h"

#ifdef CONFIG_BOOT_LED_DIAG

#include "patch.h"
#include "fio-ml.h"
#include "mem.h"
#include "imath.h"

#ifndef CARD_LED_ADDRESS
#error CONFIG_BOOT_LED_DIAG needs CARD_LED_ADDRESS, LEDON and LEDOFF
#endif

#ifndef CONFIG_NO_BOOT_DIAG_LOG
#define CONFIG_BOOT_DIAG_LOG
#endif

#define LED_ON()   (*(volatile uint32_t *)(CARD_LED_ADDRESS) = (LEDON))
#define LED_OFF()  (*(volatile uint32_t *)(CARD_LED_ADDRESS) = (LEDOFF))
#define DIGIC_US() (*(volatile uint32_t *)0xC0242014)   /* 20-bit us timer */

static const char * const milestones[] = {
    "Startup",      /* 1 */
    "PropMgr",      /* 2  property manager: reads the settings in ROM0 */
    "EventMgr",     /* 3 */
    "FileMgr",      /* 4  card is mounted after this */
    "MainCtrl",     /* 5 */
    "CSMgrTask",    /* 6 */
    "LiveViewMgr",  /* 7 */
    "PtpDps",       /* 8 */
    "GuiMainTask",  /* 9 */
};
#define NUM_MILESTONES (int)(sizeof(milestones) / sizeof(milestones[0]))

static volatile int diag_stage = 0;
static volatile int hook_errors = 0;

/* ---------------------------------------------------------------- LED */

/* busy-wait delay, usable before DryOS timers are running; roughly calibrated */
static void diag_spin(int units)
{
    for (volatile int i = 0; i < units * 200000; i++)
        ;
}

static void diag_blink_spin(int times, int on, int off)
{
    for (int i = 0; i < times; i++)
    {
        LED_ON();  diag_spin(on);
        LED_OFF(); diag_spin(off);
    }
}

static void diag_blink(int times, int on, int off)
{
    for (int i = 0; i < times; i++)
    {
        LED_ON();  msleep(on);
        LED_OFF(); msleep(off);
    }
}

static int name_eq(const char *a, const char *b)
{
    if (!a || !b) return 0;
    while (*a && *a == *b) { a++; b++; }
    return *a == *b;
}

/* ---------------------------------------------------------------- log */

#ifdef CONFIG_BOOT_DIAG_LOG

/* 5D2 2.1.2 */
#define ADDR_DEBUGMSG   0xFF86AF64
#define INSN_DEBUGMSG   0xE92D000F
#define ADDR_MPU_SEND   0xFF99F518      /* mpu_send(uint8_t *buf, int size) */
#define INSN_MPU_SEND   0xE92D41F0
#define ADDR_MPU_RX     0xFF99BA38      /* MPU receive analyser (ctx, msg *) */
#define INSN_MPU_RX     0xE92D4070
#define ADDR_SIO_WORD   0xFF99F658      /* SIO3 ISR -> per-interrupt word handler (word) */
#define INSN_SIO_WORD   0xE92D4031
#define MPU_STATUS_REG  0xC022009C      /* polled by Canon's MPU driver itself */
#define MPU_DRV_STATE   0x88A8          /* driver struct: +0 state (indexes 0x31F44), +8 tx write, +C tx read */

enum { REC_DM = 1, REC_MPU_TX, REC_MPU_RX, REC_TASK, REC_MARK, REC_SIO };
#define NUM_HOOKS 4

struct diag_rec
{
    uint16_t type;
    uint16_t len;
    uint32_t t;
    uint32_t a[8];
};

/* allocated from Canon's heap once the heartbeat is running (not in BSS:
 * on DIGIC 4, ML's BSS is carved out of Canon's malloc pool) */
#define LIN_RECS_MAX 4000               /* first records after hooking */
#define RING_RECS    256                /* most recent records */
#define CHUNK_SIZE   8192

static struct diag_rec *lin_buf = 0;
static struct diag_rec *ring_buf = 0;
static uint32_t lin_recs = 0;
static volatile uint32_t lin_n = 0;
static volatile uint32_t ring_n = 0;
static volatile uint32_t total_n = 0;
static volatile int capturing = 0;
static int capture_start_beat = 0;

#define MAX_SEEN_TASKS 96
static const char *seen_tasks[MAX_SEEN_TASKS];
static int num_seen_tasks = 0;

static struct diag_rec *rec_alloc(void)
{
    if (!capturing)
        return 0;

    uint32_t old = cli();
    uint32_t i = lin_n;
    struct diag_rec *r;
    if (i < lin_recs)
    {
        lin_n = i + 1;
        r = &lin_buf[i];
    }
    else
    {
        r = &ring_buf[ring_n % RING_RECS];
        ring_n++;
    }
    total_n++;
    sei(old);

    r->t = DIGIC_US();
    return r;
}

static void hook_debugmsg(uint32_t *regs, uint32_t *stack, uint32_t pc)
{
    struct diag_rec *r = rec_alloc();
    if (!r) return;
    r->type = REC_DM;
    r->a[0] = regs[0];      /* class */
    r->a[1] = regs[1];      /* level */
    r->a[2] = regs[2];      /* fmt */
    r->a[3] = regs[3];      /* arg 0 */
    for (int i = 0; i < 4; i++)
        r->a[4 + i] = stack[i];   /* args 1..4 */
}

static void hook_mpu_send(uint32_t *regs, uint32_t *stack, uint32_t pc)
{
    struct diag_rec *r = rec_alloc();
    if (!r) return;
    const uint8_t *buf = (const uint8_t *) regs[0];
    int size = regs[1];
    r->type = REC_MPU_TX;
    r->len = size;
    r->a[0] = (uint32_t) buf;
    uint8_t *dst = (uint8_t *) &r->a[1];
    for (int i = 0; i < 28; i++)
        dst[i] = (i < size && buf) ? buf[i] : 0;
}

static void hook_mpu_rx(uint32_t *regs, uint32_t *stack, uint32_t pc)
{
    struct diag_rec *r = rec_alloc();
    if (!r) return;
    const uint32_t *msg = (const uint32_t *) regs[1];
    r->type = REC_MPU_RX;
    for (int i = 0; i < 8; i++)
        r->a[i] = msg ? msg[i] : 0;
}

/* every SIO3 (MPU link) interrupt, both directions: r0 = word read from the
 * MPU data register (0 when the ICU is the one sending) */
static void hook_sio_word(uint32_t *regs, uint32_t *stack, uint32_t pc)
{
    struct diag_rec *r = rec_alloc();
    if (!r) return;
    r->type = REC_SIO;
    r->a[0] = regs[0] & 0xFFFF;
    r->a[1] = MEM(MPU_STATUS_REG);
    r->a[2] = MEM(MPU_DRV_STATE);
    r->a[3] = MEM(MPU_DRV_STATE + 8);
    r->a[4] = MEM(MPU_DRV_STATE + 0xC);
}

static void log_task(const char *name)
{
    for (int i = 0; i < num_seen_tasks; i++)
        if (seen_tasks[i] == name)
            return;
    if (num_seen_tasks >= MAX_SEEN_TASKS)
        return;

    struct diag_rec *r = rec_alloc();
    if (!r) return;
    seen_tasks[num_seen_tasks++] = name;
    r->type = REC_TASK;
    r->a[0] = (uint32_t) name;
}

static void log_mark(int stage)
{
    struct diag_rec *r = rec_alloc();
    if (!r) return;
    r->type = REC_MARK;
    r->a[0] = stage;
}

extern void *_AllocateMemory(size_t size);

static char *chunk = 0;

static int alloc_buffers(void)
{
    ring_buf = _AllocateMemory(RING_RECS * sizeof(struct diag_rec));
    chunk = _AllocateMemory(CHUNK_SIZE);
    if (!ring_buf || !chunk)
        return 0;

    for (uint32_t n = LIN_RECS_MAX; n >= 125; n /= 2)
    {
        lin_buf = _AllocateMemory(n * sizeof(struct diag_rec));
        if (lin_buf)
        {
            lin_recs = n;
            return 1;
        }
    }
    return 0;
}

/* hook k (1..NUM_HOOKS) */
static void install_hook(int k)
{
    int err = 0;
    switch (k)
    {
        case 1: err = patch_hook_function(ADDR_DEBUGMSG, INSN_DEBUGMSG, hook_debugmsg, "diag DebugMsg"); break;
        case 2: err = patch_hook_function(ADDR_MPU_SEND, INSN_MPU_SEND, hook_mpu_send, "diag mpu_send"); break;
        case 3: err = patch_hook_function(ADDR_MPU_RX,   INSN_MPU_RX,   hook_mpu_rx,   "diag mpu_rx");   break;
        case 4: err = patch_hook_function(ADDR_SIO_WORD, INSN_SIO_WORD, hook_sio_word, "diag sio");      break;
    }
    if (err)
        hook_errors |= 1 << (k - 1);
}

/* ------------------------------------------------------- log writing */

static int ptr_ok(uint32_t p)
{
    return (p >= 0x1000 && p < 0x20000000) ||
           (p >= 0x40001000 && p < 0x60000000) ||
           (p >= 0xF0000000 && p < 0xFFFFFF00);
}

/* copy fmt, turning %s with an unreadable argument into %x */
static void safe_fmt(char *out, int outsz, const char *fmt, const uint32_t *args, int nargs)
{
    int o = 0, ai = 0;
    for (int i = 0; fmt[i] && o < outsz - 2 && i < 200; i++)
    {
        char c = fmt[i];
        out[o++] = c;
        if (c != '%')
            continue;
        if (fmt[i + 1] == '%') { out[o++] = '%'; i++; continue; }
        /* flags, width, precision, length */
        while (fmt[i + 1] && strchr("-+ #0123456789.*lhqjzt", fmt[i + 1]) && o < outsz - 2)
        {
            if (fmt[i + 1] == '*') ai++;
            out[o++] = fmt[++i];
        }
        char conv = fmt[i + 1];
        if (!conv) break;
        i++;
        if ((conv == 's' || conv == 'S') && (ai >= nargs || !ptr_ok(args[ai])))
            conv = 'x';
        out[o++] = conv;
        ai++;
    }
    out[o] = 0;
}

static int chunk_len = 0;
static FILE *log_f = 0;

extern FILE *_FIO_CreateFile(const char *filename);
extern int _FIO_WriteFile(FILE *stream, const void *ptr, size_t count);
extern int _FIO_RemoveFile(const char *filename);

static void chunk_flush(void)
{
    if (chunk_len && log_f)
    {
        sync_caches();
        _FIO_WriteFile(log_f, UNCACHEABLE(chunk), chunk_len);
    }
    chunk_len = 0;
}

/* output goes either to the card (chunked) or to a memory buffer (PTP) */
static char *mem_sink = 0;
static int mem_len = 0, mem_max = 0;
static volatile int log_busy = 0;

static void emit(const char *line)
{
    int n = strlen(line);
    if (mem_sink)
    {
        if (mem_len + n + 1 < mem_max)
        {
            memcpy(mem_sink + mem_len, line, n);
            mem_len += n;
            mem_sink[mem_len++] = '\n';
        }
        return;
    }
    if (chunk_len + n + 1 >= CHUNK_SIZE)
        chunk_flush();
    memcpy(chunk + chunk_len, line, n);
    chunk_len += n;
    chunk[chunk_len++] = '\n';
}

static void format_rec(const struct diag_rec *r, uint32_t seq)
{
    char line[256];
    char fbuf[220];
    int n = snprintf(line, sizeof(line), "%5d %05x ", seq, r->t);

    switch (r->type)
    {
        case REC_DM:
            n += snprintf(line + n, sizeof(line) - n, "DM %2d.%d ", r->a[0], r->a[1]);
            if (ptr_ok(r->a[2]))
            {
                safe_fmt(fbuf, sizeof(fbuf), (const char *) r->a[2], &r->a[3], 5);
                snprintf(line + n, sizeof(line) - n, fbuf,
                         r->a[3], r->a[4], r->a[5], r->a[6], r->a[7]);
            }
            else
            {
                snprintf(line + n, sizeof(line) - n, "<fmt %x> %x %x %x",
                         r->a[2], r->a[3], r->a[4], r->a[5]);
            }
            break;

        case REC_MPU_TX:
        {
            n += snprintf(line + n, sizeof(line) - n, "MPU> [%d]", r->len);
            const uint8_t *b = (const uint8_t *) &r->a[1];
            for (int i = 0; i < r->len && i < 28; i++)
                n += snprintf(line + n, sizeof(line) - n, " %02x", b[i]);
            break;
        }

        case REC_MPU_RX:
            snprintf(line + n, sizeof(line) - n, "MPU< %x %x %x %x %x %x %x %x",
                     r->a[0], r->a[1], r->a[2], r->a[3], r->a[4], r->a[5], r->a[6], r->a[7]);
            break;

        case REC_TASK:
            snprintf(line + n, sizeof(line) - n, "TASK %s",
                     ptr_ok(r->a[0]) ? (const char *) r->a[0] : "?");
            break;

        case REC_SIO:
            snprintf(line + n, sizeof(line) - n, "SIO word %04x  status %02x  state %d  tx w/r %d/%d",
                     r->a[0], r->a[1] & 0xFF, r->a[2], r->a[3], r->a[4]);
            break;

        case REC_MARK:
            snprintf(line + n, sizeof(line) - n, "MILESTONE %d (%s)", r->a[0],
                     (r->a[0] >= 1 && (int) r->a[0] <= NUM_MILESTONES) ? milestones[r->a[0] - 1] : "?");
            break;

        default:
            snprintf(line + n, sizeof(line) - n, "? type %d", r->type);
    }
    emit(line);
}

static int save_failures = 0;
static int ptp_registered = 0;
static uint32_t ptp_reg_err = 0;
static volatile uint32_t ptp_calls = 0;

/* header + all records, through emit(); capture paused by the caller */
static void write_log_body(void)
{
    char line[160];
    snprintf(line, sizeof(line), "5D2 boot diag: stage %d, hook errors %x, records %d (first %d + last %d)",
             diag_stage, hook_errors, total_n, lin_n, MIN(ring_n, RING_RECS));
    emit(line);
    snprintf(line, sizeof(line), "linear buffer %d records; capture started at heartbeat %d; card save failures %d",
             lin_recs, capture_start_beat, save_failures);
    emit(line);
    snprintf(line, sizeof(line), "ptp: registered %d, register result %x, calls %d",
             ptp_registered, ptp_reg_err, ptp_calls);
    emit(line);
    snprintf(line, sizeof(line), "bootflags %x %x %x %x",
             MEM(0xF8000000), MEM(0xF8000004), MEM(0xF8000008), MEM(0xF800000C));
    emit(line);

    for (uint32_t i = 0; i < lin_n; i++)
        format_rec(&lin_buf[i], i);

    if (ring_n)
    {
        emit("---- last records ----");
        uint32_t count = MIN(ring_n, RING_RECS);
        uint32_t start = ring_n - count;
        for (uint32_t i = 0; i < count; i++)
            format_rec(&ring_buf[(start + i) % RING_RECS], lin_recs + start + i);
    }
}

static int save_log(const char *filename)
{
    if (log_busy) return 0;
    log_busy = 1;
    capturing = 0;
    msleep(50);

    _FIO_RemoveFile(filename);
    log_f = _FIO_CreateFile(filename);
    if (log_f == PTR_INVALID || !log_f)
    {
        log_f = 0;
        save_failures++;
        capturing = 1;
        log_busy = 0;
        return 0;
    }

    write_log_body();

    chunk_flush();
    FIO_CloseFile(log_f);
    log_f = 0;
    capturing = 1;
    log_busy = 0;
    return 1;
}

/* ------------------------------------------------------------ PTP */

/*
 * Opcode 0xA1D9, works without the card being mounted:
 *   param1 = 0: status -> response params: stage, hook_errors, total, lin_n, ring_n, save_failures
 *   param1 = 1: data phase = formatted log text (same as BOOTDIAG.LOG)
 *   param1 = 2: data phase = raw memory, param2 = address, param3 = length (max 1 MiB)
 * See contrib/recovery/5D2.212/bootdiag_ptp.py
 */
#include "ptp.h"
#define PTP_BOOTDIAG_CODE 0xA1D9
#define PTP_CHUNK (128 * 1024)

extern void _FreeMemory(void *ptr);

static int diag_send(struct ptp_context *ctx, const char *buf, int size)
{
    int total = size;
    sync_caches();
    while (size > 0)
    {
        int part = MIN(size, PTP_CHUNK);
        if (ctx->send_data(ctx->handle, (void *) buf, part, total, 0, 0, 0))
            return 0;
        total = 0;
        size -= part;
        buf += part;
    }
    return 1;
}

static int diag_ptp_handler(void *priv, struct ptp_context *ctx,
                            uint32_t opcode, uint32_t session, uint32_t transaction,
                            uint32_t param1, uint32_t param2, uint32_t param3,
                            uint32_t param4, uint32_t param5)
{
    ptp_calls++;
    struct ptp_msg msg = {
        .id          = PTP_RC_OK,
        .session     = session,
        .transaction = transaction,
        .param_count = 0,
    };

    switch (param1)
    {
        case 0:
            msg.param_count = 5;
            msg.param[0] = diag_stage;
            msg.param[1] = hook_errors;
            msg.param[2] = total_n;
            msg.param[3] = lin_n;
            msg.param[4] = ring_n;
            break;

        case 1:
        {
            if (log_busy || !lin_buf) { msg.id = PTP_RC_GeneralError; break; }
            int max = (lin_n + RING_RECS + 8) * 200;
            char *buf = _AllocateMemory(max);
            if (!buf) { msg.id = PTP_RC_GeneralError; break; }
            log_busy = 1;
            capturing = 0;
            mem_sink = buf; mem_len = 0; mem_max = max;
            write_log_body();
            mem_sink = 0;
            capturing = 1;
            log_busy = 0;
            if (!diag_send(ctx, buf, mem_len))
                msg.id = PTP_RC_GeneralError;
            _FreeMemory(buf);
            break;
        }

        case 2:
            if (!param2 || param3 < 1 || param3 > 0x100000 ||
                !(ptr_ok(param2) && ptr_ok(param2 + param3 - 1)))
            {
                msg.id = PTP_RC_ParameterNotSupported;
                break;
            }
            if (!diag_send(ctx, (const char *) param2, param3))
                msg.id = PTP_RC_GeneralError;
            break;

        default:
            msg.id = PTP_RC_ParameterNotSupported;
    }

    ctx->send_resp(ctx->handle, &msg);
    return 0;
}


/* returns 0 on success; 0x17 if Canon's PTP framework isn't initialised yet
 * (its handler table pointer at 0x14178 is still NULL on 5D2 2.1.2) */
static void diag_ptp_register(void)
{
    if (ptp_registered)
        return;
    ptp_reg_err = ptp_register_handler(PTP_BOOTDIAG_CODE, (void *) diag_ptp_handler, 0);
    if (ptp_reg_err == 0)
        ptp_registered = 1;
}

#endif /* CONFIG_BOOT_DIAG_LOG */

/* ------------------------------------------------------------ hooks */

void boot_diag_entry(void)
{
    diag_blink_spin(3, 15, 15);
    diag_spin(30);
}

void boot_diag_pre_init_task(void)
{
    diag_blink_spin(2, 3, 5);
    diag_spin(30);
    /* nothing else here: hooking ROM before Canon's startup has set up the
     * caches stopped the camera on real hardware */
}

/* called from the task dispatch hook on every context switch: keep it cheap */
void boot_diag_task_seen(const char *name)
{
    if (!name)
        return;

#ifdef CONFIG_BOOT_DIAG_LOG
    log_task(name);
#endif

    int s = diag_stage;
    for (int i = s; i < NUM_MILESTONES; i++)
    {
        if (name[0] == milestones[i][0] && name_eq(name, milestones[i]))
        {
            diag_stage = i + 1;
#ifdef CONFIG_BOOT_DIAG_LOG
            log_mark(i + 1);
#endif
            return;
        }
    }
}

static void diag_heartbeat_task(void *unused)
{
    int elapsed_ms = 0;
    int beat = 0;
#ifdef CONFIG_BOOT_DIAG_LOG
    int saved1 = 0, saved2 = 0;
    int hooks_done = 0;

    /* buffers and hooks before the first heartbeat, so the ICU<->MPU
     * exchange during Canon's startup is captured. (Hooking before
     * init_task stopped a real 5D2; this runs right after it returns.)
     * BOOT_DIAG_LATE_HOOKS=y restores one-hook-per-heartbeat installs. */
    int buffers_ok = alloc_buffers();
    if (buffers_ok)
    {
        capture_start_beat = 0;
        capturing = 1;
#ifndef CONFIG_BOOT_DIAG_LATE_HOOKS
        for (int k = 1; k <= NUM_HOOKS; k++)
            install_hook(k);
        hooks_done = NUM_HOOKS;
#endif
    }
    else
    {
        hook_errors |= 0x80;
    }
#endif

    while (1)
    {
        int n = diag_stage;
        int pulses = hook_errors ? 2 : 1;

        diag_blink(pulses, 800, 400);
        diag_blink(n, 150, 300);
        msleep(2000);
        elapsed_ms += pulses * 1200 + n * 450 + 2000;
        beat++;

#ifdef CONFIG_BOOT_DIAG_LOG
#ifdef CONFIG_BOOT_DIAG_LATE_HOOKS
        if (buffers_ok && hooks_done < NUM_HOOKS)
        {
            int k = hooks_done + 1;
            msleep(500);
            diag_blink(k, 40, 120);
            msleep(500);
            install_hook(k);
            hooks_done = k;
        }
#endif

        if (hooks_done)
        {
            /* Canon's own cache flushes can drop our ROM hooks; put them back */
            sync_caches();
        }

        /* PTP readout: needs the USB/PTP manager (milestone 8) to be running */
        if (buffers_ok && diag_stage >= 8)
            diag_ptp_register();

        if (buffers_ok && !saved1 && elapsed_ms > 30000 && save_log("A:/BOOTDIAG.LOG"))
        {
            saved1 = 1;
            diag_blink(5, 100, 100);
        }
        if (saved1 && !saved2 && elapsed_ms > 120000 && save_log("A:/BOOTDIA2.LOG"))
        {
            saved2 = 1;
            diag_blink(5, 100, 100);
        }
#endif
    }
}

void boot_diag_post_init_task(void)
{
    /* high priority so a busy Canon task can't starve the heartbeat */
    task_create("diag_hb", 0x08, 0x2000, diag_heartbeat_task, 0);
}

#endif /* CONFIG_BOOT_LED_DIAG */
