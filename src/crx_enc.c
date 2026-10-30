/* Canon's CR3 (CRX) encoder through AlgsMgr, see crx_enc.h.
 *
 * Found on M6II 1.1.1:
 * - StartEncode(jobs, n, cbr, ctx) posts the job to the AlgsMgr task, which
 *   takes SRM memory, sets the encoder up, runs it and calls
 *   cbr(ctx, result, ...); result[0] is the AlgsMgr handle, which holds the
 *   4 plane buffers and their compressed sizes.
 * - FreeEncBuffer(handle, 0, 0) gives the plane buffers back; when all are
 *   back, AlgsMgr returns its SRM memory (keeping it reserved made the
 *   camera crash on the next photo -> movie switch).
 * - The encoder is set up once per encoder id (flag in the AlgsMgr state);
 *   we clear the flag when our parameters change.
 * The plane buffers have no cached mapping (0x8xxxxxxx-0xBxxxxxxx), so a
 * CPU copy reads them slowly; we copy with Canon's DMA (dma_memcpy, HPCopy;
 * C-RAW's many pieces by Xdmac scatter-gather, crx_copy_parts) and check
 * the first copy against the source, falling back to the CPU if it
 * doesn't match (DMA reach and size rules not known yet). */

#include "dryos.h"
#include "crx_enc.h"
#include "ptp.h"
#include "patch.h"
#include "timer.h"

#ifdef CONFIG_CRX_ENCODER

extern uint32_t algs_start_encode(void *jobs, uint32_t n, void (*cbr)(), void *ctx);
extern void algs_free_enc_buffer(uint32_t handle, uint32_t a, uint32_t b);
/* Canon's movie writer gets the output this way (mvrGetCRXEncodeDataCBR):
 * n chunks, entry 0 Canon's 112-byte sample header, then two chunks per
 * plane (the second used when a plane wraps around the encoder's ring) */
extern uint32_t algs_get_encode_data(uint32_t handle, uint32_t *n, uint32_t **addrs, uint32_t **sizes);
extern void algs_start_decode(const void *sample, uint32_t size, void *out, uint32_t out_pitch_px,
                              uint32_t x, uint32_t y, const void *cmp1, uint32_t cmp1_size,
                              void (*cbr)(), void *ctx, uint32_t a, uint32_t b);

#define CRX_ENC_ID          1       /* single-tile lossless (Canon: reduced raw) */
#define CRX_ENC_ID_LOSSY    3       /* single-tile C-RAW: 3 DWT levels, quantised */
static int crx_enc_id = CRX_ENC_ID;
#define CRX_TIMEOUT_MS      1500    /* mlv_lite queues frames: late beats lost */

/* the job, 0x3C bytes: [0] encoder id, [1] width, [2] height, [3] start
 * x (pixels into each row), [5] bits, [6] source address, [7] pitch in
 * pixels, [8] 1 (as Canon's lossless jobs); the rest 0 */
static uint32_t job[15];
static struct semaphore *crx_sem = NULL;
static volatile uint32_t cbr_handle = 0;   /* AlgsMgr handle of the job we wait for */
static volatile uint32_t job_seq = 0;   /* passed as ctx: tells late results apart */
static int timeouts = 0;
static int needs_setup = 1;
static int dma_state = 0;           /* 0: check the next copy, 1: DMA ok, -1: CPU */
static int dma_busy = 0;            /* planes copied by the CPU: no DMA channel free */

/* event log, read over USB (PTP mailbox "crxlog"): a ring of 8-word
 * entries {tag, job, 6 values} after a count word.  Allocated at the
 * first setup (32 kB: kept out of the core's BSS) */
enum { EV_START = 1, EV_CBR, EV_GOT, EV_TIMEOUT, EV_ODD, EV_FREE, EV_LATE_FREE, EV_TOO_BIG, EV_DUMP, EV_RECOVERED };
#define CRX_LOG_ENTRIES 1024
#define CRX_LOG_BYTES ((1 + 8 * CRX_LOG_ENTRIES) * 4)
static uint32_t *crx_log = NULL;
static volatile uint32_t crx_log_len = 0;

static void crx_ev(uint32_t tag, uint32_t a, uint32_t b, uint32_t c, uint32_t d, uint32_t e, uint32_t f)
{
    if (!crx_log)
    {
        return;
    }
    uint32_t old = cli();
    uint32_t *p = &crx_log[1 + 8 * (crx_log[0] % CRX_LOG_ENTRIES)];
    p[0] = tag; p[1] = job_seq; p[2] = a; p[3] = b; p[4] = c; p[5] = d; p[6] = e; p[7] = f;
    crx_log[0]++;
    sei(old);
}

/* compare start, middle and end of each plane after the first DMA copy */
static int dma_copy_ok(const uint8_t *dst, const uint32_t *planes, const uint32_t *sizes)
{
    for (int i = 0; i < 8; i++)
    {
        const uint8_t *src = (const uint8_t *) planes[i];
        uint32_t n = sizes[i];
        uint32_t at[3] = { 0, n / 2 & ~3, n >= 64 ? n - 64 : 0 };
        for (int j = 0; j < 3; j++)
        {
            int len = MIN(64, (int)(n - at[j]));
            if (len > 0 && memcmp(dst + at[j], src + at[j], len))
            {
                return 0;
            }
        }
        dst += n;
    }
    return 1;
}

/* runs on the AlgsMgr task */
static void crx_enc_cbr(void *ctx, uint32_t *result, uint32_t a2, uint32_t a3)
{
    crx_ev(EV_CBR, (uint32_t) ctx, (uint32_t) result, result ? MEM(result) : 0,
           result ? MEM((uint32_t) result + 4) : 0, a2, a3);
    if ((uint32_t) ctx != job_seq)
    {
        /* a job we stopped waiting for: only give its buffers back, or a
         * waiting frame would take this result as its own */
        if (result && MEM(result))
        {
            crx_ev(EV_LATE_FREE, (uint32_t) ctx, MEM(result), 0, 0, 0, 0);
            algs_free_enc_buffer(MEM(result), 0, 0);
        }
        return;
    }
    /* read the handle now: Canon reuses the result record after we
     * return, and reading it later in compress_task sometimes gave a
     * garbage handle (planes 3 / 1, sizes 0) - whose frame we then never
     * freed, which stalls the encoder later */
    cbr_handle = result ? MEM(result) : 0;
    give_semaphore(crx_sem);
}

int crx_enc_available(void)
{
    /* the camera has it; whether Canon has set it up yet is crx_enc_ready
     * (a 0 here at module load used to hide the menu and reset the
     * setting) */
    return 1;
}

int crx_enc_ready(void)
{
    return MEM(ALGS_STATE_PTR) != 0;
}

int crx_enc_setup(int width, int height, int bits, int pitch_px, int lossy)
{
    if (!crx_enc_ready() || width < 64 || height < 64 || (width & 1) || (height & 1))
    {
        return -1;
    }
    if (!crx_sem)
    {
        crx_sem = create_named_semaphore("crx_enc", 0);
        crx_log = malloc(CRX_LOG_BYTES);
        if (crx_log)
        {
            memset(crx_log, 0, CRX_LOG_BYTES);
            crx_log_len = CRX_LOG_BYTES;
            ptp_mailbox_set(2, "crxlog", crx_log, &crx_log_len);
        }
    }
    memset(job, 0, sizeof(job));
    crx_enc_id = lossy ? CRX_ENC_ID_LOSSY : CRX_ENC_ID;
    job[0] = crx_enc_id;
    job[1] = width;
    job[2] = height;
    job[5] = bits;
    job[7] = pitch_px;
    job[8] = 1;
    needs_setup = 1;
    if (dma_state == 1)
    {
        dma_state = 0;      /* check again with the new sizes */
    }
    return 0;
}

static void put_be16(uint8_t *p, uint32_t v) { p[0] = v >> 8; p[1] = v; }
static void put_be32(uint8_t *p, uint32_t v) { p[0] = v >> 24; p[1] = v >> 16; p[2] = v >> 8; p[3] = v; }
static uint32_t get_be16(const uint8_t *p) { return p[0] << 8 | p[1]; }
static uint32_t get_be32(const uint8_t *p) { return (uint32_t) p[0] << 24 | p[1] << 16 | p[2] << 8 | p[3]; }

/* tile, plane and subband headers as in Canon's CR3 reduced raw */
static void crx_sample_header(uint8_t *h, const uint32_t *sizes)
{
    memset(h, 0, CRX_SAMPLE_HDR_SIZE);
    put_be16(h, 0xFF01);
    put_be16(h + 2, 8);
    put_be32(h + 4, sizes[0] + sizes[1] + sizes[2] + sizes[3]);
    for (int i = 0; i < 4; i++)
    {
        uint8_t *p = h + 12 + 24 * i;
        put_be16(p, 0xFF02);
        put_be16(p + 2, 8);
        put_be32(p + 4, sizes[i]);
        p[8] = (i << 4) | 8;
        put_be16(p + 12, 0xFF03);
        put_be16(p + 14, 8);
        put_be32(p + 16, sizes[i]);
        put_be32(p + 20, 0x00200000);   /* low 11 bits: padding at the end, 0 */
    }
}

/* Each plane's bitstream is padded to 8 bytes; how many of those bytes are
 * padding (0-7) is only in Canon's own header (entry 0 of the encoder
 * output).  Canon's decoder requires it (a wrong count fails the plane);
 * LibRaw doesn't check it.  Take the subband flags from Canon's header
 * where its plane sizes agree with ours; returns the planes taken. */
static int crx_sample_header_pads(uint8_t *h, const uint8_t *canon, uint32_t canon_size, const uint32_t *sizes)
{
    if (!canon || canon_size < CRX_SAMPLE_HDR_SIZE || get_be16(canon) != 0xFF01)
    {
        return 0;
    }
    int taken = 0;
    for (int i = 0; i < 4; i++)
    {
        const uint8_t *p = canon + 12 + 24 * i;
        if (get_be16(p) == 0xFF02 && get_be32(p + 4) == sizes[i] &&
            get_be16(p + 12) == 0xFF03 && get_be32(p + 16) == sizes[i] && (get_be32(p + 20) & 0x7FF) < 8)
        {
            memcpy(h + 12 + 24 * i + 20, p + 20, 4);
            taken++;
        }
    }
    return taken;
}

/* Copy from the encoder's uncached buffers in 32-byte bursts (LDM/STM of 8
 * registers): memcpy got ~21 MB/s there (52 ms for a 1.1 MB C-RAW frame).
 * src, dst 4-byte aligned; the tail (< 32 bytes) by memcpy. */
static void copy_burst(void *dst, const void *src, uint32_t n)
{
    if (((uintptr_t) dst | (uintptr_t) src) & 3)
    {
        memcpy(dst, src, n);
        return;
    }
    uint32_t blocks = n / 32;
    uint32_t *d = dst;
    const uint32_t *s = src;
    if (blocks)
    {
        asm volatile (
            "1:\n"
            "ldmia %[s]!, {r3, r4, r5, r6, r7, r8, r9, r10}\n"
            "stmia %[d]!, {r3, r4, r5, r6, r7, r8, r9, r10}\n"
            "subs %[n], %[n], #1\n"
            "bne 1b\n"
            : [s] "+r" (s), [d] "+r" (d), [n] "+r" (blocks)
            :
            : "r3", "r4", "r5", "r6", "r7", "r8", "r9", "r10", "memory", "cc");
    }
    memcpy(d, s, n % 32);
}

/* Scatter-gather DMA for the C-RAW subbands: up to 8 {dst, src, size}
 * descriptors in one Xdmac transfer (the limit in Xdmac.c), on a channel
 * from HPCopy's pool, started asynchronously so a transfer that never
 * completes can't hang us: one DMA per subband (40 a frame, through
 * dma_memcpy, which waits without a timeout) froze the camera.  Canon
 * itself never calls Xdmac's scatter-gather wrapper; this is its path
 * (descriptor prep, async transfer with a callback, as HPCopy's async
 * copy does with one descriptor).  On a timeout the channel stays marked
 * busy (nobody else waits on it) and DMA is off for the session. */
extern uint32_t *hpc_pool_get(void);
extern void hpc_pool_release(uint32_t *entry);
extern void xdmac_desc_prep(uint32_t *desc);
extern int xdmac_transfer_async(uint32_t *xfer, int one, void (*cbr)(int, void *), void *ctx, int zero);

#define SG_MAX          8
#define SG_TIMEOUT_MS   100
static struct semaphore *sg_sem = NULL;
static volatile int sg_status = 0;
static int sg_disabled = 0;
static int sg_checked = 0;      /* first frame compared with the source */

static void sg_cbr(int status, void *ctx)
{
    sg_status = status;
    give_semaphore(sg_sem);
}

/* 0: copied by DMA; -1: not (the caller copies) */
static int sg_copy(uint8_t *const *dst, const uint32_t *src, const uint32_t *len, int k)
{
    if (sg_disabled || k <= 0 || k > SG_MAX)
    {
        return -1;
    }
    if (!sg_sem)
    {
        sg_sem = create_named_semaphore("crx_sg", 0);
    }
    uint32_t *entry = hpc_pool_get();
    if (!entry)
    {
        return -1;
    }
    uint32_t desc[SG_MAX][10] __attribute__((aligned(8)));
    for (int i = 0; i < k; i++)
    {
        desc[i][0] = (uint32_t) dst[i];
        desc[i][1] = src[i];
        desc[i][2] = len[i];
        desc[i][3] = 0;
        xdmac_desc_prep(desc[i]);
    }
    uint32_t xfer[3] = { entry[0], (uint32_t) desc, k };
    sg_status = -1;
    if (xdmac_transfer_async(xfer, 1, sg_cbr, entry, 0) != 0)
    {
        hpc_pool_release(entry);
        return -1;
    }
    if (take_semaphore(sg_sem, SG_TIMEOUT_MS))
    {
        sg_disabled = 1;
        printf("CRX lossy: DMA of %d pieces didn't finish (ch %d, first %x->%x %d bytes): CPU copies\n",
               k, entry[0], src[0], dst[0], len[0]);
        return -1;
    }
    hpc_pool_release(entry);
    return 0;
}

/* Copy the non-empty parts 1..n-1 of an encoder result one after another
 * to dst (see crx_enc.h).  Each subband has its own encoder buffer (~1 MB
 * apart): 40 pieces of 9-68 kB a frame for a single-tile C-RAW frame, 80
 * for two tiles.  CPU (burst) copy: ~13 ms a single-tile frame and one
 * core busy; scatter-gather DMA in batches of 8: 5 transfers a frame. */
int crx_copy_parts(uint8_t *dst, uint32_t n, const uint32_t *addrs, const uint32_t *sizes)
{
    uint8_t *start = dst;
    uint8_t *bdst[SG_MAX];
    uint32_t bsrc[SG_MAX], blen[SG_MAX];
    int k = 0;
    for (uint32_t i = 1; i <= n; i++)
    {
        if (i < n && sizes[i])
        {
            bdst[k] = dst; bsrc[k] = addrs[i]; blen[k] = sizes[i];
            dst += sizes[i];
            k++;
        }
        if (k == SG_MAX || (i == n && k))
        {
            if (sg_copy(bdst, bsrc, blen, k) != 0)
            {
                for (int j = 0; j < k; j++)
                {
                    copy_burst(bdst[j], (void *) bsrc[j], blen[j]);
                }
            }
            else if (!sg_checked)
            {
                /* sample start/middle/end of each piece: comparing all of a
                 * ~1 MB frame from uncached memory took ~97 ms, the queue
                 * fell behind and two slots were encoded from the same live
                 * Mem1 picture (two identical frames in a row) */
                int same = 1;
                for (int j = 0; j < k && same; j++)
                {
                    uint32_t l = blen[j], c = MIN(64, l);
                    same = memcmp(bdst[j], (void *) bsrc[j], c) == 0
                        && memcmp(bdst[j] + l / 2, (void *) (bsrc[j] + l / 2), MIN(c, l - l / 2)) == 0
                        && memcmp(bdst[j] + l - c, (void *) (bsrc[j] + l - c), c) == 0;
                }
                if (!same)
                {
                    sg_disabled = 1;
                    for (int j = 0; j < k; j++)
                    {
                        copy_burst(bdst[j], (void *) bsrc[j], blen[j]);
                    }
                }
                printf("CRX lossy: scatter-gather DMA %s\n", same ? "checked, ok" : "MISMATCH, CPU copies");
                sg_checked = 1;
            }
            k = 0;
        }
    }
    return dst - start;
}

/* Length of a CRX sample header as LibRaw reads it: tile header, then per
 * plane a plane header and its subband headers (12 bytes each), padded
 * to 8 bytes - 112 for 0 DWT levels, 544 for 3.  0 if it isn't one. */
static int crx_header_len(const uint8_t *h, int max)
{
    if (max < 12 || get_be16(h) != 0xFF01)
    {
        return 0;
    }
    int o = 12;
    while (o + 12 <= max && (get_be16(h + o) == 0xFF02 || get_be16(h + o) == 0xFF03))
    {
        o += 12;
    }
    return (o + 7) & ~7;
}

/* C-RAW (3 DWT levels): the encoder returns Canon's sample header (entry
 * 0, with every subband's size and padding) and two chunks per subband
 * (10 per plane; the second for a wrap of its ring).  Written as Canon's
 * header, padded as LibRaw expects, then all chunks in order. */
static int crx_enc_lossy_result(uint32_t handle, uint8_t *out, int out_max, uint64_t t1)
{
    uint32_t n = 0, *addrs = 0, *csizes = 0;
    if (algs_get_encode_data(handle, &n, &addrs, &csizes) != 0 || n < 2 || n > 200 || !addrs || !csizes || !addrs[0])
    {
        printf("CRX lossy: unreadable result, handle %x n %d\n", handle, n);
        crx_ev(EV_ODD, handle, n, (uint32_t) addrs, (uint32_t) csizes, 0, 0);
        return -5;
    }
    const uint8_t *canon = (const uint8_t *) addrs[0];
    int hdr_used = crx_header_len(canon, csizes[0]);
    uint32_t data = 0;
    for (uint32_t i = 1; i < n; i++)
    {
        if (csizes[i] && addrs[i] < 0x80000000)
        {
            hdr_used = 0;   /* not the encoder's SRM buffers */
        }
        data += csizes[i];
    }
    if (!hdr_used || get_be32(canon + 4) != data)
    {
        static int reported = 0;
        if (reported++ < 3)
        {
            printf("CRX lossy: header %d bytes (%d), tile size %x, chunks %x, n %d\n",
                   hdr_used, csizes[0], hdr_used ? get_be32(canon + 4) : 0, data, n);
        }
        crx_ev(EV_ODD, handle, n, csizes[0], data, 0, 0);
        return -5;
    }
    int total = hdr_used + data;
    crx_ev(EV_GOT, handle, (uint32_t) t1, n, csizes[1], addrs[1], total);
    if (total > out_max)
    {
        crx_ev(EV_TOO_BIG, handle, total, out_max, 0, 0, 0);
        return -4;
    }
    int raw_hdr = MIN((int) csizes[0], hdr_used);
    memcpy(out, canon, raw_hdr);
    memset(out + raw_hdr, 0, hdr_used - raw_hdr);
    crx_copy_parts(out + hdr_used, n, addrs, csizes);
    return total;
}

int crx_enc_frame(void *src, int x_off, void *out, int out_max, int *enc_us, int *copy_us)
{
    if (!job[1] || !crx_sem || !crx_enc_ready())
    {
        return -1;
    }
    if (needs_setup)
    {
        MEM(MEM(ALGS_STATE_PTR) + ALGS_ENC_INITED + crx_enc_id * 4) = 0;
        needs_setup = 0;
    }

    /* EncSetParam (0xE02ACD0C) starts the tile at job word 3 pixels from
     * the source address (plus tile * tile width); Canon's reduced raw
     * uses 8 to skip the left 8 of its 1632-pixel rows */
    job[3] = x_off;
    job[6] = (uint32_t) src;
    cbr_handle = 0;
    job_seq++;
    uint64_t t0 = get_us_clock();
    crx_ev(EV_START, (uint32_t) src, (uint32_t) t0, 0, 0, 0, 0);
    if (algs_start_encode(job, 1, crx_enc_cbr, (void *) job_seq) & 1)
    {
        return -2;
    }
    if (take_semaphore(crx_sem, CRX_TIMEOUT_MS) || !cbr_handle)
    {
        /* its result, if it comes, is freed by the callback */
        crx_ev(EV_TIMEOUT, cbr_handle, (uint32_t) get_us_clock(), 0, 0, 0, 0);
        job_seq++;
        timeouts++;
        printf("CRX: no result after %d ms (%d timeouts)\n", CRX_TIMEOUT_MS, timeouts);
        return -3;
    }
    uint64_t t1 = get_us_clock();

    uint32_t handle = cbr_handle;
    if (crx_enc_id == CRX_ENC_ID_LOSSY)
    {
        int ret = crx_enc_lossy_result(handle, out, out_max, t1);
        uint64_t t2 = get_us_clock();
        crx_ev(EV_FREE, handle, (uint32_t) t2, 0, 0, 0, 0);
        algs_free_enc_buffer(handle, 0, 0);
        if (enc_us) *enc_us = t1 - t0;
        if (copy_us) *copy_us = t2 - t1;
        return ret;
    }
    uint32_t sizes[4];
    uint32_t chunk_addr[8], chunk_size[8];
    int total = CRX_SAMPLE_HDR_SIZE;
    uint32_t n = 0, *addrs = 0, *csizes = 0;
    int ok = (algs_get_encode_data(handle, &n, &addrs, &csizes) == 0) && n == 9 && addrs && csizes;
    for (int i = 0; ok && i < 4; i++)
    {
        sizes[i] = 0;
        for (int k = 0; k < 2; k++)
        {
            chunk_addr[2 * i + k] = addrs[1 + 2 * i + k];
            chunk_size[2 * i + k] = csizes[1 + 2 * i + k];
            sizes[i] += chunk_size[2 * i + k];
            /* the encoder's buffers are SRM in the second GB */
            ok &= chunk_size[2 * i + k] == 0 || chunk_addr[2 * i + k] >= 0x80000000;
        }
        ok &= sizes[i] > 0 && sizes[i] < (uint32_t) out_max;
        total += sizes[i];
    }
    crx_ev(EV_GOT, handle, (uint32_t) t1, n, ok ? sizes[0] : 0, ok ? chunk_addr[0] : 0, ok ? total : 0);
    if (!ok)
    {
        /* skip the frame, but give the handle back: kept, it stops the
         * encoder rewinding its SRM, and it stalls ~130 frames later */
        printf("CRX: unreadable result, handle %x n %d\n", handle, n);
        crx_ev(EV_ODD, handle, n, (uint32_t) addrs, (uint32_t) csizes, 0, 0);
        crx_ev(EV_FREE, handle, (uint32_t) get_us_clock(), 1, 0, 0, 0);
        algs_free_enc_buffer(handle, 0, 0);
        return -5;
    }
    if (chunk_size[1] | chunk_size[3] | chunk_size[5] | chunk_size[7])
    {
        crx_ev(EV_RECOVERED, handle, chunk_size[1], chunk_size[3], chunk_size[5], chunk_size[7], 0);
    }

    int ret = total;
    if (total > out_max)
    {
        /* a lossless frame of ~half the raw size suddenly over it: say
         * what the handle held, to tell bad sizes from bad input */
        printf("CRX: %d bytes > %d: handle %x sizes %x %x %x %x\n",
               total, out_max, handle, sizes[0], sizes[1], sizes[2], sizes[3]);
        crx_ev(EV_TOO_BIG, handle, total, out_max, 0, 0, 0);
        ret = -4;
    }
    else
    {
        uint8_t *dst = out;
        crx_sample_header(dst, sizes);
        int pads = crx_sample_header_pads(dst, (const uint8_t *) addrs[0], csizes[0], sizes);
        static int pads_reported = 0;
        if (pads != 4 && pads_reported < 3)
        {
            /* the clip still records, but Canon's decoder will reject planes */
            pads_reported++;
            printf("CRX: padding from Canon's header for %d of 4 planes (entry 0: %x, %d bytes)\n", pads, addrs[0], csizes[0]);
        }
        dst += CRX_SAMPLE_HDR_SIZE;
        uint8_t *planes_out = dst;
        for (int i = 0; i < 8; i++)
        {
            if (!chunk_size[i])
            {
                continue;
            }
            /* HPCopy returns 1 without copying when no DMA channel is free */
            if (dma_state < 0 || (int) dma_memcpy(dst, (void *) chunk_addr[i], chunk_size[i]) != 0)
            {
                memcpy(dst, (void *) chunk_addr[i], chunk_size[i]);
                if (dma_state >= 0 && (++dma_busy == 1 || dma_busy % 64 == 0))
                {
                    printf("CRX: no DMA channel free (%d planes copied by CPU)\n", dma_busy);
                }
            }
            dst += chunk_size[i];
        }
        if (dma_state == 0)
        {
            dma_state = dma_copy_ok(planes_out, chunk_addr, chunk_size) ? 1 : -1;
            printf("CRX: DMA copy %s\n", dma_state > 0 ? "checked, ok" : "MISMATCH, using CPU copies");
            if (dma_state < 0)
            {
                /* redo this frame with the CPU */
                dst = planes_out;
                for (int i = 0; i < 8; i++)
                {
                    memcpy(dst, (void *) chunk_addr[i], chunk_size[i]);
                    dst += chunk_size[i];
                }
            }
        }
    }
    uint64_t t2 = get_us_clock();

    crx_ev(EV_FREE, handle, (uint32_t) t2, 0, 0, 0, 0);
    algs_free_enc_buffer(handle, 0, 0);

    if (enc_us) *enc_us = t1 - t0;
    if (copy_us) *copy_us = t2 - t1;
    return ret;
}

/* Decode one sample (sample header + planes, as stored in the MLV:
 * lossless or C-RAW) with Canon's CR3 decoder, the way in-camera RAW
 * processing does (InnerDevelop: StartDecodeForInner).  The main header is
 * the CR3 track's CMP1 box: Canon's reduced raw one (single tile) with our
 * size, depth and DWT levels.  Output: rows of pitch_px pixels at `bits`, LSB-first.
 * sample and out must be in the first GB (the decoder's DMA). */
static struct semaphore *crx_dec_sem = NULL;
static volatile uint32_t crx_dec_status = 0;
static uint32_t crx_dec_canon_width = 0;    /* our slot's own width while we use it; 0 = Canon's */
/* the set-up's size and format (bits | DWT levels << 8, in one variable:
 * a separate one grew ML's BSS by 256 bytes, through alignment, and BSS
 * growth has hung the boot before) */
static int crx_dec_w, crx_dec_h, crx_dec_fmt;

/* While we decode, Canon's assert on a decode error (ERR70) is patched
 * out: the frame is just lost (our callback isn't called: timeout) */
static int crx_dec_assert_off = 0;     /* 1 patched, -1 failed */
static struct patch crx_dec_assert_patch = {
    .addr = (uint8_t *) ALGS_DECERR_ASSERT_BL,
    .old_value = ALGS_DECERR_ASSERT_BL_OLD,
    .new_value = 0xBF00BF00,            /* nop; nop */
    .size = 4,
    .description = "crx_dec: no assert on a decode error",
    .is_instruction = 1,
};
#define CRX_DEC_TIMEOUT_MS 300          /* a decode takes ~17 ms */

/* per-frame results, written to ML/LOGS/CRXDEC.LOG by crx_dec_done():
 * which frames decode, against their plane sizes.  Allocated on first use:
 * as a static buffer its 24 kB in ML's BSS left Canon too little memory and
 * the camera hung at boot (LED on). */
#define CRX_DEC_LOG_SIZE (24 * 1024)
static char *crx_dec_log = NULL;
static int crx_dec_log_len = 0;
static int crx_dec_log_frame = 0;
#define crx_dec_logf(...) do { if (!crx_dec_log) crx_dec_log = malloc(CRX_DEC_LOG_SIZE); \
    if (crx_dec_log && crx_dec_log_len < CRX_DEC_LOG_SIZE - 128) \
    crx_dec_log_len += snprintf(crx_dec_log + crx_dec_log_len, CRX_DEC_LOG_SIZE - crx_dec_log_len, __VA_ARGS__); } while (0)

/* Canon calls this only when a decode succeeded (an error goes to its
 * assert instead), as cbr(ctx, out, ...): the second argument is the
 * output buffer, not a status */
static void crx_dec_cbr(void *ctx, void *out)
{
    crx_dec_status = (uint32_t) out;
    give_semaphore(crx_dec_sem);
}

/* free bytes in the CRaw work arena, or -1 if it can't be read */
static int craw_arena_free(void)
{
    uint32_t a = MEM(CRAW_GLOBALS_PTR);
    if (!a)
    {
        return -1;
    }
    return (int)(MEM(a + 0x8) + MEM(a + 0xC) - MEM(a + 0x14));
}

#define CRX_DEC_ALIGN 128

/* DecInit's slot for our frames: C-RAW widths are looked up in a table of
 * their own (consts.h) */
#define CRX_DEC_SLOT(levels)      ((levels) ? ALGS_DEC_SLOT7 : ALGS_DEC_SLOT5)
#define CRX_DEC_WIDTH_OFF(levels) ((levels) ? ALGS_DEC_WIDTH_SLOT7 : ALGS_DEC_WIDTH_SLOT5)

/* give the slot we use back to Canon (set up again on its next use) */
static void crx_dec_give_back(void)
{
    uint32_t st = MEM(ALGS_STATE_PTR);
    int levels = crx_dec_fmt >> 8;
    MEM(st + CRX_DEC_WIDTH_OFF(levels)) = crx_dec_canon_width;
    MEM(st + ALGS_ENC_INITED + CRX_DEC_SLOT(levels) * 4) = 0;
    crx_dec_canon_width = 0;
    crx_dec_w = 0;
}

/* copy len bytes to a higher, overlapping address (no memmove in ML) */
static void move_up(uint8_t *dst, const uint8_t *src, int len)
{
    if (!(((uintptr_t) dst | (uintptr_t) src | len) & 3))
    {
        uint32_t *d = (uint32_t *) dst;
        const uint32_t *v = (const uint32_t *) src;
        for (int i = len / 4 - 1; i >= 0; i--) d[i] = v[i];
        return;
    }
    for (int i = len - 1; i >= 0; i--) dst[i] = src[i];
}

/* The decoder fetches each subband by DMA from an aligned address (DMLOG22:
 * planes at ...f80 and ...300 decoded, at ...130 and ...958 failed, so 32,
 * 64 or 128 bytes; 128 covers all).  CRawDirectDecAnalizeSample
 * (0xE02FFCF0) lists one transfer per subband (planes x subbands), so a
 * C-RAW sample (10 subbands per plane) needs each of its 40 aligned, as a
 * lossless one (1 per plane) its 4. */
#define CRX_DEC_MAX_SUBBANDS 40         /* 4 planes, 3 DWT levels */

/* The sample's headers: tile (ff01), then per plane ff02 and its subbands
 * (ff03), 12 bytes each, padded to 8 (Canon's mdatHdrSize: 0x70 lossless,
 * 0x220 for 3 levels).  Returns the header size and the levels, or -1 if
 * the header isn't one we can decode. */
static int crx_dec_parse(const uint8_t *s, int size, int *levels, uint8_t **sub, int *plane_of)
{
    if (size < 12 || get_be16(s) != 0xFF01)
    {
        return -1;
    }
    int h = 12, planes = 0, n = 0;
    while (h + 12 <= size && s[h] == 0xFF && (s[h + 1] == 0x02 || s[h + 1] == 0x03))
    {
        if (s[h + 1] == 0x02)
        {
            if (planes == 4) return -1;
            planes++;
        }
        else
        {
            if (!planes || n == CRX_DEC_MAX_SUBBANDS) return -1;
            sub[n] = (uint8_t *) s + h;
            plane_of[n++] = planes - 1;
        }
        h += 12;
    }
    /* 3 * levels + 1 subbands per plane; Canon's ParseMainHeader allows < 4 levels */
    if (planes != 4 || n % 4 || (n / 4 - 1) % 3)
    {
        return -1;
    }
    *levels = (n / 4 - 1) / 3;
    return (h + 7) & ~7;
}

/* Move subbands 1..n-1 up to CRX_DEC_ALIGN boundaries (subband 0 is at
 * sample + header size, aligned by the caller), zero-filling each gap and
 * counting it as padding at the end of the subband before (ff03 flags,
 * low 11 bits; LibRaw: dataSize = size - (flags & 0x7FF)), with the plane
 * (ff02) and tile (ff01) sizes to match.  In place, last subband first;
 * needs (n - 1) * (CRX_DEC_ALIGN - 1) bytes after the sample.  Returns the
 * new size, or -1 if the sizes don't fit. */
static int crx_dec_plane_len[4];     /* as recorded, before padding */

static int crx_dec_pad_subbands(uint8_t *s, int size, int hdr, int n, uint8_t **sub, const int *plane_of)
{
    int old_off[CRX_DEC_MAX_SUBBANDS], new_off[CRX_DEC_MAX_SUBBANDS], len[CRX_DEC_MAX_SUBBANDS], pad[CRX_DEC_MAX_SUBBANDS];
    int o = hdr, d = hdr;
    for (int p = 0; p < 4; p++)
    {
        crx_dec_plane_len[p] = 0;
    }
    for (int i = 0; i < n; i++)
    {
        len[i] = get_be32(sub[i] + 4);
        crx_dec_plane_len[plane_of[i]] += len[i] - (get_be32(sub[i] + 8) & 0x7FF);
        old_off[i] = o; new_off[i] = d;
        pad[i] = (i < n - 1) ? (-(d - hdr + len[i])) & (CRX_DEC_ALIGN - 1) : 0;
        if ((get_be32(sub[i] + 8) & 0x7FF) + pad[i] > 0x7FF)
        {
            return -1;
        }
        o += len[i]; d += len[i] + pad[i];
    }
    if (o > size)
    {
        return -1;
    }
    for (int i = n - 1; i >= 1; i--)
    {
        if (new_off[i] != old_off[i])
        {
            move_up(s + new_off[i], s + old_off[i], len[i]);
        }
    }
    uint32_t plane_size[4] = { 0 };
    for (int i = 0; i < n; i++)
    {
        memset(s + new_off[i] + len[i], 0, pad[i]);
        put_be32(sub[i] + 4, len[i] + pad[i]);
        uint32_t flags = get_be32(sub[i] + 8);
        put_be32(sub[i] + 8, (flags & ~0x7FF) | ((flags & 0x7FF) + pad[i]));
        plane_size[plane_of[i]] += len[i] + pad[i];
    }
    /* each plane header is just before its first subband's */
    for (int i = 0; i < n; i++)
    {
        if (i == 0 || plane_of[i] != plane_of[i - 1])
        {
            put_be32(sub[i] - 12 + 4, plane_size[plane_of[i]]);
        }
    }
    put_be32(s + 4, plane_size[0] + plane_size[1] + plane_size[2] + plane_size[3]);
    return d;
}

int crx_dec_frame(void *sample, int size, void *out, int width, int height, int bits, int pitch_px)
{
    /* the whole CMP1 box content: its length field (bytes 2-3, 0x30) + 4;
     * Canon's check (0xE0303D24) rejects a shorter size, and DecInit then
     * asserts (the first two ERR70s: 48 bytes passed) */
    static const uint8_t cmp1_reduced_raw[52] = {
        0xff, 0x00, 0x00, 0x30, 0x01, 0x00, 0x00, 0x00,  0x00, 0x00, 0x06, 0x58, 0x00, 0x00, 0x04, 0x38,
        0x00, 0x00, 0x06, 0x58, 0x00, 0x00, 0x04, 0x38,  0x0e, 0x41, 0x00, 0x00, 0x00, 0x00, 0x00, 0x70,
        0x00, 0x00, 0x00, 0x00, 0x01, 0x01, 0x00, 0x00,  0x01, 0x01, 0x00, 0x00, 0x01, 0x01, 0x00, 0x00,
        0x01, 0x01, 0x00, 0x00,
    };
    static uint8_t cmp1[52] __attribute__((aligned(32)));
    uint8_t *sub[CRX_DEC_MAX_SUBBANDS];
    int plane_of[CRX_DEC_MAX_SUBBANDS];

    if (!crx_enc_ready() || width < 64 || height < 64 || !sample || !out)
    {
        return -1;
    }
    int levels;
    int hdr = crx_dec_parse(sample, size, &levels, sub, plane_of);
    if (hdr < 0 || size <= hdr)
    {
        return -2;
    }
    /* the decoder reads the subbands by DMA, which ignores the low address
     * bits: a sample at ...a4 failed every plane (DecErr, then Canon's
     * assert: ERR70) */
    if (((uintptr_t) sample + hdr) & (CRX_DEC_ALIGN - 1) || ((uintptr_t) out & (CRX_DEC_ALIGN - 1)))
    {
        return -5;
    }
    if (!crx_dec_assert_off)
    {
        crx_dec_assert_off = apply_patches(&crx_dec_assert_patch, 1) == E_PATCH_OK ? 1 : -1;
        printf("CRX decode: assert patch %s\n", crx_dec_assert_off > 0 ? "on" : "FAILED");
        crx_dec_logf("assert patch %s\n", crx_dec_assert_off > 0 ? "on" : "FAILED (Canon debug log hooks use the one MMU page?)");
    }
    /* without the patch (Canon debug log on: dmlog's hook has the one spare
     * MMU page) decode anyway: a decode error is then ERR70, and dmlog
     * saves Canon's decoder messages on its dialog */
    /* the first frames' steps on ML's console too: readable over USB
     * (ptp_ml.py console) if the camera hangs in the decoder */
    int verbose = crx_dec_log_frame < 3;
    if (verbose)
    {
        printf("CRX decode %d: %x+%d hdr %x L%d, padding\n", crx_dec_log_frame, sample, size, hdr, levels);
    }
    int t_pad = get_ms_clock();
    size = crx_dec_pad_subbands(sample, size, hdr, 4 * (3 * levels + 1), sub, plane_of);
    t_pad = get_ms_clock() - t_pad;
    if (size < 0)
    {
        return -2;
    }
    if (!crx_dec_sem)
    {
        crx_dec_sem = create_named_semaphore("crx_dec", 0);
    }

    memcpy(cmp1, cmp1_reduced_raw, sizeof(cmp1));
    put_be32(cmp1 + 8, width);
    put_be32(cmp1 + 12, height);
    put_be32(cmp1 + 16, width);
    put_be32(cmp1 + 20, height);
    cmp1[24] = bits;
    /* C-RAW: Canon's C-RAW reduced raw CMP1 (IMG_9876) is the same but for
     * the levels (byte 26: encType 0, 3 levels) and mdatHdrSize (0x220),
     * which the decoder takes as the sample header's length */
    cmp1[26] = levels;
    put_be32(cmp1 + 28, hdr);

    /* The single-tile reduced raw decoder slot (5 lossless, 7 C-RAW) is
     * chosen by width.  Make it ours and set it up once per clip: every set-up takes tables from
     * the CRaw arena and none are given back, so one per frame used it up
     * after a few dozen frames (ERR70 at AlgsMgrExec.c:2130). */
    uint32_t st = MEM(ALGS_STATE_PTR);
    if (crx_dec_canon_width && !(crx_dec_fmt >> 8) != !levels)
    {
        /* lossless after C-RAW or the other way round: the other slot */
        crx_dec_give_back();
    }
    uint32_t *slot_width = (uint32_t *)(st + CRX_DEC_WIDTH_OFF(levels));
    uint32_t *slot_inited = (uint32_t *)(st + ALGS_ENC_INITED + CRX_DEC_SLOT(levels) * 4);
    if (!crx_dec_canon_width)
    {
        crx_dec_canon_width = *slot_width;
        crx_dec_fmt = bits | levels << 8;
        crx_dec_w = 0;
    }
    if (*slot_width != (uint32_t) width || crx_dec_w != width || crx_dec_h != height || crx_dec_fmt != (bits | levels << 8) || !*slot_inited)
    {
        /* a set-up follows: refuse rather than let Canon assert */
        int free_bytes = craw_arena_free();
        printf("CRX decode: set up %dx%d %d-bit, %d levels, CRaw arena %d bytes free\n", width, height, bits, levels, free_bytes);
        if (free_bytes < 8192)
        {
            return -4;
        }
        *slot_width = width;
        *slot_inited = 0;
        crx_dec_w = width; crx_dec_h = height; crx_dec_fmt = bits | levels << 8;
    }

    /* into Canon's debug log too (dmlog), next to its own decoder messages */
    const uint8_t *h = sample;
    DryosDebugMsg(0, 15, "crx_dec: sample %x+%d (%02x%02x %02x%02x%02x%02x) out %x pitch %d, %dx%d %d-bit, arena %d free",
                  sample, size, h[0], h[1], h[4], h[5], h[6], h[7], out, pitch_px, width, height, bits, craw_arena_free());

    if (verbose)
    {
        printf("CRX decode %d: start\n", crx_dec_log_frame);
    }
    crx_dec_status = 0xFFFFFFFF;
    /* output: rows of pitch_px pixels packed at `bits`, LSB-first - checked
     * on the camera: bit-identical to LibRaw's decode of the same sample */
    algs_start_decode(sample, size, out, pitch_px, 0, 0, cmp1, sizeof(cmp1), crx_dec_cbr, 0, 0, 0);
    int t0 = get_ms_clock();
    int timed_out = take_semaphore(crx_dec_sem, CRX_DEC_TIMEOUT_MS);
    if (verbose || timed_out)
    {
        printf("CRX decode %d: %s after %d ms\n", crx_dec_log_frame, timed_out ? "no result" : "done", get_ms_clock() - t0);
    }
    crx_dec_logf("%4d %dx%d %d-bit L%d planes %7d %7d %7d %7d  %s %d ms (pad %d ms)\n", crx_dec_log_frame++, width, height, bits, levels,
                 crx_dec_plane_len[0], crx_dec_plane_len[1], crx_dec_plane_len[2], crx_dec_plane_len[3],
                 timed_out ? "ERROR" : (void *) crx_dec_status == out ? "ok" : "ok?", get_ms_clock() - t0, t_pad);
    if (timed_out)
    {
        DryosDebugMsg(0, 15, "crx_dec: no result after %d ms (decode error)", CRX_DEC_TIMEOUT_MS);
        return -3;
    }
    return 0;
}

/* recording done: Canon sets its encoders up for its own sizes again */
void crx_enc_done(void)
{
    /* an encoder AlgsMgr set up keeps our size and depth until its flag is
     * cleared; Canon's CR3 stills use encoder 1 for the reduced raw */
    uint32_t st = MEM(ALGS_STATE_PTR);
    if (st && job[1])
    {
        MEM(st + ALGS_ENC_INITED + CRX_ENC_ID * 4) = 0;
        MEM(st + ALGS_ENC_INITED + CRX_ENC_ID_LOSSY * 4) = 0;
    }
    needs_setup = 1;
}

void crx_dec_done(void)
{
    if (crx_dec_canon_width)
    {
        crx_dec_give_back();
        printf("CRX decode: done, CRaw arena %d bytes free\n", craw_arena_free());
    }
    if (crx_dec_assert_off > 0)
    {
        unpatch_memory(ALGS_DECERR_ASSERT_BL);
    }
    crx_dec_assert_off = 0;
    if (crx_dec_log_len)
    {
        FILE *f = FIO_CreateFile("ML/LOGS/CRXDEC.LOG");
        if (f)
        {
            FIO_WriteFile(f, crx_dec_log, crx_dec_log_len);
            FIO_CloseFile(f);
        }
        crx_dec_log_len = 0;
        crx_dec_log_frame = 0;
    }
    if (crx_dec_log)
    {
        free(crx_dec_log);
        crx_dec_log = NULL;
    }
}

#else

int crx_enc_available(void) { return 0; }
int crx_enc_ready(void) { return 0; }
int crx_enc_setup(int width, int height, int bits, int pitch_px, int lossy) { return -1; }
int crx_enc_frame(void *src, int x_off, void *out, int out_max, int *enc_us, int *copy_us) { return -1; }
int crx_dec_frame(void *sample, int size, void *out, int width, int height, int bits, int pitch_px) { return -1; }
void crx_dec_done(void) { }
void crx_enc_done(void) { }

#endif
