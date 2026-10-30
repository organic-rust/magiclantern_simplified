/* M6II: Canon's movie recorder work memory as ML's "SRM" suite.
 *
 * SRM_AllocateMemoryResourceFor1stJob gives nothing on this camera
 * (CONFIG_MEMORY_SRM_NOT_WORKING).  Instead, at every movie mode LiveView
 * start Canon's MovRecResource takes "MovieRecWorkDivide" from SRM (two
 * areas, L 252 MB at 0x4CAE0000 and U 26 MB at 0x68090000) and frees it
 * when LiveView stops.  L is cut into bump-allocator pools that only the
 * H.264 encoder and the movie writer allocate from, at Canon's record
 * start; LiveView only reserves the encoder's area at the top of "recw".
 * With raw video on Canon never records (mlv_lite blocks its REC key and
 * turns its H.264 proxy off where this suite exists), so the free part of
 * "recw", the whole "writ" pool and U are idle: checked
 * over USB with markers across LiveView and raw takes.  Free: recw 81 MB
 * in 4K and 186 MB in 1080p (the encoder's reservation depends on the
 * movie size), writ 34 MB, U 26 MB.
 *
 * recw and writ are taken with Canon's own allocator (MRAlloc), so the
 * pools show them as used; U belongs to no pool in this layout and is
 * taken as it is.  Nothing in Canon frees single blocks: the pools are
 * reset when the work memory is set up again, so freeing only rewinds a
 * pool if nothing came after us.  The memory is Canon's again when
 * LiveView stops: hold the suite only while recording in LiveView. */

#include "dryos.h"
#include "mem.h"
#include "exmem.h"

/* MovRecResource state (pointer at 0x10218) */
#define MOVREC_STATE_PTR    0x10218
#define ST_RECWORK_DONE     0x04    /* 1 after mrrscAllocateRecWork */
#define ST_SEM              0x28
#define ST_U_ADDR           0x38
#define ST_U_SIZE           0x40
#define ST_POOL_RECW        0x74    /* the "recw" pool in this layout */
#define ST_POOL_ALT1        0x78    /* pools of the other layouts, which use U */
#define ST_POOL_ALT2        0x7C
#define ST_POOL_WRIT        0x80
#define ST_LAYOUT_A         0x88    /* layout flags: all 0 for MovieRecWorkDivide */
#define ST_LAYOUT_B         0x8C
#define ST_LAYOUT_C         0x90

/* pool: tag, base, used, size, reserved areas */
#define POOL_TAG            0x00
#define POOL_BASE           0x04
#define POOL_USED           0x08
#define POOL_SIZE           0x0C

/* MRAlloc pool indices (0xE0263D88) */
#define MRPOOL_WRIT         1
#define MRPOOL_RECW         2

#define TAG_RECW            0x72656377  /* 'recw' */
#define TAG_WRIT            0x77726974  /* 'writ' */

#define MIN_BLOCK           (1024 * 1024)

/* Canon asserts (camera halt) if the pool is short: check first */
extern void *MovRecAlloc(int pool, uint32_t size, uint32_t align, const char *file, int line);

static struct
{
    uint32_t pool;      /* pool struct, 0 = not taken */
    uint32_t used_before;
    uint32_t used_after;
} taken[2];

static void *take_from_pool(int k, uint32_t st, uint32_t pool_off, int mr_pool, uint32_t tag, uint32_t *size)
{
    uint32_t pool = MEM(st + pool_off);
    if (!pool || MEM(pool + POOL_TAG) != tag)
    {
        return NULL;
    }

    /* MRAlloc may round the start up to the alignment */
    uint32_t free_space = MEM(pool + POOL_SIZE) - MEM(pool + POOL_USED);
    if (free_space < MIN_BLOCK + 0x1000)
    {
        return NULL;
    }
    uint32_t want = (free_space - 0x1000) & ~0xFFF;

    uint32_t used_before = MEM(pool + POOL_USED);
    void *ptr = MovRecAlloc(mr_pool, want, 0x40, __FILE__, __LINE__);
    if (!ptr)
    {
        return NULL;
    }
    taken[k].pool = pool;
    taken[k].used_before = used_before;
    taken[k].used_after = MEM(pool + POOL_USED);
    *size = want;
    return ptr;
}

struct memSuite *movrec_malloc_suite(void)
{
    uint32_t st = MEM(MOVREC_STATE_PTR);
    if (!st || MEM(st + ST_RECWORK_DONE) != 1)
    {
        printf("movrec: no work memory (not in movie LiveView?)\n");
        return NULL;
    }
    if (MEM(st + ST_LAYOUT_A) || MEM(st + ST_LAYOUT_B) || MEM(st + ST_LAYOUT_C) ||
        MEM(st + ST_POOL_ALT1) || MEM(st + ST_POOL_ALT2))
    {
        printf("movrec: unknown work memory layout\n");
        return NULL;
    }
    if (taken[0].pool || taken[1].pool)
    {
        printf("movrec: already taken\n");
        return NULL;
    }
    /* LiveView leaves "writ" untouched; anything in it is Canon's movie
     * writer, i.e. Canon is recording (e.g. started from photo mode): its
     * pools are short if we take the rest, and Canon asserts */
    uint32_t writ = MEM(st + ST_POOL_WRIT);
    if (writ && MEM(writ + POOL_TAG) == TAG_WRIT && MEM(writ + POOL_USED))
    {
        printf("movrec: Canon's movie recorder is using its memory\n");
        return NULL;
    }

    struct memSuite *suite = NULL;
    void *ptr[3];
    uint32_t size[3] = {0};
    ptr[0] = take_from_pool(0, st, ST_POOL_RECW, MRPOOL_RECW, TAG_RECW, &size[0]);
    ptr[1] = take_from_pool(1, st, ST_POOL_WRIT, MRPOOL_WRIT, TAG_WRIT, &size[1]);
    ptr[2] = (void *) MEM(st + ST_U_ADDR);
    size[2] = MEM(st + ST_U_SIZE);
    if (size[2] < MIN_BLOCK)
    {
        ptr[2] = NULL;
    }

    for (int i = 0; i < 3; i++)
    {
        if (!ptr[i])
        {
            continue;
        }
        printf("movrec: %x, %s\n", ptr[i], format_memory_size(size[i]));
        if (!suite)
        {
            suite = CreateMemorySuite(ptr[i], size[i], 0);
        }
        else
        {
            struct memChunk *chunk = CreateMemoryChunk(ptr[i], size[i], 0);
            if (chunk)
            {
                AddMemoryChunk(suite, chunk);
            }
        }
    }
    return suite;
}

void movrec_free_suite(struct memSuite *suite)
{
    if (suite)
    {
        DeleteMemorySuite(suite);
    }

    /* rewind the pools if the work memory is still the one we took from
     * and nothing was allocated after us */
    uint32_t st = MEM(MOVREC_STATE_PTR);
    struct semaphore *sem = st ? (struct semaphore *) MEM(st + ST_SEM) : NULL;
    if (sem)
    {
        take_semaphore(sem, 0);
    }
    for (int k = 0; k < 2; k++)
    {
        uint32_t pool = taken[k].pool;
        if (pool && st && MEM(st + ST_RECWORK_DONE) == 1 &&
            (MEM(st + ST_POOL_RECW) == pool || MEM(st + ST_POOL_WRIT) == pool) &&
            MEM(pool + POOL_USED) == taken[k].used_after)
        {
            MEM(pool + POOL_USED) = taken[k].used_before;
        }
        taken[k].pool = 0;
    }
    if (sem)
    {
        give_semaphore(sem);
    }
}
