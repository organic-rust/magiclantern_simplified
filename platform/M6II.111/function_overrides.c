/** \file
 * Function overrides needed for R 1.8.0
 */
/*
 * Copyright (C) 2021 Magic Lantern Team
 *
 * This program is free software; you can redistribute it and/or
 * modify it under the terms of the GNU General Public License
 * as published by the Free Software Foundation; either version 2
 * of the License, or (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the
 * Free Software Foundation, Inc.,
 * 51 Franklin Street, Fifth Floor,
 * Boston, MA  02110-1301, USA.
 */

#include <dryos.h>
#include <property.h>
#include <bmp.h>
#include <config.h>
#include <consts.h>
#include <lens.h>
#include <edmac.h>

/*
 * Partition tables stuff. Got inlined in new generations, but this is a pretty standard one.
 * Struct copied from bootflags.c as there's no header to include right now.
 */

struct chs_entry
{
    uint8_t head;
    uint8_t sector; //sector + cyl_msb
    uint8_t cyl_lsb;
}__attribute__((packed));

struct partition
{
    uint8_t  state;
    struct   chs_entry start;
    uint8_t  type;
    struct   chs_entry end;
    uint32_t start_sector;
    uint32_t size;
}__attribute__((aligned,packed));

struct partition_table
{
    uint8_t  state; // 0x80 = bootable
    uint8_t  start_head;
    uint16_t start_cylinder_sector;
    uint8_t  type;
    uint8_t  end_head;
    uint16_t end_cylinder_sector;
    uint32_t sectors_before_partition;
    uint32_t sectors_in_partition;
}__attribute__((packed));

void fsuDecodePartitionTable(void * partIn, struct partition_table * pTable){
    struct partition * part = (struct partition *) partIn;
    pTable->state      = part->state;
    pTable->type       = part->type;
    pTable->start_head = part->start.head;
    pTable->end_head   = part->end.head;
    pTable->sectors_before_partition = part->start_sector;
    pTable->sectors_in_partition     = part->size;

    //tricky bits - TBD
    pTable->start_cylinder_sector = 0;
    pTable->end_cylinder_sector   = 0;
}

/** GUI **/
//see comments in stub.S
void gui_init_end(void){ }

//Not sure if all args are uint.
extern void gui_enqueue_message(uint32_t, uint32_t, uint32_t, uint32_t);
void GUI_Control(int bgmt_code, int obj, int arg, int unknown){
    gui_enqueue_message(0, bgmt_code, obj, arg);
}

void LoadCalendarFromRTC(struct tm *tm)
{
    _LoadCalendarFromRTC(tm, 0, 0, 16);
}

int get_task_info_by_id(int unknown_flag, int task_id, void *task_attr)
{
    // task_id is something like two u16s concatenated.  The flag argument,
    // present on D45 but not on D678 allows controlling if the task info request
    // uses the whole thing, or only the low half.
    //
    // ML calls with this set to 1, meaning task_id is used as is,
    // if 0, the high half is masked out first.
    //
    // D678 doesn't have the 1 option, we use the low half as index
    // to find the full value.
    struct task *task = first_task + (task_id & 0xffff);
    return _get_task_info_by_id(task->taskId, task_attr);
}

/** File I/O **/

/**
 * _FIO_GetFileSize returns now 64bit size in form of struct.
 * This probably should be integrated into fio-ml for CONFIG_DIGIC_VIII
 */
extern int _FIO_GetFileSize64(const char *, void *);
int _FIO_GetFileSize(const char * filename, uint32_t * size){
    uint32_t size64[2];
    int code = _FIO_GetFileSize64(filename, &size64);
    *size = size64[0]; //return "lower" part
    return code;
}

/** EDMAC **/
/* D8 wrappers around Canon's Engine::Edmac.c, giving the D4/D5 API.
 * Stubs and channel map: stubs.S, include/platform/edmac_channels.h.
 * None of this is tested on camera yet. */
#include <platform/edmac_channels.h>

extern void edmac_set_addr(unsigned int channel, void *address);
extern void edmac_set_size(unsigned int channel, struct edmac_info *info);
extern void _StartEDmac(unsigned int channel);
extern void edmac_read_kick(unsigned int channel);
extern void _AbortEDmac(unsigned int channel, void (*cbr)(void*), void *cbr_ctx);
extern void _ConnectWriteEDmac(unsigned int channel, unsigned int conn);
extern void _ConnectReadEDmac(unsigned int channel, unsigned int conn);

/* FIXME: flags are ignored.  D8 has no D4/D5-style flags register; the
 * nearest equivalent is transfer_mode (+0xC0), set by separate Canon calls
 * that are not identified yet. */
void SetEDmac(unsigned int channel, void *address, struct edmac_info *ptr, int flags)
{
    edmac_set_addr(channel, address);
    edmac_set_size(channel, ptr);
}

/* Canon asserts unless the connection value carries the channel's mask bit.
 * FIXME: which "where" values are meaningful on D8 is not known yet. */
void ConnectWriteEDmac(unsigned int channel, unsigned int where)
{
    if (channel >= COUNT(edmac_channels)
        || !(edmac_channels[channel].flags & EDMAC_CHAN_FLAG_WRITE))
    {
        return;
    }
    _ConnectWriteEDmac(channel, edmac_channels[channel].conn_mask | (where & 0xFF));
}

void ConnectReadEDmac(unsigned int channel, unsigned int where)
{
    if (channel >= COUNT(edmac_channels)
        || !(edmac_channels[channel].flags & EDMAC_CHAN_FLAG_READ)
        || where > 0x18)
    {
        return;
    }
    _ConnectReadEDmac(channel, where);
}

/* FIXME: D8 start takes no flags; D4/D5 callers pass e.g. continuous mode.
 * Canon's mem-to-mem start (0xe0639952) follows start on a read channel
 * with a kick, so do the same. */
void StartEDmac(unsigned int channel, int flags)
{
    _StartEDmac(channel);

    if (channel < COUNT(edmac_channels)
        && (edmac_channels[channel].flags & EDMAC_CHAN_FLAG_READ))
    {
        edmac_read_kick(channel);
    }
}

static void abort_edmac_cbr(void *ctx)
{
}

/* D8 abort completes asynchronously, calling cbr from the EDMAC interrupt.
 * D4/D5 callers don't wait for it, so a no-op cbr keeps the same behaviour. */
void AbortEDmac(unsigned int channel)
{
    _AbortEDmac(channel, abort_edmac_cbr, 0);
}

/** EDMAC mem -> mem copy, for mlv_lite **/
/* Canon's Engine::MemoryToMemoryEsub5.c, used the way Canon's own
 * dark-current OB copy (0xe061735e) uses it: read 61 (5_RD_SS0) ->
 * write 24 (5_WR_S0), locking { 0x5001F, 0x50023 }, copy mode 1, and
 * PwrMng wake/sleep around the copy.  Canon's Vfx path (0xe0080498)
 * uses the same channels and lock.  mlv_lite's uncompressed takes copy
 * their frames with it (correct frames on camera).  Open: ML masks the
 * addresses with 0x1FFFFFFF, though this camera has no 512 MB alias of
 * its RAM (0x04400000 and 0x24400000 differ). */
#include <edmac-memcpy.h>

struct m2m_channels
{
    uint32_t read;
    uint32_t write;
};

struct m2m_addrs
{
    void *src;
    void *dst;
};

struct m2m_copy
{
    struct edmac_info *src;
    struct edmac_info *dst;
    uint32_t mode;
};

extern int DeleteResLockEntry(struct LockEntry *lockEntry);
extern void mem_to_mem_setup_channels(struct m2m_channels *channels);
extern void mem_to_mem_set_src_dst(struct m2m_addrs *addrs);
extern void mem_to_mem_setup_copy_d8(struct m2m_copy *copy);
extern void mem_to_mem_start(void);
extern void pwrmng_wake_devices(void *desc);
extern void pwrmng_sleep_devices(void *desc);

#define M2M_READ_CHANNEL  61
#define M2M_WRITE_CHANNEL 24
#define M2M_COPY_MODE     1          /* Canon's value; 0..3, gated by channel flags */
#define M2M_PWRMNG_DESC   0xE11753CC /* what the dark-current copy wakes */

static struct semaphore *edmac_copy_sem = NULL;
static struct semaphore *mem2mem_sem = NULL;
static struct LockEntry *mem2mem_lock = NULL;

/* mlv_lite's write cbr calls this when the copy is done.  That's in
 * interrupt context, where cleanup would assert, so only release the
 * waiting edmac_copy_rectangle_cbr_start(). */
void edmac_copy_rectangle_adv_cleanup()
{
    give_semaphore(mem2mem_sem);
}

/* Blocking: returns after the copy has finished and the engine is released.
 * cbr_w is called from interrupt context when both channels complete;
 * cbr_r is not used (the engine only reports completion of both). */
void* edmac_copy_rectangle_cbr_start(void *dst, void *src,
                                     int src_width, int src_x, int src_y,
                                     int dst_width, int dst_x, int dst_y,
                                     int w, int h,
                                     void (*cbr_r)(void *), void (*cbr_w)(void *), void *cbr_ctx)
{
    if ((src == NULL) || (dst == NULL))
    {
        ASSERT(0);
        return NULL;
    }

    /* Canon's linear setup asserts 4-byte multiples, and its LV channel
     * writes 4-byte multiple rows (1080p 12-bit: 3492 bytes); its own
     * copies here use 8-byte multiples (0x30C8).  D4/D5 could fail to
     * stop on badly sized transfers, so nothing smaller. */
    if ((w % 4) || h <= 0)
    {
        DryosDebugMsg(0, 15, "m2m: unsupported copy size %d x %d", w, h);
        return NULL;
    }

    /* we write through DMA, so the destination must not be cached */
    ASSERT(dst == UNCACHEABLE(dst));

    /* clean the cache before reading from regular (cacheable) memory */
    if (src == CACHEABLE(src))
    {
        sync_caches();
    }

    void *src_adjusted = (void *)((uint32_t)src + src_x + src_y * src_width);
    void *dst_adjusted = (void *)((uint32_t)dst + dst_x + dst_y * dst_width);

    if (mem2mem_sem == NULL)
        mem2mem_sem = create_named_semaphore("mem2mem", SEM_CREATE_LOCKED);
    if (edmac_copy_sem == NULL)
        edmac_copy_sem = create_named_semaphore("edmac_copy", SEM_CREATE_UNLOCKED);
    if (take_semaphore(edmac_copy_sem, 1000))
    {
        DryosDebugMsg(0, 15, "m2m: failed to take edmac_copy_sem");
        return NULL;
    }

    if (mem2mem_lock == NULL)
    {
        uint32_t res_ids[] = { 0x5001F, 0x50023 };
        mem2mem_lock = CreateResLockEntry(res_ids, COUNT(res_ids));
        if (mem2mem_lock == NULL)
        {
            give_semaphore(edmac_copy_sem);
            return NULL;
        }
    }

    /* blocks while Canon is using the engine */
    if (LockEngineResources(mem2mem_lock) != 0)
    {
        DryosDebugMsg(0, 15, "m2m: LockEngineResources failed");
        give_semaphore(edmac_copy_sem);
        return NULL;
    }
    pwrmng_wake_devices((void *)M2M_PWRMNG_DESC);

    struct m2m_channels channels = {
        .read  = M2M_READ_CHANNEL,
        .write = M2M_WRITE_CHANNEL,
    };
    mem_to_mem_setup_channels(&channels);
    set_mem_to_mem_cbr(cbr_w, cbr_ctx);

    /* xb is width in bytes, yb is height - 1;
     * off1b is the number of bytes to skip after every xb bytes */
    struct edmac_info src_region = {
        .xb    = w,
        .yb    = h - 1,
        .off1b = src_width - w,
    };

    struct edmac_info dst_region = {
        .xb    = w,
        .yb    = h - 1,
        .off1b = dst_width - w,
    };

    struct m2m_addrs addrs = {
        .src = src_adjusted,
        .dst = dst_adjusted,
    };
    struct m2m_copy copy = {
        .src  = &src_region,
        .dst  = &dst_region,
        .mode = M2M_COPY_MODE,
    };
    mem_to_mem_set_src_dst(&addrs);
    mem_to_mem_setup_copy_d8(&copy);
    mem_to_mem_start();

    /* cbr_w -> edmac_copy_rectangle_adv_cleanup() gives this */
    int timed_out = take_semaphore(mem2mem_sem, 1000);
    if (timed_out)
    {
        /* FIXME: if cbr_w still fires later, it leaves mem2mem_sem given
         * and the next copy won't wait.  Also unknown: whether cleanup is
         * safe while the channels may still be running. */
        DryosDebugMsg(0, 15, "m2m: copy timed out");
    }

    /* Canon's order: default cbr, cleanup channels, sleep, unlock.
     * Cleanup asserts in interrupt context, hence doing it here. */
    set_default_mem_to_mem_cbr();
    cleanup_mem_to_mem();
    pwrmng_sleep_devices((void *)M2M_PWRMNG_DESC);
    UnLockEngineResources(mem2mem_lock);

    give_semaphore(edmac_copy_sem);
    return timed_out ? NULL : dst;
}

/* mlv_lite imports these; on D8 the engine lock is taken per copy above */
void edmac_memcpy_res_lock()
{
}

void edmac_memcpy_res_unlock()
{
}

/* mlv_lite imports these too, but the copy above doesn't use them */
uint32_t edmac_read_chan = M2M_READ_CHANNEL;
uint32_t edmac_write_chan = M2M_WRITE_CHANNEL;

/** WRONG: temporary overrides to get CONFIG_HELLO_WORLD working **/
/* No D8 equivalent identified yet: Canon passes the abort cbr to the abort
 * call itself, and the pop cbr looks like 0xe05815ac (events list, no
 * direct callers). */

void RegisterEDmacAbortCBR(int channel, void (*cbr)(void*), void* cbr_ctx)
{
    return;
}

void UnregisterEDmacAbortCBR(int channel)
{
    return;
}

void RegisterEDmacPopCBR(int channel, void (*cbr)(void*), void* cbr_ctx)
{
    return;
}

void UnregisterEDmacPopCBR(int channel)
{
    return;
}

void _EngDrvOut(uint32_t reg, uint32_t value)
{
    return;
}

uint32_t shamem_read(uint32_t addr)
{
    return 0;
}

void _engio_write(uint32_t* reg_list)
{
    return;
}

/* runs right after Canon's init_task: the full 3:2 tables before LiveView
 * (full32_core.c) */
extern void full32_early_start(void);
void platform_post_init()
{
    full32_early_start();
}

/* Audio levels for ML's meters (audio-common.c): Canon's peak meter,
 * AprocCtrl_GetLevelPeak(u16 out[8], mode) - mode 2 = stereo; out[0], [1]
 * L/R peak (16-bit sample scale), returns 0x11 while the audio device is
 * off.  Canon's factory audio test polls it the same way (0xE008B272).
 * Read on the left channel, the right one reuses that read. */
extern int AprocCtrl_GetLevelPeak(uint16_t *out, int mode);
int m6ii_audio_level(int channel)
{
    static uint16_t level[8];
    if (channel == 0)
    {
        if (AprocCtrl_GetLevelPeak(level, 2) != 0)
        {
            level[0] = level[1] = 0;
        }
    }
    return level[channel ? 1 : 0] & 0x7FFF;
}
