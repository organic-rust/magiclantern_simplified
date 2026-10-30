/**
 * MLV Sound addon module
 */

/*
 * Copyright (C) 2013 Magic Lantern Team
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

#include <module.h>
#include <dryos.h>
#include <menu.h>
#include <config.h>
#include <bmp.h>
#include <beep.h>
#include <propvalues.h>
#include <raw.h>
#include "ml-cbr.h"

#include "../../trace/trace.h"
#include "../mlv_rec/mlv.h"
#include "../mlv_rec/mlv_rec_interface.h"
#include "../rate_fit.h"

/* allocate that many frame slots to be used for WAVI blocks. two is the minimum to maintain operation */
#define MLV_SND_SLOTS              2
/* maximum number of WAVI blocks per slot. the larger, the longer queues will get. should we use more? */
#define MLV_SND_BLOCKS_PER_SLOT  256

static uint32_t trace_ctx = TRACE_ERROR;

static CONFIG_INT("mlv.snd.enabled", mlv_snd_enabled, 1);
static CONFIG_INT("mlv.snd.mlv_snd_enable_tracing", mlv_snd_enable_tracing, 0);
static CONFIG_INT("mlv.snd.bit.depth", mlv_snd_in_bits_per_sample, 16);
static CONFIG_INT("mlv.snd.sample.rate", mlv_snd_in_sample_rate, 48000);
static CONFIG_INT("mlv.snd.sample.rate.selection", mlv_snd_rate_sel, 0);
static CONFIG_INT("mlv.snd.vsync_delay", mlv_snd_vsync_delay, 1);

extern WEAK_FUNC(ret_0) int StartASIFDMAADC(void *, uint32_t, void *, uint32_t, void (*)(), uint32_t);
extern WEAK_FUNC(ret_0) int SetNextASIFADCBuffer(void *, uint32_t);
extern WEAK_FUNC(ret_0) int PowerAudioOutput();
extern WEAK_FUNC(ret_0) void audio_configure(int);
extern WEAK_FUNC(ret_0) int SetAudioVolumeOut(uint32_t);
extern WEAK_FUNC(ret_0) int SoundDevActiveIn(uint32_t);
extern WEAK_FUNC(ret_0) int SoundDevShutDownIn();
extern WEAK_FUNC(ret_0) int StopASIFDMAADC();
/* weak: DIGIC 8 uses Canon's sound input instead (mlv_snd_d8_*) */
extern WEAK_FUNC(ret_0) void SetSamplingRate(int sample_rate, int channels);
extern uint64_t get_us_clock();

/* DIGIC 8 (M6II): Canon's sound input with its own ring buffer; the
 * completion callback reports each new 0.5 s chunk (info words 53/54 =
 * address / size, 55/56 = the part wrapped to the ring start) */
struct sound_in_params
{
    uint32_t unk0;
    void    *stream;            /* 0: Canon's default ring (5 s), written back */
    uint32_t stream_size;
    uint32_t rate;
    uint32_t bits;
    uint32_t channels;
    void   (*complete_cbr)(void *ctx, uint32_t *info);
    void   (*stop_cbr)(void *ctx, void *info);
    void    *ctx;
};
extern WEAK_FUNC(ret_0) int SoundDevInOpen(struct sound_in_params *params);
extern WEAK_FUNC(ret_0) void SoundDevInStart();
extern WEAK_FUNC(ret_0) void SoundDevInStop();
extern WEAK_FUNC(ret_0) void SoundDevInClose();
#define D8_INFO_ADDR1 53
#define D8_INFO_SIZE1 54
#define D8_INFO_ADDR2 55
#define D8_INFO_SIZE2 56

extern void mlv_rec_get_slot_info(int32_t slot, uint32_t *size, void **address);
extern int32_t mlv_rec_get_free_slot();
extern void mlv_rec_release_slot(int32_t slot, uint32_t write);
extern void mlv_rec_set_rel_timestamp(mlv_hdr_t *hdr, uint64_t timestamp);
/* mlv_lite: the take's measured audio clock (mHz), for its frame rate
 * metadata; weak, as mlv_rec doesn't have it */
extern WEAK_FUNC(ret_0) void mlv_rec_set_audio_rate(int nominal_hz, int measured_mhz, int err_ppm);
extern void mlv_rec_skip_frames(uint32_t count);

static struct msg_queue * volatile mlv_snd_buffers_empty = NULL;
static struct msg_queue * volatile mlv_snd_buffers_done = NULL;
static volatile uint32_t mlv_snd_in_buffers = 64;
static volatile uint32_t mlv_snd_frame_number = 0;
static volatile uint32_t mlv_snd_in_buffer_size = 0;

/* for tracking chunks */
static volatile uint16_t mlv_snd_file_num = UINT16_MAX;
/* frames issued to mlv_lite for writing */
static volatile uint32_t mlv_snd_frames_queued = 0;
/* frames written to previous chunks */
static volatile uint32_t mlv_snd_frames_saved = 0;


static uint32_t mlv_snd_rates[] = { 48000, 44100, 22050, 11025, 8000 };
#define MLV_SND_RATE_TEXT "48kHz", "44.1kHz", "22kHz", "11kHz", "8kHz"

static uint32_t mlv_snd_in_channels = 2;

typedef struct
{
    uint16_t *data;
    uint32_t length;
    uint32_t frameNumber;
    uint64_t timestamp;
    
    /* these are filled when the ASIF buffer comes from a MLV frame slot */
    void *mlv_slot_buffer;
    int32_t mlv_slot_id;
    int32_t mlv_slot_end;
} audio_data_t;

audio_data_t *mlv_snd_current_buffer = NULL;
audio_data_t *mlv_snd_next_buffer = NULL;

#define MLV_SND_STATE_IDLE                   0  /* waiting for action, set by writer task upon exit */
#define MLV_SND_STATE_READY                  1  /* buffers etc are set up, set by mlv_snd_cbr_starting() */
#define MLV_SND_STATE_SOUND_RUNNING          2  /* ASIF sound recording was started, set by mlv_snd_vsync() */
#define MLV_SND_STATE_SOUND_STOPPING         3  /* stop audio recording, set by mlv_snd_stop() */
#define MLV_SND_STATE_SOUND_STOP_ASIF        4  /* waiting for ASIF to process its last buffer, set by mlv_snd_asif_in_cbr() */
#define MLV_SND_STATE_SOUND_STOP_TASK        5  /* waiting for thread to stop, set by mlv_snd_asif_in_cbr() */
#define MLV_SND_STATE_SOUND_STOPPED          6  /* all threads and stuff is stopped, finish cleanup, set by task */

static uint32_t mlv_snd_state = MLV_SND_STATE_IDLE;

/* DIGIC 8 sound input state */
static int mlv_snd_d8 = 0;
static struct sound_in_params mlv_snd_d8_params;
static struct semaphore *mlv_snd_d8_stop_sem = NULL;
static uint64_t mlv_snd_d8_start = 0;
static volatile uint32_t mlv_snd_d8_dropped = 0;
static int mlv_snd_d8_open = 0;     /* Canon's input is open */
static int mlv_snd_d8_failed = 0;   /* didn't open: don't retry every frame */

/* this tells the audio backend that we are going to record sound */
static ml_cbr_action mlv_snd_snd_rec_cbr (const char *event, void *data)
{
    uint32_t *status = (uint32_t*)data;
    
    if(mlv_snd_enabled)
    {
        *status = 1;
        return ML_CBR_STOP;
    }
    
    return ML_CBR_CONTINUE;
}

static void mlv_snd_asif_in_cbr()
{
    /* the next buffer is now being filled, so update timestamp. do this first to be closer to real start. */
    if(mlv_snd_next_buffer)
    {
        mlv_snd_next_buffer->timestamp = get_us_clock();
    }
    
    /* and pass the filled buffer into done queue */
    if(mlv_snd_current_buffer)
    {
        mlv_snd_current_buffer->frameNumber = mlv_snd_frame_number;
        mlv_snd_frame_number++;
        msg_queue_post(mlv_snd_buffers_done, (uint32_t) mlv_snd_current_buffer);
    }

    /* the "next" buffer is the current one being filled */
    mlv_snd_current_buffer = mlv_snd_next_buffer;
    mlv_snd_next_buffer = NULL;
    
    switch(mlv_snd_state)
    {
        case MLV_SND_STATE_SOUND_RUNNING:
        {
            uint32_t count = 0;
            if(msg_queue_count(mlv_snd_buffers_empty, &count))
            {
                trace_write(trace_ctx, "mlv_snd_asif_in_cbr: msg_queue_count failed");
                mlv_snd_state = MLV_SND_STATE_SOUND_STOP_ASIF;
                return;
            }
            if(count < 1)
            {
                trace_write(trace_ctx, "mlv_snd_asif_in_cbr: no free buffers available");
                mlv_snd_state = MLV_SND_STATE_SOUND_STOP_ASIF;
                return;
            }
            
            /* get the new "next" and queue */
            if(msg_queue_receive(mlv_snd_buffers_empty, &mlv_snd_next_buffer, 10))
            {
                trace_write(trace_ctx, "mlv_snd_asif_in_cbr: msg_queue_receive(mlv_snd_buffers_empty, ) failed");
                mlv_snd_state = MLV_SND_STATE_SOUND_STOP_ASIF;
                return;
            }
            trace_write(trace_ctx, "mlv_snd_asif_in_cbr: queueing buffer in slot %d", mlv_snd_next_buffer->mlv_slot_id);
            SetNextASIFADCBuffer(mlv_snd_next_buffer->data, mlv_snd_next_buffer->length);
            break;
        }
        
        case MLV_SND_STATE_SOUND_STOPPING:
            trace_write(trace_ctx, "mlv_snd_asif_in_cbr: stopping 1");
            mlv_snd_state = MLV_SND_STATE_SOUND_STOP_ASIF;
            break;
            
        case MLV_SND_STATE_SOUND_STOP_ASIF:
            trace_write(trace_ctx, "mlv_snd_asif_in_cbr: stopping 2");
            mlv_snd_state = MLV_SND_STATE_SOUND_STOP_TASK;
            break;
        
        default:
            break;
    }
}

/* DIGIC 8: runs in Canon's sound task, every 0.5 s: copy the new chunk
 * into the next empty AUDF block and pass it to the writer */
/* the audio clock measured against get_us_clock (as the video frames are
 * timestamped): each complete chunk's arrival time against the samples
 * delivered so far, dropped chunks included (Canon delivered them).  The
 * AUDF timestamps themselves come from the sample count, so the files
 * can't show it; mlv_lite asks for it at the end of a take. */
static struct rate_fit mlv_snd_afit;
static uint64_t mlv_snd_afit_samples = 0;

/* measured sample rate of the last / current take in mHz, 0 if unknown */
int mlv_snd_measured_rate_mhz(void)
{
    double r = rate_fit_rate(&mlv_snd_afit);
    return r > 0 ? (int) (r * (double) 1000 + (double) 0.5) : 0;
}

static void mlv_snd_d8_cbr(void *ctx, uint32_t *info)
{
    /* STOPPING too: Canon delivers the last, partial chunk after the
     * stop request (lost before: clips ended ~0.4 s early) */
    if (mlv_snd_state != MLV_SND_STATE_SOUND_RUNNING && mlv_snd_state != MLV_SND_STATE_SOUND_STOPPING)
    {
        return;
    }
    if (!info[D8_INFO_SIZE1] && !info[D8_INFO_SIZE2])
    {
        return;
    }

    /* arrival time first (before the copy); only whole chunks */
    {
        uint64_t now = get_us_clock();
        uint32_t frame_bytes = (mlv_snd_in_bits_per_sample / 8) * mlv_snd_in_channels;
        uint32_t chunk_bytes = info[D8_INFO_SIZE1] + info[D8_INFO_SIZE2];
        if (frame_bytes)
        {
            mlv_snd_afit_samples += chunk_bytes / frame_bytes;
            if (mlv_snd_state == MLV_SND_STATE_SOUND_RUNNING)
            {
                rate_fit_add(&mlv_snd_afit, mlv_snd_afit_samples, now);
            }
        }
    }

    uint32_t count = 0;
    audio_data_t *entry = NULL;
    if (msg_queue_count(mlv_snd_buffers_empty, &count) || count < 1 ||
        msg_queue_receive(mlv_snd_buffers_empty, &entry, 0) || !entry)
    {
        /* don't wait in Canon's task: this chunk is lost */
        mlv_snd_d8_dropped++;
        trace_write(trace_ctx, "mlv_snd_d8_cbr: no free buffer, %d chunks dropped", mlv_snd_d8_dropped);
        return;
    }

    uint32_t len1 = MIN(info[D8_INFO_SIZE1], entry->length);
    uint32_t len2 = MIN(info[D8_INFO_SIZE2], entry->length - len1);
    memcpy(entry->data, (void *) info[D8_INFO_ADDR1], len1);
    if (len2)
    {
        memcpy((uint8_t *) entry->data + len1, (void *) info[D8_INFO_ADDR2], len2);
    }
    if (len1 + len2 < entry->length)
    {
        /* short last chunk: pad with silence */
        memset((uint8_t *) entry->data + len1 + len2, 0, entry->length - len1 - len2);
    }

    /* time from the sample count: each block is entry->length bytes */
    entry->frameNumber = mlv_snd_frame_number;
    entry->timestamp = mlv_snd_d8_start + (uint64_t) mlv_snd_frame_number * entry->length * 1000000ULL
        / (mlv_snd_in_sample_rate * (mlv_snd_in_bits_per_sample / 8) * mlv_snd_in_channels);
    mlv_snd_frame_number++;
    msg_queue_post(mlv_snd_buffers_done, (uint32_t) entry);
}

static void mlv_snd_d8_stop_cbr(void *ctx, void *info)
{
    give_semaphore(mlv_snd_d8_stop_sem);
}

static int mlv_snd_d8_start_input()
{
    memset(&mlv_snd_d8_params, 0, sizeof(mlv_snd_d8_params));
    mlv_snd_d8_params.rate = mlv_snd_in_sample_rate;
    mlv_snd_d8_params.bits = mlv_snd_in_bits_per_sample;
    mlv_snd_d8_params.channels = mlv_snd_in_channels;
    mlv_snd_d8_params.complete_cbr = mlv_snd_d8_cbr;
    mlv_snd_d8_params.stop_cbr = mlv_snd_d8_stop_cbr;
    mlv_snd_d8_params.ctx = &mlv_snd_d8_params;

    if (SoundDevInOpen(&mlv_snd_d8_params) || !mlv_snd_d8_params.stream)
    {
        /* also what a camera without the stubs returns */
        trace_write(trace_ctx, "mlv_snd_d8_start_input: open failed");
        return 0;
    }
    mlv_snd_d8_open = 1;
    mlv_snd_d8_dropped = 0;
    rate_fit_reset(&mlv_snd_afit);
    mlv_snd_afit_samples = 0;
    mlv_snd_d8_start = get_us_clock();
    SoundDevInStart();
    return 1;
}

static void mlv_snd_d8_stop_input()
{
    if (!mlv_snd_d8_open)
    {
        return;
    }
    mlv_snd_d8_open = 0;
    SoundDevInStop();
    if (take_semaphore(mlv_snd_d8_stop_sem, 5000))
    {
        trace_write(trace_ctx, "mlv_snd_d8_stop_input: no stop callback in 5 s");
    }
    SoundDevInClose();
    if (mlv_snd_d8_dropped)
    {
        printf("mlv_snd: %d audio chunks (0.5 s) dropped\n", mlv_snd_d8_dropped);
    }
}

static void mlv_snd_flush_entries(struct msg_queue *queue, uint32_t clear)
{
    uint32_t msgs = 0;
    
    msg_queue_count(queue, &msgs);
    
    trace_write(trace_ctx, "mlv_snd_flush_entries: %d entries to free in queue", msgs);
    while(msgs > 0)
    {
        audio_data_t *entry = NULL;
        if(msg_queue_receive(queue, &entry, 10))
        {
            trace_write(trace_ctx, "mlv_snd_flush_entries: msg_queue_receive() failed");
            return;
        }
    
        mlv_audf_hdr_t *hdr = (mlv_audf_hdr_t *)entry->mlv_slot_buffer;
        
        if(clear)
        {
            trace_write(trace_ctx, "mlv_snd_flush_entries:   NULL slot %d entry", entry->mlv_slot_id);
            mlv_set_type((mlv_hdr_t *)hdr, "NULL");
            hdr->timestamp = 0;
        }
        else
        {
            trace_write(trace_ctx, "mlv_snd_flush_entries:   data %d entry for frame #%d", entry->mlv_slot_id, entry->frameNumber);
            mlv_set_type((mlv_hdr_t *)hdr, "AUDF");
            hdr->frameNumber = entry->frameNumber;
            mlv_rec_set_rel_timestamp((mlv_hdr_t*)hdr, entry->timestamp);
            
            /* set the highest frame number for updating header later */
            mlv_snd_frames_queued = MAX(mlv_snd_frames_queued, entry->frameNumber + 1);
        }
        
        if(entry->mlv_slot_end)
        {
            trace_write(trace_ctx, "mlv_snd_flush_entries:   last entry of slot %d -> release for writing", entry->mlv_slot_id);
            mlv_rec_release_slot(entry->mlv_slot_id, 1);
        }
        free(entry);
        
        msg_queue_count(queue, &msgs);
    }
    trace_write(trace_ctx, "mlv_snd_flush_entries: done");
}

static void mlv_snd_stop()
{
    trace_write(trace_ctx, "mlv_snd_stop: stopping worker and audio");
    
    mlv_snd_state = MLV_SND_STATE_SOUND_STOPPING;

    if (mlv_snd_d8)
    {
        /* Canon doesn't hand over the partial chunk when its input stops
         * (M6II clips ended 170-210 ms before the last frame), so keep
         * it running until a whole chunk covers the stop moment: the
         * audio then runs past the video by up to one chunk (0.5 s) */
        uint64_t stop_us = get_us_clock();
        uint64_t bytes_per_s = mlv_snd_in_sample_rate * (mlv_snd_in_bits_per_sample / 8) * mlv_snd_in_channels;
        for (int i = 0; i < 40; i++)
        {
            uint64_t covered = mlv_snd_d8_start + (uint64_t) mlv_snd_frame_number * mlv_snd_in_buffer_size * 1000000ULL / bytes_per_s;
            if (covered >= stop_us || !mlv_snd_d8_open)
            {
                break;
            }
            msleep(20);
        }

        /* no ASIF buffer chain to run out: stop Canon's input, then the writer */
        mlv_snd_d8_stop_input();
        mlv_snd_state = MLV_SND_STATE_SOUND_STOP_TASK;
    }
    
    /* wait until audio and task stopped */
    uint32_t loops = 100;
    while((mlv_snd_state != MLV_SND_STATE_SOUND_STOPPED) && (--loops > 0))
    {
        msleep(20);
    }

    if(mlv_snd_state != MLV_SND_STATE_SOUND_STOPPED)
    {
        bmp_printf(FONT(FONT_MED, COLOR_RED, COLOR_BLACK), 10, 130, "audio failed to stop, state %d", mlv_snd_state);
        trace_write(trace_ctx, "mlv_snd_stop: failed to stop audio (state %d)", mlv_snd_state);
        beep();
    }
    
    if (!mlv_snd_d8)
    {
        /* some models may need this */
        StopASIFDMAADC();
        // SoundDevShutDownIn();  /* no model seems to need this */
        audio_configure(1);
    }
    
    /* now flush the buffers */
    trace_write(trace_ctx, "mlv_snd_stop: flush mlv_snd_buffers_done");
    mlv_snd_flush_entries(mlv_snd_buffers_done, 0);
    trace_write(trace_ctx, "mlv_snd_stop: flush mlv_snd_buffers_empty");
    mlv_snd_flush_entries(mlv_snd_buffers_empty, 1);
}

static void mlv_snd_queue_slot()
{
    void *address = NULL;
    uint32_t queued = 0;
    uint32_t size = 0;
    uint32_t used = 0;
    uint32_t hdr_size = 0x100;
    uint32_t block_size = hdr_size + mlv_snd_in_buffer_size;
    
    int32_t slot = mlv_rec_get_free_slot();
    trace_write(trace_ctx, "mlv_snd_queue_slot: free slot %d", slot);
    
    /* get buffer memory address and available size */
    mlv_rec_get_slot_info(slot, &size, &address);
    
    if(!address)
    {
        trace_write(trace_ctx, "mlv_snd_queue_slot: failed to get address");
        return;
    }
    
    /* make sure that there is still place for a NULL block */
    while((used + block_size + sizeof(mlv_hdr_t) < size) && (queued < MLV_SND_BLOCKS_PER_SLOT))
    {
        /* setup AUDF header for that block */
        mlv_audf_hdr_t *hdr = (mlv_audf_hdr_t *)((uint32_t)address + used);
        
        trace_write(trace_ctx, "mlv_snd_queue_slot: used:%d / %d, block_size:%d, address: 0x%08X", used, size, block_size, hdr);
        used += block_size;
        
        mlv_set_type((mlv_hdr_t *)hdr, "NULL");
        hdr->blockSize = block_size;
        hdr->frameNumber = 0;
        hdr->frameSpace = hdr_size - sizeof(mlv_audf_hdr_t);
        hdr->timestamp = 0;
        
        /* store information about the buffer in the according queue entry */
        audio_data_t *entry = malloc(sizeof(audio_data_t));
        
        /* data is right after the header */
        entry->data = (void*)((uint32_t)hdr + hdr_size);
        entry->length = mlv_snd_in_buffer_size;
        entry->timestamp = 0;
        
        /* refer to the slot we are adding */
        entry->mlv_slot_buffer = hdr;
        entry->mlv_slot_id = slot;
        entry->mlv_slot_end = 0;
        
        /* check if this was the last frame and set end flag if so */
        if((used + block_size + sizeof(mlv_hdr_t) >= size) || (queued >= (MLV_SND_BLOCKS_PER_SLOT - 1)))
        {
            /* this tells the writer task that the buffer is filled with that entry being done and can be committed */
            entry->mlv_slot_end = 1;
        }
        
        msg_queue_post(mlv_snd_buffers_empty, (uint32_t) entry);
        queued++;
    }
    
    /* now add a trailing NULL block */
    mlv_hdr_t *hdr = (mlv_hdr_t *)((uint32_t)address + used);
    
    mlv_set_type((mlv_hdr_t *)hdr, "NULL");
    hdr->blockSize = size - used;
    hdr->timestamp = 0;
}


static void mlv_snd_prepare_audio()
{
    mlv_snd_in_sample_rate = mlv_snd_rates[mlv_snd_rate_sel];

    if (mlv_snd_d8)
    {
        /* Canon's input sets itself up; tested at 48 kHz 16-bit stereo
         * only. No ASIF registers here (0xC092011C is a DIGIC 4/5 one) */
        mlv_snd_in_sample_rate = 48000;
        mlv_snd_in_bits_per_sample = 16;
        mlv_snd_in_channels = 2;
        return;
    }

    /* some models may need this */
    SoundDevActiveIn(0);
    
    /* set up audio output according to configuration */
    SetSamplingRate(mlv_snd_in_sample_rate, 0);
    
    /* set 16 bit per sample, stereo. not nice, should be done through SetAudioChannels() (0xFF10EFF4 on 5D3) */
    MEM(0xC092011C) = 6;
}

static void mlv_snd_alloc_buffers()
{
    /* calculate buffer size; DIGIC 8: Canon's chunks are 0.5 s */
    int fps = mlv_snd_d8 ? 2 : 5;

    mlv_snd_in_buffer_size = (mlv_snd_in_sample_rate * (mlv_snd_in_bits_per_sample / 8) * mlv_snd_in_channels) / fps;
    trace_write(trace_ctx, "mlv_snd_alloc_buffers: mlv_snd_in_buffer_size = %d", mlv_snd_in_buffer_size);
    
    for(int slot = 0; slot < MLV_SND_SLOTS; slot++)
    {
        mlv_snd_queue_slot();
    }
}

static void mlv_snd_writer(int unused)
{
    uint32_t done = 0;
 
    TASK_LOOP
    {
        audio_data_t *buffer = NULL;
        
        if(done)
        {
            break;
        }
        
        switch(mlv_snd_state)
        {
            case MLV_SND_STATE_SOUND_STOP_TASK:
                trace_write(trace_ctx, "   --> WRITER: exiting");
                done = 1;
                break;
                
            case MLV_SND_STATE_SOUND_RUNNING:
            
                /* receive write job from dispatcher */
                if(msg_queue_receive(mlv_snd_buffers_done, &buffer, 500))
                {
                    static uint32_t timeouts = 0;
                    trace_write(trace_ctx, "   --> WRITER: message timed out %d times now", ++timeouts);
                    break;
                }
                
                /* this must never happen */
                if(!buffer)
                {
                    static uint32_t timeouts = 0;
                    trace_write(trace_ctx, "   --> WRITER: message NULL %d times now", ++timeouts);
                    break;
                }
                
                /* the slot was for MLV video, handle it */
                trace_write(trace_ctx, "   --> WRITER: entry is MLV slot %d, setting frame #%d", buffer->mlv_slot_id, buffer->frameNumber);
                
                mlv_audf_hdr_t *hdr = (mlv_audf_hdr_t *)buffer->mlv_slot_buffer;
                mlv_set_type((mlv_hdr_t *)hdr, "AUDF");
                
                /* fill recording information */
                hdr->frameNumber = buffer->frameNumber;
                mlv_rec_set_rel_timestamp((mlv_hdr_t*)hdr, buffer->timestamp);
                
                /* counted for the header now: a slot that isn't full at the
                 * stop is written by the flush, which only counts entries
                 * still waiting (DIGIC 8's 0.5 s chunks: a short take never
                 * fills one, and the header said 0 audio frames) */
                mlv_snd_frames_queued = MAX(mlv_snd_frames_queued, hdr->frameNumber + 1);

                /* only queue for writing if the whole mlv_rec slot was filled */
                if(buffer->mlv_slot_end)
                {
                    trace_write(trace_ctx, "   --> WRITER: entry is MLV slot %d (last buffer, so release)", buffer->mlv_slot_id);
                    mlv_rec_release_slot(buffer->mlv_slot_id, 1);
                    mlv_snd_queue_slot();
                }
                free(buffer);
                break;
            
            default:
                msleep(100);
                break;
        }
    }
    
    mlv_snd_state = MLV_SND_STATE_SOUND_STOPPED;
}

static void mlv_snd_start()
{
    if(mlv_snd_enable_tracing && (trace_ctx == TRACE_ERROR))
    {
        char filename[] = "mlv_snd.txt";
        trace_ctx = trace_start("mlv_snd", filename);
        trace_format(trace_ctx, TRACE_FMT_TIME_REL | TRACE_FMT_COMMENT | TRACE_FMT_TASK_ID | TRACE_FMT_TASK_NAME, ' ');
    }

    trace_write(trace_ctx, "mlv_snd_start: starting");
    
    mlv_snd_prepare_audio();
    task_create("mlv_snd", 0x16, 0x1000, mlv_snd_writer, NULL);
}

void mlv_fill_wavi(mlv_wavi_hdr_t *hdr, uint64_t start_timestamp)
{
    mlv_set_type((mlv_hdr_t*)hdr, "WAVI");
    hdr->blockSize = sizeof(mlv_wavi_hdr_t);
    mlv_set_timestamp((mlv_hdr_t *)hdr, start_timestamp);

    if(!mlv_snd_enabled)
    {
        /* not recording sound, don't trick MLV decoders :) */
        mlv_set_type((mlv_hdr_t*)hdr, "NULL");
        return;
    }
    
    /* this part is compatible to RIFF WAVE/fmt header */
    hdr->format = 1;
    hdr->channels = mlv_snd_in_channels;
    hdr->samplingRate = mlv_snd_in_sample_rate;
    hdr->bytesPerSecond = mlv_snd_in_sample_rate * (mlv_snd_in_bits_per_sample / 8) * mlv_snd_in_channels;
    hdr->blockAlign = (mlv_snd_in_bits_per_sample / 8) * mlv_snd_in_channels;
    hdr->bitsPerSample = mlv_snd_in_bits_per_sample;
}

static void mlv_snd_queue_wavi()
{
    trace_write(trace_ctx, "mlv_snd_queue_wavi: queueing a WAVI block");
    
    /* queue an WAVI block that contains information about the audio format */
    mlv_wavi_hdr_t *hdr = malloc(sizeof(mlv_wavi_hdr_t));
    
    mlv_fill_wavi(hdr, get_us_clock());
    
    mlv_rec_queue_block((mlv_hdr_t *)hdr);
}

static void mlv_snd_cbr_starting(uint32_t event, void *ctx, mlv_hdr_t *hdr)
{
    if(!mlv_snd_enabled)
    {
        return;
    }
    
    if(mlv_snd_state != MLV_SND_STATE_IDLE)
    {
        return;
    }
    
    /* recording is about to start, everything was set up there, now it is our turn */
    trace_write(trace_ctx, "mlv_snd_cbr_starting: starting mlv_snd");
    mlv_snd_start();
    mlv_snd_queue_wavi();
    mlv_snd_alloc_buffers();
    
    /* reset all variables first */
    mlv_snd_d8_failed = 0;
    mlv_snd_file_num = UINT16_MAX;
    mlv_snd_frame_number = 0;
    mlv_snd_frames_queued = 0;
    mlv_snd_frames_saved = 0;
    

    mlv_snd_state = MLV_SND_STATE_READY;
}

/* will get called from mlv_lite's vsync hook */
static void mlv_snd_cbr_started(uint32_t event, void *ctx, mlv_hdr_t *hdr)
{
    /* the first time we get called from vsync, start recording */
    if(mlv_snd_state != MLV_SND_STATE_READY || mlv_snd_d8_failed)
    {
        return;
    }
    
    /* "delaying audio" in the video timeline means to skip video frames */
    mlv_rec_skip_frames(mlv_snd_vsync_delay);
    
    /* fetch buffers to start recording */
    uint32_t msgs = 0;
    msg_queue_count(mlv_snd_buffers_empty, &msgs);
    
    if(msgs < 2)
    {
        trace_write(trace_ctx, "mlv_snd_cbr_started: fatal error, no buffers");
        bmp_printf(FONT(FONT_MED, COLOR_RED, COLOR_BLACK), 10, 130, "fatal: no buffers");
        beep();
        return;
    }
    
    trace_write(trace_ctx, "mlv_snd_cbr_started: starting audio");

    if (mlv_snd_d8)
    {
        /* Canon's input has its own ring; its callback takes blocks from
         * the queue in order as chunks come in. Don't take any here:
         * putting them back (as before) moved the first slot's first two
         * blocks behind the second slot - by then slot 1 was written and
         * handed back, and 1 s of sound was lost */
        mlv_snd_state = MLV_SND_STATE_SOUND_RUNNING;
        if (!mlv_snd_d8_start_input())
        {
            /* stay READY (the stop path cleans up), but don't retry */
            mlv_snd_d8_failed = 1;
            mlv_snd_state = MLV_SND_STATE_READY;
            bmp_printf(FONT(FONT_MED, COLOR_RED, COLOR_BLACK), 10, 130, "audio: sound input didn't open");
            beep();
        }
        return;
    }
    
    /* get two buffers and queue them to ASIF */
    mlv_snd_current_buffer = NULL;
    mlv_snd_next_buffer = NULL;
    
    msg_queue_receive(mlv_snd_buffers_empty, &mlv_snd_current_buffer, 10);
    msg_queue_receive(mlv_snd_buffers_empty, &mlv_snd_next_buffer, 10);
    
    if(!mlv_snd_current_buffer || !mlv_snd_next_buffer)
    {
        trace_write(trace_ctx, "mlv_snd_cbr_started: fatal error, no buffers");
        bmp_printf(FONT(FONT_MED, COLOR_RED, COLOR_BLACK), 10, 130, "fatal: no buffers");
        beep();
        return;
    }

    audio_configure(1);
    StartASIFDMAADC(mlv_snd_current_buffer->data, mlv_snd_current_buffer->length, mlv_snd_next_buffer->data, mlv_snd_next_buffer->length, mlv_snd_asif_in_cbr, 0);
    
    /* the current one will get filled right now */
    mlv_snd_current_buffer->timestamp = get_us_clock();
    trace_write(trace_ctx, "mlv_snd_cbr_started: starting audio DONE");
    
    mlv_snd_state = MLV_SND_STATE_SOUND_RUNNING;
}

static void mlv_snd_cbr_stopping(uint32_t event, void *ctx, mlv_hdr_t *hdr)
{
    if(mlv_snd_state != MLV_SND_STATE_SOUND_RUNNING)
    {
        return;
    }
    
    trace_write(trace_ctx, "mlv_snd_cbr_stopping: stopping");
    mlv_snd_stop();

    /* all chunks are in now */
    mlv_rec_set_audio_rate(mlv_snd_in_sample_rate, mlv_snd_measured_rate_mhz(), rate_fit_err_ppm(&mlv_snd_afit));
    mlv_snd_state = MLV_SND_STATE_IDLE;
}

static void mlv_snd_cbr_stopped(uint32_t event, void *ctx, mlv_hdr_t *hdr)
{
    if(mlv_snd_state == MLV_SND_STATE_IDLE)
    {
        return;
    }
    
    trace_write(trace_ctx, "mlv_snd_cbr_stopped: seems recording aborted during setup");
    mlv_snd_stop();
    mlv_snd_state = MLV_SND_STATE_IDLE;
}


static void mlv_snd_cbr_mlv_block(uint32_t event, void *ctx, mlv_hdr_t *hdr)
{
    if(hdr && !memcmp(hdr->blockType, "MLVI", 4))
    {
        mlv_file_hdr_t *file_hdr = (mlv_file_hdr_t *)hdr;
        
        uint16_t file_num = file_hdr->fileNum;
        
        trace_write(trace_ctx, "mlv_snd_cbr_mlv_block: called for file %d", file_num);
        
        /* the MLV block is filled on recording start and when the block gets updates on recording end */
        if(mlv_snd_state == MLV_SND_STATE_READY)
        {
            trace_write(trace_ctx, "mlv_snd_cbr_mlv_block: first chunk");
            /* first chunk's header is being written, save the number of frames in previous chunks */
            file_hdr->audioClass = 1; /* 0=none, 1=WAV */
            file_hdr->audioFrameCount = 0;
        }
        else if(file_num == mlv_snd_file_num)
        {
            /* block gets updated, capture AUDF count */
            uint32_t queued = mlv_snd_frames_queued;
            
            file_hdr->audioFrameCount = queued - mlv_snd_frames_saved;
            
            trace_write(trace_ctx, "mlv_snd_cbr_mlv_block: update: q:%d, s:%d", queued, mlv_snd_frames_saved);
            
            mlv_snd_frames_saved = queued;
        }
        else if(file_num > mlv_snd_file_num)
        {
            trace_write(trace_ctx, "mlv_snd_cbr_mlv_block: next");
            /* next chunk's header is being written */
            file_hdr->audioClass = 1; /* 0=none, 1=WAV */
            file_hdr->audioFrameCount = 0;
        }
        else
        {
            /* suddenly file_num goes back? unexpected, do not tamper with */
        }
        
        mlv_snd_file_num = file_num;
    }
}

static void mlv_snd_trace_buf(char *caption, uint8_t *buffer, uint32_t length)
{
    char *str = malloc(length * 2 + 1);

    for(uint32_t pos = 0; pos < length; pos++)
    {
        snprintf(&str[pos * 2], 3, "%02X", buffer[pos]);
    }

    trace_write(trace_ctx, "%s: %s", caption, str);

    free(str);
}


static struct menu_entry mlv_snd_menu[] =
{
    {
        .name       = "Sound recording",
        .select     = menu_open_submenu,
        .priv       = &mlv_snd_enabled,
        .help       = "Sound recording options provided by mlv_snd.",
        .children   = (struct menu_entry[])
        {
            {
                .name       = "Enable sound",
                .priv       = &mlv_snd_enabled,
                .max        = 1,
                .help       = "[mlv_snd] Enable sound recording for MLV.",
            },
            {
                .name       = "Sampling rate",
                .priv       = &mlv_snd_rate_sel,
                .min        = 0,
                .max        = COUNT(mlv_snd_rates)-1,
                .choices    = CHOICES(MLV_SND_RATE_TEXT),
                .help       = "[mlv_snd] Select your sampling rate.",
            },
            {
                .name = "Audio delay",
                .priv = &mlv_snd_vsync_delay,
                .min = 0,
                .max = 32,
                .help = "Delay the audio that many frames. (experimental)",
            },
            {
                .name       = "Trace output",
                .priv       = &mlv_snd_enable_tracing,
                .min        = 0,
                .max        = 1,
                .help       = "[mlv_snd] Enable log file tracing. Needs camera restart.",
            },
            MENU_EOL,
        },
    },
};

static unsigned int mlv_snd_init()
{
    /* causes ERR70 ?! */
    //if(mlv_snd_enable_tracing)
    //{
    //    char filename[] = "mlv_snd.txt";
    //    trace_ctx = trace_start("mlv_snd", filename);
    //    trace_format(trace_ctx, TRACE_FMT_TIME_REL | TRACE_FMT_COMMENT, ' ');
    //}
    
    /* DIGIC 8: Canon's sound input instead of ASIF DMA (M6II stubs) */
    mlv_snd_d8 = (get_digic_version() >= 8);
    mlv_snd_d8_stop_sem = create_named_semaphore("mlv_snd_d8_stop", 0);

    trace_write(trace_ctx, "mlv_snd_init: init queues");
    mlv_snd_buffers_empty = (struct msg_queue *) msg_queue_create("mlv_snd_buffers_empty", MLV_SND_BLOCKS_PER_SLOT * MLV_SND_SLOTS);
    mlv_snd_buffers_done = (struct msg_queue *) msg_queue_create("mlv_snd_buffers_done", MLV_SND_BLOCKS_PER_SLOT * MLV_SND_SLOTS);

    /* will the same menu work in both submenus? probably not */
    if (menu_get_value_from_script("Movie", "RAW video") != INT_MIN)
    {
        menu_add("RAW video", mlv_snd_menu, COUNT(mlv_snd_menu));
    }
    else if (menu_get_value_from_script("Movie", "RAW video (MLV)") != INT_MIN)
    {
        menu_add("RAW video (MLV)", mlv_snd_menu, COUNT(mlv_snd_menu));
    }

    trace_write(trace_ctx, "mlv_snd_init: done");
    
    /* register callbacks */
    mlv_rec_register_cbr(MLV_REC_EVENT_STARTING, &mlv_snd_cbr_starting, NULL);
    mlv_rec_register_cbr(MLV_REC_EVENT_STARTED, &mlv_snd_cbr_started, NULL);
    mlv_rec_register_cbr(MLV_REC_EVENT_STOPPING, &mlv_snd_cbr_stopping, NULL);
    mlv_rec_register_cbr(MLV_REC_EVENT_STOPPED, &mlv_snd_cbr_stopped, NULL);
    mlv_rec_register_cbr(MLV_REC_EVENT_BLOCK, &mlv_snd_cbr_mlv_block, NULL);
    
    return 0;
}


static unsigned int mlv_snd_deinit()
{
    if(trace_ctx != TRACE_ERROR)
    {
        trace_stop(trace_ctx, 0);
        trace_ctx = TRACE_ERROR;
    }
    return 0;
}

MODULE_INFO_START()
    MODULE_INIT(mlv_snd_init)
    MODULE_DEINIT(mlv_snd_deinit)
MODULE_INFO_END()

MODULE_CBRS_START()
    MODULE_NAMED_CBR("snd_rec_enabled", mlv_snd_snd_rec_cbr)
MODULE_CBRS_END()

MODULE_CONFIGS_START()
    MODULE_CONFIG(mlv_snd_enabled)
    MODULE_CONFIG(mlv_snd_enable_tracing)
    MODULE_CONFIG(mlv_snd_in_bits_per_sample)
    MODULE_CONFIG(mlv_snd_rate_sel)
    MODULE_CONFIG(mlv_snd_in_sample_rate)
    MODULE_CONFIG(mlv_snd_vsync_delay)
MODULE_CONFIGS_END()
