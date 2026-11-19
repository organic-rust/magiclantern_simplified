#include "dryos.h"
#include "module.h"
#include "lens.h"
#include "raw.h"
#include "edmac.h"

struct TwoInTwoOutLosslessPath_args
{ 
  uint32_t * ResLockKey;
  uint32_t   ResLockKeySize;
  uint32_t * engio_cmd_1_prepare;
  uint32_t * engio_cmd_2_prepare;
  uint32_t * engio_cmd_pre_jpcore;
  uint32_t * engio_cmd_post_jpcore;
  uint32_t * engio_cmd_stop;
  uint32_t * engio_cmd_6_prepare;
  uint32_t * engio_cmd_7_prepare;
  uint32_t * engio_cmd_8_HIV_PPR;
  uint16_t * PPR_table;
  uint32_t * goes_to_obinteg_C0F0E000_size_0x188;
  uint32_t * goes_to_C0F21000_size_0x7E0;
  uint32_t * LuckyEnable_goes_to_C0F20000_size_0x1000;
  uint16_t   LuckyEnable;
  uint16_t   xRes;
  uint16_t   yRes;
  uint16_t   off_0x3E;
  uint32_t   SamplePrecision;
  uint32_t   off_0x44;
  uint32_t   RD1_Channel;
  uint32_t   RD1_StartFlags;
  uint32_t   RD1_Flags;
  uint32_t   RD1_Connection;
  void *     RD1_Address;
  uint32_t   RD1_Offset;
  uint32_t   RD2_Channel;
  uint32_t   RD2_StartFlags;
  uint32_t   RD2_Flags;
  uint32_t   RD2_Connection;
  void *     RD2_Address;
  uint32_t   RD2_Offset;
  uint32_t   WR1_Channel;
  uint32_t   WR1_StartFlags;
  uint32_t   WR1_Flags;
  uint32_t   WR1_Connection;
  struct memSuite * WR1_MemSuite;
  uint32_t   off_0x8C;
  uint32_t   WR2_Channel;
  uint32_t   WR2_StartFlags;
  uint32_t   WR2_Flags;
  uint32_t   WR2_Connection;
  void *     WR2_Address;
  uint32_t   WR2_Offset;
};

static struct TwoInTwoOutLosslessPath_args TTL_Args;
static struct LockEntry * TTL_ResLock = 0;
static struct semaphore * lossless_sem = 0;
static int verbose = 0;

static void LosslessCompleteCBR()
{
//    DryosDebugMsg(0, 0, "LosslessCompleteCBR\n");
    give_semaphore(lossless_sem);
}

/* pack two 16-bit values into a 32-bit one */
#define PACK32(lo,hi) (((uint32_t)(lo) & 0xFFFF) | ((uint32_t)(hi) << 16))

static void (*TTL_SetArgs)(int unused, void * TTL_Args, int PictureSize) = NULL;
static void (*TTL_Prepare)(void * ResLock, void * TTL_Args) = NULL;
static void (*TTL_RegisterCBR)(int id, void * cbr, void * cbr_arg) = NULL;
static void (*TTL_SetFlags)(int PictureType) = NULL;
static void (*TTL_Start)(void * TTL_Args) = NULL;
static void (*TTL_Stop)(void * TTL_Args) = NULL;
static void (*TTL_Finish)(void * ResLock, void * TTL_Args, uint32_t * output_size) = NULL;

/* edmac-memcpy.c */
extern uint32_t edmac_write_chan;
extern uint32_t edmac_read_chan;

/* 5D2 (DIGIC 4): ProcessTwoInTwoOutJpegPath
 *
 * Canon's CR2 path (sdfExecuteMem1ToRawPath, FF888C8C) fills a TTJ_Args struct with
 * FF9BEB18 (same role as TTL_SetArgs), then makes one blocking call:
 * FFA59B58(lock, args, &output_size). That call locks the engine resources,
 * sets up JPCORE + EDMAC, starts, waits for completion (event flag, 5 s timeout),
 * stops and unlocks. There are no separate Prepare/Start/Stop/Finish steps to call.
 *
 * The layout is DIGIC 5's TwoInTwoOutLosslessPath_args minus one engio pointer;
 * offsets below were checked against FF9BEB18, FFA594C4, FFA591A8 and FFA590A4,
 * and against Canon's live struct (RAM 0x2AC34) after a RAW capture.
 *
 * First milestone: full-resolution frames only, with Canon's RAW engio lists
 * unchanged (three vertical slices of 1936/1936/1920 px, as in a CR2).
 */
struct TwoInTwoOutJpegPath_args_5D2
{
    uint32_t * ResLockKey;              /* 0x00: Canon's resource list (not used by FFA59B58) */
    uint32_t   ResLockKeySize;          /* 0x04 */
    uint32_t * engio_prep1;             /* 0x08: input path, RD1 EDMAC geometry, slicing */
    uint32_t * engio_prep2;             /* 0x0C: JPCORE settings (Canon: "VARIABLE_DATA"+0x338) */
    uint32_t * engio_pre_jpcore;        /* 0x10 */
    uint32_t * engio_post_jpcore;       /* 0x14 */
    uint32_t * engio_stop;              /* 0x18 */
    uint32_t * engio_after_ppr;         /* 0x1C */
    uint32_t * engio_ppr;               /* 0x20: run after writing PPR_table to C0F0857C */
    uint16_t * PPR_table;               /* 0x24 */
    uint32_t * goes_to_obinteg_C0F0E000_size_0x188;     /* 0x28: Canon: "RAW_OBWB" */
    uint32_t * goes_to_C0F21000_size_0x7E0;             /* 0x2C: Canon: "LUCKY_ANGEL" (sRAW only) */
    uint32_t * LuckyEnable_goes_to_C0F20000_size_0x1000;/* 0x30 */
    uint16_t   LuckyEnable;             /* 0x34: 0xFFFF = skip */
    uint16_t   xRes;                    /* 0x36 */
    uint16_t   yRes;                    /* 0x38 */
    uint16_t   off_0x3A;
    uint32_t   SamplePrecision;         /* 0x3C: 14 = RAW (lossless), 15 = sRAW, 16 = mRAW */
    uint32_t   off_0x40;                /* 0x40: low byte = table index */
    uint32_t   RD1_Channel;             /* 0x44 */
    uint32_t   RD1_StartFlags;
    uint32_t   RD1_Flags;
    uint32_t   RD1_Connection;
    void *     RD1_Address;             /* 0x54: EDMAC start = RD1_Address + RD1_Offset */
    int32_t    RD1_Offset;              /* 0x58 */
    uint32_t   RD2_Channel;             /* 0x5C */
    uint32_t   RD2_StartFlags;
    uint32_t   RD2_Flags;
    uint32_t   RD2_Connection;
    void *     RD2_Address;             /* 0x6C: 0 = not used */
    int32_t    RD2_Offset;
    uint32_t   WR1_Channel;             /* 0x74 */
    uint32_t   WR1_StartFlags;
    uint32_t   WR1_Flags;
    uint32_t   WR1_Connection;
    struct memSuite * WR1_MemSuite;     /* 0x84 */
    uint32_t   off_0x88;
    uint32_t   WR2_Channel;             /* 0x8C */
    uint32_t   WR2_StartFlags;
    uint32_t   WR2_Flags;
    uint32_t   WR2_Connection;
    void *     WR2_Address;             /* 0x9C: 0 = not used */
    uint32_t   WR2_Offset;              /* 0xA0 */
};

_Static_assert(sizeof(struct TwoInTwoOutJpegPath_args_5D2) == 0xA4, "TTJ_Args 5D2 size");
_Static_assert(__builtin_offsetof(struct TwoInTwoOutJpegPath_args_5D2, xRes) == 0x36, "xRes");
_Static_assert(__builtin_offsetof(struct TwoInTwoOutJpegPath_args_5D2, SamplePrecision) == 0x3C, "type");
_Static_assert(__builtin_offsetof(struct TwoInTwoOutJpegPath_args_5D2, RD1_Address) == 0x54, "RD1");
_Static_assert(__builtin_offsetof(struct TwoInTwoOutJpegPath_args_5D2, WR1_MemSuite) == 0x84, "WR1");
_Static_assert(__builtin_offsetof(struct TwoInTwoOutJpegPath_args_5D2, WR2_Address) == 0x9C, "WR2");

static struct TwoInTwoOutJpegPath_args_5D2 TTJ_Args_5D2;
static void (*TTJ_SetArgs_5D2)(int unused, void * TTJ_Args, int PictureSize) = NULL;
static int  (*TTJ_Process_5D2)(struct LockEntry * lock, void * TTJ_Args, uint32_t * output_size) = NULL;
static int lossless_compress_raw_5D2(
    struct memSuite * dst_suite, void * src,
    int src_width, int src_x, int src_y,
    int width, int height
);

static void decompress_init();
int lossless_init()
{
    if (is_camera("5D3", "1.1.3"))
    {
        /* ProcessTwoInTwoOutLosslessPath, 5D3 1.1.3 */
        TTL_SetArgs     = (void *) 0xFF32330C;  /* fills TTL_Args struct; PictureSize(Mem1ToRaw) */
        TTL_Prepare     = (void *) 0xFF3D4680;  /* called right after ProcessTwoInTwoOutLosslessPath(R) Start; */
                                                /* calls [TTL] GetPathResources and sets up the encoder for RAW/SRAW/MRAW */
        TTL_RegisterCBR = (void *) 0xFF3D3774;  /* RegisterTwoInTwoOutLosslessPathCompleteCBR */
        TTL_SetFlags    = (void *) 0xFF32B418;  /* called next, with PictureType as arguments */
        TTL_Start       = (void *) 0xFF3D46F0;  /* called next; starts the EDmac transfers */
        TTL_Stop        = (void *) 0xFF3D4728;  /* called right after sssStopMem1ToRawPath */
        TTL_Finish      = (void *) 0xFF3D4760;  /* called next; calls UnlockEngineResources and returns output size from JpCoreCompleteCBR */
    }

    if (is_camera("5D3", "1.2.3"))
    {
        /* ProcessTwoInTwoOutLosslessPath, 5D3 1.2.3 */
        TTL_SetArgs     = (void *) 0xFF327DE8;  /* fills TTL_Args struct; PictureSize(Mem1ToRaw) */
        TTL_Prepare     = (void *) 0xFF3DD574;  /* called right after ProcessTwoInTwoOutLosslessPath(R) Start; */
                                                /* calls [TTL] GetPathResources and sets up the encoder for RAW/SRAW/MRAW */
        TTL_RegisterCBR = (void *) 0xFF3DC668;  /* RegisterTwoInTwoOutLosslessPathCompleteCBR */
        TTL_SetFlags    = (void *) 0xFF32FF2C;  /* called next, with PictureType as arguments */
        TTL_Start       = (void *) 0xFF3DD5E4;  /* called next; starts the EDmac transfers */
        TTL_Stop        = (void *) 0xFF3DD61C;  /* called right after sssStopMem1ToRawPath */
        TTL_Finish      = (void *) 0xFF3DD654;  /* called next; calls UnlockEngineResources and returns output size from JpCoreCompleteCBR */
    }

    if (is_camera("700D", "1.1.5"))
    {
        /* ProcessTwoInTwoOutJpegath, 700D 1.1.5 */
        TTL_SetArgs     = (void *) 0xFF35F510;  /* fills TTJ_Args struct; PictureSize(Mem1ToRaw) */
        TTL_Prepare     = (void *) 0xFF424BA4;  /* called right after ProcessTwoInTwoOutJpegath(R) Start(%d); */
                                                /* calls [TTJ] GetPathResources and sets up the encoder for RAW */
        TTL_RegisterCBR = (void *) 0xFF423B88;  /* RegisterTwoInTwoOutJpegPathCompleteCBR */
        TTL_SetFlags    = (void *) 0xFF36B2D8;  /* alternate StartTwoInTwoOutJpegPath http://www.magiclantern.fm/forum/index.php?topic=18443.msg188721#msg188721 */
        TTL_Start       = (void *) 0xFF424C4C;  /* called next; starts the EDmac transfers */
        TTL_Stop        = (void *) 0xFF423DD4;  /* called right after sssStopMem1ToRawPath */
        TTL_Finish      = (void *) 0xFF424CBC;  /* called next; calls UnlockEngineResources and returns output size from JpCoreCompleteCBR */
    }

    if (is_camera("650D", "1.0.4"))
    {
        /* ProcessTwoInTwoOutJpegath, 650D 1.0.4 */
        TTL_SetArgs     = (void *) 0xFF35C9C0;  /* fills TTJ_Args struct; PictureSize(Mem1ToRaw) */
        TTL_Prepare     = (void *) 0xFF4210BC;  /* called right after ProcessTwoInTwoOutJpegath(R) Start(%d); */
                                                /* calls [TTJ] GetPathResources and sets up the encoder for RAW */
        TTL_RegisterCBR = (void *) 0xFF4200A0;  /* RegisterTwoInTwoOutJpegPathCompleteCBR */
        TTL_SetFlags    = (void *) 0xFF368788;  /* alternate StartTwoInTwoOutJpegPath http://www.magiclantern.fm/forum/index.php?topic=18443.msg188721#msg188721 */
        TTL_Start       = (void *) 0xFF421164;  /* called next; starts the EDmac transfers */
        TTL_Stop        = (void *) 0xFF4202EC;  /* called right after sssStopMem1ToRawPath */
        TTL_Finish      = (void *) 0xFF4211D4;  /* called next; calls UnlockEngineResources and returns output size from JpCoreCompleteCBR */
    }

    if (is_camera("EOSM", "2.0.2"))
    {
        /* ProcessTwoInTwoOutJpegath, EOSM 2.0.2 */
        TTL_SetArgs     = (void *) 0xFF361498;  /* fills TTJ_Args struct; PictureSize(Mem1ToRaw) */
        TTL_Prepare     = (void *) 0xFF429210;  /* called right after ProcessTwoInTwoOutJpegath(R) Start(%d); */
                                                /* calls [TTJ] GetPathResources and sets up the encoder for RAW */
        TTL_RegisterCBR = (void *) 0xFF4281F4;  /* RegisterTwoInTwoOutJpegPathCompleteCBR */
        TTL_SetFlags    = (void *) 0xFF36D124;  /* called next, with PictureType as arguments */
        TTL_Start       = (void *) 0xFF4292B8;  /* called next; starts the EDmac transfers */
        TTL_Stop        = (void *) 0xFF428440;  /* called right after sssStopMem1ToRawPath */
        TTL_Finish      = (void *) 0xFF429328;  /* called next; calls UnlockEngineResources and returns output size from JpCoreCompleteCBR */
    }
    if (is_camera("100D", "1.0.1"))
    {
        /* ProcessTwoInTwoOutJpegath, 100D 1.0.1 */
        TTL_SetArgs     = (void *) 0xFF3647D0;  /* fills TTJ_Args struct; PictureSize(Mem1ToRaw) */
        TTL_Prepare     = (void *) 0xFF42Bf8C;  /* called right after ProcessTwoInTwoOutJpegath(R) Start(%d); */
                                                /* calls [TTJ] GetPathResources and sets up the encoder for RAW */
        TTL_RegisterCBR = (void *) 0xFF42AF70;  /* RegisterTwoInTwoOutJpegPathCompleteCBR */
        TTL_SetFlags    = (void *) 0xFF363148;  /* called next, with PictureType as arguments */ 
        TTL_Start       = (void *) 0xFF42c034;  /* called next; starts the EDmac transfers */
        TTL_Stop        = (void *) 0xFF42B1BC;  /* called right after sssStopMem1ToRawPath */
        TTL_Finish      = (void *) 0xFF42C0A4;  /* called next; calls UnlockEngineResources and returns output size from JpCoreCompleteCBR */
    }

    if (is_camera("6D", "1.1.6"))
    {
        /* ProcessTwoInTwoOutLosslessPath, 6D 1.1.6 */
        TTL_SetArgs     = (void *) 0xFF3491C8;  /* fills TTL_Args struct; PictureSize(Mem1ToRaw) */
        TTL_Prepare     = (void *) 0xFF4129BC;  /* called right after ProcessTwoInTwoOutLosslessPath(R) Start; */
                                                /* calls [TTL] GetPathResources and sets up the encoder for RAW/SRAW/MRAW */
        TTL_RegisterCBR = (void *) 0xFF411A44;  /* RegisterTwoInTwoOutLosslessPathCompleteCBR */
        TTL_SetFlags    = (void *) 0xFF359C78;  /* called next, with PictureType as arguments */
        TTL_Start       = (void *) 0xFF412A2C;  /* called next; starts the EDmac transfers */
        TTL_Stop        = (void *) 0xFF412A64;  /* called right after sssStopMem1ToRawPath */
        TTL_Finish      = (void *) 0xFF412A9C;  /* called next; calls UnlockEngineResources and returns output size from JpCoreCompleteCBR */
    }

    if (is_camera("70D", "1.1.2"))
    {
        /* ProcessTwoInTwoOutLosslessPath, 70D 1.1.2 */
        TTL_SetArgs     = (void *) 0xFF362174;  /* fills TTL_Args struct; PictureSize(Mem1ToRaw) */
        TTL_Prepare     = (void *) 0xFF438404;  /* called right after ProcessTwoInTwoOutLosslessPath(R) Start; */
                                                /* calls [TTL] GetPathResources and sets up the encoder for RAW/SRAW/MRAW */
        TTL_RegisterCBR = (void *) 0xFF43748C;  /* RegisterTwoInTwoOutLosslessPathCompleteCBR */
        TTL_SetFlags    = (void *) 0xFF372AE8;  /* called next, with PictureType as arguments */
        TTL_Start       = (void *) 0xFF438474;  /* called next; starts the EDmac transfers */
        TTL_Stop        = (void *) 0xFF4384AC;  /* called right after sssStopMem1ToRawPath */
        TTL_Finish      = (void *) 0xFF4384E4;  /* called next; calls UnlockEngineResources and returns output size from JpCoreCompleteCBR */
    }

    if (is_camera("5D2", "2.1.2"))
    {
        /* ProcessTwoInTwoOutJpegPath, 5D2 2.1.2 (one blocking call, see above) */
        TTJ_SetArgs_5D2 = (void *) 0xFF9BEB18;  /* fills TTJ_Args struct; PictureSize(Mem1ToRaw) */
        TTJ_Process_5D2 = (void *) 0xFFA59B58;  /* lock, setup, start, wait, stop, unlock; returns output size */
    }

    lossless_sem = create_named_semaphore(NULL, SEM_CREATE_LOCKED);
    
    if (is_camera("700D", "*") || is_camera("650D", "*") || is_camera("EOSM", "*") || is_camera("100D", "*"))
    {
        uint32_t resources[] = {
            0x00000 | edmac_channel_to_index(edmac_write_chan),
            0x10000 | edmac_channel_to_index(edmac_read_chan),
            0x20005,
            0x20016,
            0x30002,
            0x50034,
            0x5002d,
            0x50010,
            0x90001,
            0x90000,
            0xa0000,
            0x160000,
            0x260000,
            0x260001,
            0x260002,
            0x260003,
        };

        TTL_ResLock = CreateResLockEntry(resources, COUNT(resources));
    }
    else if (is_camera("5D3", "*") || is_camera("6D", "*"))
    {
        uint32_t resources[] = {
            0x00000 | edmac_channel_to_index(edmac_write_chan),
            0x10000 | edmac_channel_to_index(edmac_read_chan),
            0x30001,    /* Read connection 1 (uncompressed input) */
            0x2002d,    /* Write connection 45 (compressed output) */
          //0x20016,    /* Write connection 22 (for WR2 - not used) */
            0x50034,
            0x5002d,
            0x50010,
            0x90001,
            0x230000,
            0x160000,
            0x260000,
            0x260001,
            0x260002,
            0x260003,
        };
        
        TTL_ResLock = CreateResLockEntry(resources, COUNT(resources));
    }
    else if (is_camera("70D", "*"))
    {
        uint32_t resources[] = {
            0x00000 | edmac_channel_to_index(edmac_write_chan),
            0x10000 | edmac_channel_to_index(edmac_read_chan),
            0x30001,    /* Read connection 1 (uncompressed input) */
            0x2002d,    /* Write connection 45 (compressed output) */
          //0x20016,    /* Write connection 22 (for WR2 - not used) */
            0x50034,
          //0x5002d,    /* workaround, TTL_Prepare stucks otherwise if called in LV */
            0x50010,
            0x90001,
            0x230000,
            0x160000,
            0x260000,
            0x260001,
            0x260002,
            0x260003,
        };
        
        TTL_ResLock = CreateResLockEntry(resources, COUNT(resources));
    }
    
    else if (is_camera("5D2", "*"))
    {
        /* Canon's own list for the RAW path (RAM 0xFC00, 17 entries, read from the camera);
         * it uses Canon's EDMAC channels, which are idle once LiveView is paused */
        uint32_t resources[] = {
            0x10002,    /* read EDMAC index 2 = channel 0x0A (RD1, uncompressed input) */
            0x00003,    /* write EDMAC channel 3 (WR1, compressed output) */
            0x00004,    /* write EDMAC channel 4 (WR2, not started) */
            0x30000,    /* read connection 0 */
            0x20005,    /* write connection 5 */
            0x20016,    /* write connection 22 */
            0x50003,
            0x5000d,
            0x5000f,
            0x5001a,
            0x80000,
            0x90000,
            0xa0000,
            0x160000,
            0x130003,
            0x130004,
            0x130005,
        };

        TTL_ResLock = CreateResLockEntry(resources, COUNT(resources));
    }

    /* optionally initialize the decompression routines */
    decompress_init();

    /* return 1 if everything looks alright */
    return (TTL_Start || TTJ_Process_5D2) && lossless_sem && TTL_ResLock;
}

static uint32_t start_time = 0;

/* 5D2: Canon takes these from a per-shot object ("VARIABLE_DATA"+0x338), which only
 * exists during a regular capture. Values read from the camera right after a RAW shot.
 * They change slightly between shots (high halves track the measured black level,
 * 0x400 / 0x3FA / 0x3FD); these are from the shot that RAW_OBWB below comes from. */
static uint32_t TTJ_JpCore_5D2[] = {
    0xC0F12040, 0x00000001,
    0xC0F12044, 0x03FD04D0,
    0xC0F12048, 0x03FD0380,
    0xC0F1204C, 0x03FD0380,
    0xC0F12050, 0x03FD14DB,
    0xC0F12054, 0x3A8107FB,
    0xFFFFFFFF, 0xFFFFFFFF,
};

/* 5D2: Canon's RAW_OBWB table (args +0x28, copied to C0F0E000, 0x188 bytes),
 * read from the camera right after a RAW shot. It configures the OB / WB stage
 * in front of JPCORE as pass-through (black offset 0 at +0x14, all gains 0x400).
 * Without it, the stage keeps LiveView's settings and the output is
 * gain * (raw - 1024) + 512 per channel. The words at +0xF0..+0x10C are black
 * levels measured on that shot. */
static uint32_t TTJ_RAW_OBWB_5D2[0x188 / 4] = {
    0x00000000, 0x00000000, 0x00000000, 0x00000000, 0x000001FF, 0x00000000, 0x00000000, 0x00000000,  /* +0x000 */
    0x00000000, 0x00000000, 0x00000000, 0x00000000, 0x00000000, 0x00000000, 0x00000000, 0x00000000,  /* +0x020 */
    0x00000000, 0x00000000, 0x00000000, 0x00000000, 0x00000000, 0x00000400, 0x00000400, 0x00000400,  /* +0x040 */
    0x00000400, 0x00000400, 0x00000400, 0x00000400, 0x00000400, 0x00000400, 0x00000400, 0x00000400,  /* +0x060 */
    0x00000400, 0x00000400, 0x00000400, 0x00000400, 0x00000400, 0x00000000, 0x00000001, 0x00000000,  /* +0x080 */
    0x00000000, 0x00000000, 0x00000000, 0x00000000, 0x00000000, 0x00000000, 0x00000000, 0x00000000,  /* +0x0A0 */
    0x00000000, 0x00000000, 0x00000000, 0x00000000, 0x00000000, 0x00000000, 0x00000000, 0x00000000,  /* +0x0C0 */
    0x00000000, 0x00000000, 0x00000000, 0x00000000, 0x000003FD, 0x00003FFF, 0x00000001, 0x00000000,  /* +0x0E0 */
    0x000003FE, 0x000003FF, 0x000003FB, 0x000003FB, 0x00000000, 0x00000000, 0x00000000, 0x00000000,  /* +0x100 */
    0x00000000, 0x00000000, 0x00000000, 0x00000000, 0x00000000, 0x00000000, 0x00000000, 0x00000000,  /* +0x120 */
    0x00000000, 0x00000000, 0x00000000, 0x00000000, 0x00000000, 0x00000000, 0x00000000, 0x00000000,  /* +0x140 */
    0x00000000, 0x00000000, 0x00000000, 0x00000000, 0x00000000, 0x00000000, 0x00000000, 0x00000000,  /* +0x160 */
    0x00000000, 0x00000000,  /* +0x180 */
};

/* 5D2: geometry hardcoded in Canon's RAW engio list (prep1) */
#define TTJ_5D2_WIDTH   5792
#define TTJ_5D2_HEIGHT  3804
#define TTJ_5D2_ROWS    3808    /* rows read by RD1 (C0F04A10) */

/* first milestone: also write the input, the output and a log to ML/LOGS,
 * so the output can be decoded on a PC and compared pixel by pixel */
#define TTJ_5D2_DUMP 1

#if TTJ_5D2_DUMP
static void lossless_5D2_dump(void * src, void * out, uint32_t out_size,
                              int err, uint32_t elapsed_us)
{
    FILE * f = FIO_CreateFile("ML/LOGS/LJ92.LOG");
    if (f)
    {
        my_fprintf(f, "5D2 lossless test\n");
        my_fprintf(f, "src %x, out %x, out_size %d, err %d, elapsed %d us\n",
            src, out, out_size, err, elapsed_us);
        my_fprintf(f, "args after the call:\n");
        uint32_t * a = (uint32_t *) &TTJ_Args_5D2;
        for (int i = 0; i < (int) (sizeof(TTJ_Args_5D2) / 4); i++)
        {
            my_fprintf(f, "+%02x %08x\n", i * 4, a[i]);
        }
        FIO_CloseFile(f);
    }

    f = FIO_CreateFile("ML/LOGS/LJ92SRC.RAW");
    if (f)
    {
        FIO_WriteFile(f, src, TTJ_5D2_WIDTH * 14 / 8 * TTJ_5D2_ROWS);
        FIO_CloseFile(f);
    }

    if (out && out_size)
    {
        f = FIO_CreateFile("ML/LOGS/LJ92OUT.BIN");
        if (f)
        {
            FIO_WriteFile(f, out, out_size);
            FIO_CloseFile(f);
        }
    }
}
#endif

static int lossless_compress_raw_5D2(
    struct memSuite * dst_suite, void * src,
    int src_width, int src_x, int src_y,
    int width, int height
)
{
    /* Canon's RAW engio lists are for full-resolution frames only;
     * other sizes need patched copies (not done yet) */
    if (src_x || src_y || src_width != TTJ_5D2_WIDTH ||
        width != TTJ_5D2_WIDTH || height < TTJ_5D2_HEIGHT)
    {
        printf("[TTJ] %dx%d (%d,%d of %d): not supported yet\n",
            width, height, src_x, src_y, src_width);
        return -4;
    }

    struct TwoInTwoOutJpegPath_args_5D2 * a = &TTJ_Args_5D2;

    /* PictureSize 0 = RAW: Canon's resource list, engio lists, channels, type 14 */
    TTJ_SetArgs_5D2(0, a, 0);

    if (a->SamplePrecision != 14 || a->engio_prep1 == NULL)
    {
        printf("[TTJ] unexpected SetArgs result (%d, %x)\n", a->SamplePrecision, a->engio_prep1);
        return -5;
    }

    /* what Canon's caller sets after SetArgs (FF888CE0..FF888E78) */
    a->xRes         = TTJ_5D2_WIDTH;
    a->yRes         = TTJ_5D2_HEIGHT;
    a->engio_prep2  = TTJ_JpCore_5D2;
    a->goes_to_obinteg_C0F0E000_size_0x188 = TTJ_RAW_OBWB_5D2;
    a->RD1_Address  = UNCACHEABLE(src);
    /* RD1_Offset stays at SetArgs' -0x1C (16 px): the input path crops 16 px on the
     * left (and 2 rows at the top), so the output starts at the buffer start */
    a->RD2_Address  = 0;
    a->WR1_MemSuite = dst_suite;
    a->WR2_Address  = 0;    /* WR2 is only started when nonzero (FFA591A8) */

    void * WR1_Address = (dst_suite)
        ? GetMemoryAddressOfMemoryChunk(GetFirstChunkFromSuite(dst_suite))
        : 0;

    if (WR1_Address)
    {
        MEM(WR1_Address) = 0;
    }

    /* output size is only written when JPCORE reports completion;
     * on timeout FFA59B58 still cleans up and returns 0, leaving this at 0 */
    uint32_t output_size = 0;

    start_time = get_us_clock();
    int err = TTJ_Process_5D2(TTL_ResLock, a, &output_size);
    uint32_t elapsed = get_us_clock() - start_time;

    printf("[TTJ] err %d, output %d bytes, %d us\n", err, output_size, elapsed);

#if TTJ_5D2_DUMP
    lossless_5D2_dump(src, WR1_Address, output_size, err, elapsed);
#endif

    if (output_size == 0)
    {
        /* err set: LockEngineResources failed (FFA59B0C);
         * otherwise timeout or JPCORE error (FFA59B58 cleans up and returns 0) */
        return err ? -6 : -2;
    }

    if (WR1_Address && MEM(WR1_Address) != 0xC4FFD8FF)
    {
        return -3;
    }

    return output_size;
}

/* returns output size if successful, negative on error */
int lossless_compress_raw_rectangle(
    struct memSuite * dst_suite, void * src,
    int src_width, int src_x, int src_y,
    int width, int height
)
{
    if (TTJ_Process_5D2 && TTL_ResLock)
    {
        return lossless_compress_raw_5D2(dst_suite, src, src_width, src_x, src_y, width, height);
    }

    if (!TTL_ResLock || !lossless_sem || !TTL_Start)
    {
        /* not initialized */
        return -1;
    }

    /* setup photo quality (valid values: 0=RAW, 1=MRAW, 2=SRAW, 14, 15) */
    TTL_SetArgs(0, &TTL_Args, 0);

    /* trick the encoder so it configures slice width = image width */
    /* we'll have two slices on top of each other; this will give
     * valid lossless DNG as well, if we prepend a header :)
     */
    TTL_Args.xRes = width;
    TTL_Args.yRes = height;

    /* Default EDMAC channels may be used in LiveView;
     * use the ones from edmac_memcpy instead */
    TTL_Args.RD1_Channel = edmac_read_chan;
    TTL_Args.WR1_Channel = edmac_write_chan;

    /* set starting point (top-left corner) */
    /* we need to skip a multiple of 8 pixels horizontally for raw_pixblock alignment
     * and an even number of pixels vertically, to preserve the Bayer pattern
     */
    uint32_t src_adjusted = ((uint32_t)src & 0x1FFFFFFF)
        + ((src_x/8*8) + (src_y/2*2) * src_width) * 14/8;

    /* set buffers */
    TTL_Args.WR1_MemSuite = dst_suite;
    TTL_Args.WR2_Address  = 0;
    TTL_Args.RD1_Address  = (void *) src_adjusted;

    /* to check whether the compression was successful */
    /* note: in dummy mode, dst_suite is NULL - don't check this case */
    void * WR1_Address = (TTL_Args.WR1_MemSuite)
        ? GetMemoryAddressOfMemoryChunk(GetFirstChunkFromSuite(TTL_Args.WR1_MemSuite))
        : 0;

    if (WR1_Address)
    {
        MEM(WR1_Address) = 0;
    }

    /* configure the processing modules */
    TTL_Prepare(TTL_ResLock, &TTL_Args);

    if (is_camera("6D", "*") ||
        is_camera("650D", "*"))
    {
        /* not sure what exactly these do, but they seem to be required to get correct image
         * taken from register log diffs: http://www.magiclantern.fm/forum/index.php?topic=18443.msg197987#msg197987 */
        EngDrvOut(0xC0F37610, 0);       /* 0x11 on 650D, 0 on all other D5 (not sure if needed) */
        EngDrvOut(0xC0F37628, 0x71000); /* 72000 on 650D, 71000 on all other D5 */
        EngDrvOut(0xC0F3762C, 0x71000); /* 72000 on 650D, 71000 on all other D5 */
        EngDrvOut(0xC0F37630, 0x71000); /* 72000 on 650D, 71000 on all other D5 */
        EngDrvOut(0xC0F37634, 0x71000); /* 72000 on 6D and 650D, 71000 on all other D5 */
        EngDrvOut(0xC0F3763C, 0);       /* 0x1000000 on 6D and 650D, 0 on all other D5 */
        EngDrvOut(0xC0F37640, 0);       /* 0x2000000 on 6D and 650D, 0 on all other D5 */
        EngDrvOut(0xC0F37644, 0);       /* 0x4000000 on 6D and 650D, 0 on all other D5 */
        EngDrvOut(0xC0F37648, 0);       /* 0x8000000 on 6D and 650D, 0 on all other D5 */
    }

    if (is_camera("70D", "*"))
    {
        /* 70D is different */
        // EngDrvOut(0xC0F373F4, 0x00000000);  /* alternative fixing method; 0x7FFF7FFF on 70D */
        EngDrvOut(0xC0F373B4, 0);              /* 0x11 on 70D */
        EngDrvOut(0xC0F37300, PACK32(width    - 1,  height/2  - 1));  /* 0xE7B0ADF on 70D */
        EngDrvOut(0xC0F373E8, PACK32(width    - 1,  height/2  - 1));  /* 0xE7B0ADF on 70D */
    }
    else if (is_camera("DIGIC", "5"))
    {
        /* all other D5 models use this register instead */
        /* the hardware encoder (and other image processing modules that might be used)
         * expect slice width and slice height for these registers
         * Canon configuration: slice width = image width / 2, slice height = image height
         * our configuration:   slice width = image width, slice height = image height / 2 */
        EngDrvOut(0xC0F375B4, PACK32(width    - 1,  height/2  - 1));  /* 0xF6D0B8F on 5D3 */
    }

    if (is_camera("DIGIC", "5"))
    {
        /* required for full-res silent pictures? */
        /* this one doesn't seem to refer to slice size, but to total image size */
        /* sht_mirror shows 0xC0F13000 - 0xC0F13064 as RABBIT */
        EngDrvOut(0xC0F13068, PACK32(width    - 1,  height    - 1));  /* 0xF6D171F on 5D3 */
    }

    /* there are a few registers possibly related to RD2/WR2 - we don't use them */
    /* 5D3: [0xC0F375B4] <- 0xF6D0B8F, [0xC0F12020] <- 0x18A0127  =>  slice width / 10, slice height / 10 */
    /* 6D:  [0xC0F375B4] <- 0xE7B0ADF, [0xC0F12020] <- 0x13400E7  =>  slice width / 12, slice height / 12 */
    /* 70D: [0xC0F37300] <- 0xE7B0ADF, [0xC0F12020] <- 0x13400E7  =>  slice width / 12, slice height / 12 */
    /* 650D, 700D, 100D, EOSM, EOSM2:
     *      [0xC0F375B4] <- 0xDC70A4F, [0xC0F12020] <- 0x1B80149  =>  slice width / 8, slice height / 8 */
    if (0)
    {
        /* values for 5D3 - just for future reference */
        EngDrvOut(0xC0F12010,        width    - 1                 );  /* 0xB8F     */
        EngDrvOut(0xC0F12014, PACK32(width    - 1,  height/2  - 1));  /* 0xF6D0B8F */
        EngDrvOut(0xC0F1201C,        width/10 - 1                 );  /* 0x127     */
        EngDrvOut(0xC0F12020, PACK32(width/10 - 1,  height/20 - 1));  /* 0x18A0127 */
    }

    /* need to read the image data in 2 slices
     * default configuration is 2 vertical slices;
     * however, using 2 horizontal slices makes it easy
     * to just slap a DNG header, resulting in valid output.
     * 
     * => the input EDMAC will simply read the image as usual.
     */
    struct edmac_info RD1_info = {
        .xb     = width * 14/8,
        .yb     = height - 1,
        .off1b  = src_width * 14/8 - width * 14/8,
    };

    SetEDmac(TTL_Args.RD1_Channel, TTL_Args.RD1_Address, &RD1_info, TTL_Args.RD1_Flags);

    if (verbose >= 2)
    {
        const char * WR1_SizeFmt = format_memory_size(GetSizeOfMemoryChunk(GetFirstChunkFromSuite(dst_suite)));
        printf("[TTL] %dx%d %dbpp\n", TTL_Args.xRes, TTL_Args.yRes, TTL_Args.SamplePrecision);
        printf(" WR1: %x EDMAC#%d<%d> (%x %s)\n",  WR1_Address,  TTL_Args.WR1_Channel, TTL_Args.WR1_Connection, TTL_Args.WR1_MemSuite, WR1_SizeFmt);
        printf(" WR2: %x EDMAC#%d<%d>\n", TTL_Args.WR2_Address,  TTL_Args.WR2_Channel, TTL_Args.WR2_Connection);
        printf(" RD1: %x EDMAC#%d<%d>\n", TTL_Args.RD1_Address,  TTL_Args.RD1_Channel, TTL_Args.RD1_Connection);
        printf(" RD2: %x EDMAC#%d<%d>\n", TTL_Args.RD2_Address,  TTL_Args.RD1_Channel, TTL_Args.RD2_Connection);
    }

    /* register our CBR, to be called when finished */
    TTL_RegisterCBR(4, LosslessCompleteCBR, 0);
    
    /* this changes a few registers that appear to be bit fields */
    TTL_SetFlags(0x10000);

    /* time the operation */
    start_time = get_us_clock();

    /* this starts the EDmac channels */
    TTL_Start(&TTL_Args);

    /* wait until finished */
    int err = take_semaphore(lossless_sem, 1000);

    if (verbose >= 2)
    {
        uint32_t stop_time = get_us_clock();
        printf("[TTL] Elapsed time: %d us\n", (int)(stop_time - start_time));
    }

    /* stop processing; this will report output size */
    uint32_t output_size = 0;
    TTL_Stop(&TTL_Args);
    TTL_Finish(TTL_ResLock, &TTL_Args, &output_size);

    if (verbose >= 1)
    {
        /* compute input size (uncompressed) */
        uint32_t input_size = RD1_info.xb * (RD1_info.yb + 1);

        int ratio_x100 = output_size * 10000.0 / input_size;
        printf("[TTL] Output size : %s (%s%d.%02d%%)\n", format_memory_size(output_size), FMT_FIXEDPOINT2(ratio_x100));
    }

    if (err)
    {
        return -2;
    }

    /* do we have valid JPEG data in the output buffer? */
    if (WR1_Address && MEM(WR1_Address) != 0xC4FFD8FF)
    {
        return -3;
    }

    return output_size;
}

int lossless_compress_raw(struct raw_info * raw_info, struct memSuite * output_memsuite)
{
    return lossless_compress_raw_rectangle(
        output_memsuite, raw_info->buffer,
        raw_info->width, 0, 0,
        raw_info->width, raw_info->height
    );
}


/* decompression stuff wizardry goes here */
struct DecodeLossless_args
{
    void *address;
    int depth_type;
    int is_not_16bpp;
    int x_2;
    int x_1;
    int x_mul;
    int y_2;
    int ysize_raw;
    int y_mul;
};

static void (*Setup_DecodeLosslessRawPath) (struct DecodeLossless_args *args, void (*read_cbr)(int *), void (*done_cbr)(int *), int *cbr_ctx, int *a5) = NULL;
static void (*Start_DecodeLosslessPath) (struct memSuite *a1) = NULL;
static void (*Cleanup_DecodeLosslessPath) (void) = NULL;
static struct semaphore *decompress_sem = NULL;

static void decompress_init()
{
    /* now check for the needed decompression functions */
    if (is_camera("5D3", "1.1.3"))
    {
        Setup_DecodeLosslessRawPath = (void *) 0xFF3CB010;
        Start_DecodeLosslessPath    = (void *) 0xFF3CB0D8;
        Cleanup_DecodeLosslessPath  = (void *) 0xFF3CB23C;
    }

    if (is_camera("5D3", "1.2.3"))
    {
        Setup_DecodeLosslessRawPath = (void *) 0xff3d3f04;
        Start_DecodeLosslessPath    = (void *) 0xff3d3fcc;
        Cleanup_DecodeLosslessPath  = (void *) 0xff3d4130;
    }
    
    if (is_camera("700D", "1.1.5"))
    {
        Setup_DecodeLosslessRawPath = (void *) 0xFF4294DC;
        Start_DecodeLosslessPath    = (void *) 0xFF4295A4;
        Cleanup_DecodeLosslessPath  = (void *) 0xFF429708;
    }

    if (is_camera("650D", "1.0.4"))
    {
        Setup_DecodeLosslessRawPath = (void *) 0xFF4259F4;
        Start_DecodeLosslessPath    = (void *) 0xFF425ABC;
        Cleanup_DecodeLosslessPath  = (void *) 0xFF425C20;
    }

    if (is_camera("EOSM", "2.0.2"))
    {
        Setup_DecodeLosslessRawPath = (void *) 0xFF42DBD0;
        Start_DecodeLosslessPath    = (void *) 0xFF42DC98;
        Cleanup_DecodeLosslessPath  = (void *) 0xFF42DDFC;
    }

    if (is_camera("100D", "1.0.1"))
    {
        Setup_DecodeLosslessRawPath = (void *) 0xFF42F4C8;
        Start_DecodeLosslessPath    = (void *) 0xFF42F590;
        Cleanup_DecodeLosslessPath  = (void *) 0xFF42F6F4;
    }

    if (is_camera("6D", "1.1.6"))
    {
        Setup_DecodeLosslessRawPath = (void *) 0xFF409218;
        Start_DecodeLosslessPath    = (void *) 0xFF4092E0;
        Cleanup_DecodeLosslessPath  = (void *) 0xFF409444;
    }

    if (is_camera("70D", "1.1.2"))
    {
        Setup_DecodeLosslessRawPath = (void *) 0xFF4309A4;
        Start_DecodeLosslessPath    = (void *) 0xFF430A6C;
        Cleanup_DecodeLosslessPath  = (void *) 0xFF430BD0;
    }

    /* all functions known? having the semaphore is an indicator we can decompress */
    if (Setup_DecodeLosslessRawPath && Start_DecodeLosslessPath && Cleanup_DecodeLosslessPath)
    {
        decompress_sem = create_named_semaphore("decompress_sem", SEM_CREATE_LOCKED);
    }
}

/* this one is called when decompression is done */
static void DecodeLossless_DoneCBR(int *done)
{
    give_semaphore(decompress_sem);
}

/* the read cbr is not used */
static void DecodeLossless_ReadCBR(int *done)
{
}

int lossless_decompress_raw(
    struct memSuite * src, void * dst,
    int width, int height,
    int output_bpp
)
{
    if (!decompress_sem)
    {
        return -1;
    }

    struct DecodeLossless_args decode_opts;

    decode_opts.address = dst;
    decode_opts.depth_type = 4;
    decode_opts.is_not_16bpp = (output_bpp == 16) ? 0 : 1;
    decode_opts.x_1 = width;
    decode_opts.x_2 = 0;
    decode_opts.x_mul = 0;
    decode_opts.ysize_raw = height;
    decode_opts.y_2 = 0;
    decode_opts.y_mul = 0;
    
    /* we dont use that one */
    int done = 0;
    Setup_DecodeLosslessRawPath(&decode_opts, DecodeLossless_ReadCBR, DecodeLossless_DoneCBR, &done, &done);
    Start_DecodeLosslessPath(src);
    
    /* wait for decompression to finish */
    take_semaphore(decompress_sem, 0);
    
    /* clean up */
    Cleanup_DecodeLosslessPath();

    return 0;
}
