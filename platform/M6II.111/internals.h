/**
 * Camera internals for M6II 1.1.1
 */

/** This camera has a DIGIC VIII chip */
#define CONFIG_DIGIC_VIII

/** EDMAC channel map decoded from ROM, see include/platform/edmac_channels.h */
#define CONFIG_EDMAC_CHANNEL_TABLE

// has inter-core RPC (so far this has always been dependent on SGI, 0xc)
#define CONFIG_RPC

#define CONFIG_PLATFORM_POST_INIT   // boot probe, function_overrides.c
// Cam has MMU (by itself, does nothing, see CONFIG_MMU_REMAP)
#define CONFIG_MMU

/** Digic 8 does not have bitmap font in ROM, try to load it from card **/
#define CONFIG_NO_BFNT

/** disable SRM for now
 * in current state SRM_AllocateMemoryResourceFor1stJob makes camera crash
 * even if just one buffer is requested.
 */
#define CONFIG_MEMORY_SRM_NOT_WORKING

/* srm_malloc_suite gives Canon's movie recorder work memory instead
 * (movrec_mem.c), valid while recording raw in movie mode LiveView */
#define CONFIG_MEMORY_SRM_MOVREC

/* FIO_CreateFileSC / FIO_CloseFileSC: Canon's speed class file mode */
#define CONFIG_FIO_SPEED_CLASS

/* has LV */
#define CONFIG_LIVEVIEW

/* Copied from R/RP, where hooking EVF_STATE once crashed EvfCap.  On M6II the
 * EVF_STATE hook is installed and running (seen in a LV RAM dump). */
#define CONFIG_STATE_OBJECT_HOOKS

/* vsync from EvfState: same numbering as D5, input 5 = evfReadOutDoneInterrupt
 * (0xe069b0a2), input 4 = evfSetParamInterrupt (0xe069ae98); table 0xe0b29fd0.
 * Needed for mlv_lite's CBR_VSYNC. */
#define CONFIG_EVF_STATE_SYNC

/* movie mode isn't SHOOTMODE_MOVIE on the mode dial: use PROP_FIXED_MOVIE
 * (0x80030037, 1 = on), as Canon's own handlers do (e.g. 0xe029d50c,
 * BLEMgrState; 0xe08e11d6, DlgGuidanceModeDial) */
#define CONFIG_MOVIE_MODE_FROM_FIXED_MOVIE

/* Frame timing registers not found yet (not at the D6/D7 0xD0006000 block;
 * D8 engio_write 0xe0663a7a writes lists without a shadow copy), so
 * fps_get_current_x1000 measures the vsync period instead. */
#define CONFIG_FPS_MEASURED_FROM_VSYNC

/* ISO / shutter / aperture / AE comp properties are 16-bit values in
 * 1/768 EV (logged with dmlog: ISO 100 = 0x0F00, f/22 = 0x1B00) */
#define CONFIG_PROP_EXPO_1_768_EV

/** Canon's CR3 encoder can compress LV raw frames (src/crx_enc.c, mlv_lite) */
#define CONFIG_CRX_ENCODER

/** Canon's 1:1 sensor mode (the RAW burst's, 5376x3560) as a video source
 * in movie mode: Canon's CR3 encoder crops and compresses each frame,
 * mlv_lite records them (preshoot.c, src/preshoot.h) */
#define CONFIG_PRESHOOT_1TO1

/** PROP_BATTERY_REPORT has the LP-E6 layout with the LP-E17 too (byte 4
 * level, byte 5 recharge performance, "LP-E17" from byte 11), but the
 * level only takes Canon's icon steps (100, 67, 33, ...) */
#define CONFIG_BATTERY_INFO
#define CONFIG_BATTERY_COARSE
/* no PROP_BATTERY_HISTORY (0x0204000F not readable): no history table */
#define CONFIG_BATTERY_NO_HISTORY

/** Large total memory, leading to unusual memory mapping,
 * CACHEABLE / UNCACHEABLE changes
 */
#define CONFIG_MEM_2GB

#define CONFIG_MALLOC_STRUCT_V2

#define CONFIG_TASK_STRUCT_V2_SMP
#define CONFIG_TASK_ATTR_STRUCT_V5
