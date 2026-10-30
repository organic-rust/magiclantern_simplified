/*
 *  EOS M6 Mark II consts
 */

#define CARD_LED_ADDRESS            0xD01300D4   /* 0xD01300D8/DC: (no errors, no obvious reaction) */
#define LEDON                       0xD0002
#define LEDOFF                      0xC0003

#define CANON_ORIG_MMU_TABLE_ADDR 0xe0000000

#define BR_DCACHE_CLN_1     0xe0040068   /* first call to dcache_clean, before cstart */
#define BR_ICACHE_INV_1     0xe0040072   /* first call to icache_invalidate, before cstart */
#define BR_DCACHE_CLN_2     0xe00400a0   /* second call to dcache_clean, before cstart */
#define BR_ICACHE_INV_2     0xe00400aa   /* second call to icache_invalidate, before cstart */
#define BR_BR_CSTART        0xe00400c0   // indirect branch to cstart; the first branch is
                                         // absolute to original, we must patch
#define BR_BZERO32          0xe0040152   /* called from cstart */
#define BR_CREATE_ITASK     0xe00401b4   /* called from cstart */

// Used for copying and modifying ROM code before transferring control.
// Approximately: look at BR_ macros for the highest address, subtract MAIN_FIRMWARE_ADDR,
// align up.  This may not be exactly enough.  See boot-d678.c for longer explanation.
#define FIRMWARE_ENTRY_LEN 0x228

#define DRYOS_ASSERT_HANDLER  0x4000    //from debug_asset function, hard to miss
#define DRYOS_SGI_HANDLERS_PTR 0x402c // holds pointer to base of SGI handlers (each is 8 bytes, a pointer and something else)

#define MALLOC_STRUCT_ADDR    0x2d99c   // via meminfo -m

#define CANON_SHUTTER_RATING 100000

/**
IDLEHandler seems to be a goldmine of id -> name mapping... just a part of those
2002 START_PLAY_MODE
2003 START_MENU_MODE
2004 START_RTCSET_MODE

200F START_MENU_STROBO_MODE
2010 GUIMODE_MENU_MOVIE_SIZE

2014 START_MENU_FUTURE_SPECIAL_ADP
2015 missing
2016 GUIMODE_USB_ERR
2017 START_MENU_ISO_MODE
2018 START_SERVICE_AFADJUST
2019 START_SERVICE_MENU
201A START_MENU_DATE_TIME_ZONE
201B START_MENU_STROBO_SETTINGS_MODE
201C START_MENU_STROBO_SETTING_LV_MODE
201D START_MENU_GPS_FUNC_MODE
201E START_MENU_SAFEMODE_LENS
201F START_MENU_SAFEMODE_STROBO
2020 START_MENU_SAFEMODE_BATTERY_GRIP
2021 START_MENU_NETWORK_ERROR
2022 START_MENU_NETWORK_ERROR_LIVESTREAMING
2023 START_MENU_CMST_USB
2024 START_MENU_SMARTPHONE_BLE
2025 START_MENU_WIFI_MODE
2026 START_MENU_SMARTPHONE_BTC
2027 START_PLAYMENU_HINT_GUIDE
2028 START_PLAYMENU_SPECIAL_SCENE

202F START_MENU_FOLDER
2030 START_MENU_MECHALESS_SETTING

2033 START_MENU_LIVESTREAMING_DISCONNECT

3039 START_INFO_MODE

303A START_WARNING_SW1OFF

303C START_WARNING_DISABLE_LV

3046 START_WARNING_PANNING_ASSIST_UNSUPPORT_LENS
3047 START_WARNING_LOCK_BUTTON

5036 START_QR_ERASE_MODE

7038 START_QR_MAGNIFY_MODE

C048 Q menu
9081 Flash settings
9082 ISO menu
8484 shot priority

*/
/**
 More fun, GUIRQMODE (via GUI_ChangeMode_Post):
 0x8 GUIRQMODE_LV
 0xA GUIRQMODE_OLC
 */
#define CURRENT_GUI_MODE   (*(int*)0x8484) // via SetGUIRequestMode
#define GUIMODE_PLAY 0x2002
#define GUIMODE_MENU 0x2003
#define GUIMODE_ML_MENU (lv ? 0xc048 : GUIMODE_MENU) // Use Q mode dialog when in LV.  You want to find a dialog
                                                     // that is available in every different LV sub-mode (e.g. Av, P, A+).
                                                     // If you get this wrong, ML can think cam is not in LV,
                                                     // and will restrict features visible in ML menus.
//#define GUIMODE_ML_MENU (RECORDING ? 0 : lv ? 0x68 : GUIMODE_MENU) // TODO: y

#define GMT_FUNCTABLE               0xe096b554           //from gui_main_task
#define GMT_NFUNCS                  0x7                  //size of table above



#define LV_OVERLAYS_MODE MEM(0x16800)  // via LvInfoToggle_Update()


#define LVAE_STRUCT 0x5167c // First value written in 0x2251b1e
#define CONTROL_BV      (*(uint16_t*)(LVAE_STRUCT+0x28)) // via "lvae_setcontrolbv"
#define CONTROL_BV_TV   (*(uint16_t*)(LVAE_STRUCT+0x3e)) // via "lvae_setcontrolaeparam"
#define CONTROL_BV_AV   (*(uint16_t*)(LVAE_STRUCT+0x40)) // via "lvae_setcontrolaeparam"
#define CONTROL_BV_ISO  (*(uint16_t*)(LVAE_STRUCT+0x42)) // via "lvae_setcontrolaeparam"
#define CONTROL_BV_ZERO (*(uint16_t*)(LVAE_STRUCT+0x44)) // via "lvae_setcontrolaccumh"
#define LVAE_DISP_GAIN  (*(uint16_t*)(LVAE_STRUCT+0x70)) // via "lvae_setdispgain"
#define LVAE_MOV_M_CTRL (*(uint8_t* )(LVAE_STRUCT+0x24)) // via "lvae_setmoviemanualcontrol"

#define LVAE_ISO_STRUCT 0x69fe4
#define LVAE_ISO_MIN    (*(uint8_t* )LVAE_ISO_STRUCT + 0x0E) // via string: ISOMin:%d

      //#define LVAE_ISO_HIS    (*(uint8_t* )(LVAE_STRUCT+0xXX)) // no idea, not referenced in ./src?!
      //#define LVAE_ISO_SPEED  (*(uint8_t* )(LVAE_STRUCT+0xXX))  //WRONG, not sure how to follow

/*
On R DispVram was 0x374, on M6II it is 0x474... nice.

VramState, accidentally those match R.
IDK what's on 4th and 5th line, wasn't able to trigger it... yet?
EC811[1]>VramState
*************** Panel ***************
,,0x9f420000,0x9f814800,0x9fc09000

*************** Evf ***************
,,0x9f420000,0x9f814800,0x9fc09000

*************** Line ***************
,,0x74c10000,0x75fd6800,0x7739d000

***************  ***************
,,0x00000000,0x00000000,0x00000000

*************** FHD ***************
,,0x00000000,0x00000000,0x00000000
*/
// Note that all three regular buffers are in Uncacheable-only region!
#define YUV422_LV_BUFFER_1   0x9F230000 // For CleanHDMI 0x76087000
#define YUV422_LV_BUFFER_2   0x9F624800 // For CleanHDMI 0x7744d800
#define YUV422_LV_BUFFER_3   0x9FA19000 // For CleanHDMI 0x78814000

#define DISPVRAM_STRUCT_PTR *(unsigned int *)0xb090             // DispVram structure
#define DV_DISP_TYPE  *((unsigned int *)(DISPVRAM_STRUCT_PTR + 0xC))   // Display type mask
#define DV_VRAM_LINE  *((unsigned int *)(DISPVRAM_STRUCT_PTR + 0xA4))  // Pointer to LV buffer for HDMI output
#define DV_VRAM_PANEL *((unsigned int *)(DISPVRAM_STRUCT_PTR + 0xAC))  // Pointer to LV buffer for Panel output
#define DV_VRAM_EVF   *((unsigned int *)(DISPVRAM_STRUCT_PTR + 0xB4))  // Pointer to LV buffer for EVF output

/* Hardcoded to Panel for now. It would be easier if we can replace this with a
 * function call that would be put into functon_overrides.c. Then we could just
 * define full structs there instead of playing with pointers */
#define YUV422_LV_BUFFER_DISPLAY_ADDR DV_VRAM_PANEL
#define YUV422_LV_PITCH               736 // Is it 736 or 720? No scalling on XCM but UI is 720, OutputChunk 736
#define YUV422_HD_BUFFER_DMA_ADDR     0x0 // TODO: Fix it, null pointer!. It expects this to be shamem_read(some_DMA_ADDR)

// At time of writing R uses here "DispOperator_PropertyMasterSetDisplayTurnOffOn (%d)"
// but I already found that our code expects this to be display statobject state
#define _DISPDEV_STRUCT *(unsigned int *)0xb088
#define DISPLAY_STATEOBJ (*(struct state_object **)_DISPDEV_STRUCT + 0x8)    // DispDevSingleScreenState on D6 and up
#define DISPLAY_IS_ON (DISPLAY_STATEOBJ->current_state != 0)

#define INFO_BTN_NAME               "INFO"
#define Q_BTN_NAME                  "Q/SET"


#define NUM_PICSTYLES 11

#define MIN_MSLEEP 11
// TODO: What is GUISTATE?
#define PLAY_MODE (gui_state == GUISTATE_PLAYMENU && CURRENT_GUI_MODE == GUIMODE_PLAY)
#define MENU_MODE (gui_state == GUISTATE_PLAYMENU && CURRENT_GUI_MODE == GUIMODE_MENU)

/* 101AC marks start of WINSYS data structure.
 * Struct itself is easy to confirm as a few fields later there's a harcoded
 * pointer (exists in romcpy itself) to string "Window Instance".
 * As for "dirty bit flag", D6 to early D8 cameras have two flags - one prohibits
 * redraw from dialog_redraw only, 2nd prohibits redraw at all - this is
 * the one that we expected.
 * But then... they finally removed 2nd flag somewhere around M6II...
 * thus routing it to the existing one.
 */
#define WINSYS_BMP_DIRTY_BIT_NEG MEM(0x11ae4+0x14)
//address of XimrContext structure to redraw in FEATURE_VRAM_RGBA
#define XCM_PTR *(unsigned int *)0x78ce8
#define XIMR_CONTEXT ((unsigned int *)(XCM_PTR + 0x10))

// Now it goes to GRAPHICS_ImageZoom*... eg GRAPHICS_ImageZoomDown e053dd0a
// It calls e041744c that returns from 0x1000C... confirmed on camera
// and behaves like 5D3 description.
#define IMGPLAY_ZOOM_LEVEL_ADDR (0x1000C)
#define IMGPLAY_ZOOM_LEVEL_MAX 14
// commented out due to some dependency in af_patterns.c I don't understand
// wants IMGPLAY_ZOOM_POS_DELTA_X/Y and yet 5D3 doesn't define those
//#define IMGPLAY_ZOOM_POS_X MEM(0x10100) // Saved just before ZoomResult CenterX print
//#define IMGPLAY_ZOOM_POS_Y MEM(0x10104)
#define IMGPLAY_ZOOM_POS_X_CENTER 360
#define IMGPLAY_ZOOM_POS_Y_CENTER 240

/* Looking at older models it is "whatever fits" on given model */
#define HALFSHUTTER_PRESSED 0
// Possibly look around e0479db2
// Focusinfo is fn f81bc08c at 5D3 this seems to be the successor
#define FOCUS_CONFIRMATION 0                    // fake bool
#define LV_BOTTOM_BAR_DISPLAYED 0x0             // fake bool
#define AUDIO_MONITORING_HEADPHONES_CONNECTED 0 // fake bool


// For feature "keep ML on format". Requires winsys structs tho.
#define DIALOG_MnCardFormatBegin   (0x20698) // just before StartMnCardFormatBeginApp it is checked to be DialogClass
#define DIALOG_MnCardFormatExecute (0x24e9c) // similar, MnCardFormatexcuteApp; yes - typos in name.
#define FORMAT_BTN       BGMT_INFO
#define FORMAT_BTN_NAME  "[INFO]"
    #define FORMAT_STR_LOC   13     // Resource ID, to be checked

// Things definitely wrong.
// this block all copied from 50D, and probably wrong, though likely safe
/* LV raw source.  Canon's LiveViewDrive::Mem1Path writes the head of the
 * LV pipeline (raw type HEAD: before channel gains, white balance, shading
 * and defect correction) through EDMAC channel 3, EDOMAIN_EDMAC_1_WR_M1,
 * every frame: 1080p 2384x1348 at 14 bit, optical black at columns 0-47
 * and rows 0-35, RGGB from (0, 0).  Its pack unit is set from Mem1Path's
 * state (0x10970 + 0x6C: f0, f1, f2 = depth code, f3, f4; register
 * f1<<20 | f4<<16 | f0<<8 | f2<<4 | f3), every frame.  f1 = 1, f3 = 0:
 * ML's 16-bit word layout (Canon's setting); f1 = 0, f3 = 1: the D8
 * LSB-first layout the CR3 encoder reads.  Both hold at 14, 12 and 10
 * bit (f2 = 2 / 1 / 0); raw.c picks one (raw_lv_request_layout). */
#define RAW_LV_SOURCE_MEM1
#ifdef RAW_LV_SOURCE_MEM1
#define RAW_LV_EDMAC_CHANNEL_ADDR 0xD0420200
#define RAW_LV_MEM1_STATE         0x10970
#define RAW_LV_MEM1_PACK_FIELDS   0x6C
/* Mem1Path's per-frame code (RAM copy, 0x02285904) computes the row size
 * as width * 7 / 4 (14 bit only) in these two Thumb-2 instructions:
 * rsb r0, r0, r0, lsl #3 / ubfx r0, r0, #2, #29 */
#define RAW_LV_MEM1_XB_CODE       0x02285974
#else
/* EDMAC channel 1, EDOMAIN_EDMAC_1_WR_L0: Canon's processed LV raw (photo
 * 1928x932, 1080p 2328x1306, 4K 3488x1958; 12-bit, D8 native layout): white
 * balanced and corrected, starts on a green row, no optical black. */
#define RAW_LV_EDMAC_CHANNEL_ADDR 0xD0420000
/* EDMAC channel 48, EDOMAIN_EDMAC_2_RD_M0: Canon's LV display reads the raw
 * buffer back (USB register scan); raw.c keeps its depth matched. */
#define RAW_LV_READER_EDMAC_CHANNEL_ADDR 0xD0440300
#endif

/* Canon's applied exposure and white balance (AutoExposure, found over
 * USB by setting ISO / shutter / WB with PTP and diffing RAM).  Tv and Sv
 * in 1/768 EV: ISO 100 = 3840, 1/50
 * (Canon's 1/8 EV code) = 4320 = 2^-5.625 s.  Sv is the ISO auto ISO
 * picked.  WB: Canon's colour temperature of the applied gains, and the
 * gains themselves (R, G1, G2, B; 1024 = 1.0) in the AE/WB state. */
#define AE_APPLIED_TV       (*(volatile int16_t *) 0x8B2BC)
#define AE_APPLIED_SV       (*(volatile int16_t *) 0x8B2C4)
#define WB_APPLIED_KELVIN   (*(volatile int32_t *) 0x8B2D0)
#define AEWB_STATE_PTR      0xAA40
#define AEWB_STATE_WB_GAINS 0x84

/* CR3 encoder (src/crx_enc.c): AlgsMgr state pointer and its
 * per-encoder-id "set up" flags */
#define ALGS_STATE_PTR      0x5BBC
#define ALGS_ENC_INITED     0x138
/* decoder slots: AlgsMgrState.c DecInit (0xE02AEEF2) picks the slot by
 * matching the width with these, in one table for lossless (CMP1 levels 0:
 * +0x5C 7128 -> 4, +0x60 1624 -> 5, +0x70 7136 -> 9, ...; an unknown width
 * asserts) and one for C-RAW (levels > 0: +0x64 7128 -> 6, +0x68 1624 -> 7,
 * +0x8C -> 0x10, +0x9C -> 0x14; an unknown width gets slot 0x17, which
 * froze the camera).  Slots 5 and 7 are Canon's single-tile reduced raws
 * (1624); a slot's set-up flag is ALGS_ENC_INITED + slot * 4 like the
 * encoders' */
#define ALGS_DEC_WIDTH_SLOT5 0x60
#define ALGS_DEC_SLOT5       5
#define ALGS_DEC_WIDTH_SLOT7 0x68
#define ALGS_DEC_SLOT7       7
/* CRaw driver work arena (globals pointer 0x143B8 -> struct): +0x8 base,
 * +0xC size (0x18000), +0x14 free pointer.  Every decoder slot set-up
 * (SetDecBuf 0xE02FF84C) takes its tables from it; when it runs out the
 * set-up fails and Canon asserts (AlgsMgrExec.c:2130) */
#define CRAW_GLOBALS_PTR     0x143B8
/* the 'bl assert' after "DecErr %08x" in the decode result handler
 * (0xE02545BA, AlgsMgrState.c:1245); the handler carries on after it */
#define ALGS_DECERR_ASSERT_BL     0xE02545EC
#define ALGS_DECERR_ASSERT_BL_OLD 0xF852F3D2

#ifndef RAW_LV_SOURCE_MEM1
/* Channel 1's LV raw has no optical black area to measure from.  Lens cap,
 * 1080p: 12-bit values 117..139, mean 127.5, stdev ~2 => 512 / 8.0 at
 * 14 bit.  raw.c scales these to the stream's actual bit depth.  (Mem1
 * has optical black; raw.c measures it.) */
#define RAW_LV_BLACK_LEVEL_14BIT        512
#define RAW_LV_BLACK_NOISE_14BIT_X100   800
#endif

#define FASTEST_SHUTTER_SPEED_RAW 160
#define MAX_AE_EV 2
#define FLASH_MAX_EV 3
#define FLASH_MIN_EV -10
#define COLOR_FG_NONLV 80
#define AF_BTN_HALFSHUTTER 0
#define AF_BTN_STAR 2
// another block copied from 50D
#define GUIMODE_WB 5
#define GUIMODE_FOCUS_MODE 9
#define GUIMODE_DRIVE_MODE 8
#define GUIMODE_PICTURE_STYLE 4
#define GUIMODE_Q_UNAVI 0x18
#define GUIMODE_FLASH_AE 0x22
#define GUIMODE_PICQ 6

// all these MVR ones are junk, don't try and record video and they probably don't get used?
#define MVR_190_STRUCT (*(void**)0x1ed8) // look in MVR_Initialize for AllocateMemory call;
                                         // decompile it and see where ret_AllocateMemory is stored.
#define div_maybe(a,b) ((a)/(b))
// see mvrGetBufferUsage, which is not really safe to call => err70
// macros copied from arm-console
#define MVR_BUFFER_USAGE 70 /* obviously wrong, don't try and record video
       // div_maybe(-100*MEM(236 + MVR_190_STRUCT) - \
       // 100*MEM(244 + MVR_190_STRUCT) - 100*MEM(384 + MVR_190_STRUCT) - \
       // 100*MEM(392 + MVR_190_STRUCT) + 100*MEM(240 + MVR_190_STRUCT) + \
       // 100*MEM(248 + MVR_190_STRUCT), \
       // - MEM(236 + MVR_190_STRUCT) - MEM(244 + MVR_190_STRUCT) + \
       // MEM(240 + MVR_190_STRUCT) +  MEM(248 + MVR_190_STRUCT)) */
#define MVR_FRAME_NUMBER (*(int*)(220 + MVR_190_STRUCT))
//#define MVR_LAST_FRAME_SIZE (*(int*)(512 + MVR_752_STRUCT))
#define MVR_BYTES_WRITTEN MEM((212 + MVR_190_STRUCT))
    

/* ISO of the current LiveView frame, in ML's raw ISO units (1/8 EV,
 * ISO 100 = 0x48).  LiveView::SensorDrive's state (0x47CF0, SetTgParam
 * 0xE06D5D98, "SetCtrlParam(%d, ...)") +0x3C holds the frame's Sv in
 * 1/768 EV: Canon's AE prints Tv/Av/Sv in the same units ("AEC change",
 * e.g. 0x10E0 = Tv 1/50, 0xD20 = Av f/4.5, 0x1C20 = Sv 9.38). Raw =
 * Sv * 8 + 32.  Checked with exposure compensation in M + auto ISO:
 * 0/-1/+1 EV -> 7200/6448/7920 (ISO 2075/1037/3805). */
#define FRAME_ISO ((uint8_t)((*(volatile int16_t *)0x47D2C + 48) / 96 + 32))

/* PROP_IMAGE_TEMP (PROP_EFIC_TEMP here): image sensor temperature in deg C
 * + 128, whole degrees.  Canon's temperature service caches its readings
 * at 0x2E3F0 (12-byte entries, u16 at +4 in 1/16 deg C): 0 image sensor
 * (I2C device 0x4A), 1 a constant 23 C (no shutter sensor), 2 / 3 devices
 * 0x49 / 0x48, 4 the battery.  26 C read 0x9A. */
#define EFIC_CELSIUS ((int)efic_temp - 128)

