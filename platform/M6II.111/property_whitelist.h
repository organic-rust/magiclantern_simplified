#ifndef _property_whitelist_h_
#define _property_whitelist_h_

#include "property.h"

// By default, we allow "reads" (property_handler, ML hooking) for any property.
//
// Initial support for Digic 6, 7, 8 cams found some prop handlers triggered
// bad behaviour (crashes, hangs).  These are blacklisted for both read and write
// until investigated and fixed.
//
// For Digic 4 and 5 cams, "writes" (prop_request_change()) are controlled
// by a global flag, CONFIG_PROP_REQUEST_CHANGE.  This is all or nothing.
//
// For D678, we disallow all "writes" via the same flag, so,
// CONFIG_PROP_REQUEST_CHANGE should be #undef for new ports in features.h.
// Writing to props can brick cams if mistakes are made.
//
// Additionally, even when defined, we now have a per property check (on D678),
// so if prop_write_allow[] is empty, no writes will be attempted even
// with CONFIG_PROP_REQUEST_CHANGE.  This allows devs to enable properties
// one at a time after checking correctness via reversing, tests, etc.

// DO NOT put the same property in both lists;
// that would allow writes but not reads, which is untested
// (and unnecessary as far as I know).

// deny reads / do not register property handlers for these
const uint32_t prop_handler_deny[] =
{
    /* PROP_ISO and PROP_LV_AFFRAME were denied after crashes on Digic 8.
     * Likely causes, now fixed: ISO is in 1/768 EV units on D8
     * (CONFIG_PROP_EXPO_1_768_EV), and AFFRAME is 1568 bytes, which
     * overran shoot.c's 512-byte afframe[] on every update. */
    PROP_MVR_REC_START, // probably related to MVR stubs being all wrong
};

// allow writes / allow prop_request_change() for these:
const uint32_t prop_write_allow[] =
{
    PROP_ICU_AUTO_POWEROFF,
    PROP_PICTURE_STYLE,
    PROP_PICSTYLE_SETTINGS_STANDARD,
    PROP_PICSTYLE_SETTINGS_PORTRAIT,
    PROP_PICSTYLE_SETTINGS_LANDSCAPE,
    PROP_PICSTYLE_SETTINGS_NEUTRAL,
    PROP_PICSTYLE_SETTINGS_FAITHFUL,
    PROP_PICSTYLE_SETTINGS_MONOCHROME,
    PROP_PICSTYLE_SETTINGS_USERDEF1,
    PROP_PICSTYLE_SETTINGS_USERDEF2,
    PROP_PICSTYLE_SETTINGS_USERDEF3,
    PROP_PICSTYLE_SETTINGS_AUTO,
    PROP_PICSTYLE_SETTINGS_FINEDETAIL,
    /* RAW burst and its pre-shooting (Canon's "Mechaless"): 4-byte 0 / 1,
     * set by Canon's GUI the same way (0xE0574358 / 0xE0574380 ->
     * _prop_request_change) */
    PROP_MECHALESS,
    PROP_MECHALESS_PRE_REC,
    /* Canon's movie size and frame rate (PROP_MOVIE_PARAM, 40 bytes: [1]
     * size 0 = 1080p, 5 = 4K, [2] rate in 1/100 fps, [3] rate / 2):
     * d8_modes' Sensor mode writes combinations Canon's menu makes, as
     * Canon's lv_set_movie_resol eventproc (0xE04FDE52) does */
    PROP_VIDEO_MODE
};

// anything not listed above will allow reads but not writes

#endif
