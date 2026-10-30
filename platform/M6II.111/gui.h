#ifndef _cameraspecific_gui_h_
#define _cameraspecific_gui_h_

/* As in M6II 1.1.1 */


#define BGMT_WHEEL_UP                0x00
#define BGMT_WHEEL_DOWN              0x01
#define BGMT_WHEEL_LEFT              0x02
#define BGMT_WHEEL_RIGHT             0x03

/* Rotation events, from gui_massive_event_loop (0xe03db75a), which turns
 * each code into a Canon dialog event (in brackets).  0/1 and 2/3 are both
 * "SUB_DIAL" to Canon dialogs (DlgDialFuncMain); 8/9 is the main dial
 * (ROTATE_ELECTRONIC_MAIN_DIAL_LEFT/RIGHT, keyboard dialogs).  The lock
 * mask passed alongside is 1/2/4/8 for 0-1/2-3/4-5/8-9 and none for 6/7.
 * Which physical dial is 0-7 still needs Debug > Show GUI events. */
#define BGMT_DIAL3_LEFT              0x04 // [0x820]
#define BGMT_DIAL3_RIGHT             0x05 // [0x81f]
#define BGMT_DIAL4_LEFT              0x06 // [0x822]
#define BGMT_DIAL4_RIGHT             0x07 // [0x821]
#define BGMT_MAIN_DIAL_LEFT          0x08 // [0x823]
#define BGMT_MAIN_DIAL_RIGHT         0x09 // [0x824]

#define BGMT_PRESS_SET               0x0A
#define BGMT_UNPRESS_SET             0x0B

#define BGMT_MENU                    0x0C // unpress 0xD
#define BGMT_INFO                    0x0E // unpress 0xF

#define BGMT_PLAY                    0x12 // I think?

/* Buttons passed to Canon dialogs as 0x10000000 | code (the default case
 * of gui_massive_event_loop); codes from dialog handlers that log the name,
 * e.g. IDLEHandler 0xe07d41ba, DlgPlayMain, DlgCreativeAssistMain.
 * UNPRESS codes marked "guess" are PRESS + 1, not seen in a handler. */
#define BGMT_PRESS_ERASE             0x14 // PRESS_ERASE_BUTTON (DlgPlayMain)
#define BGMT_UNPRESS_ERASE           0x15 // guess
#define BGMT_PRESS_FUNC              0x1F // PRESS_FUNC_BUTTON (DlgPlayMain)
#define BGMT_PRESS_MFN               0x21 // PRESS_MULTI_FUNCTION_BUTTON
#define BGMT_UNPRESS_MFN             0x22 // UNPRESS_MULTI_FUNCTION_BUTTON
#define BGMT_PRESS_MFN2              0x23 // PRESS_MULTI_FUNCTION2_BUTTON
#define BGMT_UNPRESS_MFN2            0x24 // UNPRESS_MULTI_FUNCTION2_BUTTON
#define BGMT_PRESS_AF_ON             0x45 // one handler for AEL / AF_ON / LENS_AFSTOP;
#define BGMT_UNPRESS_AF_ON           0x46 // guess        0x47 is AEL (DlgCreativeAssistMain),
#define BGMT_PRESS_AE_LOCK           0x47 //              so 0x45 is AF-ON or the lens AF-stop
#define BGMT_UNPRESS_AE_LOCK         0x48 // guess
#define BGMT_PRESS_AF_POINT          0x49 // PRESS_FPSEL_BUTTON (IDLEHandler)

/* Movie record: IDLEHandler and DlgPlayMain send 0x2C and 0x2E to one case,
 * "PRESS_LV_MOVIE_START_BUTTON/PRESS_LV_START_BUTTON"; which is the record
 * button isn't known yet, so modules (mlv_lite) get both as MODULE_KEY_REC.
 * BGMT_LV stays a placeholder: core code fakes LV presses with it. */
#define BGMT_REC                     0x2C
#define BGMT_REC2                    0x2E
#define BGMT_UNPRESS_AF_POINT        0x4A // guess

/* Long press of Erase opens the ML menu and M-Fn goes back to Canon.
 * Enable once Debug > Show GUI events confirms 0x14 / 0x15; until then
 * M-Fn opens the ML menu, so a wrong code can't lock you out of ML. */
//#define CONFIG_LONG_PRESS_ERASE_MENU

#ifdef CONFIG_LONG_PRESS_ERASE_MENU
#define BGMT_TRASH                   BGMT_PRESS_ERASE
#else
#define BGMT_TRASH                   BGMT_PRESS_MFN
#endif

#define BGMT_PRESS_RIGHT             0x3B
#define BGMT_UNPRESS_RIGHT           0x3C
#define BGMT_PRESS_LEFT              0x3D
#define BGMT_UNPRESS_LEFT            0x3E
#define BGMT_PRESS_UP                0x3F
#define BGMT_UNPRESS_UP              0x40
#define BGMT_PRESS_DOWN              0x41
#define BGMT_UNPRESS_DOWN            0x42

#define BGMT_PRESS_HALFSHUTTER       0x9C // unpress 0x9D

// needed for correct shutdown from powersave modes
#define GMT_GUICMD_LOCK_OFF          0xAC // GUICMD_LOCK_OFF
#define GMT_GUICMD_START_AS_CHECK    0xB3 // GUICMD_START_AS_CHECK
#define GMT_OLC_INFO_CHANGED         0xB4 // copyOlcDataToStorage uiCommand(%d)

/* Codes below are WRONG: DNE in R */
#define BGMT_LV                      -0xF3 // MILC, always LV.

#define BGMT_PRESS_ZOOM_IN           -0xF4
#define GMT_GUICMD_OPEN_SLOT_COVER   -0xF5 // N/A on this model
//#define BGMT_UNPRESS_ZOOM_IN         -0xF5
//#define BGMT_PRESS_ZOOM_OUT          -0xF6
//#define BGMT_UNPRESS_ZOOM_OUT        -0xF7
#endif
