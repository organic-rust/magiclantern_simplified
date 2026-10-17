/*
 * Boot progress diagnostics on the card LED (CONFIG_BOOT_LED_DIAG).
 *
 * Reports how far Canon's startup gets when the camera hangs with a black
 * screen before the card is mounted (no display, no file I/O available).
 *
 *   entry       3 slow blinks   ML is about to jump into Canon's startup code
 *   pre-init    2 fast blinks   DryOS is up, Canon's init_task is about to run
 *   heartbeat   long pulse + N short pulses, repeated every few seconds,
 *               N = furthest milestone task that has been scheduled (0..9)
 *
 * No heartbeat after the 2 fast blinks: stuck inside init_task itself (or
 * the scheduler is dead). Heartbeat with a fixed N: the scheduler is alive and
 * startup stalled after milestone N.
 *
 * Milestones are the first-run order of tasks observed when booting a 5D2
 * 2.1.2 in qemu-eos; other models may need a different list.
 */

#include "dryos.h"
#include "boot-diag.h"

#ifdef CONFIG_BOOT_LED_DIAG

#ifndef CARD_LED_ADDRESS
#error CONFIG_BOOT_LED_DIAG needs CARD_LED_ADDRESS, LEDON and LEDOFF
#endif

#define LED_ON()   (*(volatile uint32_t *)(CARD_LED_ADDRESS) = (LEDON))
#define LED_OFF()  (*(volatile uint32_t *)(CARD_LED_ADDRESS) = (LEDOFF))

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

void boot_diag_entry(void)
{
    diag_blink_spin(3, 15, 15);
    diag_spin(30);
}

void boot_diag_pre_init_task(void)
{
    diag_blink_spin(2, 3, 5);
    diag_spin(30);
}

static int name_eq(const char *a, const char *b)
{
    if (!a || !b) return 0;
    while (*a && *a == *b) { a++; b++; }
    return *a == *b;
}

/* called from the task dispatch hook on every context switch: keep it cheap */
void boot_diag_task_seen(const char *name)
{
    int s = diag_stage;
    if (s >= NUM_MILESTONES || !name)
        return;

    for (int i = s; i < NUM_MILESTONES; i++)
    {
        if (name[0] == milestones[i][0] && name_eq(name, milestones[i]))
        {
            diag_stage = i + 1;
            return;
        }
    }
}

static void diag_heartbeat_task(void *unused)
{
    while (1)
    {
        int n = diag_stage;
        LED_ON();  msleep(800);
        LED_OFF(); msleep(400);
        for (int i = 0; i < n; i++)
        {
            LED_ON();  msleep(150);
            LED_OFF(); msleep(300);
        }
        msleep(2000);
    }
}

void boot_diag_post_init_task(void)
{
    /* high priority so a busy Canon task can't starve the heartbeat */
    task_create("diag_hb", 0x08, 0x400, diag_heartbeat_task, 0);
}

#endif /* CONFIG_BOOT_LED_DIAG */
