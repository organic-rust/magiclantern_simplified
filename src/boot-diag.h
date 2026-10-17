#ifndef _boot_diag_h_
#define _boot_diag_h_

/*
 * Boot progress diagnostics on the card LED (CONFIG_BOOT_LED_DIAG).
 * For cameras that hang with a black screen before Canon mounts the card,
 * so neither the display nor the card can be used to report progress.
 * See contrib/recovery/5D2.212/BOOT-LED-DIAG.md for the blink codes.
 */

#ifdef CONFIG_BOOT_LED_DIAG
void boot_diag_entry(void);              /* copy_and_restart, before Canon startup (no DryOS) */
void boot_diag_pre_init_task(void);      /* my_init_task, before Canon's init_task */
void boot_diag_post_init_task(void);     /* after Canon's init_task returns: starts heartbeat */
void boot_diag_task_seen(const char *);  /* from the task dispatch hook */
#endif

#endif
