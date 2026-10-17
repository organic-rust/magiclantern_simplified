# Boot progress on the card LED (`BOOT_LED_DIAG`)

For a 5D2 that hangs with a black screen and never mounts the card, so
neither the display nor a log file can show how far Canon's startup gets.

**Status: untested.** Not built in the environment that wrote it (no ARM
toolchain there); syntax-checked only.

## Build

```
cd platform/5D2.212
make clean && make BOOT_LED_DIAG=y
```

Copy `autoexec.bin` (and the usual `ML/` folder) to a bootable card as for a
normal ML install. Without `BOOT_LED_DIAG=y` nothing changes.

## What the card LED shows

| Pattern | Meaning |
|---|---|
| 3 slow blinks | ML passed its firmware check and is about to enter Canon's startup code |
| 2 fast blinks | DryOS is running; Canon's `init_task` is about to be called |
| repeating: 1 long pulse, then **N** short pulses, then a pause | heartbeat: the scheduler is alive and the furthest milestone task reached is **N** |

Brief irregular flickers from card access may appear in between once the card
is mounted (milestone 4 and later).

No heartbeat after the 2 fast blinks means startup is stuck inside
`init_task` itself, or the DryOS scheduler is no longer running.

## Milestones (N)

First-run order of tasks when this camera's own ROM dumps boot to the GUI in
qemu-eos (5D2 2.1.2):

| N | Task | Notes |
|---|---|---|
| 0 | — | no milestone task has run yet |
| 1 | Startup | |
| 2 | PropMgr | property manager; loads the settings from ROM0 |
| 3 | EventMgr | |
| 4 | FileMgr | card is mounted after this |
| 5 | MainCtrl | |
| 6 | CSMgrTask | |
| 7 | LiveViewMgr | |
| 8 | PtpDps | USB/PTP manager (site of the NULL read seen in qemu) |
| 9 | GuiMainTask | GUI task; display comes up shortly after |

A task being *scheduled* means it reached its first context switch, not that
its own initialisation completed. If the camera sticks at N, the stall is
after that task started and before the next milestone task ran.

## Implementation

* `src/boot-diag.c`, `src/boot-diag.h`
* hooks (all under `#ifdef CONFIG_BOOT_LED_DIAG`):
  * `src/boot-d45.c` `copy_and_restart()` — entry blinks, busy-wait
  * `src/boot.h` `my_init_task()` — blinks before `init_task`; heartbeat task
    started right after `init_task` returns (before `boot_post_init_task`,
    which waits for Canon's GUI)
  * `src/init.c` `my_task_dispatch_hook()` — milestone tracking by task name
* the card LED register is written directly (`CARD_LED_ADDRESS`); the 5D2's
  `CONFIG_BLUE_LED` path uses an eventproc that isn't usable this early
* heartbeat task priority 0x08 so a busy Canon task can't starve it
