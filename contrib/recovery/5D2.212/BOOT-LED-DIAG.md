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
| 1–3 very fast flickers between heartbeats | about to install log hook 1, 2 or 3 (only during the first few heartbeats) |
| 2 long pulses instead of 1 at the start of each heartbeat | a log hook failed to install, or the log buffers couldn't be allocated |
| 5 fast blinks | a log file was written to the card |

Brief irregular flickers from card access may appear in between once the card
is mounted (milestone 4 and later).

No heartbeat after the 2 fast blinks means startup is stuck inside
`init_task` itself, or the DryOS scheduler is no longer running.

## Log files

After the first heartbeat the same build allocates its buffers from Canon's
heap and installs three hooks, one per heartbeat (with 1, 2, 3 fast flickers
just before each):

1. `DryosDebugMsg` (class, level, format and up to 5 arguments)
2. `mpu_send(buf, size)` (`FF99F518`), first 28 bytes sent to the MPU
3. the MPU receive analyser (`FF99BA38`), first 8 words of the decoded message

plus the first context switch of every task seen after that, and each milestone.

Hooking before Canon's `init_task` (the first version of this) stopped the
camera with the LED stuck on, so nothing is patched that early any more. If
the camera stops again, the last flicker count tells which hook did it.

Nothing is formatted inside the hooks; records are stored raw (first up to
2000 + last 256) and formatted by the heartbeat task, which writes:

* `A:/BOOTDIAG.LOG` about 30 s after startup
* `A:/BOOTDIA2.LOG` about 120 s after startup

Leave the camera running for at least two minutes (watch for the 5 fast
blinks), then remove the battery and copy both files off the card.
The hooks re-apply themselves every heartbeat in case Canon's cache flushes
remove them. Build with `CFLAGS_PLATFORM += -DCONFIG_NO_BOOT_DIAG_LOG` to get
the LED codes only.

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
