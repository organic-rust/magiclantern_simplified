# 5D2.212: restore the MAIN FIRMWARE boot flag

**Status: untested on hardware.** Built and disassembly-checked only. Test in
qemu-eos (below) before putting it on a camera.

## What it fixes

A 5D Mark II whose FIRMWARE boot flag (`0xF8000000`) is left disabled boots
straight into *Firmware update program — Update file cannot be found*. This
happens e.g. when a firmware update fails at the last step ("update file error"
at 99%). If the BOOTDISK flag is still enabled (it is on any camera that had
Magic Lantern installed), the bootloader will run `AUTOEXEC.BIN` from a
bootable card, and this program re-enables the FIRMWARE flag.

The Rescue / ROM dumper shows the state as `FIR=-1 BOOT=-1`.

## How it works

The 5D2 bootloader (`Boot loader Ver 0.30`, `<K218 Board>`) copies its
FROMUTILITY half from ROM `0xFFFF2B18` to RAM `0x100000`
(RAM = ROM − `0xFFFF0000` + `0xFD4E8`) and keeps a RAM shadow of the 16-byte
flag block at `0x10D770`:

| +0 FIRMWARE (0 = on) | +4 BOOTDISK (-1 = on) | +8 RAM_EXE (0 = on) | +C UPDATE (0 = on) |
|---|---|---|---|

| Stub | ROM | RAM |
|---|---|---|
| `flags_read()` — flash → shadow | FFFF4C78 | 0x102160 |
| `flags_write()` — shadow → flash, verified | FFFF4C60 | 0x102148 |
| `flash_write(addr, src, len, verify)` | FFFF45C0 | 0x101AA8 |
| `set_bootdisk(v)` / `set_ram_exe(v)` / `set_update(v)` | FFFF4CA0 / 4CAC / 4C88 | 0x102188 / 0x102194 / 0x102170 |

FROMUTILITY has no FIRMWARE setter, so the program does
`flags_read(); shadow[0] = 0; flags_write();`.

`flags_write()` erases one 8 KB parameter sector at `0xF8000000` (ROM1 is
8 × 8 KB + 126 × 64 KB; the main firmware's flash driver at `FF86FA88`
confirms the same map), programs 16 bytes and verifies. Nothing else lives in
that sector. An interrupted write leaves the sector erased, which reads as
`FIR=-1 BOOT=-1` — the starting state — so the card route stays open.

The main firmware itself never checks the FIRMWARE flag at boot; only the
bootloader does. Its only flag code is the six `Enable/Disable{MainFirm,
BootDisk,Firmware}` eventprocs at `FF86ABCC–FF86AC10`.

## Guards

Nothing is written unless all of these hold:

* bootloader string at `0xFFFF0760` starts with `<K218 Board>`
* `MEM(0x102160) == 0xE51F000C`, `MEM(0x102148) == 0xE59F100C`,
  `MEM(0x10215C) == 0x0010D770`
* after `flags_read()`, the shadow matches the four words at `0xF8000000`

## LED codes (card LED)

| Pattern | Meaning |
|---|---|
| solid on | success — FIRMWARE flag is now 0 |
| slow blink | flag already enabled, nothing written |
| fast blink | a guard failed, nothing written |
| long-short | `flags_write()` error or readback mismatch |

Remove the battery afterwards. Without a card the camera should then start the
main firmware.

## Build

```
make            # needs arm-none-eabi-gcc
```

Copy `AUTOEXEC.BIN` to the root of a card made bootable (EOS_DEVELOP +
BOOTDISK, e.g. with EOScard / MacBoot / make_bootable.sh). No `.FIR` file on
the card.

## Test in qemu-eos first

```
export QEMU_EOS_WORKDIR=/path/to/work    # contains 5D2/ROM0.BIN, 5D2/ROM1.BIN
mcopy -o -i cf.img@@$((99*512)) AUTOEXEC.BIN ::/AUTOEXEC.BIN   # bootable image
qemu-system-arm -drive if=sd,format=raw,file=sd.img \
                -drive if=ide,format=raw,file=cf.img \
                -display none -M 5D2 -d romw
```

`-d romw` should show writes only inside `0xF8000000–0xF8001FFF`.

## Porting

`find_bl_flag_stubs.py ROM1.BIN --log RESCUE.LOG` finds the same stubs, guard
words and RAM relocation in other bootloaders of this family (anchored on the
FROMUTILITY "Is flg written" prompt). DIGIC 5+ bootloaders have a generic
`set_bootflag(flag, value)` instead, which the old `recovery` branch handles.

## Note for this particular camera

On the camera this was written for, the user-settings RING store (property
group `0x02` in ROM0 at `0xF0310000`) is also damaged: subgroup 6 is truncated
and subgroups 7–9 are missing. Fixing the flag gets past the update prompt;
it does not repair those settings.
