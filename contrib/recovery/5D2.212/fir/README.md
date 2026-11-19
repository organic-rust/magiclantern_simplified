# 5D2 2.1.x FIR tools

Tools for decrypting, checking and rebuilding Canon 5D Mark II 2.1.1 / 2.1.2
firmware update files. They are re-implemented from the 2.1.2 updater
("updater1", base `0x800120`). They were written to recover a camera whose
updates kept failing in the final script (see below).

**No keys are included.** Both 32-byte keys are read from the camera's own
ROM1 dump. The tools need a `ROM1.BIN` (16 MB, as written by the ML ROM
dumper) and Canon's `.FIR` file.

| Tool | What it does |
|---|---|
| `fir5d2_decrypt.py ROM1.BIN FILE.FIR OUT.BIN` | decrypts the firmware payload |
| `fir5d2_check.py ROM1.BIN FILE.FIR [payload_out.bin]` | re-runs every check the updater applies; exit status 0 only if all pass |
| `fir5d2_build_recovery.py ROM1.BIN 5D200212.FIR FIX.bin OUT.FIR` | builds the recovery variant described below |
| `mpu_srec.py PAYLOAD.BIN OUTDIR` | decodes the MPU pieces (binary Motorola S-records) to plain images |

Needs Python 3 and `cryptography`. updater1 itself decrypts with Arm.Indy's
`fir_tool2.py -x` (XOR layer).

## Format (2.1.x)

- **Keys:** ROM1 at `F87C0000`: `[+4]` → payload key, `[+8]` → HMAC key.
  Each is 32 bytes, obfuscated (2 rounds of word rotations, constant
  add/sub, a 256-bit rotate and word reversal; updater `0x8785C4`).
- **KDF:** `SHA1(x||01) || SHA1(x||02)[:12]` (updater `0x848310`).
- **Payload decryption** (header at FIR `+[0x30]+[hdr]`, updater `0x80A3A0`):
  - KEK = KDF(salt `[hdr+12]` || secret || plain length `[hdr+4]`);
  - the content key (`hdr+32`) is unwrapped with AES-128-CTR (IV `hdr+16`)
    and checked against KDF(content key) at `hdr+80`;
  - the payload is AES-128-CTR (IV `hdr+64`), where only the last 4 counter
    bytes increment (`0x8A212C`). The AES wrapper always uses a 16-byte key.
- **Tags:** the tag at `0x68` covers header + updater1, the tag at `0x88`
  covers the encrypted firmware section. Each is
  `HMAC-SHA1(KDF(seed@0x40 || secret2), MD5(MD5(data)||fir[0:16]) ||
  MD5(MD5(data)||fir[16:32]*2))`, stored as 20 bytes + 12 zero bytes
  (`0x809E3C`, `0x809F58`, `0x848424`, `0x848008`).
- **File checksum @0x20:** `~sum` of all other bytes.
- **CheckFirData (`0x804D2C`):** payload `+8` == 2, payload `+0` ==
  `~sum(payload[4:end])`.
- **Pieces** (2.1.2), written in table order. Progress is weighted by the
  last column and truncated, so pieces 1–5 = 38%:

  | # | tag | target | size | weight |
  |---|---|---|---|---|
  | 1 | 0x200 | MPU EEPROM tables `k218_eep.mot` | 0x5D2 | 700 |
  | 2 | 0x200 | MPU program `k218_mpu.mot` | 0x8B7E8 | 44500 |
  | 3 | 0x100 | ROM0 `F0330000` FIX/Tune (generic template) | 0x2F5AC | 3000 |
  | 4 | 0x100 | ROM0 `F0010000` resources | 0x1C78BC | 15000 |
  | 5 | 0x100 | ROM1 `F8760000` FPGA | 0x4DCB8 | 2000 |
  | 6 | 0x100 | ROM1 `F8010000` main firmware | 0x4E1B40 | 101000 |
  | 7 | 0x102 | final event-procedure script (text) | 0xE70 | 2000 |

  `0x100` pieces are compared with flash and skipped if identical. If they
  differ, the updater erases and rewrites the target in 64 KB chunks.
  Piece 7 runs `UPD_AdjustWB`, 20× `UPD_AddPropertyDataIfNotExist`, then
  erase + rewrite of property groups `0x02` (RING), `0x05` and `0x03`.
  `updWriteFlg(0)` (main firmware re-enabled) and `_REBOOT ICU -> MPU` run
  only after the last piece succeeds.

## Recovery build

`fir5d2_build_recovery.py` makes a 2.1.2 FIR for **one** camera:

- **Piece 3** becomes that camera's current `F0330000` contents, read from
  the camera with `bootdiag_ptp.py mem 0xF0330000 0x2F5AC`. Its FIX/Tune
  area is then skipped instead of being overwritten with the template. The
  original relies on `UPD_AdjustWB` to restore the per-unit values.
- **Piece 7** becomes comment lines only.
- **Everything else** is byte-identical to Canon's file.

The payload is re-encrypted with Canon's content key, and the payload sum,
tag2 and the checksum are recomputed. Check the result with
`fir5d2_check.py` before use.

The piece-3 snapshot must match the camera's flash at the time of the
update. A stale or foreign snapshot would be written over that camera's
calibration.

To run it on a camera whose menus don't work:
1. Disable the FIRMWARE flag with `make disable` in `..` (see `../README.md`).
2. Boot from a plain card holding only the FIR.
3. When it reports completion, turn the camera off with the power switch.

On the camera this was written for, the update completed and the MPU came
back. Its final-script failure turned out to be a wedged RING settings store:
every slot had been used by interrupted saves. `erasering` in the boot-diag
build fixes that; see `../BOOT-LED-DIAG.md`.
