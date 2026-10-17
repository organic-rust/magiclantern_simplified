#!/usr/bin/env python3
"""
Locate Canon DIGIC 4-era bootloader (FROMUTILITY) boot-flag stubs in a ROM1 dump.

Usage: find_bl_flag_stubs.py ROM1.BIN [--rom1-size 0x800000] [--log RESCUE.LOG]

Method (string-anchored, so it should port across bodies with the same bootloader family):
  1. Bootloader = last 64 KB of the ROM1 chip, mapped at 0xFFFF0000.
  2. Find the FROMUTILITY prompt "Is flg written(Y=ON(0xFFFFFFFF)/N=OFF(0x00000000))? :"
     (positive-logic flag, i.e. BOOTDISK) and the ADR that references it.
  3. In the enclosing menu handler, the BL calls are, in order:
        flags_read() , <getline>, set_bootdisk(v), flags_write()
  4. flags_read is "ldr r0,=shadow; mov r2,#0x10; mov r1,#0xF8000000; b flash_read16"
     -> gives the RAM shadow buffer address.
  5. flags_write is "ldr r1,=shadow; mov r3,#1; mov r2,#0x10; mov r0,#0xF8000000; b flash_write"
  6. RAM relocation of the FROMUTILITY copy is derived from the instructions the ML
     portable dumper patched (from RESCUE.LOG) if given, else reported as unknown.
Prints ROM and RAM addresses plus the expected instruction words, to use as guards.
"""
import sys, struct, re, argparse

def u32(b, o): return struct.unpack_from('<I', b, o)[0]

def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('rom1')
    ap.add_argument('--rom1-size', type=lambda x: int(x, 0), default=0x800000)
    ap.add_argument('--log')
    a = ap.parse_args()
    rom = open(a.rom1, 'rb').read()[:a.rom1_size]
    BASE = 0xFFFF0000
    bl = rom[-0x10000:]
    W = [u32(bl, i) for i in range(0, len(bl), 4)]
    addr = lambda k: BASE + 4 * k
    def rot(i12):
        r = (i12 >> 8) * 2; v = i12 & 0xff
        return ((v >> r) | (v << (32 - r))) & 0xffffffff if r else v
    def adr_target(k):
        w = W[k]; pc = addr(k) + 8
        if (w & 0x0FFF0000) == 0x028F0000: return (pc + rot(w & 0xfff)) & 0xffffffff
        if (w & 0x0FFF0000) == 0x024F0000: return (pc - rot(w & 0xfff)) & 0xffffffff
    def bl_target(k):
        w = W[k]
        if (w & 0x0F000000) not in (0x0B000000, 0x0A000000) or (w >> 28) == 0xF: return None
        off = w & 0xFFFFFF
        if off & 0x800000: off -= 1 << 24
        return (addr(k) + 8 + (off << 2)) & 0xffffffff, bool(w & 0x01000000)
    def ldr_lit(k):
        w = W[k]
        if (w & 0x0F7F0000) != 0x051F0000: return None
        imm = w & 0xfff; t = addr(k) + 8 + (imm if (w >> 23) & 1 else -imm)
        return u32(bl, t - BASE)
    def W_at(x): return W[(x - BASE) // 4]

    s = bl.find(b"Is flg written(Y=ON(0xFFFFFFFF)/N=OFF(0x00000000))? :")
    if s < 0: sys.exit("prompt string not found - different bootloader family")
    sa = BASE + s
    refs = [k for k in range(len(W)) if adr_target(k) == sa]
    if not refs: sys.exit("no ADR reference to prompt")
    k = refs[0]
    while (W[k] & 0xFFFF0000) != 0xE92D0000 and not (W[k] == 0xE52DE004): k -= 1
    calls = []
    j = k
    while len(calls) < 6 and j < k + 64:
        t = bl_target(j)
        if t and t[1]: calls.append(t[0])
        if (W[j] & 0x0FFF0000) in (0x08BD0000,) or W[j] == 0xE49DF004: break
        j += 1
    flags_read, setter, flags_write = calls[0], calls[-2], calls[-1]
    shadow = ldr_lit((flags_read - BASE) // 4)
    rd_tail = bl_target((flags_read - BASE) // 4 + 3)
    wr_tail = bl_target((flags_write - BASE) // 4 + 4)
    # per-flag setters live next to each other: str r0,[r1,#off]
    setters = {}
    for kk in range((flags_read - BASE) // 4, (flags_read - BASE) // 4 + 40):
        if ldr_lit(kk) == shadow and (W[kk + 1] & 0xFFFFF000) == 0xE5810000:
            setters[W[kk + 1] & 0xfff] = addr(kk)
    delta = None
    if a.log:
        pats = re.findall(r'Patching ([0-9a-fA-F]+) from ([0-9a-fA-F]{8})', open(a.log).read())
        for d in range(0, 0x200000, 4):
            ok = True
            for ra, w in pats:
                o = int(ra, 16) - d
                if not (0 <= o < 0x10000 - 3 and u32(bl, o) == int(w, 16)): ok = False; break
            if ok and pats: delta = d; break
    ram = (lambda x: x - BASE + delta) if delta is not None else (lambda x: None)
    fmt = lambda x: f"{x:#08x}" if x is not None else "?"
    print(f"bootloader prompt @ {sa:#x}, menu handler @ {addr(k):#x}")
    print(f"shadow flag buffer (RAM): {shadow:#x}  [+0 FIRMWARE(neg) +4 BOOTDISK(pos) +8 RAM_EXE(neg) +C UPDATE(neg)]")
    if delta is not None:
        print(f"FROMUTILITY RAM copy: RAM = ROM - 0xFFFF0000 + {delta:#x}  (RAM 0x100000 == ROM {BASE + 0x100000 - delta:#x})")
    rows = [("flags_read()", flags_read), ("flags_write()", flags_write),
            ("flash_read16(dst,src,n16)", rd_tail[0] if rd_tail else None),
            ("flash_write(addr,src,len,verify)", wr_tail[0] if wr_tail else None)]
    names = {0: 'set_firmware', 4: 'set_bootdisk', 8: 'set_ram_exe', 0xc: 'set_update'}
    for off, x in sorted(setters.items()): rows.append((names.get(off, f'set_+{off:#x}') + "(v)", x))
    if 0 not in setters: rows.append(("set_firmware(v)", None))
    for n, x in rows:
        if x is None: print(f"  {n:34} NOT PRESENT (write shadow[0] directly)"); continue
        print(f"  {n:34} ROM {x:#010x}  RAM {fmt(ram(x))}  guard word {W_at(x):08x}")

if __name__ == '__main__':
    main()
