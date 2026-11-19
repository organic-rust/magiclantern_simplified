#!/usr/bin/env python3
"""
Build a recovery variant of Canon's 5D Mark II 2.1.2 update (5D200212.FIR)
for ONE specific camera:

  * piece 3 (tag 0x100, ROM0 F0330000, FIX/Tune area) is replaced with that
    camera's own current contents, so the updater's flash writer sees
    "File == FROM" and does not erase or rewrite the area (the original piece
    is a generic template; Canon's final script normally restores the per-unit
    values afterwards with UPD_AdjustWB);
  * piece 7 (tag 0x102, the final event-procedure script) is replaced by a
    comment-only script of the same length, so nothing in it runs
    (originally: UPD_AdjustWB, 20 x UPD_AddPropertyDataIfNotExist, then erase
    and rewrite of the RING, WFT and Tune property areas);
  * everything else (MPU EEPROM tables, MPU program, ROM0 resources, FPGA,
    main firmware, updater code, header) is byte-identical to Canon's file.

The payload is re-encrypted with Canon's own content key and IV (AES-128-CTR),
then the payload sum, the firmware-section HMAC tag (@0x88) and the file
checksum (@0x20) are recomputed exactly as the updater checks them.

usage: fir5d2_build_recovery.py ROM1.BIN 5D200212.FIR FIX_F0330000.bin OUT.FIR
"""
import sys, struct, os, hashlib
sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from fir5d2_decrypt import aes_ctr
from fir5d2_check import decrypt_payload, compute_tags, file_checksum, u32

EXPECTED_ORIG_MD5 = 'a14a0e9ec981831d8091b8bdc5560ce7'      # Canon 5D200212.FIR
FIX_ADDR, FIX_SIZE = 0xF0330000, 0x2F5AC
SCRIPT_SIZE = 0xE70

def make_script(size):
    head = (b"; 5D Mark II recovery build: Canon's final script removed.\r\n"
            b"; Removed: UPD_AdjustWB, 20x UPD_AddPropertyDataIfNotExist,\r\n"
            b"; RING/WFT/Tune erase and rewrite. Piece 3 carries this camera's\r\n"
            b"; own F0330000 contents, so the updater skips that area.\r\n")
    body = bytearray(head)
    while size - len(body) >= 64 + 3:
        body += b';' * 62 + b'\r\n'
    body += b';' * (size - len(body) - 2) + b'\r\n'
    assert len(body) == size and body.endswith(b'\r\n')
    for line in bytes(body).split(b'\r\n')[:-1]:
        assert line.startswith(b';')                    # every line a comment
    return bytes(body)

def main():
    rom1 = open(sys.argv[1], 'rb').read()
    orig = open(sys.argv[2], 'rb').read()
    fix = open(sys.argv[3], 'rb').read()
    out_path = sys.argv[4]

    assert hashlib.md5(orig).hexdigest() == EXPECTED_ORIG_MD5, 'not Canon\'s 5D200212.FIR'
    assert len(fix) == FIX_SIZE, 'FIX snapshot must be %#x bytes' % FIX_SIZE

    key_ok, plain_full, plain_len, (h, ck) = decrypt_payload(rom1, orig)
    assert key_ok
    enc_len = u32(orig, h)
    # decrypt again over the full (padded) encrypted length so padding round-trips
    iv2 = orig[h + 64:h + 80]
    old_cipher = orig[h + 112:h + 112 + enc_len]
    full = bytearray(aes_ctr(ck[:16], iv2, old_cipher))

    cnt, eoff, esz, total = struct.unpack_from('<4I', full, 0x10)
    pieces = [struct.unpack_from('<6I', full, eoff + i * esz) for i in range(cnt)]
    assert cnt == 7
    no3, tag3, off3, size3, addr3, _ = pieces[2]
    no7, tag7, off7, size7, addr7, _ = pieces[6]
    assert (tag3, addr3, size3) == (0x100, FIX_ADDR, FIX_SIZE), pieces[2]
    assert (tag7, size7) == (0x102, SCRIPT_SIZE), pieces[6]

    full[off3:off3 + size3] = fix
    full[off7:off7 + size7] = make_script(size7)
    # CheckFirData sum (updater 0x804d2c)
    data_end = eoff + cnt * esz + total
    assert data_end == plain_len
    struct.pack_into('<I', full, 0, (~sum(full[4:data_end])) & 0xFFFFFFFF)

    new_cipher = aes_ctr(ck[:16], iv2, bytes(full))
    fir = bytearray(orig)
    fir[h + 112:h + 112 + enc_len] = new_cipher

    t1, t2, pairs = compute_tags(rom1, bytes(fir))
    assert t1 == orig[0x68:0x88], 'tag1 changed: header/updater1 must be untouched'
    fir[0x88:0xa8] = t2
    struct.pack_into('<I', fir, 0x20, file_checksum(bytes(fir)))

    open(out_path, 'wb').write(fir)
    changed = [i for i in range(len(fir)) if fir[i] != orig[i]]
    print('wrote %s (%d bytes), md5 %s' % (out_path, len(fir), hashlib.md5(fir).hexdigest()))
    print('bytes differing from Canon\'s file: %d, first %#x, last %#x'
          % (len(changed), changed[0], changed[-1]))

if __name__ == '__main__':
    main()
