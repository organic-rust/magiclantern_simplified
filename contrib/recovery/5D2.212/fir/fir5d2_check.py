#!/usr/bin/env python3
"""
Independent re-implementation of every integrity check the 5D2 2.1.x updater
(and fir_tool's checksum) applies to a .FIR, so a rebuilt file can be verified
before it goes near a camera.

Checks, with the updater1 (2.1.2, base 0x800120) code they mirror:
  1. file checksum @0x20 = ~(sum of all bytes except 0x20..0x23)
  2. tag1 @0x68: header + updater1  (0x809e3c, "_First" in the .FIR path)
  3. tag2 @0x88: encrypted firmware section (0x809f58)
       digest32 = MD5(MD5(data)||fir[0:16]) || MD5(MD5(data)||fir[16:32]*2)   (0x848424)
       tag      = HMAC-SHA1(KDF(seed@0x40 || deobf(ROM1 key [F87C0000+8])), digest32)
                  stored as 20 bytes + 12 zero bytes                          (0x848008)
  4. content-key hash (payload key unwrap, 0x80a3a0)
  5. CheckFirData on the decrypted payload (0x804d2c):
       version word @+8 == 2, word @+0 == ~sum(payload[4 : 0xc8 + total_data])

usage: fir5d2_check.py ROM1.BIN FILE.FIR [decrypted_payload_out.bin]
exit status 0 only if every check passes.
"""
import sys, struct, hashlib, hmac, os
sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from fir5d2_decrypt import deobf, kdf, aes_ctr

def u32(b, o): return struct.unpack_from('<I', b, o)[0]

def md5_tag_digest(fir, data):
    d = hashlib.md5(data).digest()
    a = fir[0:16]
    b = fir[16:32] * 2
    return hashlib.md5(d + a).digest() + hashlib.md5(d + b).digest()

def hmac_key(rom1, fir):
    ko = 0x7C0000
    off = u32(rom1, ko + 8)
    secret = deobf(rom1[ko + off: ko + off + 32])
    return kdf(struct.pack('<I', u32(fir, 0x40)) + secret)

def compute_tags(rom1, fir):
    key = hmac_key(rom1, fir)
    count = u32(fir, 0x44)
    pairs = [struct.unpack_from('<II', fir, 0x48 + 8 * i) for i in range(count)]
    data1 = b''.join(fir[o:o + l] for o, l in pairs[:-1])
    o, l = pairs[-1]
    data2 = fir[o:o + l]
    t1 = hmac.new(key, md5_tag_digest(fir, data1), hashlib.sha1).digest() + bytes(12)
    t2 = hmac.new(key, md5_tag_digest(fir, data2), hashlib.sha1).digest() + bytes(12)
    return t1, t2, pairs

def file_checksum(fir):
    return (~(sum(fir[0:0x20]) + sum(fir[0x24:]))) & 0xFFFFFFFF

def decrypt_payload(rom1, fir):
    ko = 0x7C0000
    secret = deobf(rom1[ko + u32(rom1, ko + 4): ko + u32(rom1, ko + 4) + 32])
    fw = u32(fir, 0x30)
    h = fw + u32(fir, fw)
    enc_len, plain_len, _, salt = struct.unpack_from('<4I', fir, h)
    kek = kdf(struct.pack('<I', salt) + secret + struct.pack('<I', plain_len))
    ck = aes_ctr(kek[:16], fir[h + 16:h + 32], fir[h + 32:h + 64])
    key_ok = kdf(ck) == fir[h + 80:h + 112]
    plain = aes_ctr(ck[:16], fir[h + 64:h + 80], fir[h + 112:h + 112 + enc_len])
    return key_ok, plain, plain_len, (h, ck)

def check_fir_data(plain):
    ver = u32(plain, 8)
    total = u32(plain, 0x14) + u32(plain, 0x10) * u32(plain, 0x18) + u32(plain, 0x1c)
    s = (~sum(plain[4:4 + total - 4])) & 0xFFFFFFFF
    return ver, u32(plain, 0), s, total

def main():
    rom1 = open(sys.argv[1], 'rb').read()
    fir = open(sys.argv[2], 'rb').read()
    ok = True
    def report(name, good, detail=''):
        nonlocal ok
        ok &= bool(good)
        print('%-34s %s  %s' % (name, 'PASS' if good else 'FAIL', detail))

    report('model id 0x80000218', u32(fir, 0) == 0x80000218, '%08x' % u32(fir, 0))
    report('version string', fir[0x10:0x15] == b'2.1.2', fir[0x10:0x15].decode('latin1'))
    report('embedded size == file size', u32(fir, 0x38) == len(fir), '%#x' % len(fir))
    cs = file_checksum(fir)
    report('file checksum @0x20', cs == u32(fir, 0x20), 'stored %08x computed %08x' % (u32(fir, 0x20), cs))
    t1, t2, pairs = compute_tags(rom1, fir)
    report('tag1 @0x68 (header+updater1)', t1 == fir[0x68:0x88], fir[0x68:0x7c].hex())
    report('tag2 @0x88 (firmware section)', t2 == fir[0x88:0xa8], fir[0x88:0x9c].hex())
    key_ok, plain, plain_len, _ = decrypt_payload(rom1, fir)
    report('content-key hash', key_ok)
    ver, stored, calc, total = check_fir_data(plain)
    report('CheckFirData version == 2', ver == 2, str(ver))
    report('CheckFirData sum @+0', stored == calc, 'stored %08x computed %08x' % (stored, calc))
    report('payload length', total == plain_len, 'table %#x header %#x' % (total, plain_len))
    if len(sys.argv) > 3:
        open(sys.argv[3], 'wb').write(plain[:plain_len])
        print('decrypted payload written to', sys.argv[3])
    print('ALL CHECKS PASS' if ok else 'SOME CHECKS FAILED')
    return 0 if ok else 1

if __name__ == '__main__':
    sys.exit(main())
