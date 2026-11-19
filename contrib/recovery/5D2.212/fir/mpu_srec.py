#!/usr/bin/env python3
"""Decode the binary S-record MPU pieces (tag 0x200) of a decrypted 5D2 FIR payload.
Record layout (as consumed by updater 0x847c28): [type][count][addr..][data..][checksum]
usage: mpu_srec.py PAYLOAD.BIN OUTDIR"""
import sys, struct, os

def parse_srec(blob):
    i = 0; recs = []
    while i + 2 <= len(blob):
        t, n = blob[i], blob[i + 1]
        body = blob[i + 2:i + 2 + n]
        alen = {0: 2, 1: 2, 2: 3, 3: 4, 5: 2, 7: 4, 8: 3, 9: 2}.get(t)
        if alen is None or len(body) < n:
            recs.append((t, None, b'', False)); i += n + 2; continue
        addr = int.from_bytes(body[:alen], 'big'); data = body[alen:-1]
        ok = ((n + sum(body[:-1])) & 0xFF) ^ 0xFF == body[-1]
        recs.append((t, addr, data, ok)); i += n + 2
    return recs

def ranges(recs):
    """merge data records into contiguous (start, bytes) ranges"""
    out = []
    for t, a, d, ok in recs:
        if t not in (1, 2, 3) or not d: continue
        if out and out[-1][0] + len(out[-1][1]) == a:
            out[-1][1].extend(d)
        else:
            out.append([a, bytearray(d)])
    return out

def main():
    p = open(sys.argv[1], 'rb').read(); od = sys.argv[2]
    cnt, eoff, esz, total = struct.unpack_from('<4I', p, 0x10)
    for i in range(cnt):
        no, tag, off, size, addr, aux = struct.unpack_from('<6I', p, eoff + i * esz)
        if tag != 0x200: continue
        recs = parse_srec(p[off:off + size])
        name = next((d for t, a, d, ok in recs if t == 0), b'')
        types = {}
        for r in recs: types[r[0]] = types.get(r[0], 0) + 1
        bad = sum(1 for r in recs if r[3] is False)
        rg = ranges(recs)
        print('piece #%d %s: %d records %s, bad checksums %d' % (no, name, len(recs), types, bad))
        for a, d in rg:
            fn = os.path.join(od, 'mpu_piece%d_%08x.bin' % (no, a))
            open(fn, 'wb').write(d)
            print('   %08x-%08x (%6d bytes) -> %s' % (a, a + len(d) - 1, len(d), fn))

if __name__ == '__main__':
    main()
