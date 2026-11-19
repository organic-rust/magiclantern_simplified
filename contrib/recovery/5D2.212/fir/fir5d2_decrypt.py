#!/usr/bin/env python3
"""
Decrypt the firmware payload of a Canon 5D Mark II .FIR, re-implementing what
the 2.1.x updater does (updater1 addresses are for 5D200212.FIR, base 0x800120):

  * two 32-byte obfuscated keys live in the CAMERA's ROM1 at 0xF87C0000
    ([+4] -> payload key, [+8] -> HMAC key), read at updater 0x804e14
  * de-obfuscation: updater 0x8785c4 (2 rounds of word rotations, constant
    add/sub, 256-bit rotate, word reversal)
  * KDF: updater 0x848310: SHA1(x||01) || SHA1(x||02)[:12]   (32 bytes)
  * payload header (FIR + [0x30] + [hdr+0]):  see decrypt_payload() / updater 0x80a3a0
  * AES wrapper 0x8481ec always uses a 16-byte key (AES-128); mode driver 0x8a212c is CTR

usage: fir5d2_decrypt.py ROM1.BIN FIRFILE OUT.BIN
"""
import sys, struct, hashlib
from cryptography.hazmat.primitives.ciphers import Cipher, algorithms, modes

M = 0xFFFFFFFF
def rotr(x, k): k %= 32; return ((x >> k) | (x << (32 - k))) & M
def rotl(x, k): k %= 32; return ((x << k) | (x >> (32 - k))) & M

def deobf(src):
    n = len(src) // 4
    w = list(struct.unpack('<%dI' % n, src))
    i1 = int(n / 4 + 0.5); i2 = int(n / 2 + 0.5); i3 = int(n / 4 * 3.0 + 0.5)
    for _ in range(2):
        w = [rotr(x, 6) for x in w]
        w[0] = (w[0] - 0x8903203a) & M
        w[i1] = (w[i1] - 0x903288bc) & M
        w[i2] = (w[i2] + 0x0721dca3) & M
        w[i3] = (w[i3] + 0x378ffdf0) & M
        w = [rotl(x, 1) for x in w]
        top = w[0] & 0xe0000000
        for i in range(n - 1):
            w[i] = ((w[i] << 3) | (w[i + 1] >> 29)) & M
        w[n - 1] = ((w[n - 1] << 3) | (top >> 29)) & M
        w = [rotr(x, 3) for x in reversed(w)]
    return struct.pack('<%dI' % n, *w)

def kdf(x):
    return hashlib.sha1(x + b'\x01').digest() + hashlib.sha1(x + b'\x02').digest()[:12]

def aes_ctr(key, iv, data):
    """AES-CTR as in updater 0x8a212c: keystream = AES-encrypt(counter);
    only the last 4 counter bytes increment (big-endian, no carry into byte 11)."""
    nblk = (len(data) + 15) // 16
    head, ctr0 = iv[:12], int.from_bytes(iv[12:], 'big')
    ctrs = b''.join(head + ((ctr0 + i) & 0xFFFFFFFF).to_bytes(4, 'big') for i in range(nblk))
    e = Cipher(algorithms.AES(key), modes.ECB()).encryptor()
    ks = e.update(ctrs) + e.finalize()
    return bytes(a ^ b for a, b in zip(data, ks))

def main():
    rom1 = open(sys.argv[1], 'rb').read()
    fir = open(sys.argv[2], 'rb').read()
    ko = 0x7C0000                                   # F87C0000 in the ROM1 dump
    hdr = struct.unpack_from('<3I', rom1, ko)
    key_payload_obf = rom1[ko + hdr[1]: ko + hdr[1] + 32]
    secret = deobf(key_payload_obf)
    fw = struct.unpack_from('<I', fir, 0x30)[0]
    h = fw + struct.unpack_from('<I', fir, fw)[0]  # "offset to decryption data"
    enc_len, plain_len, _, salt = struct.unpack_from('<4I', fir, h)
    iv1 = fir[h + 16:h + 32]; wrapped = fir[h + 32:h + 64]
    iv2 = fir[h + 64:h + 80]; khash = fir[h + 80:h + 112]
    x = struct.pack('<I', salt) + secret + struct.pack('<I', plain_len)
    kek = kdf(x)
    for klen in (16, 32):
        ck = aes_ctr(kek[:klen], iv1, wrapped)
        ok = kdf(ck) == khash
        print('AES-%d unwrap: key-hash check %s' % (klen * 8, 'OK' if ok else 'failed'))
        if ok:
            data = fir[h + 112: h + 112 + enc_len]
            out = aes_ctr(ck[:klen], iv2, data)
            open(sys.argv[3], 'wb').write(out)
            print('payload: %d bytes encrypted, header says %d plain; wrote %s'
                  % (enc_len, plain_len, sys.argv[3]))
            return 0
    return 1

if __name__ == '__main__':
    sys.exit(main())
