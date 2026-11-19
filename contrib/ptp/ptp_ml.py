#!/usr/bin/env python3
"""Talk to Magic Lantern over USB (PTP), using ML's CHDK-style operation
0x9999 (src/ptp-chdk.c): read/write camera memory without the card.

  ptp_ml.py info                       USB device and CHDK protocol version
  ptp_ml.py read ADDR LEN [FILE]       read memory (hex dump, or save)
  ptp_ml.py words ADDR [COUNT]         read 32-bit words (registers)
  ptp_ml.py write ADDR VALUE           write one 32-bit word
  ptp_ml.py mailbox                    list ML's PTP mailbox slots
  ptp_ml.py screenshot [OUT.png]       what the screen shows: LV/playback YUV with
                                       Canon's RGBA overlay (ML menus) on top;
                                       also OUT-yuv.png and OUT-overlay.png
  ptp_ml.py log [FILE]                 fetch dmlog's log (mailbox slot "dmlog";
                                       --slot crxpool: CRX pool plane log)
  ptp_ml.py lv [SECONDS]               remote LiveView test: on, report, off
  ptp_ml.py get CAMFILE [LOCAL]        download a file from the card
                                       (e.g. B:/DCIM/100CANON/M19-1234.MLV)
  ptp_ml.py findmlv                    find MLV file names in camera RAM
  ptp_ml.py ls DIR                     list a card directory (e.g. B:/DCIM/)
  ptp_ml.py latest [LOCAL]             download the newest MLV in DCIM
  ptp_ml.py console                    ML's console text (printf output)
  ptp_ml.py put LOCAL CAMFILE          upload a file to the card (read back to verify)
  ptp_ml.py rm CAMFILE / mkdir DIR / mv SRC DST   card file operations (ML 2026-10-19+)
  ptp_ml.py install [BUILD.zip]        install an ML build zip on the card: changed
                                       files only, each verified, autoexec.bin last;
                                       restart the camera afterwards
  --lv                                 run the command with remote LiveView on
                                       (the camera leaves LV when tethered)

  --cam 5D2|5D4|M6II                   camera preset (default: detected from the USB
                                       product name): card letter for install, and
                                       the build tree in this repository for the
                                       default install zip and --sym

The mailbox address comes from magiclantern.sym of the build on the card
(--sym, default: the camera's build tree; screenshot is M6II-only).  Needs pyusb
and access to the USB device (run with sudo, or add a udev rule for Canon,
vendor 04a9).

Copied from rea-work scripts/ptp_ml.py (5D4 version) with camera presets.
"""
import argparse
import os
import struct
import sys

import usb.core
import usb.util

CANON_VID = 0x04A9
PTP_OC_OpenSession = 0x1002
PTP_OC_CloseSession = 0x1003
PTP_OC_CHDK = 0x9999
PTP_CHDK_Version, PTP_CHDK_GetMemory, PTP_CHDK_SetMemory, PTP_CHDK_CallFunction = 0, 1, 2, 3
PTP_CHDK_TempData, PTP_CHDK_DownloadFile = 4, 6
PTP_CHDK_ML_ListDir = 0x4D4C   # ML extension (src/ptp-chdk.c)
PTP_CHDK_ML_FileOp = 0x4D4D    # ML extension: 1 mkdir, 2 rename, 3 remove
PTP_CHDK_ML_ReadRange = 0x4D4E # ML extension: 64-bit offset read / file size
RANGE_CHUNK = 16 << 20
PTP_CHDK_UploadFile = 5
REPO = os.path.abspath(os.path.join(os.path.dirname(__file__), "..", ".."))
# preset: (USB product name contains, card for install, platform directory)
CAMERAS = {
    "5D2":  ("5D Mark II",  "A:", "5D2.212"),
    "5D4":  ("5D Mark IV",  "A:", "5D4.133"),
    "M6II": ("M6 Mark II",  "B:", "M6II.111"),
}
# USB product IDs, for cameras whose product string is just "Canon Digital Camera"
CAMERA_PIDS = {
    0x3199: "5D2",
}


def cam_build_file(cam, name):
    return os.path.join(REPO, "platform", CAMERAS[cam][2], "build", name)


def detect_cam(dev):
    if getattr(dev, "idProduct", None) in CAMERA_PIDS:
        return CAMERA_PIDS[dev.idProduct]
    try:
        product = usb.util.get_string(dev, dev.iProduct) or ""
    except (usb.core.USBError, ValueError):
        product = ""
    import re
    for cam, (needle, _, _) in CAMERAS.items():
        if re.search(re.escape(needle) + r"(?![A-Za-z])", product):  # not 5D Mark III
            return cam
    return None
PTP_RC_OK = 0x2001
# Canon EOS remote control (as EOS Utility / gphoto2 use it)
PTP_OC_EOS_SetDevicePropValueEx = 0x9110
PTP_OC_EOS_SetRemoteMode = 0x9114
PTP_OC_EOS_SetEventMode = 0x9115
PTP_OC_EOS_GetEvent = 0x9116
PTP_DPC_EOS_EVFOutputDevice = 0xD1B0   # 1 camera screen, 2 PC (bitmask)
PTP_DPC_EOS_EVFMode = 0xD1B1
CONTAINER_CMD, CONTAINER_DATA, CONTAINER_RESP = 1, 2, 3
MAILBOX_MAGIC = 0x4D4C4D42
MAILBOX_SLOTS = 8
CHUNK = 1 << 20     # the camera stages each read in an ML buffer

DEFAULT_SYM = None     # from --cam


class PTP:
    def __init__(self):
        dev = None
        for d in usb.core.find(find_all=True, idVendor=CANON_VID):
            for cfg in d:
                for intf in cfg:
                    if intf.bInterfaceClass == 6:   # still image (PTP)
                        dev, self.intf = d, intf
                        break
        if dev is None:
            sys.exit("no Canon PTP device found (camera on, USB connected?)")
        self.dev = dev
        if dev.is_kernel_driver_active(self.intf.bInterfaceNumber):
            dev.detach_kernel_driver(self.intf.bInterfaceNumber)
        usb.util.claim_interface(dev, self.intf.bInterfaceNumber)
        eps = self.intf.endpoints()
        self.ep_out = next(e for e in eps if usb.util.endpoint_direction(e.bEndpointAddress) == usb.util.ENDPOINT_OUT
                           and usb.util.endpoint_type(e.bmAttributes) == usb.util.ENDPOINT_TYPE_BULK)
        self.ep_in = next(e for e in eps if usb.util.endpoint_direction(e.bEndpointAddress) == usb.util.ENDPOINT_IN
                          and usb.util.endpoint_type(e.bmAttributes) == usb.util.ENDPOINT_TYPE_BULK)
        self.tid = 0
        self.transact(PTP_OC_OpenSession, [1])

    def _cmd(self, code, params):
        self.tid += 1
        hdr = struct.pack("<IHHI", 12 + 4 * len(params), CONTAINER_CMD, code, self.tid)
        self.ep_out.write(hdr + b"".join(struct.pack("<I", p & 0xFFFFFFFF) for p in params))

    def _read_container(self):
        data = bytes(self.ep_in.read(max(512, self.ep_in.wMaxPacketSize), timeout=10000))
        length, ctype, code, tid = struct.unpack_from("<IHHI", data)
        while len(data) < length:
            data += bytes(self.ep_in.read(min(length - len(data) + 512, 1 << 20), timeout=10000))
        return ctype, code, data[12:length]

    def transact(self, code, params, data_out=None, expect_data=False):
        self._cmd(code, params)
        if data_out is not None:
            self.ep_out.write(struct.pack("<IHHI", 12 + len(data_out), CONTAINER_DATA, code, self.tid) + data_out)
        payload = b""
        ctype, rcode, body = self._read_container()
        if ctype == CONTAINER_DATA:
            payload = body
            ctype, rcode, body = self._read_container()
        if ctype != CONTAINER_RESP or rcode != PTP_RC_OK:
            raise RuntimeError(f"PTP op {code:#x} failed: type {ctype} code {rcode:#x}")
        rparams = struct.unpack("<%dI" % (len(body) // 4), body[:len(body) // 4 * 4])
        return payload, rparams

    def chdk_version(self):
        return self.transact(PTP_OC_CHDK, [PTP_CHDK_Version])[1]

    def read(self, addr, length):
        if addr == 0:
            # the camera answers address 0 with an error and no data phase,
            # which leaves the session stuck
            raise ValueError("GetMemory can't read address 0")
        out = b""
        while len(out) < length:
            n = min(CHUNK, length - len(out))
            out += self.transact(PTP_OC_CHDK, [PTP_CHDK_GetMemory, addr + len(out), n], expect_data=True)[0]
        return out

    def words(self, addr, count=1):
        return struct.unpack("<%dI" % count, self.read(addr, 4 * count))

    def write32(self, addr, value):
        self.transact(PTP_OC_CHDK, [PTP_CHDK_SetMemory, addr, 4], data_out=struct.pack("<I", value))

    def download(self, camfile, progress=True):
        self.transact(PTP_OC_CHDK, [PTP_CHDK_TempData, 0], data_out=camfile.encode())
        self._cmd(PTP_OC_CHDK, [PTP_CHDK_DownloadFile])
        first = bytes(self.ep_in.read(1 << 16, timeout=20000))
        length, ctype, rcode, _ = struct.unpack_from("<IHHI", first)
        if ctype != CONTAINER_DATA:
            raise RuntimeError(f"download of {camfile} failed: code {rcode:#x}")
        chunks = [first[12:]]
        got = len(first)
        while got < length:
            c = bytes(self.ep_in.read(min(length - got + 512, 1 << 20), timeout=20000))
            chunks.append(c)
            got += len(c)
            if progress:
                if (got >> 20) != ((got - len(c)) >> 20):
                    print(f"\r{got >> 20} / {length >> 20} MB", end="", file=sys.stderr, flush=True)
        if progress:
            print(file=sys.stderr)
        ctype, rcode, _ = self._read_container()
        if ctype != CONTAINER_RESP or rcode != PTP_RC_OK:
            raise RuntimeError(f"download of {camfile} failed: code {rcode:#x}")
        return b"".join(chunks)[:length - 12]

    def file_size64(self, camfile):
        """64-bit file size via ReadRange (None if the camera's ML lacks it)"""
        try:
            self.transact(PTP_OC_CHDK, [PTP_CHDK_TempData, 0], data_out=camfile.encode())
            lo, hi = self.transact(PTP_OC_CHDK, [PTP_CHDK_ML_ReadRange, 0, 0, 0])[1][:2]
        except RuntimeError:
            return None
        return lo | hi << 32

    def read_range(self, camfile, offset, length):
        self.transact(PTP_OC_CHDK, [PTP_CHDK_TempData, 0], data_out=camfile.encode())
        return self.transact(PTP_OC_CHDK, [PTP_CHDK_ML_ReadRange, offset & 0xFFFFFFFF, offset >> 32, length],
                             expect_data=True)[0]

    def download_ranged(self, camfile, out, size, progress=True):
        """stream a file of any size to the open file `out`"""
        done = 0
        while done < size:
            data = self.read_range(camfile, done, min(RANGE_CHUNK, size - done))
            if not data:
                raise RuntimeError(f"read of {camfile} at {done} returned nothing")
            out.write(data)
            done += len(data)
            if progress:
                print(f"\r{done >> 20} / {size >> 20} MB", end="", file=sys.stderr, flush=True)
        if progress:
            print(file=sys.stderr)
        return done

    def listdir(self, path):
        """[(is_dir, size, timestamp, name)]"""
        self.transact(PTP_OC_CHDK, [PTP_CHDK_TempData, 0], data_out=path.encode())
        text = self.transact(PTP_OC_CHDK, [PTP_CHDK_ML_ListDir], expect_data=True)[0]
        out = []
        for line in text.decode(errors="replace").splitlines():
            parts = line.split(" ", 3)
            if len(parts) == 4:
                try:
                    size, ts = int(parts[1]) & 0xFFFFFFFF, int(parts[2]) & 0xFFFFFFFF
                except ValueError:      # builds before f73cc286c's fix
                    size, ts = 0, 0
                out.append((parts[0].startswith("D"), size, ts, parts[3]))
        return out

    def latest_mlv(self):
        for drive in ("B", "A"):
            best = None
            top = f"{drive}:/DCIM/"
            for is_dir, _, _, name in self.listdir(top):
                if not is_dir or name.startswith("."):
                    continue
                for d, size, ts, fname in self.listdir(top + name + "/"):
                    key = (ts, fname)   # Mdd-hhmm names sort by time as well
                    if not d and fname.upper().endswith(".MLV") and (best is None or key > best[0]):
                        best = (key, f"{top}{name}/{fname}", size)
            if best:
                return best[1], best[2]
        return None, 0

    def call(self, func, *args):
        """call a camera function (CHDK CallFunction): func(args...), <= 6 args"""
        data = struct.pack("<%dI" % (1 + len(args)), func, *[a & 0xFFFFFFFF for a in args])
        return self.transact(PTP_OC_CHDK, [PTP_CHDK_CallFunction], data_out=data)[1]

    def upload(self, camfile, data):
        name = camfile.encode()
        self.transact(PTP_OC_CHDK, [PTP_CHDK_UploadFile],
                      data_out=struct.pack("<I", len(name)) + name + data)

    def fileop(self, op, *paths):
        """op 1 mkdir, 2 rename (src, dst), 3 remove; returns the FIO result"""
        arg = b"\0".join(p.encode() for p in paths)
        self.transact(PTP_OC_CHDK, [PTP_CHDK_TempData, 0], data_out=arg)
        return self.transact(PTP_OC_CHDK, [PTP_CHDK_ML_FileOp, op])[1][0]

    def has_fileop(self):
        try:
            self.transact(PTP_OC_CHDK, [PTP_CHDK_TempData, 0], data_out=b"B:/")
            self.transact(PTP_OC_CHDK, [PTP_CHDK_ML_FileOp, 0])
            return True
        except RuntimeError:
            return False

    def read_file(self, camfile):
        """file contents, or None if missing; empty files via the listing (ML's
        DownloadFile has no data phase for them)"""
        folder, name = camfile.rsplit("/", 1)
        entry = [e for e in self.listdir(folder + "/") if e[3].upper() == name.upper()]
        if not entry:
            # 5D4 builds before dc2380150 garble the listing: ask for the file
            try:
                return self.download(camfile, progress=False)
            except RuntimeError:
                return None
        if entry[0][1] == 0:
            return b""
        try:
            return self.download(camfile, progress=False)
        except RuntimeError:
            return None

    def put_verified(self, camfile, data, use_tmp):
        """upload; with use_tmp, write ~UPLOAD.TMP in the same folder, check it,
        then replace camfile, so a broken transfer never leaves a bad file"""
        target = camfile
        if use_tmp:
            target = camfile.rsplit("/", 1)[0] + "/~UPLOAD.TMP"
        for attempt in range(3):
            self.upload(target, data)
            if self.read_file(target) == data:
                break
        else:
            raise RuntimeError(f"{camfile}: read-back differs after 3 uploads")
        if use_tmp:
            self.fileop(3, camfile)             # FAT rename needs a free name
            if self.fileop(2, target, camfile) != 0:
                raise RuntimeError(f"{camfile}: rename failed (new data is in {target})")

    def eos_setprop(self, prop, value):
        self.transact(PTP_OC_EOS_SetDevicePropValueEx, [],
                      data_out=struct.pack("<III", 12, prop, value))

    def eos_events(self):
        """drain Canon's event queue (EOS Utility polls it all the time)"""
        return self.transact(PTP_OC_EOS_GetEvent, [], expect_data=True)[0]

    def remote_lv(self, on):
        if on:
            self.transact(PTP_OC_EOS_SetRemoteMode, [1])
            self.transact(PTP_OC_EOS_SetEventMode, [1])
            self.eos_events()
            try:
                self.eos_setprop(PTP_DPC_EOS_EVFMode, 1)
            except RuntimeError:
                pass    # not all models have it
            # camera screen + PC; fall back to PC only
            for value in (3, 2):
                try:
                    self.eos_setprop(PTP_DPC_EOS_EVFOutputDevice, value)
                    return value
                except RuntimeError:
                    continue
            raise RuntimeError("EVF output device refused")
        else:
            self.eos_setprop(PTP_DPC_EOS_EVFOutputDevice, 0)
            self.transact(PTP_OC_EOS_SetEventMode, [0])
            self.transact(PTP_OC_EOS_SetRemoteMode, [0])

    def close(self):
        try:
            self.transact(PTP_OC_CloseSession, [])
        finally:
            usb.util.release_interface(self.dev, self.intf.bInterfaceNumber)


def sym_address(path, name):
    with open(path) as f:
        for line in f:
            parts = line.split()
            if len(parts) == 2 and parts[1] == name:
                return int(parts[0], 16)
    # statics are only in the ELF next to the .sym file
    import subprocess
    elf = os.path.join(os.path.dirname(path), "magiclantern")
    try:
        nm = subprocess.run(["arm-none-eabi-nm", elf], capture_output=True, text=True).stdout
    except FileNotFoundError:
        nm = ""
    for line in nm.splitlines():
        parts = line.split()
        if len(parts) == 3 and parts[2] == name:
            return int(parts[0], 16)
    sys.exit(f"{name} not in {path} or {elf}")


def read_mailbox(ptp, sym):
    base = sym_address(sym, "ptp_mailbox")
    raw = ptp.read(base, 4 + MAILBOX_SLOTS * 16)
    if struct.unpack_from("<I", raw)[0] != MAILBOX_MAGIC:
        sys.exit("mailbox magic mismatch: is the card's ML build the one --sym points to?")
    slots = []
    for i in range(MAILBOX_SLOTS):
        ptr, lenptr = struct.unpack_from("<II", raw, 4 + 16 * i)
        name = raw[4 + 16 * i + 8:4 + 16 * i + 16].split(b"\0")[0].decode(errors="replace")
        if ptr:
            length = ptp.words(lenptr)[0] if lenptr else 0
            slots.append((i, name, ptr, length))
    return slots


def hexdump(data, addr):
    for i in range(0, len(data), 16):
        chunk = data[i:i + 16]
        print(f"{addr + i:08x}: {chunk.hex(' ')}")


def write_png(path, width, height, rgb):
    """rgb: bytes, 3 per pixel, rows top to bottom"""
    import zlib
    raw = b"".join(b"\0" + rgb[y * width * 3:(y + 1) * width * 3] for y in range(height))
    def chunk(t, c):
        return struct.pack(">I", len(c)) + t + c + struct.pack(">I", zlib.crc32(t + c) & 0xffffffff)
    open(path, "wb").write(b"\x89PNG\r\n\x1a\n" + chunk(b"IHDR", struct.pack(">IIBBBBB", width, height, 8, 2, 0, 0, 0))
                           + chunk(b"IDAT", zlib.compress(raw, 6)) + chunk(b"IEND", b""))


def screenshot(ptp, sym, out):
    """M6II: the panel's YUV buffer (DispVram 0xB090 + 0xAC, size from ML's
    vram_lv) and Canon's RGBA layer (ML's rgb_vram_info MARV: ARGB, 960x540,
    the 720x480 screen at 120, 30), composited by alpha"""
    disp = ptp.words(0xB090)[0]
    panel = ptp.words(disp + 0xAC)[0] if disp else 0
    vram, w, pitch, h = ptp.words(sym_address(sym, "vram_lv"), 4)
    if not (320 <= w <= 2048 and w * 2 <= pitch <= 8192 and 200 <= h <= 2048):
        # vram_lv from a .sym that doesn't match the running build, or not
        # set up yet: M6II's panel buffer is 736x480 UYVY
        vram, w, pitch, h = 0, 736, 1472, 480
    # 0x1000000 = no buffer (LiveView off, e.g. Canon's PC connection screen)
    yuv_addr = next((a for a in (panel, vram) if a > 0x100000 and a != 0x1000000), 0)
    if not yuv_addr or not w or not h:
        sys.exit(f"no LiveView/playback picture (DispVram panel {panel:#x}, vram_lv {vram:#x} {w}x{h}); "
                 "try --lv 2 (remote LiveView), or the camera is in a menu")
    pitch = pitch or w * 2
    yuv = ptp.read(yuv_addr, pitch * h)
    rgb = bytearray(w * h * 3)
    clamp = lambda v: 0 if v < 0 else 255 if v > 255 else int(v)
    # chroma bytes hover around 128 when stored with an offset (D6-8)
    sample = yuv[pitch * (h // 2):pitch * (h // 2) + pitch]
    chroma = list(sample[0::4]) + list(sample[2::4])
    unsigned_chroma = sum(1 for c in chroma if 64 <= c < 192) > len(chroma) // 2
    for y in range(h):
        row = yuv[y * pitch:y * pitch + w * 2]
        for x in range(0, w - 1, 2):
            u, y0, v, y1 = row[2 * x], row[2 * x + 1], row[2 * x + 2], row[2 * x + 3]
            if unsigned_chroma:                # DIGIC 6-8: 0x80 = neutral
                u -= 128; v -= 128
            else:                              # DIGIC 4/5: signed
                u = u - 256 if u > 127 else u
                v = v - 256 if v > 127 else v
            for k, yy in ((0, y0), (1, y1)):
                o = (y * w + x + k) * 3
                rgb[o] = clamp(yy + 1.402 * v)
                rgb[o + 1] = clamp(yy - 0.344 * u - 0.714 * v)
                rgb[o + 2] = clamp(yy + 1.772 * u)
    base = out[:-4] if out.lower().endswith(".png") else out
    write_png(base + "-yuv.png", w, h, bytes(rgb))

    marv = ptp.words(sym_address(sym, "rgb_vram_info"))[0]
    if marv:
        sig, data, opacity, flags, lw, lh = ptp.words(marv, 6)
        if data and lw and lh:
            layer = ptp.read(data, lw * lh * 4)
            ov = bytearray(w * h * 3)
            ox, oy = (lw - 720) // 2 if lw >= 720 else 0, 30 if lh >= 510 else 0
            ox = 120 if lw == 960 else ox
            for y in range(min(h, lh - oy)):
                for x in range(min(w, lw - ox)):
                    b, g, r, a = layer[((y + oy) * lw + x + ox) * 4:((y + oy) * lw + x + ox) * 4 + 4]
                    o = (y * w + x) * 3
                    ov[o:o + 3] = bytes((r, g, b))
                    if a:
                        rgb[o] = (r * a + rgb[o] * (255 - a)) // 255
                        rgb[o + 1] = (g * a + rgb[o + 1] * (255 - a)) // 255
                        rgb[o + 2] = (b * a + rgb[o + 2] * (255 - a)) // 255
            write_png(base + "-overlay.png", w, h, bytes(ov))
            print(f"overlay: MARV {marv:#x} data {data:#x} {lw}x{lh}")
    write_png(base + ".png", w, h, bytes(rgb))
    print(f"YUV {yuv_addr:#x} {w}x{h} pitch {pitch} -> {base}.png (+ -yuv, -overlay)")


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--sym", default=DEFAULT_SYM,
                    help="magiclantern.sym of the build on the card (default: from --cam)")
    ap.add_argument("--cam", choices=sorted(CAMERAS), default=None,
                    help="camera preset (default: detected from the USB product name)")
    ap.add_argument("--lv", type=float, default=0, metavar="SECONDS",
                    help="start remote LiveView and wait SECONDS before the command")
    ap.add_argument("cmd", choices=["info", "read", "words", "write", "mailbox", "log", "lv",
                                    "get", "findmlv", "ls", "latest", "console",
                                    "put", "rm", "mkdir", "mv", "install", "screenshot"])
    ap.add_argument("--card", default=None, help="card for install (default: from --cam; 5D2/5D4 A:, M6II B:)")
    ap.add_argument("--slot", default="dmlog", help="mailbox slot for the log command")
    ap.add_argument("args", nargs="*")
    a = ap.parse_args()
    num = lambda s: int(s, 0)

    ptp = PTP()
    cam = a.cam or detect_cam(ptp.dev)
    if cam:
        a.card = a.card or CAMERAS[cam][1]
        a.sym = a.sym or cam_build_file(cam, "magiclantern.sym")
    elif a.card is None or (a.cmd == "install" and not a.args):
        sys.exit("camera not recognised: pass --cam (and --card / a build zip)")
    lv_on = False
    try:
        if a.lv or a.cmd == "lv":
            import time
            value = ptp.remote_lv(True)
            lv_on = True
            wait = a.lv or (float(a.args[0]) if a.args else 3)
            t_end = time.time() + wait
            while time.time() < t_end:
                ptp.eos_events()      # keep the session alive
                time.sleep(0.2)
            print(f"remote LiveView on (EVF output {value})", file=sys.stderr)
        if a.cmd == "info":
            d = ptp.dev
            print(f"{d.idVendor:04x}:{d.idProduct:04x} {usb.util.get_string(d, d.iProduct)}")
            print("CHDK protocol version", ptp.chdk_version())
        elif a.cmd == "read":
            addr, length = num(a.args[0]), num(a.args[1])
            data = ptp.read(addr, length)
            if len(a.args) > 2:
                open(a.args[2], "wb").write(data)
                print(f"saved {len(data)} bytes to {a.args[2]}")
            else:
                hexdump(data, addr)
        elif a.cmd == "words":
            addr = num(a.args[0]); count = num(a.args[1]) if len(a.args) > 1 else 1
            for i, w in enumerate(ptp.words(addr, count)):
                print(f"{addr + 4 * i:08x}: {w:08x}")
        elif a.cmd == "write":
            ptp.write32(num(a.args[0]), num(a.args[1]))
            print("ok")
        elif a.cmd == "mailbox":
            for i, name, ptr, length in read_mailbox(ptp, a.sym):
                print(f"slot {i}: {name:8s} ptr {ptr:#010x} len {length}")
        elif a.cmd == "lv":
            lv_addr = sym_address(a.sym, "lv")
            print("ML lv =", ptp.words(lv_addr)[0])
            print("LV raw EDMAC ch1 addr/xb:", " ".join(f"{w:08x}" for w in
                  (ptp.words(0xD04200A0)[0], ptp.words(0xD042005C)[0])))
        elif a.cmd == "ls":
            for is_dir, size, ts, name in ptp.listdir(a.args[0]):
                print(f"{'D' if is_dir else ' '} {size:12d} {ts:10d} {name}")
        elif a.cmd in ("get", "latest"):
            import time
            if a.cmd == "latest":
                camfile, size = ptp.latest_mlv()
                if not camfile:
                    sys.exit("no MLV files found in B:/DCIM or A:/DCIM")
                print(f"newest: {camfile} ({size >> 20} MB)")
                a.args = [camfile] + a.args
            t = time.time()
            local = a.args[1] if len(a.args) > 1 else a.args[0].replace("\\", "/").split("/")[-1]
            size = ptp.file_size64(a.args[0])
            if size is not None:
                # ranged reads: any size (DownloadFile stops at 2 GB)
                with open(local, "wb") as out:
                    n = ptp.download_ranged(a.args[0], out, size)
            else:
                data = ptp.download(a.args[0])
                open(local, "wb").write(data)
                n = len(data)
            dt = time.time() - t
            print(f"saved {n} bytes to {local} ({n / dt / 1e6:.1f} MB/s)")
        elif a.cmd == "findmlv":
            import re
            found = set()
            for base in range(0x00100000, 0x01000000, CHUNK):
                for m in re.finditer(rb"[A-Z]:/DCIM/[0-9A-Z_]{8}/[A-Z0-9_-]{4,8}\.MLV", ptp.read(base, CHUNK)):
                    found.add(m.group().decode())
            for f in sorted(found):
                print(f)
        elif a.cmd == "put":
            ptp.put_verified(a.args[1], open(a.args[0], "rb").read(), ptp.has_fileop())
            print("ok")
        elif a.cmd in ("rm", "mkdir", "mv"):
            op = {"mkdir": 1, "mv": 2, "rm": 3}[a.cmd]
            print("result", ptp.fileop(op, *a.args))
        elif a.cmd == "install":
            import zipfile
            z = zipfile.ZipFile(a.args[0] if a.args else cam_build_file(cam, "magiclantern.zip"))
            files = [i for i in z.infolist() if not i.filename.endswith("/")]
            files.sort(key=lambda i: i.filename.lower() == "autoexec.bin")   # last
            use_tmp = ptp.has_fileop()
            if not use_tmp:
                print("camera's ML has no file operations yet: direct uploads (verified)")
                print("  if the transfer breaks, the card may need fixing in a reader")
            dirs = sorted({a.card + "/" + i.filename.rsplit("/", 1)[0] for i in files if "/" in i.filename})
            for d in dirs:
                if use_tmp:
                    ptp.fileop(1, d)      # fails harmlessly if it exists
            changed = 0
            for i in files:
                camfile = a.card + "/" + i.filename
                data = z.read(i)
                if ptp.read_file(camfile) == data:
                    continue
                print(f"  {camfile} ({len(data)} bytes)")
                ptp.put_verified(camfile, data, use_tmp)
                changed += 1
            # a module hidden in an older build stays hidden while its .hid
            # marker is on the card: drop markers the build no longer has
            hid = {i.filename.lower() for i in files if i.filename.lower().endswith(".hid")}
            if use_tmp and any(f.startswith("ml/modules/") for f in hid):
                for is_dir, size, ts, name in ptp.listdir(a.card + "/ML/modules/"):
                    if not is_dir and name.lower().endswith(".hid") and "ml/modules/" + name.lower() not in hid:
                        print(f"  removing stale {a.card}/ML/modules/{name}")
                        ptp.fileop(3, a.card + "/ML/modules/" + name)
                        changed += 1
            print(f"installed: {changed} of {len(files)} files changed; restart the camera to run it")
        elif a.cmd == "screenshot":
            screenshot(ptp, a.sym, a.args[0] if a.args else "screen.png")
        elif a.cmd == "console":
            W, H = 80, 21           # src/console.c
            buf = ptp.read(sym_address(a.sym, "console_buffer"), W * H)
            idx = ptp.words(sym_address(a.sym, "console_buffer_index"))[0]
            start = (idx // W + 1) % H      # oldest line first
            for i in range(H):
                line = buf[((start + i) % H) * W:((start + i) % H + 1) * W]
                text = line.decode("latin-1").rstrip()
                if text:
                    print(text)
        elif a.cmd == "log":
            slots = {name: (ptr, length) for _, name, ptr, length in read_mailbox(ptp, a.sym)}
            if a.slot not in slots:
                sys.exit(f"no {a.slot} slot" + (": start Debug > Canon debug log first" if a.slot == "dmlog" else ""))
            ptr, length = slots[a.slot]
            data = ptp.read(ptr, length)
            if a.args:
                open(a.args[0], "wb").write(data)
                print(f"saved {length} bytes to {a.args[0]}")
            else:
                sys.stdout.write(data.decode(errors="replace"))
    finally:
        if lv_on:
            try:
                ptp.remote_lv(False)
            except RuntimeError as e:
                print("remote LiveView off:", e, file=sys.stderr)
        ptp.close()


if __name__ == "__main__":
    main()
