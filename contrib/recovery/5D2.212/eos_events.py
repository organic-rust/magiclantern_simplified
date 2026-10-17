#!/usr/bin/env python3
"""
Decode a Canon EOS GetEvent (0x9116) data blob, as saved by
`bootdiag_ptp.py info` (5d2-event0.bin etc.).

Records: u32 size, u32 type, payload; a record of type 0 ends the list.
Only a few property IDs are named; the rest are printed raw.

usage: eos_events.py 5d2-event0.bin [more.bin ...]
"""
import sys
import struct

EVENT_NAMES = {
    0xC181: "ObjectAddedEx",
    0xC182: "RequestObjectTransfer",
    0xC189: "PropValueChanged",
    0xC18A: "AvailListChanged",
    0xC18B: "CameraStatusChanged",
    0xC18D: "WillSoonShutdown",
    0xC18E: "ShutdownTimerUpdated",
}

# conservative: only IDs whose meaning is well established for EOS bodies
PROP_NAMES = {
    0xD101: "Aperture",
    0xD102: "ShutterSpeed",
    0xD103: "ISOSpeed",
    0xD104: "ExposureCompensation",
    0xD105: "AutoExposureMode",
    0xD106: "DriveMode",
    0xD107: "MeteringMode",
    0xD108: "FocusMode",
    0xD109: "WhiteBalance",
}


def decode(blob):
    off = 0
    out = []
    while off + 8 <= len(blob):
        size, etype = struct.unpack_from("<II", blob, off)
        if size < 8 or off + size > len(blob):
            out.append("!! malformed record at 0x%x (size %d)" % (off, size))
            break
        payload = blob[off + 8:off + size]
        name = EVENT_NAMES.get(etype, "0x%04x" % etype)
        if etype == 0:
            out.append("-- end of events (offset 0x%x)" % off)
            break
        if etype == 0xC189 and len(payload) >= 4:
            pid = struct.unpack_from("<I", payload, 0)[0]
            val = payload[4:]
            words = " ".join("%08x" % w for w in struct.unpack_from("<%dI" % (len(val) // 4), val))
            tail = val[len(val) // 4 * 4:]
            out.append("%-18s %06x %-22s [%d] %s%s" % (
                name, pid, PROP_NAMES.get(pid, ""), len(val), words,
                (" +" + tail.hex()) if tail else ""))
        elif etype == 0xC18A and len(payload) >= 12:
            pid, vtype, count = struct.unpack_from("<III", payload, 0)
            vals = payload[12:]
            n = len(vals) // 4
            sample = " ".join("%x" % v for v in struct.unpack_from("<%dI" % min(n, 16), vals))
            out.append("%-18s %06x %-22s type %d, %d values: %s%s" % (
                name, pid, PROP_NAMES.get(pid, ""), vtype, count, sample, " ..." if n > 16 else ""))
        else:
            out.append("%-18s [%d] %s" % (name, len(payload), payload[:48].hex()))
        off += size
    return out


def main(paths):
    if not paths:
        sys.exit(__doc__)
    for p in paths:
        data = open(p, "rb").read()
        print("==== %s (%d bytes)" % (p, len(data)))
        for line in decode(data):
            print(line)


if __name__ == "__main__":
    main(sys.argv[1:])
