#!/usr/bin/env python3
"""
Read the BOOT_LED_DIAG log from a 5D2 over USB (PTP opcode 0xA1D9).

Works while the camera is in the black-screen state, as long as it shows up
on USB. Needs pyusb and a libusb backend:

    pip install pyusb libusb-package

Windows: the camera normally uses the WIA/MTP driver, which libusb can't
open. Use Zadig (https://zadig.akeo.ie) to switch the "Canon Digital Camera"
device to WinUSB. Reverse it later in Device Manager (uninstall the device
and tick "delete the driver software"), or just use a different USB port.

Linux: stop gvfs grabbing the camera first, e.g.
    pkill -f gvfs-gphoto2-volume-monitor ; pkill -f gvfsd-gphoto2
and run as root or add a udev rule for 04a9.

Usage:
    bootdiag_ptp.py status
    bootdiag_ptp.py log  [out.txt]          (default BOOTDIAG-USB.LOG)
    bootdiag_ptp.py mem  ADDR LEN [out.bin] (hex or decimal)
"""
import sys
import struct

import usb.core
import usb.util

try:
    import libusb_package
    _backend = libusb_package.get_libusb1_backend()
except ImportError:
    _backend = None

CANON_VID = 0x04A9
OP_OPEN_SESSION = 0x1002
OP_CLOSE_SESSION = 0x1003
OP_BOOTDIAG = 0xA1D9
RC_OK = 0x2001
RC_SESSION_ALREADY_OPEN = 0x201E
TYPE_COMMAND, TYPE_DATA, TYPE_RESPONSE = 1, 2, 3
TIMEOUT_MS = 20000


class PTP:
    def __init__(self):
        dev = usb.core.find(idVendor=CANON_VID, backend=_backend)
        if dev is None:
            sys.exit("No Canon device found on USB.")
        self.dev = dev
        try:
            if dev.is_kernel_driver_active(0):
                dev.detach_kernel_driver(0)
        except (NotImplementedError, usb.core.USBError):
            pass
        dev.set_configuration()
        cfg = dev.get_active_configuration()
        intf = usb.util.find_descriptor(cfg, bInterfaceClass=6) or cfg[(0, 0)]
        usb.util.claim_interface(dev, intf.bInterfaceNumber)
        self.intf = intf

        def ep(direction, kind):
            return usb.util.find_descriptor(
                intf, custom_match=lambda e:
                usb.util.endpoint_direction(e.bEndpointAddress) == direction and
                usb.util.endpoint_type(e.bmAttributes) == kind)

        self.ep_out = ep(usb.util.ENDPOINT_OUT, usb.util.ENDPOINT_TYPE_BULK)
        self.ep_in = ep(usb.util.ENDPOINT_IN, usb.util.ENDPOINT_TYPE_BULK)
        if self.ep_out is None or self.ep_in is None:
            sys.exit("Couldn't find the PTP bulk endpoints.")
        self.tid = 0

    def _send_command(self, code, params):
        self.tid += 1
        hdr = struct.pack("<IHHI", 12 + 4 * len(params), TYPE_COMMAND, code, self.tid)
        self.ep_out.write(hdr + b"".join(struct.pack("<I", p & 0xFFFFFFFF) for p in params), TIMEOUT_MS)

    def _read_container(self):
        buf = bytes(self.ep_in.read(512 * 1024, TIMEOUT_MS))
        length, ctype, code, tid = struct.unpack_from("<IHHI", buf, 0)
        while len(buf) < length:
            buf += bytes(self.ep_in.read(max(512, length - len(buf)), TIMEOUT_MS))
        return ctype, code, buf[12:length]

    def transaction(self, code, params=()):
        """Returns (response code, response params, data or None)."""
        self._send_command(code, list(params))
        data = None
        while True:
            ctype, rcode, payload = self._read_container()
            if ctype == TYPE_DATA:
                data = (data or b"") + payload
                continue
            if ctype == TYPE_RESPONSE:
                rparams = list(struct.unpack("<%dI" % (len(payload) // 4), payload[:len(payload) // 4 * 4]))
                return rcode, rparams, data

    def open(self):
        rc, _, _ = self.transaction(OP_OPEN_SESSION, [1])
        if rc not in (RC_OK, RC_SESSION_ALREADY_OPEN):
            sys.exit("OpenSession failed: 0x%04x" % rc)

    def close(self):
        try:
            self.transaction(OP_CLOSE_SESSION)
        except usb.core.USBError:
            pass
        usb.util.release_interface(self.dev, self.intf.bInterfaceNumber)
        usb.util.dispose_resources(self.dev)


def main(argv):
    if len(argv) < 2 or argv[1] not in ("status", "log", "mem"):
        sys.exit(__doc__)
    ptp = PTP()
    ptp.open()
    try:
        if argv[1] == "status":
            rc, p, _ = ptp.transaction(OP_BOOTDIAG, [0])
            if rc != RC_OK:
                sys.exit("status failed: 0x%04x (is the BOOT_LED_DIAG build running?)" % rc)
            names = ("stage", "hook_errors", "records", "linear", "ring")
            for n, v in zip(names, p):
                print("%-12s %d (0x%x)" % (n, v, v))
        elif argv[1] == "log":
            out = argv[2] if len(argv) > 2 else "BOOTDIAG-USB.LOG"
            rc, _, data = ptp.transaction(OP_BOOTDIAG, [1])
            if rc != RC_OK or data is None:
                sys.exit("log failed: 0x%04x" % rc)
            open(out, "wb").write(data)
            print("wrote %d bytes to %s" % (len(data), out))
        else:
            addr, length = int(argv[2], 0), int(argv[3], 0)
            out = argv[4] if len(argv) > 4 else "MEM_%08X.BIN" % addr
            rc, _, data = ptp.transaction(OP_BOOTDIAG, [2, addr, length])
            if rc != RC_OK or data is None:
                sys.exit("mem read failed: 0x%04x" % rc)
            open(out, "wb").write(data)
            print("wrote %d bytes to %s" % (len(data), out))
    finally:
        ptp.close()


if __name__ == "__main__":
    main(sys.argv)
