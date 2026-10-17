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
    bootdiag_ptp.py probe                   (step-by-step connectivity check;
                                             a timeout wedges the camera's USB,
                                             so unplug/replug before the next run)
    bootdiag_ptp.py info  [prefix]          (device info, storage, EOS property dump;
                                             only requests known to work)
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
TIMEOUT_MS = 5000
OP_GET_DEVICE_INFO = 0x1001
VERBOSE = True


def log(msg):
    if VERBOSE:
        print("[ptp] " + msg, flush=True)


class PTP:
    def __init__(self):
        log("looking for a Canon device (VID 04a9)...")
        dev = usb.core.find(idVendor=CANON_VID, backend=_backend)
        if dev is None:
            sys.exit("No Canon device found on USB (on Windows: is the WinUSB driver installed with Zadig?).")
        log("found %04x:%04x" % (dev.idVendor, dev.idProduct))
        self.dev = dev
        try:
            if dev.is_kernel_driver_active(0):
                dev.detach_kernel_driver(0)
        except (NotImplementedError, usb.core.USBError):
            pass
        try:
            cfg = dev.get_active_configuration()
            log("device already configured (config %d)" % cfg.bConfigurationValue)
        except usb.core.USBError:
            log("setting configuration")
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
        log("interface %d, bulk out 0x%02x, bulk in 0x%02x" %
            (intf.bInterfaceNumber, self.ep_out.bEndpointAddress, self.ep_in.bEndpointAddress))
        self.tid = 0

    def recover(self):
        """After a timeout: PTP class reset + clear endpoint halts."""
        for fn, name in ((self.device_reset, "DEVICE_RESET"),
                         (lambda: self.dev.clear_halt(self.ep_in), "clear halt IN"),
                         (lambda: self.dev.clear_halt(self.ep_out), "clear halt OUT")):
            try:
                fn()
                log("recover: %s ok" % name)
            except usb.core.USBError as e:
                log("recover: %s failed (%s)" % (name, e))

    def device_status(self):
        """PTP class request GET_DEVICE_STATUS (0x67): returns the status code."""
        r = self.dev.ctrl_transfer(0xA1, 0x67, 0, self.intf.bInterfaceNumber, 64, TIMEOUT_MS)
        r = bytes(r)
        return struct.unpack_from("<H", r, 2)[0] if len(r) >= 4 else None

    def device_reset(self):
        """PTP class request DEVICE_RESET (0x66)."""
        self.dev.ctrl_transfer(0x21, 0x66, 0, self.intf.bInterfaceNumber, None, TIMEOUT_MS)

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
        log("-> op 0x%04x %s" % (code, " ".join("0x%x" % p for p in params)))
        self._send_command(code, list(params))
        log("   command sent, waiting for the camera...")
        data = None
        while True:
            ctype, rcode, payload = self._read_container()
            log("   <- container type %d code 0x%04x, %d bytes" % (ctype, rcode, len(payload)))
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
    if len(argv) < 2 or argv[1] not in ("probe", "info", "status", "log", "mem"):
        sys.exit(__doc__)
    ptp = PTP()

    if argv[1] == "probe":
        steps = [
            ("class request GET_DEVICE_STATUS", lambda: "0x%04x" % (ptp.device_status() or 0)),
            ("GetDeviceInfo (no session needed)", lambda: "response 0x%04x, %d bytes"
                % (lambda r: (r[0], len(r[2] or b"")))(ptp.transaction(OP_GET_DEVICE_INFO))),
            ("OpenSession", lambda: "response 0x%04x" % ptp.transaction(OP_OPEN_SESSION, [1])[0]),
            ("GetStorageIDs 0x1004 (standard op with a real handler)", lambda: "response 0x%04x, %d bytes"
                % (lambda r: (r[0], len(r[2] or b"")))(ptp.transaction(0x1004))),
            ("Canon SetRemoteMode 0x9114(1)", lambda: "response 0x%04x" % ptp.transaction(0x9114, [1])[0]),
            ("Canon SetEventMode 0x9115(1)", lambda: "response 0x%04x" % ptp.transaction(0x9115, [1])[0]),
            ("Canon GetEvent 0x9116", lambda: "response 0x%04x, %d bytes"
                % (lambda r: (r[0], len(r[2] or b"")))(ptp.transaction(0x9116))),
            ("bootdiag status (0xA1D9, 0)", lambda: "response 0x%04x params %s"
                % (lambda r: (r[0], r[1]))(ptp.transaction(OP_BOOTDIAG, [0]))),
        ]
        for name, fn in steps:
            print("== %s" % name, flush=True)
            try:
                print("   OK: %s" % fn(), flush=True)
            except usb.core.USBTimeoutError:
                print("   TIMEOUT after %d ms - the camera didn't answer this step" % TIMEOUT_MS, flush=True)
                ptp.recover()
                try:
                    print("   device status now: 0x%04x" % (ptp.device_status() or 0), flush=True)
                except usb.core.USBError as e:
                    print("   (device status also failed: %s)" % e, flush=True)
                break
            except usb.core.USBError as e:
                print("   USB error: %s" % e, flush=True)
                break
        ptp.close()
        return

    ptp.open()
    try:
        if argv[1] == "info":
            prefix = argv[2] if len(argv) > 2 else "5d2"
            rc, _, di = ptp.transaction(OP_GET_DEVICE_INFO)
            open(prefix + "-deviceinfo.bin", "wb").write(di or b"")
            rc, _, sids = ptp.transaction(0x1004)
            open(prefix + "-storageids.bin", "wb").write(sids or b"")
            ids = []
            if sids and len(sids) >= 4:
                n = struct.unpack_from("<I", sids, 0)[0]
                ids = list(struct.unpack_from("<%dI" % n, sids, 4))
            print("storage IDs: %s" % ", ".join("0x%08x" % i for i in ids) or "none")
            for sid in ids:
                rc, _, si = ptp.transaction(0x1005, [sid])     # GetStorageInfo
                print("  storage 0x%08x: GetStorageInfo 0x%04x, %d bytes" % (sid, rc, len(si or b"")))
                open(prefix + "-storageinfo-%08x.bin" % sid, "wb").write(si or b"")
            ptp.transaction(0x9114, [1])                       # SetRemoteMode
            ptp.transaction(0x9115, [1])                       # SetEventMode
            for i in range(3):
                rc, _, ev = ptp.transaction(0x9116)            # GetEvent
                fn = "%s-event%d.bin" % (prefix, i)
                open(fn, "wb").write(ev or b"")
                print("GetEvent #%d: 0x%04x, %d bytes -> %s" % (i, rc, len(ev or b""), fn))
            print("send me the %s-*.bin files" % prefix)
        elif argv[1] == "status":
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
    except usb.core.USBTimeoutError:
        print("timed out; resetting the PTP pipe", flush=True)
        ptp.recover()
        raise
    finally:
        ptp.close()


if __name__ == "__main__":
    main(sys.argv)
