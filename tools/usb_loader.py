#!/usr/bin/env python3
"""Load data into the RP2350 USB-to-SPI-slave bridge.

Frame format (little endian):
    AA 55 | type | seq[2] | len[2] | payload[len] | crc16[2]

crc16 is CCITT-FALSE (poly 0x1021, init 0xFFFF) over type..payload.

Host -> device:
    0x01 WRITE   append payload to the device ring buffer
    0x02 STATUS  read counters and buffer levels

Device -> host:
    0x81 ACK     u32 accepted, u32 used, u32 free
    0x82 STATUS  u32 capacity, used, free, stage_ahead, spi_consumed,
                 zero_fill, crc_errors, bad_frames, underrun_events, ring_full

The SPI master drains 64 bytes every millisecond (64 kB/s = 64,000 B/s). WRITE is
flow-controlled: the loader retries the unaccepted remainder until the
whole chunk has been queued.
"""

import argparse
import binascii
import os
import struct
import sys
import time

try:
    import serial
    from serial.tools import list_ports
except ImportError:
    sys.exit("pyserial is required: pip install pyserial")

MAGIC = b"\xAA\x55"
CMD_WRITE = 0x01
CMD_STATUS = 0x02
RSP_ACK = 0x81
RSP_STATUS = 0x82

FRAME_MAX_PAYLOAD = 4096
DRAIN_RATE = 64 * 1000
# pico_stdio_usb defaults on RP2350: 2e8a:0009 (RP2040 uses 2e8a:000a).
USB_VID_PID = (0x2E8A, 0x0009)

STATUS_FIELDS = (
    "capacity",
    "used",
    "free",
    "stage_ahead",
    "spi_consumed",
    "zero_fill",
    "crc_errors",
    "bad_frames",
    "underrun_events",
    "ring_full_events",
)


class ProtocolError(Exception):
    pass


def find_port():
    ports = list(list_ports.comports())
    matches = [p.device for p in ports if (p.vid, p.pid) == USB_VID_PID]
    if matches:
        return matches[0]
    available = ", ".join(p.device for p in ports) or "none"
    raise SystemExit(
        f"no Pico USB CDC device found ({USB_VID_PID[0]:04x}:{USB_VID_PID[1]:04x}); "
        f"use --port. Available: {available}"
    )


class Bridge:
    def __init__(self, port, baud=115200, timeout=5.0):
        self.ser = serial.Serial(port, baud, timeout=0.05)
        self.timeout = timeout
        self.seq = 0
        self.buf = bytearray()
        self.crc_errors = 0

    def close(self):
        self.ser.close()

    def __enter__(self):
        return self

    def __exit__(self, *exc):
        self.close()

    def _send_frame(self, typ, seq, payload=b""):
        body = struct.pack("<BHH", typ, seq, len(payload)) + payload
        crc = binascii.crc_hqx(body, 0xFFFF)
        self.ser.write(MAGIC + body + struct.pack("<H", crc))
        self.ser.flush()

    def _recv_more(self, deadline):
        remaining = deadline - time.monotonic()
        if remaining <= 0:
            raise TimeoutError("device did not reply (is the firmware running?)")
        self.ser.timeout = min(remaining, 0.05)
        data = self.ser.read(256)
        if data:
            self.buf.extend(data)

    def _read_frame(self, deadline):
        while True:
            pos = self.buf.find(MAGIC)
            if pos < 0:
                # Drop junk but keep a trailing AA in case it is a split magic.
                if self.buf[-1:] == b"\xAA":
                    del self.buf[:-1]
                else:
                    self.buf.clear()
                self._recv_more(deadline)
                continue

            if pos:
                del self.buf[:pos]
            if len(self.buf) < 7:
                self._recv_more(deadline)
                continue

            typ, seq, length = struct.unpack_from("<BHH", self.buf, 2)
            if length > FRAME_MAX_PAYLOAD:
                del self.buf[:1]
                continue

            end = 7 + length
            if len(self.buf) < end + 2:
                self._recv_more(deadline)
                continue

            body = bytes(self.buf[2:end])
            (crc_rx,) = struct.unpack_from("<H", self.buf, end)
            del self.buf[: end + 2]
            if binascii.crc_hqx(body, 0xFFFF) != crc_rx:
                self.crc_errors += 1
                continue
            return typ, seq, body[5:]

    def _request(self, typ, payload, rsp_type):
        self.seq = (self.seq + 1) & 0xFFFF
        seq = self.seq
        self._send_frame(typ, seq, payload)

        deadline = time.monotonic() + self.timeout
        while True:
            rtype, rseq, rpayload = self._read_frame(deadline)
            if rtype == rsp_type and rseq == seq:
                return rpayload
            # Stale reply from an earlier timeout; keep waiting.

    def status(self):
        payload = self._request(CMD_STATUS, b"", RSP_STATUS)
        if len(payload) != 40:
            raise ProtocolError(f"unexpected STATUS length {len(payload)}")
        return dict(zip(STATUS_FIELDS, struct.unpack("<10I", payload)))

    def _write_once(self, data):
        payload = self._request(CMD_WRITE, data, RSP_ACK)
        if len(payload) != 12:
            raise ProtocolError(f"unexpected ACK length {len(payload)}")
        return struct.unpack("<III", payload)

    def write(self, data, chunk=FRAME_MAX_PAYLOAD, stall_timeout=10.0):
        offset = 0
        last_progress = time.monotonic()

        while offset < len(data):
            piece = data[offset : offset + chunk]
            accepted, _used, _free = self._write_once(piece)
            if accepted > len(piece):
                raise ProtocolError(
                    f"device accepted {accepted} of {len(piece)} bytes"
                )
            offset += accepted
            if accepted:
                last_progress = time.monotonic()
            if accepted == len(piece):
                continue
            if time.monotonic() - last_progress > stall_timeout:
                raise TimeoutError(
                    "device ring stayed full; is the SPI master clocking data out?"
                )
            missing = len(piece) - accepted
            time.sleep(min(max(missing / DRAIN_RATE, 0.01), 0.25))


def print_status(st):
    for name in STATUS_FIELDS:
        value = st[name]
        if name in ("capacity", "used", "free", "stage_ahead", "spi_consumed", "zero_fill"):
            print(f"  {name:<16} {value:10d} B  ({value / 1024:.1f} KiB)")
        else:
            print(f"  {name:<16} {value:10d}")


def load_file(bridge, path, chunk, stall_timeout, quiet):
    if path == "-":
        f = sys.stdin.buffer
        total_size = None
    else:
        f = open(path, "rb")
        total_size = os.fstat(f.fileno()).st_size

    total = 0
    started = time.monotonic()
    last_update = 0.0
    try:
        while True:
            block = f.read(chunk)
            if not block:
                break
            bridge.write(block, chunk=chunk, stall_timeout=stall_timeout)
            total += len(block)

            now = time.monotonic()
            if not quiet and now - last_update >= 0.2:
                last_update = now
                rate = total / max(now - started, 1e-9)
                if total_size:
                    msg = f"\r{total}/{total_size} B ({100.0 * total / total_size:5.1f}%)  {rate / 1024:.1f} KiB/s"
                else:
                    msg = f"\r{total} B  {rate / 1024:.1f} KiB/s"
                print(msg, end="", file=sys.stderr, flush=True)
    finally:
        if f is not sys.stdin.buffer:
            f.close()

    if not quiet:
        print(file=sys.stderr)
    return total


def main():
    ap = argparse.ArgumentParser(
        description="Load data into the RP2350 USB-to-SPI-slave bridge.",
        formatter_class=argparse.RawDescriptionHelpFormatter,
    )
    ap.add_argument("file", nargs="?", help="binary file to preload, or '-' for stdin")
    ap.add_argument("-p", "--port", help="USB CDC port (default: auto-detect 2e8a:0009)")
    ap.add_argument("-b", "--baud", type=int, default=115200,
                    help="ignored by USB CDC (default: 115200)")
    ap.add_argument("-c", "--chunk", type=int, default=FRAME_MAX_PAYLOAD,
                    help=f"WRITE payload size, max {FRAME_MAX_PAYLOAD} (default: {FRAME_MAX_PAYLOAD})")
    ap.add_argument("-s", "--status", action="store_true",
                    help="print bridge counters (also printed after loading)")
    ap.add_argument("-q", "--quiet", action="store_true", help="suppress progress and status")
    ap.add_argument("-w", "--stall-timeout", type=float, default=10.0,
                    help="seconds to tolerate a full ring without progress (default: 10)")
    args = ap.parse_args()

    if not 1 <= args.chunk <= FRAME_MAX_PAYLOAD:
        ap.error(f"--chunk must be 1..{FRAME_MAX_PAYLOAD}")
    if not args.file and not args.status:
        ap.error("nothing to do: pass a file and/or --status")

    port = args.port or find_port()
    with Bridge(port, args.baud) as bridge:
        try:
            if args.file:
                if not args.quiet:
                    print(f"Loading {args.file} into {port} ...", file=sys.stderr)
                total = load_file(bridge, args.file, args.chunk, args.stall_timeout, args.quiet)
                if not args.quiet:
                    print(f"Loaded {total} bytes.", file=sys.stderr)
            if args.status or not args.quiet:
                print("Bridge status:")
                print_status(bridge.status())
        except (ProtocolError, TimeoutError) as exc:
            sys.exit(f"error: {exc}")


if __name__ == "__main__":
    main()
