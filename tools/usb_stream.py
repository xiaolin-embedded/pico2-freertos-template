#!/usr/bin/env python3
"""Load data into the RP2350 USB-to-SPI-slave bridge.

Packet format:
    0xAA | chksum | len_lo | len_hi | payload[len] | 0x55

chksum makes the 8-bit sum of the complete packet zero. payload[0] is the
command; payload[1] is reserved in requests and holds the status in responses.

Host -> device:
    0x01 WRITE_DATA      seq[2], offset[4], data[N]  (N % 64 == 0, N <= 960)
    0x02 GET_STATUS
    0x03 CLEAR_COUNTERS
    0x04 GET_VERSION

WRITE_DATA response:
    seq[2], offset[4], accepted[2], stream_accepted[4], free[4], err[2]
err is packed high byte first (type byte, then code byte); the values are
bridge-specific.

The SPI master drains 64 bytes every millisecond (64,000 B/s). The device
ring holds 384 KiB (~6 s). WRITE_DATA is flow-controlled: when the ring is
full the device returns ERR_RING_FULL plus the accepted byte count, and the
loader retries the unaccepted tail. The offset makes a retry after a lost
ACK idempotent.
"""

import argparse
import struct
import sys
import time

try:
    import serial
    from serial.tools import list_ports
except ImportError:
    sys.exit("pyserial is required: pip install pyserial")

MAGIC_HEAD = 0xAA
MAGIC_REAR = 0x55
MAX_PACKET = 1024
MAX_PAYLOAD = MAX_PACKET - 5
FRAME_BYTES = 64
WRITE_MAX_DATA = 15 * FRAME_BYTES
WRITE_HEADER_SIZE = 8

CMD_WRITE_DATA = 0x01
CMD_GET_STATUS = 0x02
CMD_CLEAR_COUNTERS = 0x03
CMD_GET_VERSION = 0x04

STATUS_SUCCESS = 0x01
STATUS_BUSY = 0x02
STATUS_ERROR = 0x03

ERR_SUCCESS = 0x0000
ERR_INVALID_LENGTH = 0x0101
ERR_UNSUPPORT = 0x0104
ERR_CHECKSUM = 0x0106
ERR_RING_FULL = 0x0201
ERR_NOT_ALIGNED = 0x0202
ERR_OFFSET = 0x0203
ERR_UNEXPECT = 0xFFFF

ERR_NAMES = {
    ERR_SUCCESS: "success",
    ERR_INVALID_LENGTH: "invalid length",
    ERR_UNSUPPORT: "unsupported command",
    ERR_CHECKSUM: "checksum error",
    ERR_RING_FULL: "ring full",
    ERR_NOT_ALIGNED: "write not 64-byte aligned",
    ERR_OFFSET: "stream offset mismatch",
    ERR_UNEXPECT: "unexpected error",
}

USB_VID_PID = (0x2E8A, 0x0009)  # pico_stdio_usb on RP2350

STATUS_FIELDS = (
    "capacity",
    "used",
    "free",
    "stage_ahead",
    "spi_consumed",
    "zero_fill",
    "checksum_errors",
    "bad_packets",
    "ring_full_events",
    "underrun_events",
    "packets_ok",
    "stream_accepted",
    "stream_offset",
)


class ProtocolError(Exception):
    pass


class PacketTimeout(Exception):
    pass


def make_packet(payload):
    if len(payload) > MAX_PAYLOAD:
        raise ValueError("payload too large")
    packet = bytearray(5 + len(payload))
    packet[0] = MAGIC_HEAD
    packet[1] = 0
    packet[2] = len(payload) & 0xFF
    packet[3] = (len(payload) >> 8) & 0xFF
    packet[4:4 + len(payload)] = payload
    packet[-1] = MAGIC_REAR
    packet[1] = (-sum(packet)) & 0xFF
    return bytes(packet)


def err_name(err):
    return ERR_NAMES.get(err, "error 0x%04X" % err)


class PacketReader:
    def __init__(self, ser):
        self.ser = ser
        self.buf = bytearray()

    def _fill(self, deadline):
        remaining = deadline - time.monotonic()
        if remaining <= 0:
            raise PacketTimeout("timed out waiting for a packet")
        self.ser.timeout = min(0.05, remaining)
        data = self.ser.read(max(1, self.ser.in_waiting))
        if data:
            self.buf.extend(data)

    def read(self, timeout):
        deadline = time.monotonic() + timeout
        while True:
            while self.buf and self.buf[0] != MAGIC_HEAD:
                del self.buf[0]

            if len(self.buf) >= 5:
                payload_len = self.buf[2] | (self.buf[3] << 8)
                if payload_len > MAX_PAYLOAD:
                    del self.buf[0]  # false sync
                    continue
                total = payload_len + 5
                if len(self.buf) >= total:
                    packet = bytes(self.buf[:total])
                    if packet[-1] == MAGIC_REAR and (sum(packet) & 0xFF) == 0:
                        del self.buf[:total]
                        return packet[4:4 + payload_len]
                    del self.buf[0]  # checksum or rear mismatch; resync
                    continue

            self._fill(deadline)


class Bridge:
    def __init__(self, port, baud=115200, timeout=2.0, stall_timeout=10.0,
                 retries=5):
        self.ser = serial.Serial(port, baud, timeout=0.05)
        self.ser.reset_input_buffer()
        self.reader = PacketReader(self.ser)
        self.timeout = timeout
        self.stall_timeout = stall_timeout
        self.retries = retries
        self.seq = 0
        self.offset = 0
        self.last_free = 0

    def close(self):
        self.ser.close()

    def _request(self, payload, expect, timeout=None):
        self.ser.write(make_packet(payload))
        deadline = time.monotonic() + (self.timeout if timeout is None else timeout)
        while True:
            remaining = deadline - time.monotonic()
            if remaining <= 0:
                raise PacketTimeout("no response for command 0x%02X" % expect)
            response = self.reader.read(remaining)
            if response and response[0] == expect:
                return response

    def status(self):
        response = self._request(bytes([CMD_GET_STATUS, 0]), CMD_GET_STATUS)
        if len(response) != 2 + 52 or response[1] != STATUS_SUCCESS:
            raise ProtocolError("bad GET_STATUS response")
        values = struct.unpack_from("<13I", response, 2)
        return dict(zip(STATUS_FIELDS, values))

    def version(self):
        response = self._request(bytes([CMD_GET_VERSION, 0]), CMD_GET_VERSION)
        if len(response) != 6 or response[1] != STATUS_SUCCESS:
            raise ProtocolError("bad GET_VERSION response")
        return tuple(response[2:6])

    def clear_counters(self):
        response = self._request(bytes([CMD_CLEAR_COUNTERS, 0]),
                                CMD_CLEAR_COUNTERS)
        if len(response) != 2 or response[1] != STATUS_SUCCESS:
            raise ProtocolError("bad CLEAR_COUNTERS response")

    def write_chunk(self, offset, data):
        seq = self.seq
        self.seq = (self.seq + 1) & 0xFFFF
        payload = struct.pack("<BBHI", CMD_WRITE_DATA, 0, seq, offset) + data
        response = self._request(payload, CMD_WRITE_DATA)

        if len(response) != 20:
            raise ProtocolError("bad WRITE_DATA response length")
        rcmd, rstatus, rseq, roffset, accepted, stream_accepted, free_bytes = \
            struct.unpack_from("<BBHIHII", response, 0)
        err = (response[18] << 8) | response[19]
        if rcmd != CMD_WRITE_DATA or rseq != seq or roffset != offset:
            raise ProtocolError("WRITE_DATA response mismatch")
        if rstatus == STATUS_ERROR:
            pass
        elif rstatus != STATUS_SUCCESS:
            raise ProtocolError("unexpected WRITE_DATA status 0x%02X" % rstatus)
        self.last_free = free_bytes
        return accepted, stream_accepted, free_bytes, err

    def write_data(self, data, chunk=WRITE_MAX_DATA, stall_timeout=None):
        if len(data) % FRAME_BYTES:
            raise ValueError("data length must be a multiple of 64")
        if (chunk < FRAME_BYTES or chunk > WRITE_MAX_DATA or
                chunk % FRAME_BYTES):
            raise ValueError("chunk must be 64..960 and a multiple of 64")

        stall_timeout = (self.stall_timeout if stall_timeout is None
                         else stall_timeout)
        pos = 0
        last_progress = time.monotonic()
        retries = 0

        while pos < len(data):
            piece_len = min(chunk, len(data) - pos)
            piece = data[pos:pos + piece_len]
            try:
                accepted, stream_accepted, free_bytes, err = \
                    self.write_chunk(self.offset, piece)
            except PacketTimeout:
                # Retry the same offset. If the first copy was queued, the
                # device answers ERR_OFFSET instead of appending it again.
                retries += 1
                if retries > self.retries:
                    raise
                continue

            retries = 0
            if accepted > piece_len:
                raise ProtocolError("device accepted more than sent")

            if err == ERR_SUCCESS:
                if accepted != piece_len:
                    raise ProtocolError(
                        "short write without error: %d/%d" %
                        (accepted, piece_len))
                self.offset = (self.offset + accepted) & 0xFFFFFFFF
                pos += accepted
                last_progress = time.monotonic()

            elif err == ERR_RING_FULL:
                self.offset = (self.offset + accepted) & 0xFFFFFFFF
                pos += accepted
                if accepted:
                    last_progress = time.monotonic()
                if time.monotonic() - last_progress > stall_timeout:
                    raise PacketTimeout(
                        "ring stalled; is the SPI master clocking?")
                time.sleep(0.001)

            elif err == ERR_OFFSET:
                delta = (stream_accepted - self.offset) & 0xFFFFFFFF
                if delta == 0:
                    # Nothing was queued by the timed-out request.
                    if time.monotonic() - last_progress > stall_timeout:
                        raise PacketTimeout(
                            "ring stalled; is the SPI master clocking?")
                    time.sleep(0.001)
                elif delta <= piece_len:
                    # The timed-out request was (partly) queued already.
                    self.offset = stream_accepted
                    pos += delta
                    last_progress = time.monotonic()
                else:
                    raise ProtocolError(
                        "device offset 0x%08X does not match host 0x%08X" %
                        (stream_accepted, self.offset))

            else:
                raise ProtocolError(
                    "WRITE failed: %s (accepted %d, free %d)" %
                    (err_name(err), accepted, free_bytes))

        return self.offset

    def send_chunk(self, data):
        """Queue one 64-byte-aligned chunk (<= 960 B) completely."""
        if not data or len(data) % FRAME_BYTES or len(data) > WRITE_MAX_DATA:
            raise ValueError("chunk must be 64..960 bytes, multiple of 64")
        self.write_data(data, chunk=len(data))


def find_port():
    ports = list(list_ports.comports())
    matches = [p.device for p in ports if (p.vid, p.pid) == USB_VID_PID]
    if matches:
        return matches[0]
    available = ", ".join(p.device for p in ports) or "none"
    sys.exit("USB CDC device 2e8a:0009 not found (available: %s)" % available)


def print_status(status):
    for name in STATUS_FIELDS:
        print("%-18s %u" % (name, status[name]))


def load_file(bridge, path, chunk):
    padded = 0
    with open(path, "rb") as stream:
        while True:
            block = stream.read(64 * 1024)
            if not block:
                break
            if len(block) % FRAME_BYTES:
                pad = FRAME_BYTES - (len(block) % FRAME_BYTES)
                block += bytes(pad)
                padded += pad
            bridge.write_data(block, chunk=chunk)
    if padded:
        print("warning: padded %d zero bytes to keep 64-byte frames" % padded,
              file=sys.stderr)


def load_stdin(bridge, chunk):
    pending = b""
    while True:
        block = sys.stdin.buffer.read(64 * 1024)
        if not block:
            break
        pending += block
        while len(pending) >= 64 * 1024:
            bridge.write_data(pending[:64 * 1024], chunk=chunk)
            pending = pending[64 * 1024:]
    if pending:
        if len(pending) % FRAME_BYTES:
            pending += bytes(FRAME_BYTES - (len(pending) % FRAME_BYTES))
        bridge.write_data(pending, chunk=chunk)


def print_stream_report(elapsed, status):
    print("[%8.1fs] used=%u free=%u stage=%u spi=%u zero=%u full=%u "
          "under=%u crc=%u bad=%u stream=%u off=%u" %
          (elapsed, status["used"], status["free"], status["stage_ahead"],
           status["spi_consumed"], status["zero_fill"],
           status["ring_full_events"], status["underrun_events"],
           status["checksum_errors"], status["bad_packets"],
           status["stream_accepted"], status["stream_offset"]),
          file=sys.stderr, flush=True)


def pattern_source(chunk):
    per_chunk = chunk // FRAME_BYTES
    frame = 0

    def next_chunk():
        nonlocal frame
        block = bytearray(per_chunk * FRAME_BYTES)
        for i in range(per_chunk):
            value = (frame + i) & 0xFF
            block[i * FRAME_BYTES:(i + 1) * FRAME_BYTES] = \
                bytes([value]) * FRAME_BYTES
        frame += per_chunk
        return bytes(block)

    return next_chunk


def file_loop_source(stream, chunk):
    def next_chunk():
        data = stream.read(chunk)
        if not data:
            stream.seek(0)
            data = stream.read(chunk)
        if not data:
            raise ProtocolError("cannot loop an empty file")
        if len(data) % FRAME_BYTES:
            data += bytes(FRAME_BYTES - (len(data) % FRAME_BYTES))
        return data

    return next_chunk


def stream_continuous(bridge, next_chunk, chunk, capacity, initial_used,
                      watermark_ms, report_s, duration_s):
    """Feed an endless source, keeping the device ring near a target level."""
    target = max(watermark_ms * 64, chunk) if watermark_ms > 0 else 0
    if target > capacity:
        target = capacity
    used = initial_used

    if target:
        while used < target:
            block = next_chunk()
            bridge.send_chunk(block)
            used = (capacity - bridge.last_free
                    if bridge.last_free <= capacity else 0)
        print("preloaded %u bytes (target %u); streaming continuously" %
              (used, target), file=sys.stderr, flush=True)

    start = time.monotonic()
    last_report = start
    while True:
        now = time.monotonic()
        if duration_s > 0 and now - start >= duration_s:
            break
        if report_s > 0 and now - last_report >= report_s:
            status = bridge.status()
            print_stream_report(now - start, status)
            used = status["used"]
            last_report = now
        if target and used > target:
            wait_ms = min((used - target) // 64, 100)
            if wait_ms < 1:
                wait_ms = 1
            time.sleep(wait_ms / 1000.0)
            used = max(0, used - wait_ms * 64)
            continue
        block = next_chunk()
        bridge.send_chunk(block)
        used = (capacity - bridge.last_free
                if bridge.last_free <= capacity else 0)


def load_pattern(bridge, frames, chunk):
    """Send 64-byte frames; frame N is filled with byte (N & 0xFF)."""
    per_chunk = chunk // FRAME_BYTES
    frame = 0
    last_report = 0
    if frames == 0:
        print("sending endless pattern; Ctrl-C to stop", file=sys.stderr)
    while frames == 0 or frame < frames:
        count = per_chunk if frames == 0 else min(per_chunk, frames - frame)
        block = bytearray(count * FRAME_BYTES)
        for i in range(count):
            value = (frame + i) & 0xFF
            block[i * FRAME_BYTES:(i + 1) * FRAME_BYTES] = bytes([value]) * FRAME_BYTES
        bridge.write_data(bytes(block), chunk=chunk)
        frame += count
        if frame - last_report >= 1024 or (frames != 0 and frame == frames):
            print("\r%u frames sent" % frame, end="", file=sys.stderr, flush=True)
            last_report = frame
    print(file=sys.stderr)


def main():
    parser = argparse.ArgumentParser(
        description="Load data or a test pattern into the Pico 2 USB-to-SPI bridge")
    parser.add_argument("file", nargs="?",
                        help="binary file to preload, or '-' for stdin")
    parser.add_argument("-p", "--port",
                        help="USB CDC port (default: auto-detect 2e8a:0009)")
    parser.add_argument("-b", "--baud", type=int, default=115200,
                        help="ignored by USB CDC (default: 115200)")
    parser.add_argument("-c", "--chunk", type=int, default=WRITE_MAX_DATA,
                        help="WRITE payload bytes, multiple of 64, max 960")
    parser.add_argument("--pattern", action="store_true",
                        help="send test frames: frame index N (from 0) is 64 x (N & 0xFF)")
    parser.add_argument("--frames", type=int, default=None,
                        help="number of frames for --pattern (omit or 0 = continuous)")
    parser.add_argument("--loop", action="store_true",
                        help="replay the input file continuously")
    parser.add_argument("--watermark-ms", type=int, default=200,
                        help="continuous mode target ring level in ms (0 = no throttle)")
    parser.add_argument("--report", type=float, default=5.0,
                        help="seconds between status reports in continuous mode (0 = off)")
    parser.add_argument("--duration", type=float, default=0.0,
                        help="seconds to stream in continuous mode (0 = until Ctrl-C)")
    parser.add_argument("-s", "--status", action="store_true",
                        help="print device status and exit")
    parser.add_argument("--version", action="store_true",
                        help="print protocol/firmware version and exit")
    parser.add_argument("--clear-counters", action="store_true",
                        help="clear error counters before loading")
    parser.add_argument("--start-offset", type=lambda v: int(v, 0),
                        help="stream offset (default: continue at device offset)")
    parser.add_argument("--stall-timeout", type=float, default=10.0,
                        help="seconds without ring progress before giving up")
    parser.add_argument("--retries", type=int, default=5,
                        help="retries after a response timeout")

    args = parser.parse_args()

    if (args.chunk < FRAME_BYTES or args.chunk > WRITE_MAX_DATA or
            args.chunk % FRAME_BYTES):
        parser.error("--chunk must be 64..960 and a multiple of 64")
    if args.frames is not None and not args.pattern:
        parser.error("--frames requires --pattern")
    if args.frames is not None and args.frames < 0:
        parser.error("--frames must be >= 0")
    if args.pattern and args.file:
        parser.error("--pattern does not take a file")
    if args.loop and args.pattern:
        parser.error("--loop cannot be combined with --pattern")
    if args.loop and not args.file:
        parser.error("--loop requires a file")
    if args.loop and args.file == "-":
        parser.error("--loop requires a seekable file")
    if args.watermark_ms < 0:
        parser.error("--watermark-ms must be >= 0")
    if args.report < 0:
        parser.error("--report must be >= 0")
    if args.duration < 0:
        parser.error("--duration must be >= 0")

    port = args.port or find_port()
    if args.port is None:
        print("using %s" % port, file=sys.stderr)
    bridge = Bridge(port, args.baud, stall_timeout=args.stall_timeout,
                    retries=args.retries)
    try:
        if args.version:
            major, minor, fw_major, fw_minor = bridge.version()
            print("protocol %d.%d, firmware %d.%d" %
                  (major, minor, fw_major, fw_minor))
            return

        if args.clear_counters:
            bridge.clear_counters()
            print("counters cleared")

        status = bridge.status()
        if args.status or (not args.file and not args.pattern):
            print_status(status)
            return

        bridge.offset = (status["stream_accepted"]
                         if args.start_offset is None else args.start_offset)
        print("device stream offset 0x%08X, ring %u free" %
              (bridge.offset, status["free"]), file=sys.stderr)

        continuous_pattern = (args.pattern and
                              (args.frames is None or args.frames == 0))
        continuous_file = bool(args.file) and args.loop

        try:
            if continuous_pattern:
                stream_continuous(
                    bridge, pattern_source(args.chunk), args.chunk,
                    status["capacity"], status["used"], args.watermark_ms,
                    args.report, args.duration)
            elif continuous_file:
                with open(args.file, "rb") as stream:
                    stream_continuous(
                        bridge, file_loop_source(stream, args.chunk),
                        args.chunk, status["capacity"], status["used"],
                        args.watermark_ms, args.report, args.duration)
            elif args.pattern:
                load_pattern(bridge, args.frames, args.chunk)
            elif args.file == "-":
                load_stdin(bridge, args.chunk)
            else:
                load_file(bridge, args.file, args.chunk)
        except KeyboardInterrupt:
            print("\ninterrupted", file=sys.stderr)
            try:
                print_status(bridge.status())
            except Exception:
                pass
            return

        print_status(bridge.status())
    finally:
        bridge.close()


if __name__ == "__main__":
    try:
        main()
    except (ProtocolError, PacketTimeout) as exc:
        sys.exit("error: %s" % exc)
    except KeyboardInterrupt:
        sys.exit(130)
