// C++17 host stream tool for the RP2350 USB -> SPI-slave bridge (Linux/POSIX).
//
// Packet format:
//   0xAA | chksum | len_lo | len_hi | payload[len] | 0x55
// chksum makes the 8-bit sum of the complete packet zero.
//
// Build: g++ -std=c++17 -O2 -Wall -Wextra -Werror -o usb_stream usb_stream.cpp
// Usage: ./usb_stream /dev/ttyACM0 data.bin
//        ./usb_stream /dev/ttyACM0 --status

#include <algorithm>
#include <array>
#include <cerrno>
#include <chrono>
#include <cstdint>
#include <cstring>
#include <fcntl.h>
#include <fstream>
#include <iostream>
#include <poll.h>
#include <stdexcept>
#include <string>
#include <termios.h>
#include <thread>
#include <unistd.h>
#include <utility>
#include <vector>

namespace {

using Clock = std::chrono::steady_clock;

constexpr uint8_t kHead = 0xAA;
constexpr uint8_t kRear = 0x55;
constexpr size_t kMaxPacket = 1024;
constexpr size_t kMaxPayload = kMaxPacket - 5;
constexpr size_t kFrameBytes = 64;
constexpr size_t kWriteMaxData = 15 * kFrameBytes;
constexpr size_t kWriteHeader = 8;

constexpr uint8_t kCmdWrite = 0x01;
constexpr uint8_t kCmdStatus = 0x02;
constexpr uint8_t kCmdClearCounters = 0x03;
constexpr uint8_t kCmdVersion = 0x04;

constexpr uint8_t kStatusSuccess = 0x01;
constexpr uint8_t kStatusError = 0x03;

constexpr uint16_t kErrSuccess = 0x0000;
constexpr uint16_t kErrInvalidLength = 0x0101;
constexpr uint16_t kErrUnsupported = 0x0104;
constexpr uint16_t kErrRingFull = 0x0201;
constexpr uint16_t kErrNotAligned = 0x0202;
constexpr uint16_t kErrOffset = 0x0203;

struct TimeoutError : std::runtime_error {
    using std::runtime_error::runtime_error;
};

uint16_t le16(const uint8_t *p) {
    return static_cast<uint16_t>(p[0] | (p[1] << 8));
}

uint32_t le32(const uint8_t *p) {
    return static_cast<uint32_t>(p[0]) |
           (static_cast<uint32_t>(p[1]) << 8) |
           (static_cast<uint32_t>(p[2]) << 16) |
           (static_cast<uint32_t>(p[3]) << 24);
}

void put16(std::vector<uint8_t> &v, uint16_t value) {
    v.push_back(static_cast<uint8_t>(value));
    v.push_back(static_cast<uint8_t>(value >> 8));
}

void put32(std::vector<uint8_t> &v, uint32_t value) {
    v.push_back(static_cast<uint8_t>(value));
    v.push_back(static_cast<uint8_t>(value >> 8));
    v.push_back(static_cast<uint8_t>(value >> 16));
    v.push_back(static_cast<uint8_t>(value >> 24));
}

std::vector<uint8_t> make_packet(const std::vector<uint8_t> &payload) {
    if (payload.size() > kMaxPayload) {
        throw std::invalid_argument("payload too large");
    }
    std::vector<uint8_t> p;
    p.reserve(payload.size() + 5);
    p.push_back(kHead);
    p.push_back(0);
    put16(p, static_cast<uint16_t>(payload.size()));
    p.insert(p.end(), payload.begin(), payload.end());
    p.push_back(kRear);
    uint8_t sum = 0;
    for (uint8_t b : p) {
        sum = static_cast<uint8_t>(sum + b);
    }
    p[1] = static_cast<uint8_t>(0u - sum);
    return p;
}

const char *err_name(uint16_t err) {
    switch (err) {
    case kErrSuccess: return "success";
    case kErrInvalidLength: return "invalid length";
    case kErrUnsupported: return "unsupported command";
    case kErrRingFull: return "ring full";
    case kErrNotAligned: return "write not 64-byte aligned";
    case kErrOffset: return "stream offset mismatch";
    default: return "error";
    }
}

int open_port(const std::string &path) {
    int fd = ::open(path.c_str(), O_RDWR | O_NOCTTY | O_CLOEXEC);
    if (fd < 0) {
        throw std::runtime_error("open " + path + ": " + std::strerror(errno));
    }
    termios tio{};
    if (::tcgetattr(fd, &tio) != 0) {
        int err = errno;
        ::close(fd);
        throw std::runtime_error(std::string("tcgetattr: ") + std::strerror(err));
    }
    ::cfmakeraw(&tio);
    tio.c_cflag |= CLOCAL | CREAD;
    tio.c_cflag &= ~CRTSCTS;
    tio.c_cc[VMIN] = 0;
    tio.c_cc[VTIME] = 0;
    // CDC ACM ignores this line-coding value; termios requires a valid speed.
    ::cfsetispeed(&tio, B115200);
    ::cfsetospeed(&tio, B115200);
    if (::tcsetattr(fd, TCSANOW, &tio) != 0) {
        int err = errno;
        ::close(fd);
        throw std::runtime_error(std::string("tcsetattr: ") + std::strerror(err));
    }
    ::tcflush(fd, TCIOFLUSH);
    return fd;
}

class Bridge {
public:
    explicit Bridge(const std::string &path, int timeout_ms = 2000,
                    int stall_ms = 10000, int retries = 5)
        : fd_(open_port(path)), timeout_ms_(timeout_ms), stall_ms_(stall_ms),
          retries_(retries) {}

    ~Bridge() {
        if (fd_ >= 0) {
            ::close(fd_);
        }
    }

    Bridge(const Bridge &) = delete;
    Bridge &operator=(const Bridge &) = delete;

    struct Status {
        uint32_t capacity, used, free, stage_ahead, spi_consumed;
        uint32_t zero_fill, checksum_errors, bad_packets, ring_full_events;
        uint32_t underrun_events, packets_ok, stream_accepted, stream_offset;
    };

    struct Version {
        uint8_t protocol_major, protocol_minor, fw_major, fw_minor;
    };

    void set_offset(uint32_t offset) { offset_ = offset; }
    uint32_t offset() const { return offset_; }

    Status status() {
        std::vector<uint8_t> p = transact({kCmdStatus, 0}, kCmdStatus);
        if (p.size() != 54 || p[1] != kStatusSuccess) {
            throw std::runtime_error("bad GET_STATUS response");
        }
        const uint8_t *d = p.data() + 2;
        return {le32(d + 0),  le32(d + 4),  le32(d + 8),  le32(d + 12),
                le32(d + 16), le32(d + 20), le32(d + 24), le32(d + 28),
                le32(d + 32), le32(d + 36), le32(d + 40), le32(d + 44),
                le32(d + 48)};
    }

    Version version() {
        std::vector<uint8_t> p = transact({kCmdVersion, 0}, kCmdVersion);
        if (p.size() != 6 || p[1] != kStatusSuccess) {
            throw std::runtime_error("bad GET_VERSION response");
        }
        return {p[2], p[3], p[4], p[5]};
    }

    void clear_counters() {
        std::vector<uint8_t> p =
            transact({kCmdClearCounters, 0}, kCmdClearCounters);
        if (p.size() != 2 || p[1] != kStatusSuccess) {
            throw std::runtime_error("bad CLEAR_COUNTERS response");
        }
    }

    // Push a buffer whose size is a multiple of 64. Retries partial writes
    // and timeout retries; the stream offset makes a retry idempotent.
    void write_data(const uint8_t *data, size_t len, size_t chunk) {
        if (len % kFrameBytes != 0) {
            throw std::invalid_argument("length must be a multiple of 64");
        }
        if (chunk < kFrameBytes || chunk > kWriteMaxData ||
            chunk % kFrameBytes != 0) {
            throw std::invalid_argument("chunk must be 64..960");
        }

        size_t pos = 0;
        auto last_progress = Clock::now();
        int retries = 0;

        while (pos < len) {
            const size_t piece_len = std::min(chunk, len - pos);
            WriteRsp rsp;
            try {
                rsp = write_chunk(offset_, data + pos, piece_len);
            } catch (const TimeoutError &) {
                if (++retries > retries_) {
                    throw;
                }
                continue;
            }
            retries = 0;
            if (rsp.accepted > piece_len) {
                throw std::runtime_error("device accepted more than sent");
            }

            if (rsp.err == kErrSuccess) {
                if (rsp.accepted != piece_len) {
                    throw std::runtime_error("short write without error");
                }
                offset_ += rsp.accepted;
                pos += rsp.accepted;
                last_progress = Clock::now();
            } else if (rsp.err == kErrRingFull) {
                offset_ += rsp.accepted;
                pos += rsp.accepted;
                if (rsp.accepted > 0) {
                    last_progress = Clock::now();
                }
                if (Clock::now() - last_progress >
                    std::chrono::milliseconds(stall_ms_)) {
                    throw TimeoutError(
                        "ring stalled; is the SPI master clocking?");
                }
                std::this_thread::sleep_for(std::chrono::milliseconds(1));
            } else if (rsp.err == kErrOffset) {
                const uint32_t delta = rsp.stream_accepted - offset_;
                if (delta == 0) {
                    if (Clock::now() - last_progress >
                        std::chrono::milliseconds(stall_ms_)) {
                        throw TimeoutError(
                            "ring stalled; is the SPI master clocking?");
                    }
                    std::this_thread::sleep_for(std::chrono::milliseconds(1));
                } else if (delta <= piece_len) {
                    // A timed-out request was (partly) queued already.
                    offset_ = rsp.stream_accepted;
                    pos += delta;
                    last_progress = Clock::now();
                } else {
                    throw std::runtime_error(
                        "device offset does not match the host");
                }
            } else {
                throw std::runtime_error(
                    std::string("WRITE failed: ") + err_name(rsp.err) +
                    " (accepted " + std::to_string(rsp.accepted) + ")");
            }
        }
    }

private:
    struct WriteRsp {
        uint16_t accepted = 0;
        uint32_t stream_accepted = 0;
        uint16_t err = 0xFFFF;
    };

    std::vector<uint8_t> transact(const std::vector<uint8_t> &payload,
                                  uint8_t expect) {
        const std::vector<uint8_t> packet = make_packet(payload);
        write_all(packet.data(), packet.size());

        const auto deadline =
            Clock::now() + std::chrono::milliseconds(timeout_ms_);
        std::vector<uint8_t> response;
        while (true) {
            if (!read_packet(response, deadline)) {
                throw TimeoutError("no response for command");
            }
            if (!response.empty() && response[0] == expect) {
                return response;
            }
        }
    }

    WriteRsp write_chunk(uint32_t offset, const uint8_t *data, size_t len) {
        const uint16_t seq = seq_++;
        std::vector<uint8_t> payload;
        payload.reserve(8 + len);
        payload.push_back(kCmdWrite);
        payload.push_back(0);
        put16(payload, seq);
        put32(payload, offset);
        payload.insert(payload.end(), data, data + len);

        const std::vector<uint8_t> p =
            transact(payload, kCmdWrite);
        if (p.size() != 20 || p[1] != kStatusSuccess) {
            if (p.size() == 20 && p[1] == kStatusError) {
                // Report the error below.
            } else {
                throw std::runtime_error("bad WRITE_DATA response");
            }
        }

        WriteRsp rsp;
        rsp.accepted = le16(&p[8]);
        rsp.stream_accepted = le32(&p[10]);
        rsp.err = static_cast<uint16_t>((p[18] << 8) | p[19]);
        const uint16_t rsp_seq = le16(&p[2]);
        const uint32_t rsp_offset = le32(&p[4]);
        if (rsp_seq != seq || rsp_offset != offset) {
            throw std::runtime_error("WRITE_DATA response mismatch");
        }
        return rsp;
    }

    bool read_packet(std::vector<uint8_t> &payload,
                     Clock::time_point deadline) {
        while (true) {
            size_t start = 0;
            while (start < rx_.size() && rx_[start] != kHead) {
                ++start;
            }
            if (start > 0) {
                rx_.erase(rx_.begin(), rx_.begin() + static_cast<long>(start));
            }

            if (rx_.size() >= 5) {
                const size_t len = static_cast<size_t>(rx_[2]) |
                                   (static_cast<size_t>(rx_[3]) << 8);
                if (len > kMaxPayload) {
                    rx_.erase(rx_.begin());
                    continue;
                }
                const size_t total = len + 5;
                if (rx_.size() >= total) {
                    uint8_t sum = 0;
                    for (size_t i = 0; i < total; ++i) {
                        sum = static_cast<uint8_t>(sum + rx_[i]);
                    }
                    if (rx_[total - 1] == kRear && sum == 0) {
                        payload.assign(rx_.begin() + 4,
                                       rx_.begin() + 4 +
                                           static_cast<long>(len));
                        rx_.erase(rx_.begin(),
                                  rx_.begin() + static_cast<long>(total));
                        return true;
                    }
                    rx_.erase(rx_.begin());
                    continue;
                }
            }

            if (!fill(deadline)) {
                return false;
            }
        }
    }

    bool fill(Clock::time_point deadline) {
        const auto now = Clock::now();
        if (now >= deadline) {
            return false;
        }
        const auto left = std::chrono::duration_cast<std::chrono::milliseconds>(
            deadline - now);
        pollfd pfd{fd_, POLLIN, 0};
        const int rc = ::poll(&pfd, 1, static_cast<int>(left.count()));
        if (rc < 0) {
            if (errno == EINTR) {
                return true;
            }
            throw std::runtime_error(std::string("poll: ") +
                                     std::strerror(errno));
        }
        if (rc == 0) {
            return false;
        }
        if (pfd.revents & (POLLERR | POLLHUP | POLLNVAL)) {
            throw std::runtime_error("USB device disconnected");
        }
        uint8_t temp[256];
        const ssize_t n = ::read(fd_, temp, sizeof temp);
        if (n < 0) {
            if (errno == EINTR || errno == EAGAIN) {
                return true;
            }
            throw std::runtime_error(std::string("read: ") +
                                     std::strerror(errno));
        }
        if (n == 0) {
            throw std::runtime_error("USB device disconnected");
        }
        rx_.insert(rx_.end(), temp, temp + n);
        return true;
    }

    void write_all(const uint8_t *p, size_t len) {
        size_t sent = 0;
        while (sent < len) {
            pollfd pfd{fd_, POLLOUT, 0};
            const int rc = ::poll(&pfd, 1, 2000);
            if (rc < 0) {
                if (errno == EINTR) {
                    continue;
                }
                throw std::runtime_error(std::string("poll: ") +
                                         std::strerror(errno));
            }
            if (rc == 0) {
                throw TimeoutError("USB write timeout");
            }
            const ssize_t n = ::write(fd_, p + sent, len - sent);
            if (n < 0) {
                if (errno == EINTR || errno == EAGAIN) {
                    continue;
                }
                throw std::runtime_error(std::string("write: ") +
                                         std::strerror(errno));
            }
            sent += static_cast<size_t>(n);
        }
    }

    int fd_;
    int timeout_ms_;
    int stall_ms_;
    int retries_;
    uint16_t seq_ = 0;
    uint32_t offset_ = 0;
    std::vector<uint8_t> rx_;
};

void print_status(const Bridge::Status &s) {
    std::cout << "capacity          " << s.capacity << "\n"
              << "used              " << s.used << "\n"
              << "free              " << s.free << "\n"
              << "stage_ahead       " << s.stage_ahead << "\n"
              << "spi_consumed      " << s.spi_consumed << "\n"
              << "zero_fill         " << s.zero_fill << "\n"
              << "checksum_errors   " << s.checksum_errors << "\n"
              << "bad_packets       " << s.bad_packets << "\n"
              << "ring_full_events  " << s.ring_full_events << "\n"
              << "underrun_events   " << s.underrun_events << "\n"
              << "packets_ok        " << s.packets_ok << "\n"
              << "stream_accepted   " << s.stream_accepted << "\n"
              << "stream_offset     " << s.stream_offset << "\n";
}

// Frame index N (starts at 0) is 64 copies of byte (N & 0xFF); 256 frames per cycle.
void write_pattern(Bridge &bridge, size_t chunk, uint64_t frames) {
    const size_t per_chunk = chunk / kFrameBytes;
    uint64_t frame = 0;
    uint64_t last_report = 0;
    std::vector<uint8_t> block;

    while (frames == 0 || frame < frames) {
        size_t count = per_chunk;
        if (frames != 0 && frames - frame < count) {
            count = static_cast<size_t>(frames - frame);
        }
        block.resize(count * kFrameBytes);
        for (size_t i = 0; i < count; ++i) {
            const uint8_t value =
                static_cast<uint8_t>((frame + i) & 0xFF);
            std::fill_n(block.begin() + static_cast<long>(i * kFrameBytes),
                        kFrameBytes, value);
        }
        bridge.write_data(block.data(), block.size(), chunk);
        frame += count;
        if (frame - last_report >= 1024 ||
            (frames != 0 && frame == frames)) {
            std::cerr << "\r" << frame << " frames sent" << std::flush;
            last_report = frame;
        }
    }
    std::cerr << "\n";
}

}  // namespace

int main(int argc, char **argv) {
    if (argc < 2) {
        std::cerr << "usage: " << argv[0]
                  << " <port> [file | --pattern] [--status] [--version]"
                     " [--clear-counters] [--chunk N] [--start-offset N]"
                     " [--frames N]\n";
        return 2;
    }

    try {
        Bridge bridge(argv[1]);
        bool status_only = false;
        bool show_version = false;
        bool clear_counters = false;
        bool pattern = false;
        bool frames_set = false;
        uint64_t frames = 0;
        std::string file;
        size_t chunk = kWriteMaxData;
        bool have_start_offset = false;
        uint32_t start_offset = 0;

        for (int i = 2; i < argc; ++i) {
            const std::string arg = argv[i];
            if (arg == "--status") {
                status_only = true;
            } else if (arg == "--version") {
                show_version = true;
            } else if (arg == "--clear-counters") {
                clear_counters = true;
            } else if (arg == "--pattern") {
                pattern = true;
            } else if (arg == "--frames" && i + 1 < argc) {
                const long long value = std::stoll(argv[++i]);
                if (value < 0) {
                    throw std::runtime_error("--frames must be >= 0");
                }
                frames = static_cast<uint64_t>(value);
                frames_set = true;
            } else if (arg == "--chunk" && i + 1 < argc) {
                chunk = std::stoul(argv[++i]);
            } else if (arg == "--start-offset" && i + 1 < argc) {
                start_offset = static_cast<uint32_t>(
                    std::stoul(argv[++i], nullptr, 0));
                have_start_offset = true;
            } else if (!arg.empty() && arg[0] == '-') {
                throw std::runtime_error("unknown option " + arg);
            } else {
                file = arg;
            }
        }

        if (chunk < kFrameBytes || chunk > kWriteMaxData ||
            chunk % kFrameBytes != 0) {
            throw std::runtime_error(
                "--chunk must be 64..960 and a multiple of 64");
        }
        if (frames_set && !pattern) {
            throw std::runtime_error("--frames requires --pattern");
        }
        if (pattern && !file.empty()) {
            throw std::runtime_error("--pattern does not take a file");
        }

        if (show_version) {
            const Bridge::Version v = bridge.version();
            std::cout << "protocol " << static_cast<int>(v.protocol_major)
                      << "." << static_cast<int>(v.protocol_minor)
                      << ", firmware " << static_cast<int>(v.fw_major)
                      << "." << static_cast<int>(v.fw_minor) << "\n";
            return 0;
        }

        if (clear_counters) {
            bridge.clear_counters();
            std::cout << "counters cleared\n";
        }

        Bridge::Status status = bridge.status();
        if (status_only || (file.empty() && !pattern)) {
            print_status(status);
            return 0;
        }

        bridge.set_offset(have_start_offset ? start_offset
                                            : status.stream_accepted);
        std::cerr << "device stream offset 0x" << std::hex
                  << bridge.offset() << std::dec << ", ring " << status.free
                  << " free\n";

        if (pattern) {
            write_pattern(bridge, chunk, frames);
            print_status(bridge.status());
            return 0;
        }

        std::ifstream file_in;
        std::istream *in = &std::cin;
        if (file != "-") {
            file_in.open(file, std::ios::binary);
            if (!file_in) {
                throw std::runtime_error("cannot open " + file);
            }
            in = &file_in;
        }

        std::array<uint8_t, 64 * 1024> buf{};
        while (true) {
            in->read(reinterpret_cast<char *>(buf.data()),
                     static_cast<std::streamsize>(buf.size()));
            const std::streamsize n = in->gcount();
            if (n > 0) {
                size_t len = static_cast<size_t>(n);
                if (len % kFrameBytes != 0) {
                    const size_t pad = kFrameBytes - len % kFrameBytes;
                    std::fill_n(buf.begin() + static_cast<long>(len), pad, 0);
                    len += pad;
                    std::cerr << "warning: padded " << pad
                              << " zero bytes\n";
                }
                bridge.write_data(buf.data(), len, chunk);
            }
            if (in->eof()) {
                break;
            }
            if (!*in) {
                throw std::runtime_error("read error");
            }
        }

        print_status(bridge.status());
    } catch (const std::exception &e) {
        std::cerr << "error: " << e.what() << "\n";
        return 1;
    }
    return 0;
}
