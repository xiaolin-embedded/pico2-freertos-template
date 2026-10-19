// C++17 host loader for the RP2350 USB -> SPI-slave bridge (Linux/POSIX).
// Build: g++ -std=c++17 -O2 -Wall -Wextra -o usb_bridge_loader usb_bridge_loader.cpp
// Usage: ./usb_bridge_loader /dev/ttyACM0 data.bin
//        ./usb_bridge_loader /dev/ttyACM0 --status

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

constexpr uint8_t kMagic0 = 0xAA;
constexpr uint8_t kMagic1 = 0x55;
constexpr uint8_t kCmdWrite = 0x01;
constexpr uint8_t kCmdStatus = 0x02;
constexpr uint8_t kRspAck = 0x81;
constexpr uint8_t kRspStatus = 0x82;
constexpr size_t kMaxPayload = 4096;
constexpr size_t kRecord = 64;          // one SPI transaction
constexpr size_t kDefaultChunk = 4096;  // 64 records

uint16_t crc16_update(uint16_t crc, const uint8_t *p, size_t n) {
    for (size_t i = 0; i < n; ++i) {
        crc ^= static_cast<uint16_t>(p[i]) << 8;
        for (int b = 0; b < 8; ++b) {
            crc = (crc & 0x8000) ? static_cast<uint16_t>((crc << 1) ^ 0x1021)
                                 : static_cast<uint16_t>(crc << 1);
        }
    }
    return crc;
}

uint16_t crc16_ccitt(const uint8_t *p, size_t n) {
    return crc16_update(0xFFFF, p, n);
}

uint32_t le32(const uint8_t *p) {
    return static_cast<uint32_t>(p[0]) |
           (static_cast<uint32_t>(p[1]) << 8) |
           (static_cast<uint32_t>(p[2]) << 16) |
           (static_cast<uint32_t>(p[3]) << 24);
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
    ::cfsetispeed(&tio, B115200);  // ignored by CDC ACM
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
    explicit Bridge(const std::string &path) : fd_(open_port(path)) {}

    ~Bridge() {
        if (fd_ >= 0) {
            ::close(fd_);
        }
    }

    Bridge(const Bridge &) = delete;
    Bridge &operator=(const Bridge &) = delete;

    struct Ack {
        uint32_t accepted;
        uint32_t used;
        uint32_t free;
    };

    struct Status {
        uint32_t capacity;
        uint32_t used;
        uint32_t free;
        uint32_t stage_ahead;
        uint32_t spi_consumed;
        uint32_t zero_fill;
        uint32_t crc_errors;
        uint32_t bad_frames;
        uint32_t underrun_events;
        uint32_t ring_full_events;
    };

    Ack write_chunk(const uint8_t *data, size_t len) {
        std::vector<uint8_t> p = transact(kCmdWrite, data, len, kRspAck);
        if (p.size() != 12) {
            throw std::runtime_error("bad ACK length");
        }
        return {le32(&p[0]), le32(&p[4]), le32(&p[8])};
    }

    Status status() {
        std::vector<uint8_t> p = transact(kCmdStatus, nullptr, 0, kRspStatus);
        if (p.size() != 40) {
            throw std::runtime_error("bad STATUS length");
        }
        return {le32(&p[0]),  le32(&p[4]),  le32(&p[8]),  le32(&p[12]),
                le32(&p[16]), le32(&p[20]), le32(&p[24]), le32(&p[28]),
                le32(&p[32]), le32(&p[36])};
    }

    // Push exactly len bytes. len must be a multiple of 64 so that every
    // 64-byte SPI transaction stays aligned across zero-fill underruns.
    void write_stream(const uint8_t *data, size_t len, size_t chunk,
                      int stall_ms = 10000) {
        if (len % kRecord != 0) {
            throw std::invalid_argument("length must be a multiple of 64");
        }
        if (chunk == 0 || chunk > kMaxPayload || chunk % kRecord != 0) {
            throw std::invalid_argument("chunk must be 64..4096, multiple of 64");
        }
        size_t off = 0;
        auto last_progress = Clock::now();
        while (off < len) {
            const size_t n = std::min(chunk, len - off);
            const Ack a = write_chunk(data + off, n);
            if (a.accepted > n) {
                throw std::runtime_error("device accepted more than sent");
            }
            off += a.accepted;
            if (a.accepted > 0) {
                last_progress = Clock::now();
            }
            if (a.accepted < n) {
                // Ring full: the SPI master paces consumption. Retry the tail.
                if (Clock::now() - last_progress >
                    std::chrono::milliseconds(stall_ms)) {
                    throw std::runtime_error(
                        "ring stalled (is the SPI master clocking?)");
                }
                std::this_thread::sleep_for(std::chrono::milliseconds(1));
            }
        }
    }

private:
    struct Frame {
        uint8_t type = 0;
        uint16_t seq = 0;
        std::vector<uint8_t> payload;
    };

    std::vector<uint8_t> transact(uint8_t type, const uint8_t *data, size_t len,
                                  uint8_t expect) {
        const uint16_t seq = ++seq_;
        send_frame(type, seq, data, len);
        auto deadline = Clock::now() + std::chrono::milliseconds(2000);
        Frame f;
        while (read_frame(f, deadline)) {
            if (f.type == expect && f.seq == seq) {
                return std::move(f.payload);
            }
        }
        throw std::runtime_error("timeout waiting for response");
    }

    void send_frame(uint8_t type, uint16_t seq, const uint8_t *data, size_t len) {
        std::vector<uint8_t> f;
        f.reserve(9 + len);
        f.push_back(kMagic0);
        f.push_back(kMagic1);
        f.push_back(type);
        f.push_back(static_cast<uint8_t>(seq));
        f.push_back(static_cast<uint8_t>(seq >> 8));
        f.push_back(static_cast<uint8_t>(len));
        f.push_back(static_cast<uint8_t>(len >> 8));
        if (len > 0) {
            f.insert(f.end(), data, data + len);
        }
        const uint16_t crc = crc16_ccitt(f.data() + 2, 5 + len);
        f.push_back(static_cast<uint8_t>(crc));
        f.push_back(static_cast<uint8_t>(crc >> 8));
        write_all(f.data(), f.size());
    }

    bool read_frame(Frame &f, Clock::time_point deadline) {
        int state = 0;
        for (;;) {  // resynchronise on AA 55
            uint8_t b;
            if (!read_exact(&b, 1, deadline)) {
                return false;
            }
            if (state == 0) {
                state = (b == kMagic0) ? 1 : 0;
            } else if (b == kMagic1) {
                break;
            } else {
                state = (b == kMagic0) ? 1 : 0;
            }
        }
        uint8_t hdr[5];
        if (!read_exact(hdr, sizeof hdr, deadline)) {
            return false;
        }
        const uint8_t type = hdr[0];
        const uint16_t seq = static_cast<uint16_t>(hdr[1] | (hdr[2] << 8));
        const uint16_t len = static_cast<uint16_t>(hdr[3] | (hdr[4] << 8));
        if (len > kMaxPayload) {
            return false;  // desynchronised; caller will time out
        }
        std::vector<uint8_t> payload(len);
        if (len > 0 && !read_exact(payload.data(), len, deadline)) {
            return false;
        }
        uint8_t crcb[2];
        if (!read_exact(crcb, sizeof crcb, deadline)) {
            return false;
        }
        uint16_t crc = crc16_update(0xFFFF, hdr, sizeof hdr);
        crc = crc16_update(crc, payload.data(), payload.size());
        if (crc != static_cast<uint16_t>(crcb[0] | (crcb[1] << 8))) {
            return false;
        }
        f.type = type;
        f.seq = seq;
        f.payload = std::move(payload);
        return true;
    }

    bool read_exact(void *dst, size_t len, Clock::time_point deadline) {
        auto *p = static_cast<uint8_t *>(dst);
        size_t got = 0;
        while (got < len) {
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
                    continue;
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
            const ssize_t n = ::read(fd_, p + got, len - got);
            if (n < 0) {
                if (errno == EINTR || errno == EAGAIN) {
                    continue;
                }
                throw std::runtime_error(std::string("read: ") +
                                         std::strerror(errno));
            }
            if (n == 0) {
                throw std::runtime_error("USB device disconnected");
            }
            got += static_cast<size_t>(n);
        }
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
                throw std::runtime_error("USB write timeout");
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
    uint16_t seq_ = 0;
};

void print_status(const Bridge::Status &s) {
    std::cout << "capacity        " << s.capacity << "\n"
              << "used            " << s.used << "\n"
              << "free            " << s.free << "\n"
              << "stage_ahead     " << s.stage_ahead << "\n"
              << "spi_consumed    " << s.spi_consumed << "\n"
              << "zero_fill       " << s.zero_fill << "\n"
              << "crc_errors      " << s.crc_errors << "\n"
              << "bad_frames      " << s.bad_frames << "\n"
              << "underrun_events " << s.underrun_events << "\n"
              << "ring_full       " << s.ring_full_events << "\n";
}

}  // namespace

int main(int argc, char **argv) {
    if (argc < 2) {
        std::cerr << "usage: " << argv[0]
                  << " <port> [file | --status] [--chunk N]\n";
        return 2;
    }
    try {
        Bridge bridge(argv[1]);
        bool status_only = false;
        std::string file;
        size_t chunk = kDefaultChunk;
        for (int i = 2; i < argc; ++i) {
            const std::string a = argv[i];
            if (a == "--status") {
                status_only = true;
            } else if (a == "--chunk" && i + 1 < argc) {
                chunk = std::stoul(argv[++i]);
            } else {
                file = a;
            }
        }

        if (status_only || file.empty()) {
            print_status(bridge.status());
            return 0;
        }

        std::ifstream in(file, std::ios::binary | std::ios::ate);
        if (!in) {
            throw std::runtime_error("cannot open " + file);
        }
        const std::streamoff size = in.tellg();
        if (size < 0 ||
            size % static_cast<std::streamoff>(kRecord) != 0) {
            throw std::runtime_error(
                "file size must be a multiple of 64 bytes (pad the last record)");
        }
        in.seekg(0);

        std::array<uint8_t, 64 * 1024> buf{};
        uint64_t total = 0;
        for (;;) {
            in.read(reinterpret_cast<char *>(buf.data()),
                    static_cast<std::streamsize>(buf.size()));
            const std::streamsize n = in.gcount();
            if (n > 0) {
                bridge.write_stream(buf.data(), static_cast<size_t>(n), chunk);
                total += static_cast<uint64_t>(n);
                std::cerr << "\r" << total << " bytes queued" << std::flush;
            }
            if (in.eof()) {
                break;
            }
            if (!in) {
                throw std::runtime_error("read error");
            }
        }
        std::cerr << "\n";
        print_status(bridge.status());
    } catch (const std::exception &e) {
        std::cerr << "error: " << e.what() << "\n";
        return 1;
    }
    return 0;
}
