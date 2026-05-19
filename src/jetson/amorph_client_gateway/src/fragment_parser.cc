#include "amorph_client_gateway/fragment_parser.hpp"

#include <arpa/inet.h>
#include <errno.h>
#include <poll.h>
#include <sys/socket.h>
#include <time.h>
#include <unistd.h>

#include <algorithm>
#include <cstring>

namespace amorph {

// ---------------------------------------------------------------------------
// Helpers
// ---------------------------------------------------------------------------

namespace {

double monotonic_sec() {
    struct timespec ts {};
    clock_gettime(CLOCK_MONOTONIC, &ts);
    return static_cast<double>(ts.tv_sec) + static_cast<double>(ts.tv_nsec) * 1e-9;
}

std::uint16_t read_u16_be(const std::uint8_t* p) {
    return static_cast<std::uint16_t>((p[0] << 8) | p[1]);
}

std::uint32_t read_u32_be(const std::uint8_t* p) {
    return static_cast<std::uint32_t>(
        (p[0] << 24) | (p[1] << 16) | (p[2] << 8) | p[3]);
}

}  // namespace

// ---------------------------------------------------------------------------
// FragmentParser
// ---------------------------------------------------------------------------

FragmentParser::FragmentParser(int port) : _port(port) {}

FragmentParser::~FragmentParser() { stop(); }

void FragmentParser::start() {
    _sock_fd = socket(AF_INET, SOCK_DGRAM, 0);
    if (_sock_fd < 0) {
        perror("FragmentParser: socket()");
        return;
    }

    int reuse = 1;
    setsockopt(_sock_fd, SOL_SOCKET, SO_REUSEADDR, &reuse, sizeof(reuse));

    // Increase receive buffer to avoid drops under 60fps H.265 load
    int rcvbuf = 4 * 1024 * 1024;  // 4 MiB
    setsockopt(_sock_fd, SOL_SOCKET, SO_RCVBUF, &rcvbuf, sizeof(rcvbuf));

    struct sockaddr_in addr {};
    addr.sin_family = AF_INET;
    addr.sin_addr.s_addr = INADDR_ANY;
    addr.sin_port = htons(static_cast<std::uint16_t>(_port));

    if (bind(_sock_fd, reinterpret_cast<struct sockaddr*>(&addr), sizeof(addr)) < 0) {
        perror("FragmentParser: bind()");
        close(_sock_fd);
        _sock_fd = -1;
        return;
    }

    _running = true;
    _thread = std::thread(&FragmentParser::reader_loop, this);
}

void FragmentParser::stop() {
    _running = false;
    if (_thread.joinable()) {
        _thread.join();
    }
    if (_sock_fd >= 0) {
        close(_sock_fd);
        _sock_fd = -1;
    }
}

std::optional<std::vector<std::uint8_t>> FragmentParser::get_next_frame() {
    std::lock_guard<std::mutex> lock(_mutex);

    const double now = monotonic_sec();
    std::vector<std::uint16_t> completed;
    std::vector<std::uint16_t> stale;

    for (const auto& [frame_num, state] : _pending) {
        if (state.total_size > 0 && state.received_bytes >= state.total_size) {
            completed.push_back(frame_num);
        } else if (now - state.last_seen_sec > kMaxFrameAgeSec) {
            stale.push_back(frame_num);
        }
    }

    for (auto fnum : stale) {
        _pending.erase(fnum);
    }

    if (completed.empty()) {
        return std::nullopt;
    }

    // Return the oldest completed frame.
    std::sort(completed.begin(), completed.end());
    const std::uint16_t frame_num = completed.front();
    auto& state = _pending[frame_num];

    std::vector<std::uint8_t> result;
    result.reserve(state.total_size);
    for (const auto& [seq, frag] : state.fragments) {
        result.insert(result.end(), frag.begin(), frag.end());
    }

    _pending.erase(frame_num);
    return result;
}

// ---------------------------------------------------------------------------
// Private
// ---------------------------------------------------------------------------

void FragmentParser::reader_loop() {
    struct pollfd pfd {};
    pfd.fd = _sock_fd;
    pfd.events = POLLIN;

    std::uint8_t buffer[65536];

    while (_running) {
        int ret = poll(&pfd, 1, 100);  // 100 ms timeout
        if (ret < 0) {
            if (errno == EINTR) {
                continue;
            }
            break;
        }
        if (ret == 0) {
            continue;  // timeout, loop back to check _running
        }

        ssize_t n = recvfrom(_sock_fd, buffer, sizeof(buffer), 0, nullptr, nullptr);
        if (n > 0) {
            process_packet(buffer, static_cast<std::size_t>(n));
        }
    }
}

void FragmentParser::process_packet(const std::uint8_t* data, std::size_t len) {
    if (len < kHeaderSize) {
        return;
    }

    const std::uint16_t frame_num = read_u16_be(data);
    const std::uint16_t frag_seq = read_u16_be(data + 2);
    const std::uint32_t total_size = read_u32_be(data + 4);

    const std::uint8_t* payload = data + kHeaderSize;
    const std::size_t payload_len = len - kHeaderSize;

    std::lock_guard<std::mutex> lock(_mutex);

    auto& state = _pending[frame_num];
    state.total_size = total_size;
    state.fragments[frag_seq].assign(payload, payload + payload_len);
    state.received_bytes += payload_len;
    state.last_seen_sec = monotonic_sec();
}

}  // namespace amorph
