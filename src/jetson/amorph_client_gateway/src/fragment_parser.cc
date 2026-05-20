#include "amorph_client_gateway/fragment_parser.hpp"

#include <arpa/inet.h>
#include <errno.h>
#include <poll.h>
#include <sys/socket.h>
#include <time.h>
#include <unistd.h>

#include <algorithm>
#include <cstdio>
#include <cstring>

namespace amorph {

// ---------------------------------------------------------------------------
// Helpers
// ---------------------------------------------------------------------------

namespace {

double MonotonicSec() {
    struct timespec ts {};
    clock_gettime(CLOCK_MONOTONIC, &ts);
    return static_cast<double>(ts.tv_sec) + static_cast<double>(ts.tv_nsec) * 1e-9;
}

std::uint16_t ReadU16Be(const std::uint8_t* p) {
    return static_cast<std::uint16_t>((p[0] << 8) | p[1]);
}

std::uint32_t ReadU32Be(const std::uint8_t* p) {
    return static_cast<std::uint32_t>(
        (p[0] << 24) | (p[1] << 16) | (p[2] << 8) | p[3]);
}

}  // namespace

// ---------------------------------------------------------------------------
// FragmentParser
// ---------------------------------------------------------------------------

FragmentParser::FragmentParser(int port) : port_(port) {}

FragmentParser::~FragmentParser() { Stop(); }

void FragmentParser::Start() {
    sock_fd_ = socket(AF_INET, SOCK_DGRAM, 0);
    if (sock_fd_ < 0) {
        perror("FragmentParser: socket()");
        return;
    }

    int reuse = 1;
    setsockopt(sock_fd_, SOL_SOCKET, SO_REUSEADDR, &reuse, sizeof(reuse));

    // Increase receive buffer to avoid drops under 60fps H.265 load
    int rcvbuf = 4 * 1024 * 1024;  // 4 MiB
    setsockopt(sock_fd_, SOL_SOCKET, SO_RCVBUF, &rcvbuf, sizeof(rcvbuf));

    struct sockaddr_in addr {};
    addr.sin_family = AF_INET;
    addr.sin_addr.s_addr = INADDR_ANY;
    addr.sin_port = htons(static_cast<std::uint16_t>(port_));

    if (bind(sock_fd_, reinterpret_cast<struct sockaddr*>(&addr), sizeof(addr)) < 0) {
        perror("FragmentParser: bind()");
        close(sock_fd_);
        sock_fd_ = -1;
        return;
    }

    running_ = true;
    thread_ = std::thread(&FragmentParser::ReaderLoop, this);
}

void FragmentParser::Stop() {
    running_ = false;
    if (thread_.joinable()) {
        thread_.join();
    }
    if (sock_fd_ >= 0) {
        close(sock_fd_);
        sock_fd_ = -1;
    }
}

std::optional<std::vector<std::uint8_t>> FragmentParser::GetNextFrame() {
    std::lock_guard<std::mutex> lock(mutex_);

    const double now = MonotonicSec();
    std::vector<std::uint16_t> completed;
    std::vector<std::uint16_t> stale;

    for (const auto& [frame_num, state] : pending_) {
        if (state.total_size > 0 && state.received_bytes >= state.total_size) {
            completed.push_back(frame_num);
        } else if (now - state.last_seen_sec > kMaxFrameAgeSec) {
            stale.push_back(frame_num);
        }
    }

    for (auto fnum : stale) {
        pending_.erase(fnum);
    }

    if (completed.empty()) {
        return std::nullopt;
    }

    // Return the oldest completed frame.
    std::sort(completed.begin(), completed.end());
    const std::uint16_t frame_num = completed.front();
    auto& state = pending_[frame_num];

    std::vector<std::uint8_t> result;
    result.reserve(state.total_size);
    for (const auto& [seq, frag] : state.fragments) {
        result.insert(result.end(), frag.begin(), frag.end());
    }

    pending_.erase(frame_num);
    return result;
}

// ---------------------------------------------------------------------------
// Private
// ---------------------------------------------------------------------------

void FragmentParser::ReaderLoop() {
    struct pollfd pfd {};
    pfd.fd = sock_fd_;
    pfd.events = POLLIN;

    std::uint8_t buffer[65536];

    while (running_) {
        int ret = poll(&pfd, 1, 100);  // 100 ms timeout
        if (ret < 0) {
            if (errno == EINTR) {
                continue;
            }
            break;
        }
        if (ret == 0) {
            continue;  // timeout, loop back to check running_
        }

        ssize_t n = recvfrom(sock_fd_, buffer, sizeof(buffer), 0, nullptr, nullptr);
        if (n > 0) {
            ProcessPacket(buffer, static_cast<std::size_t>(n));
        }
    }
}

void FragmentParser::ProcessPacket(const std::uint8_t* data, std::size_t len) {
    if (len < kHeaderSize) {
        return;
    }

    const std::uint16_t frame_num = ReadU16Be(data);
    const std::uint16_t frag_seq = ReadU16Be(data + 2);
    const std::uint32_t total_size = ReadU32Be(data + 4);

    const std::uint8_t* payload = data + kHeaderSize;
    const std::size_t payload_len = len - kHeaderSize;

    std::lock_guard<std::mutex> lock(mutex_);

    auto& state = pending_[frame_num];
    state.total_size = total_size;
    state.fragments[frag_seq].assign(payload, payload + payload_len);
    state.received_bytes += payload_len;
    state.last_seen_sec = MonotonicSec();
}

}  // namespace amorph
