#pragma once

#include <atomic>
#include <cstddef>
#include <cstdint>
#include <map>
#include <mutex>
#include <optional>
#include <thread>
#include <unordered_map>
#include <vector>

namespace amorph {

/**
 * @brief Holds in-progress state for a single frame being reassembled.
 */
struct PendingFrame {
    uint32_t total_size = 0;
    uint32_t received_bytes = 0;
    std::map<uint16_t, std::vector<uint8_t>> fragments;
    double last_seen_sec = 0.0;
};

/**
 * @brief Receives fragmented H.265 video stream over UDP and reassembles
 *        complete frames.
 *
 * Protocol (8-byte custom header, big-endian):
 *   uint16 frame_num  — incrementing frame counter
 *   uint16 frag_seq   — fragment sequence within this frame
 *   uint32 total_size — total byte size of the complete frame
 *
 * Thread-safe: designed to be called from a timer thread (GetNextFrame)
 * while the internal I/O thread handles socket reads.
 */
class FragmentParser {
public:
    explicit FragmentParser(int port = 3334);
    ~FragmentParser();

    FragmentParser(const FragmentParser&) = delete;
    FragmentParser& operator=(const FragmentParser&) = delete;
    FragmentParser(FragmentParser&&) = delete;
    FragmentParser& operator=(FragmentParser&&) = delete;

    /// Bind socket & start reader thread.
    void Start();

    /// Stop reader thread & close socket.
    void Stop();

    /**
     * @brief Non-blocking: return the oldest complete frame if available.
     * @return std::nullopt if no complete frame is ready; otherwise a vector
     *         containing the reassembled H.265 byte-stream.
     *
     * Also performs garbage collection of stale (>5 s) incomplete frames.
     */
    std::optional<std::vector<uint8_t>> GetNextFrame();

private:
    void ReaderLoop();
    void ProcessPacket(const uint8_t* data, size_t len);

    static constexpr double kMaxFrameAgeSec = 5.0;
    static constexpr size_t kHeaderSize = 8;

    int port_;
    int sock_fd_ = -1;
    std::atomic<bool> running_{false};
    std::thread thread_;

    std::mutex mutex_;
    std::unordered_map<uint16_t, PendingFrame> pending_;
};

}  // namespace amorph
