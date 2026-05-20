#pragma once

#include <cstddef>
#include <cstdint>
#include <functional>

#include <gst/gst.h>
#include <gst/app/gstappsink.h>

namespace cv {
class Mat;
}

namespace amorph {

/// Callback signature: receives a decoded BGR frame (cv::Mat, 8UC3).
using FrameCallback = std::function<void(const cv::Mat& bgr_frame)>;

/**
 * @brief Wraps a GStreamer pipeline that decodes H.265 byte-stream → BGR.
 *
 * Pipeline:
 *   appsrc ! h265parse ! avdec_h265 ! videoconvert ! video/x-raw,format=BGR ! appsink
 *
 * Usage:
 *   VideoDecoder dec([](const cv::Mat& f) { ... });
 *   dec.Start();
 *   dec.push_frame(data, len);
 *   dec.Stop();
 */
class VideoDecoder {
public:
    explicit VideoDecoder(FrameCallback on_frame);
    ~VideoDecoder();

    VideoDecoder(const VideoDecoder&) = delete;
    VideoDecoder& operator=(const VideoDecoder&) = delete;
    VideoDecoder(VideoDecoder&&) = delete;
    VideoDecoder& operator=(VideoDecoder&&) = delete;

    /// Set pipeline to PLAYING state.
    void Start();

    /// Set pipeline to NULL state (teardown).
    void Stop();

    /**
     * @brief Push a complete H.265 frame into the pipeline.
     * @param data Pointer to H.265 byte-stream.
     * @param len  Number of bytes.
     *
     * Thread-safe: may be called from any thread.
     */
    void PushFrame(const uint8_t* data, size_t len);

private:
    /// Static GStreamer callback; user_data points to the VideoDecoder instance.
    static GstFlowReturn OnNewSample(GstAppSink* sink, gpointer user_data);

    FrameCallback on_frame_;
    GstElement* pipeline_ = nullptr;
    GstElement* appsrc_ = nullptr;
    GstElement* appsink_ = nullptr;
};

}  // namespace amorph
