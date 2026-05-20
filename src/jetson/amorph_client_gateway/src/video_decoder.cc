#include "amorph_client_gateway/video_decoder.hpp"

#include <gst/gst.h>
#include <gst/app/gstappsink.h>
#include <gst/app/gstappsrc.h>

#include <cstring>
#include <string>
#include <opencv2/core/mat.hpp>

namespace amorph {

// ---------------------------------------------------------------------------
// H.265 decoder auto-detection
// ---------------------------------------------------------------------------

/// Probe available GStreamer decoder factories in priority order.
/// Returns the factory name of the best available decoder.
static const char* FindH265Decoder() {
    return "vah265dec";
    // Priority: NVIDIA NVDEC > VA-API (Intel/AMD) > newer VA > software
    static const char* kCandidates[] = {
        "nvh265dec",      // NVIDIA NVDEC  (gstreamer1.0-plugins-bad)
        "vaapih265dec",   // VA-API        (gstreamer1.0-vaapi)
        "vah265dec",      // VA (newer)    (gstreamer1.0-va)
        "avdec_h265",     // software       (always available)
    };

    for (auto* name : kCandidates) {
        GstElementFactory* factory = gst_element_factory_find(name);
        if (factory != nullptr) {
            gst_object_unref(factory);
            return name;
        }
    }
    return "avdec_h265";  // ultimate fallback
}

// ---------------------------------------------------------------------------
// Constructor / Destructor
// ---------------------------------------------------------------------------
VideoDecoder::VideoDecoder(FrameCallback on_frame)
    : on_frame_(std::move(on_frame)) {
    gst_init(nullptr, nullptr);

    const char* decoder = FindH265Decoder();
    g_print("VideoDecoder: using %s for H.265 decode\n", decoder);

    std::string pipeline_desc =
        std::string("appsrc name=src is-live=true block=true format=time ")
        + "! h265parse "
        + "! queue max-size-buffers=2 "
        + "! " + decoder + " "
        + "! queue max-size-buffers=1 "
        + "! videoconvert "
        + "! video/x-raw,format=BGR "
        + "! appsink name=sink emit-signals=true max-buffers=1 drop=true sync=false";

    GError* error = nullptr;
    pipeline_ = gst_parse_launch(pipeline_desc.c_str(), &error);
    if (error != nullptr) {
        g_printerr("VideoDecoder: gst_parse_launch failed: %s\n", error->message);
        g_error_free(error);
        return;
    }

    appsrc_ = gst_bin_get_by_name(GST_BIN(pipeline_), "src");
    appsink_ = gst_bin_get_by_name(GST_BIN(pipeline_), "sink");

    if (appsink_ == nullptr) {
        g_printerr("VideoDecoder: failed to get appsink element\n");
        return;
    }

    g_signal_connect(appsink_, "new-sample", G_CALLBACK(OnNewSample), this);
}

VideoDecoder::~VideoDecoder() { Stop(); }

// ---------------------------------------------------------------------------
// Public API
// ---------------------------------------------------------------------------

void VideoDecoder::Start() {
    if (pipeline_ != nullptr) {
        gst_element_set_state(pipeline_, GST_STATE_PLAYING);
    }
}

void VideoDecoder::Stop() {
    if (pipeline_ != nullptr) {
        gst_element_set_state(pipeline_, GST_STATE_NULL);
        gst_object_unref(pipeline_);
        pipeline_ = nullptr;
        appsrc_ = nullptr;
        appsink_ = nullptr;
    }
}

void VideoDecoder::PushFrame(const uint8_t* data, size_t len) {
    if (appsrc_ == nullptr || len == 0) {
        return;
    }

    GstBuffer* buf = gst_buffer_new_allocate(nullptr, len, nullptr);
    GstMapInfo map;
    if (gst_buffer_map(buf, &map, GST_MAP_WRITE)) {
        std::memcpy(map.data, data, len);
        gst_buffer_unmap(buf, &map);
        gst_app_src_push_buffer(GST_APP_SRC(appsrc_), buf);
    } else {
        gst_buffer_unref(buf);
    }
}

// ---------------------------------------------------------------------------
// Static GStreamer callback
// ---------------------------------------------------------------------------

GstFlowReturn VideoDecoder::OnNewSample(GstAppSink* /*sink*/, gpointer user_data) {
    auto* self = static_cast<VideoDecoder*>(user_data);

    GstSample* sample = gst_app_sink_pull_sample(GST_APP_SINK(self->appsink_));
    if (sample == nullptr) {
        return GST_FLOW_ERROR;
    }

    GstCaps* caps = gst_sample_get_caps(sample);
    GstStructure* structure = gst_caps_get_structure(caps, 0);

    int width = 0;
    int height = 0;
    if (!gst_structure_get_int(structure, "width", &width) ||
        !gst_structure_get_int(structure, "height", &height)) {
        gst_sample_unref(sample);
        return GST_FLOW_ERROR;
    }

    GstBuffer* buf = gst_sample_get_buffer(sample);
    GstMapInfo map;
    if (!gst_buffer_map(buf, &map, GST_MAP_READ)) {
        gst_sample_unref(sample);
        return GST_FLOW_ERROR;
    }

    // Build cv::Mat over GStreamer buffer and ref the buffer so data stays
    // alive after unmap. The custom deleter unrefs the buffer once cv::Mat
    // is destroyed, avoiding an extra clone.
    gst_buffer_ref(buf);
    cv::Mat frame(height, width, CV_8UC3, map.data);
    cv::Mat owned = cv::Mat(height, width, CV_8UC3);
    frame.copyTo(owned);  // single copy out of mapped GPU→CPU memory

    gst_buffer_unmap(buf, &map);
    gst_buffer_unref(buf);
    gst_sample_unref(sample);

    self->on_frame_(owned);
    return GST_FLOW_OK;
}

}  // namespace amorph
