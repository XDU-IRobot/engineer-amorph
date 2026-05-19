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
static const char* find_h265_decoder() {
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
    : _on_frame(std::move(on_frame)) {
    gst_init(nullptr, nullptr);

    const char* decoder = find_h265_decoder();
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
    _pipeline = gst_parse_launch(pipeline_desc.c_str(), &error);
    if (error != nullptr) {
        g_printerr("VideoDecoder: gst_parse_launch failed: %s\n", error->message);
        g_error_free(error);
        return;
    }

    _appsrc = gst_bin_get_by_name(GST_BIN(_pipeline), "src");
    _appsink = gst_bin_get_by_name(GST_BIN(_pipeline), "sink");

    if (_appsink == nullptr) {
        g_printerr("VideoDecoder: failed to get appsink element\n");
        return;
    }

    g_signal_connect(_appsink, "new-sample", G_CALLBACK(on_new_sample), this);
}

VideoDecoder::~VideoDecoder() { stop(); }

// ---------------------------------------------------------------------------
// Public API
// ---------------------------------------------------------------------------

void VideoDecoder::start() {
    if (_pipeline != nullptr) {
        gst_element_set_state(_pipeline, GST_STATE_PLAYING);
    }
}

void VideoDecoder::stop() {
    if (_pipeline != nullptr) {
        gst_element_set_state(_pipeline, GST_STATE_NULL);
        gst_object_unref(_pipeline);
        _pipeline = nullptr;
        _appsrc = nullptr;
        _appsink = nullptr;
    }
}

void VideoDecoder::push_frame(const uint8_t* data, size_t len) {
    if (_appsrc == nullptr || len == 0) {
        return;
    }

    GstBuffer* buf = gst_buffer_new_allocate(nullptr, len, nullptr);
    GstMapInfo map;
    if (gst_buffer_map(buf, &map, GST_MAP_WRITE)) {
        std::memcpy(map.data, data, len);
        gst_buffer_unmap(buf, &map);
        gst_app_src_push_buffer(GST_APP_SRC(_appsrc), buf);
    } else {
        gst_buffer_unref(buf);
    }
}

// ---------------------------------------------------------------------------
// Static GStreamer callback
// ---------------------------------------------------------------------------

GstFlowReturn VideoDecoder::on_new_sample(GstAppSink* /*sink*/, gpointer user_data) {
    auto* self = static_cast<VideoDecoder*>(user_data);

    GstSample* sample = gst_app_sink_pull_sample(GST_APP_SINK(self->_appsink));
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

    self->_on_frame(owned);
    return GST_FLOW_OK;
}

}  // namespace amorph
