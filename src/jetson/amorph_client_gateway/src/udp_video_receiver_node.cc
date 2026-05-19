#include "amorph_client_gateway/udp_video_receiver_node.hpp"

#include <cv_bridge/cv_bridge.hpp>
#include <rclcpp_components/register_node_macro.hpp>
#include <std_msgs/msg/header.hpp>

namespace amorph {

// ---------------------------------------------------------------------------
// Constructor
// ---------------------------------------------------------------------------

UdpVideoReceiverNode::UdpVideoReceiverNode(const rclcpp::NodeOptions& options)
    : Node("udp_video_receiver", options) {
    // -- parameters ----------------------------------------------------------
    int udp_port = this->declare_parameter("udp_port", 3334);
    _frame_id = this->declare_parameter("frame_id", std::string("camera_optical_frame"));
    double poll_period = this->declare_parameter("poll_period_sec", 0.01);

    // -- publishers ----------------------------------------------------------
    rclcpp::QoS video_qos(1);
    video_qos.best_effort();
    video_qos.durability_volatile();

    rclcpp::QoS info_qos(1);
    info_qos.reliable();
    info_qos.transient_local();

    _img_pub = this->create_publisher<sensor_msgs::msg::Image>("image_raw", video_qos);
    _cinfo_pub = this->create_publisher<sensor_msgs::msg::CameraInfo>("camera_info", info_qos);

    // -- FPS counter ---------------------------------------------------------
    _last_fps_log = this->get_clock()->now();

    // -- pipeline ------------------------------------------------------------
    _parser = std::make_unique<FragmentParser>(udp_port);
    _parser->start();

    _decoder = std::make_unique<VideoDecoder>(
        [this](const cv::Mat& f) { on_decoded_frame(f); });
    _decoder->start();

    // -- timer (feeds parser → decoder) -------------------------------------
    _timer = this->create_wall_timer(
        std::chrono::duration<double>(poll_period),
        [this]() { poll(); });

    RCLCPP_INFO(this->get_logger(),
        "Listening on UDP :%d, publishing to image_raw + camera_info", udp_port);
}

UdpVideoReceiverNode::~UdpVideoReceiverNode() {
    _timer.reset();        // Stop polling
    if (_decoder) _decoder->stop();  // Stop GStreamer pipeline, no more callbacks
    if (_parser)  _parser->stop();   // Stop UDP reader thread
}

// ---------------------------------------------------------------------------
// Private
// ---------------------------------------------------------------------------

void UdpVideoReceiverNode::poll() {
    auto frame = _parser->get_next_frame();
    if (frame.has_value()) {
        _decoder->push_frame(frame->data(), frame->size());
    }
}

void UdpVideoReceiverNode::on_decoded_frame(const cv::Mat& bgr_frame) {
    auto stamp = this->get_clock()->now();

    // -- FPS logging ---------------------------------------------------------
    _frame_count++;
    auto elapsed = stamp - _last_fps_log;
    if (elapsed >= std::chrono::seconds(5)) {
        double fps = _frame_count / (elapsed.seconds());
        RCLCPP_INFO(this->get_logger(), "%.1f fps (%lux%lu)", fps, bgr_frame.cols, bgr_frame.rows);
        _frame_count = 0;
        _last_fps_log = stamp;
    }

    // -- Image ---------------------------------------------------------------
    std_msgs::msg::Header header;
    header.stamp = stamp;
    header.frame_id = _frame_id;

    auto img_msg = cv_bridge::CvImage(header, "bgr8", bgr_frame).toImageMsg();
    _img_pub->publish(*img_msg);

    // -- CameraInfo (only when resolution changes) ---------------------------
    const int width = static_cast<int>(bgr_frame.cols);
    const int height = static_cast<int>(bgr_frame.rows);

    if (width == _last_cinfo_width && height == _last_cinfo_height) {
        return;  // camera info hasn't changed, skip publishing
    }
    _last_cinfo_width = width;
    _last_cinfo_height = height;

    sensor_msgs::msg::CameraInfo cinfo;
    cinfo.header = header;
    cinfo.width = width;
    cinfo.height = height;

    // Default intrinsics: unit focal length, centered principal point.
    // Replace with real calibration if available.
    cinfo.k.fill(0.0);
    cinfo.k[0] = static_cast<double>(width);
    cinfo.k[2] = static_cast<double>(width) / 2.0;
    cinfo.k[4] = static_cast<double>(width);
    cinfo.k[5] = static_cast<double>(height) / 2.0;
    cinfo.k[8] = 1.0;

    // No distortion
    cinfo.d = {0.0, 0.0, 0.0, 0.0, 0.0};

    // P = K | [0 0 0]
    cinfo.p.fill(0.0);
    cinfo.p[0] = cinfo.k[0];
    cinfo.p[2] = cinfo.k[2];
    cinfo.p[5] = cinfo.k[5];
    cinfo.p[10] = 1.0;

    cinfo.binning_x = 1;
    cinfo.binning_y = 1;
    cinfo.roi.x_offset = 0;
    cinfo.roi.y_offset = 0;
    cinfo.roi.width = width;
    cinfo.roi.height = height;
    cinfo.roi.do_rectify = false;

    _cinfo_pub->publish(cinfo);

    RCLCPP_INFO_ONCE(this->get_logger(), "CameraInfo published (%dx%d)", width, height);
}

}  // namespace amorph

RCLCPP_COMPONENTS_REGISTER_NODE(amorph::UdpVideoReceiverNode)
