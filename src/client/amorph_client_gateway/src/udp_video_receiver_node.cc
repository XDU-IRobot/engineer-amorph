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
    frame_id_ = this->declare_parameter("frame_id", std::string("camera_optical_frame"));
    double poll_period = this->declare_parameter("poll_period_sec", 0.01);
    camera_name_ = this->declare_parameter("camera_name", std::string("camera"));
    std::string camera_info_url = this->declare_parameter("camera_info_url", std::string(""));
    topic_namespace_ = this->declare_parameter("topic_namespace", std::string(""));
    max_fps_ = this->declare_parameter("max_fps", 0.0);

    // -- camera_info_manager (loads calibration from YAML) -------------------
    cinfo_manager_ = std::make_shared<camera_info_manager::CameraInfoManager>(
        this, camera_name_, camera_info_url);
    if (cinfo_manager_->isCalibrated()) {
        RCLCPP_INFO(this->get_logger(),
            "Loaded camera calibration for '%s' from %s",
            camera_name_.c_str(), camera_info_url.c_str());
    } else {
        RCLCPP_WARN(this->get_logger(),
            "No calibration loaded for '%s' — using default intrinsics (url=%s)",
            camera_name_.c_str(), camera_info_url.c_str());
    }

    // -- image_transport publishers ------------------------------------------
    rclcpp::QoS video_qos(1);
    video_qos.best_effort();
    video_qos.durability_volatile();

    rclcpp::QoS info_qos(1);
    info_qos.reliable();
    info_qos.transient_local();

    // -- topic name ----------------------------------------------------------
    std::string image_topic = topic_namespace_.empty()
        ? "image_raw"
        : topic_namespace_ + "/image_raw";

    cam_pub_ = image_transport::create_camera_publisher(this, image_topic,
        video_qos.get_rmw_qos_profile());

    // -- FPS counter ---------------------------------------------------------
    last_fps_log_ = this->get_clock()->now();
    last_publish_time_ = this->get_clock()->now();

    // -- pipeline ------------------------------------------------------------
    parser_ = std::make_unique<FragmentParser>(udp_port);
    parser_->Start();

    decoder_ = std::make_unique<VideoDecoder>(
        [this](const cv::Mat& f) { OnDecodedFrame(f); });
    decoder_->Start();

    // -- timer (feeds parser → decoder) -------------------------------------
    timer_ = this->create_wall_timer(
        std::chrono::duration<double>(poll_period),
        [this]() { Poll(); });

    RCLCPP_INFO(this->get_logger(),
        "Listening on UDP :%d, publishing to %s + %s/camera_info",
        udp_port, image_topic.c_str(), image_topic.c_str());
}

UdpVideoReceiverNode::~UdpVideoReceiverNode() {
    timer_.reset();           // Stop polling
    cam_pub_.shutdown();      // Shutdown image_transport publishers
    if (decoder_) decoder_->Stop();  // Stop GStreamer pipeline, no more callbacks
    if (parser_)  parser_->Stop();   // Stop UDP reader thread
}

// ---------------------------------------------------------------------------
// Private
// ---------------------------------------------------------------------------

void UdpVideoReceiverNode::Poll() {
    auto frame = parser_->GetNextFrame();
    if (frame.has_value()) {
        decoder_->PushFrame(frame->data(), frame->size());
    }
}

void UdpVideoReceiverNode::OnDecodedFrame(const cv::Mat& bgr_frame) {
    auto stamp = this->get_clock()->now();

    // -- FPS logging ---------------------------------------------------------
    frame_count_++;
    auto elapsed = stamp - last_fps_log_;
    if (elapsed >= std::chrono::seconds(5)) {
        double fps = frame_count_ / (elapsed.seconds());
        RCLCPP_INFO(this->get_logger(), "%.1f fps (%dx%d)", fps, bgr_frame.cols, bgr_frame.rows);
        frame_count_ = 0;
        last_fps_log_ = stamp;
    }

    // -- FPS throttle --------------------------------------------------------
    if (max_fps_ > 0.0) {
        auto min_interval = rclcpp::Duration::from_seconds(1.0 / max_fps_);
        if (stamp - last_publish_time_ < min_interval) {
            return;  // skip this frame
        }
    }

    // -- Image + CameraInfo via image_transport ------------------------------
    std_msgs::msg::Header header;
    header.stamp = stamp;
    header.frame_id = frame_id_;

    auto img_msg = cv_bridge::CvImage(header, "bgr8", bgr_frame).toImageMsg();

    // -- CameraInfo via camera_info_manager ----------------------------------
    auto cinfo_msg = std::make_shared<sensor_msgs::msg::CameraInfo>(
        cinfo_manager_->getCameraInfo());
    cinfo_msg->header = header;

    cam_pub_.publish(img_msg, cinfo_msg);
    last_publish_time_ = stamp;
}

}  // namespace amorph

RCLCPP_COMPONENTS_REGISTER_NODE(amorph::UdpVideoReceiverNode)
