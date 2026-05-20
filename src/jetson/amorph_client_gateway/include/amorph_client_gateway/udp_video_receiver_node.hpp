#pragma once

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/camera_info.hpp>
#include <sensor_msgs/msg/image.hpp>

#include <camera_info_manager/camera_info_manager.hpp>
#include <image_transport/image_transport.hpp>

#include <memory>

#include "amorph_client_gateway/fragment_parser.hpp"
#include "amorph_client_gateway/video_decoder.hpp"

namespace amorph {

/**
 * @brief ROS 2 node that bridges UDP H.265 stream → Image + CameraInfo topics.
 *
 * Uses image_transport so subscribers can negotiate compressed transport
 * (e.g. /image_raw/compressed) to drastically reduce DDS bandwidth for
 * 1080p@60fps streams.
 *
 * Parameters:
 *   - udp_port (int, default 3334)
 *   - frame_id (string, default "camera_optical_frame")
 *   - poll_period_sec (double, default 0.01)
 *   - camera_name (string, default "camera")
 *   - camera_info_url (string, default "") — file:// or package:// URL to calibration YAML
 *   - topic_namespace (string, default "") — namespace prefix for topics, e.g. "vt_camera" → /vt_camera/image_raw
 *   - max_fps (double, default 0.0) — throttle output rate; 0 = no limit
 *
 * Publishers (via image_transport):
 *   - image_raw   (sensor_msgs/Image)    — BEST_EFFORT, VOLATILE
 *   - image_raw/compressed (sensor_msgs/CompressedImage) — BEST_EFFORT, VOLATILE
 *   - camera_info (sensor_msgs/CameraInfo) — RELIABLE, TRANSIENT_LOCAL
 */
class UdpVideoReceiverNode : public rclcpp::Node {
public:
    explicit UdpVideoReceiverNode(
        const rclcpp::NodeOptions& options = rclcpp::NodeOptions()
                                                .allow_undeclared_parameters(true)
                                                .automatically_declare_parameters_from_overrides(true));

    ~UdpVideoReceiverNode() override;

private:
    /// Timer callback: pull frames from parser, push to decoder.
    void Poll();

    /// GStreamer callback (runs in GStreamer thread): publish Image + CameraInfo.
    void OnDecodedFrame(const cv::Mat& bgr_frame);

    // Parameters
    std::string frame_id_;

    // image_transport (declared before parser/decoder so it outlives the pipeline)
    image_transport::CameraPublisher cam_pub_;

    // Timer
    rclcpp::TimerBase::SharedPtr timer_;

    // Pipeline (unique_ptr because FragmentParser/VideoDecoder are non-movable)
    std::unique_ptr<FragmentParser> parser_;
    std::unique_ptr<VideoDecoder> decoder_;

    // FPS counter / throttle
    int frame_count_ = 0;
    rclcpp::Time last_fps_log_;
    double max_fps_ = 0.0;
    rclcpp::Time last_publish_time_;

    // CameraInfo via camera_info_manager (loads calibration from YAML)
    std::shared_ptr<camera_info_manager::CameraInfoManager> cinfo_manager_;
    std::string camera_name_;
    std::string topic_namespace_;
};

}  // namespace amorph
