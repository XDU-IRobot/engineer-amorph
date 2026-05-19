#pragma once

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/camera_info.hpp>
#include <sensor_msgs/msg/image.hpp>

#include <memory>

#include "amorph_client_gateway/fragment_parser.hpp"
#include "amorph_client_gateway/video_decoder.hpp"

namespace amorph {

/**
 * @brief ROS 2 node that bridges UDP H.265 stream → Image + CameraInfo topics.
 *
 * Parameters:
 *   - udp_port (int, default 3334)
 *   - frame_id (string, default "camera_optical_frame")
 *   - poll_period_sec (double, default 0.01)
 *
 * Publishers:
 *   - image_raw   (sensor_msgs/Image)    — BEST_EFFORT, VOLATILE
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
    void poll();

    /// GStreamer callback (runs in GStreamer thread): publish Image + CameraInfo.
    void on_decoded_frame(const cv::Mat& bgr_frame);

    // Parameters
    std::string _frame_id;

    // Publishers (declared before parser/decoder so they outlive the pipeline)
    rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr _img_pub;
    rclcpp::Publisher<sensor_msgs::msg::CameraInfo>::SharedPtr _cinfo_pub;

    // Timer
    rclcpp::TimerBase::SharedPtr _timer;

    // Pipeline (unique_ptr because FragmentParser/VideoDecoder are non-movable)
    std::unique_ptr<FragmentParser> _parser;
    std::unique_ptr<VideoDecoder> _decoder;

    // FPS counter
    int _frame_count = 0;
    rclcpp::Time _last_fps_log;

    // CameraInfo throttle — only publish when resolution changes
    int _last_cinfo_width = 0;
    int _last_cinfo_height = 0;
};

}  // namespace amorph
