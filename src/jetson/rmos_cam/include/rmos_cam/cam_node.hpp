#ifndef RMOS_CAM_NODE_HPP
#define RMOS_CAM_NODE_HPP

// std
#include <chrono>
#include <memory>
#include <mutex>
#include <string>
#include <thread>

// other
#include <opencv2/core.hpp>

// ros
#include <cv_bridge/cv_bridge.h>

#include <camera_info_manager/camera_info_manager.hpp>
#include <image_transport/image_transport.hpp>
#include <image_transport/publisher.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/camera_info.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <std_msgs/msg/int32.hpp>
#include <std_msgs/msg/int8.hpp>

#include "camera_interface.hpp"
#include "daheng.hpp"
#include "rcl_interfaces/msg/set_parameters_result.hpp"
#include "virtual_cam.hpp"

// #include "rmos_interfaces/msg/mode.hpp"
// #include "rmos_interfaces/msg/exp.hpp"
namespace rmos_cam {
class CamNode : public rclcpp::Node {
 public:
  CamNode(const std::string &node_name, const rclcpp::NodeOptions &options) : Node(node_name, options) {
    RCLCPP_INFO(this->get_logger(), "Starting node [%s]", node_name.c_str());
  };

 protected:
  image_transport::CameraPublisher img_pub_;      // 信息发布
  sensor_msgs::msg::CameraInfo camera_info_msg_;  // 相机消息
  sensor_msgs::msg::Image::SharedPtr image_msg_;

  std_msgs::msg::Int32 exp_msg;

  cv::Mat image_;
  std::unique_ptr<camera_info_manager::CameraInfoManager> cam_info_manager_;
  rclcpp::Publisher<sensor_msgs::msg::CameraInfo>::SharedPtr camera_info_pub_;

  uint32_t frame_id_ = 0;  // 帧计数器
};

class DahengCamNode : public virtual CamNode {
 public:
  DahengCamNode(const rclcpp::NodeOptions &options = rclcpp::NodeOptions());
  ~DahengCamNode();

  rcl_interfaces::msg::SetParametersResult parametersCallback(const std::vector<rclcpp::Parameter> &parameters);

 protected:
  void autoExpChange();

  // exp
  int normal_exposure_ = 2500;
  int normal_gamma_ = 9;

  int auto_exp_change_ = 0;
  int max_exp_ = 5000;
  int min_exp_ = 300;
  int change_num = 0;

  int time_offset_ = 0;  // 单位ns

  rclcpp::Publisher<std_msgs::msg::Int32>::SharedPtr exp_pub_;
  std::shared_ptr<camera::DahengCam> cam_dev_;
  std::thread capture_thread_;  // 采图线程

  OnSetParametersCallbackHandle::SharedPtr callback_handle_;
};

class VirtualCamNode : public virtual CamNode {
 public:
  VirtualCamNode(const rclcpp::NodeOptions &options = rclcpp::NodeOptions());
  ~VirtualCamNode();

 protected:
  std::shared_ptr<camera::VirtualCam> virtual_dev_;
  std::thread capture_thread_;  // 采图线程

  std::vector<std::string> image_paths;
  bool is_video;
};
}  // namespace rmos_cam

#endif  // RMOS_CAM_NODE_HPP
