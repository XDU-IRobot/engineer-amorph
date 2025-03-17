// std
#include <chrono>
#include <sstream>

// ros
#include <cv_bridge/cv_bridge.h>

#include <ament_index_cpp/get_package_share_directory.hpp>

#include "rmos_cam/cam_node.hpp"

using namespace std;

namespace rmos_cam {
DahengCamNode::DahengCamNode(const rclcpp::NodeOptions &options) : CamNode("daheng_camera", options) {
  // cam dev
  cam_dev_ = std::make_shared<camera::DahengCam>();

  // parameter
  int width = this->declare_parameter("width", 1280);
  int height = this->declare_parameter("height", 720);
  int exposure = this->declare_parameter("exposure", 2000);
  bool auto_exposure = this->declare_parameter("auto_exposure", false);
  bool auto_white_balance = this->declare_parameter("auto_white_balace", false);
  int gamma = this->declare_parameter("gain", 9);
  int fps = this->declare_parameter("fps", 100);
  float r_gain = this->declare_parameter("rgain", 19.8);
  float b_gain = this->declare_parameter("bgain", 19.8);
  float g_gain = this->declare_parameter("ggain", 10.0);
  this->time_offset_ = this->declare_parameter("time_offset", 0);
  this->auto_exp_change_ = this->declare_parameter("auto_exp_change", false);
  this->max_exp_ = this->declare_parameter("max_exp", 0);
  this->min_exp_ = this->declare_parameter("min_exp", 0);

  this->normal_exposure_ = exposure;
  this->normal_gamma_ = gamma;
  // set paramter
  cam_dev_->set_parameter(camera::CamParamType::Height, height);
  cam_dev_->set_parameter(camera::CamParamType::Width, width);
  cam_dev_->set_parameter(camera::CamParamType::AutoExposure, auto_exposure);
  cam_dev_->set_parameter(camera::CamParamType::Exposure, exposure);
  cam_dev_->set_parameter(camera::CamParamType::AutoWhiteBalance, auto_white_balance);
  cam_dev_->set_parameter(camera::CamParamType::RGain, r_gain);
  cam_dev_->set_parameter(camera::CamParamType::GGain, g_gain);
  cam_dev_->set_parameter(camera::CamParamType::BGain, b_gain);
  cam_dev_->set_parameter(camera::CamParamType::Gamma, gamma);
  cam_dev_->set_parameter(camera::CamParamType::Fps, fps);

  cam_dev_->open();

  img_pub_ = image_transport::create_camera_publisher(this, "/image_raw", rmw_qos_profile_default);

  // load camera_info
  cam_info_manager_ = std::make_unique<camera_info_manager::CameraInfoManager>(this, "DahengCam");
  auto pkg_path = ament_index_cpp::get_package_share_directory("rmos_cam");
  auto yaml_path = "file://" + pkg_path + "/config/daheng_cam_info.yaml";
  if (!cam_info_manager_->loadCameraInfo(yaml_path)) {
    RCLCPP_WARN(this->get_logger(), "Load Camera Info Fail!");
  } else {
    camera_info_msg_ = cam_info_manager_->getCameraInfo();
  }

  // exp publish
  this->exp_pub_ = this->create_publisher<std_msgs::msg::Int32>("/exp_info", rclcpp::SensorDataQoS());

  camera_info_pub_ = create_publisher<sensor_msgs::msg::CameraInfo>("/daheng_camera_info", 10);

  capture_thread_ = std::thread{[this]() -> void {
    // 设置循环频率为100Hz
    rclcpp::Rate loop_rate(100);

    while (rclcpp::ok()) {
      if (!cam_dev_->is_open()) {
        RCLCPP_WARN(this->get_logger(), "Faild open camera!!");
        cam_dev_->open();
      }
      // sensor_msgs::msg::Image image_msg_;

      if (cam_dev_->grab_image(image_)) {
        image_msg_ = cv_bridge::CvImage(std_msgs::msg::Header(), "bgr8", image_).toImageMsg();
        (*image_msg_).header.stamp = camera_info_msg_.header.stamp =
            this->now() + rclcpp::Duration(0, this->time_offset_);
        (*image_msg_).header.frame_id = "camera";
        camera_info_msg_.header.frame_id = (*image_msg_).header.frame_id;
        camera_info_pub_->publish(camera_info_msg_);

        img_pub_.publish(*image_msg_, camera_info_msg_);

        if (this->auto_exp_change_) autoExpChange();

        exp_msg.data = cam_dev_->params_[camera::CamParamType::Exposure];
        exp_pub_->publish(exp_msg);
        img_pub_.publish(*image_msg_, camera_info_msg_);
      } else {
        std::cout << cam_dev_->error_message() << std::endl;
        cam_dev_->open();
      }
    }
  }};
  callback_handle_ =
      this->add_on_set_parameters_callback(std::bind(&DahengCamNode::parametersCallback, this, std::placeholders::_1));
}

void DahengCamNode::autoExpChange() {
  int now_exp = cam_dev_->params_[camera::CamParamType::Exposure];
  if (this->change_num < 5) {
    change_num++;
    return;
  }
  change_num = 0;
  now_exp += 100;
  if (now_exp < this->min_exp_ || now_exp > this->max_exp_) now_exp = this->min_exp_;
  cam_dev_->SetExposure(now_exp);
  std::cout << "exp:" << cam_dev_->params_[camera::CamParamType::Exposure] << std::endl;
}

DahengCamNode::~DahengCamNode() {
  if (capture_thread_.joinable()) {
    capture_thread_.join();
  }
  cam_dev_->close();
  RCLCPP_INFO(this->get_logger(), "Camera node destroyed!");
}

rcl_interfaces::msg::SetParametersResult DahengCamNode::parametersCallback(
    const std::vector<rclcpp::Parameter> &parameters) {
  rcl_interfaces::msg::SetParametersResult result;
  result.successful = true;
  result.reason = "success";
  // Here update class attributes, do some actions, etc.

  return result;
}
}  // namespace rmos_cam

#include "rclcpp_components/register_node_macro.hpp"

// Register the component with class_loader.
// This acts as a sort of entry point, allowing the component to be discoverable
// when its library is being loaded into a running process.
RCLCPP_COMPONENTS_REGISTER_NODE(rmos_cam::DahengCamNode)