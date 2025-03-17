
#include <iostream>

#include <opencv2/opencv.hpp>
#include <cv_bridge/cv_bridge.h>

#include "ore_detector/ore_detector_node.hpp"
#include "ore_detector/detection.hpp"

OreDetectorNode::OreDetectorNode(const rclcpp::NodeOptions &options) : Node("ore_detector", options) {
  declare_parameter("lower_b", 150);
  declare_parameter("upper_b", 255);
  declare_parameter("lower_h", 20);
  declare_parameter("upper_h", 30);
  declare_parameter("lower_s", 100);
  declare_parameter("upper_s", 255);
  declare_parameter("lower_v", 100);
  declare_parameter("upper_v", 255);
  declare_parameter("min_contour_area", 10);
  lowerB_ = get_parameter("lower_b").as_int();
  upperB_ = get_parameter("upper_b").as_int();
  lowerH_ = get_parameter("lower_h").as_int();
  upperH_ = get_parameter("upper_h").as_int();
  lowerS_ = get_parameter("lower_s").as_int();
  upperS_ = get_parameter("upper_s").as_int();
  lowerV_ = get_parameter("lower_v").as_int();
  upperV_ = get_parameter("upper_v").as_int();
  minContourArea_ = get_parameter("min_contour_area").as_int();

  publisher_ = this->create_publisher<std_msgs::msg::Float32MultiArray>("pnp_points", rclcpp::SystemDefaultsQoS());

  image_sub_ = this->create_subscription<sensor_msgs::msg::Image>(
      "/image_raw", 10, [&](const sensor_msgs::msg::Image::SharedPtr msg) {
        cv_bridge::CvImagePtr cv_ptr;
        try {
          cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
        } catch (cv_bridge::Exception &e) {
          RCLCPP_ERROR(this->get_logger(), "cv_bridge exception: %s", e.what());
          return;
        }
        imageCallback(cv_ptr->image);
      });
}

OreDetectorNode::~OreDetectorNode() {}

void OreDetectorNode::imageCallback(const cv::Mat &frame) {
  if (!frame.empty()) {
    processFrame(frame);
  }
}

void OreDetectorNode::processFrame(const cv::Mat &frame) {
  std::vector<cv::Point2f> corners =
      detectOreCorners(frame, lowerB_, upperB_, lowerH_, upperH_, lowerS_, upperS_, lowerV_, upperV_, minContourArea_);
  std_msgs::msg::Float32MultiArray msg;
  for (const auto &pt : corners) {
    msg.data.push_back(pt.x);
    msg.data.push_back(pt.y);
  }
  publisher_->publish(msg);
}

#include <rclcpp_components/register_node_macro.hpp>

// Register the component with class_loader.
// This acts as a sort of entry point, allowing the component to be discoverable when its library
// is being loaded into a running process.
RCLCPP_COMPONENTS_REGISTER_NODE(OreDetectorNode)