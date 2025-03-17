#ifndef ORE_DETECTOR_HPP
#define ORE_DETECTOR_HPP

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/float32_multi_array.hpp"
#include "sensor_msgs/msg/image.hpp"
#include <filesystem>
#include <opencv2/opencv.hpp>
#include <vector>

class OreDetectorNode : public rclcpp::Node {
 public:
  OreDetectorNode(const rclcpp::NodeOptions &options = rclcpp::NodeOptions()
                                                           .allow_undeclared_parameters(true)
                                                           .automatically_declare_parameters_from_overrides(true));
  ~OreDetectorNode();

 private:
  void imageCallback(const cv::Mat &frame);
  void processFrame(const cv::Mat &frame);

  rclcpp::Publisher<std_msgs::msg::Float32MultiArray>::SharedPtr publisher_;
  rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr image_sub_;
  cv::VideoCapture cap_;  // VideoCapture object for USB camera
  int lowerB_, upperB_, lowerH_, upperH_, lowerS_, upperS_, lowerV_, upperV_;
  int minContourArea_;
};

#endif  // ORE_DETECTOR_HPP
