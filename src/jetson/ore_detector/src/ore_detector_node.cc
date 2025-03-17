#include "ore_detector/ore_detector_node.hpp"
#include "ore_detector/detection.hpp"
#include <iostream>
#include <opencv2/opencv.hpp>

OreDetectorNode::OreDetectorNode(const rclcpp::NodeOptions &options) : Node("ore_detector") {
  std::string config_file = "detector/config/config.xml";

  cv::FileStorage fs(config_file, cv::FileStorage::READ);

  if (!fs.isOpened()) {
    throw std::runtime_error("Config file not found: " + config_file);
  }

  lowerB_ = (int)fs["lowerB"];
  upperB_ = (int)fs["upperB"];
  lowerH_ = (int)fs["lowerH"];
  upperH_ = (int)fs["upperH"];
  lowerS_ = (int)fs["lowerS"];
  upperS_ = (int)fs["upperS"];
  lowerV_ = (int)fs["lowerV"];
  upperV_ = (int)fs["upperV"];
  minContourArea_ = (int)fs["minContourArea"];
  int device_id = (int)fs["device_id"];

  int debug_mode = (int)fs["debug_mode"];

  publisher_ = this->create_publisher<std_msgs::msg::Float32MultiArray>("pnp_points", 10);

  cap_ = cv::VideoCapture(device_id);
  if (!cap_.isOpened()) {
    RCLCPP_ERROR(this->get_logger(), "Failed to open camera");
    return;
  }

  if (debug_mode) {
    cv::namedWindow("Trackbars", cv::WINDOW_AUTOSIZE);
    cv::createTrackbar("LowerH", "Trackbars", &lowerH_, 179);
    cv::createTrackbar("UpperH", "Trackbars", &upperH_, 179);
    cv::createTrackbar("LowerS", "Trackbars", &lowerS_, 255);
    cv::createTrackbar("UpperS", "Trackbars", &upperS_, 255);
    cv::createTrackbar("LowerV", "Trackbars", &lowerV_, 255);
    cv::createTrackbar("UpperV", "Trackbars", &upperV_, 255);
  }

  bool recording = false;
  cv::VideoWriter video_writer;

  rclcpp::Rate rate(30);
  while (rclcpp::ok()) {
    cv::Mat frame;
    cap_ >> frame;
    if (!frame.empty()) {
      processFrame(frame);

      if (recording) {
        video_writer.write(frame);
      }

      int key = cv::waitKey(1);
      if (key == 'r') {
        if (!recording) {
          std::string video_file = "/home/lzy/workspace/OreDetector/detector/output/video.avi";
          video_writer.open(video_file, cv::VideoWriter::fourcc('M', 'J', 'P', 'G'), 30, frame.size());
          if (!video_writer.isOpened()) {
            RCLCPP_ERROR(this->get_logger(), "Failed to open video writer");
          } else {
            recording = true;
            RCLCPP_INFO(this->get_logger(), "Started recording video");
          }
        } else {
          recording = false;
          video_writer.release();
          RCLCPP_INFO(this->get_logger(), "Stopped recording video");
        }
      }
    }
    rate.sleep();
  }
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