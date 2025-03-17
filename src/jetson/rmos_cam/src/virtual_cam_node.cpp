// std
#include <dirent.h>

#include <chrono>
#include <sstream>

// ros
#include <cv_bridge/cv_bridge.h>

#include <ament_index_cpp/get_package_share_directory.hpp>

#include "rmos_cam/cam_node.hpp"
// #include "rmos_cam/virtual_cam.hpp"

namespace rmos_cam {
VirtualCamNode::VirtualCamNode(const rclcpp::NodeOptions &options) : CamNode("virtual_camera", options) {
  is_video = this->declare_parameter("use_video", false);
  // video path
  std::string video_path = this->declare_parameter("video_path", "/home/lizengyang/Videos/2021-1280-720.mp4");
  std::string folder_path = this->declare_parameter("images_floder_path", "/home/lizengyang/Pictures/std_imgs/");

  // cam dev
  virtual_dev_ = std::make_shared<camera::VirtualCam>();
  virtual_dev_->set_path(video_path);

  img_pub_ = image_transport::create_camera_publisher(this, "/image_raw", rmw_qos_profile_default);

  // load camera_info
  cam_info_manager_ = std::make_unique<camera_info_manager::CameraInfoManager>(this, "VirtualCam");

  auto pkg_path = ament_index_cpp::get_package_share_directory("rmos_bringup");
  auto yaml_path = "file://" + pkg_path + "/config/virtual_cam_info.yaml";
  if (!cam_info_manager_->loadCameraInfo(yaml_path)) {
    RCLCPP_WARN(this->get_logger(), "Load Camera Info Fail!");
  } else {
    camera_info_msg_ = cam_info_manager_->getCameraInfo();
  }

  // std::regex image_regex(".*\\.(jpg|png)$");

  // 读取文件夹中的所有文件
  DIR *dir;
  struct dirent *ent;
  if ((dir = opendir(folder_path.c_str())) != NULL) {
    while ((ent = readdir(dir)) != NULL) {
      std::string file_name = ent->d_name;
      this->image_paths.push_back(folder_path + file_name);
    }
    closedir(dir);
  } else {
    RCLCPP_ERROR(this->get_logger(), "Failed to open directory");
  }

  std::sort(image_paths.begin(), image_paths.end());
  camera_info_pub_ = create_publisher<sensor_msgs::msg::CameraInfo>("/daheng_camera_info", 10);

  capture_thread_ = std::thread([this]() -> void {
    while (rclcpp::ok()) {
      if (!is_video) {
        int frame_count_ = 0;
        for (const auto &pic_path : this->image_paths) {
          image_ = cv::imread(pic_path);
          if (image_.empty()) {
            RCLCPP_WARN(this->get_logger(), "Failed to load image: %s", pic_path.c_str());
            continue;
          }
          image_msg_ = cv_bridge::CvImage(std_msgs::msg::Header(), "bgr8", image_).toImageMsg();
          (*image_msg_).header.stamp = camera_info_msg_.header.stamp = this->now();
          (*image_msg_).header.frame_id = "camera";
          camera_info_msg_.header.frame_id = (*image_msg_).header.frame_id;
          camera_info_pub_->publish(camera_info_msg_);
          img_pub_.publish(*image_msg_, camera_info_msg_);
          frame_count_++;
          RCLCPP_INFO(this->get_logger(), "Published image: %s", pic_path.c_str());
          // 根据需要添加适当的延时
          std::this_thread::sleep_for(std::chrono::milliseconds(1000));  // 例如，每100ms发布一张图片
        }
      } else {
        virtual_dev_->open();
        if (!virtual_dev_->is_open()) {
          RCLCPP_ERROR(this->get_logger(), "Fail to open video path: %s", virtual_dev_->getVideoPath().c_str());
          exit(1);
        }
        if (virtual_dev_->grab_image(image_)) {
          image_msg_ = cv_bridge::CvImage(std_msgs::msg::Header(), "bgr8", image_).toImageMsg();
          (*image_msg_).header.stamp = camera_info_msg_.header.stamp = this->now();
          (*image_msg_).header.frame_id = "camera";
          camera_info_msg_.header.frame_id = (*image_msg_).header.frame_id;
          // RCLCPP_WARN(this->get_logger(), " Image from video");
          img_pub_.publish(*image_msg_, camera_info_msg_);
          std::this_thread::sleep_for(std::chrono::milliseconds(50));
        } else {
          std::cout << virtual_dev_->error_message() << std::endl;
          // exit(0);
        }
      }
    }
    // int count_ = 120;
    // int frame_count_ = 0;
    // while (rclcpp::ok()) {
    //     std::string pic_path
    //     ="/home/nuc12/Desktop/Vision_code/vision_test_data/picture/blue/" +
    //     std::to_string(count_)+".png"; image_ = cv::imread(pic_path);
    //     image_msg_ = cv_bridge::CvImage(std_msgs::msg::Header(), "bgr8",
    //     image_).toImageMsg();
    //     (*image_msg_).header.stamp = camera_info_msg_.header.stamp =
    //     this->now();
    //     (*image_msg_).header.frame_id = "camera";
    //     camera_info_msg_.header.frame_id = (*image_msg_).header.frame_id;
    //     RCLCPP_WARN(this->get_logger(), std::to_string(count_));
    //     img_pub_.publish(*image_msg_, camera_info_msg_);
    //     frame_count_++;
    //     if(frame_count_%200==0)
    //     {
    //         count_++;
    //     }
    //     std::this_thread::sleep_for(std::chrono::milliseconds(1000));

    //    if (!virtual_dev_->is_open()) {
    //        exit(0);
    //    }
    //     //     if (virtual_dev_->grab_image(image_)) {
    //     //     image_msg_ = cv_bridge::CvImage(std_msgs::msg::Header(),
    //     "bgr8", image_).toImageMsg();
    //     //     (*image_msg_).header.stamp = camera_info_msg_.header.stamp =
    //     this->now();
    //     //     (*image_msg_).header.frame_id = "camera";
    //     //     camera_info_msg_.header.frame_id =
    //     (*image_msg_).header.frame_id;
    //     //     RCLCPP_WARN(this->get_logger(), " Image from video");
    //     //     img_pub_.publish(*image_msg_, camera_info_msg_);
    //     //     //std::this_thread::sleep_for(std::chrono::milliseconds(10));
    //     // } else {
    //     //     std::cout << virtual_dev_->error_message() << std::endl;
    //     //     exit(0);
    //     // }
    // }
  });
}

VirtualCamNode::~VirtualCamNode() {
  if (capture_thread_.joinable()) {
    capture_thread_.join();
  }
  RCLCPP_INFO(this->get_logger(), "Camera node destroyed!");
}

}  // namespace rmos_cam

#include "rclcpp_components/register_node_macro.hpp"

// Register the component with class_loader.
// This acts as a sort of entry point, allowing the component to be discoverable
// when its library is being loaded into a running process.
RCLCPP_COMPONENTS_REGISTER_NODE(rmos_cam::VirtualCamNode)