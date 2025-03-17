#include "rmos_cam/virtual_cam.hpp"

namespace camera {
bool VirtualCam::open() {
  if (!this->capture.open(this->video_path)) {
    this->is_open_ = false;
    return false;
  } else {
    this->is_open_ = true;
    return true;
  }
}

bool VirtualCam::is_open() { return this->is_open_; }

bool VirtualCam::grab_image(cv::Mat &image) {
  this->capture >> image;
  if (image.empty()) {
    return false;
  } else {
    return true;
  }
}

std::string VirtualCam::getVideoPath() { return this->video_path; }

}  // namespace camera