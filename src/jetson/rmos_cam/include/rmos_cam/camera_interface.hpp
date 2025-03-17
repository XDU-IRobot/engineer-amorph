
#ifndef RMOS_CAMERA_INTERFACE_HPP
#define RMOS_CAMERA_INTERFACE_HPP

#include <opencv2/core.hpp>

namespace camera {

enum class CamParamType {
  Width,
  Height,
  AutoExposure,
  Exposure,
  Brightness,
  AutoWhiteBalance,
  WhiteBalance,
  Gain,
  RGain,
  GGain,
  BGain,
  Gamma,
  Contrast,
  Saturation,
  Hue,
  Fps
};

// common interface for camera device (usb cam, virtual cam, etc.)
class CamInterface {
 public:
  virtual ~CamInterface() = default;

  virtual bool open() = 0;
  virtual bool close() = 0;
  virtual bool is_open() = 0;

  virtual bool grab_image(cv::Mat &imgae) = 0;

  // set and get parameter
  virtual bool set_parameter(CamParamType type, int value) = 0;
  virtual bool get_parameter(CamParamType type, int &value) = 0;
  // get error message when above api return false.
  virtual std::string error_message() = 0;
};

}  // namespace camera

#endif  // RMOS_CAMERA_INTERFACE_HPP
