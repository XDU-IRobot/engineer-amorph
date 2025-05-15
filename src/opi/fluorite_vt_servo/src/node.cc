
#include <algorithm>

#include <chrono>
#include <rclcpp/rclcpp.hpp>

#include <sensor_msgs/msg/joint_state.hpp>
#include <Eigen/Dense>
#include <thread>

#if __has_include("wiringPi.h")
#include "gpio.hpp"
#else
#include "gpio_mock.hpp"
#endif

class FluoriteVTServoNode : public rclcpp::Node {
 public:
  FluoriteVTServoNode(rclcpp::NodeOptions options = rclcpp::NodeOptions()
                                                        .allow_undeclared_parameters(true)
                                                        .automatically_declare_parameters_from_overrides(true))
      : Node("fluorite_vt_servo_node", options) {
    // get_parameter("pwm_pin", ros_params_.pwm_pin);
    get_parameter("angle_multiplier", ros_params_.angle_multiplier);
    get_parameter("zero_offset_degs", ros_params_.zero_offset_degs);
    // RCLCPP_INFO(get_logger(), "pwm_pin: %d", ros_params_.pwm_pin);
    RCLCPP_INFO(get_logger(), "angle_multiplier: %f", ros_params_.angle_multiplier);
    RCLCPP_INFO(get_logger(), "zero_offset_degs: %f", ros_params_.zero_offset_degs);
    // pwm_pin_ = Pin<PWM_OUTPUT>(ros_params_.pwm_pin);
    joint_state_sub_ = create_subscription<sensor_msgs::msg::JointState>(
        "/joint_states", rclcpp::SensorDataQoS(), [&](sensor_msgs::msg::JointState::SharedPtr msg) {
          const double endeff_yaw_heading = std::clamp(
              EndEffYaw(msg->position[3], std::clamp(msg->position[4], -M_PI / 2, M_PI / 2)), -M_PI / 2, M_PI / 2);
          if (std::abs(endeff_yaw_heading) < 17.0 / 360.0 * 2.0 * M_PI) {
            pwm_pin_.Write(AngleToDuty(ros_params_.zero_offset_degs / 180.0 * M_PI));
          } else {
            pwm_pin_.Write(AngleToDuty(-endeff_yaw_heading * ros_params_.angle_multiplier + ros_params_.zero_offset_degs/ 180.0 * M_PI));
          }
        });
  }

 private:
  static double AngleToDuty(double angle_rad) {
    constexpr double kCenter = 0.075;
    constexpr double kOneRad = 100.0 / 90.0 / 360.0 * 2.0 * M_PI;
    return kCenter + angle_rad * kOneRad;
  }

  static double EndEffYaw(double roll_rad, double pitch_rad) {
    const Eigen::Matrix3d rmat_roll = Eigen::AngleAxisd(roll_rad, Eigen::Vector3d::UnitX()).toRotationMatrix();
    const Eigen::Matrix3d rmat_pitch = Eigen::AngleAxisd(pitch_rad, Eigen::Vector3d::UnitY()).toRotationMatrix();
    const Eigen::Matrix3d rmat = rmat_roll * rmat_pitch;
    const Eigen::Vector3d eff_heading = rmat.col(0);
    return std::atan2(eff_heading.y(), eff_heading.x());
  }

  PwmOutputPin pwm_pin_ = Pin<PWM_OUTPUT>(13);
  rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr joint_state_sub_{nullptr};
  struct {
    int pwm_pin{};
    double angle_multiplier{};
    double zero_offset_degs{};
  } ros_params_{};
};

int main(int argc, char *argv[]) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<FluoriteVTServoNode>());
  rclcpp::shutdown();
  return 0;
}