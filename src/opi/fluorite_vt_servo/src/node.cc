
#include <algorithm>

#include <gpio_mock.hpp>
#include <rclcpp/rclcpp.hpp>

#include <sensor_msgs/msg/joint_state.hpp>
#include <Eigen/Dense>

#include "gpio.hpp"

class FluoriteVTServoNode : public rclcpp::Node {
 public:
  FluoriteVTServoNode(rclcpp::NodeOptions options = rclcpp::NodeOptions()
                                                        .allow_undeclared_parameters(true)
                                                        .automatically_declare_parameters_from_overrides(true))
      : Node("fluorite_vt_servo_node", options) {
    joint_state_sub_ = create_subscription<sensor_msgs::msg::JointState>(
        "/joint_states", rclcpp::SensorDataQoS(), [&](sensor_msgs::msg::JointState::SharedPtr msg) {
          const double endeff_yaw_heading =
              std::clamp(EndEffYaw(msg->position[3], msg->position[4]), -M_PI / 2, M_PI / 2);
          RCLCPP_INFO(get_logger(), "endeff heading: %f", endeff_yaw_heading);
        });
  }

 private:
  static double AngleToDuty(double angle) { return angle; }

  static double EndEffYaw(double roll_rad, double pitch_rad) {
    const Eigen::Matrix3d rmat_roll = Eigen::AngleAxisd(roll_rad, Eigen::Vector3d::UnitX()).toRotationMatrix();
    const Eigen::Matrix3d rmat_pitch = Eigen::AngleAxisd(pitch_rad, Eigen::Vector3d::UnitY()).toRotationMatrix();
    const Eigen::Matrix3d rmat = rmat_roll * rmat_pitch;
    const Eigen::Vector3d eff_heading = rmat.col(0);
    return std::atan2(eff_heading.y(), eff_heading.x());
  }

  PwmOutputPin pwm_pin_ = Pin<PWM_OUTPUT>(16);
  rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr joint_state_sub_{nullptr};
};

int main(int argc, char *argv[]) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<FluoriteVTServoNode>());
  rclcpp::shutdown();
  return 0;
}