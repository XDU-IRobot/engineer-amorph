
#include <rclcpp/rclcpp.hpp>

#include "gpio.hpp"

class FluoriteVTServoNode : public rclcpp::Node {
 public:
  FluoriteVTServoNode(rclcpp::NodeOptions options = rclcpp::NodeOptions()
                                                        .allow_undeclared_parameters(true)
                                                        .automatically_declare_parameters_from_overrides(true))
      : Node("fluorite_vt_servo_node", options) {
    while (true) {
      for (size_t i = 0; i < 100; ++i) {
        pwm_pin_.Write(i / 100.0);
        std::this_thread::sleep_for(std::chrono::milliseconds(30));
      }
    }
  }

 private:
  PwmOutputPin pwm_pin_ = Pin<PWM_OUTPUT>(13);
};

int main(int argc, char *argv[]) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<FluoriteVTServoNode>());
  rclcpp::shutdown();
  return 0;
}