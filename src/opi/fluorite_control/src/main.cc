
#include <thread>

#include "arm/node.hpp"
#include "chassis/node.hpp"

int main(int argc, char *argv[]) {
  rclcpp::init(argc, argv);

  std::thread arm_thread([] {
    rclcpp::executors::SingleThreadedExecutor executor;
    auto arm_node = std::make_shared<fluorite::arm::FluoriteArm>();
    executor.add_node(arm_node);
    executor.spin();
  });

  std::thread chassis_thread([] {
    rclcpp::executors::SingleThreadedExecutor executor;
    auto chassis_node = std::make_shared<fluorite::chassis::FluoriteChassis>();
    executor.add_node(chassis_node);
    executor.spin();
  });

  arm_thread.join();
  chassis_thread.join();

  rclcpp::shutdown();
  return 0;
}