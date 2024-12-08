
#include <rclcpp/rclcpp.hpp>
#include <moveit/move_group_interface/move_group_interface.h>

#include <chrono>

class FluoriteArm : public rclcpp::Node {
 public:
  FluoriteArm(const rclcpp::NodeOptions &options = rclcpp::NodeOptions()
                                                       .allow_undeclared_parameters(true)
                                                       .automatically_declare_parameters_from_overrides(true))
      : Node("fluorite_arm", options) {
    move_group_ =
        std::make_shared<moveit::planning_interface::MoveGroupInterface>(rclcpp::Node::SharedPtr(this), "arm");
    TryPlan();
  }

  void TryPlan() {
    auto target_pose = [&]() {
      geometry_msgs::msg::Pose target_pose;
      target_pose.position.x = 0.5;
      target_pose.position.y = 0.0;
      target_pose.position.z = 0.5;
      target_pose.orientation.x = 0.0;
      target_pose.orientation.y = 0.0;
      target_pose.orientation.z = 0.0;
      target_pose.orientation.w = 1.0;
      return target_pose;
    }();

    auto start_time = std::chrono::high_resolution_clock::now();
    for (int i = 0; i < 1000; i++) {
      target_pose.orientation.x += 0.001;
      move_group_->setPoseTarget(target_pose);
      move_group_->setPlanningTime(0.1);
      move_group_->setNumPlanningAttempts(10);
      move_group_->setGoalTolerance(0.01);
      auto const [success, plan] = [&]() {
        moveit::planning_interface::MoveGroupInterface::Plan plan;
        auto const ok = move_group_->plan(plan);
        return std::make_pair(ok, plan);
      }();
    }
    auto end_time = std::chrono::high_resolution_clock::now();
    RCLCPP_INFO(get_logger(), "Time elapsed: %f ms",
                std::chrono::duration<double, std::milli>(end_time - start_time).count());

    // if (success) {
    //   RCLCPP_INFO(get_logger(), "Plan succeeded");
    //   // print plan
    //   for (auto const &waypoint : plan.trajectory_.joint_trajectory.points) {
    //     for (auto const &position : waypoint.positions) {
    //       RCLCPP_INFO(get_logger(), "%f", position);
    //     }
    //   }
    // } else {
    //   RCLCPP_ERROR(get_logger(), "Plan failed");
    // }
  }

 private:
  std::shared_ptr<moveit::planning_interface::MoveGroupInterface> move_group_;
};

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<FluoriteArm>());
  rclcpp::shutdown();
  return 0;
}