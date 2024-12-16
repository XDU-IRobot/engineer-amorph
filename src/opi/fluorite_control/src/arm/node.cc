
#include <rclcpp/rclcpp.hpp>
#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit_visual_tools/moveit_visual_tools.h>
#include <sensor_msgs/msg/joy.hpp>
#include <wiringPi.h>

#include <chrono>
#include <thread>

class FluoriteArm : public rclcpp::Node {
 public:
  FluoriteArm(const rclcpp::NodeOptions &options = rclcpp::NodeOptions()
                                                       .allow_undeclared_parameters(true)
                                                       .automatically_declare_parameters_from_overrides(true))
      : Node("fluorite_arm", options) {
    // move_group_ =
    //     std::make_shared<moveit::planning_interface::MoveGroupInterface>(rclcpp::Node::SharedPtr(this), "arm");
    // TryPlan();
    this->InitGpio();
    this->Remote();
  }

  void InitGpio() {
    using namespace std::chrono_literals;
    wiringPiSetup();
    pinMode(2, PWM_OUTPUT);
    pwmSetRange(2, 1024);
    pwmWrite(2, 0);
  }

  void Remote() {
    remote_sub_ = this->create_subscription<sensor_msgs::msg::Joy>("/rm2_remote/joy", rclcpp::SensorDataQoS(),
                                                                   [&](const sensor_msgs::msg::Joy::SharedPtr msg) {
                                                                     RCLCPP_INFO(get_logger(), "%f", msg->axes[3]);
                                                                     pwmWrite(2, std::abs(msg->axes[3]) * 1024.0);
                                                                   });
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

    auto moveit_visual_tools = moveit_visual_tools::MoveItVisualTools{
        rclcpp::Node::SharedPtr{this}, "base_link", rviz_visual_tools::RVIZ_MARKER_TOPIC, move_group_->getRobotModel()};
    moveit_visual_tools.deleteAllMarkers();
    moveit_visual_tools.loadRemoteControl();
    auto const draw_title = [&moveit_visual_tools](auto text) {
      auto const text_pose = [] {
        auto msg = Eigen::Isometry3d::Identity();
        msg.translation().z() = 1.0;  // Place text 1m above the base link
        return msg;
      }();
      moveit_visual_tools.publishText(text_pose, text, rviz_visual_tools::WHITE, rviz_visual_tools::XLARGE);
    };
    auto const prompt = [&moveit_visual_tools](auto text) { moveit_visual_tools.prompt(text); };
    auto const draw_trajectory_tool_path =
        [&moveit_visual_tools, jmg = move_group_->getRobotModel()->getJointModelGroup("arm")](auto const trajectory) {
          moveit_visual_tools.publishTrajectoryLine(trajectory, jmg);
        };

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

      if (success) {
        draw_trajectory_tool_path(plan.trajectory_);
        moveit_visual_tools.trigger();
        prompt("Press 'Next' in the RvizVisualToolsGui window to execute");
        draw_title("Executing");
        moveit_visual_tools.trigger();
        // move_group_->execute(plan);
      } else {
        draw_title("Planning Failed!");
        moveit_visual_tools.trigger();
        RCLCPP_ERROR(get_logger(), "Planning failed!");
      }
    }
    auto end_time = std::chrono::high_resolution_clock::now();
    RCLCPP_INFO(get_logger(), "Time elapsed: %f ms",
                std::chrono::duration<double, std::milli>(end_time - start_time).count());
  }

 private:
  rclcpp::Subscription<sensor_msgs::msg::Joy>::SharedPtr remote_sub_{nullptr};
  std::shared_ptr<moveit::planning_interface::MoveGroupInterface> move_group_;
};

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<FluoriteArm>());
  rclcpp::shutdown();
  return 0;
}