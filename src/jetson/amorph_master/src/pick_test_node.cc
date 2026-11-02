#include <memory>
#include <ratio>
#include <string>
#include <thread>
#include <vector>
#include <chrono>
#include <format>

#include <console_bridge/console.h>
#include <moveit/moveit_cpp/moveit_cpp.hpp>
#include <moveit/moveit_cpp/planning_component.hpp>
#include <rclcpp/executors/single_threaded_executor.hpp>
#include <rclcpp/node_options.hpp>
#include <rclcpp/rclcpp.hpp>

#include "visualization.hpp"

namespace amorph_master {

class SinglePickNode : public rclcpp::Node {
 public:
  explicit SinglePickNode(
      const rclcpp::NodeOptions &options =
          rclcpp::NodeOptions().allow_undeclared_parameters(true).automatically_declare_parameters_from_overrides(true))
      : rclcpp::Node{"single_pick_node", options} {}

  ~SinglePickNode() override;

  /// @brief 创建 moveit_cpp 和机械臂规划组件，并开始处理话题。
  void Start() {
    rclcpp::Node::SharedPtr node = shared_from_this();
    executor_.add_node(node);
    executor_thread_ = std::thread([this]() { executor_.spin(); });

    moveit_cpp_ = std::make_shared<moveit_cpp::MoveItCpp>(node);
    arm_ = std::make_shared<moveit_cpp::PlanningComponent>(kGroupName, moveit_cpp_);
    visualization_ = std::make_unique<Visualization>(node);
  }

  /// @brief 规划 arm_l 从当前状态到 target 关节位置的轨迹，parameters_namespace 选择使用哪一组规划器参数。
  void PlanArm(const std::vector<double> &target, const std::string &parameters_namespace) {
    moveit::core::RobotState goal{moveit_cpp_->getRobotModel()};
    goal.setJointGroupPositions(kGroupName, target);
    goal.update();

    arm_->setStartStateToCurrentState();
    arm_->setGoal(goal);

    moveit_cpp::PlanningComponent::PlanRequestParameters parameters;
    parameters.load(shared_from_this(), parameters_namespace);

    {
      planning_interface::MotionPlanResponse result;
      const auto start = std::chrono::steady_clock::now();
      for (int i = 0; i < 20; ++i) {
        result = arm_->plan(parameters);
      }
      const auto duration_ms =
          std::chrono::duration_cast<std::chrono::microseconds>(std::chrono::steady_clock::now() - start).count();
      std::string log{std::format("planning cost {} us", duration_ms / 20)};
      RCLCPP_INFO(get_logger(), "%s", log.c_str());
      RCLCPP_INFO(get_logger(), "Planning result code: %d", result.error_code.val);

      // if (result.trajectory != nullptr) {
      //   visualization_->PublishTrajectory(*result.trajectory);
      // }
    }
  }

 private:
  static constexpr const char *kGroupName = "arm_l";

  rclcpp::executors::SingleThreadedExecutor executor_;
  std::thread executor_thread_;
  moveit_cpp::MoveItCppPtr moveit_cpp_;
  moveit_cpp::PlanningComponentPtr arm_;
  std::unique_ptr<Visualization> visualization_;
};

SinglePickNode::~SinglePickNode() {
  executor_.cancel();
  executor_thread_.join();
}

}  // namespace amorph_master

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);

  auto node = std::make_shared<amorph_master::SinglePickNode>();
  node->Start();
  node->PlanArm({0.0, 2.1, 0.0, 0.0, 0.0, 0.0}, "pilz_ptp");
  // 让节点多活一会儿，等 RViz 的订阅建立后再退出
  rclcpp::sleep_for(std::chrono::seconds(5));

  // MoveIt 内部有些插件对象比创建它们的 ClassLoader 活得更久，退出时 class_loader 会刷一串
  // "Attempting to unload library while objects created by this loader exist in the heap" 警告。
  // 这些警告无害，退出前直接关掉 console_bridge 的 WARN 及以下输出。
  console_bridge::setLogLevel(console_bridge::CONSOLE_BRIDGE_LOG_ERROR);

  rclcpp::shutdown();
  return 0;
}
