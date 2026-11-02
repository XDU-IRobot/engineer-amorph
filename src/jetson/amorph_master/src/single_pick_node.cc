// 单次抓取：收到触发信号后，从最近一帧视觉检测结果里挑最近的能量单元，拟合架面后分段规划出抓取轨迹。
#include <array>
#include <atomic>
#include <cmath>
#include <memory>
#include <mutex>
#include <optional>
#include <string>
#include <thread>
#include <utility>

#include <Eigen/Geometry>
#include <console_bridge/console.h>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <moveit/moveit_cpp/moveit_cpp.hpp>
#include <moveit/moveit_cpp/planning_component.hpp>
#include <moveit/planning_scene/planning_scene.hpp>
#include <moveit/planning_scene_monitor/planning_scene_monitor.hpp>
#include <rclcpp/executors/multi_threaded_executor.hpp>
#include <rclcpp/node_options.hpp>
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/empty.hpp>
#include <tf2/exceptions.h>
#include <tf2/time.h>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>
#include <vision_msgs/msg/detection3_d_array.hpp>

#include "rack_plane.hpp"
#include "visualization.hpp"

namespace amorph_master {

/// @brief 视觉节点输出能量单元位姿，这里挑最近的一个，沿架面法向接近、夹取后沿主轴抽出。
/// 规划由外部空消息触发，避免每帧检测结果都重启一次规划。目前只规划并发布轨迹，不执行。
class SinglePickNode : public rclcpp::Node {
 public:
  explicit SinglePickNode(
      const rclcpp::NodeOptions &options =
          rclcpp::NodeOptions().allow_undeclared_parameters(true).automatically_declare_parameters_from_overrides(true))
      : rclcpp::Node{"single_pick_node", options}, tf_buffer_(get_clock()), tf_listener_(tf_buffer_) {}

  ~SinglePickNode() override;

  /// @brief 创建 moveit_cpp、机械臂规划组件和可视化模块，开始监听检测结果与抓取触发。
  void Start() {
    rclcpp::Node::SharedPtr node = shared_from_this();
    executor_.add_node(node);
    executor_thread_ = std::thread([this]() { executor_.spin(); });

    moveit_cpp_ = std::make_shared<moveit_cpp::MoveItCpp>(node);
    arm_ = std::make_shared<moveit_cpp::PlanningComponent>(kGroupName, moveit_cpp_);
    visualization_ = std::make_unique<Visualization>(node);

    const std::string detections_topic = declare_parameter("detections_topic", "eu_detections");
    const std::string trigger_topic = declare_parameter("trigger_topic", "single_pick_trigger");
    detection_subscription_ = create_subscription<vision_msgs::msg::Detection3DArray>(
        detections_topic, rclcpp::SensorDataQoS(),
        [this](const vision_msgs::msg::Detection3DArray::ConstSharedPtr &message) { OnDetections(*message); });
    trigger_subscription_ = create_subscription<std_msgs::msg::Empty>(
        trigger_topic, rclcpp::QoS(1), [this](const std_msgs::msg::Empty::ConstSharedPtr &) { OnTrigger(); });
  }

 private:
  /// @brief 与其它线程的抓取流程互斥，处理期间到达的检测结果直接丢掉。
  static std::optional<vision_msgs::msg::Detection3D> Nearest(const vision_msgs::msg::Detection3DArray &detections) {
    std::optional<vision_msgs::msg::Detection3D> nearest;
    double nearest_distance = 0.0;
    for (const vision_msgs::msg::Detection3D &detection : detections.detections) {
      // 检测结果在相机光学系下，原点到位姿的距离即到相机的距离
      const geometry_msgs::msg::Point &position = detection.bbox.center.position;
      const double distance = std::hypot(std::hypot(position.x, position.y), position.z);
      if (!nearest.has_value() || distance < nearest_distance) {
        nearest = detection;
        nearest_distance = distance;
      }
    }
    return nearest;
  }

  /// @brief 查询 source 在 target 下的位姿，失败时返回 nullopt。
  std::optional<Eigen::Isometry3d> LookupTransform(const std::string &target, const std::string &source) {
    try {
      const geometry_msgs::msg::TransformStamped transform =
          tf_buffer_.lookupTransform(target, source, tf2::TimePointZero);
      Eigen::Isometry3d pose = Eigen::Isometry3d::Identity();
      const auto &rotation = transform.transform.rotation;
      pose.linear() = Eigen::Quaterniond{rotation.w, rotation.x, rotation.y, rotation.z}.toRotationMatrix();
      const auto &translation = transform.transform.translation;
      pose.translation() = Eigen::Vector3d{translation.x, translation.y, translation.z};
      return pose;
    } catch (const tf2::TransformException &error) {
      RCLCPP_WARN(get_logger(), "lookup %s -> %s failed: %s", target.c_str(), source.c_str(), error.what());
      return std::nullopt;
    }
  }

  /// @brief 每帧检测结果都重新拟合一次架面并发布；拟合结果与这帧检测结果绑定，抓取时直接用。
  void OnDetections(const vision_msgs::msg::Detection3DArray &detections) {
    std::optional<Plane> plane = FitRackPlane(detections);
    if (plane.has_value()) {
      visualization_->PublishPlane(detections.header.frame_id, plane->center, plane->normal, kPlaneSize,
                                   {0.2f, 0.8f, 1.0f, 0.4f}, kPlaneTopic);
    }
    std::lock_guard<std::mutex> lock(detections_mutex_);
    latest_detections_ = detections;
    latest_plane_ = std::move(plane);
  }

  /// @brief 触发一次规划，用的是最近收到的一帧检测结果。
  void OnTrigger() {
    bool expected = false;
    if (!picking_.compare_exchange_strong(expected, true)) {
      RCLCPP_WARN(get_logger(), "previous pick still running, trigger ignored");
      return;
    }
    std::optional<vision_msgs::msg::Detection3DArray> detections;
    std::optional<Plane> plane;
    {
      std::lock_guard<std::mutex> lock(detections_mutex_);
      detections = std::move(latest_detections_);
      plane = std::move(latest_plane_);
    }
    if (!detections.has_value()) {
      RCLCPP_WARN(get_logger(), "no detection received yet, trigger ignored");
      picking_ = false;
      return;
    }
    // 架面定不出方向就不做，宁可这次不抓
    if (!plane.has_value()) {
      RCLCPP_WARN(get_logger(), "plane fit failed on %zu detections, skip this pick", detections->detections.size());
      picking_ = false;
      return;
    }
    Pick(*detections, *plane);
    picking_ = false;
  }

  void Pick(const vision_msgs::msg::Detection3DArray &detections, const Plane &plane) {
    // 1. 选距离相机最近的那个能量单元作为抓的目标
    const std::optional<vision_msgs::msg::Detection3D> target = Nearest(detections);
    RCLCPP_INFO(get_logger(), "pick triggered on %zu detections, nearest is %s", detections.detections.size(),
                target.has_value() ? target->id.c_str() : "none");
    if (!target.has_value()) {
      RCLCPP_INFO_THROTTLE(get_logger(), *get_clock(), 2000, "no energy unit detected");
      return;
    }

    // 2. 取相机到规划坐标系（world）的变换，后面所有位姿都要换到这个系下
    const std::string model_frame = moveit_cpp_->getRobotModel()->getModelFrame();
    const std::optional<Eigen::Isometry3d> camera = LookupTransform(model_frame, detections.header.frame_id);
    if (!camera.has_value()) {
      return;
    }

    // 3. 把目标位姿、主轴、架面法向量都从相机系换到 world
    const geometry_msgs::msg::Pose &target_pose = target->bbox.center;
    const Eigen::Quaterniond target_orientation{target_pose.orientation.w, target_pose.orientation.x,
                                                target_pose.orientation.y, target_pose.orientation.z};
    const Eigen::Vector3d camera_position{target_pose.position.x, target_pose.position.y, target_pose.position.z};
    const Eigen::Vector3d position = camera->linear() * camera_position + camera->translation();
    // 主轴：能量单元自身的 z 轴，抽出时沿它平移
    const Eigen::Vector3d axis = (camera->linear() * (target_orientation * Eigen::Vector3d::UnitZ())).normalized();
    // 架面法向：已保证指向相机一侧，也就是从架子里伸出来的方向
    const Eigen::Vector3d normal = (camera->linear() * plane.normal).normalized();

    // 4. 构造夹爪姿态。URDF 里两指沿 l6_l 的 +Z 伸出，所以 tool_z 要从夹爪指向能量单元，即朝架内。
    //    Y 轴对齐主轴让两指夹在圆柱侧面（X 轴为两指连线，必然垂直于主轴）。
    const Eigen::Vector3d tool_z = -normal;
    // 把主轴投影到垂直于 tool_z 的平面上，得到夹爪的 Y 轴
    Eigen::Vector3d tool_y = axis - axis.dot(tool_z) * tool_z;
    if (tool_y.norm() < kMinToolAxisNorm) {
      // 主轴与接近方向几乎平行，投影退化，随便取一个垂直轴避免除零
      tool_y = tool_z.unitOrthogonal();
    }
    tool_y.normalize();
    // 由 Y、Z 叉出 X 轴，保证右手正交基 (X = Y x Z)
    Eigen::Matrix3d tool_rotation;
    tool_rotation.col(0) = tool_y.cross(tool_z);
    tool_rotation.col(1) = tool_y;
    tool_rotation.col(2) = tool_z;
    const Eigen::Quaterniond tool_orientation{tool_rotation};

    // 夹爪在各段轨迹中姿态不变，只有位置在动，所以这里固定好朝向，后面只换点
    const auto make_pose = [&](const Eigen::Vector3d &point) {
      geometry_msgs::msg::PoseStamped pose;
      pose.header.frame_id = model_frame;
      pose.pose.position.x = point.x();
      pose.pose.position.y = point.y();
      pose.pose.position.z = point.z();
      pose.pose.orientation.x = tool_orientation.x();
      pose.pose.orientation.y = tool_orientation.y();
      pose.pose.orientation.z = tool_orientation.z();
      pose.pose.orientation.w = tool_orientation.w();
      return pose;
    };

    // 把定下来的末端位姿画出来，方便在 RViz 里核对夹爪朝向
    // 夹爪指尖在 l6_l 的 +Z 侧，抓取时 l6_l 原点沿 -tool_z 退开一个抓取深度，指尖才落在能量单元中心
    const Eigen::Vector3d grasp_position = position - kGraspDepth * tool_z;
    const Eigen::Vector3d pregrasp_position = grasp_position - kApproachOffset * tool_z;
    const auto publish_frame = [&](const Eigen::Vector3d &point, int marker_id) {
      Eigen::Isometry3d pose = Eigen::Isometry3d::Identity();
      pose.linear() = tool_rotation;
      pose.translation() = point;
      visualization_->PublishFrame(model_frame, pose, kFrameLength, kFrameTopic, marker_id);
    };
    publish_frame(pregrasp_position, 0);
    publish_frame(grasp_position, 1);

    // 把这两个位姿用 IK 解成关节角发出去，RViz 里加两个 RobotState 显示就能直接看到夹爪摆上去长什么样
    PublishRobotStateForPose(make_pose(pregrasp_position), kPregraspStateTopic, {0.2f, 0.8f, 1.0f, 1.0f});
    PublishRobotStateForPose(make_pose(grasp_position), kGraspStateTopic, {1.0f, 0.6f, 0.1f, 1.0f});

    // 5. 定抽出方向：能量单元高于 tag 就沿主轴正向抽出，低于则反向
    const std::optional<Eigen::Isometry3d> tag = LookupTransform(model_frame, kTagFrame);
    if (!tag.has_value()) {
      RCLCPP_WARN(get_logger(), "lookup of tag tf %s -> %s failed, pick aborted", model_frame.c_str(), kTagFrame);
      return;
    }
    const double extract_sign = position.z() > tag->translation().z() ? 1.0 : -1.0;
    RCLCPP_INFO(get_logger(), "extract %s", extract_sign > 0.0 ? "upward" : "downward");

    // 6. 规划过程中机器人不动，用 state 在各段之间接力：上一段的终点就是下一段的起点。
    //    各段同时拼进一条完整轨迹，最后统一发布，RViz 里能一次看完整动作。
    moveit::core::RobotState state{*moveit_cpp_->getCurrentState()};
    const moveit::core::RobotState initial_state{state};
    std::optional<robot_trajectory::RobotTrajectory> full_trajectory;

    // 7. 预抓取：从当前位置 PTP 到架外，夹爪正对能量单元侧面
    if (!PlanPose(make_pose(pregrasp_position), "pilz_ptp", state, full_trajectory)) {
      return;
    }
    // 8. 沿法向反向直线推进 7cm，指尖落到能量单元侧面
    if (!PlanPose(make_pose(grasp_position), "pilz_lin", state, full_trajectory)) {
      return;
    }
    // 9. 沿主轴直线抽出 15cm（此时夹爪应已夹住物体）
    if (!PlanPose(make_pose(grasp_position + extract_sign * kExtractDistance * axis), "pilz_lin", state,
                  full_trajectory)) {
      return;
    }
    // 10. PTP 收回初始位姿
    if (PlanJoints(initial_state, "pilz_ptp", state, full_trajectory)) {
      visualization_->PublishTrajectory(*full_trajectory);
      full_trajectory->print(std::cout);
    }
  }

  /// @brief 用 IK 解出末端位姿对应的关节角，发布给 RViz 的 RobotState 插件渲染，方便核对夹爪朝向。
  void PublishRobotStateForPose(const geometry_msgs::msg::PoseStamped &goal, const std::string &topic,
                                const std::array<float, 4> &color) {
    moveit::core::RobotState state{*moveit_cpp_->getCurrentState()};
    if (!state.setFromIK(state.getJointModelGroup(kGroupName), goal.pose, kTipLink, kIkTimeout)) {
      RCLCPP_WARN(get_logger(), "IK failed for %s, robot state not visualized", topic.c_str());
      return;
    }
    state.update();
    visualization_->PublishRobotState(state, topic, {kTipLink, kGripperLeftLink, kGripperRightLink}, color);
  }

  bool PlanPose(const geometry_msgs::msg::PoseStamped &goal, const std::string &parameters_namespace,
                moveit::core::RobotState &state, std::optional<robot_trajectory::RobotTrajectory> &full_trajectory) {
    arm_->setStartState(state);
    arm_->setGoal(goal, kTipLink);
    return PlanAndAdvance(parameters_namespace, state, full_trajectory);
  }

  bool PlanJoints(const moveit::core::RobotState &goal, const std::string &parameters_namespace,
                  moveit::core::RobotState &state, std::optional<robot_trajectory::RobotTrajectory> &full_trajectory) {
    arm_->setStartState(state);
    arm_->setGoal(goal);
    return PlanAndAdvance(parameters_namespace, state, full_trajectory);
  }

  /// @brief 规划一段轨迹，成功后把它接进 full_trajectory，并把 state 推进到轨迹终点供下一段接力。
  bool PlanAndAdvance(const std::string &parameters_namespace, moveit::core::RobotState &state,
                      std::optional<robot_trajectory::RobotTrajectory> &full_trajectory) {
    moveit_cpp::PlanningComponent::PlanRequestParameters parameters;
    parameters.load(shared_from_this(), parameters_namespace);

    // 调试阶段先不看碰撞：URDF 里夹爪与腕部本身有干涉，正常抓取也会被判成自碰撞。
    // Pilz 用场景自带的 ACM 做 IK 的可行性检查，这里把 ACM 里所有 link 之间都设为允许碰撞。
    // 注意 setEntry(true) 只改 entries_ 里已存在的项，SRDF 没列出的组合走 default_entries_，
    // 所以必须逐个 link 设 setDefaultEntry，否则 l4_l 与夹爪这种未列出的组合仍会被判成碰撞。
    planning_scene::PlanningScenePtr scene = moveit_cpp_->getPlanningSceneMonitorNonConst()->copyPlanningScene();
    collision_detection::AllowedCollisionMatrix &acm = scene->getAllowedCollisionMatrixNonConst();
    for (const moveit::core::LinkModel *link : scene->getRobotModel()->getLinkModelsWithCollisionGeometry()) {
      acm.setDefaultEntry(link->getName(), true);
    }
    acm.setEntry(true);

    const planning_interface::MotionPlanResponse result = arm_->plan(parameters, scene);
    if (!result) {
      RCLCPP_WARN(get_logger(), "%s planning failed (code %d)", parameters_namespace.c_str(), result.error_code.val);
      return false;
    }
    // 段间不停顿，dt 传 0；首段没有前一段，直接整体拷贝过来
    if (full_trajectory.has_value()) {
      full_trajectory->append(*result.trajectory, 0.0);
    } else {
      full_trajectory = *result.trajectory;
    }
    state = result.trajectory->getLastWayPoint();
    return true;
  }

  static constexpr const char *kGroupName = "arm_l";
  static constexpr const char *kTipLink = "l6_l";
  static constexpr const char *kGripperLeftLink = "l_gripper_l_l";
  static constexpr const char *kGripperRightLink = "l_gripper_r_l";
  static constexpr const char *kTagFrame = "tag36h11:1";
  static constexpr const char *kPlaneTopic = "rack_plane";
  static constexpr const char *kFrameTopic = "pregrasp_frame";
  static constexpr const char *kPregraspStateTopic = "pregrasp_robot_state";
  static constexpr const char *kGraspStateTopic = "grasp_robot_state";
  static constexpr double kIkTimeout = 0.1;    ///< 仅用于可视化时的 IK 超时，单位秒
  static constexpr double kPlaneSize = 1.0;    ///< 发布平面的边长，单位米
  static constexpr double kFrameLength = 0.1;  ///< 预抓取坐标系三根轴的长度，单位米
  static constexpr double kGraspDepth = 0.10;  ///< l6_l 原点到两指中间抓取点的距离，单位米
  static constexpr double kApproachOffset = 0.07;  ///< 预抓取点相对抓取点在接近方向上的后退量，单位米
  static constexpr double kExtractDistance = 0.15;  ///< 沿主轴抽出能量单元的距离，单位米
  static constexpr double kMinToolAxisNorm = 1e-3;  ///< 主轴与接近方向几乎平行时无法定向，改用任意垂直轴

  rclcpp::executors::MultiThreadedExecutor executor_{rclcpp::ExecutorOptions(), 2};
  std::thread executor_thread_;
  tf2_ros::Buffer tf_buffer_;
  tf2_ros::TransformListener tf_listener_;
  moveit_cpp::MoveItCppPtr moveit_cpp_;
  moveit_cpp::PlanningComponentPtr arm_;
  std::unique_ptr<Visualization> visualization_;
  rclcpp::Subscription<vision_msgs::msg::Detection3DArray>::SharedPtr detection_subscription_;
  rclcpp::Subscription<std_msgs::msg::Empty>::SharedPtr trigger_subscription_;
  std::mutex detections_mutex_;
  std::optional<vision_msgs::msg::Detection3DArray> latest_detections_;
  std::optional<Plane> latest_plane_;
  std::atomic<bool> picking_{false};
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

  // Start() 已经在后台线程里 spin，这里只等退出信号
  while (rclcpp::ok()) {
    rclcpp::sleep_for(std::chrono::milliseconds(100));
  }

  // MoveIt 内部有些插件对象比创建它们的 ClassLoader 活得更久，退出时 class_loader 会刷一串
  // "Attempting to unload library while objects created by this loader exist in the heap" 警告。
  // 这些警告无害，退出前直接关掉 console_bridge 的 WARN 及以下输出。
  console_bridge::setLogLevel(console_bridge::CONSOLE_BRIDGE_LOG_ERROR);

  rclcpp::shutdown();
  return 0;
}
