
#include <moveit/robot_model_loader/robot_model_loader.h>
#include <moveit/robot_model/robot_model.h>
#include <moveit/robot_state/robot_state.h>

#include <visualization_msgs/msg/marker.hpp>

int main(int argc, char** argv) {
  rclcpp::init(argc, argv);
  rclcpp::NodeOptions node_options;

  node_options.automatically_declare_parameters_from_overrides(true);
  auto node = rclcpp::Node::make_shared("robot_model_and_state_tutorial", node_options);
  const auto& LOGGER = node->get_logger();
  auto marker_publisher =
      node->create_publisher<visualization_msgs::msg::Marker>("visualization_marker", rclcpp::QoS(10));
  // clear markers
  marker_publisher->publish(visualization_msgs::msg::Marker());

  robot_model_loader::RobotModelLoader robot_model_loader(node);
  const moveit::core::RobotModelPtr& kinematic_model = robot_model_loader.getModel();
  RCLCPP_INFO(LOGGER, "Model frame: %s", kinematic_model->getModelFrame().c_str());

  moveit::core::RobotStatePtr robot_state(new moveit::core::RobotState(kinematic_model));
  robot_state->setToDefaultValues();
  const moveit::core::JointModelGroup* joint_model_group = kinematic_model->getJointModelGroup("fluorite_arm");

  const std::vector<std::string>& joint_names = joint_model_group->getVariableNames();

  // Get Joint Values
  // ^^^^^^^^^^^^^^^^
  // We can retrieve the current set of joint values stored in the state for the Panda arm.
  std::vector<double> joint_values;
  robot_state->copyJointGroupPositions(joint_model_group, joint_values);
  for (std::size_t i = 0; i < joint_names.size(); ++i) {
    RCLCPP_INFO(LOGGER, "Joint %s: %f", joint_names[i].c_str(), joint_values[i]);
  }

  // Joint Limits
  // ^^^^^^^^^^^^
  // setJointGroupPositions() does not enforce joint limits by itself, but a call to enforceBounds() will do it.
  /* Set one joint in the Panda arm outside its joint limit */
  joint_values[0] = 5.57;
  robot_state->setJointGroupPositions(joint_model_group, joint_values);

  /* Check whether any joint is outside its joint limits */
  RCLCPP_INFO_STREAM(LOGGER, "Current state is " << (robot_state->satisfiesBounds() ? "valid" : "not valid"));

  /* Enforce the joint limits for this state and check again*/
  robot_state->enforceBounds();
  RCLCPP_INFO_STREAM(LOGGER, "Current state is " << (robot_state->satisfiesBounds() ? "valid" : "not valid"));

  // Forward Kinematics
  // ^^^^^^^^^^^^^^^^^^
  // Now, we can compute forward kinematics for a set of random joint
  // values. Note that we would like to find the pose of the
  // "panda_link8" which is the most distal link in the
  // "panda_arm" group of the robot.
  // robot_state->setToRandomPositions(joint_model_group);
  robot_state->setToDefaultValues();
  const Eigen::Isometry3d& end_effector_state = robot_state->getGlobalLinkTransform("end_yaw_2_link");
  const auto target_end_eff_state = [&] {
    Eigen::Isometry3d target_end_eff_state = end_effector_state;
    target_end_eff_state.translation().x() += 0.1;
    target_end_eff_state.translation().y() -= 0.1;
    target_end_eff_state.translation().z() += 0.1;
    return target_end_eff_state;
  }();

  /* Print end-effector pose. Remember that this is in the model frame */
  RCLCPP_INFO_STREAM(LOGGER, "Translation: \n" << end_effector_state.translation() << "\n");
  RCLCPP_INFO_STREAM(LOGGER, "Rotation: \n" << end_effector_state.rotation() << "\n");

  // draw end-effector
  const auto marker = [&] {
    visualization_msgs::msg::Marker marker;
    marker.header.frame_id = "base_yaw_link";
    marker.header.stamp = node->now();
    marker.ns = "end_effector";
    marker.id = 0;
    marker.type = visualization_msgs::msg::Marker::SPHERE;
    marker.action = visualization_msgs::msg::Marker::ADD;
    marker.pose.position.x = end_effector_state.translation().x() + 0.1;
    marker.pose.position.y = end_effector_state.translation().y() - 0.1;
    marker.pose.position.z = end_effector_state.translation().z() + 0.1;
    marker.pose.orientation.x = 0.0;
    marker.pose.orientation.y = 0.0;
    marker.pose.orientation.z = 0.0;
    marker.pose.orientation.w = 1.0;
    marker.scale.x = 0.05;
    marker.scale.y = 0.05;
    marker.scale.z = 0.05;
    marker.color.a = 1.0;  // Don't forget to set the alpha!
    marker.color.r = 0.0;
    marker.color.g = 1.0;
    marker.color.b = 0.0;
    return marker;
  }();
  marker_publisher->publish(marker);

  // Inverse Kinematics
  // ^^^^^^^^^^^^^^^^^^
  // We can now solve inverse kinematics (IK) for the Panda robot.
  // To solve IK, we will need the following:
  //
  //  * The desired pose of the end-effector (by default, this is the last link in the "panda_arm" chain):
  //    end_effector_state that we computed in the step above.
  //  * The timeout: 0.1 s
  double timeout = 0.1;
  bool found_ik = robot_state->setFromIK(joint_model_group, target_end_eff_state, timeout);

  // Now, we can print out the IK solution (if found):
  if (found_ik) {
    robot_state->copyJointGroupPositions(joint_model_group, joint_values);
    for (std::size_t i = 0; i < joint_names.size(); ++i) {
      RCLCPP_INFO(LOGGER, "Joint %s: %f", joint_names[i].c_str(), joint_values[i]);
    }
  } else {
    RCLCPP_INFO(LOGGER, "Did not find IK solution");
  }

  // Get the Jacobian
  // ^^^^^^^^^^^^^^^^
  // We can also get the Jacobian from the
  // :moveit_codedir:`RobotState<moveit_core/robot_state/include/moveit/robot_state/robot_state.hpp>`.
  Eigen::Vector3d reference_point_position(0.0, 0.0, 0.0);
  Eigen::MatrixXd jacobian;
  robot_state->getJacobian(joint_model_group, robot_state->getLinkModel(joint_model_group->getLinkModelNames().back()),
                           reference_point_position, jacobian);
  RCLCPP_INFO_STREAM(LOGGER, "Jacobian: \n" << jacobian << "\n");
  // END_TUTORIAL

  rclcpp::shutdown();
  return 0;
}