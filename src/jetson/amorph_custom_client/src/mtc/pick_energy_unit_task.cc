#include "amorph_custom_client/mtc/pick_energy_unit_task.hpp"

#include <Eigen/Geometry>

#include "amorph_custom_client/pick_energy_unit_task_params.hpp"
#include "amorph_custom_client/mtc/object_factory.hpp"
#include "amorph_custom_client/mtc/moveit_helper.hpp"
#include "amorph_custom_client/mtc/logger.hpp"

#include <moveit/task_constructor/trajectory_execution_info.h>

namespace amorph {

namespace mtc = moveit::task_constructor;

PickEnergyUnitTask::PickEnergyUnitTask(rclcpp::Node* node, const std::string& task_name)
    : node_(node),
      task_name_(task_name) {}

bool PickEnergyUnitTask::Setup() {
  L::I("Setting up PickEnergyUnitTask pipeline");

  auto node = node_->shared_from_this();

  // 使用外部调用者通过 SetObjectPose 设置的位姿
  const auto& object_pose = object_pose_;

  const auto param_listener =
      std::make_shared<pick_energy_unit_task::ParamListener>(node, "pick_energy_unit_task");
  const auto params = param_listener->get_params();

  // Prepare execution info for arm and hand using controller names
  mtc::TrajectoryExecutionInfo arm_exec_info;
  arm_exec_info.controller_names = {params.arm_controller_name};
  mtc::TrajectoryExecutionInfo hand_exec_info;
  hand_exec_info.controller_names = {params.hand_controller_name};
  mtc::TrajectoryExecutionInfo both_exec_info;
  both_exec_info.controller_names = {params.arm_controller_name, params.hand_controller_name};

  // Add object to planning scene
  rclcpp::sleep_for(std::chrono::microseconds(100));  // Wait for ApplyPlanningScene service
  auto& mvh = MoveitHelper::GetInstance();

  // 清空场景，确保没有残留物体
  mvh.ClearPlanningScene();

  // 生成能量单元
  mvh.SpawnObject(
      object_factory::EnergyUnit(params.object_name, params.object_reference_frame, object_pose));

  // 清空旧 task，构造新 task
  task_.reset();
  task_.reset(new moveit::task_constructor::Task());

  mtc::Task& t = *task_;
  t.stages()->setName(task_name_);
  t.loadRobotModel(node);

  /* Create planners used in various stages. */
  auto sampling_planner = std::make_shared<mtc::solvers::PipelinePlanner>(node);
  sampling_planner->setProperty("goal_joint_tolerance", 1e-5);

  auto cartesian_planner = std::make_shared<mtc::solvers::CartesianPath>();
  cartesian_planner->setMaxVelocityScalingFactor(1.0);
  cartesian_planner->setMaxAccelerationScalingFactor(1.0);
  cartesian_planner->setStepSize(.01);

  // Set task properties
  t.setProperty("group", params.arm_group_name);
  t.setProperty("eef", params.eef_name);
  t.setProperty("hand", params.hand_group_name);
  t.setProperty("hand_grasping_frame", params.hand_frame);
  t.setProperty("ik_frame", params.hand_frame);

  /****************************************************
   *               Current State                      *
   ***************************************************/
  {
    auto current_state = std::make_unique<mtc::stages::CurrentState>("current state");

    auto applicability_filter =
        std::make_unique<mtc::stages::PredicateFilter>("applicability test", std::move(current_state));
    applicability_filter->setPredicate([object = params.object_name](const mtc::SolutionBase& s, std::string& comment) {
      if (s.start()->scene()->getCurrentState().hasAttachedBody(object)) {
        comment = "object with id '" + object + "' is already attached and cannot be picked";
        return false;
      }
      return true;
    });
    t.add(std::move(applicability_filter));
  }

  /****************************************************
   *               Open Hand                          *
   ***************************************************/
  mtc::Stage* initial_state_ptr = nullptr;
  {
    auto stage = std::make_unique<mtc::stages::MoveTo>("open hand", sampling_planner);
    stage->setGroup(params.hand_group_name);
    stage->setGoal(params.hand_open_pose);
    stage->properties().set("trajectory_execution_info", hand_exec_info);
    initial_state_ptr = stage.get();
    t.add(std::move(stage));
  }

  /****************************************************
   *               Move to Pick                       *
   ***************************************************/
  {
    mtc::stages::Connect::GroupPlannerVector planners = {{params.arm_group_name, sampling_planner}};
    auto stage = std::make_unique<mtc::stages::Connect>("move to pick", planners);
    stage->setTimeout(5.0);
    stage->properties().configureInitFrom(mtc::Stage::PARENT);
    stage->properties().set("trajectory_execution_info", arm_exec_info);
    t.add(std::move(stage));
  }

  /****************************************************
   *               Pick Object                        *
   ***************************************************/
  {
    auto grasp = std::make_unique<mtc::SerialContainer>("pick object");
    t.properties().exposeTo(grasp->properties(), {"eef", "hand", "group", "ik_frame"});
    grasp->properties().configureInitFrom(mtc::Stage::PARENT, {"eef", "hand", "group", "ik_frame"});

    /****************************************************
     *     Allow Collision (hand,object) for grasp     *
     ***************************************************/
    {
      auto allow_collision_grasp =
          std::make_unique<mtc::stages::ModifyPlanningScene>("allow collision (hand,object) for grasp");
      allow_collision_grasp->allowCollisions(
          params.object_name,
          t.getRobotModel()->getJointModelGroup(params.hand_group_name)->getLinkModelNamesWithCollisionGeometry(),
          true);
      grasp->insert(std::move(allow_collision_grasp));
    }

    /****************************************************
     *               Approach Object                    *
     ***************************************************/
    {
      auto stage = std::make_unique<mtc::stages::MoveRelative>("approach object", cartesian_planner);
      stage->properties().set("marker_ns", "approach_object");
      stage->properties().set("link", params.hand_frame);
      stage->properties().configureInitFrom(mtc::Stage::PARENT, {"group"});
      stage->properties().set("trajectory_execution_info", both_exec_info);
      stage->setMinMaxDistance(params.approach_object_min_dist, params.approach_object_max_dist);

      geometry_msgs::msg::Vector3Stamped vec;
      vec.header.frame_id = params.hand_frame;
      vec.vector.y = -1.0;
      stage->setDirection(vec);
      grasp->insert(std::move(stage));
    }

    /****************************************************
     *               Allow Collision (hand object)      *
     ***************************************************/
    {
      auto stage = std::make_unique<mtc::stages::ModifyPlanningScene>("allow collision (hand,object)");
      stage->allowCollisions(params.object_name, true);
      grasp->insert(std::move(stage));
    }

    /****************************************************
     *               Generate Grasp Pose                *
     ***************************************************/
    {
      auto stage = std::make_unique<mtc::stages::GenerateGraspPose>("generate grasp pose");
      stage->properties().configureInitFrom(mtc::Stage::PARENT);
      stage->properties().set("marker_ns", "grasp_pose");
      stage->setPreGraspPose(params.hand_open_pose);
      stage->setObject(params.object_name);
      stage->setAngleDelta(M_PI / 12);
      stage->setMonitoredStage(initial_state_ptr);

      auto wrapper = std::make_unique<mtc::stages::ComputeIK>("grasp pose IK", std::move(stage));
      wrapper->setMaxIKSolutions(8);
      wrapper->setMinSolutionDistance(1.0);
      wrapper->setIKFrame(MoveitHelper::VectorToEigen(params.grasp_frame_transform), params.hand_frame);
      wrapper->properties().configureInitFrom(mtc::Stage::PARENT, {"eef", "group"});
      wrapper->properties().configureInitFrom(mtc::Stage::INTERFACE, {"target_pose"});
      grasp->insert(std::move(wrapper));
    }

    /****************************************************
     *               Close Hand                         *
     ***************************************************/
    {
      auto allow_collision_close =
          std::make_unique<mtc::stages::ModifyPlanningScene>("allow collision (hand,object) for close");
      allow_collision_close->allowCollisions(params.object_name, true);
      grasp->insert(std::move(allow_collision_close));
    }
    {
      auto stage = std::make_unique<mtc::stages::MoveTo>("close hand", sampling_planner);
      stage->setGroup(params.hand_group_name);
      stage->setGoal(params.hand_close_pose);
      stage->properties().set("trajectory_execution_info", hand_exec_info);
      grasp->insert(std::move(stage));
    }

    /****************************************************
     *               Attach Object                      *
     ***************************************************/
    {
      auto stage = std::make_unique<mtc::stages::ModifyPlanningScene>("attach object");
      stage->attachObject(params.object_name, params.hand_frame);
      grasp->insert(std::move(stage));
    }

    /****************************************************
     *               Lift Object                        *
     ***************************************************/
    {
      auto stage = std::make_unique<mtc::stages::MoveRelative>("lift object", cartesian_planner);
      stage->properties().configureInitFrom(mtc::Stage::PARENT, {"group"});
      stage->setMinMaxDistance(params.lift_object_min_dist, params.lift_object_max_dist);
      stage->setIKFrame(params.hand_frame);
      stage->properties().set("marker_ns", "lift_object");
      stage->properties().set("trajectory_execution_info", arm_exec_info);

      geometry_msgs::msg::Vector3Stamped vec;
      vec.header.frame_id = "energy_unit_selected";
      vec.vector.z = lift_upward_ ? 1.0 : -1.0;
      stage->setDirection(vec);
      grasp->insert(std::move(stage));
    }

    t.add(std::move(grasp));
  }

  /******************************************************
   *          Move to Home                              *
   *****************************************************/
  {
    auto stage = std::make_unique<mtc::stages::MoveTo>("move home", sampling_planner);
    stage->properties().configureInitFrom(mtc::Stage::PARENT, {"group"});
    stage->setGoal(params.arm_home_pose);
    stage->restrictDirection(mtc::stages::MoveTo::FORWARD);
    stage->properties().set("trajectory_execution_info", arm_exec_info);
    t.add(std::move(stage));
  }

  // prepare Task structure for planning
  try {
    t.init();
  } catch (mtc::InitStageException& e) {
    RCLCPP_ERROR_STREAM(rclcpp::get_logger("amorph_custom_client"), "Initialization failed: " << e);
    return false;
  }

  return true;
}

bool PickEnergyUnitTask::plan(std::size_t max_solutions) {
  L::I("Start planning task");
  try {
    task_->plan(max_solutions);
  } catch (const std::exception& e) {
    L::E("Planning failed with exception: %s", e.what());
    return false;
  }

  if (task_->numSolutions() > 0) {
    L::I("Planning succeeded, found %zu solutions", task_->numSolutions());
    return true;
  } else {
    L::E("Planning failed, no solutions found");
    return false;
  }
}

bool PickEnergyUnitTask::execute() {
  L::I("Executing task solution");
  if (task_->numSolutions() > 0) {
    return true;
  }
  return false;
}

}  // namespace amorph
