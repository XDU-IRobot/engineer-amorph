#include "pick_energy_unit_task.hpp"

#include <Eigen/Geometry>
#include <geometry_msgs/msg/pose.hpp>
#include <tf2_eigen/tf2_eigen.hpp>

#include "amorph_host/pick_place_task_params.hpp"  // 复用 pick_place_task 的参数，或者新建一个
#include "mtc/object_factory.hpp"
#include "mtc/moveit_helper.hpp"

#include "amorph_host/logger.hpp"

namespace amorph {

namespace mtc = moveit::task_constructor;

PickEnergyUnitTask::PickEnergyUnitTask(const std::string& task_name) : task_name_(task_name) {}

bool PickEnergyUnitTask::init(const rclcpp::Node::SharedPtr& node, const geometry_msgs::msg::Pose& object_pose) {
  L::I("Initializing PickEnergyUnitTask pipeline");

  // 这里暂时复用 pick_place_task 的参数，实际项目中建议新建一个 parameters.yaml
  const auto param_listener = std::make_shared<pick_place_task::ParamListener>(node, "pick_place_task");
  const auto params = param_listener->get_params();

  // Add object to planning scene
  rclcpp::sleep_for(std::chrono::microseconds(100));  // Wait for ApplyPlanningScene service
  auto& mvh = MoveitHelper::GetInstance();

  // 清空场景，确保没有残留物体
  mvh.ClearPlanningScene();

  // 生成能量单元物体
  // 注意：这里使用传入的 object_pose
  mvh.SpawnObject(object_factory::EnergyUnit(params.object_name, params.object_reference_frame, object_pose));

  // Reset ROS introspection before constructing the new object
  task_.reset();
  task_.reset(new moveit::task_constructor::Task());

  // Individual movement stages are collected within the Task object
  mtc::Task& t = *task_;
  t.stages()->setName(task_name_);
  t.loadRobotModel(node);

  /* Create planners used in various stages. */
  // Sampling planner
  auto sampling_planner = std::make_shared<mtc::solvers::PipelinePlanner>(node);
  sampling_planner->setProperty("goal_joint_tolerance", 1e-5);

  // Cartesian planner
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

    // Verify that object is not attached
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
    initial_state_ptr = stage.get();
    t.add(std::move(stage));
  }

  /****************************************************
   *               Move to Pick                       *
   ***************************************************/
  {
    mtc::stages::Connect::GroupPlannerVector planners = {{params.arm_group_name, sampling_planner},
                                                         {params.hand_group_name, sampling_planner}};
    auto stage = std::make_unique<mtc::stages::Connect>("move to pick", planners);
    stage->setTimeout(5.0);
    stage->properties().configureInitFrom(mtc::Stage::PARENT);
    t.add(std::move(stage));
  }

  /****************************************************
   *               Pick Object                        *
   ***************************************************/
  mtc::Stage* pick_stage_ptr = nullptr;
  {
    auto grasp = std::make_unique<mtc::SerialContainer>("pick object");
    t.properties().exposeTo(grasp->properties(), {"eef", "hand", "group", "ik_frame"});
    grasp->properties().configureInitFrom(mtc::Stage::PARENT, {"eef", "hand", "group", "ik_frame"});

    /****************************************************
     *               Approach Object                    *
     ***************************************************/
    {
      auto stage = std::make_unique<mtc::stages::MoveRelative>("approach object", cartesian_planner);
      stage->properties().set("marker_ns", "approach_object");
      stage->properties().set("link", params.hand_frame);
      stage->properties().configureInitFrom(mtc::Stage::PARENT, {"group"});
      stage->setMinMaxDistance(params.approach_object_min_dist, params.approach_object_max_dist);

      // Set hand forward direction
      geometry_msgs::msg::Vector3Stamped vec;
      vec.header.frame_id = params.hand_frame;
      vec.vector.z = 1.0;
      stage->setDirection(vec);
      grasp->insert(std::move(stage));
    }

    /****************************************************
     *               Allow Collision (hand object)      *
     ***************************************************/
    {
      auto stage = std::make_unique<mtc::stages::ModifyPlanningScene>("allow collision (hand,object)");
      stage->allowCollisions(
          params.object_name,
          t.getRobotModel()->getJointModelGroup(params.hand_group_name)->getLinkModelNamesWithCollisionGeometry(),
          true);
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
      auto stage = std::make_unique<mtc::stages::MoveTo>("close hand", sampling_planner);
      stage->setGroup(params.hand_group_name);
      stage->setGoal(params.hand_close_pose);
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

      // Set upward direction
      geometry_msgs::msg::Vector3Stamped vec;
      vec.header.frame_id = "world";
      vec.vector.z = 1.0;
      stage->setDirection(vec);
      grasp->insert(std::move(stage));
    }

    pick_stage_ptr = grasp.get();
    t.add(std::move(grasp));
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
    L::I("Planning succeeded, found %d solutions", task_->numSolutions());
    return true;
  } else {
    L::E("Planning failed, no solutions found");
    return false;
  }
}

bool PickEnergyUnitTask::execute() {
  L::I("Executing task solution");
  if (task_->numSolutions() > 0) {
    const auto& solution = task_->solutions().front();
    // TODO: Execute the solution
    // moveit_task_constructor_msgs::action::ExecuteTaskSolution::Goal goal;
    // goal.solution = *solution;
    // ...
    return true;
  }
  return false;
}

}  // namespace amorph
