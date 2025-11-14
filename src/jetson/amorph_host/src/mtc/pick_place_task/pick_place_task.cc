
#include "pick_place_task.hpp"

#include <Eigen/Geometry>
#include <geometry_msgs/msg/pose.hpp>
#include <tf2_eigen/tf2_eigen.hpp>

#include "amorph_host/pick_place_task_params.hpp"
#include "mtc/object_factory.hpp"
#include "mtc/moveit_helper.hpp"

static const rclcpp::Logger kLogger = rclcpp::get_logger("moveit_task_constructor_demo");

namespace mtc = moveit::task_constructor;

void SetupDemoScene(const pick_place_task::Params& params) {
  // Add table and object to planning scene
  rclcpp::sleep_for(std::chrono::microseconds(100));  // Wait for ApplyPlanningScene service
  moveit::planning_interface::PlanningSceneInterface psi;
  SpawnObject(psi, object_factory::CreateTable(params.table_name, params.table_reference_frame, params.table_dimensions,
                                               params.table_pose));
  SpawnObject(psi,
              object_factory::CreateEnergyUnit(params.object_name, params.object_reference_frame, params.object_pose));
}

PickPlaceTask::PickPlaceTask(const std::string& task_name) : task_name_(task_name) {}

bool PickPlaceTask::init(const rclcpp::Node::SharedPtr& node) {
  RCLCPP_INFO(kLogger, "Initializing task pipeline");

  const auto param_listener = std::make_shared<pick_place_task::ParamListener>(node);
  const auto params = param_listener->get_params();
  SetupDemoScene(params);

  // Reset ROS introspection before constructing the new object
  // TODO(v4hn): global storage for Introspection services to enable one-liner
  task_.reset();
  task_.reset(new moveit::task_constructor::Task());

  // Individual movement stages are collected within the Task object
  mtc::Task& t = *task_;
  t.stages()->setName(task_name_);
  t.loadRobotModel(node);

  /* Create planners used in various stages. Various options are available,
     namely Cartesian, MoveIt pipeline, and joint interpolation. */
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
   *                                                  *
   *               Current State                      *
   *                                                  *
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
   *                                                  *
   *               Open Hand                          *
   *                                                  *
   ***************************************************/
  mtc::Stage* initial_state_ptr = nullptr;
  {
    auto stage = std::make_unique<mtc::stages::MoveTo>("open hand", sampling_planner);
    stage->setGroup(params.hand_group_name);
    stage->setGoal(params.hand_open_pose);
    initial_state_ptr = stage.get();  // remember start state for monitoring grasp pose generator
    t.add(std::move(stage));
  }

  /****************************************************
   *                                                  *
   *               Move to Pick                       *
   *                                                  *
   ***************************************************/
  // Connect initial open-hand state with pre-grasp pose defined in the following
  {
    mtc::stages::Connect::GroupPlannerVector planners = {{params.arm_group_name, sampling_planner},
                                                         {params.hand_group_name, sampling_planner}};
    auto stage = std::make_unique<mtc::stages::Connect>("move to pick", planners);
    stage->setTimeout(5.0);
    stage->properties().configureInitFrom(mtc::Stage::PARENT);
    t.add(std::move(stage));
  }

  /****************************************************
   *                                                  *
   *               Pick Object                        *
   *                                                  *
   ***************************************************/
  mtc::Stage* pick_stage_ptr = nullptr;
  {
    // A SerialContainer combines several sub-stages, here for picking the object
    auto grasp = std::make_unique<mtc::SerialContainer>("pick object");
    t.properties().exposeTo(grasp->properties(), {"eef", "hand", "group", "ik_frame"});
    grasp->properties().configureInitFrom(mtc::Stage::PARENT, {"eef", "hand", "group", "ik_frame"});

    /****************************************************
---- *               Approach Object                    *
     ***************************************************/
    {
      // Move the eef link forward along its z-axis by an amount within the given min-max range
      auto stage = std::make_unique<mtc::stages::MoveRelative>("approach object", cartesian_planner);
      stage->properties().set("marker_ns", "approach_object");
      stage->properties().set("link", params.hand_frame);                    // link to perform IK for
      stage->properties().configureInitFrom(mtc::Stage::PARENT, {"group"});  // inherit group from parent stage
      stage->setMinMaxDistance(params.approach_object_min_dist, params.approach_object_max_dist);

      // Set hand forward direction
      geometry_msgs::msg::Vector3Stamped vec;
      vec.header.frame_id = params.hand_frame;
      vec.vector.z = 1.0;
      stage->setDirection(vec);
      grasp->insert(std::move(stage));
    }

    /****************************************************
---- *               Allow Collision (hand object)   *
                 ***************************************************/
    {
      // Modify planning scene (w/o altering the robot's pose) to allow touching the object for picking
      auto stage = std::make_unique<mtc::stages::ModifyPlanningScene>("allow collision (hand,object)");
      stage->allowCollisions(
          params.object_name,
          t.getRobotModel()->getJointModelGroup(params.hand_group_name)->getLinkModelNamesWithCollisionGeometry(),
          true);
      grasp->insert(std::move(stage));
    }

    /****************************************************
---- *               Generate Grasp Pose                *
     ***************************************************/
    {
      // Sample grasp pose candidates in angle increments around the z-axis of the object
      auto stage = std::make_unique<mtc::stages::GenerateGraspPose>("generate grasp pose");
      stage->properties().configureInitFrom(mtc::Stage::PARENT);
      stage->properties().set("marker_ns", "grasp_pose");
      stage->setPreGraspPose(params.hand_open_pose);
      stage->setObject(params.object_name);  // object to sample grasps for
      stage->setAngleDelta(M_PI / 12);
      stage->setMonitoredStage(initial_state_ptr);  // hook into successful initial-phase solutions

      // Compute IK for sampled grasp poses
      auto wrapper = std::make_unique<mtc::stages::ComputeIK>("grasp pose IK", std::move(stage));
      wrapper->setMaxIKSolutions(8);  // limit number of solutions
      wrapper->setMinSolutionDistance(1.0);
      // define virtual frame to reach the target_pose
      wrapper->setIKFrame(VectorToEigen(params.grasp_frame_transform), params.hand_frame);
      wrapper->properties().configureInitFrom(mtc::Stage::PARENT, {"eef", "group"});  // inherit properties from parent
      wrapper->properties().configureInitFrom(mtc::Stage::INTERFACE,
                                              {"target_pose"});  // inherit property from child solution
      grasp->insert(std::move(wrapper));
    }

    /****************************************************
---- *               Close Hand                      *
     ***************************************************/
    {
      auto stage = std::make_unique<mtc::stages::MoveTo>("close hand", sampling_planner);
      stage->setGroup(params.hand_group_name);
      stage->setGoal(params.hand_close_pose);
      grasp->insert(std::move(stage));
    }

    /****************************************************
.... *               Attach Object                      *
     ***************************************************/
    {
      auto stage = std::make_unique<mtc::stages::ModifyPlanningScene>("attach object");
      stage->attachObject(params.object_name, params.hand_frame);  // attach object to hand_frame_
      grasp->insert(std::move(stage));
    }

    /****************************************************
.... *               Allow collision (object support)   *
     ***************************************************/
    {
      auto stage = std::make_unique<mtc::stages::ModifyPlanningScene>("allow collision (object,support)");
      stage->allowCollisions({params.object_name}, {params.surface_link}, true);
      grasp->insert(std::move(stage));
    }

    /****************************************************
.... *               Lift object                        *
     ***************************************************/
    {
      auto stage = std::make_unique<mtc::stages::MoveRelative>("lift object", cartesian_planner);
      stage->properties().configureInitFrom(mtc::Stage::PARENT, {"group"});
      stage->setMinMaxDistance(params.lift_object_min_dist, params.lift_object_max_dist);
      stage->setIKFrame(params.hand_frame);
      stage->properties().set("marker_ns", "lift_object");

      // Set upward direction
      geometry_msgs::msg::Vector3Stamped vec;
      vec.header.frame_id = params.world_frame;
      vec.vector.z = 1.0;
      stage->setDirection(vec);
      grasp->insert(std::move(stage));
    }

    /****************************************************
.... *               Forbid collision (object support)  *
     ***************************************************/
    {
      auto stage = std::make_unique<mtc::stages::ModifyPlanningScene>("forbid collision (object,surface)");
      stage->allowCollisions({params.object_name}, {params.surface_link}, false);
      grasp->insert(std::move(stage));
    }

    pick_stage_ptr = grasp.get();  // remember for monitoring place pose generator

    // Add grasp container to task
    t.add(std::move(grasp));
  }

  /******************************************************
   *                                                    *
   *          Move to Place                             *
   *                                                    *
   *****************************************************/
  {
    // Connect the grasped state to the pre-place state, i.e. realize the object transport
    auto stage = std::make_unique<mtc::stages::Connect>(
        "move to place", mtc::stages::Connect::GroupPlannerVector{{params.arm_group_name, sampling_planner}});
    stage->setTimeout(5.0);
    stage->properties().configureInitFrom(mtc::Stage::PARENT);
    t.add(std::move(stage));
  }

  /******************************************************
   *                                                    *
   *          Place Object                              *
   *                                                    *
   *****************************************************/
  // All placing sub-stages are collected within a serial container again
  {
    auto place = std::make_unique<mtc::SerialContainer>("place object");
    t.properties().exposeTo(place->properties(), {"eef", "hand", "group"});
    place->properties().configureInitFrom(mtc::Stage::PARENT, {"eef", "hand", "group"});

    /******************************************************
---- *          Lower Object                              *
     *****************************************************/
    {
      auto stage = std::make_unique<mtc::stages::MoveRelative>("lower object", cartesian_planner);
      stage->properties().set("marker_ns", "lower_object");
      stage->properties().set("link", params.hand_frame);
      stage->properties().configureInitFrom(mtc::Stage::PARENT, {"group"});
      stage->setMinMaxDistance(.03, .13);

      // Set downward direction
      geometry_msgs::msg::Vector3Stamped vec;
      vec.header.frame_id = params.world_frame;
      vec.vector.z = -1.0;
      stage->setDirection(vec);
      place->insert(std::move(stage));
    }

    /******************************************************
---- *          Generate Place Pose                       *
     *****************************************************/
    {
      // Generate Place Pose
      auto stage = std::make_unique<mtc::stages::GeneratePlacePose>("generate place pose");
      stage->properties().configureInitFrom(mtc::Stage::PARENT, {"ik_frame"});
      stage->properties().set("marker_ns", "place_pose");
      stage->setObject(params.object_name);

      // Set target pose
      geometry_msgs::msg::PoseStamped p;
      p.header.frame_id = params.object_reference_frame;
      p.pose = VectorToPose(params.place_pose);
      p.pose.position.z += 0.5 * params.object_dimensions[0] + params.place_surface_offset;
      stage->setPose(p);
      stage->setMonitoredStage(pick_stage_ptr);  // hook into successful pick solutions

      // Compute IK
      auto wrapper = std::make_unique<mtc::stages::ComputeIK>("place pose IK", std::move(stage));
      wrapper->setMaxIKSolutions(2);
      wrapper->setIKFrame(VectorToEigen(params.grasp_frame_transform), params.hand_frame);
      wrapper->properties().configureInitFrom(mtc::Stage::PARENT, {"eef", "group"});
      wrapper->properties().configureInitFrom(mtc::Stage::INTERFACE, {"target_pose"});
      place->insert(std::move(wrapper));
    }

    /******************************************************
---- *          Open Hand                              *
     *****************************************************/
    {
      auto stage = std::make_unique<mtc::stages::MoveTo>("open hand", sampling_planner);
      stage->setGroup(params.hand_group_name);
      stage->setGoal(params.hand_open_pose);
      place->insert(std::move(stage));
    }

    /******************************************************
---- *          Forbid collision (hand, object)        *
     *****************************************************/
    {
      auto stage = std::make_unique<mtc::stages::ModifyPlanningScene>("forbid collision (hand,object)");
      stage->allowCollisions(params.object_name, *t.getRobotModel()->getJointModelGroup(params.hand_group_name), false);
      place->insert(std::move(stage));
    }

    /******************************************************
---- *          Detach Object                             *
     *****************************************************/
    {
      auto stage = std::make_unique<mtc::stages::ModifyPlanningScene>("detach object");
      stage->detachObject(params.object_name, params.hand_frame);
      place->insert(std::move(stage));
    }

    /******************************************************
---- *          Retreat Motion                            *
     *****************************************************/
    {
      auto stage = std::make_unique<mtc::stages::MoveRelative>("retreat after place", cartesian_planner);
      stage->properties().configureInitFrom(mtc::Stage::PARENT, {"group"});
      stage->setMinMaxDistance(.12, .25);
      stage->setIKFrame(params.hand_frame);
      stage->properties().set("marker_ns", "retreat");
      geometry_msgs::msg::Vector3Stamped vec;
      vec.header.frame_id = params.hand_frame;
      vec.vector.z = -1.0;
      stage->setDirection(vec);
      place->insert(std::move(stage));
    }

    // Add place container to task
    t.add(std::move(place));
  }

  /******************************************************
   *                                                    *
   *          Move to Home                              *
   *                                                    *
   *****************************************************/
  {
    auto stage = std::make_unique<mtc::stages::MoveTo>("move home", sampling_planner);
    stage->properties().configureInitFrom(mtc::Stage::PARENT, {"group"});
    stage->setGoal(params.arm_home_pose);
    stage->restrictDirection(mtc::stages::MoveTo::FORWARD);
    t.add(std::move(stage));
  }

  // prepare Task structure for planning
  try {
    t.init();
  } catch (mtc::InitStageException& e) {
    RCLCPP_ERROR_STREAM(kLogger, "Initialization failed: " << e);
    return false;
  }

  return true;
}

bool PickPlaceTask::plan(std::size_t max_solutions) {
  RCLCPP_INFO(kLogger, "Start searching for task solutions");

  return static_cast<bool>(task_->plan(max_solutions));
}

bool PickPlaceTask::execute() {
  RCLCPP_INFO(kLogger, "Executing solution trajectory");
  moveit_msgs::msg::MoveItErrorCodes execute_result;

  execute_result = task_->execute(*task_->solutions().front());

  if (execute_result.val != moveit_msgs::msg::MoveItErrorCodes::SUCCESS) {
    RCLCPP_ERROR_STREAM(kLogger, "Task execution failed and returned: " << execute_result.val);
    return false;
  }

  return true;
}
