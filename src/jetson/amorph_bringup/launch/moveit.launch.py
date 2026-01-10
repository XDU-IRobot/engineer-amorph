from launch import LaunchDescription
from launch_ros.actions import Node
from moveit_configs_utils import MoveItConfigsBuilder
from moveit_configs_utils.launches import generate_move_group_launch


def generate_launch_description():
    moveit_config = (
        MoveItConfigsBuilder("amorph_arm_v1")
        .planning_pipelines(pipelines=["ompl"])
        .robot_description(file_path="config/amorph.urdf.xacro")
        # .trajectory_execution(file_path="config/moveit_controllers.yaml")
        .joint_limits(file_path="config/joint_limits.yaml")
        .robot_description_kinematics(file_path="config/kinematics.yaml")
        .to_moveit_configs()
    )

    # # Load  ExecuteTaskSolutionCapability so we can execute found solutions in simulation
    # move_group_capabilities = {
    #     "capabilities": "move_group/ExecuteTaskSolutionCapability"}

    nodes = list()
    
    # Static TF(world -> arm_base)
    static_tf_world_to_base = Node(
        package="tf2_ros",
        executable="static_transform_publisher",
        name="static_tf_world_to_base",
        output="log",
        arguments=["--frame-id", "world", "--child-frame-id", "arm_base"],
    )
    nodes.append(static_tf_world_to_base)

    # Static TF(arm_base -> link_1)
    static_tf_base_to_arm = Node(
        package="tf2_ros",
        executable="static_transform_publisher",
        name="static_tf_base_to_arm",
        output="log",
        arguments=["--frame-id", "arm_base", "--child-frame-id", "link_1"],
    )
    nodes.append(static_tf_base_to_arm)

    # Static TF(arm_base -> camera_link)
    static_tf_base_to_cam = Node(
        package="tf2_ros",
        executable="static_transform_publisher",
        name="static_tf_base_to_cam",
        output="log",
        arguments=["--frame-id", "arm_base", 
                    "--child-frame-id", "camera_link",
                    "--x", "0.672108",
                    "--y", "1.215842",
                    "--z", "0.605491",
                    "--qx", "0.029197",
                    "--qy", "-0.004083",
                    "--qz", "-0.724784",
                    "--qw", "0.688345"],
    )
    nodes.append(static_tf_base_to_cam)

    # rsp
    robot_state_publisher = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        # name="robot_state_publisher",
        output="both",
        parameters=[moveit_config.robot_description],
    )
    nodes.append(robot_state_publisher)

    launch = LaunchDescription(nodes)
    launch.add_entity(generate_move_group_launch(moveit_config))
    return launch
