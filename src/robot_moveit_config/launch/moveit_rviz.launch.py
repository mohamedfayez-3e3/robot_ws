from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import SetRemap

from moveit_configs_utils import MoveItConfigsBuilder
from moveit_configs_utils.launch_utils import (
    DeclareBooleanLaunchArg,
    add_debuggable_node,
)


def generate_launch_description():
    moveit_config = MoveItConfigsBuilder(
        "arm_assembly", package_name="robot_moveit_config"
    ).to_moveit_configs()

    launch_description = LaunchDescription(
        [
            SetRemap(src="joint_states", dst="/moveit_joint_states"),
            DeclareBooleanLaunchArg("debug", default_value=False),
            DeclareLaunchArgument(
                "rviz_config",
                default_value=str(moveit_config.package_path / "config/moveit.rviz"),
            ),
        ]
    )

    # RViz must use the same MoveIt-only mimic-joint model as move_group. The
    # Gazebo robot_state_publisher publishes the physical four-joint URDF on
    # /robot_description, which does not contain gripper_position_joint.
    rviz_parameters = [
        moveit_config.robot_description,
        moveit_config.robot_description_semantic,
        moveit_config.planning_pipelines,
        moveit_config.robot_description_kinematics,
        moveit_config.joint_limits,
    ]

    add_debuggable_node(
        launch_description,
        package="rviz2",
        executable="rviz2",
        output="log",
        respawn=False,
        arguments=["-d", LaunchConfiguration("rviz_config")],
        parameters=rviz_parameters,
    )

    return launch_description
